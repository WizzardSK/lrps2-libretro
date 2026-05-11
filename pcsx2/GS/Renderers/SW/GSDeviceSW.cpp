/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2021 PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with PCSX2.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstring>
#include <memory>

#include <libretro.h>

#include "common/AlignedMalloc.h"
#include "common/Console.h"
#include "GS/GSExtra.h"
#include "GS/Renderers/SW/GSDeviceSW.h"

extern retro_environment_t    environ_cb;
extern retro_video_refresh_t  video_cb;

/* CPU-backed GSDownloadTexture. The SW renderer doesn't actually
 * invoke the download path in the present flow (its m_output is
 * already in CPU memory), but GSDevice::CreateDownloadTexture is a
 * pure virtual so we need *something* that satisfies the interface
 * for any code path that asks. */
namespace
{
	class GSDownloadTextureSW final : public GSDownloadTexture
	{
	private:
		u8* m_buffer = nullptr;
		u32 m_buffer_size = 0;

	public:
		GSDownloadTextureSW(u32 width, u32 height, GSTexture::Format format)
			: GSDownloadTexture(width, height, format)
		{
			const u32 bpp = (format == GSTexture::Format::UNorm8) ? 1 : 4;
			m_current_pitch = Common::AlignUpPow2(width * bpp, VECTOR_ALIGNMENT);
			m_buffer_size = m_current_pitch * height;
			m_buffer = (u8*)_aligned_malloc(m_buffer_size ? m_buffer_size : VECTOR_ALIGNMENT, VECTOR_ALIGNMENT);
			if (m_buffer && m_buffer_size)
				std::memset(m_buffer, 0, m_buffer_size);
		}

		~GSDownloadTextureSW() override
		{
			safe_aligned_free(m_buffer);
		}

		void CopyFromTexture(const GSVector4i& /*drc*/, GSTexture* /*stex*/, const GSVector4i& /*src*/,
			u32 /*src_level*/, bool /*use_transfer_pitch*/) override
		{
			/* SW renderer's GSRendererSW already has its pixels in
			 * m_output (CPU); the present path does not go through
			 * CopyFromTexture. If something else hits this in a SW-
			 * only build, leaving the buffer zero-filled is a
			 * defensible failure mode. */
		}

		bool Map(const GSVector4i& /*read_rc*/) override
		{
			m_map_pointer = m_buffer;
			return m_map_pointer != nullptr;
		}

		void Unmap() override
		{
			m_map_pointer = nullptr;
		}

		void Flush() override
		{
			/* No GPU work to flush. */
		}
	};
} // namespace

GSDeviceSW::GSDeviceSW() = default;

GSDeviceSW::~GSDeviceSW()
{
	safe_aligned_free(m_present_buffer);
}

bool GSDeviceSW::Create()
{
	if (!GSDevice::Create())
		return false;

	AcquireWindow();

	/* The SW renderer reads these to decide what to support. We have
	 * no GPU features at all - it's all CPU - so every advanced flag
	 * stays false. The SW renderer doesn't actually check most of
	 * these (they're HW-renderer concerns); zero defaults are right. */
	m_features = FeatureSupport();

	return true;
}

void GSDeviceSW::Destroy()
{
	GSDevice::Destroy();
	safe_aligned_free(m_present_buffer);
	m_present_buffer = nullptr;
	m_present_buffer_capacity = 0;
	m_present_pitch = 0;
	m_present_width = 0;
	m_present_height = 0;
	m_direct_rendering_checked = false;
	m_direct_rendering_supported = false;
}

GSTexture* GSDeviceSW::CreateSurface(GSTexture::Type type, int width, int height, int /*levels*/, GSTexture::Format format)
{
	return new GSTextureSW(type, width, height, format);
}

std::unique_ptr<GSDownloadTexture> GSDeviceSW::CreateDownloadTexture(u32 width, u32 height, GSTexture::Format format)
{
	return std::make_unique<GSDownloadTextureSW>(width, height, format);
}

void GSDeviceSW::DoMerge(GSTexture* /*sTex*/[3], GSVector4* /*sRect*/, GSTexture* /*dTex*/, GSVector4* /*dRect*/,
	const GSRegPMODE& /*PMODE*/, const GSRegEXTBUF& /*EXTBUF*/, u32 /*c*/, const bool /*linear*/)
{
	/* Stage 2: implement CPU compositing of the two circuits into dTex. */
}

void GSDeviceSW::DoInterlace(GSTexture* /*sTex*/, const GSVector4& /*sRect*/, GSTexture* /*dTex*/, const GSVector4& /*dRect*/,
	ShaderInterlace /*shader*/, bool /*linear*/, const InterlaceConstantBuffer& /*cb*/)
{
	/* Stage 3: implement weave/bob/blend/MAD/adaptive on CPU. */
}

void GSDeviceSW::CopyRect(GSTexture* /*sTex*/, GSTexture* /*dTex*/, const GSVector4i& /*r*/, u32 /*destX*/, u32 /*destY*/)
{
	/* Stage 2. */
}

void GSDeviceSW::StretchRect(GSTexture* /*sTex*/, const GSVector4& /*sRect*/, GSTexture* /*dTex*/, const GSVector4& /*dRect*/,
	ShaderConvert /*shader*/, bool /*linear*/)
{
	/* Stage 2. */
}

void GSDeviceSW::StretchRect(GSTexture* /*sTex*/, const GSVector4& /*sRect*/, GSTexture* /*dTex*/, const GSVector4& /*dRect*/,
	bool /*red*/, bool /*green*/, bool /*blue*/, bool /*alpha*/, ShaderConvert /*shader*/)
{
	/* Stage 2. */
}

void GSDeviceSW::PresentRect(GSTexture* /*sTex*/, const GSVector4& /*sRect*/, GSTexture* /*dTex*/, const GSVector4& /*dRect*/)
{
	/* Stage 2: composite sTex into dTex (or into m_present_buffer if
	 * dTex is null - that's the "draw to backbuffer" signal). */
}

void GSDeviceSW::UpdateCLUTTexture(GSTexture* /*sTex*/, float /*sScale*/, u32 /*offsetX*/, u32 /*offsetY*/,
	GSTexture* /*dTex*/, u32 /*dOffset*/, u32 /*dSize*/)
{
	/* HW-renderer-only path - the SW renderer doesn't call this. */
}

void GSDeviceSW::ConvertToIndexedTexture(GSTexture* /*sTex*/, float /*sScale*/, u32 /*offsetX*/, u32 /*offsetY*/,
	u32 /*SBW*/, u32 /*SPSM*/, GSTexture* /*dTex*/, u32 /*DBW*/, u32 /*DPSM*/)
{
	/* HW-renderer-only path. */
}

void GSDeviceSW::FilteredDownsampleTexture(GSTexture* /*sTex*/, GSTexture* /*dTex*/, u32 /*downsample_factor*/,
	const GSVector2i& /*clamp_min*/, const GSVector4& /*dRect*/)
{
	/* HW-renderer-only path. */
}

void GSDeviceSW::RenderHW(GSHWDrawConfig& /*config*/)
{
	/* GSRendererHW dispatches drawing here. The SW renderer never
	 * does. If this fires, the device was wired up to the wrong
	 * renderer - that's a build/init bug, not a runtime condition we
	 * can recover from. */
	Console.Error("GSDeviceSW::RenderHW called - HW renderer is not compatible with the CPU-only device.");
}

void GSDeviceSW::ClearSamplerCache()
{
	/* No samplers exist. */
}

GSDevice::PresentResult GSDeviceSW::BeginPresent(bool frame_skip)
{
	if (frame_skip)
		return PresentResult::FrameSkipped;
	return PresentResult::OK;
}

void GSDeviceSW::EnsurePresentBuffer(int width, int height)
{
	const int bpp = 4; /* XRGB8888 */
	const int pitch = static_cast<int>(Common::AlignUpPow2(static_cast<u32>(width * bpp), VECTOR_ALIGNMENT));
	const int needed = pitch * height;
	if (needed <= 0)
		return;

	if (needed > m_present_buffer_capacity)
	{
		safe_aligned_free(m_present_buffer);
		m_present_buffer = (u8*)_aligned_malloc(needed, VECTOR_ALIGNMENT);
		m_present_buffer_capacity = m_present_buffer ? needed : 0;
	}
	m_present_pitch  = pitch;
	m_present_width  = width;
	m_present_height = height;
}

void GSDeviceSW::EndPresent()
{
	/* Logical present size: prefer the window info, fall back to
	 * whatever the SW renderer thinks the output texture is. For
	 * stage 1 the window size is always the right answer because we
	 * don't have a real composite yet. */
	int w = static_cast<int>(m_window_info.surface_width);
	int h = static_cast<int>(m_window_info.surface_height);
	if (w <= 0 || h <= 0)
	{
		/* No window info populated yet - skip this frame instead of
		 * sending nonsense dimensions to the frontend. */
		return;
	}

	/* Try direct rendering on the first present, then cache the
	 * answer. Re-query if dimensions change (frontend may have
	 * format constraints that depend on size). */
	const bool dims_changed = (w != m_present_width || h != m_present_height);
	if (!m_direct_rendering_checked || dims_changed)
	{
		retro_framebuffer fb = {};
		fb.width        = static_cast<unsigned>(w);
		fb.height       = static_cast<unsigned>(h);
		fb.access_flags = RETRO_MEMORY_ACCESS_WRITE;

		m_direct_rendering_supported =
			environ_cb &&
			environ_cb(RETRO_ENVIRONMENT_GET_CURRENT_SOFTWARE_FRAMEBUFFER, &fb) &&
			fb.data != nullptr &&
			fb.format == RETRO_PIXEL_FORMAT_XRGB8888;
		m_direct_rendering_checked = true;
	}

	void* out_data = nullptr;
	size_t out_pitch = 0;

	if (m_direct_rendering_supported)
	{
		retro_framebuffer fb = {};
		fb.width        = static_cast<unsigned>(w);
		fb.height       = static_cast<unsigned>(h);
		fb.access_flags = RETRO_MEMORY_ACCESS_WRITE;
		if (environ_cb &&
			environ_cb(RETRO_ENVIRONMENT_GET_CURRENT_SOFTWARE_FRAMEBUFFER, &fb) &&
			fb.data &&
			fb.format == RETRO_PIXEL_FORMAT_XRGB8888 &&
			fb.width == static_cast<unsigned>(w) &&
			fb.height == static_cast<unsigned>(h))
		{
			out_data  = fb.data;
			out_pitch = fb.pitch;
		}
		else
		{
			/* Frontend changed its mind mid-session. Fall back. */
			m_direct_rendering_supported = false;
		}
	}

	if (!out_data)
	{
		EnsurePresentBuffer(w, h);
		out_data  = m_present_buffer;
		out_pitch = static_cast<size_t>(m_present_pitch);
	}

	if (!out_data || out_pitch == 0)
		return;

	/* Stage 1: emit black. Stage 2 will composite the merged frame
	 * here from m_current (an internal GSTextureSW the device will
	 * own once StretchRect/PresentRect are implemented). */
	u8* row = static_cast<u8*>(out_data);
	for (int y = 0; y < h; y++)
	{
		std::memset(row, 0, static_cast<size_t>(w) * 4);
		row += out_pitch;
	}

	if (video_cb)
		video_cb(out_data, static_cast<unsigned>(w), static_cast<unsigned>(h), out_pitch);
}
