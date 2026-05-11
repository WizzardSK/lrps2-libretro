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

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>

#include <libretro.h>

#include "common/AlignedMalloc.h"
#include "common/Console.h"
#include "GS/GSExtra.h"
#include "GS/Renderers/SW/GSDeviceSW.h"

extern retro_environment_t    environ_cb;
extern retro_video_refresh_t  video_cb;

namespace
{
	/* ====================================================================
	 * Inline pixel helpers
	 *
	 * Everything in this device operates on 32 bpp BGRA/XRGB8888-ish
	 * pixels. PS2 PSMCT32 stores ABGR (R in byte 0). The libretro
	 * XRGB8888 format the frontend expects has B in byte 0. The HW
	 * backends paper over this with shader swizzles; we have to do it
	 * by hand on the way to the present buffer.
	 *
	 * However, for intermediate GSTextureSW <-> GSTextureSW copies
	 * we keep the source byte order as-is - swizzling once, at present
	 * time, is sufficient and cheapest.
	 * ==================================================================== */

	__fi u32 LoadPx(const u8* row, int x) { return *reinterpret_cast<const u32*>(row + x * 4); }
	__fi void StorePx(u8* row, int x, u32 v) { *reinterpret_cast<u32*>(row + x * 4) = v; }

	/* PS2-native ABGR -> libretro XRGB8888 (B in byte 0). Forces
	 * alpha to 0xFF since the frontend doesn't blend output. */
	__fi u32 PS2ToXRGB(u32 abgr)
	{
		const u32 r = (abgr      ) & 0xFFu;
		const u32 g = (abgr >>  8) & 0xFFu;
		const u32 b = (abgr >> 16) & 0xFFu;
		return (0xFFu << 24) | (r << 16) | (g << 8) | b;
	}

	/* Clear a rect of `dst` (BGRA/XRGB8888 layout) to the given
	 * packed color. */
	void ClearRectPx(u8* dst, int dst_pitch, int dst_w, int dst_h, u32 color_xrgb)
	{
		for (int y = 0; y < dst_h; y++)
		{
			u32* row = reinterpret_cast<u32*>(dst + (size_t)y * dst_pitch);
			for (int x = 0; x < dst_w; x++)
				row[x] = color_xrgb;
		}
	}

	/* Nearest-neighbor copy from a source GSTextureSW region to a
	 * destination GSTextureSW region. Source and destination texels
	 * are bit-equal (no PS2->XRGB conversion); the conversion happens
	 * only when writing into the libretro present buffer. */
	void NearestCopyPS2(
		const u8* src, int src_pitch, int src_w, int src_h,
		float sx0, float sy0, float sx1, float sy1,
		u8* dst, int dst_pitch,
		int dx0, int dy0, int dx1, int dy1,
		int dst_clip_w, int dst_clip_h)
	{
		const int out_w = dx1 - dx0;
		const int out_h = dy1 - dy0;
		if (out_w <= 0 || out_h <= 0)
			return;

		/* Normalized UV -> source pixel coords. Add 0.5 to sample
		 * texel centers, matching nearest-neighbor texturing. */
		const float src_x_scale = (sx1 - sx0) * static_cast<float>(src_w) / static_cast<float>(out_w);
		const float src_y_scale = (sy1 - sy0) * static_cast<float>(src_h) / static_cast<float>(out_h);
		const float src_x_base  = sx0 * static_cast<float>(src_w);
		const float src_y_base  = sy0 * static_cast<float>(src_h);

		for (int y = 0; y < out_h; y++)
		{
			const int dy = dy0 + y;
			if (dy < 0 || dy >= dst_clip_h)
				continue;

			int sy = static_cast<int>(src_y_base + (static_cast<float>(y) + 0.5f) * src_y_scale);
			if (sy < 0)         sy = 0;
			else if (sy >= src_h) sy = src_h - 1;

			const u8* src_row = src + (size_t)sy * src_pitch;
			u8* dst_row       = dst + (size_t)dy * dst_pitch;

			for (int x = 0; x < out_w; x++)
			{
				const int dx = dx0 + x;
				if (dx < 0 || dx >= dst_clip_w)
					continue;

				int sx = static_cast<int>(src_x_base + (static_cast<float>(x) + 0.5f) * src_x_scale);
				if (sx < 0)         sx = 0;
				else if (sx >= src_w) sx = src_w - 1;

				StorePx(dst_row, dx, LoadPx(src_row, sx));
			}
		}
	}

	/* Variant that converts source ABGR (PS2 native) -> XRGB8888 on
	 * write. Used by PresentRect to fill the libretro framebuffer. */
	void NearestCopyToXRGB(
		const u8* src, int src_pitch, int src_w, int src_h,
		float sx0, float sy0, float sx1, float sy1,
		u8* dst, int dst_pitch,
		int dx0, int dy0, int dx1, int dy1,
		int dst_clip_w, int dst_clip_h)
	{
		const int out_w = dx1 - dx0;
		const int out_h = dy1 - dy0;
		if (out_w <= 0 || out_h <= 0)
			return;

		const float src_x_scale = (sx1 - sx0) * static_cast<float>(src_w) / static_cast<float>(out_w);
		const float src_y_scale = (sy1 - sy0) * static_cast<float>(src_h) / static_cast<float>(out_h);
		const float src_x_base  = sx0 * static_cast<float>(src_w);
		const float src_y_base  = sy0 * static_cast<float>(src_h);

		for (int y = 0; y < out_h; y++)
		{
			const int dy = dy0 + y;
			if (dy < 0 || dy >= dst_clip_h)
				continue;

			int sy = static_cast<int>(src_y_base + (static_cast<float>(y) + 0.5f) * src_y_scale);
			if (sy < 0)         sy = 0;
			else if (sy >= src_h) sy = src_h - 1;

			const u8* src_row = src + (size_t)sy * src_pitch;
			u8* dst_row       = dst + (size_t)dy * dst_pitch;

			for (int x = 0; x < out_w; x++)
			{
				const int dx = dx0 + x;
				if (dx < 0 || dx >= dst_clip_w)
					continue;

				int sx = static_cast<int>(src_x_base + (static_cast<float>(x) + 0.5f) * src_x_scale);
				if (sx < 0)         sx = 0;
				else if (sx >= src_w) sx = src_w - 1;

				StorePx(dst_row, dx, PS2ToXRGB(LoadPx(src_row, sx)));
			}
		}
	}

	/* Alpha-blend a source GSTextureSW region onto a destination
	 * region. Per-source-pixel alpha or a constant alpha is used:
	 *   - mode_constant_alpha=true: alpha8 is the same for every pixel
	 *     (PMODE.MMOD == 1, AKA constant alpha from PMODE.ALP).
	 *   - mode_constant_alpha=false: alpha8 ignored; per-pixel alpha
	 *     read from the source.
	 *
	 * PS2 PSMCT32 alpha is stored as 0..0x80, where 0x80 represents
	 * "fully opaque" (1.0 in shader space). Anything above 0x80
	 * saturates. The HW backends paper over this in the merge
	 * pixel shader by computing alpha * 2 with saturation; we do
	 * the same here. PMODE.ALP (constant alpha) follows the same
	 * convention. */
	void BlendOverPS2(
		const u8* src, int src_pitch, int src_w, int src_h,
		float sx0, float sy0, float sx1, float sy1,
		u8* dst, int dst_pitch,
		int dx0, int dy0, int dx1, int dy1,
		int dst_clip_w, int dst_clip_h,
		bool mode_constant_alpha, u8 alpha8)
	{
		const int out_w = dx1 - dx0;
		const int out_h = dy1 - dy0;
		if (out_w <= 0 || out_h <= 0)
			return;

		const float src_x_scale = (sx1 - sx0) * static_cast<float>(src_w) / static_cast<float>(out_w);
		const float src_y_scale = (sy1 - sy0) * static_cast<float>(src_h) / static_cast<float>(out_h);
		const float src_x_base  = sx0 * static_cast<float>(src_w);
		const float src_y_base  = sy0 * static_cast<float>(src_h);

		/* Pre-compute the doubled constant alpha (clamped to 255). */
		const u32 const_a2 = std::min<u32>(static_cast<u32>(alpha8) * 2u, 255u);

		for (int y = 0; y < out_h; y++)
		{
			const int dy = dy0 + y;
			if (dy < 0 || dy >= dst_clip_h)
				continue;

			int sy = static_cast<int>(src_y_base + (static_cast<float>(y) + 0.5f) * src_y_scale);
			if (sy < 0)         sy = 0;
			else if (sy >= src_h) sy = src_h - 1;

			const u8* src_row = src + (size_t)sy * src_pitch;
			u8* dst_row       = dst + (size_t)dy * dst_pitch;

			for (int x = 0; x < out_w; x++)
			{
				const int dx = dx0 + x;
				if (dx < 0 || dx >= dst_clip_w)
					continue;

				int sx = static_cast<int>(src_x_base + (static_cast<float>(x) + 0.5f) * src_x_scale);
				if (sx < 0)         sx = 0;
				else if (sx >= src_w) sx = src_w - 1;

				const u32 s = LoadPx(src_row, sx);
				const u32 d = LoadPx(dst_row, dx);

				u32 a;
				if (mode_constant_alpha)
					a = const_a2;
				else
				{
					const u32 raw = (s >> 24) & 0xFFu;
					a = std::min<u32>(raw * 2u, 255u);
				}
				const u32 ia = 255u - a;

				const u32 sr = (s      ) & 0xFFu;
				const u32 sg = (s >>  8) & 0xFFu;
				const u32 sb = (s >> 16) & 0xFFu;
				const u32 dr = (d      ) & 0xFFu;
				const u32 dg = (d >>  8) & 0xFFu;
				const u32 db = (d >> 16) & 0xFFu;

				/* +127 for rounding before the /255 (approximated as
				 * >>8; with the +127 the max error is <1 ulp). */
				const u32 r = ((sr * a + dr * ia + 127u) >> 8) & 0xFFu;
				const u32 g = ((sg * a + dg * ia + 127u) >> 8) & 0xFFu;
				const u32 b = ((sb * a + db * ia + 127u) >> 8) & 0xFFu;

				StorePx(dst_row, dx, (d & 0xFF000000u) | (b << 16) | (g << 8) | r);
			}
		}
	}
} // anonymous namespace

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
	m_direct_render_buffer = nullptr;
	m_direct_render_pitch = 0;
	m_present_pending = false;
}

GSTexture* GSDeviceSW::CreateSurface(GSTexture::Type type, int width, int height, int /*levels*/, GSTexture::Format format)
{
	return new GSTextureSW(type, width, height, format);
}

std::unique_ptr<GSDownloadTexture> GSDeviceSW::CreateDownloadTexture(u32 width, u32 height, GSTexture::Format format)
{
	return std::make_unique<GSDownloadTextureSW>(width, height, format);
}

void GSDeviceSW::DoMerge(GSTexture* sTex[3], GSVector4* sRect, GSTexture* dTex, GSVector4* dRect,
	const GSRegPMODE& PMODE, const GSRegEXTBUF& EXTBUF, u32 c, const bool /*linear*/)
{
	if (!dTex)
		return;

	GSTextureSW* dst = static_cast<GSTextureSW*>(dTex);
	const int dst_w  = dst->GetWidth();
	const int dst_h  = dst->GetHeight();
	const int dst_p  = dst->GetPitch();
	u8* dst_data     = dst->GetPointer();
	if (!dst_data)
		return;

	/* PS2 BGCOLOR is u32 0x00BBGGRR with alpha from PMODE.ALP packed
	 * by the caller into bits 24..31. Treat the contents of GSTextureSW
	 * as PS2-native ABGR (R in byte 0), matching what the SW renderer
	 * wrote when it Update()d circuit textures from m_output. */
	const u32 bg_abgr = (c & 0x00FFFFFFu) | 0xFF000000u;
	ClearRectPx(dst_data, dst_p, dst_w, dst_h, bg_abgr);

	/* Feedback writes are an EXTBUF/PCRTC loopback path the HW
	 * backends route through dTex for one frame then read back. The
	 * SW renderer doesn't exercise this path in practice (Xenosaga
	 * cutscenes use it, but go through GetFeedbackOutput which is a
	 * separate SW-side hook). Leave unhandled for stage 2 - if a
	 * game depends on it under the CPU path we'll see it as a
	 * specific visual regression and can address then. */
	(void)EXTBUF;

	/* Circuit 1: copy with no blend (background already filled). */
	if (sTex[1] && PMODE.SLBG == 0)
	{
		GSTextureSW* src = static_cast<GSTextureSW*>(sTex[1]);
		const int dx0 = static_cast<int>(dRect[1].x);
		const int dy0 = static_cast<int>(dRect[1].y);
		const int dx1 = static_cast<int>(dRect[1].z);
		const int dy1 = static_cast<int>(dRect[1].w);
		NearestCopyPS2(
			src->GetPointer(), src->GetPitch(), src->GetWidth(), src->GetHeight(),
			sRect[1].x, sRect[1].y, sRect[1].z, sRect[1].w,
			dst_data, dst_p,
			dx0, dy0, dx1, dy1,
			dst_w, dst_h);
	}

	/* Circuit 0: alpha-blended composite over whatever is in dTex. */
	if (sTex[0])
	{
		GSTextureSW* src = static_cast<GSTextureSW*>(sTex[0]);
		const int dx0 = static_cast<int>(dRect[0].x);
		const int dy0 = static_cast<int>(dRect[0].y);
		const int dx1 = static_cast<int>(dRect[0].z);
		const int dy1 = static_cast<int>(dRect[0].w);
		const bool constant_alpha = (PMODE.MMOD == 1);
		const u8 alpha8 = static_cast<u8>((c >> 24) & 0xFFu);
		BlendOverPS2(
			src->GetPointer(), src->GetPitch(), src->GetWidth(), src->GetHeight(),
			sRect[0].x, sRect[0].y, sRect[0].z, sRect[0].w,
			dst_data, dst_p,
			dx0, dy0, dx1, dy1,
			dst_w, dst_h,
			constant_alpha, alpha8);
	}
}

void GSDeviceSW::DoInterlace(GSTexture* /*sTex*/, const GSVector4& /*sRect*/, GSTexture* /*dTex*/, const GSVector4& /*dRect*/,
	ShaderInterlace /*shader*/, bool /*linear*/, const InterlaceConstantBuffer& /*cb*/)
{
	/* Stage 3: implement weave/bob/blend/MAD/adaptive on CPU. */
}

void GSDeviceSW::CopyRect(GSTexture* sTex, GSTexture* dTex, const GSVector4i& r, u32 destX, u32 destY)
{
	if (!sTex || !dTex)
		return;

	GSTextureSW* src = static_cast<GSTextureSW*>(sTex);
	GSTextureSW* dst = static_cast<GSTextureSW*>(dTex);
	if (!src->GetPointer() || !dst->GetPointer())
		return;

	const int sx = r.x;
	const int sy = r.y;
	const int sw = r.width();
	const int sh = r.height();
	if (sw <= 0 || sh <= 0)
		return;

	const int dx_start = static_cast<int>(destX);
	const int dy_start = static_cast<int>(destY);

	for (int row = 0; row < sh; row++)
	{
		const int sy_row = sy + row;
		const int dy_row = dy_start + row;
		if (sy_row < 0 || sy_row >= src->GetHeight()) continue;
		if (dy_row < 0 || dy_row >= dst->GetHeight()) continue;

		const u8* src_row = src->GetPointer() + (size_t)sy_row * src->GetPitch();
		u8* dst_row       = dst->GetPointer() + (size_t)dy_row * dst->GetPitch();

		for (int col = 0; col < sw; col++)
		{
			const int sx_col = sx + col;
			const int dx_col = dx_start + col;
			if (sx_col < 0 || sx_col >= src->GetWidth()) continue;
			if (dx_col < 0 || dx_col >= dst->GetWidth()) continue;
			StorePx(dst_row, dx_col, LoadPx(src_row, sx_col));
		}
	}
}

void GSDeviceSW::StretchRect(GSTexture* sTex, const GSVector4& sRect, GSTexture* dTex, const GSVector4& dRect,
	ShaderConvert shader, bool /*linear*/)
{
	if (!sTex || !dTex)
		return;

	GSTextureSW* src = static_cast<GSTextureSW*>(sTex);
	GSTextureSW* dst = static_cast<GSTextureSW*>(dTex);
	if (!src->GetPointer() || !dst->GetPointer())
		return;

	const int dx0 = static_cast<int>(dRect.x);
	const int dy0 = static_cast<int>(dRect.y);
	const int dx1 = static_cast<int>(dRect.z);
	const int dy1 = static_cast<int>(dRect.w);

	/* The SW renderer's present path only ever uses
	 * ShaderConvert::COPY. Anything more exotic (HDR<->LDR conversion,
	 * YUV, depth packing, etc.) is HW-renderer machinery; for the SW
	 * path we treat any non-COPY shader as a plain copy. If a future
	 * code path needs a real conversion we'll see it as bad output
	 * rather than a crash. */
	(void)shader;

	NearestCopyPS2(
		src->GetPointer(), src->GetPitch(), src->GetWidth(), src->GetHeight(),
		sRect.x, sRect.y, sRect.z, sRect.w,
		dst->GetPointer(), dst->GetPitch(),
		dx0, dy0, dx1, dy1,
		dst->GetWidth(), dst->GetHeight());
}

void GSDeviceSW::StretchRect(GSTexture* sTex, const GSVector4& sRect, GSTexture* dTex, const GSVector4& dRect,
	bool /*red*/, bool /*green*/, bool /*blue*/, bool /*alpha*/, ShaderConvert shader)
{
	/* Channel-mask variant - used by HW backends to do partial
	 * channel writes via blend equations. The SW renderer's present
	 * flow doesn't call this; route to the plain stretch so a stray
	 * call doesn't crash. */
	StretchRect(sTex, sRect, dTex, dRect, shader, true);
}

void GSDeviceSW::PresentRect(GSTexture* sTex, const GSVector4& sRect, GSTexture* dTex, const GSVector4& dRect)
{
	if (!sTex)
		return;

	GSTextureSW* src = static_cast<GSTextureSW*>(sTex);
	if (!src->GetPointer())
		return;

	/* GSRenderer::VSync signals "blank this frame" by passing
	 * sRect = (0,0,0,0). Match HW backends: clear sTex to opaque
	 * black, then sample the now-zero source. */
	const bool blank_frame = (sRect.z == 0.0f && sRect.w == 0.0f);
	if (blank_frame)
	{
		ClearRectPx(src->GetPointer(), src->GetPitch(), src->GetWidth(), src->GetHeight(), 0xFF000000u);
	}

	const float sx0 = blank_frame ? 0.0f : sRect.x;
	const float sy0 = blank_frame ? 0.0f : sRect.y;
	const float sx1 = blank_frame ? 1.0f : sRect.z;
	const float sy1 = blank_frame ? 1.0f : sRect.w;

	if (dTex)
	{
		/* GSTextureSW->GSTextureSW path: no format conversion. */
		GSTextureSW* dst = static_cast<GSTextureSW*>(dTex);
		if (!dst->GetPointer())
			return;
		NearestCopyPS2(
			src->GetPointer(), src->GetPitch(), src->GetWidth(), src->GetHeight(),
			sx0, sy0, sx1, sy1,
			dst->GetPointer(), dst->GetPitch(),
			static_cast<int>(dRect.x), static_cast<int>(dRect.y),
			static_cast<int>(dRect.z), static_cast<int>(dRect.w),
			dst->GetWidth(), dst->GetHeight());
		return;
	}

	/* dTex == nullptr: "draw to backbuffer". Write into either the
	 * frontend's direct-render buffer or our own m_present_buffer.
	 * PS2-ABGR -> XRGB8888 swizzle here - intermediate GSTextureSW
	 * textures stay in PS2 byte order; only the final hand-off to
	 * the frontend needs converting.
	 *
	 * Output dimensions are dRect.{z,w}, matching what HW backends
	 * pass to video_cb. The frontend handles fitting the frame into
	 * its window (letterboxing, scaling). */
	const int dx0 = static_cast<int>(dRect.x);
	const int dy0 = static_cast<int>(dRect.y);
	const int dx1 = static_cast<int>(dRect.z);
	const int dy1 = static_cast<int>(dRect.w);
	const int out_w = dx1 - dx0;
	const int out_h = dy1 - dy0;
	if (out_w <= 0 || out_h <= 0)
		return;

	u8* out = nullptr;
	int out_pitch = 0;

	if (AcquireDirectRenderBuffer(out_w, out_h))
	{
		out       = m_direct_render_buffer;
		out_pitch = static_cast<int>(m_direct_render_pitch);
	}
	else
	{
		EnsurePresentBuffer(out_w, out_h);
		if (!m_present_buffer)
			return;
		out       = m_present_buffer;
		out_pitch = m_present_pitch;
	}

	NearestCopyToXRGB(
		src->GetPointer(), src->GetPitch(), src->GetWidth(), src->GetHeight(),
		sx0, sy0, sx1, sy1,
		out, out_pitch,
		0, 0, out_w, out_h,
		out_w, out_h);

	m_present_width  = out_w;
	m_present_height = out_h;
	m_present_pending = true;
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
	m_present_pending = false;
	m_direct_render_buffer = nullptr;
	m_direct_render_pitch = 0;

	if (frame_skip)
		return PresentResult::FrameSkipped;

	return PresentResult::OK;
}

bool GSDeviceSW::AcquireDirectRenderBuffer(int width, int height)
{
	/* The libretro contract requires fetching the framebuffer every
	 * frame if direct rendering is used (the pointer is only valid
	 * for the current retro_run). We do still cache a "supported"
	 * flag to short-circuit the probe path when the frontend has
	 * already told us no.
	 *
	 * Re-probe on dimension change since some frontends only
	 * direct-render at specific size constraints. */
	if (!environ_cb)
		return false;

	const bool dims_changed = (width != m_present_width || height != m_present_height);
	if (!m_direct_rendering_checked || dims_changed)
	{
		m_direct_rendering_supported = true;  /* tentatively yes; fetch below decides */
		m_direct_rendering_checked   = true;
	}
	else if (!m_direct_rendering_supported)
		return false;

	retro_framebuffer fb = {};
	fb.width        = static_cast<unsigned>(width);
	fb.height       = static_cast<unsigned>(height);
	fb.access_flags = RETRO_MEMORY_ACCESS_WRITE;
	if (!environ_cb(RETRO_ENVIRONMENT_GET_CURRENT_SOFTWARE_FRAMEBUFFER, &fb) ||
		!fb.data ||
		fb.format != RETRO_PIXEL_FORMAT_XRGB8888 ||
		fb.width  != static_cast<unsigned>(width) ||
		fb.height != static_cast<unsigned>(height))
	{
		m_direct_rendering_supported = false;
		return false;
	}

	m_direct_render_buffer = static_cast<u8*>(fb.data);
	m_direct_render_pitch  = fb.pitch;
	return true;
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
	if (!m_present_pending)
	{
		/* PresentRect wasn't called this frame (no GSTexture* current
		 * to draw - very-early frames before the SW renderer has
		 * produced anything). Don't deliver an undefined buffer. */
		return;
	}

	if (m_direct_render_buffer && m_direct_render_pitch > 0)
	{
		if (video_cb)
			video_cb(m_direct_render_buffer,
				static_cast<unsigned>(m_present_width),
				static_cast<unsigned>(m_present_height),
				m_direct_render_pitch);
	}
	else if (m_present_buffer && m_present_pitch > 0)
	{
		if (video_cb)
			video_cb(m_present_buffer,
				static_cast<unsigned>(m_present_width),
				static_cast<unsigned>(m_present_height),
				static_cast<size_t>(m_present_pitch));
	}
}
