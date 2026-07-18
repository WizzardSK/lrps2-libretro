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

#pragma once

#include "../../GSRegs.h"
#include "../../GSVector.h"
#include "../HW/GSVertexHW.h"
#include "../SW/GSVertexSW.h"

struct alignas(32) GSVertex
{
	union
	{
		struct
		{
			GIFRegST ST;       // S:0, T:4
			GIFRegRGBAQ RGBAQ; // RGBA:8, Q:12
			GIFRegXYZ XYZ;     // XY:16, Z:20
			union { u32 UV; struct { u16 U, V; }; }; // UV:24
			u32 FOG;        // FOG:28
		};

#if defined(ARCH_X86)
#if _M_SSE >= 0x500
		__m256i mx;
#endif
		__m128i m[2];
#elif defined(ARCH_ARM64)
		int32x4_t m[2];
#endif
	};
};

struct alignas(32) GSVertexPT1
{
	GSVector4 p;
	GSVector2 t;
	char pad[4];
	union { u32 c; struct { u8 r, g, b, a; }; };
};

// Vertex field accessors (ported from upstream refactor 26bd916e3,
// "GS: Add utility functions for vertices/quads"). Used by the texture-shuffle
// detection refactor. m[0] holds ST/RGBAQ, m[1] holds XYZ/UV/FOG.
__forceinline_odr GSVector4i GetVertexXY(const GSVertex& v)
{
	return GSVector4i(v.m[1]).upl16().xyxy();
}

__forceinline_odr GSVector4i GetVertexZ(const GSVertex& v)
{
	return GSVector4i(v.m[1]).yyyy();
}

__forceinline_odr GSVector4i GetVertexUV(const GSVertex& v)
{
	return GSVector4i(v.m[1]).uph16().xyxy();
}

__forceinline_odr GSVector4 GetVertexST(const GSVertex& v)
{
	return GSVector4::cast(GSVector4i(v.m[0])).xyxy();
}

__forceinline_odr GSVector4i GetVertexRGBA(const GSVertex& v)
{
	return GSVector4i(v.m[0]).uph8().upl16();
}

__forceinline_odr GSVector4 GetVertexQ(const GSVertex& v)
{
	return GSVector4::cast(GSVector4i(v.m[0])).wwww();
}

__forceinline_odr GSVector4i GetVertexFOG(const GSVertex& v)
{
	return GSVector4i(v.m[1]).wwww();
}
