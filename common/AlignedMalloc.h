/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2010  PCSX2 Dev Team
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

#include "Pcsx2Defs.h"
#include <cstring>
#include <cstdlib>
#include <new> // std::bad_alloc
#include <memory>
#include <type_traits>
#include <utility>

#if defined(_WIN32)
#include <malloc.h>
#endif

// Implementation note: all known implementations of _aligned_free check the pointer for
// NULL status (our implementation under GCC, and microsoft's under MSVC), so no need to
// do it here.
#define safe_aligned_free(ptr) \
	((void)(_aligned_free(ptr), (ptr) = NULL))

// aligned_malloc: Implement/declare linux equivalents here!
#if !defined(_WIN32)
extern void* _aligned_malloc(size_t size, size_t align);
extern void _aligned_free(void* pmem);
#endif

// pcsx2_aligned_realloc differs from MSVCRT's _aligned_realloc in one
// important way: callers (Vif_HashBucket::add) are allowed to change the
// alignment between the original allocation and the realloc.  MSDN
// explicitly says:
//   "It's an error to reallocate memory and change the alignment of a block."
// (cpp-docs/c-runtime-library/reference/aligned-realloc.md)
// So we cannot just forward to _aligned_realloc on Windows; we have to
// emulate the POSIX-side behaviour of malloc-new + memcpy + free-old.
// We provide an inline definition for every platform so no out-of-line
// symbol is needed (matches the historical mingw situation where
// AlignedMalloc.cpp's #if !defined(_WIN32) guard left the symbol
// undefined - the build only ever linked because the mingw pre-d2d1ebc
// __forceinline behaviour kept the call inlined into a single site).
static inline void* pcsx2_aligned_realloc(void* handle, size_t new_size, size_t align, size_t old_size)
{
	void* newbuf = _aligned_malloc(new_size, align);

	if (newbuf != NULL && handle != NULL)
	{
		std::memcpy(newbuf, handle, old_size < new_size ? old_size : new_size);
		_aligned_free(handle);
	}
	return newbuf;
}
