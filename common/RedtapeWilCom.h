/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2022  PCSX2 Dev Team
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

#ifdef _WIN32

#include "RedtapeWindows.h"

// ---------------------------------------------------------------------------
// Minimal wil::com_ptr_nothrow<T> shim built on Microsoft::WRL::ComPtr<T>.
//
// The codebase used Microsoft's WIL (Windows Implementation Library) for
// exactly one feature: the `wil::com_ptr_nothrow<T>` smart pointer.  WIL
// itself is a 100k+-line vendored dependency whose remaining surface area
// here -- a refcounted COM smart pointer -- is exactly what
// Microsoft::WRL::ComPtr<T> already provides.  WRL ships in both the MSVC
// Windows SDK (<wrl/client.h>) and mingw-w64's headers under the same name
// and identical API, so a thin alias keeps source files unchanged while
// removing the WIL dependency entirely.
//
// The methods exposed below are exactly those used in the LRPS2 sources:
//   .get(), .put(), .reset(), .detach(), .addressof()
// plus the inherited operator->/operator bool/copy/move from WRL::ComPtr.
//
// operator& is overridden to return T** rather than WRL's ComPtrRef proxy.
// IUnknown::QueryInterface has a templated overload `QueryInterface(Q** pp)`
// whose argument deduction does not consider the user-defined conversions
// ComPtrRef provides, so `obj->QueryInterface(&p)` fails to compile against
// the WRL ComPtrRef return type.  Returning T** directly matches WIL's
// original semantics and works for both QueryInterface(&p) and the
// IID_PPV_ARGS(&p) and IID_PPV_ARGS(p.put()) forms on both MSVC and mingw.
// ---------------------------------------------------------------------------
#include <wrl/client.h>

namespace wil
{
	template <typename T>
	class com_ptr_nothrow : public Microsoft::WRL::ComPtr<T>
	{
		using Base = Microsoft::WRL::ComPtr<T>;

	public:
		using Base::Base;
		using Base::operator=;

		com_ptr_nothrow() = default;

		T*  get()       const noexcept { return this->Get(); }
		T** put()             noexcept { return this->ReleaseAndGetAddressOf(); }
		T** addressof()       noexcept { return this->GetAddressOf(); }
		void reset()          noexcept { this->Reset(); }
		T*  detach()          noexcept { return this->Detach(); }

		// Hide WRL's operator& (which returns ComPtrRef) -- see comment above.
		T** operator&()       noexcept { return this->ReleaseAndGetAddressOf(); }
	};
} // namespace wil

#endif
