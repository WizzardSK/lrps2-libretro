# Minimal CMakeLists for the vendored copy of WIL.
#
# Upstream's CMakeLists is geared towards building WIL's own test
# suite, packaging, and installation logic.  None of that is needed
# here -- the libretro core build only uses WIL as a header-only
# library.  Replicate just the public interface target the upstream
# build exports, so consumers can keep linking against `WIL::WIL`
# unchanged.

add_library(WIL INTERFACE)
add_library(WIL::WIL ALIAS WIL)

target_include_directories(WIL
	INTERFACE
	"${CMAKE_CURRENT_SOURCE_DIR}/include"
)
