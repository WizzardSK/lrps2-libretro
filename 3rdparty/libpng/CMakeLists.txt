add_library(png
	png.c
	pngerror.c
	pngget.c
	pngmem.c
	pngpread.c
	pngread.c
	pngrio.c
	pngrtran.c
	pngrutil.c
	pngset.c
	pngtrans.c
	pngwio.c
	pngwrite.c
	pngwtran.c
	pngwutil.c
)

target_link_libraries(png PRIVATE ZLIB::ZLIB)
target_include_directories(png PUBLIC .)

# The arm/ NEON sources aren't part of this build, but pngpriv.h auto-enables the
# NEON filter optimisation on aarch64 (referencing png_init_filter_functions_neon).
# Disable it on non-x86 so the bundled libpng links without those sources.
if(NOT _M_X86)
	target_compile_definitions(png PRIVATE PNG_ARM_NEON_OPT=0)
endif()

add_library(PNG::PNG ALIAS png)
