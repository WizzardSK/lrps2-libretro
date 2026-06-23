# Vendored libretro subset: only muglm is compiled. The aabb/frustum/
# interpolation/math/transforms TUs are not referenced by the libretro core
# link closure (verified via nm) and are dropped along with their headers.
add_granite_internal_lib(granite-math
        muglm/muglm.cpp muglm/muglm.hpp
        muglm/muglm_impl.hpp muglm/matrix_helper.hpp
        simd_headers.hpp)

target_include_directories(granite-math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})