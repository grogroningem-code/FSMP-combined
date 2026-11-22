set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)

# Pin the MSVC toolset that vcpkg uses to build dependencies.
#
# vcpkg selects its own compiler for dependencies independently of the CMake preset toolset, and by default picks the
# NEWEST installed Visual Studio. A mix (dependencies on one MSVC, the plugin on another) destabilised the physics
# upstream, so dependencies and plugin must use the same compiler. Upstream pins 14.44 (v143); this fork builds
# everything with MSVC 14.51 (v145): this pin and the vs2026 presets' toolset must stay the same.
#
# Included by every overlay triplet (noavx/avx/avx2/avx512) so all four variants build their dependencies with the same
# compiler.
set(VCPKG_PLATFORM_TOOLSET v145)
set(VCPKG_PLATFORM_TOOLSET_VERSION "14.51")
