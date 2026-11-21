# FSMP combined build

A personal build of [Faster HDT-SMP](https://github.com/DaymareOn/hdtSMP64) 4.1.1 (FSMP) that puts several
GPL-3.0 changes into one `hdtSMP64.dll`, since each of them ships its own copy of that file and only one can load.

Work in progress: it compiles, but it has not been tested in game yet. There is no Nexus release.

## What is in it

| Change | Source | Author |
|---|---|---|
| Faster HDT-SMP 4.1.1 (commit e52ad96) | [DaymareOn/hdtSMP64](https://github.com/DaymareOn/hdtSMP64) | DaymareOn and contributors |
| Settle 1.1.1: world-collision interface for seats and mounts, seated skirt hug | [Flob26/Settle-hdtSMP64](https://github.com/Flob26/Settle-hdtSMP64), branch `settle-guard` | Flob26 |
| Clothing Clipping Fix: carries cloth through teleports, restarts it after pose swaps, holds new outfits at rest until the pose settles | [YoucefNabil/FSMP-Discontinuity-Guard](https://github.com/YoucefNabil/FSMP-Discontinuity-Guard) (merged through Settle's `settle-guard`) | Youcef |
| Isolate TBB loops run under the physics lock (deadlock crash) | upstream PR #484, unmerged | Alan Tse |
| Drain the physics task queue on FrameEvent (step and setup race) | upstream PR #428, unmerged | DaymareOn |
| Cache whether a skeleton is first person | upstream PR #317, unmerged | asdasdduck |
| Build with Visual Studio 2026 (MSVC 14.51) for the plugin and all vcpkg dependencies | upstream branch `vs2026`, plus `cmake/triplets/_pin_toolset.cmake` | DaymareOn, grogroningem-code |

The commit history keeps every author. Upstream pins its dependencies to MSVC 14.44; this build uses 14.51 for
everything instead, which upstream has not tested.

## Building

Same as upstream, with the VS 2026 presets: `cmake --preset vs2026-windows-avx2 -T v145
-DCompiledPluginsPath=<output folder> -DCOPY_OUTPUT=OFF`, then `cmake --build out/build/vs2026-windows-avx2
--config Release`.

## License

GPL-3.0 with the exceptions in `EXCEPTIONS`, like the original. All credit for Faster HDT-SMP goes to DaymareOn and
its contributors, and for the merged changes to the authors listed above.

---

# Faster HDT-SMP (FSMP)

Skinned-mesh physics for Skyrim **SE / AE / VR** — the actively-maintained continuation of HDT-SMP.

Maintained by DaydreamingDay / DaymareOn. Fork of [Karonar1's hdtSMP64](https://github.com/Karonar1/hdtSMP64), itself a fork of the [version](https://github.com/aers/hdtSMP64) by aers, from the [original code](https://github.com/HydrogensaysHDT/hdt-skyrimse-mods) by HydrogensaysHDT.

Mod page: <https://www.nexusmods.com/skyrimspecialedition/mods/57339>

## Documentation

Most documentation lives in the wikis; both have a sidebar linking every page.

- **Players** — the [FSMP wiki](https://github.com/DaymareOn/hdtSMP64/wiki) covers installation, configuration, the in-game config menu (via the [SKSE Menu Framework](https://www.nexusmods.com/skyrimspecialedition/mods/120352)), console commands, solving problems, and the changelog.
- **Mod authors** — the [SMP Modder Guide](https://github.com/DaymareOn/FSMP-Validator/wiki) covers authoring the physics XML and meshes, next to the XSD/Schematron schemas that define a valid file. See also `smp report` (validate a whole load order from the console) and the DynamicHDT Papyrus API (control physics from scripts), both in the FSMP wiki.
- **Developers** — building FSMP, the `smp_replay` benchmark, and the code analyses are in the wiki's "Building FSMP" section. Build steps: [How to compile your own FSMP](https://github.com/DaymareOn/hdtSMP64/wiki/6-%E2%80%90-How-to-compile-your-own-FSMP).

## Changes

For the full, maintained changelog, see the [wiki Changelog](https://github.com/DaymareOn/hdtSMP64/wiki).

## Known issues

For current known bugs, see the [Nexus bugs tab](https://nexusmods.com/skyrimspecialedition/mods/57339?tab=bugs) and [GitHub issues](https://github.com/DaymareOn/hdtSMP64/issues).

## Credits

See the [Hall of Fame](https://github.com/DaymareOn/hdtSMP64/wiki/01-%E2%80%90-Hall-of-Fame) for the full list of contributors and acknowledgements.
