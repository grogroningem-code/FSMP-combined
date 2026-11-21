# Settle build of Faster HDT-SMP

This is a modified version of [Faster HDT-SMP](https://github.com/DaymareOn/hdtSMP64) 4.1.1 (commit e52ad96), used by
[Settle - Seat Collision for SMP Cloth](https://github.com/Flob26/Settle). Modified by Flob26, November 2026.

Changes (see the commit history of branch `flobrim-world-collision` for details):
- New world-collision plugin interface (`src/WorldCollisionAPI.h`, `src/hdtWorldCollider.*`): other SKSE plugins can register
  triangle meshes (e.g. furniture seats) as kinematic collision bodies for SMP cloth.
- Seated hug (`src/hdtSeatHug.*`): while an actor sits, lower skirt cloth bones can be tied to the legs.
- Small hooks in `src/CMakeLists.txt`, `src/PluginInterfaceImpl.cpp`, `src/hdtSkyrimBody.cpp`, `src/hdtSkyrimSystem.*`,
  `src/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp`, `src/hdtSkinnedMesh/hdtSkinnedMeshBody.h`.

Licensed like the original: GPL-3.0 with the exceptions in `EXCEPTIONS`. All credit for Faster HDT-SMP goes to DaymareOn and its contributors.

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
