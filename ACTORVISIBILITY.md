# FSMP + ActorVisibility (fork notes)

This fork is based on DaymareOn/hdtSMP64 `master` at 2215ec3 ("bump FSMP to version 4.1.1", the build shipped on Nexus as 4.1.1). It is GPL-3.0 with the upstream EXCEPTIONS, unchanged. It adds one optional setting that consults **ActorVisibility.dll** (a separate MIT SKSE plugin) at runtime.

## Setting
In `SKSE/Plugins/hdtSkinnedMeshConfigs/configs.json` (or your `userConfigs.json`), section `smp`, add `"actorVisibilityMode"`. It can also be changed in the in-game menu under Distance / screen-size culling.

| value | effect |
|---|---|
| 0 | Off (default). Stock FSMP culling, byte-for-byte the same decision. |
| 1 | Also cull: a skeleton that ActorVisibility reports OFFSCREEN/OCCLUDED is made inactive (and skips FSMP's own LOS ray). Visible/unknown skeletons still go through FSMP's own frustum + LOS test, so this mode can only deactivate *more* skeletons than stock. |
| 2 | Replace: for skeletons ActorVisibility reports VISIBLE, FSMP skips its own per-frame `NodeInFrustum` + `Actor_CalculateLOS` (the screen-size test still applies). Hidden ones are inactive. Unknown ones fall back to stock. ActorVisibility's hysteresis and multi-point/static-only rays replace FSMP's single per-frame ray, so there is less flicker and fewer LOS calls. |

Rules that apply in every mode:
- `minCullingDistance` (always-on distance) is checked first and is never overridden.
- The player is never queried.
- `maximumActiveSkeletons`, the budget auto-adjust, `skipDeadActors` and `minScreenSizePercent` all compose unchanged.

The DLL is resolved at SKSE `kPostLoad` with `GetModuleHandleA("ActorVisibility.dll")` and `GetProcAddress("AV_GetApiVersion"/"AV_GetState")`, and the API version must be 1. If it is missing, everything behaves as mode 0, and the log says so.

Wake-up: re-activation uses FSMP's normal path (`PhysicsItem::updateActive` → `SkyrimPhysicsWorld::addSkinnedMeshSystem`, which sets `m_initialized = false`, so the next `readTransform` resets the system to the animated pose with zero velocity). This is exactly what happens today when an NPC walks back into view.

## Why no "reduced rate" mode
All active systems share one Bullet world and are stepped together (`SkyrimPhysicsWorld::doUpdate2ndStep` → `stepSimulation`). There is no per-system step rate, and an inactive system is removed from the world entirely (it is reset on wake). A per-skeleton "simulate every N frames" would need changes inside the solver (freezing bodies or per-system substeps), which is risky for explosions, so it was not done.

## Files changed
- `src/ActorVisibilityBridge.h/.cpp`: new, the runtime loader.
- `src/ActorManager.h/.cpp`: the `m_actorVisibilityMode` member and the check in `Skeleton::isInPlayerView()`.
- `src/GlobalConfig.h/.cpp`, `src/config.cpp`, `configs/configs.json`: the key, with a [0,2] clamp, JSON round-trip and log line.
- `src/UI/FSMPMenu.cpp`: the menu row.
- `src/main.cpp`: the `kPostLoad` call.
- `src/CMakeLists.txt`: the new sources.
- `CMakePresets.json`: `vs2026-windows*` presets (default VS 2026 toolset) and build presets.
- `tests/config/test_globalconfig.cpp`: a clamp/default test.

## Non-actor skeleton culling (`smp.cullNonActorSkeletons`)

Intended for **SMP banners** (and any other non-actor SMP skeleton). Two registration paths are covered:

1. **ActorManager non-actor skeletons** — stock FSMP can register an attach-armor skeleton with an "NPC" node whose owner is not an `Actor`. The old `isInPlayerView()` early-returned `true` for those. Behind this setting they use the same padded frustum + staggered havok LOS as below (no `minScreenSizePercent` gate — banners are thin and would always fail it).
2. **World systems added outside ActorManager** — [Banner Physics HDT-SMP](https://www.nexusmods.com/skyrimspecialedition/mods/195831) (JPSteel2) creates `SkyrimSystem`s via `scanBBP` / `createOrUpdateSystem` and `SkyrimPhysicsWorld::addSkinnedMeshSystem`, then forces `activeSkeletons` so the world keeps ticking. Those systems never appear in ActorManager, so culling runs in `SkyrimPhysicsWorld::doUpdate` before `readTransform`: out-of-view / occluded banners are soft-removed from the Bullet world (pose frozen on the Ni nodes) and re-attached with `addSkinnedMeshSystemKeepPose` (no `RESET_PHYSICS`, no `m_initialized = false`) when they become visible again. Per-system hysteresis and log state live in a `.cpp` static map keyed by pointer — **no** changes to `SkyrimPhysicsWorld` / `SkinnedMeshSystem` / `ActorManager` layout (required by SMPFixes and Banner Physics `Detect()`).

| value | effect |
|---|---|
| 0 | Off (default in shipped `configs.json`). Stock behaviour: non-actor / banner systems stay active. |
| 1 | Cull banners and other non-actor SMP systems: padded frustum (`1.25 * radius + 64`), staggered multi-point havok LOS (up to 4 world-system rays per frame; ActorManager non-actors have their own budget of 4). Hide after 2 consecutive fails; show on the first pass. Near-camera systems inside `minCullingDistance` stay active. |

`position()` falls back to the owner's 3D world translate (then the skeleton node) when `"NPC Root [Root]"` is missing, regardless of this setting.

**Banner Physics PDB:** this custom `hdtSMP64.dll` is not in Banner Physics' SHA whitelist. Ship the matching `hdtSMP64.pdb` next to the DLL (Release already emits `/DEBUG:FULL` and the post-build copies both into `CompiledPluginsPath\SKSE\Plugins\`). Without it Banner Physics logs `Unsupported FSMP binary` and installs no hooks.

Additional files: `src/NonActorRayCaster.h/.cpp` (MIT approach from our ActorVisibility RayCaster, adapted for `TESObjectREFR` / CommonLibVR-ng APIs); `SkinnedMeshWorld::addSkinnedMeshSystemKeepPose`.
