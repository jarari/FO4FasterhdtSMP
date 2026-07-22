# FO4 Faster HDT-SMP

FO4 Faster HDT-SMP is a CommonLibF4/F4SE port of Faster HDT-SMP-style physics for Fallout 4 armor, head parts, and hair.

## Runtime support

The current code has been tested in game on:

| Runtime | Version | Status |
| --- | --- | --- |
| Fallout 4 OG | 1.10.163 | Tested |
| Fallout 4 AE | 1.11.221 | Tested |

Hooks use paired CommonLibF4 relocation IDs and runtime-specific instruction offsets for OG and AE. Other executable versions are not currently tested; do not assume that an unlisted runtime is compatible.

## Features

- Armor, head-part, and hair physics driven by physics XML files.
- XML selection through `HDT Skinned Mesh Physics Object` NIF extra data or `defaultBBPs.xml` shape mappings and remaps.
- Bullet rigid bodies, per-vertex and per-triangle skinned-mesh collision, collision filtering, and generic, cone-twist, and stiff-spring constraints.
- Fallout 4 lifecycle handling for armor attach/detach, actor load and 3D reset, head initialization, FaceGen rebuilds, LooksMenu, loading screens, and new/load game transitions.
- Runtime transform readback/writeback using Fallout 4's live armor bone bindings, with rebuild stabilization and ownership-aware cleanup.
- NPC enable/disable controls, actor distance limits, frame-time budgeting, optional automatic actor-budget adjustment, and soft suspension of distant actors.
- Optional weather-aware wind with smoothing and per-bone randomization.
- Optional in-game Bullet debug visualization and geometry diagnostics.
- Baseline, AVX2, and AVX-512 build variants.

## Requirements

- Fallout 4 1.10.163 or 1.11.221.
- A matching F4SE installation.
- Address Library for F4SE Plugins for the selected runtime.

## Installation layout

Install exactly one DLL variant. The optimized DLLs are alternatives to the baseline plugin, not additional plugins to load beside it.

```text
Data/
  F4SE/
    Plugins/
      FO4FasterHdtSMP.dll              # or the AVX2/AVX-512 DLL
      FO4FasterHdtSMP/
        configs.xml
        defaultBBPs.xml
        prototype-sample.xml
        <your physics XML files>
```

Choose the baseline `FO4FasterHdtSMP.dll` for the widest CPU compatibility. Use `FO4FasterHdtSMP-AVX2.dll` or `FO4FasterHdtSMP-AVX512.dll` only when the target CPU supports that instruction set.

## Physics XML discovery

For armor, the plugin searches for `NiStringExtraData` named `HDT Skinned Mesh Physics Object` on the relevant armor/attach objects and roots, then falls back to `defaultBBPs.xml`. Head and hair physics are discovered from the actor's face/head subtree after the corresponding FaceGen lifecycle events.

Relative XML paths are resolved in this order:

1. The path exactly as written.
2. `Data/<path>`.
3. `Data/F4SE/Plugins/FO4FasterHdtSMP/<path>`.
4. `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/<path>` for legacy migration.

`<prototypePhysicsXml>` in `configs.xml` remains available as a global test fallback, but normal armor/head/hair setups should use NIF extra data or `defaultBBPs.xml`.

See the complete [English physics XML guide](docs/physics-xml-guide.en.md) or [Korean physics XML guide](docs/physics-xml-guide.ko.md) for the supported schema, examples, runtime behavior, and troubleshooting advice.

## Configuration

Runtime settings live in `Data/F4SE/Plugins/FO4FasterHdtSMP/configs.xml`. The shipped file documents the current defaults and includes:

- Solver controls: iterations, ERP, minimum simulation FPS, and maximum substeps.
- Stability controls: rotation clamping, rotation speed limits, and reset thresholds.
- Performance controls: frame budget, timing mode, sample size, actor limits, and actor distance.
- Visibility controls: first-person physics, NPC physics, and SMP hair suppression while a wig is equipped.
- Diagnostics: log level, geometry diagnostics, Bullet visualization, and the prototype XML fallback.
- Wind: fixed or weather-derived wind, strength/direction, distance falloff, cooldowns, smoothing, and per-bone randomization.

The runtime log is written to `Documents/My Games/Fallout4/F4SE/FO4FasterHdtSMP.log`.

## Building from source

Builds require Windows, an MSVC toolchain with C++23 support, [xmake](https://xmake.io/) 3.0 or newer, and the CommonLibF4 submodule.

```bat
git submodule update --init --recursive
xmake f -y -m releasedbg
```

Build one CPU variant:

```bat
xmake -y FO4FasterHdtSMP
xmake -y FO4FasterHdtSMP-AVX2
xmake -y FO4FasterHdtSMP-AVX512
```

Each command writes its DLL and PDB under `build/windows/x64/releasedbg`. To create an installable staging directory for one variant:

```bat
xmake install -y -o build/package FO4FasterHdtSMP
```

Replace the final target name with the AVX2 or AVX-512 target when packaging an optimized build. The install step also includes `configs.xml`, `defaultBBPs.xml`, and `prototype-sample.xml` under `F4SE/Plugins/FO4FasterHdtSMP`.

## License

FO4 Faster HDT-SMP is licensed under [GPL-3.0](LICENSE) with the additional modding/linking terms in [EXCEPTIONS](EXCEPTIONS).
