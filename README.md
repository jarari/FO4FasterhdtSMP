# FO4 Faster HDT-SMP

Fallout 4 CommonLibF4/F4SE porting workspace for Faster HDT-SMP-style armor, head, and hair physics.

The current implementation is still prototype-labeled, but it now contains the main runtime path: XML discovery, defaultBBP mapping, Bullet body/mesh/constraint creation, actor/head lifecycle hooks, transform writeback, wind, actor budgeting, and rebuild handling for load/reset/customization events.

## Build

Use the release-with-debug-info profile for validation:

```bat
xmake f -y -m releasedbg
xmake -y
```

The target plugin is `FO4FasterHdtSMP.dll`. The xmake install step also installs these sample/runtime files under `F4SE/Plugins/FO4FasterHdtSMP`:

- `res/configs.xml`
- `res/defaultBBPs.xml`
- `res/prototype-sample.xml`

## Runtime Support

The plugin metadata advertises these Fallout 4 runtimes:

- 1.10.163 (`og`)
- 1.11.191 (`ae`)

The implementation is still strongest on 1.10.163. Some AE relocation IDs are present, but several OG-only call sites are intentionally guarded behind `REX::FModule::IsRuntimeOG()` until the AE hook sites are verified. Next-gen (`ng`) IDs remain disabled and are not guessed.

## Runtime Pipeline

On load, the plugin:

1. Loads `configs.xml`.
2. Reloads `defaultBBPs.xml`.
3. Clears and rebuilds the physics XML summary cache.
4. Installs lifecycle hooks and registers for F4SE messaging.
5. Creates one Bullet dynamics world for active prototype actors.

Physics can be built from:

- Armor attach/apply events from `BipedAnim`.
- Equipped biped objects discovered after actor load or reset.
- Head and hair rebuilds from `Actor::OnHeadInitialized` and `BSFaceGenUtils::PrepareHeadPart`.
- A configured `<prototypePhysicsXml>` fallback for testing.

The world steps from `Main::OnIdle` on verified OG builds, reads current bone transforms before stepping, applies Bullet simulation, and writes transforms back before the main sync/swap point. It suspends or rebuilds state around loading screens, LooksMenu, `Reset3D`, `Set3D`, and `Update3DModel`.

## Configuration

`configs.xml` contains the runtime settings:

- Solver: `<numIterations>`, `<erp>`, `<min-fps>`, `<maxSubSteps>`.
- Stability/writeback: `<clampRotations>`, `<rotationSpeedLimit>`, `<unclampedResets>`, `<unclampedResetAngle>`.
- Performance: `<budgetMs>`, `<sampleSize>`, `<enableNpcPhysics>`, `<autoAdjustMaxActors>`, `<maxActiveActors>`, `<maxActorDistance>`.
- Visibility/filtering: `<disable1stPersonViewPhysics>`, `<disableSMPHairWhenWigEquipped>`.
- Diagnostics and migration: `<enablePrototypeDiagnostics>`, `<prototypePhysicsXml>`, `<backupNodeByName>`.
- Wind: `<enabled>`, `<useWeather>`, `<windStrength>`, direction, obstruction distances, weather cooldowns, smoothing, and per-bone randomization.

Physics XML files are resolved from explicit paths, `Data`, `Data/F4SE/Plugins/FO4FasterHdtSMP`, and the legacy `Data/SKSE/Plugins/hdtSkinnedMeshConfigs` directory for migration testing.

## Physics XML Selection

Armor XML is selected from direct `NiStringExtraData` named `HDT Skinned Mesh Physics Object`, nearby attach ancestors, source/destination roots, `defaultBBPs.xml`, or the test fallback in `<prototypePhysicsXml>`.

Head and hair XML is discovered from the face/head subtree. Direct `NiStringExtraData` is preferred, but `defaultBBPs.xml` can also match source geometry. Hair detection uses the actor hair headpart model/editor keys, and hair mesh bodies can be disabled automatically when a wig occupies the hair biped slots.

See [docs/physics-xml-guide.en.md](docs/physics-xml-guide.en.md) and [docs/physics-xml-guide.ko.md](docs/physics-xml-guide.ko.md) for authoring details.
