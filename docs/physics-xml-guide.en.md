# FO4 Faster HDT-SMP Physics XML Guide

This guide describes the XML format and runtime behavior currently implemented by this Fallout 4 port. The format is close to FSMP/HDT-SMP XML, but only the elements described here should be treated as supported.

## File Placement

Physics XML files are resolved from these candidates:

- The path exactly as written.
- `Data/<path>` when the XML path is relative.
- `Data/F4SE/Plugins/FO4FasterHdtSMP/<path>`.
- `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/<path>` for legacy migration.

Paths must end in `.xml`.

## XML Selection

Armor XML can be selected through:

- `NiStringExtraData` named `HDT Skinned Mesh Physics Object` on the armor object.
- The same extra data on a nearby attach ancestor, source object, source root, or destination root.
- A shape-to-file match in `defaultBBPs.xml`.
- `<prototypePhysicsXml>` in `configs.xml` as a test fallback.

Head and hair XML are discovered from the actor face/head subtree after head initialization and headpart preparation. Direct `NiStringExtraData` is preferred. `defaultBBPs.xml` can also match the original FaceGen/source geometry. Hair candidates are classified with the actor hair headpart model/editor keys; other face candidates are treated as head physics.

## Minimal Structure

Every physics XML file needs a `<system>` root:

```xml
<?xml version="1.0" encoding="utf-8"?>
<system>
  ...
</system>
```

Names in XML must match FO4 NIF names. `<bone>` names match skinned `NiNode` names. `<per-vertex-shape>` and `<per-triangle-shape>` names match `BSTriShape` geometry names, or a `defaultBBPs.xml` remap alias.

## Recommended Starter XML

```xml
<?xml version="1.0" encoding="utf-8"?>
<system>
  <bone-default>
    <mass>0.15</mass>
    <linearDamping>0.35</linearDamping>
    <angularDamping>0.45</angularDamping>
    <friction>0.5</friction>
    <gravity-factor>0.6</gravity-factor>
    <wind-factor>0.0</wind-factor>
    <shape type="sphere">
      <radius>2.0</radius>
    </shape>
  </bone-default>

  <bone name="ArmorRoot">
    <mass>0</mass>
  </bone>

  <bone name="ArmorCloth01" />
  <bone name="ArmorCloth02" />

  <per-vertex-shape name="ArmorTriShapeName">
    <margin>0.5</margin>
    <penetration>1.0</penetration>
    <shared>public</shared>
    <tag>ARMOR_CLOTH</tag>
    <weight-threshold bone="ArmorCloth01">0.05</weight-threshold>
  </per-vertex-shape>

  <generic-constraint name="Root To Cloth01" bodyA="ArmorRoot" bodyB="ArmorCloth01">
    <frameInLerp>
      <translationLerp>0.5</translationLerp>
      <rotationLerp>0.5</rotationLerp>
    </frameInLerp>
    <linearLowerLimit x="0" y="0" z="0" />
    <linearUpperLimit x="0" y="0" z="0" />
    <angularLowerLimit x="-0.35" y="-0.35" z="-0.35" />
    <angularUpperLimit x="0.35" y="0.35" z="0.35" />
    <angularStiffness x="8" y="8" z="8" />
    <angularDamping x="0.35" y="0.35" z="0.35" />
  </generic-constraint>
</system>
```

Replace `ArmorRoot`, `ArmorCloth01`, `ArmorCloth02`, and `ArmorTriShapeName` with real names from the NIF.

## Bones

`<bone-default>` supplies inherited defaults. A named `<bone-default name="...">` can be referenced by `<bone template="...">`. Named defaults can use `extends="OtherTemplate"`.

Supported bone fields:

- `<mass>`: `0` or lower creates a kinematic anchor. Positive mass creates a simulated body.
- `<linearDamping>` and `<angularDamping>`.
- `<friction>`, `<rollingFriction>`, `<restitution>`.
- `<gravity-factor>`: clamped from `0.0` to `1.0`.
- `<wind-factor>`: `0.0` disables wind force for that bone.
- `<margin-multiplier>`: scales rigid-body collision margin.
- `<collision-filter>`: parsed and stored.
- `<localInertia>` or `<inertia x="..." y="..." z="..." />`.
- `<centerOfMassTransform>` with `<origin>`, `<basis>`, or `<basis-axis-angle>`.
- `<can-collide-with-bone>` and `<no-collide-with-bone>`.
- `<shape>`.

Example:

```xml
<bone name="Hair01">
  <mass>0.08</mass>
  <centerOfMassTransform>
    <origin x="0" y="0" z="-1.5" />
  </centerOfMassTransform>
  <no-collide-with-bone>HairRoot</no-collide-with-bone>
</bone>
```

## Collision Shapes

Supported `<shape>` types:

```xml
<shape type="sphere">
  <radius>2.0</radius>
</shape>

<shape type="box">
  <halfExtend x="1" y="1" z="2" />
  <margin>0.1</margin>
</shape>

<shape type="capsule">
  <radius>1.0</radius>
  <height>4.0</height>
</shape>

<shape type="cylinder">
  <radius>1.0</radius>
  <height>4.0</height>
  <margin>0.1</margin>
</shape>

<shape type="hull">
  <point x="-1" y="0" z="0" />
  <point x="1" y="0" z="0" />
  <point x="0" y="1" z="0" />
  <point x="0" y="0" z="1" />
  <margin>0.1</margin>
</shape>
```

Named shapes can be reused:

```xml
<shape name="SmallSphere" type="sphere">
  <radius>1.5</radius>
</shape>

<bone name="Hair02">
  <shape type="ref" name="SmallSphere" />
</bone>
```

Compound shapes are parsed:

```xml
<shape type="compound">
  <child>
    <transform>
      <origin x="0" y="0" z="-1" />
    </transform>
    <shape type="sphere">
      <radius>1.0</radius>
    </shape>
  </child>
</shape>
```

## Mesh Collision

Use mesh collision when simulated bones should collide against the actual skinned armor, head, or hair surface.

`<per-vertex-shape>` builds colliders from decoded vertices and is the safest first choice.

`<per-triangle-shape>` builds triangle colliders from the decoded index buffer. It is more detailed, but it is skipped when CPU index data is unavailable or invalid.

Supported mesh fields:

- `<margin>`.
- `<penetration>`. The misspelled `<prenetration>` is also accepted.
- `<shared>`: `public`, `internal`, `external`, or `private`.
- `<tag>`, `<can-collide-with-tag>`, `<no-collide-with-tag>`.
- `<can-collide-with-bone>`, `<no-collide-with-bone>`.
- `<weight-threshold bone="...">`.
- `<disable-tag>` and `<disable-priority>`.

Mesh bodies with a `disable-tag` are disabled when another active mesh body advertises that tag. If multiple mesh bodies share the same disable tag, the highest `<disable-priority>` stays enabled.

Example:

```xml
<per-vertex-shape name="HairTriShape">
  <margin>0.35</margin>
  <shared>public</shared>
  <tag>HAIR</tag>
  <weight-threshold bone="Hair03">0.03</weight-threshold>
</per-vertex-shape>
```

## Constraints

Constraints connect two parsed bone bodies. `bodyA` and `bodyB` must resolve to XML bone names.

### Generic Constraint

Generic constraints are the recommended default for armor cloth strips and hair chains.

```xml
<generic-constraint name="Hair01 To Hair02" bodyA="Hair01" bodyB="Hair02">
  <frameInLerp>
    <translationLerp>0.5</translationLerp>
    <rotationLerp>0.5</rotationLerp>
  </frameInLerp>
  <linearLowerLimit x="0" y="0" z="0" />
  <linearUpperLimit x="0" y="0" z="0" />
  <angularLowerLimit x="-0.25" y="-0.25" z="-0.25" />
  <angularUpperLimit x="0.25" y="0.25" z="0.25" />
  <angularStiffness x="6" y="6" z="6" />
  <angularDamping x="0.4" y="0.4" z="0.4" />
</generic-constraint>
```

Supported frame modes:

- `<frameInA>...</frameInA>`
- `<frameInB>...</frameInB>`
- `<frameInLerp>...</frameInLerp>`
- `<AWithXPointToB />`, `<AWithYPointToB />`, `<AWithZPointToB />`
- Lowercase dashed forms such as `<a-with-x-point-to-b />`

Supported generic fields include linear/angular limits, stiffness, damping, equilibrium, bounce, motors, servo motors, target velocity, max motor force, and ERP/CFM fields. Non-Hookean fields are parsed and warned about, but the current FO4 Bullet path does not apply them.

### Cone Twist Constraint

```xml
<conetwist-constraint name="Root To Tip" bodyA="Root" bodyB="Tip">
  <frameInLerp />
  <swingSpan1>0.4</swingSpan1>
  <swingSpan2>0.4</swingSpan2>
  <twistSpan>0.2</twistSpan>
  <limitSoftness>1.0</limitSoftness>
  <biasFactor>0.3</biasFactor>
  <relaxationFactor>1.0</relaxationFactor>
</conetwist-constraint>
```

Aliases such as `<coneLimit>`, `<planeLimit>`, `<twistLimit>`, `<limitX>`, `<limitY>`, and `<limitZ>` are parsed.

### Stiff Spring Constraint

```xml
<stiffspring-constraint name="Spring" bodyA="Root" bodyB="Tip">
  <minDistanceFactor>0.8</minDistanceFactor>
  <maxDistanceFactor>1.2</maxDistanceFactor>
  <stiffness>5.0</stiffness>
  <damping>0.25</damping>
  <equilibrium>0.5</equilibrium>
</stiffspring-constraint>
```

Constraint defaults are supported with `<generic-constraint-default>`, `<conetwist-constraint-default>`, and `<stiffspring-constraint-default>`. Constraint defaults also support `extends="OtherTemplate"`.

## defaultBBPs.xml

`defaultBBPs.xml` uses a `<default-bbps>` root.

```xml
<default-bbps>
  <map shape="ArmorTriShapeName" file="my-armor.xml" />
</default-bbps>
```

`<map>` selects an XML file when a geometry with the matching shape name is found.

`<remap>` can alias real geometry names to an XML descriptor name:

```xml
<default-bbps>
  <map shape="SharedClothShape" file="shared-cloth.xml" />

  <remap target="SharedClothShape">
    <requires>ArmorBody</requires>
    <source priority="10">ArmorCloth_L</source>
    <source priority="20">ArmorCloth_R</source>
  </remap>
</default-bbps>
```

When all `<requires>` shapes exist and one of the `<source>` shapes is present, the target shape can match the XML mesh descriptor. Higher-priority source entries are considered first.

## Runtime Notes

- XML summaries are cached and reloaded when the file timestamp changes.
- Bodies are matched from skin bones on matched geometry. If XML has mesh descriptors, all skin bones from matched geometry can be considered.
- The runtime skips suspicious skin instances, missing CPU vertex/index buffers, invalid triangle indices, unresolved bones, and duplicate/self/kinematic-only constraints.
- Bone transforms are reset/read for several frames after rebuilds to avoid exploding from stale poses.
- Loading screens suspend physics and reset bodies to current node poses on resume.
- LooksMenu suspends active prototype states and reloads armor records after customization closes.
- If `<disable1stPersonViewPhysics>` is true, first-person player physics is skipped/suspended.
- If `<disableSMPHairWhenWigEquipped>` is true, hair-domain mesh bodies are disabled while hair biped slots are occupied.
- Wind applies only when global wind is enabled and each bone has `<wind-factor>` greater than zero. Weather wind requires a valid exterior sky/weather state and uses LOS obstruction from the actor head area.

## Authoring Workflow

1. Find exact `BSTriShape` geometry names and skin bone names in the NIF.
2. Create one kinematic root bone with `<mass>0</mass>`.
3. Create dynamic bones for the moving chain or cloth section.
4. Add one mesh collision descriptor for the geometry.
5. Connect bones with generic constraints.
6. Add direct `NiStringExtraData` or a `defaultBBPs.xml` map.
7. Start with small angular limits and increase only after the chain is stable.

For hair, prefer direct `NiStringExtraData` on the hair/headpart subtree, low mass, higher damping, and `<per-vertex-shape>` before trying triangle collision.

## Troubleshooting

- If nothing builds, verify the XML has a `<system>` root and that the selected XML path resolves.
- If a defaultBBP match fails, check the real geometry name and any remap `<requires>` entries.
- If bodies build but constraints do not, check `bodyA` and `bodyB` names.
- If mesh collision is missing, check the `BSTriShape` name, skin data, and CPU vertex/index availability.
- If per-triangle collision is skipped, the CPU index buffer may not be readable.
- If a body appears offset, adjust `<centerOfMassTransform>`.
- If motion explodes, reduce angular limits, reduce mass, increase damping, or use a kinematic root.
