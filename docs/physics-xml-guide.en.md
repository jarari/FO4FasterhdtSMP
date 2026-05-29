# FO4 Faster HDT-SMP Physics XML Guide

This guide describes the physics XML format currently parsed by this Fallout 4 port. The format is intentionally close to FSMP/HDT-SMP XML, but only the elements described here should be considered supported for armor and hair authoring.

## File Placement

Physics XML files are resolved from the plugin config paths:

- `Data/F4SE/Plugins/FO4FasterHdtSMP/`
- `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/` for legacy compatibility

Armor XML can be selected in three ways:

- Add `NiStringExtraData` named `HDT Skinned Mesh Physics Object` to the armor NIF object or a nearby parent. Its string data should be the XML path.
- Add a shape-to-file entry in `defaultBBPs.xml`.
- Set `<prototypePhysicsXml>` in `configs.xml` as a fallback while testing.

Hair and head-part XML currently require direct `NiStringExtraData` named `HDT Skinned Mesh Physics Object` on the head/hair subtree. `defaultBBPs.xml` is armor-oriented and is not the recommended discovery path for hair.

## Minimal Structure

Every physics XML file needs a `<system>` root:

```xml
<?xml version="1.0" encoding="utf-8"?>
<system>
  ...
</system>
```

Names in XML must match the actual FO4 skin bone and geometry names from the NIF. A `<bone>` name should match a skinned bone node. A `<per-vertex-shape>` or `<per-triangle-shape>` name should match a `BSTriShape` geometry name.

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
    <linearStiffness x="0" y="0" z="0" />
    <angularStiffness x="8" y="8" z="8" />
    <linearDamping x="0" y="0" z="0" />
    <angularDamping x="0.35" y="0.35" z="0.35" />
  </generic-constraint>
</system>
```

Replace `ArmorRoot`, `ArmorCloth01`, `ArmorCloth02`, and `ArmorTriShapeName` with real names from the NIF.

## Bones

`<bone-default>` supplies inherited defaults for later `<bone>` entries. A named `<bone-default name="...">` can be used as a template and then referenced with `<bone template="...">`.

Supported bone fields:

- `<mass>`: `0` or lower creates a kinematic anchor body. Positive mass creates a simulated body.
- `<linearDamping>` and `<angularDamping>`: reduce velocity and rotation.
- `<friction>`, `<rollingFriction>`, `<restitution>`: Bullet rigid body material values.
- `<gravity-factor>`: clamped from `0.0` to `1.0`.
- `<wind-factor>`: `0.0` disables wind response for that bone.
- `<margin-multiplier>`: collision margin scale used by skinned mesh collision.
- `<collision-filter>`: parsed, reserved for filtering behavior.
- `<localInertia>` or `<inertia x="..." y="..." z="..." />`: optional explicit inertia.
- `<centerOfMassTransform>`: moves/rotates the body relative to the node.
- `<can-collide-with-bone>` and `<no-collide-with-bone>`: bone collision allow/deny lists.
- `<shape>`: rigid body collision shape.

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

Compound shapes are also parsed:

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

Use mesh collision when the simulated bones should collide against the actual skinned armor or hair surface.

`<per-vertex-shape>` builds colliders from vertices. It is cheaper and is a good first choice.

`<per-triangle-shape>` builds triangle colliders from the mesh index buffer. It is more detailed but requires readable CPU index data.

Supported mesh fields:

- `<margin>`: collision margin.
- `<penetration>`: penetration tolerance. The misspelled `<prenetration>` is also accepted for compatibility.
- `<shared>`: `public`, `internal`, `external`, or `private`.
- `<tag>`, `<can-collide-with-tag>`, `<no-collide-with-tag>`.
- `<can-collide-with-bone>`, `<no-collide-with-bone>`.
- `<weight-threshold bone="...">`: ignores weak vertex influence for a specific bone.
- `<disable-tag>` and `<disable-priority>`.

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

Constraints connect two XML bodies. `bodyA` and `bodyB` must resolve to parsed bones.

### Generic Constraint

This is the recommended default constraint for armor cloth strips and hair chains.

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
- Lowercase dashed forms like `<a-with-x-point-to-b />`

Supported generic fields include linear/angular limits, stiffness, damping, equilibrium, bounce, motors, servo motors, target velocity, max motor force, and ERP/CFM fields.

Non-Hookean fields are parsed but currently not applied by the FO4 Bullet constraint path.

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

Aliases such as `<coneLimit>`, `<planeLimit>`, `<twistLimit>`, `<limitX>`, `<limitY>`, and `<limitZ>` are also parsed.

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

## Templates

Templates reduce repetition:

```xml
<bone-default name="HairDynamic">
  <mass>0.08</mass>
  <linearDamping>0.3</linearDamping>
  <angularDamping>0.5</angularDamping>
</bone-default>

<bone name="Hair01" template="HairDynamic" />
<bone name="Hair02" template="HairDynamic" />

<generic-constraint-default name="SoftJoint">
  <frameInLerp />
  <linearLowerLimit x="0" y="0" z="0" />
  <linearUpperLimit x="0" y="0" z="0" />
  <angularLowerLimit x="-0.25" y="-0.25" z="-0.25" />
  <angularUpperLimit x="0.25" y="0.25" z="0.25" />
</generic-constraint-default>

<generic-constraint template="SoftJoint" bodyA="Hair01" bodyB="Hair02" />
```

Named defaults can use `extends="OtherTemplate"`.

## Armor Workflow

1. Find the exact `BSTriShape` geometry name and skin bone names in the armor NIF.
2. Create one kinematic root bone with `<mass>0</mass>`.
3. Create dynamic bones for the moving chain or cloth section.
4. Add one mesh collision descriptor for the armor geometry.
5. Connect the bones with generic constraints.
6. Add direct `NiStringExtraData` or a `defaultBBPs.xml` map.
7. Start with small angular limits and increase only after the chain is stable.

## Hair Workflow

1. Find the hair head-part model and its skinned hair geometry.
2. Put `NiStringExtraData` named `HDT Skinned Mesh Physics Object` on the hair subtree.
3. Use a kinematic root near the scalp and dynamic bones down the hair chain.
4. Use low mass and higher angular damping than armor.
5. Prefer `<per-vertex-shape>` first; use `<per-triangle-shape>` only if needed.

## Troubleshooting

- If nothing builds, verify the XML has a `<system>` root and that the selected XML path resolves.
- If bodies build but constraints do not, check `bodyA` and `bodyB` names.
- If mesh collision is missing, check that the mesh name matches the actual `BSTriShape` name and that the mesh has skin data.
- If per-triangle collision is skipped, the CPU index buffer may not be readable.
- If a body appears offset, adjust `<centerOfMassTransform>`.
- If motion explodes, reduce angular limits, reduce mass, increase damping, or use a kinematic root.
