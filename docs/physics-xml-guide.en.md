# FO4 Faster HDT-SMP Physics XML Guide

This guide documents the XML format currently implemented by this Fallout 4 port, plus the Bullet behavior that actually matters at runtime. The supported surface is narrower than the older FSMP/HDT-SMP ecosystem, so treat anything not described here as unsupported unless the code explicitly says otherwise.

## File Placement

XML files are resolved from these locations:

- The path exactly as written.
- `Data/<path>` when the path is relative.
- `Data/F4SE/Plugins/FO4FasterHdtSMP/<path>`.
- `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/<path>` for legacy migration.

The path must end in `.xml`.

## How XML Is Chosen

Armor XML can come from:

- `NiStringExtraData` named `HDT Skinned Mesh Physics Object` on the armor object.
- The same extra data on a nearby attach ancestor, source object, source root, or destination root.
- A shape-to-file match in `defaultBBPs.xml`.
- `<prototypePhysicsXml>` in `configs.xml`, which is intended as a prototype/test fallback.

Head and hair XML are discovered from the actor face/head subtree after head initialization and headpart preparation. Direct `NiStringExtraData` wins first. `defaultBBPs.xml` can also match the original FaceGen/source geometry. Hair candidates are classified with the actor hair headpart model/editor keys; the remaining face candidates are treated as head physics.

## XML Rules

Every physics XML file needs a `<system>` root:

```xml
<?xml version="1.0" encoding="utf-8"?>
<system>
  ...
</system>
```

Important parsing rules:

- Element and attribute names are case-sensitive.
- Numeric text is trimmed.
- Floats accept either `.` or a single `,` as the decimal separator.
- Booleans accept `1`, `0`, `true`, and `false`.
- Template and named-shape lookups are order-sensitive. Define a thing before referencing it.
- Unknown elements are ignored unless they are part of a supported block.

## Naming and Inheritance

The parser supports named templates for bones and constraints.

- `<bone-default>` provides inherited defaults.
- A blank/unnamed `<bone-default>` becomes the global fallback for bones.
- `<bone-default name="X" extends="Y">` copies `Y` first, then overrides it.
- `<bone name="X" template="Y">` copies template `Y` first, then applies the local bone settings.
- The same pattern applies to `<generic-constraint-default>`, `<conetwist-constraint-default>`, and `<stiffspring-constraint-default>`.

Template lookup is local to the file and happens as the file is parsed. A template must already exist when it is referenced.

## Bones

Bone descriptors define rigid bodies for skinned bones.

### Bone Fields

- `<mass>`: `0` or lower makes the bone a kinematic anchor. Positive mass makes it a dynamic rigid body. Negative values are treated like `0` at build time.
- `<linearDamping>` and `<angularDamping>`: Bullet damping values for the rigid body.
- `<friction>`: Coulomb friction for the rigid body. Higher values resist sliding when the body is in contact with something else. `0` makes the body slippery. This only matters during contact, not while the body is moving freely.
- `<rollingFriction>`: resistance to rolling or tumbling motion at the contact point. This matters most for spheres, capsules, cylinders, or rounded parts that roll instead of slide. It is usually less useful for flat cloth pieces.
- `<restitution>`: bounce or rebound. `0` means no bounce. Higher values make the body return more energy after impact. The runtime clamps the value to `0` or above, but in practice you usually want to stay near the `0` to `1` range.
- `<gravity-factor>`: clamped to `0.0` to `1.0`. `0` removes gravity from the bone, `1` keeps full world gravity.
- `<wind-factor>`: negative values are clamped to `0`. Wind only matters when global wind is enabled and the runtime has a valid wind/weather state.
- `<margin-multiplier>`: scales the per-bone collision margin used by skinned mesh collision. This is not the same as a Bullet rigid-body margin.
- `<collision-filter>`: parsed and stored, but currently not applied by the build path.
- `<localInertia>` or `<inertia x="..." y="..." z="..." />`: explicit local inertia. If supplied, negative components are clamped to `0`. If omitted and the mass is positive, Bullet computes inertia from the shape.
- `<centerOfMassTransform>`: local offset/orientation between the node and the rigid body. Use this when the node origin is not the physics pivot you want.
- `<can-collide-with-bone>` and `<no-collide-with-bone>`: bone-level collision filters. If any allow-list entries exist, only those bones collide. Otherwise the deny-list is used.
- `<shape>`: the collision shape for the bone. If omitted, the bone gets an empty shape.

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

### When To Use Each Bone Setting

- Use `mass = 0` for anchor bones, root bones, and any body that should follow animation exactly.
- Use a small positive mass for cloth strips, tails, straps, and hair chains.
- Use higher damping when motion needs to settle quickly or avoid oscillation.
- Use `friction` when you want a part to stop sliding too easily, such as a tail root, a strap that should catch, or a rounded collider that should not skate across surfaces.
- Use `rollingFriction` when the shape tends to roll and you want to slow that rotation down, such as spherical ornaments, capsules, or rod-like pieces.
- Use `restitution` when you want a part to bounce back after contact instead of dead-stopping. Keep it low for cloth and hair.
- Use `centerOfMassTransform` when the simulated pivot is offset from the bone origin.
- Leave `collision-filter` alone unless you know another part of the runtime consumes it later.

## Collision Shapes

Named shapes can be reused with `<shape type="ref" name="...">`. The reference must resolve to a shape that has already been parsed.

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

Compound shapes are supported:

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

Shape behavior:

- `sphere`: radius is clamped to a minimum of `0.01`.
- `box`: half extents are each clamped to a minimum of `0.01`. `margin` is applied to the Bullet box shape.
- `capsule`: radius and height are each clamped to a minimum of `0.01`. This shape does not take a custom margin field in this implementation.
- `cylinder`: radius and height are each clamped to a minimum of `0.01`. `margin` is applied.
- `hull`: adds the listed points to a convex hull. An empty point list is invalid.
- `compound`: each child can contain its own `<transform>` and `<shape>`. An empty compound is invalid.
- `ref`: copies a previously defined named shape.

### When To Use Each Shape

- `sphere`: simple anchors, tails, and tips.
- `box`: flat or blocky pieces, especially when you want a stable, predictable collider.
- `capsule`: chains and limbs where rounded ends make sense.
- `cylinder`: vertical or rod-like pieces where a capsule is too rounded.
- `hull`: irregular shapes that still need a convex collider.
- `compound`: one bone needs multiple primitive colliders instead of one approximate hull.

## Mesh Collision

Mesh descriptors build colliders from skinned armor, head, or hair geometry.

Supported mesh fields:

- `<margin>`: collision padding.
- `<penetration>`: extra padding floor used by triangle collision. The misspelled `<prenetration>` is also accepted.
- `<shared>`: `public`, `internal`, `external`, or `private`.
- `<tag>`: collision tag used for grouping and mutual exclusion.
- `<can-collide-with-tag>` and `<no-collide-with-tag>`: tag filters.
- `<can-collide-with-bone>` and `<no-collide-with-bone>`: bone filters.
- `<weight-threshold bone="...">`: minimum bone influence weight for that bone to participate in the collider.
- `<disable-tag>` and `<disable-priority>`: mutually exclusive mesh group control.

Example:

```xml
<per-vertex-shape name="HairTriShape">
  <margin>0.35</margin>
  <shared>public</shared>
  <tag>HAIR</tag>
  <weight-threshold bone="Hair03">0.03</weight-threshold>
</per-vertex-shape>
```

### Per-Vertex Shape

`<per-vertex-shape>` builds collision from decoded vertices. It is the safest default because it is less sensitive to triangle quality and missing index data.

Use it when:

- You want the least fragile first pass for armor, cloth, or hair.
- The mesh has good vertex weights but complex triangles.
- You want collision that tracks the skinned surface without paying for triangle collision.

### Per-Triangle Shape

`<per-triangle-shape>` builds collision from the decoded index buffer. It is more detailed, but also heavier and more fragile.

Use it when:

- The surface needs sharper contact than per-vertex collision can give you.
- The mesh has a reliable CPU index buffer.
- You are willing to trade cost for detail.

The runtime skips triangle bodies when the CPU index buffer is missing, invalid, or cannot be read.

### Shared Scope

`<shared>` controls who a mesh body can collide with:

- `public`: collide with any compatible body.
- `internal`: collide only with bodies on the same actor.
- `external`: collide only with bodies on other actors.
- `private`: collide only with bodies on the same actor and the same build group.

Use `private` for mutually dependent pieces inside one build group. Use `internal` when related parts on the same actor should still interact. Use `external` when the body should only matter against other actors. Use `public` for the normal case.

The runtime also disables collision when both mesh bodies are kinematic.

### Tag Filters

Tag filters work after shared-scope checks:

- If `can-collide-with-tag` is empty, `no-collide-with-tag` acts as a deny-list.
- If one or more `can-collide-with-tag` entries exist, they become an allow-list and the deny-list is ignored for matching.

Use tags to separate armor variants, prevent self-collision between related pieces, or keep alternate mesh bodies from activating one another.

### Bone Filters

`can-collide-with-bone` and `no-collide-with-bone` filter against resolved prototype bones, not against raw XML text alone. If a bone name cannot be resolved, the entry is ignored and logged.

Use bone filters when a mesh should only interact with a small part of the skeleton, or when one bone should be excluded from a specific collider group.

### Weight Thresholds

`<weight-threshold bone="...">` trims collider participation for the named bone.

Use it when:

- A vertex carries a small accidental influence from a bone you do not want to drive collision.
- A mesh has blended weights and you want to keep the collider focused on the dominant bone.
- You need to suppress noisy edges without editing the mesh weights themselves.

### Disable Groups

`<disable-tag>` and `<disable-priority>` implement mutually exclusive mesh groups.

Behavior:

- If any active mesh body already advertises the same disable tag, every body in that group is disabled.
- If the group is not already active, the body with the highest `disable-priority` stays enabled and the rest are disabled.

Use this for alternate variants that should never run at the same time, such as layered hair pieces, optional armor attachments, or multiple versions of the same geometry.

Hair bodies can also be disabled automatically when the runtime's wig-equipment setting says to suppress SMP hair.

### Margin and Penetration

`<margin>` and `<penetration>` matter differently depending on the mesh type:

- Per-vertex collision uses `margin` as its padding factor.
- Per-triangle collision uses `margin` and `penetration` together. The triangle bounds expand by the averaged margin, but `penetration` provides the minimum floor for that expansion.

Practical rule:

- Increase `margin` when collision happens too late or slips through thin shapes.
- Increase `penetration` when triangle collision feels too tight or needs a stronger contact buffer.

## Constraints

Constraints connect two parsed bone bodies. `bodyA` and `bodyB` must match XML bone names that were actually resolved into bodies.

### Constraint Defaults and Templates

- `<generic-constraint-default>`
- `<conetwist-constraint-default>`
- `<stiffspring-constraint-default>`

These work like bone templates. A blank name gives you an unnamed fallback template. `extends="OtherTemplate"` copies another template first.

Actual constraints can use `template="OtherTemplate"` to inherit from a default before applying their own settings.

The parser also accepts `<constraint-group>` as a wrapper. It has no special physics behavior of its own; it only groups constraint nodes.

### Generic Constraint

`<generic-constraint>` is the most flexible constraint type and the one to use for most cloth strips, hair chains, and simple articulated armor pieces.

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

Frame modes:

- `<frameInB>`: default. The explicit `frame` is interpreted in body B space.
- `<frameInA>`: the explicit `frame` is interpreted in body A space.
- `<frameInLerp>`: interpolates the two node transforms using `translationLerp` and `rotationLerp`, both clamped to `0.0` to `1.0`.
- `<AWithXPointToB />`, `<AWithYPointToB />`, `<AWithZPointToB />`: orient A's X/Y/Z axis toward B.
- Lowercase dashed forms such as `<a-with-x-point-to-b />` are also accepted.

Generic fields actually used by the runtime:

- `useLinearReferenceFrameA`
- `linearLowerLimit`, `linearUpperLimit`
- `angularLowerLimit`, `angularUpperLimit`
- `linearStiffness`, `angularStiffness`
- `linearDamping`, `angularDamping`
- `linearNonHookeanDamping`, `angularNonHookeanDamping`
- `linearNonHookeanStiffness`, `angularNonHookeanStiffness`
- `linearEquilibrium`, `angularEquilibrium`
- `linearBounce`, `angularBounce`
- `linearTargetVelocity`, `angularTargetVelocity`
- `linearMaxMotorForce`, `angularMaxMotorForce`
- `enableLinearSprings`, `enableAngularSprings`
- `linearStiffnessLimited`, `angularStiffnessLimited`
- `springDampingLimited`
- `linearMotors`, `angularMotors`
- `linearServoMotors`, `angularServoMotors`
- `motorERP`, `motorCFM`, `stopERP`, `stopCFM`

How they behave:

- Lower/upper limits use Bullet's standard convention: `upper < lower` means free, `upper == lower` means locked, and `upper > lower` means a limited range.
- `useLinearReferenceFrameA` swaps the constraint body order and reverses the frame-dependent values so the linear frame behaves like body A's reference frame.
- `linearStiffness` and `angularStiffness` enable spring behavior on the corresponding axis when the value is positive and the spring is enabled.
- `linearDamping` and `angularDamping` damp the spring motion.
- The non-Hookean fields add higher-order damping/stiffness terms for a harder or softer response curve near motion extremes.
- `linearEquilibrium` and `angularEquilibrium` define the resting point. They also become servo targets.
- `linearTargetVelocity` and `angularTargetVelocity` drive motor motion.
- `linearMaxMotorForce` and `angularMaxMotorForce` cap motor strength.
- `motorERP` and `motorCFM` tune motor correction and softness.
- `stopERP` and `stopCFM` tune limit correction and softness.
- If an axis is fully locked, the runtime forces its stop ERP to `1.0`.

When to use it:

- Use it for chains, straps, tails, light cloth strips, and generic articulated links.
- Use tight symmetric limits when you want a bone to stay near a pivot.
- Use a free axis with spring stiffness when you want pendulum-like motion.
- Use motors only when the XML needs to drive motion rather than react to it.

### Cone Twist Constraint

`<conetwist-constraint>` is the right choice for shoulder-like, elbow-like, or tail-joint motion where one axis can twist and two axes can swing.

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

Supported cone-twist fields:

- `swingSpan1`
- `swingSpan2`
- `twistSpan`
- `coneLimit`
- `planeLimit`
- `twistLimit`
- `limitX`
- `limitY`
- `limitZ`
- `limitSoftness`
- `biasFactor`
- `relaxationFactor`

How they behave:

- `swingSpan1` and `swingSpan2` define the swing ellipse around the main axis.
- `twistSpan` defines the twist range.
- `limitSoftness` softens the boundary before the hard limit is hit.
- `biasFactor` controls how aggressively Bullet pushes the joint back toward the allowed range.
- `relaxationFactor` controls how quickly the correction settles.

Important implementation note:

- The runtime only uses the frame setup and the cone-twist limit fields.
- Generic spring, motor, ERP/CFM, and equilibrium fields are parsed for this node type, but they do not affect the resulting cone-twist constraint.

When to use it:

- Use it for biological joints, tail bases, shoulder-like pivots, and other angular joints that should stay inside a cone.
- Use smaller spans for stiffer motion.
- Use `limitSoftness < 1.0` when a hard edge feels too abrupt.

### Stiff Spring Constraint

`<stiffspring-constraint>` is a custom distance spring built on this port's runtime. It is not the same as Bullet's generic 6DOF spring.

```xml
<stiffspring-constraint name="Spring" bodyA="Root" bodyB="Tip">
  <minDistanceFactor>0.8</minDistanceFactor>
  <maxDistanceFactor>1.2</maxDistanceFactor>
  <stiffness>5.0</stiffness>
  <damping>0.25</damping>
  <equilibrium>0.5</equilibrium>
</stiffspring-constraint>
```

Supported fields:

- `minDistanceFactor`
- `maxDistanceFactor`
- `stiffness`
- `damping`
- `equilibrium`

How they behave:

- `minDistanceFactor` and `maxDistanceFactor` are applied to the initial anchor distance between the two bodies.
- If the two values are equal, the link behaves like a fixed-length tether.
- If they differ, the joint can compress and extend inside that band.
- `stiffness` controls how strongly the spring tries to return to equilibrium when the distance is inside the band.
- `damping` reduces oscillation.
- `equilibrium` is the resting point inside the allowed band, clamped to `0.0` to `1.0`.

Important implementation note:

- The custom stiff-spring constraint uses only these distance settings.
- Generic frame, limit, motor, and cone-twist fields are parsed for this node type because the reader is shared, but they do not change the stiff-spring behavior.

When to use it:

- Use it for simple tethers, straps, hanging pieces, and rope-like motion where angular control is unnecessary.
- Use it when you want a distance band rather than a rotational envelope.
- Use it for stable spring chains where a full 6DOF joint would be too much.

## defaultBBPs.xml

`defaultBBPs.xml` uses a `<default-bbps>` root.

```xml
<default-bbps>
  <map shape="ArmorTriShapeName" file="my-armor.xml" />
</default-bbps>
```

`<map>` selects an XML file when a geometry with the matching shape name is found.

`<remap>` aliases real geometry names to one XML descriptor name:

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

Behavior:

- Every `<requires>` shape must exist.
- At least one `<source>` shape must exist.
- Higher-priority sources are tested first.
- The remap lets multiple real geometry names share one descriptor.

Use `defaultBBPs.xml` when the same XML should cover several geometry names, or when FaceGen/source geometry needs to be normalized before XML selection.

## Runtime Behavior

- XML summaries are cached and reloaded when the file timestamp changes.
- Bodies are matched from skin bones on matched geometry.
- Suspicious skin instances, missing CPU vertex/index buffers, invalid triangle indices, unresolved bones, duplicate constraints, and self constraints are skipped.
- Armor builds apply the actor Havok reference pose during skeleton merge and body setup when it is available, then restore the actor pose before final commit.
- After a successful armor build, plugin-owned cloned armor nodes are reset to their stored merge local pose and reread from NiNode transforms. This is runtime behavior, not XML behavior, but it affects what you see after edits.
- Bone transforms are reset and reread for several frames after rebuilds to avoid stale-pose explosions.
- Loading screens suspend physics and restore tracked armor records after loading finishes.
- NPCs outside the active actor range are soft-suspended. Fresh out-of-range armor builds are reset to their stored reference merge pose before Bullet is removed from them.
- LooksMenu suspends active prototype states and reloads armor records after customization closes.
- `disable1stPersonViewPhysics` skips first-person player physics when enabled.
- `disableSMPHairWhenWigEquipped` disables hair-domain mesh bodies while wig slots are occupied.
- Wind only affects bones when global wind is enabled and each bone's `<wind-factor>` is above zero.

## Authoring Workflow

1. Find the exact `BSTriShape` geometry names and skinned bone names in the NIF.
2. Decide which bodies are anchors and set those bones to `<mass>0</mass>`.
3. Add dynamic bones for the moving chain, cloth strip, tail, or hair section.
4. Add one or more mesh descriptors for the geometry.
5. Add the required constraints.
6. Hook the XML up through direct `NiStringExtraData` or `defaultBBPs.xml`.
7. Start with small angular limits and small masses, then open things up only after the build is stable.

Practical defaults:

- For armor chains, use real actor skeleton bones like `Pelvis` only when they are true trusted skeleton nodes, not merely because a name lookup found them under `Root`.
- Armor-specific bones should stay in XML and be cloned as plugin-owned prefixed nodes.
- For hair, prefer direct `NiStringExtraData`, low mass, stronger damping, and `<per-vertex-shape>` before trying triangle collision.
- For cloth, begin with a kinematic root, then add one dynamic segment at a time.

## Troubleshooting

- If nothing builds, verify the XML has a `<system>` root and that the selected path resolves.
- If a template or shape reference fails, check that the referenced name was declared earlier in the file.
- If bodies build but constraints do not, check `bodyA` and `bodyB`.
- If mesh collision is missing, check the `BSTriShape` name, skin data, and CPU vertex/index availability.
- If per-triangle collision is skipped, the CPU index buffer may be unreadable.
- If a body appears offset, adjust `<centerOfMassTransform>`.
- If a bone binds to the wrong place, verify that the bone exists in XML and that the geometry is actually skinned to it.
- If a collider is too loose or too tight, tune `margin`, `penetration`, or the relevant bone shape.
- If motion explodes, reduce angular limits, reduce mass, increase damping, or use a kinematic root.
- If an armor node that only happens to appear under actor `Root` is chosen by name, assume it is not a trusted actor skeleton bone unless the runtime says otherwise.
