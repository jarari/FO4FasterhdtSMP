# FO4 Faster HDT-SMP 물리 XML 가이드

이 문서는 현재 Fallout 4 포트가 실제로 구현한 XML 형식과 런타임 동작을 설명합니다. 형식은 FSMP/HDT-SMP XML과 가깝지만, 여기 적힌 요소만 지원 범위로 보는 것이 안전합니다.

## 파일 위치

물리 XML은 다음 후보 경로에서 해석됩니다.

- XML에 적힌 경로 그대로.
- 상대 경로일 때 `Data/<path>`.
- `Data/F4SE/Plugins/FO4FasterHdtSMP/<path>`.
- 마이그레이션 호환용 `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/<path>`.

경로는 `.xml`로 끝나야 합니다.

## XML 선택 방식

아머 XML은 다음 방식으로 선택할 수 있습니다.

- 아머 오브젝트의 `HDT Skinned Mesh Physics Object` 이름 `NiStringExtraData`.
- 가까운 attach 조상, source object, source root, destination root에 붙은 같은 extra data.
- `defaultBBPs.xml`의 shape-to-file 매핑.
- 테스트 fallback인 `configs.xml`의 `<prototypePhysicsXml>`.

헤드와 헤어 XML은 head 초기화와 headpart 준비 이후 actor face/head 서브트리에서 찾습니다. 직접 붙은 `NiStringExtraData`가 우선입니다. `defaultBBPs.xml`도 원본 FaceGen/source 지오메트리에 매칭될 수 있습니다. 헤어 후보는 actor hair headpart의 모델/에디터 키로 분류되고, 나머지 face 후보는 head 물리로 취급됩니다.

## 기본 구조

모든 물리 XML은 `<system>` 루트를 가져야 합니다.

```xml
<?xml version="1.0" encoding="utf-8"?>
<system>
  ...
</system>
```

XML 이름은 FO4 NIF 이름과 맞아야 합니다. `<bone>` 이름은 스킨된 `NiNode` 이름과 맞습니다. `<per-vertex-shape>`와 `<per-triangle-shape>` 이름은 `BSTriShape` 지오메트리 이름 또는 `defaultBBPs.xml` remap alias와 맞습니다.

## 시작용 XML

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

`ArmorRoot`, `ArmorCloth01`, `ArmorCloth02`, `ArmorTriShapeName`은 실제 NIF 이름으로 바꿔야 합니다.

## 본

`<bone-default>`는 뒤에 오는 `<bone>`의 기본값을 제공합니다. 이름이 있는 `<bone-default name="...">`는 `<bone template="...">`로 참조할 수 있습니다. 이름 있는 기본값은 `extends="OtherTemplate"`로 다른 기본값을 상속할 수 있습니다.

지원되는 본 필드:

- `<mass>`: `0` 이하이면 kinematic anchor, 양수이면 시뮬레이션 바디입니다.
- `<linearDamping>`과 `<angularDamping>`.
- `<friction>`, `<rollingFriction>`, `<restitution>`.
- `<gravity-factor>`: `0.0`부터 `1.0`까지로 제한됩니다.
- `<wind-factor>`: `0.0`이면 해당 본에 바람 힘을 적용하지 않습니다.
- `<margin-multiplier>`: rigid body 충돌 margin을 배율로 조정합니다.
- `<collision-filter>`: 파싱해서 저장합니다.
- `<localInertia>` 또는 `<inertia x="..." y="..." z="..." />`.
- `<centerOfMassTransform>`과 그 안의 `<origin>`, `<basis>`, `<basis-axis-angle>`.
- `<can-collide-with-bone>`과 `<no-collide-with-bone>`.
- `<shape>`.

예시:

```xml
<bone name="Hair01">
  <mass>0.08</mass>
  <centerOfMassTransform>
    <origin x="0" y="0" z="-1.5" />
  </centerOfMassTransform>
  <no-collide-with-bone>HairRoot</no-collide-with-bone>
</bone>
```

## 충돌 Shape

지원되는 `<shape>` 타입:

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

이름 있는 shape은 재사용할 수 있습니다.

```xml
<shape name="SmallSphere" type="sphere">
  <radius>1.5</radius>
</shape>

<bone name="Hair02">
  <shape type="ref" name="SmallSphere" />
</bone>
```

복합 shape도 파싱됩니다.

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

## 메시 충돌

시뮬레이션 본이 실제 스킨된 아머, 헤드, 헤어 표면과 충돌해야 할 때 메시 충돌을 사용합니다.

`<per-vertex-shape>`는 디코딩된 정점으로 콜라이더를 만들며, 가장 먼저 시도하기 좋은 방식입니다.

`<per-triangle-shape>`는 디코딩된 index buffer로 triangle collider를 만듭니다. 더 정밀하지만 CPU index 데이터가 없거나 유효하지 않으면 건너뜁니다.

지원되는 메시 필드:

- `<margin>`.
- `<penetration>`. 호환성을 위해 오타 형태인 `<prenetration>`도 허용합니다.
- `<shared>`: `public`, `internal`, `external`, `private`.
- `<tag>`, `<can-collide-with-tag>`, `<no-collide-with-tag>`.
- `<can-collide-with-bone>`, `<no-collide-with-bone>`.
- `<weight-threshold bone="...">`.
- `<disable-tag>`와 `<disable-priority>`.

`disable-tag`가 있는 mesh body는 다른 활성 mesh body가 같은 tag를 가지고 있을 때 비활성화됩니다. 같은 disable tag를 가진 mesh body가 여러 개이면 `<disable-priority>`가 가장 높은 하나만 유지됩니다.

예시:

```xml
<per-vertex-shape name="HairTriShape">
  <margin>0.35</margin>
  <shared>public</shared>
  <tag>HAIR</tag>
  <weight-threshold bone="Hair03">0.03</weight-threshold>
</per-vertex-shape>
```

## 제약

제약은 파싱된 본 바디 두 개를 연결합니다. `bodyA`와 `bodyB`는 XML 본 이름으로 해석되어야 합니다.

### Generic Constraint

Generic constraint는 아머 천 조각과 헤어 체인의 기본 선택지입니다.

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

지원되는 frame 모드:

- `<frameInA>...</frameInA>`
- `<frameInB>...</frameInB>`
- `<frameInLerp>...</frameInLerp>`
- `<AWithXPointToB />`, `<AWithYPointToB />`, `<AWithZPointToB />`
- `<a-with-x-point-to-b />` 같은 소문자 dash 표기

Generic field는 linear/angular limit, stiffness, damping, equilibrium, bounce, motor, servo motor, target velocity, max motor force, ERP/CFM을 포함합니다. Non-Hookean field는 파싱하고 경고하지만, 현재 FO4 Bullet 경로에서는 적용하지 않습니다.

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

`<coneLimit>`, `<planeLimit>`, `<twistLimit>`, `<limitX>`, `<limitY>`, `<limitZ>` 같은 alias도 파싱됩니다.

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

Constraint 기본값은 `<generic-constraint-default>`, `<conetwist-constraint-default>`, `<stiffspring-constraint-default>`로 만들 수 있습니다. Constraint 기본값도 `extends="OtherTemplate"`를 지원합니다.

## defaultBBPs.xml

`defaultBBPs.xml`은 `<default-bbps>` 루트를 사용합니다.

```xml
<default-bbps>
  <map shape="ArmorTriShapeName" file="my-armor.xml" />
</default-bbps>
```

`<map>`은 일치하는 shape 이름의 지오메트리가 발견되면 XML 파일을 선택합니다.

`<remap>`은 실제 지오메트리 이름을 XML descriptor 이름으로 alias할 수 있습니다.

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

모든 `<requires>` shape이 있고 `<source>` 중 하나가 있으면 target shape이 XML mesh descriptor와 매칭될 수 있습니다. priority가 높은 source가 먼저 고려됩니다.

## 런타임 참고

- XML summary는 캐시되며 파일 timestamp가 바뀌면 다시 로드됩니다.
- Body는 매칭된 지오메트리의 skin bone에서 찾습니다. XML에 mesh descriptor가 있으면 매칭된 지오메트리의 모든 skin bone이 후보가 될 수 있습니다.
- 의심스러운 skin instance, CPU vertex/index buffer 누락, 잘못된 triangle index, 해석되지 않은 본, 중복/self/kinematic-only constraint는 건너뜁니다.
- Rebuild 직후에는 오래된 pose 폭주를 줄이기 위해 몇 프레임 동안 본 transform을 reset/read합니다.
- Loading screen 중에는 물리를 중지하고 resume 시 현재 node pose로 body를 리셋합니다.
- LooksMenu 중에는 활성 prototype state를 중지하고 닫힌 뒤 armor record를 다시 로드합니다.
- `<disable1stPersonViewPhysics>`가 true이면 1인칭 player physics를 건너뛰거나 중지합니다.
- `<disableSMPHairWhenWigEquipped>`가 true이면 hair biped slot에 wig가 있을 때 hair-domain mesh body를 비활성화합니다.
- Wind는 전역 wind가 켜져 있고 각 본의 `<wind-factor>`가 0보다 클 때만 적용됩니다. Weather wind는 유효한 exterior sky/weather 상태가 필요하며 actor 머리 주변에서 LOS obstruction을 계산합니다.

## 제작 흐름

1. NIF에서 정확한 `BSTriShape` 지오메트리 이름과 skin bone 이름을 찾습니다.
2. `<mass>0</mass>`인 kinematic root bone을 하나 만듭니다.
3. 움직일 chain 또는 천 영역에 dynamic bone을 만듭니다.
4. 지오메트리에 대한 mesh collision descriptor를 하나 추가합니다.
5. 본들을 generic constraint로 연결합니다.
6. 직접 `NiStringExtraData`를 추가하거나 `defaultBBPs.xml` map을 추가합니다.
7. 처음에는 작은 angular limit로 시작하고, 안정된 뒤에만 범위를 늘립니다.

헤어는 hair/headpart 서브트리에 직접 `NiStringExtraData`를 붙이는 방식을 우선하고, 낮은 mass와 높은 damping을 사용하며, triangle collision보다 `<per-vertex-shape>`를 먼저 시도하는 것이 좋습니다.

## 문제 해결

- 아무것도 생성되지 않으면 XML에 `<system>` 루트가 있는지, 선택된 XML 경로가 실제로 해석되는지 확인합니다.
- defaultBBP 매칭이 실패하면 실제 geometry 이름과 remap `<requires>` 항목을 확인합니다.
- Body는 생성되지만 constraint가 생성되지 않으면 `bodyA`와 `bodyB` 이름을 확인합니다.
- Mesh collision이 없으면 `BSTriShape` 이름, skin data, CPU vertex/index 사용 가능 여부를 확인합니다.
- Per-triangle collision이 건너뛰어지면 CPU index buffer를 읽을 수 없는 상태일 수 있습니다.
- Body 위치가 어긋나면 `<centerOfMassTransform>`을 조정합니다.
- 움직임이 폭주하면 angular limit를 줄이고, mass를 낮추고, damping을 높이거나 kinematic root를 사용합니다.
