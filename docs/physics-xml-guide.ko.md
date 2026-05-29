# FO4 Faster HDT-SMP 물리 XML 가이드

이 문서는 현재 Fallout 4 포트에서 실제로 파싱하고 런타임에서 사용하는 물리 XML 형식을 설명합니다. 형식은 FSMP/HDT-SMP XML과 비슷하지만, 아머와 헤어 제작에서는 이 문서에 있는 요소를 우선 지원 범위로 보면 됩니다.

## 파일 위치

물리 XML은 플러그인 설정 경로에서 검색됩니다.

- `Data/F4SE/Plugins/FO4FasterHdtSMP/`
- 레거시 호환용 `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/`

아머 XML은 다음 방법으로 선택할 수 있습니다.

- 아머 NIF 오브젝트나 가까운 부모 노드에 `HDT Skinned Mesh Physics Object` 이름의 `NiStringExtraData`를 추가합니다. 문자열 데이터에는 XML 경로를 넣습니다.
- `defaultBBPs.xml`에 쉐이프 이름과 XML 파일 매핑을 추가합니다.
- 테스트 중에는 `configs.xml`의 `<prototypePhysicsXml>`에 fallback XML을 지정할 수 있습니다.

헤어와 헤드파트 XML은 현재 헤드/헤어 서브트리에 직접 붙은 `HDT Skinned Mesh Physics Object` `NiStringExtraData`가 필요합니다. `defaultBBPs.xml`은 아머 중심 경로이므로 헤어에는 권장하지 않습니다.

## 기본 구조

모든 물리 XML은 `<system>` 루트를 가져야 합니다.

```xml
<?xml version="1.0" encoding="utf-8"?>
<system>
  ...
</system>
```

XML의 이름은 NIF 안의 실제 FO4 스킨 본 이름과 지오메트리 이름과 맞아야 합니다. `<bone>` 이름은 스킨 본 노드와 맞아야 하고, `<per-vertex-shape>` 또는 `<per-triangle-shape>` 이름은 `BSTriShape` 지오메트리 이름과 맞아야 합니다.

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
    <linearStiffness x="0" y="0" z="0" />
    <angularStiffness x="8" y="8" z="8" />
    <linearDamping x="0" y="0" z="0" />
    <angularDamping x="0.35" y="0.35" z="0.35" />
  </generic-constraint>
</system>
```

`ArmorRoot`, `ArmorCloth01`, `ArmorCloth02`, `ArmorTriShapeName`은 실제 NIF 이름으로 바꿔야 합니다.

## 본

`<bone-default>`는 뒤에 오는 `<bone>` 항목의 기본값을 제공합니다. `<bone-default name="...">`로 이름 있는 템플릿을 만들고 `<bone template="...">`에서 참조할 수 있습니다.

지원되는 본 필드:

- `<mass>`: `0` 이하이면 키네마틱 앵커 바디가 됩니다. 양수이면 시뮬레이션 바디가 됩니다.
- `<linearDamping>`와 `<angularDamping>`: 이동 속도와 회전을 감쇠합니다.
- `<friction>`, `<rollingFriction>`, `<restitution>`: Bullet 리지드 바디 재질 값입니다.
- `<gravity-factor>`: `0.0`부터 `1.0`까지로 제한됩니다.
- `<wind-factor>`: `0.0`이면 해당 본은 바람 영향을 받지 않습니다.
- `<margin-multiplier>`: 스킨 메시 충돌에서 쓰는 충돌 마진 배율입니다.
- `<collision-filter>`: 파싱은 되지만 현재는 필터링용 예약 값에 가깝습니다.
- `<localInertia>` 또는 `<inertia x="..." y="..." z="..." />`: 선택적 명시 관성 값입니다.
- `<centerOfMassTransform>`: 노드 기준으로 바디의 중심과 회전을 이동합니다.
- `<can-collide-with-bone>`과 `<no-collide-with-bone>`: 본 충돌 허용/차단 목록입니다.
- `<shape>`: 리지드 바디 충돌 쉐이프입니다.

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

## 충돌 쉐이프

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

이름 있는 쉐이프는 재사용할 수 있습니다.

```xml
<shape name="SmallSphere" type="sphere">
  <radius>1.5</radius>
</shape>

<bone name="Hair02">
  <shape type="ref" name="SmallSphere" />
</bone>
```

복합 쉐이프도 파싱됩니다.

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

시뮬레이션 본이 실제 스킨 아머나 헤어 표면과 충돌해야 할 때 메시 충돌을 사용합니다.

`<per-vertex-shape>`는 정점 기반 콜라이더를 만듭니다. 비용이 낮아서 먼저 시도하기 좋습니다.

`<per-triangle-shape>`는 메시 인덱스 버퍼의 삼각형으로 콜라이더를 만듭니다. 더 자세하지만 CPU에서 읽을 수 있는 인덱스 데이터가 필요합니다.

지원되는 메시 필드:

- `<margin>`: 충돌 마진입니다.
- `<penetration>`: 관통 허용 값입니다. 호환성을 위해 오타인 `<prenetration>`도 허용됩니다.
- `<shared>`: `public`, `internal`, `external`, `private`.
- `<tag>`, `<can-collide-with-tag>`, `<no-collide-with-tag>`.
- `<can-collide-with-bone>`, `<no-collide-with-bone>`.
- `<weight-threshold bone="...">`: 특정 본의 약한 정점 영향도를 무시합니다.
- `<disable-tag>`와 `<disable-priority>`.

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

제약은 XML 바디 두 개를 연결합니다. `bodyA`와 `bodyB`는 파싱된 본 이름으로 해석되어야 합니다.

### Generic Constraint

아머 천 조각이나 헤어 체인에는 기본적으로 이 제약을 권장합니다.

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

지원되는 프레임 모드:

- `<frameInA>...</frameInA>`
- `<frameInB>...</frameInB>`
- `<frameInLerp>...</frameInLerp>`
- `<AWithXPointToB />`, `<AWithYPointToB />`, `<AWithZPointToB />`
- `<a-with-x-point-to-b />` 같은 소문자 대시 표기

Generic 제약은 선형/각도 제한, stiffness, damping, equilibrium, bounce, motor, servo motor, target velocity, max motor force, ERP/CFM 필드를 지원합니다.

Non-Hookean 필드는 파싱되지만 현재 FO4 Bullet 제약 경로에서는 적용되지 않습니다.

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

`<coneLimit>`, `<planeLimit>`, `<twistLimit>`, `<limitX>`, `<limitY>`, `<limitZ>` 같은 별칭도 파싱됩니다.

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

## 템플릿

템플릿을 쓰면 반복을 줄일 수 있습니다.

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

이름 있는 기본값은 `extends="OtherTemplate"`로 다른 템플릿을 상속할 수 있습니다.

## 아머 제작 흐름

1. 아머 NIF에서 정확한 `BSTriShape` 지오메트리 이름과 스킨 본 이름을 찾습니다.
2. `<mass>0</mass>`인 키네마틱 루트 본을 하나 만듭니다.
3. 움직일 체인이나 천 부분에 동적 본을 만듭니다.
4. 아머 지오메트리에 대한 메시 충돌 descriptor를 하나 추가합니다.
5. 본들을 generic constraint로 연결합니다.
6. 직접 `NiStringExtraData`를 넣거나 `defaultBBPs.xml` 매핑을 추가합니다.
7. 처음에는 작은 각도 제한으로 시작하고, 안정된 뒤에만 범위를 늘립니다.

## 헤어 제작 흐름

1. 헤어 헤드파트 모델과 스킨된 헤어 지오메트리를 찾습니다.
2. 헤어 서브트리에 `HDT Skinned Mesh Physics Object` 이름의 `NiStringExtraData`를 넣습니다.
3. 두피 쪽에는 키네마틱 루트를 두고, 아래 체인에는 동적 본을 둡니다.
4. 아머보다 낮은 mass와 높은 angular damping을 우선 사용합니다.
5. 먼저 `<per-vertex-shape>`를 사용하고, 꼭 필요할 때만 `<per-triangle-shape>`를 사용합니다.

## 문제 해결

- 아무것도 생성되지 않으면 XML에 `<system>` 루트가 있는지, XML 경로가 실제로 해석되는지 확인합니다.
- 바디는 생성되지만 제약이 생성되지 않으면 `bodyA`와 `bodyB` 이름을 확인합니다.
- 메시 충돌이 없으면 메시 이름이 실제 `BSTriShape` 이름과 맞는지, 메시가 스킨 데이터를 가지고 있는지 확인합니다.
- per-triangle 충돌이 건너뛰어지면 CPU 인덱스 버퍼를 읽을 수 없는 상태일 수 있습니다.
- 바디 위치가 어긋나면 `<centerOfMassTransform>`을 조정합니다.
- 움직임이 폭주하면 각도 제한을 줄이고, mass를 낮추고, damping을 높이거나 키네마틱 루트를 사용합니다.
