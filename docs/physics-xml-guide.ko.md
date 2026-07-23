# FO4 Faster HDT-SMP Physics XML 가이드

이 문서는 이 Fallout 4 포트에서 실제로 구현된 XML 형식과, 런타임에서 중요한 Bullet 동작을 기준으로 정리한 가이드입니다. 이전 FSMP/HDT-SMP 생태계와 비슷해 보여도 지원 범위는 더 좁으므로, 여기서 설명하지 않은 요소는 코드가 명시적으로 처리하지 않는 한 지원되지 않는다고 보는 편이 맞습니다.

## 파일 위치

XML 파일은 다음 위치에서 해석됩니다.

- 작성한 경로를 그대로 사용
- 상대 경로일 때 `Data/<path>`
- `Data/F4SE/Plugins/FO4FasterHdtSMP/<path>`
- 레거시 마이그레이션용 `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/<path>`

경로는 반드시 `.xml`로 끝나야 합니다.

## XML 선택 방식

Armor XML은 다음 우선순위로 선택됩니다.

- armor 오브젝트에 붙은 `HDT Skinned Mesh Physics Object` 이름의 `NiStringExtraData`
- 같은 extra data가 붙은 가까운 attach ancestor, source object, source root, destination root
- `defaultBBPs.xml`의 shape-to-file 매칭
- `configs.xml`의 `<prototypePhysicsXml>`, 이것은 prototype/test fallback 용도입니다

Head와 hair XML은 head 초기화와 headpart 준비가 끝난 뒤 actor의 face/head subtree에서 찾아냅니다. 직접 붙은 `NiStringExtraData`가 가장 우선입니다. `defaultBBPs.xml`은 원본 FaceGen/source geometry에도 매칭할 수 있습니다. Hair 후보는 actor hair headpart의 model/editor key로 분류하고, 나머지 face 후보는 head physics로 취급합니다.

## XML 규칙

모든 physics XML 파일은 `<system>` root가 필요합니다.

```xml
<?xml version="1.0" encoding="utf-8"?>
<system>
  ...
</system>
```

중요한 파싱 규칙:

- element 이름과 attribute 이름은 대소문자를 구분합니다.
- 숫자 텍스트는 앞뒤 공백이 제거됩니다.
- float는 `.` 또는 하나의 `,`를 소수점으로 받아들입니다.
- bool은 `1`, `0`, `true`, `false`를 허용합니다.
- template과 named-shape 조회는 선언 순서에 영향을 받습니다. 참조하기 전에 먼저 선언해야 합니다.
- 지원되는 블록이 아니면 알 수 없는 element는 무시됩니다.

## 이름과 상속

파서는 bone과 constraint에 대해 이름이 있는 template을 지원합니다.

- `<bone-default>`는 상속용 기본값을 제공합니다.
- 이름이 없는 `<bone-default>`는 bone 전역 fallback이 됩니다.
- `<bone-default name="X" extends="Y">`는 먼저 `Y`를 복사한 뒤 local 설정으로 덮어씁니다.
- `<bone name="X" template="Y">`는 먼저 template `Y`를 복사한 뒤 local bone 설정을 적용합니다.
- 같은 방식이 `<generic-constraint-default>`, `<conetwist-constraint-default>`, `<stiffspring-constraint-default>`에도 적용됩니다.

template 조회는 파일을 읽는 동안 순차적으로 이뤄집니다. 참조 시점에 이미 존재해야 합니다.

## Bone

Bone descriptor는 skinned bone에 대한 rigid body를 정의합니다.

### Bone 필드

- `<mass>`: `0` 이하이면 kinematic anchor가 됩니다. 양수면 dynamic rigid body가 됩니다. 음수는 build 시 `0`처럼 취급됩니다.
- `<linearDamping>`, `<angularDamping>`: rigid body의 Bullet damping 값입니다.
- `<friction>`: rigid body의 Coulomb 마찰 계수입니다. 값이 높을수록 다른 물체와 접촉할 때 미끄러짐을 더 강하게 막습니다. `0`이면 매우 미끄럽습니다. 이 값은 자유 운동 중이 아니라 접촉 중에만 의미가 있습니다.
- `<rollingFriction>`: 접점에서의 구름/굴림 저항입니다. 구, 캡슐, 원통처럼 구르기 쉬운 형태나 둥근 파트에서 특히 의미가 큽니다. 평평한 cloth 조각에는 보통 덜 유용합니다.
- `<restitution>`: 튕김, 즉 반발입니다. `0`이면 튕기지 않고, 값이 높을수록 충돌 후 더 많은 에너지를 되돌립니다. 런타임은 `0` 이상으로만 clamp 하지만, 실무에서는 보통 `0`~`1` 범위 안에서 두는 편이 안정적입니다.
- `<gravity-factor>`: `0.0`부터 `1.0`까지 clamp 됩니다. `0`이면 중력이 사라지고, `1`이면 월드 중력을 그대로 받습니다.
- `<wind-factor>`: 음수는 `0`으로 clamp 됩니다. 전역 wind가 켜져 있고 런타임에 유효한 wind/weather 상태가 있을 때만 의미가 있습니다.
- `<margin-multiplier>`: skinned mesh collision에서 사용하는 bone별 collision margin을 스케일합니다. Bullet rigid-body margin과는 다릅니다.
- `<collision-filter>`: 파싱되고 저장되지만 현재 build path에서는 적용되지 않습니다.
- `<localInertia>` 또는 `<inertia x="..." y="..." z="..." />`: 명시적 local inertia입니다. 값이 있으면 음수 성분은 `0`으로 clamp 됩니다. mass가 양수인데 생략하면 Bullet이 shape에서 inertia를 계산합니다.
- `<centerOfMassTransform>`: node와 rigid body 사이의 local offset/orientation입니다. node origin이 원하는 physics pivot이 아닐 때 사용합니다.
- `<can-collide-with-bone>`, `<no-collide-with-bone>`: bone 단위 collision filter입니다. allow-list가 하나라도 있으면 그 목록만 collide 합니다. 아니면 deny-list가 적용됩니다.
- `<shape>`: bone의 collision shape입니다. 생략하면 empty shape가 됩니다.

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

### 언제 무엇을 쓸지

- `mass = 0`: anchor bone, root bone, 애니메이션을 그대로 따라가야 하는 body
- 작은 양수 mass: cloth strip, tail, strap, hair chain
- 높은 damping: 빠르게 가라앉아야 하거나 흔들림을 줄여야 할 때
- `friction`: 파트가 너무 쉽게 미끄러지지 않게 하고 싶을 때, 예를 들어 tail root, 걸리게 해야 하는 strap, 표면을 타고 미끄러지면 안 되는 둥근 collider
- `rollingFriction`: 형태가 굴러가려는 성질을 줄이고 싶을 때, 예를 들어 구형 장식, capsule, 막대형 파트
- `restitution`: 접촉 후 다시 튕겨 나오게 하고 싶을 때. cloth와 hair에는 보통 낮게 둡니다.
- `centerOfMassTransform`: physics pivot이 bone origin에서 벗어났을 때
- `collision-filter`: 현재 runtime이 소비하지 않으므로 특별한 이유가 없으면 건드리지 않는 편이 좋습니다

## Collision Shape

이름이 있는 shape는 `<shape type="ref" name="...">`로 재사용할 수 있습니다. 참조 대상은 이미 이전에 파싱된 shape여야 합니다.

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

Compound shape도 지원합니다.

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

shape 동작:

- `sphere`: radius는 최소 `0.01`로 clamp 됩니다.
- `box`: half extent 각각이 최소 `0.01`로 clamp 됩니다. `margin`은 Bullet box shape에 적용됩니다.
- `capsule`: radius와 height가 각각 최소 `0.01`로 clamp 됩니다. 이 구현에서는 별도 margin 필드를 받지 않습니다.
- `cylinder`: radius와 height가 각각 최소 `0.01`로 clamp 됩니다. `margin`이 적용됩니다.
- `hull`: 나열된 point로 convex hull을 만듭니다. point가 비어 있으면 invalid 입니다.
- `compound`: 각 child는 자신의 `<transform>`과 `<shape>`를 가질 수 있습니다. child가 하나도 없으면 invalid 입니다.
- `ref`: 이전에 정의된 named shape를 복사합니다.

### 언제 무엇을 쓸지

- `sphere`: 단순한 anchor, tail, tip
- `box`: 평평하거나 각진 파트, 예측 가능한 collider가 필요할 때
- `capsule`: 둥근 끝이 자연스러운 chain이나 limb
- `cylinder`: capsule보다 덜 둥글고 막대에 가까운 파트
- `hull`: 불규칙하지만 convex collider가 필요한 경우
- `compound`: 한 bone에 primitive collider 여러 개를 붙여야 할 때

## Mesh Collision

Mesh descriptor는 skinned armor, head, hair geometry에서 collider를 만듭니다.

지원되는 mesh 필드:

- `<margin>`: collision padding
- `<penetration>`: triangle collision에서 사용하는 추가 padding floor. 오타인 `<prenetration>`도 허용됩니다.
- `<shared>`: `public`, `internal`, `external`, `private`
- `<tag>`: 그룹화와 상호 배제를 위한 collision tag
- `<can-collide-with-tag>`, `<no-collide-with-tag>`: tag filter
- `<can-collide-with-bone>`, `<no-collide-with-bone>`: bone filter
- `<weight-threshold bone="...">`: 해당 bone이 collider에 참여하기 위한 최소 influence weight
- `<disable-tag>`, `<disable-priority>`: 상호 배타적인 mesh group 제어

예시:

```xml
<per-vertex-shape name="HairTriShape">
  <margin>0.35</margin>
  <shared>public</shared>
  <tag>HAIR</tag>
  <weight-threshold bone="Hair03">0.03</weight-threshold>
</per-vertex-shape>
```

### Per-Vertex Shape

`<per-vertex-shape>`는 decoded vertex에서 collision을 만듭니다. index data 의존도가 낮아서 가장 안전한 기본값입니다.

이럴 때 사용합니다:

- armor, cloth, hair에 대해 가장 덜 깨지는 첫 시도
- vertex weight는 괜찮지만 triangle이 복잡한 mesh
- triangle collision 비용 없이 skinned surface를 따라가고 싶을 때

### Per-Triangle Shape

`<per-triangle-shape>`는 decoded index buffer에서 collision을 만듭니다. 더 세밀하지만 더 무겁고 더 쉽게 실패합니다.

이럴 때 사용합니다:

- per-vertex보다 더 날카로운 contact가 필요할 때
- CPU index buffer가 안정적으로 읽히는 mesh
- 비용을 감수하고 디테일을 얻고 싶을 때

CPU index buffer가 없거나 invalid 하거나 읽을 수 없으면 triangle body는 건너뜁니다.

### Shared Scope

`<shared>`는 mesh body가 누구와 collide할 수 있는지를 정합니다.

- `public`: 호환되는 모든 body와 collide
- `internal`: 같은 actor 안의 body와만 collide
- `external`: 다른 actor의 body와만 collide
- `private`: 같은 actor이면서 같은 build group인 body와만 collide

`private`는 같은 build group 안의 서로 종속된 파트에 적합합니다. `internal`은 같은 actor 안의 관련 파트가 서로 상호작용해야 할 때, `external`은 다른 actor와만 반응해야 할 때, `public`은 일반적인 경우에 씁니다.

두 mesh body가 모두 kinematic이면 collision이 꺼집니다.

### Tag Filter

tag filter는 shared-scope 검사 뒤에 적용됩니다.

- `can-collide-with-tag`가 비어 있으면 `no-collide-with-tag`가 deny-list로 동작합니다.
- 하나 이상의 `can-collide-with-tag`가 있으면 allow-list가 되고 deny-list는 matching에 쓰이지 않습니다.

tag는 armor variant 분리, 관련 파트의 self-collision 방지, 대체 mesh body의 상호 활성화 방지 등에 유용합니다.

### Bone Filter

`can-collide-with-bone`, `no-collide-with-bone`는 XML 문자열 자체가 아니라 해석된 prototype bone과 비교합니다. bone name을 해석하지 못하면 그 항목은 무시되고 로그만 남습니다.

mesh가 skeleton의 일부와만 상호작용해야 하거나, 특정 collider group에서 특정 bone을 제외해야 할 때 사용합니다.

### Weight Threshold

`<weight-threshold bone="...">`는 해당 bone의 collider 참여를 잘라냅니다.

이럴 때 사용합니다:

- 원치 않는 작은 influence를 가진 vertex가 있을 때
- blended weight가 많아서 collider를 dominant bone에 집중시키고 싶을 때
- mesh weight를 직접 수정하지 않고 noisy edge를 줄이고 싶을 때

### Disable Group

`<disable-tag>`, `<disable-priority>`는 상호 배타적인 mesh group을 만듭니다.

동작:

- 같은 disable tag를 이미 광고하는 active mesh body가 있으면 그 group의 모든 body를 disable합니다.
- 아직 active가 아니라면 `disable-priority`가 가장 높은 body만 enabled 상태로 남고 나머지는 disabled 됩니다.

layered hair, optional armor attachment, 같은 geometry의 대체 버전처럼 동시에 절대 같이 돌면 안 되는 경우에 사용합니다.

런타임 설정에 따라 wig가 장착되면 hair domain mesh body가 자동으로 꺼질 수도 있습니다.

### Margin 과 Penetration

`<margin>`과 `<penetration>`은 mesh 타입에 따라 다르게 작동합니다.

- per-vertex collision은 `margin`을 padding factor로 사용합니다.
- per-triangle collision은 `margin`과 `penetration`을 함께 사용합니다. triangle bounds는 평균 margin으로 확장되고, `penetration`은 그 확장의 최소값 역할을 합니다.

실전 기준:

- collision이 너무 늦게 닿거나 얇은 형상을 통과하면 `margin`을 올리세요.
- triangle collision이 너무 타이트하거나 contact buffer가 약하면 `penetration`을 올리세요.

## Constraints

Constraint는 파싱된 두 bone body를 연결합니다. `bodyA`, `bodyB`는 실제로 body로 해석되는 XML bone name이어야 합니다.

### Constraint Defaults 와 Templates

- `<generic-constraint-default>`
- `<conetwist-constraint-default>`
- `<stiffspring-constraint-default>`

이것들은 bone template과 같은 방식으로 동작합니다. 이름이 비어 있으면 unnamed fallback template이 됩니다. `extends="OtherTemplate"`는 다른 template을 먼저 복사합니다.

실제 constraint는 `template="OtherTemplate"`로 default를 상속한 뒤 자기 설정을 덮어쓸 수 있습니다.

파서는 `<constraint-group>`도 지원합니다. 이건 물리 의미가 없는 wrapper일 뿐이고, constraint 노드를 묶는 용도입니다.

### Generic Constraint

`<generic-constraint>`는 가장 유연한 constraint 타입이며, 대부분의 cloth strip, hair chain, 단순 articulated armor에 쓰기 좋습니다.

```xml
<generic-constraint name="Hair02 To Hair01" bodyA="Hair02" bodyB="Hair01">
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

frame mode:

- `<frameInB>`: 기본값입니다. 명시한 `frame`은 body B space로 해석됩니다.
- `<frameInA>`: 명시한 `frame`은 body A space로 해석됩니다.
- `<frameInLerp>`: 두 node transform을 `translationLerp`, `rotationLerp`로 보간합니다. 명시적인 `<frameInLerp>`는 child를 읽기 전에 두 값을 `0.0`으로 초기화하며, hdtSMP64와 동일하게 clamp하지 않고 그대로 전달합니다.
- `<AWithXPointToB />`, `<AWithYPointToB />`, `<AWithZPointToB />`: A의 X/Y/Z 축이 B의 origin을 향하도록 맞춥니다.
- `<a-with-x-point-to-b />` 같은 소문자 dashed form도 허용됩니다.

실제로 runtime이 사용하는 generic 필드:

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

동작:

- lower/upper limit는 Bullet 표준 규칙을 따릅니다. `upper < lower`는 free, `upper == lower`는 locked, `upper > lower`는 limited range입니다.
- `useLinearReferenceFrameA`는 constraint body와 frame 순서를 바꿉니다. limit, equilibrium point, servo target, target velocity의 XML 부호는 hdtSMP64와 동일하게 그대로 유지합니다.
- `enableLinearSprings`, `enableAngularSprings`는 현재 stiffness와 damping 값이 0이어도 spring row를 직접 제어합니다.
- `linearStiffness`, `angularStiffness`는 해당 축의 spring strength를 설정합니다.
- `linearDamping`, `angularDamping`은 spring motion을 감쇠시킵니다.
- non-Hookean 필드는 motion 끝단 근처에서 더 강하거나 더 부드러운 반응 곡선을 만드는 higher-order 항입니다.
- `linearEquilibrium`, `angularEquilibrium`는 resting point를 정의합니다. servo target으로도 사용됩니다.
- `linearTargetVelocity`, `angularTargetVelocity`는 motor motion을 구동합니다.
- `linearMaxMotorForce`, `angularMaxMotorForce`는 motor strength를 제한합니다.
- `motorERP`, `motorCFM`은 motor correction과 softness를 조정합니다.
- `stopERP`, `stopCFM`은 limit correction과 softness를 조정합니다.
- 설정된 `stopERP`는 limited 축과 완전히 locked 된 축 모두에 변경 없이 사용됩니다.

언제 쓰는가:

- chain, strap, tail, light cloth strip, generic articulated link
- pivot 주변에 머물게 하고 싶을 때는 대칭적인 작은 limit
- pendulum처럼 흔들리게 하고 싶으면 free axis + spring stiffness
- 움직임을 반응시키는 대신 강제로 구동해야 할 때만 motor 사용

### Cone Twist Constraint

`<conetwist-constraint>`는 shoulder, elbow, tail joint처럼 한 축은 twist하고 두 축은 swing하는 모션에 적합합니다.

```xml
<conetwist-constraint name="Tip To Root" bodyA="Tip" bodyB="Root">
  <frameInLerp />
  <swingSpan1>0.4</swingSpan1>
  <swingSpan2>0.4</swingSpan2>
  <twistSpan>0.2</twistSpan>
  <limitSoftness>1.0</limitSoftness>
  <biasFactor>0.3</biasFactor>
  <relaxationFactor>1.0</relaxationFactor>
</conetwist-constraint>
```

지원 필드:

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

동작:

- `swingSpan1`, `swingSpan2`는 main axis 주변의 swing ellipse를 정의합니다.
- `twistSpan`은 twist 범위를 정의합니다.
- `limitSoftness`는 hard limit에 닿기 전에 경계를 부드럽게 만듭니다.
- `biasFactor`는 Bullet이 허용 범위로 되돌리는 강도를 정합니다.
- `relaxationFactor`는 보정이 얼마나 빨리 가라앉는지 정합니다.

중요한 구현상 주의:

- 이 runtime은 frame 설정과 cone-twist limit 필드만 실제로 사용합니다.
- generic spring, motor, ERP/CFM, equilibrium 필드는 이 타입에서 파싱은 되지만 결과 constraint에는 영향을 주지 않습니다.

언제 쓰는가:

- biological joint, tail base, shoulder-like pivot, cone 안에만 있어야 하는 관절
- 더 딱딱하게 만들고 싶으면 span을 줄입니다
- 경계가 너무 갑작스럽다면 `limitSoftness < 1.0`을 씁니다

### Stiff Spring Constraint

`<stiffspring-constraint>`는 이 포트에 포함된 custom distance spring입니다. Bullet의 generic 6DOF spring과는 다릅니다.

```xml
<stiffspring-constraint name="Spring" bodyA="Root" bodyB="Tip">
  <minDistanceFactor>0.8</minDistanceFactor>
  <maxDistanceFactor>1.2</maxDistanceFactor>
  <stiffness>5.0</stiffness>
  <damping>0.25</damping>
  <equilibrium>0.5</equilibrium>
</stiffspring-constraint>
```

지원 필드:

- `minDistanceFactor`
- `maxDistanceFactor`
- `stiffness`
- `damping`
- `equilibrium`

동작:

- `minDistanceFactor`, `maxDistanceFactor`는 두 body의 초기 anchor distance에 적용됩니다.
- 두 값이 같으면 fixed-length tether처럼 동작합니다.
- 값이 다르면 그 범위 안에서 압축/신장을 허용합니다.
- `stiffness`는 distance가 band 안에 있을 때 equilibrium으로 돌아가려는 힘을 정합니다.
- `damping`은 oscillation을 줄입니다.
- `equilibrium`은 허용 band 내부의 rest point이며 `0.0`~`1.0`으로 clamp 됩니다.

중요한 구현상 주의:

- 이 custom stiff-spring constraint는 distance 설정만 사용합니다.
- generic frame, limit, motor, cone-twist 필드는 같은 reader를 공유해서 파싱되지만 stiff-spring 동작 자체를 바꾸지는 않습니다.

언제 쓰는가:

- simple tether, strap, hanging piece, rope-like motion
- angular control보다 거리 band가 더 맞을 때
- full 6DOF joint는 과한데 안정적인 spring chain이 필요할 때

## defaultBBPs.xml

`defaultBBPs.xml`은 `<default-bbps>` root를 사용합니다.

```xml
<default-bbps>
  <map shape="ArmorTriShapeName" file="my-armor.xml" />
</default-bbps>
```

`<map>`은 matching shape name을 가진 geometry가 보이면 해당 XML 파일을 선택합니다.

`<remap>`은 실제 geometry 이름을 하나의 XML descriptor 이름으로 묶습니다.

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

동작:

- 모든 `<requires>` shape가 존재해야 합니다.
- 하나 이상의 `<source>` shape가 존재해야 합니다.
- priority가 높은 source가 먼저 검사됩니다.
- remap은 여러 실제 geometry 이름이 하나의 descriptor를 공유하도록 해줍니다.

같은 XML을 여러 geometry 이름에 적용하거나, FaceGen/source geometry 이름을 정규화해서 선택하려면 `defaultBBPs.xml`을 사용합니다.

## 런타임 동작

- XML summary는 캐시되고 파일 timestamp가 바뀌면 다시 로드됩니다.
- body는 매칭된 geometry의 skin bone에서 매칭됩니다.
- 의심스러운 skin instance, CPU vertex/index buffer 누락, 잘못된 triangle index, 해석되지 않은 bone, 중복 constraint, self constraint는 건너뜁니다.
- armor build는 skeleton merge와 body setup 동안 actor Havok reference pose를 적용할 수 있으면 적용하고, 최종 commit 전에 다시 복원합니다.
- armor build가 성공하면 plugin-owned cloned armor node는 저장된 merge local pose로 reset되고, NiNode transform에서 다시 읽힙니다. 이것은 XML보다 runtime 동작에 가깝지만, 수정 결과를 볼 때 중요합니다.
- rebuild 후 몇 frame 동안은 stale pose 때문에 폭주하지 않도록 bone transform을 다시 읽고 reset합니다.
- loading screen에서는 physics가 suspend되고, 로딩이 끝나면 추적 중인 armor record를 복원합니다.
- active actor range 밖의 NPC는 soft-suspend 됩니다. range 밖에서 새로 build된 armor는 Bullet에서 즉시 제거되기 전에 저장된 reference merge pose로 reset됩니다.
- LooksMenu가 닫히면 active prototype state가 suspend되고 armor record가 다시 로드됩니다.
- `disable1stPersonViewPhysics`가 켜져 있으면 1인칭 player physics를 건너뜁니다.
- `disableSMPHairWhenWigEquipped`가 켜져 있으면 wig slot이 차 있을 때 hair domain mesh body를 비활성화합니다.
- wind는 전역 wind가 활성화되어 있고 각 bone의 `<wind-factor>`가 0보다 클 때만 작동합니다.

## 작성 순서

1. NIF에서 정확한 `BSTriShape` geometry 이름과 skinned bone 이름을 찾습니다.
2. 어떤 body가 anchor인지 정하고, 해당 bone은 `<mass>0</mass>`로 둡니다.
3. 움직이는 chain, cloth strip, tail, hair section용 dynamic bone을 추가합니다.
4. geometry용 mesh descriptor를 하나 이상 추가합니다.
5. 필요한 constraint를 추가합니다.
6. direct `NiStringExtraData` 또는 `defaultBBPs.xml`로 XML을 연결합니다.
7. 작은 angular limit와 작은 mass부터 시작한 뒤, build가 안정적일 때만 범위를 넓힙니다.

실전 팁:

- armor chain에서는 `Pelvis` 같은 실제 actor skeleton bone을, 그것이 진짜 trusted skeleton node일 때만 씁니다. 이름이 `Root` 아래에서 보인다고 해서 actor skeleton bone이라고 단정하면 안 됩니다.
- armor-specific bone은 XML에 남겨 두고 plugin-owned prefixed node로 clone하는 편이 맞습니다.
- hair는 direct `NiStringExtraData`, 낮은 mass, 강한 damping, `<per-vertex-shape>` 순서로 시작하세요.
- cloth는 kinematic root 하나부터 시작해서 dynamic segment를 한 개씩 늘리는 편이 안정적입니다.

## 문제 해결

- 아무것도 build되지 않으면 XML에 `<system>` root가 있는지, 그리고 선택된 path가 실제로 해석되는지 확인하세요.
- template이나 shape reference가 실패하면, 참조된 이름이 파일에서 먼저 선언되었는지 확인하세요.
- body는 만들어지는데 constraint가 없으면 `bodyA`, `bodyB`를 확인하세요.
- mesh collision이 없으면 `BSTriShape` 이름, skin data, CPU vertex/index availability를 확인하세요.
- per-triangle collision이 건너뛰어지면 CPU index buffer를 읽을 수 없는 상태일 수 있습니다.
- body가 어긋나 보이면 `<centerOfMassTransform>`을 조정하세요.
- bone이 엉뚱한 위치에 붙으면 XML에 그 bone이 실제로 정의돼 있는지, 그리고 geometry가 그 bone에 스킨되어 있는지 확인하세요.
- collider가 너무 느슨하거나 너무 타이트하면 `margin`, `penetration`, 또는 해당 bone shape를 조정하세요.
- motion이 폭주하면 angular limit를 줄이고, mass를 낮추고, damping을 높이거나, kinematic root를 쓰세요.
- actor `Root` 아래에 보인다는 이유만으로 선택된 armor node는 trusted actor skeleton bone이 아니라고 가정하는 편이 안전합니다.
