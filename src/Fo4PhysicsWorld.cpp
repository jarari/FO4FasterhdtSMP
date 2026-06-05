#include "Fo4PhysicsWorld.h"

#include "BSSkin.h"
#include "ConfigPaths.h"
#include "DefaultBBP.h"
#include "Fo4MeshExtractor.h"
#include "Fo4NiObjectUtils.h"
#include "Fo4SkinnedMeshBone.h"
#include "Fo4TransformConversion.h"
#include "BulletVisualization.h"
#include "PhysicsName.h"
#include "PhysicsXml.h"
#include "PhysicsXmlSelection.h"
#include "SmpConfig.h"
#include "hdtSkinnedMesh/hdtDispatcher.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"
#include "RE/B/BipedAnim.h"
#include "RE/B/BGSHeadPart.h"
#include "RE/B/bhkPickData.h"
#include "RE/B/BSUtilities.h"
#include "RE/B/BSTimer.h"
#include "RE/C/CFilter.h"
#include "RE/C/COL_LAYER.h"
#include "RE/H/hkArray.h"
#include "RE/H/hkQsTransformf.h"
#include "RE/H/hkReferencedObject.h"
#include "RE/H/hkRefPtr.h"
#include "RE/M/Main.h"
#include "RE/M/MenuOpenCloseEvent.h"
#include "RE/N/NiCloningProcess.h"
#include "RE/N/NiStringExtraData.h"
#include "RE/N/NiUpdateData.h"
#include "RE/P/PlayerCamera.h"
#include "RE/S/Sky.h"
#include "RE/T/TESObjectCELL.h"
#include "RE/T/TESNPC.h"
#include "RE/U/UI.h"

#include <Windows.h>
#ifdef min
#	undef min
#endif
#ifdef max
#	undef max
#endif

#include <btBulletDynamicsCommon.h>

#include <array>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <tbb/parallel_for.h>
#include <tbb/task_group.h>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
	constexpr float kMinimumStepSeconds = 1.0F / 240.0F;
	constexpr float kMinimumShapeExtent = 0.01F;
	constexpr float kGameUnitsPerMeter = 1.0F / 0.01425F;
	constexpr float kGravityAcceleration = -9.80665F * kGameUnitsPerMeter;
	constexpr std::uint32_t kMaxAttachAncestorScanDepth = 2;
	constexpr std::uint32_t kHeadInitializedRebuildDelayFrames = 2;
	constexpr std::uint32_t kArmorChangeRebuildDelayTasks = 0;
	constexpr std::uint32_t kCpuCopyPendingRetryDelayTasks = 10;
	constexpr std::uint32_t kCpuCopyPendingMaxRetries = 3;
	constexpr std::uint64_t kPendingRebuildRetryIntervalFrames = 15;
	std::atomic<std::uint32_t> PrototypeArmorRenameId{ 0 };
	std::atomic<std::uint32_t> PrototypeHeadRenameId{ 0 };
	using Clock = std::chrono::steady_clock;

	bool IsHairBipedObject(const RE::BIPED_OBJECT a_bipedObject)
	{
		return a_bipedObject == RE::BIPED_OBJECT::kHairTop || a_bipedObject == RE::BIPED_OBJECT::kHairLong;
	}

	struct Fo4HkStringPtr
	{
		std::uintptr_t stringAndFlag{ 0 };
	};
	static_assert(sizeof(Fo4HkStringPtr) == 0x8);

	struct Fo4HkaBone
	{
		Fo4HkStringPtr name;
		bool lockTranslation{ false };
		std::byte pad09[0x7]{};
	};
	static_assert(sizeof(Fo4HkaBone) == 0x10);

	struct Fo4HkaSkeleton :
		public RE::hkReferencedObject
	{
		Fo4HkStringPtr name;
		RE::hkArray<std::int16_t> parentIndices;
		RE::hkArray<Fo4HkaBone> bones;
		RE::hkArray<RE::hkQsTransformf> referencePose;
		RE::hkArray<float> referenceFloats;
		RE::hkArray<Fo4HkStringPtr> floatSlots;
		RE::hkArray<std::byte> localFrames;
		RE::hkArray<std::byte> partitions;
	};
	static_assert(sizeof(Fo4HkaSkeleton) == 0x88);

	struct Fo4HkbCharacterSetup :
		public RE::hkReferencedObject
	{
		std::byte pad10[0x10]{};
		RE::hkRefPtr<const Fo4HkaSkeleton> animationSkeleton;
	};
	static_assert(offsetof(Fo4HkbCharacterSetup, animationSkeleton) == 0x20);

	struct Fo4HkbCharacter
	{
		std::byte pad00[0x78]{};
		RE::hkRefPtr<const Fo4HkbCharacterSetup> setup;
	};
	static_assert(offsetof(Fo4HkbCharacter, setup) == 0x78);

	struct Fo4BShkbAnimationGraph
	{
		std::byte pad00[0x1C8]{};
		Fo4HkbCharacter characterInstance;
	};
	static_assert(offsetof(Fo4BShkbAnimationGraph, characterInstance) == 0x1C8);

	struct Fo4BSFaceGenModelMeshData
	{
		std::byte pad00[0x08]{};
		RE::NiPointer<RE::NiAVObject> faceNode;
		RE::NiPointer<RE::NiAVObject> geometry;
		std::byte pad18[0x10]{};
	};
	static_assert(offsetof(Fo4BSFaceGenModelMeshData, faceNode) == 0x08);
	static_assert(sizeof(Fo4BSFaceGenModelMeshData) == 0x28);

	struct Fo4BSFaceGenModel
	{
		std::byte pad00[0x10]{};
		Fo4BSFaceGenModelMeshData* modelMeshData{ nullptr };
		std::byte pad18[0x08]{};
	};
	static_assert(offsetof(Fo4BSFaceGenModel, modelMeshData) == 0x10);
	static_assert(sizeof(Fo4BSFaceGenModel) == 0x20);

	struct Fo4BSFaceGenModelExtraData :
		public RE::NiExtraData
	{
		Fo4BSFaceGenModel* model{ nullptr };
		RE::BSFixedString bones[0x80];
	};
	static_assert(offsetof(Fo4BSFaceGenModelExtraData, model) == 0x18);
	static_assert(offsetof(Fo4BSFaceGenModelExtraData, bones) == 0x20);
	static_assert(sizeof(Fo4BSFaceGenModelExtraData) == 0x420);

	float ElapsedMs(const Clock::time_point a_start, const Clock::time_point a_end)
	{
		return std::chrono::duration<float, std::milli>(a_end - a_start).count();
	}

	thread_local float         FrameCollisionMs{ 0.0F };
	thread_local std::uint32_t FrameCollisionCalls{ 0 };

	void ResetFrameCollisionProfile()
	{
		FrameCollisionMs = 0.0F;
		FrameCollisionCalls = 0;
	}

	void AddFrameCollisionProfile(const float a_collisionMs)
	{
		FrameCollisionMs += std::max(a_collisionMs, 0.0F);
		++FrameCollisionCalls;
	}

	float ConsumeFrameCollisionProfile(std::uint32_t& a_collisionCalls)
	{
		a_collisionCalls = FrameCollisionCalls;
		const auto collisionMs = FrameCollisionMs;
		ResetFrameCollisionProfile();
		return collisionMs;
	}

	class OwnedCompoundShape :
		public btCompoundShape
	{
	public:
		std::vector<std::unique_ptr<btCollisionShape>> children;
	};

	class PrototypeDynamicsWorld :
		public btDiscreteDynamicsWorld
	{
	public:
		using btDiscreteDynamicsWorld::btDiscreteDynamicsWorld;

		int StepReference(const btScalar a_remainingTimeStep, const btScalar a_fixedTimeStep)
		{
			auto remainingTimeStep = a_remainingTimeStep;
			applyGravity();

			while (remainingTimeStep > a_fixedTimeStep) {
				internalSingleStepSimulation(a_fixedTimeStep);
				remainingTimeStep -= a_fixedTimeStep;
			}

			constexpr auto minPossiblePeriod = btScalar(1.0F / 300.0F);
			if (remainingTimeStep > minPossiblePeriod) {
				internalSingleStepSimulation(remainingTimeStep);
			}

			clearForces();
			return 0;
		}

		void applyGravity() override
		{
			const auto worldGravity = getGravity();
			for (int index = 0; index < m_collisionObjects.size(); ++index) {
				auto* body = btRigidBody::upcast(m_collisionObjects[index]);
				if (!body || body->isStaticOrKinematicObject() || (body->getFlags() & BT_DISABLE_WORLD_GRAVITY)) {
					continue;
				}

				if (const auto* bone = static_cast<hdt::SkinnedMeshBone*>(body->getUserPointer())) {
					body->setGravity(worldGravity * std::clamp(bone->m_gravityFactor, 0.0F, 1.0F));
				}
			}

			btDiscreteDynamicsWorld::applyGravity();
		}

		void performDiscreteCollisionDetection() override
		{
			const auto profileStart = Clock::now();
			for (int index = 0; index < m_collisionObjects.size(); ++index) {
				if (auto* rigidBody = btRigidBody::upcast(m_collisionObjects[index])) {
					if (auto* bone = static_cast<hdt::SkinnedMeshBone*>(rigidBody->getUserPointer())) {
						bone->internalUpdate();
					}
				}
			}

			for (int index = 0; index < m_collisionObjects.size(); ++index) {
				auto* object = m_collisionObjects[index];
				if (!object || !object->getCollisionShape() || object->getCollisionShape()->getShapeType() != CUSTOM_CONCAVE_SHAPE_TYPE) {
					continue;
				}

				auto* meshBody = static_cast<hdt::SkinnedMeshBody*>(object);
				meshBody->updateBoundingSphereAabb();
			}

			btDispatcherInfo& dispatchInfo = getDispatchInfo();
			for (int index = 0; index < m_collisionObjects.size(); ++index) {
				auto* object = m_collisionObjects[index];
				auto* proxy = object ? object->getBroadphaseHandle() : nullptr;
				if (!object || !proxy || (proxy->m_collisionFilterGroup == 0 && proxy->m_collisionFilterMask == 0)) {
					continue;
				}

				btVector3 minAabb;
				btVector3 maxAabb;
				object->getCollisionShape()->getAabb(object->getWorldTransform(), minAabb, maxAabb);
				m_broadphasePairCache->setAabb(proxy, minAabb, maxAabb, m_dispatcher1);
			}

			m_broadphasePairCache->calculateOverlappingPairs(m_dispatcher1);
			if (m_dispatcher1) {
				m_dispatcher1->dispatchAllCollisionPairs(m_broadphasePairCache->getOverlappingPairCache(), dispatchInfo, m_dispatcher1);
			}
			AddFrameCollisionProfile(ElapsedMs(profileStart, Clock::now()));
		}

		void integrateTransforms(const btScalar a_timeStep) override
		{
			for (int index = 0; index < m_collisionObjects.size(); ++index) {
				auto* body = m_collisionObjects[index];
				if (!body || !body->isKinematicObject()) {
					continue;
				}

				btTransformUtil::integrateTransform(
					body->getWorldTransform(),
					body->getInterpolationLinearVelocity(),
					body->getInterpolationAngularVelocity(),
					a_timeStep,
					body->getInterpolationWorldTransform());
				body->setWorldTransform(body->getInterpolationWorldTransform());
			}

			const btVector3 limitMin(-1e9F, -1e9F, -1e9F);
			const btVector3 limitMax(1e9F, 1e9F, 1e9F);
			for (int index = 0; index < m_nonStaticRigidBodies.size(); ++index) {
				auto* body = m_nonStaticRigidBodies[index];
				if (!body) {
					continue;
				}

				auto linearVelocity = body->getLinearVelocity();
				linearVelocity.setMax(limitMin);
				linearVelocity.setMin(limitMax);
				body->setLinearVelocity(linearVelocity);

				auto angularVelocity = body->getAngularVelocity();
				angularVelocity.setMax(limitMin);
				angularVelocity.setMin(limitMax);
				body->setAngularVelocity(angularVelocity);
			}

			btDiscreteDynamicsWorld::integrateTransforms(a_timeStep);
		}
	};

	using ArmorPhysicsXmlSelection = Smp::PhysicsXmlSelection::ArmorSelection;

	struct ArmorPhysicsXmlBuildCandidate
	{
		RE::NiAVObject* object{ nullptr };
		ArmorPhysicsXmlSelection selection;
		RE::BIPOBJECT* bipObject{ nullptr };
		RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
		RE::NiAVObject* sourceObject{ nullptr };
		RE::NiNode* sourceRoot{ nullptr };
	};

	struct HeadPhysicsXmlBuildCandidate
	{
		RE::NiAVObject* object{ nullptr };
		std::filesystem::path path;
		Smp::DefaultBBP::NameMap meshNameMap;
		RE::NiPointer<RE::NiAVObject> sourceObject;
		RE::NiPointer<RE::NiAVObject> sourceRoot;
		bool preserveMergeSourceNames{ false };
		Smp::PrototypeBuildDomain domain{ Smp::PrototypeBuildDomain::kHead };
	};

	std::optional<ArmorPhysicsXmlSelection> FindArmorPhysicsXml(RE::NiAVObject* a_object);
	RE::NiNode* FindNodeByName(RE::NiAVObject* a_root, std::string_view a_name);
	RE::NiPoint3 ResolveWindRayStart(RE::Actor* a_actor);
	bool IsReadableMemory(const void* a_address, std::size_t a_minSize);
	bool IsProbablyValidNiObject(const RE::NiObject* a_object);
	void CollectParentInheritedExclusions(
		RE::NiAVObject* a_object,
		std::unordered_set<RE::NiAVObject*>& a_exclusionSet,
		std::vector<RE::NiAVObject*>& a_exclusions);

	float DistanceSquared(const RE::NiPoint3& a_lhs, const RE::NiPoint3& a_rhs)
	{
		const auto dx = a_lhs.x - a_rhs.x;
		const auto dy = a_lhs.y - a_rhs.y;
		const auto dz = a_lhs.z - a_rhs.z;
		return dx * dx + dy * dy + dz * dz;
	}

	RE::NiAVObject* CalculateActorLOS(RE::Actor* a_actor, const RE::NiPoint3& a_targetPosition, RE::NiPoint3& a_hitPosition)
	{
		using ActorCalculateLOSFn = RE::NiAVObject* (*)(RE::Actor*, const RE::NiPoint3&, RE::NiPoint3&, float);
		static REL::Relocation<ActorCalculateLOSFn> actorCalculateLOS{ REL::ID{ 1324305, 0 } };
		return actorCalculateLOS(a_actor, a_targetPosition, a_hitPosition, 6.28F);
	}

	RE::NiPoint3 ResolveActorCullPosition(RE::Actor* a_actor, RE::NiAVObject* a_root)
	{
		if (a_root) {
			if (auto* npcRoot = FindNodeByName(a_root, "NPC Root [Root]")) {
				return npcRoot->world.translate;
			}
			return a_root->world.translate;
		}
		return a_actor ? a_actor->GetPosition() : RE::NiPoint3::ZERO;
	}

	bool IsActorInReferenceCullView(RE::Actor* a_actor, RE::NiAVObject* a_root, const bool a_firstPerson)
	{
		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (!a_actor || a_actor == player) {
			return true;
		}

		if (a_actor) {
			if (auto* primaryRoot = a_actor->Get3D(a_firstPerson)) {
				a_root = primaryRoot;
			} else if (!a_firstPerson) {
				if (auto* fallbackRoot = a_actor->Get3D()) {
					a_root = fallbackRoot;
				}
			}
		}
		if (!a_root) {
			return false;
		}

		const auto* playerCamera = RE::PlayerCamera::GetSingleton();
		if (!playerCamera || !playerCamera->cameraRoot) {
			return true;
		}

		constexpr float kReferenceMinCullingDistance = 500.0F;
		const auto cameraPosition = playerCamera->cameraRoot->world.translate;
		const auto actorPosition = ResolveActorCullPosition(a_actor, a_root);
		if (DistanceSquared(actorPosition, cameraPosition) < kReferenceMinCullingDistance * kReferenceMinCullingDistance) {
			return true;
		}

		auto* worldCamera = RE::Main::WorldRootCamera();
		if (worldCamera && !worldCamera->NodeInFrustum(a_root)) {
			return false;
		}

		RE::NiPoint3 hitPosition;
		const auto* obstacle = CalculateActorLOS(a_actor, cameraPosition, hitPosition);
		return !obstacle;
	}

	const char* PrototypeDomainName(const Smp::PrototypeBuildDomain a_domain)
	{
		switch (a_domain) {
		case Smp::PrototypeBuildDomain::kArmor:
			return "armor";
		case Smp::PrototypeBuildDomain::kHead:
			return "head";
		case Smp::PrototypeBuildDomain::kHair:
			return "hair";
		default:
			return "unknown";
		}
	}

	bool HasPendingCpuCopyExtraction(const Smp::Fo4MeshExtractionResult& a_extraction)
	{
		return a_extraction.stats.matchedGeometries > 0 &&
			(a_extraction.stats.pendingVertexCopies > 0 || a_extraction.stats.pendingIndexCopies > 0);
	}

	std::string NormalizeHeadpartKey(std::string a_value)
	{
		a_value = Smp::ConfigPaths::LowerString(Smp::ConfigPaths::Trim(std::move(a_value)));
		if (a_value.empty()) {
			return {};
		}

		std::replace(a_value.begin(), a_value.end(), '\\', '/');
		if (const auto slash = a_value.find_last_of('/'); slash != std::string::npos) {
			a_value.erase(0, slash + 1);
		}
		if (const auto dot = a_value.find_last_of('.'); dot != std::string::npos) {
			a_value.erase(dot);
		}

		return a_value.size() >= 4 ? a_value : std::string{};
	}

	void AddHeadpartKey(std::vector<std::string>& a_keys, std::string a_value)
	{
		auto key = NormalizeHeadpartKey(std::move(a_value));
		if (key.empty() || std::ranges::find(a_keys, key) != a_keys.end()) {
			return;
		}

		a_keys.push_back(std::move(key));
	}

	void AddHairHeadpartKeys(RE::BGSHeadPart* a_headPart, std::vector<std::string>& a_keys)
	{
		if (!a_headPart || a_headPart->type.get() != RE::BGSHeadPart::HeadPartType::kHair) {
			return;
		}

		AddHeadpartKey(a_keys, a_headPart->ChargenModel.GetModel());
		AddHeadpartKey(a_keys, std::string(std::string_view(a_headPart->formEditorID)));
		for (auto* extraPart : a_headPart->extraParts) {
			AddHairHeadpartKeys(extraPart, a_keys);
		}
	}

	std::vector<std::string> BuildHairHeadpartKeys(RE::Actor* a_actor)
	{
		std::vector<std::string> keys;
		auto* npc = a_actor ? a_actor->GetNPC() : nullptr;
		if (!npc) {
			return keys;
		}

		for (auto* headPart : npc->GetHeadParts(true)) {
			AddHairHeadpartKeys(headPart, keys);
		}
		return keys;
	}

	bool IsHairSubtree(RE::NiAVObject* a_object, const std::vector<std::string>& a_hairKeys)
	{
		if (!a_object || a_hairKeys.empty()) {
			return false;
		}

		const auto name = Smp::ConfigPaths::LowerString(std::string(std::string_view(a_object->GetName())));
		if (name.empty()) {
			return false;
		}

		return std::ranges::any_of(a_hairKeys, [&name](const std::string& a_key) {
			return name.find(a_key) != std::string::npos;
		});
	}

	void LogObjectHierarchy(RE::NiAVObject* a_object, const char* a_label, const std::uint32_t a_depth = 0, std::uint32_t* a_remaining = nullptr)
	{
		if (!a_object) {
			return;
		}

		std::uint32_t localRemaining = 4096;
		if (!a_remaining) {
			a_remaining = std::addressof(localRemaining);
		}
		if (*a_remaining == 0) {
			return;
		}
		--(*a_remaining);

		auto* geometry = a_object->IsGeometry();
		auto* node = a_object->IsNode();
		std::string treePrefix;
		if (a_depth > 0) {
			treePrefix.assign(a_depth, '-');
			treePrefix.push_back(' ');
		}
		spdlog::debug(
			"{} {}{} ({}/{})",
			a_label,
			treePrefix,
			std::string_view(a_object->GetName()),
			static_cast<void*>(a_object),
			geometry ? "geometry" : node ? "node" : "object");

		if (a_object->extra) {
			for (auto* extra : *a_object->extra) {
				if (auto* stringExtra = netimmerse_cast<RE::NiStringExtraData*>(extra)) {
					spdlog::debug(
						"{} {}@{}='{}' ({}/extra)",
						a_label,
						treePrefix,
						std::string_view(stringExtra->name),
						std::string_view(stringExtra->data),
						static_cast<void*>(a_object));
				}
			}
		}

		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			LogObjectHierarchy(child.get(), a_label, a_depth + 1, a_remaining);
			if (*a_remaining == 0) {
				spdlog::debug("{} hierarchy truncated after 4096 objects", a_label);
				return;
			}
		}
	}

	btVector3 WindDirectionFromFo4SkyAngle(const float a_radians)
	{
		return btVector3(std::sin(a_radians), std::cos(a_radians), 0.0F);
	}

	RE::NiPoint3 ToNiPoint3(const btVector3& a_value)
	{
		return RE::NiPoint3(a_value.x(), a_value.y(), a_value.z());
	}

	btVector3 ToBulletVector(const RE::NiPoint3& a_value)
	{
		return btVector3(a_value.x, a_value.y, a_value.z);
	}

	void ClearWindState(btVector3& a_currentWind, btVector3& a_targetWind, float& a_cooldown, const float a_longCooldown)
	{
		a_currentWind.setZero();
		a_targetWind.setZero();
		a_cooldown = std::max(a_longCooldown, 0.0F);
	}

	bool IsWeatherWindSkyValid(const RE::Sky* a_sky)
	{
		return a_sky &&
			a_sky->currentWeather &&
			a_sky->mode == RE::Sky::Mode::kFull &&
			!a_sky->flags.any(RE::Sky::Flags::kHideSky);
	}

	RE::NiPoint3 ResolveWindRayStart(RE::Actor* a_actor)
	{
		if (a_actor) {
			if (auto* root = a_actor->Get3D(false); root) {
				if (auto* headNode = FindNodeByName(root, "NPC Head [Head]")) {
					return headNode->world.translate;
				}
			}
			if (auto* root = a_actor->Get3D(); root) {
				if (auto* headNode = FindNodeByName(root, "NPC Head [Head]")) {
					return headNode->world.translate;
				}
			}
			auto start = a_actor->GetPosition();
			start.z += 100.0F;
			return start;
		}

		return RE::NiPoint3::ZERO;
	}

	bool IsActorWeatherWindCellValid(RE::Actor* a_actor)
	{
		auto* cell = a_actor ? a_actor->GetParentCell() : nullptr;
		return cell && cell->IsExterior() && cell->worldSpace;
	}

	float ResolveActorWindObstructionFactor(RE::Actor* a_actor, const btVector3& a_windDirection, const float a_noWindDistance, const float a_fullWindDistance)
	{
		const auto rayDistance = std::max(a_fullWindDistance, 1.0F);
		const auto noWindDistance = std::clamp(a_noWindDistance, 0.0F, rayDistance);
		if (!a_actor || a_windDirection.length2() <= SIMD_EPSILON || rayDistance <= 0.0F) {
			return 1.0F;
		}

		auto* cell = a_actor->GetParentCell();
		if (!cell) {
			return 1.0F;
		}

		auto upwind = a_windDirection;
		upwind.normalize();
		upwind = -upwind;

		const auto start = ResolveWindRayStart(a_actor);
		const auto end = ToNiPoint3(ToBulletVector(start) + (upwind * rayDistance));

		RE::bhkPickData pickData;
		RE::CFilter filter{};
		filter.SetCollisionLayer(RE::COL_LAYER::kLOS);
		pickData.castQuery.filterData.collisionFilterInfo = filter.filter;
		pickData.SetStartEnd(start, end);
		[[maybe_unused]] auto* pickedObject = cell->Pick(pickData);

		if (pickData.pickFailed || !pickData.HasHit()) {
			return 1.0F;
		}

		const auto hitDistance = std::clamp(pickData.GetHitFraction(), 0.0F, 1.0F) * rayDistance;
		if (hitDistance <= noWindDistance) {
			return 0.0F;
		}
		if (rayDistance <= noWindDistance) {
			return 1.0F;
		}
		return std::clamp((hitDistance - noWindDistance) / (rayDistance - noWindDistance), 0.0F, 1.0F);
	}

	float StableWindVariation(const std::string_view a_name)
	{
		std::uint32_t hash = 2166136261U;
		for (const auto ch : a_name) {
			hash ^= static_cast<std::uint8_t>(ch);
			hash *= 16777619U;
		}
		const auto normalized = static_cast<float>(hash % 1000U) / 999.0F;
		return 0.85F + (normalized * 0.30F);
	}

	bool IsAttachCandidate(const Smp::LifecycleEventType a_type)
	{
		switch (a_type) {
		case Smp::LifecycleEventType::kArmorApplySkinnedObjects:
		case Smp::LifecycleEventType::kArmorAttachSkinnedObject:
		case Smp::LifecycleEventType::kArmorAttachToParent:
		case Smp::LifecycleEventType::kActorLoad3D:
		case Smp::LifecycleEventType::kActorSet3D:
			return true;
		default:
			return false;
		}
	}

	bool IsArmorAttachCandidate(const Smp::LifecycleEventType a_type)
	{
		switch (a_type) {
		case Smp::LifecycleEventType::kArmorApplySkinnedObjects:
		case Smp::LifecycleEventType::kArmorAttachSkinnedObject:
		case Smp::LifecycleEventType::kArmorAttachToParent:
			return true;
		default:
			return false;
		}
	}

	bool IsHeadCandidate(const Smp::LifecycleEventType a_type)
	{
		return a_type == Smp::LifecycleEventType::kActorHeadInitialized ||
			a_type == Smp::LifecycleEventType::kHeadPrepareHeadPart;
	}

	bool IsPlayerFirstPersonView()
	{
		const auto* player = RE::PlayerCharacter::GetSingleton();
		const auto* camera = RE::PlayerCamera::GetSingleton();
		if (!player || !camera || !camera->currentState) {
			return false;
		}

		const auto* firstPersonState = camera->cameraStates[RE::CameraState::kFirstPerson].get();
		return firstPersonState && camera->currentState.get() == firstPersonState;
	}

	bool IsIgnoredFirstPersonEvent(const Smp::LifecycleEvent& a_event, const bool a_disableFirstPersonViewPhysics)
	{
		if (!a_disableFirstPersonViewPhysics) {
			return false;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (a_event.firstPerson) {
			return true;
		}
		if (player && a_event.biped && player->firstPersonBipedAnim.get() == a_event.biped) {
			return true;
		}
		if (player && a_event.object && player->firstPerson3D.get() == a_event.object) {
			return true;
		}

		return false;
	}

	bool IsGamePaused()
	{
		if (!RE::Main::QGameSystemsShouldUpdate()) {
			return true;
		}

		if (const auto* main = RE::Main::GetSingleton(); main && main->inMenuMode) {
			return true;
		}

		if (const auto* ui = RE::UI::GetSingleton()) {
			return ui->menuMode > 0 || ui->freezeFramePause > 0;
		}

		return false;
	}

	std::string LowerMenuName(const RE::BSFixedString& a_name)
	{
		std::string result{ std::string_view(a_name) };
		for (auto& ch : result) {
			ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
		}
		return result;
	}

	std::optional<std::filesystem::path> FindDirectPhysicsXmlExtraData(RE::NiAVObject* a_object)
	{
		return Smp::PhysicsXmlSelection::FindDirectPhysicsXmlExtraData(a_object);
	}

	RE::NiNode* ResolveFaceGenOriginalRoot(RE::NiAVObject* a_object)
	{
		if (!a_object || !a_object->extra) {
			return nullptr;
		}

		for (auto* extra : *a_object->extra) {
			if (!extra || !Smp::PhysicsNamesEqual(std::string_view(extra->name), "FMD")) {
				continue;
			}

			auto* fmd = static_cast<Fo4BSFaceGenModelExtraData*>(extra);
			auto* meshData = fmd->model ? fmd->model->modelMeshData : nullptr;
			auto* faceNode = meshData ? meshData->faceNode.get() : nullptr;
			if (auto* root = faceNode ? faceNode->IsNode() : nullptr) {
				return root;
			}
		}

		return nullptr;
	}

	void AppendHeadCandidate(
		std::vector<HeadPhysicsXmlBuildCandidate>& a_candidates,
		RE::NiAVObject* a_object,
		std::filesystem::path a_path,
		Smp::DefaultBBP::NameMap a_meshNameMap,
		RE::NiAVObject* a_sourceObject,
		RE::NiNode* a_sourceRoot,
		const bool a_preserveMergeSourceNames,
		const Smp::PrototypeBuildDomain a_domain)
	{
		if (!a_object || a_path.empty()) {
			return;
		}

		const auto normalizedPath = Smp::ConfigPaths::LowerString(a_path.string());
		const auto duplicate = std::ranges::any_of(a_candidates, [&](const HeadPhysicsXmlBuildCandidate& a_candidate) {
			return a_candidate.object == a_object &&
				a_candidate.domain == a_domain &&
				Smp::ConfigPaths::LowerString(a_candidate.path.string()) == normalizedPath;
		});
		if (duplicate) {
			return;
		}

		a_candidates.push_back({
			.object = a_object,
			.path = std::move(a_path),
			.meshNameMap = std::move(a_meshNameMap),
			.sourceObject = a_sourceObject,
			.sourceRoot = a_sourceRoot,
			.preserveMergeSourceNames = a_preserveMergeSourceNames,
			.domain = a_domain,
		});
	}

	std::optional<ArmorPhysicsXmlSelection> FindArmorPhysicsXml(const Smp::LifecycleEvent& a_event)
	{
		if (!a_event.physicsXmlPath.empty()) {
			return ArmorPhysicsXmlSelection{ .path = a_event.physicsXmlPath };
		}
		if (auto selection = FindArmorPhysicsXml(a_event.object)) {
			return selection;
		}
		std::uint32_t ancestorDepth = 0;
		for (auto* parent = a_event.object ? a_event.object->parent : nullptr; parent && ancestorDepth < kMaxAttachAncestorScanDepth; parent = parent->parent, ++ancestorDepth) {
			if (auto path = FindDirectPhysicsXmlExtraData(parent)) {
				spdlog::debug(
					"resolved armor physics XML from attached object ancestor={} name='{}' for attached object={}",
					static_cast<void*>(parent),
					std::string_view(parent->GetName()),
					static_cast<void*>(a_event.object));
				return ArmorPhysicsXmlSelection{ .path = *path };
			}
			if (parent == a_event.destinationRoot) {
				break;
			}
		}
		if (auto selection = FindArmorPhysicsXml(a_event.sourceObject)) {
			spdlog::debug("resolved armor physics XML from source object={} for attached object={}", static_cast<void*>(a_event.sourceObject), static_cast<void*>(a_event.object));
			return selection;
		}
		if (auto selection = FindArmorPhysicsXml(a_event.sourceRoot)) {
			spdlog::debug("resolved armor physics XML from source root={} for attached object={}", static_cast<void*>(a_event.sourceRoot), static_cast<void*>(a_event.object));
			return selection;
		}
		if (auto selection = FindArmorPhysicsXml(a_event.destinationRoot)) {
			spdlog::debug("resolved armor physics XML from destination root={} for attached object={}", static_cast<void*>(a_event.destinationRoot), static_cast<void*>(a_event.object));
			return selection;
		}

		return std::nullopt;
	}

	void CollectDirectArmorPhysicsXmlSelections(RE::NiAVObject* a_object, std::vector<ArmorPhysicsXmlBuildCandidate>& a_candidates)
	{
		if (!a_object) {
			return;
		}

		if (auto directXml = FindDirectPhysicsXmlExtraData(a_object)) {
			a_candidates.push_back({
				.object = a_object,
				.selection = ArmorPhysicsXmlSelection{ .path = *directXml },
			});
			return;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			if (auto defaultBbp = Smp::DefaultBBP::GetSingleton()->Find(a_object)) {
				a_candidates.push_back({
					.object = a_object,
					.selection = ArmorPhysicsXmlSelection{
						.path = defaultBbp->physicsXml,
						.meshNameMap = std::move(defaultBbp->meshNameMap),
					},
				});
			}
			return;
		}

		const auto previousCandidateCount = a_candidates.size();
		for (auto& child : node->children) {
			CollectDirectArmorPhysicsXmlSelections(child.get(), a_candidates);
		}
		if (a_candidates.size() == previousCandidateCount) {
			if (auto defaultBbp = Smp::DefaultBBP::GetSingleton()->Find(a_object)) {
				a_candidates.push_back({
					.object = a_object,
					.selection = ArmorPhysicsXmlSelection{
						.path = defaultBbp->physicsXml,
						.meshNameMap = std::move(defaultBbp->meshNameMap),
					},
				});
			}
		}
	}

	RE::BipedAnim* ResolveEventBiped(const Smp::LifecycleEvent& a_event)
	{
		if (a_event.biped) {
			return a_event.biped;
		}
		if (!a_event.actor) {
			return nullptr;
		}

		const auto& biped = a_event.actor->GetBiped(a_event.firstPerson);
		if (biped) {
			return biped.get();
		}
		return a_event.actor->GetBiped().get();
	}

	RE::BIPED_OBJECT ResolveEventBipedObject(const Smp::LifecycleEvent& a_event)
	{
		if (a_event.bipedObject != RE::BIPED_OBJECT::kTotal) {
			return a_event.bipedObject;
		}
		if (!a_event.bipObject) {
			return RE::BIPED_OBJECT::kTotal;
		}

		auto* biped = ResolveEventBiped(a_event);
		if (!biped) {
			return RE::BIPED_OBJECT::kTotal;
		}

		for (auto index = 0; index < std::to_underlying(RE::BIPED_OBJECT::kTotal); ++index) {
			const auto bipedObject = static_cast<RE::BIPED_OBJECT>(index);
			if (biped->GetBipObject(bipedObject) == a_event.bipObject) {
				return bipedObject;
			}
		}

		return RE::BIPED_OBJECT::kTotal;
	}

	std::vector<RE::NiAVObject*> BuildBipedPartCloneExclusions(const Smp::LifecycleEvent& a_event)
	{
		std::vector<RE::NiAVObject*> exclusions;
		auto* primaryActorRoot = a_event.actor ? a_event.actor->Get3D(a_event.firstPerson) : nullptr;
		auto* thirdPersonActorRoot = a_event.actor ? a_event.actor->Get3D(false) : nullptr;
		auto* firstPersonActorRoot = a_event.actor ? a_event.actor->Get3D(true) : nullptr;
		auto addExclusion = [&exclusions](RE::NiAVObject* a_object) {
			if (a_object && std::ranges::find(exclusions, a_object) == exclusions.end()) {
				exclusions.push_back(a_object);
			}
		};
		auto addArmorExclusion = [&](RE::NiAVObject* a_object) {
			if (!a_object || a_object == primaryActorRoot || a_object == thirdPersonActorRoot || a_object == firstPersonActorRoot) {
				return;
			}
			addExclusion(a_object);
		};

		addArmorExclusion(a_event.object);
		if (!a_event.mergeSourceObject) {
			addArmorExclusion(a_event.sourceObject);
			addArmorExclusion(a_event.sourceRoot);
		}
		addArmorExclusion(a_event.mergeSourceObject);

		auto* biped = ResolveEventBiped(a_event);
		if (!biped) {
			return exclusions;
		}

		for (auto index = 0; index < std::to_underlying(RE::BIPED_OBJECT::kTotal); ++index) {
			const auto bipedObject = static_cast<RE::BIPED_OBJECT>(index);
			auto* bipObject = biped->GetBipObject(bipedObject);
			if (bipObject && bipObject->partClone) {
				addExclusion(bipObject->partClone.get());
			}
		}

		std::unordered_set<RE::NiAVObject*> exclusionSet(exclusions.begin(), exclusions.end());
		CollectParentInheritedExclusions(primaryActorRoot, exclusionSet, exclusions);
		if (thirdPersonActorRoot != primaryActorRoot) {
			CollectParentInheritedExclusions(thirdPersonActorRoot, exclusionSet, exclusions);
		}
		if (firstPersonActorRoot != primaryActorRoot && firstPersonActorRoot != thirdPersonActorRoot) {
			CollectParentInheritedExclusions(firstPersonActorRoot, exclusionSet, exclusions);
		}

		return exclusions;
	}

	bool HasEquippedHairSlotObject(RE::Actor* a_actor)
	{
		if (!a_actor) {
			return false;
		}

		Smp::LifecycleEvent event{
			.type = Smp::LifecycleEventType::kArmorAttachSkinnedObject,
			.actor = a_actor,
			.firstPerson = false,
		};
		auto* biped = ResolveEventBiped(event);
		if (!biped) {
			return false;
		}

		for (const auto slot : { RE::BIPED_OBJECT::kHairTop, RE::BIPED_OBJECT::kHairLong }) {
			auto* bipObject = biped->GetBipObject(slot);
			if (bipObject && bipObject->partClone) {
				return true;
			}
		}
		return false;
	}

	bool HasArmorBuildCandidateObject(
		const std::vector<ArmorPhysicsXmlBuildCandidate>& a_candidates,
		RE::NiAVObject* a_object)
	{
		if (!a_object) {
			return true;
		}

		return std::ranges::any_of(a_candidates, [a_object](const ArmorPhysicsXmlBuildCandidate& a_candidate) {
			return a_candidate.object == a_object;
		});
	}

	void AppendEquippedArmorCandidate(
		std::vector<ArmorPhysicsXmlBuildCandidate>& a_candidates,
		ArmorPhysicsXmlBuildCandidate a_candidate,
		RE::BIPOBJECT* a_bipObject,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_partClone)
	{
		if (!a_candidate.object || HasArmorBuildCandidateObject(a_candidates, a_candidate.object)) {
			return;
		}

		a_candidate.bipObject = a_bipObject;
		a_candidate.bipedObject = a_bipedObject;
		a_candidate.sourceObject = a_partClone;
		a_candidate.sourceRoot = a_partClone ? a_partClone->IsNode() : nullptr;
		a_candidates.push_back(std::move(a_candidate));
	}

	void CollectEquippedArmorPhysicsXmlSelections(
		const Smp::LifecycleEvent& a_event,
		std::vector<ArmorPhysicsXmlBuildCandidate>& a_candidates)
	{
		auto* biped = ResolveEventBiped(a_event);
		if (!biped) {
			return;
		}

		const auto previousCandidateCount = a_candidates.size();
		for (auto index = 0; index < std::to_underlying(RE::BIPED_OBJECT::kTotal); ++index) {
			const auto bipedObject = static_cast<RE::BIPED_OBJECT>(index);
			auto* bipObject = biped->GetBipObject(bipedObject);
			auto* partClone = bipObject ? bipObject->partClone.get() : nullptr;
			if (!partClone) {
				continue;
			}

			std::vector<ArmorPhysicsXmlBuildCandidate> subtreeCandidates;
			CollectDirectArmorPhysicsXmlSelections(partClone, subtreeCandidates);
			if (subtreeCandidates.empty()) {
				if (auto selection = FindArmorPhysicsXml(partClone)) {
					subtreeCandidates.push_back({
						.object = partClone,
						.selection = std::move(*selection),
					});
				}
			}

			for (auto& candidate : subtreeCandidates) {
				AppendEquippedArmorCandidate(
					a_candidates,
					std::move(candidate),
					bipObject,
					bipedObject,
					partClone);
			}
		}

		const auto added = a_candidates.size() - previousCandidateCount;
		if (added > 0) {
			spdlog::debug(
				"found {} equipped biped armor physics candidates for actor={} biped={} root={}",
				added,
				static_cast<void*>(a_event.actor),
				static_cast<void*>(biped),
				static_cast<void*>(biped->GetRoot()));
		}
	}

	void CollectHeadPhysicsXmlSelections(
		RE::NiAVObject* a_object,
		const std::vector<std::string>& a_hairKeys,
		std::vector<HeadPhysicsXmlBuildCandidate>& a_candidates,
		const bool a_parentIsHair = false)
	{
		if (!a_object) {
			return;
		}

		const auto isHair = a_parentIsHair || IsHairSubtree(a_object, a_hairKeys);
		auto* originalRoot = ResolveFaceGenOriginalRoot(a_object);
		auto* sourceObject = originalRoot ? static_cast<RE::NiAVObject*>(originalRoot) : a_object;
		if (auto directXml = FindDirectPhysicsXmlExtraData(a_object)) {
			AppendHeadCandidate(
				a_candidates,
				a_object,
				*directXml,
				{},
				sourceObject,
				originalRoot ? originalRoot : a_object->IsNode(),
				originalRoot != nullptr,
				isHair ? Smp::PrototypeBuildDomain::kHair : Smp::PrototypeBuildDomain::kHead);
			return;
		}

		if (originalRoot) {
			if (auto directXml = FindDirectPhysicsXmlExtraData(originalRoot)) {
				AppendHeadCandidate(
					a_candidates,
					a_object,
					*directXml,
					{},
					sourceObject,
					originalRoot,
					true,
					isHair ? Smp::PrototypeBuildDomain::kHair : Smp::PrototypeBuildDomain::kHead);
				return;
			}
		}

		if (auto defaultBbp = Smp::DefaultBBP::GetSingleton()->Find(sourceObject)) {
			AppendHeadCandidate(
				a_candidates,
				a_object,
				defaultBbp->physicsXml,
				std::move(defaultBbp->meshNameMap),
				sourceObject,
				originalRoot ? originalRoot : sourceObject->IsNode(),
				originalRoot != nullptr,
				isHair ? Smp::PrototypeBuildDomain::kHair : Smp::PrototypeBuildDomain::kHead);
			return;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			CollectHeadPhysicsXmlSelections(child.get(), a_hairKeys, a_candidates, isHair);
		}
	}

	std::optional<ArmorPhysicsXmlSelection> FindArmorPhysicsXml(RE::NiAVObject* a_object)
	{
		return Smp::PhysicsXmlSelection::FindArmorPhysicsXml(a_object);
	}

	bool IsResetCandidate(const Smp::LifecycleEventType a_type)
	{
		switch (a_type) {
		case Smp::LifecycleEventType::kActorReset3D:
			return true;
		default:
			return false;
		}
	}

	bool IsArmorDetachCandidate(const Smp::LifecycleEventType a_type)
	{
		switch (a_type) {
		case Smp::LifecycleEventType::kArmorDetachBegin:
		case Smp::LifecycleEventType::kArmorDetachEnd:
			return true;
		default:
			return false;
		}
	}

	bool IsNamedPrototypeBone(const std::vector<std::string>& a_boneNames, const std::string_view a_nodeName)
	{
		if (a_boneNames.empty()) {
			return true;
		}
		return Smp::FindMatchingPhysicsName(a_boneNames, a_nodeName).has_value();
	}

	const Smp::DefaultBBP::NameSet* FindMeshAliases(const Smp::DefaultBBP::NameMap& a_meshNameMap, const std::string_view a_name)
	{
		return Smp::PhysicsXmlSelection::FindMeshAliases(a_meshNameMap, a_name);
	}

	bool MeshNameMatches(const std::string_view a_descriptorName, const std::string_view a_geometryName, const Smp::DefaultBBP::NameMap& a_meshNameMap)
	{
		return Smp::PhysicsXmlSelection::MeshNameMatches(a_descriptorName, a_geometryName, a_meshNameMap);
	}

	std::vector<std::string> BuildMeshMatchNames(const Smp::PhysicsXmlSummary& a_summary, const Smp::DefaultBBP::NameMap& a_meshNameMap)
	{
		return Smp::PhysicsXmlSelection::BuildMeshMatchNames(a_summary, a_meshNameMap);
	}

	bool IsNamedPrototypeMesh(const std::vector<std::string>& a_meshNames, RE::BSGeometry* a_geometry)
	{
		if (a_meshNames.empty()) {
			return true;
		}

		const auto name = a_geometry ? a_geometry->GetName() : "";
		if (name.empty()) {
			return false;
		}

		const std::string_view geometryName(name);
		return Smp::FindMatchingPhysicsName(a_meshNames, geometryName).has_value();
	}

	struct MatchedSkinBone
	{
		struct SkinWorldTransformSlot
		{
			RE::NiPointer<RE::BSSkin::Instance> skin;
			std::uint32_t index{ 0 };
			RE::NiPointer<RE::NiAVObject> originalBone;
			RE::NiTransform* originalWorldTransform{ nullptr };
			RE::NiPointer<RE::NiAVObject> originalRootNode;
		};

		RE::NiNode* node{ nullptr };
		RE::NiNode* sourceNode{ nullptr };
		std::string name;
		bool resolvedFromSkeleton{ false };
		bool useActorKinematicBody{ false };
		std::vector<SkinWorldTransformSlot> skinWorldTransforms;
	};

	struct MergedSkeletonNode
	{
		std::string originalName;
		std::string renamedName;
		RE::NiNode* node{ nullptr };
	};

	struct MergedRootNode
	{
		RE::NiNode* parent{ nullptr };
		RE::NiPointer<RE::NiAVObject> node;
		RE::NiNode* sourceNode{ nullptr };
		std::string originalName;
		std::string recordParentName;
		RE::NiTransform localToParent{ RE::NiTransform::IDENTITY };
		RE::NiTransform recordLocalToParent{ RE::NiTransform::IDENTITY };
		bool hasLocalToParent{ false };
		bool hasRecordLocalToParent{ false };
		bool recordMergeParentBinding{ false };
	};

	struct SavedNodeLocalPose
	{
		RE::NiNode* node{ nullptr };
		RE::NiTransform local;
	};

	enum class SourceBoneOwnership
	{
		kIgnored,
		kActorSkeleton,
		kArmorOwned
	};

	std::string MakeReferenceArmorRenamePrefix(const std::uint32_t a_id)
	{
		char buffer[48]{};
		std::snprintf(buffer, sizeof(buffer), "hdtSSEPhysics_AutoRename_Armor_%08X ", a_id);
		return buffer;
	}

	std::string MakeReferenceHeadRenamePrefix(const std::uint32_t a_id)
	{
		char buffer[48]{};
		std::snprintf(buffer, sizeof(buffer), "hdtSSEPhysics_AutoRename_Head_%08X ", a_id);
		return buffer;
	}

	std::string MakeReferenceSmpClonedPrefix(const std::string& a_prefix)
	{
		auto result = a_prefix;
		result += "SMPCloned ";
		return result;
	}

	MatchedSkinBone* FindMatchedSkinBone(std::vector<MatchedSkinBone>& a_nodes, RE::NiNode* a_node)
	{
		const auto found = std::ranges::find_if(a_nodes, [a_node](const auto& a_entry) {
			return a_entry.node == a_node;
		});
		return found == a_nodes.end() ? nullptr : std::addressof(*found);
	}

	MatchedSkinBone* FindMatchedSkinBoneByName(std::vector<MatchedSkinBone>& a_nodes, const std::string_view a_name)
	{
		const auto found = std::ranges::find_if(a_nodes, [a_name](const auto& a_entry) {
			return Smp::PhysicsNamesEqual(a_entry.name, a_name);
		});
		return found == a_nodes.end() ? nullptr : std::addressof(*found);
	}

	MatchedSkinBone* FindMatchedSkinBoneByMergedName(
		std::vector<MatchedSkinBone>& a_nodes,
		const std::vector<MergedSkeletonNode>& a_renamedNodes,
		const std::string_view a_name)
	{
		if (auto* matched = FindMatchedSkinBoneByName(a_nodes, a_name)) {
			return matched;
		}

		const auto renamed = std::ranges::find_if(a_renamedNodes, [a_name](const MergedSkeletonNode& a_entry) {
			return Smp::PhysicsNamesEqual(a_entry.renamedName, a_name);
		});
		if (renamed == a_renamedNodes.end()) {
			return nullptr;
		}

		return FindMatchedSkinBoneByName(a_nodes, renamed->originalName);
	}

	void AddSkinWorldTransformSlot(MatchedSkinBone& a_bone, RE::BSSkin::Instance* a_skin, const std::uint32_t a_index)
	{
		if (!a_skin || a_index >= a_skin->worldTransforms.size() || !a_skin->worldTransforms[a_index]) {
			return;
		}

		const auto found = std::ranges::find_if(a_bone.skinWorldTransforms, [a_skin, a_index](const MatchedSkinBone::SkinWorldTransformSlot& a_slot) {
			return a_slot.skin.get() == a_skin && a_slot.index == a_index;
		});
		if (found != a_bone.skinWorldTransforms.end()) {
			return;
		}

		a_bone.skinWorldTransforms.push_back({
			.skin = a_skin,
			.index = a_index,
			.originalBone = a_index < a_skin->bones.size() ? a_skin->bones[a_index] : nullptr,
			.originalWorldTransform = a_skin->worldTransforms[a_index],
			.originalRootNode = a_skin->rootNode,
		});
	}

	RE::NiNode* FindNodeByName(RE::NiAVObject* a_root, const std::string_view a_name)
	{
		return Smp::NiObject::FindNodeByName(a_root, a_name);
	}

	bool IsExcludedMergeSearchObject(
		RE::NiAVObject* a_object,
		const std::vector<RE::NiAVObject*>& a_excludedObjects)
	{
		if (!a_object) {
			return true;
		}

		return std::ranges::find(a_excludedObjects, a_object) != a_excludedObjects.end();
	}

	RE::NiAVObject* ResolveSkeletonSearchRoot(const Smp::LifecycleEvent& a_event)
	{
		if (a_event.actor) {
			if (auto* root = a_event.actor->Get3D(a_event.firstPerson)) {
				return root;
			}
			if (auto* root = a_event.actor->Get3D()) {
				return root;
			}
		}
		return nullptr;
	}

	bool IsNodeInTree(RE::NiAVObject* a_object, RE::NiNode* a_needle)
	{
		if (!a_object || !a_needle) {
			return false;
		}

		if (a_object == a_needle) {
			return true;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return false;
		}

		for (auto& child : node->children) {
			if (IsNodeInTree(child.get(), a_needle)) {
				return true;
			}
		}

		return false;
	}

	bool IsObjectInTree(RE::NiAVObject* a_object, RE::NiAVObject* a_needle)
	{
		if (!a_object || !a_needle) {
			return false;
		}

		if (a_object == a_needle) {
			return true;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return false;
		}

		for (auto& child : node->children) {
			if (IsObjectInTree(child.get(), a_needle)) {
				return true;
			}
		}

		return false;
	}

	bool IsObjectDescendantOf(RE::NiAVObject* a_object, RE::NiAVObject* a_ancestor)
	{
		return Smp::NiObject::IsDescendantOf(a_object, a_ancestor);
	}

	void CollectParentInheritedExclusions(
		RE::NiAVObject* a_object,
		std::unordered_set<RE::NiAVObject*>& a_exclusionSet,
		std::vector<RE::NiAVObject*>& a_exclusions)
	{
		if (!a_object) {
			return;
		}

		if (a_object->parent && a_exclusionSet.contains(a_object->parent)) {
			if (a_exclusionSet.insert(a_object).second) {
				a_exclusions.push_back(a_object);
			}
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			CollectParentInheritedExclusions(child.get(), a_exclusionSet, a_exclusions);
		}
	}

	void CollectObjectTree(RE::NiAVObject* a_object, std::unordered_set<RE::NiAVObject*>& a_result)
	{
		if (!a_object || !a_result.insert(a_object).second) {
			return;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			if (auto* childObject = child.get()) {
				CollectObjectTree(childObject, a_result);
			}
		}
	}

	std::unordered_set<RE::NiAVObject*> BuildKnownArmorNodeSet(
		const Smp::LifecycleEvent& a_event,
		const std::vector<MergedRootNode>* a_mergedRoots = nullptr)
	{
		std::vector<RE::NiAVObject*> roots;
		auto* primaryActorRoot = a_event.actor ? a_event.actor->Get3D(a_event.firstPerson) : nullptr;
		auto* thirdPersonActorRoot = a_event.actor ? a_event.actor->Get3D(false) : nullptr;
		auto* firstPersonActorRoot = a_event.actor ? a_event.actor->Get3D(true) : nullptr;
		auto addRoot = [&roots](RE::NiAVObject* a_object) {
			if (a_object && std::ranges::find(roots, a_object) == roots.end()) {
				roots.push_back(a_object);
			}
		};
		auto addArmorRoot = [&](RE::NiAVObject* a_object) {
			if (!a_object || a_object == primaryActorRoot || a_object == thirdPersonActorRoot || a_object == firstPersonActorRoot) {
				return;
			}
			addRoot(a_object);
		};

		addArmorRoot(a_event.object);
		if (!a_event.mergeSourceObject) {
			addArmorRoot(a_event.sourceObject);
			addArmorRoot(a_event.sourceRoot);
		}
		addArmorRoot(a_event.mergeSourceObject);
		if (auto* biped = ResolveEventBiped(a_event)) {
			for (auto index = 0; index < std::to_underlying(RE::BIPED_OBJECT::kTotal); ++index) {
				const auto bipedObject = static_cast<RE::BIPED_OBJECT>(index);
				auto* bipObject = biped->GetBipObject(bipedObject);
				if (bipObject && bipObject->partClone) {
					addRoot(bipObject->partClone.get());
				}
			}
		}
		if (a_mergedRoots) {
			for (const auto& mergedRoot : *a_mergedRoots) {
				addRoot(mergedRoot.node.get());
			}
		}

		std::unordered_set<RE::NiAVObject*> result;
		for (auto* root : roots) {
			if (IsProbablyValidNiObject(root)) {
				CollectObjectTree(root, result);
			}
		}
		return result;
	}

	std::unordered_set<RE::NiAVObject*> BuildTrustedActorSkeletonNodeSet(const Smp::LifecycleEvent& a_event)
	{
		std::unordered_set<RE::NiAVObject*> result;
		result.reserve(a_event.trustedActorSkeletonNodes.size());
		for (auto* object : a_event.trustedActorSkeletonNodes) {
			if (!IsProbablyValidNiObject(object)) {
				continue;
			}
			auto* node = object->IsNode();
			if (!IsProbablyValidNiObject(node)) {
				continue;
			}
			const auto name = node->GetName();
			if (!name.empty() && Smp::IsAutoRenamedPhysicsName(name)) {
				continue;
			}
			result.insert(node);
		}
		return result;
	}

	std::string NormalizeActorLookupName(const std::string_view a_name)
	{
		return Smp::ConfigPaths::LowerString(std::string(a_name));
	}

	struct ActorSkeletonLookup
	{
		std::unordered_map<std::string, RE::NiNode*> nodesByName;
		std::unordered_set<RE::NiAVObject*> trustedNodes;
	};

	bool IsTrustedActorSkeletonCandidate(
		RE::NiAVObject* a_candidate,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		return a_candidate && (a_trustedActorSkeletonNodes.empty() || a_trustedActorSkeletonNodes.contains(a_candidate));
	}

	void AddActorSkeletonLookupNode(ActorSkeletonLookup& a_lookup, RE::NiAVObject* a_object)
	{
		if (!IsProbablyValidNiObject(a_object)) {
			return;
		}

		auto* node = a_object->IsNode();
		if (!IsProbablyValidNiObject(node)) {
			return;
		}

		const auto name = node->GetName();
		if (name.empty()) {
			return;
		}
		if (Smp::IsAutoRenamedPhysicsName(name)) {
			return;
		}

		const auto key = NormalizeActorLookupName(name);
		if (!key.empty() && !a_lookup.nodesByName.contains(key)) {
			a_lookup.nodesByName.emplace(key, node);
		}
	}

	void CollectActorSkeletonLookupNodes(
		ActorSkeletonLookup& a_lookup,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_excludedRootA,
		RE::NiAVObject* a_excludedRootB,
		RE::NiAVObject* a_excludedRootC,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes)
	{
		if (!a_object ||
			IsExcludedMergeSearchObject(a_object, a_excludedObjects) ||
			a_knownArmorNodes.contains(a_object)) {
			return;
		}

		auto* node = a_object->IsNode();
		if (!IsProbablyValidNiObject(node)) {
			return;
		}
		const auto name = node->GetName();
		if (!name.empty() && Smp::IsAutoRenamedPhysicsName(name)) {
			return;
		}

		AddActorSkeletonLookupNode(a_lookup, node);

		for (auto& child : node->children) {
			CollectActorSkeletonLookupNodes(a_lookup, child.get(), a_excludedRootA, a_excludedRootB, a_excludedRootC, a_excludedObjects, a_knownArmorNodes);
		}
	}

	ActorSkeletonLookup BuildActorSkeletonLookup(
		RE::NiAVObject* a_actorRoot,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		ActorSkeletonLookup lookup;
		lookup.trustedNodes = a_trustedActorSkeletonNodes;
		if (!lookup.trustedNodes.empty()) {
			lookup.nodesByName.reserve(lookup.trustedNodes.size());
			for (auto* object : lookup.trustedNodes) {
				if (IsProbablyValidNiObject(object) && !a_knownArmorNodes.contains(object)) {
					AddActorSkeletonLookupNode(lookup, object);
				}
			}
			return lookup;
		}

		CollectActorSkeletonLookupNodes(lookup, a_actorRoot, nullptr, nullptr, nullptr, a_excludedObjects, a_knownArmorNodes);
		return lookup;
	}

	RE::NiNode* FindActorSkeletonLookupNode(const ActorSkeletonLookup& a_lookup, const std::string_view a_name)
	{
		if (a_name.empty()) {
			return nullptr;
		}

		const auto found = a_lookup.nodesByName.find(NormalizeActorLookupName(a_name));
		return found != a_lookup.nodesByName.end() ? found->second : nullptr;
	}

	const Smp::MergeParentBinding* FindMergeParentBinding(
		const std::vector<Smp::MergeParentBinding>& a_bindings,
		const std::string_view a_sourceName)
	{
		if (a_sourceName.empty()) {
			return nullptr;
		}

		const auto found = std::ranges::find_if(a_bindings, [a_sourceName](const Smp::MergeParentBinding& a_binding) {
			return Smp::PhysicsNamesEqual(a_binding.sourceName, a_sourceName);
		});
		return found == a_bindings.end() ? nullptr : std::addressof(*found);
	}

	RE::NiNode* FindNodeByNameExcludingKnownNodes(
		RE::NiAVObject* a_root,
		std::string_view a_name,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes);

	RE::NiNode* FindTrustedBoundActorParent(
		RE::NiAVObject* a_actorRoot,
		const ActorSkeletonLookup& a_actorSkeletonLookup,
		const std::vector<RE::NiAVObject*>& a_actorSkeletonSearchExclusions,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes,
		const Smp::MergeParentBinding& a_binding)
	{
		if (a_binding.parentName.empty()) {
			return nullptr;
		}

		auto* parent = FindActorSkeletonLookupNode(a_actorSkeletonLookup, a_binding.parentName);
		if (!parent) {
			parent = FindNodeByNameExcludingKnownNodes(
				a_actorRoot,
				a_binding.parentName,
				a_actorSkeletonSearchExclusions,
				a_knownArmorNodes);
		}
		if (!IsTrustedActorSkeletonCandidate(parent, a_trustedActorSkeletonNodes)) {
			return nullptr;
		}
		return parent;
	}

	RE::NiNode* FindNodeByNameExcludingKnownNodes(
		RE::NiAVObject* a_root,
		const std::string_view a_name,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes)
	{
		if (!a_root ||
			a_name.empty() ||
			IsExcludedMergeSearchObject(a_root, a_excludedObjects) ||
			a_knownArmorNodes.contains(a_root)) {
			return nullptr;
		}

		if (const auto name = a_root->GetName(); !name.empty() && Smp::PhysicsNamesEqual(name, a_name)) {
			return a_root->IsNode();
		}

		auto* node = a_root->IsNode();
		if (!node) {
			return nullptr;
		}

		for (auto& child : node->children) {
			if (auto* found = FindNodeByNameExcludingKnownNodes(child.get(), a_name, a_excludedObjects, a_knownArmorNodes)) {
				return found;
			}
		}

		return nullptr;
	}

	bool StartsWithInsensitive(const std::string_view a_value, const std::string_view a_prefix)
	{
		return a_value.size() >= a_prefix.size() &&
			_memicmp(a_value.data(), a_prefix.data(), a_prefix.size()) == 0;
	}

	bool IsArmorAutoRenameNode(RE::NiAVObject* a_object)
	{
		if (!a_object) {
			return false;
		}

		const auto name = std::string_view(a_object->GetName());
		constexpr std::string_view prefix = "hdtSSEPhysics_AutoRename_Armor_";
		return StartsWithInsensitive(name, prefix);
	}

	bool IsTrackedMergedObject(const std::vector<RE::NiAVObject*>& a_trackedObjects, RE::NiAVObject* a_object)
	{
		return std::ranges::find(a_trackedObjects, a_object) != a_trackedObjects.end();
	}

	void CollectStaleArmorMergedNodes(
		RE::NiNode* a_parent,
		const std::vector<RE::NiAVObject*>& a_trackedObjects,
		std::vector<std::pair<RE::NiNode*, RE::NiAVObject*>>& a_result)
	{
		if (!a_parent) {
			return;
		}

		for (auto& child : a_parent->children) {
			auto* object = child.get();
			if (!object) {
				continue;
			}

			if (IsArmorAutoRenameNode(object)) {
				if (!IsTrackedMergedObject(a_trackedObjects, object)) {
					a_result.push_back({ a_parent, object });
				}
				continue;
			}

			if (auto* node = object->IsNode()) {
				CollectStaleArmorMergedNodes(node, a_trackedObjects, a_result);
			}
		}
	}

	std::uint32_t DetachStaleArmorMergedNodes(
		RE::NiAVObject* a_actorRoot,
		const std::vector<RE::NiAVObject*>& a_trackedObjects,
		RE::Actor* a_actor,
		const std::string_view a_reason)
	{
		auto* rootNode = a_actorRoot ? a_actorRoot->IsNode() : nullptr;
		if (!rootNode) {
			return 0;
		}

		std::vector<std::pair<RE::NiNode*, RE::NiAVObject*>> staleMergedNodes;
		CollectStaleArmorMergedNodes(rootNode, a_trackedObjects, staleMergedNodes);
		for (const auto& [parent, object] : staleMergedNodes) {
			if (!parent || !object) {
				continue;
			}
			spdlog::debug(
				"pruning stale armor merge node '{}'={} parent={} reason={} actor={}",
				std::string_view(object->GetName()),
				static_cast<void*>(object),
				static_cast<void*>(parent),
				a_reason,
				static_cast<void*>(a_actor));
			parent->DetachChild(object);
		}
		return static_cast<std::uint32_t>(staleMergedNodes.size());
	}

	const Smp::PhysicsBoneDescriptor* FindBoneDescriptor(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name)
	{
		const auto found = std::ranges::find_if(a_summary.boneDescriptors, [a_name](const Smp::PhysicsBoneDescriptor& a_descriptor) {
			return Smp::PhysicsNamesEqual(a_descriptor.name, a_name);
		});
		return found == a_summary.boneDescriptors.end() ? nullptr : std::addressof(*found);
	}

	template <class Body>
	void AddPrototypeBodyBuildGroup(
		Body& a_body,
		const std::uint64_t a_buildGroup,
		const Smp::PrototypeBuildDomain a_domain,
		const RE::BIPED_OBJECT a_bipedObject)
	{
		if (a_buildGroup == 0) {
			return;
		}
		if (std::ranges::find(a_body.buildGroups, a_buildGroup) == a_body.buildGroups.end()) {
			a_body.buildGroups.push_back(a_buildGroup);
		}
		if (std::ranges::find(a_body.buildGroupDomains, std::pair{ a_buildGroup, a_domain }) == a_body.buildGroupDomains.end()) {
			a_body.buildGroupDomains.push_back({ a_buildGroup, a_domain });
		}
		if (std::ranges::find(a_body.buildGroupBipedObjects, std::pair{ a_buildGroup, a_bipedObject }) == a_body.buildGroupBipedObjects.end()) {
			a_body.buildGroupBipedObjects.push_back({ a_buildGroup, a_bipedObject });
		}
		if (a_body.bipedObject == RE::BIPED_OBJECT::kTotal) {
			a_body.bipedObject = a_bipedObject;
		}
	}

	template <class Body>
	bool PrototypeBodyHasBuildGroup(const Body& a_body, const std::uint64_t a_buildGroup)
	{
		if (a_buildGroup == 0) {
			return false;
		}
		return a_body.buildGroup == a_buildGroup || std::ranges::find(a_body.buildGroups, a_buildGroup) != a_body.buildGroups.end();
	}

	const Smp::PhysicsMeshShapeDescriptor* FindMeshDescriptor(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name, const Smp::DefaultBBP::NameMap& a_meshNameMap)
	{
		const auto found = std::ranges::find_if(a_summary.meshDescriptors, [a_name, &a_meshNameMap](const Smp::PhysicsMeshShapeDescriptor& a_descriptor) {
			return MeshNameMatches(a_descriptor.name, a_name, a_meshNameMap);
		});
		return found == a_summary.meshDescriptors.end() ? nullptr : std::addressof(*found);
	}

	bool IsProbablyValidNiObject(const RE::NiObject* a_object)
	{
		constexpr std::uintptr_t kCanonicalUserSpaceMax = 0x00007FFFFFFFFFFFULL;
		if (!a_object || reinterpret_cast<std::uintptr_t>(a_object) > kCanonicalUserSpaceMax) {
			return false;
		}
		if (!IsReadableMemory(a_object, sizeof(void*))) {
			return false;
		}

		const auto vtable = *reinterpret_cast<void* const* const*>(a_object);
		if (!vtable || reinterpret_cast<std::uintptr_t>(vtable) > kCanonicalUserSpaceMax) {
			return false;
		}
		if (!IsReadableMemory(vtable, sizeof(void*) * 5)) {
			return false;
		}

		const auto asNode = vtable[4];
		return asNode &&
			reinterpret_cast<std::uintptr_t>(asNode) <= kCanonicalUserSpaceMax &&
			IsReadableMemory(asNode, 1);
	}

	void CollectMatchedSkinBones(
		RE::NiAVObject* a_object,
		const std::vector<std::string>& a_boneNames,
		const std::vector<std::string>& a_meshNames,
		std::vector<MatchedSkinBone>& a_result)
	{
		if (!a_object) {
			return;
		}

		if (auto* geometry = a_object->IsGeometry()) {
			if (!geometry->skinInstance) {
				return;
			}

			const auto meshMatched = IsNamedPrototypeMesh(a_meshNames, geometry);
			const auto includeAllSkinBones = !a_meshNames.empty() && meshMatched;
			const auto includeAllWhenUnfiltered = a_boneNames.empty() && a_meshNames.empty();
			if (!includeAllSkinBones && !includeAllWhenUnfiltered && a_boneNames.empty()) {
				return;
			}
			auto* skin = geometry->skinInstance.get();
			if (skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				spdlog::warn("skipping suspicious FO4 skin instance with {} bones on geometry '{}'", skin->bones.size(), geometry->GetName());
				return;
			}
			if (!skin->worldTransforms.empty() && skin->worldTransforms.size() != skin->bones.size()) {
				spdlog::warn("FO4 skin instance on geometry '{}' has {} bones but {} world transforms", geometry->GetName(), skin->bones.size(), skin->worldTransforms.size());
			}
			for (std::uint32_t index = 0; index < skin->bones.size(); ++index) {
				auto* boneObject = skin->bones[index];
				if (!boneObject) {
					continue;
				}
				if (!IsProbablyValidNiObject(boneObject)) {
					spdlog::warn(
						"skipping invalid skin bone pointer={} slot={} on geometry '{}'",
						static_cast<void*>(boneObject),
						index,
						geometry->GetName());
					continue;
				}

				auto* bone = boneObject->IsNode();
				if (!bone) {
					spdlog::warn("skipping non-node skin bone '{}' on geometry '{}'", boneObject->GetName(), geometry->GetName());
					continue;
				}

				const auto name = boneObject->GetName();
				const auto matchedName = Smp::FindMatchingPhysicsName(a_boneNames, name);
				if (!name.empty() && (includeAllSkinBones || includeAllWhenUnfiltered || matchedName)) {
					if (auto* existing = FindMatchedSkinBone(a_result, bone)) {
						AddSkinWorldTransformSlot(*existing, skin, index);
						continue;
					}

					auto& matchedBone = a_result.emplace_back(MatchedSkinBone{
						.node = bone,
						.sourceNode = bone,
						.name = matchedName ? std::string(*matchedName) : std::string(name),
					});
					AddSkinWorldTransformSlot(matchedBone, skin, index);
				}
			}
			return;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			CollectMatchedSkinBones(child.get(), a_boneNames, a_meshNames, a_result);
		}
	}

	void UpdateNodeWorldFromLocal(RE::NiNode* a_node)
	{
		if (!a_node) {
			return;
		}

		if (a_node->parent) {
			a_node->world = a_node->parent->world * a_node->local;
		} else {
			a_node->world = a_node->local;
		}

		for (auto& child : a_node->children) {
			if (auto* childNode = child ? child->IsNode() : nullptr) {
				UpdateNodeWorldFromLocal(childNode);
			}
		}
	}

	void UpdateTransformUpDown(RE::NiAVObject* a_object, const bool a_dirty)
	{
		if (!a_object) {
			return;
		}

		RE::NiUpdateData updateData;
		updateData.flags = a_dirty ? 1U : 0U;
		a_object->UpdateWorldData(std::addressof(updateData));

		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			if (child) {
				UpdateTransformUpDown(child.get(), a_dirty);
			}
		}
	}

	bool IsReadableMemory(const void* a_address, const std::size_t a_minSize = 1)
	{
		if (!a_address || a_minSize == 0) {
			return false;
		}

		MEMORY_BASIC_INFORMATION info{};
		if (VirtualQuery(a_address, std::addressof(info), sizeof(info)) == 0) {
			return false;
		}

		if (info.State != MEM_COMMIT || (info.Protect & PAGE_GUARD) != 0 || (info.Protect & PAGE_NOACCESS) != 0) {
			return false;
		}

		const auto base = reinterpret_cast<std::uintptr_t>(info.BaseAddress);
		const auto address = reinterpret_cast<std::uintptr_t>(a_address);
		const auto offset = address >= base ? address - base : 0;
		return offset < info.RegionSize && a_minSize <= (info.RegionSize - offset);
	}

	const char* HkStringPtrData(const Fo4HkStringPtr& a_string)
	{
		const auto pointer = a_string.stringAndFlag & ~static_cast<std::uintptr_t>(1);
		auto* data = pointer != 0 ? reinterpret_cast<const char*>(pointer) : nullptr;
		if (!IsReadableMemory(data)) {
			return nullptr;
		}
		return strnlen_s(data, 256) > 0 ? data : nullptr;
	}

	RE::NiTransform ToNiTransform(const RE::hkQsTransformf& a_transform)
	{
		const auto* rotation = a_transform.rotation.vec.quad.m128_f32;
		const float qx = rotation[0];
		const float qy = rotation[1];
		const float qz = rotation[2];
		const float qw = rotation[3];
		const float sqx = qx * qx;
		const float sqy = qy * qy;
		const float sqz = qz * qz;
		const float sqw = qw * qw;
		const float lengthSquared = sqx + sqy + sqz + sqw;

		RE::NiTransform result;
		if (lengthSquared <= FLT_EPSILON) {
			result.rotate.MakeIdentity();
		} else {
			const float invLengthSquared = 1.0F / lengthSquared;
			result.rotate.entry[0][0] = (sqx - sqy - sqz + sqw) * invLengthSquared;
			result.rotate.entry[1][1] = (-sqx + sqy - sqz + sqw) * invLengthSquared;
			result.rotate.entry[2][2] = (-sqx - sqy + sqz + sqw) * invLengthSquared;

			const auto setCross = [&](const float a, const float b, const float c, const float d, const int i, const int j) {
				result.rotate.entry[i][j] = 2.0F * (a * b + c * d) * invLengthSquared;
				result.rotate.entry[j][i] = 2.0F * (a * b - c * d) * invLengthSquared;
			};
			setCross(qx, qy, qz, qw, 1, 0);
			setCross(qx, qz, qy, qw, 0, 2);
			setCross(qy, qz, qx, qw, 2, 1);
		}

		const auto* translation = a_transform.translation.quad.m128_f32;
		result.translate.x = translation[0];
		result.translate.y = translation[1];
		result.translate.z = translation[2];
		result.scale = std::max(a_transform.scale.quad.m128_f32[0], FLT_EPSILON);
		return result;
	}

	const Fo4HkaSkeleton* GetHavokReferenceSkeleton(RE::Actor* a_actor)
	{
		if (!a_actor) {
			return nullptr;
		}

		RE::BSTSmartPointer<RE::BSAnimationGraphManager> graphManager;
		if (!a_actor->GetAnimationGraphManagerImpl(graphManager) || !graphManager || graphManager->graph.empty()) {
			return nullptr;
		}

		auto* graph = graphManager->graph[0].get();
		if (!graph) {
			return nullptr;
		}

		const auto* fo4Graph = reinterpret_cast<const Fo4BShkbAnimationGraph*>(graph);
		const auto* setup = fo4Graph->characterInstance.setup.ptr;
		return setup ? setup->animationSkeleton.ptr : nullptr;
	}

	bool HasSavedLocalPose(const std::vector<SavedNodeLocalPose>& a_savedPoses, RE::NiNode* a_node)
	{
		return std::ranges::any_of(a_savedPoses, [a_node](const SavedNodeLocalPose& a_entry) {
			return a_entry.node == a_node;
		});
	}

	bool ApplyHavokReferencePose(
		RE::Actor* a_actor,
		RE::NiNode* a_root,
		const std::vector<RE::NiAVObject*>& a_actorSkeletonSearchExclusions,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes,
		std::vector<SavedNodeLocalPose>& a_savedPoses)
	{
		const auto* skeleton = GetHavokReferenceSkeleton(a_actor);
		if (!skeleton || !a_root || skeleton->bones.size <= 0 || skeleton->referencePose.size <= 0) {
			return false;
		}

		const auto count = std::min(skeleton->bones.size, skeleton->referencePose.size);
		std::uint32_t applied = 0;
		a_savedPoses.reserve(a_savedPoses.size() + static_cast<std::size_t>(count));
		for (std::int32_t index = 0; index < count; ++index) {
			const auto* boneName = HkStringPtrData(skeleton->bones.data[index].name);
			if (!boneName || *boneName == '\0') {
				continue;
			}

			auto* boneNode = FindNodeByNameExcludingKnownNodes(
				a_root,
				boneName,
				a_actorSkeletonSearchExclusions,
				a_knownArmorNodes);
			if (!IsTrustedActorSkeletonCandidate(boneNode, a_trustedActorSkeletonNodes)) {
				continue;
			}

			if (!HasSavedLocalPose(a_savedPoses, boneNode)) {
				a_savedPoses.push_back({
					.node = boneNode,
					.local = boneNode->local,
				});
			}
			boneNode->local = ToNiTransform(skeleton->referencePose.data[index]);
			++applied;
		}

		if (applied > 0) {
			UpdateTransformUpDown(a_root, true);
			spdlog::debug(
				"applied Havok reference pose for prototype build actor={} root={} bones={} matched={}",
				static_cast<void*>(a_actor),
				static_cast<void*>(a_root),
				count,
				applied);
			return true;
		}

		return false;
	}

	void RestoreSavedLocalPoses(std::vector<SavedNodeLocalPose>& a_savedPoses, RE::NiAVObject* a_updateRoot)
	{
		if (a_savedPoses.empty()) {
			return;
		}
		for (auto& saved : a_savedPoses) {
			if (saved.node) {
				saved.node->local = saved.local;
			}
		}
		a_savedPoses.clear();
		UpdateTransformUpDown(a_updateRoot, true);
	}

	void RestoreClonedNodeTreeLocalPose(RE::NiNode* a_clone, RE::NiNode* a_source)
	{
		if (!a_clone || !a_source) {
			return;
		}

		a_clone->local = a_source->local;
		for (decltype(a_clone->children.size()) index = 0; index < a_clone->children.size() && index < a_source->children.size(); ++index) {
			auto* cloneChild = a_clone->children[index] ? a_clone->children[index]->IsNode() : nullptr;
			auto* sourceChild = a_source->children[index] ? a_source->children[index]->IsNode() : nullptr;
			if (cloneChild && sourceChild) {
				RestoreClonedNodeTreeLocalPose(cloneChild, sourceChild);
			}
		}
	}

	RE::NiPointer<RE::NiAVObject> CloneNodeExact(RE::NiNode* a_source)
	{
		if (!a_source) {
			return nullptr;
		}

		RE::NiCloningProcess cloneProcess;
		cloneProcess.appendChar = '$';
		cloneProcess.copyType = RE::NiCloningProcess::CopyType::kCopyExact;
		cloneProcess.scale = { 1.0F, 1.0F, 1.0F };

		auto* cloneObject = a_source->CreateClone(cloneProcess);
		a_source->ProcessClone(cloneProcess);
		auto* cloneNode = cloneObject ? cloneObject->IsNode() : nullptr;
		RestoreClonedNodeTreeLocalPose(cloneNode, a_source);
		return cloneNode ? static_cast<RE::NiAVObject*>(cloneNode) : nullptr;
	}

	void RenameMergedNodeTree(
		RE::NiNode* a_clone,
		RE::NiNode* a_source,
		const std::string& a_prefix,
		std::vector<MergedSkeletonNode>& a_renamedNodes)
	{
		if (!a_clone || !a_source) {
			return;
		}

		const auto originalName = a_source->GetName();
		if (!originalName.empty()) {
			auto renamed = a_prefix;
			renamed += std::string_view(originalName);
			a_renamedNodes.push_back({
				.originalName = std::string(originalName),
				.renamedName = renamed,
				.node = a_clone,
			});
			a_clone->name = renamed.c_str();
		}

		for (decltype(a_clone->children.size()) index = 0; index < a_clone->children.size() && index < a_source->children.size(); ++index) {
			auto* cloneChild = a_clone->children[index] ? a_clone->children[index]->IsNode() : nullptr;
			auto* sourceChild = a_source->children[index] ? a_source->children[index]->IsNode() : nullptr;
			if (cloneChild && sourceChild) {
				RenameMergedNodeTree(cloneChild, sourceChild, a_prefix, a_renamedNodes);
			}
		}
	}

	RE::NiNode* CloneMergedNodeTree(RE::NiNode* a_source, const std::string& a_prefix, std::vector<MergedSkeletonNode>& a_renamedNodes)
	{
		if (!a_source) {
			return nullptr;
		}

		RE::NiCloningProcess cloneProcess;
		cloneProcess.appendChar = '$';
		cloneProcess.copyType = RE::NiCloningProcess::CopyType::kCopyExact;
		cloneProcess.scale = { 1.0F, 1.0F, 1.0F };

		auto* cloneObject = a_source->CreateClone(cloneProcess);
		a_source->ProcessClone(cloneProcess);
		auto* cloneNode = cloneObject ? cloneObject->IsNode() : nullptr;
		if (!cloneNode) {
			return nullptr;
		}

		RestoreClonedNodeTreeLocalPose(cloneNode, a_source);
		RenameMergedNodeTree(cloneNode, a_source, a_prefix, a_renamedNodes);
		return cloneNode;
	}

	std::optional<float> FindXmlBoneMass(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name)
	{
		if (const auto* descriptor = FindBoneDescriptor(a_summary, a_name)) {
			return std::max(descriptor->mass, 0.0F);
		}
		if (a_summary.defaultBoneDescriptor) {
			return std::max(a_summary.defaultBoneDescriptor->mass, 0.0F);
		}
		return std::nullopt;
	}

	bool IsExplicitXmlBone(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name)
	{
		return FindBoneDescriptor(a_summary, a_name) != nullptr;
	}

	bool IsExplicitDynamicXmlBone(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name)
	{
		const auto* descriptor = FindBoneDescriptor(a_summary, a_name);
		return descriptor && descriptor->mass > 0.0F;
	}

	bool IsReferencedXmlBoneName(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name)
	{
		return !a_name.empty() &&
			std::ranges::find_if(a_summary.boneNames, [a_name](const std::string& a_boneName) {
				return Smp::PhysicsNamesEqual(a_boneName, a_name);
			}) != a_summary.boneNames.end();
	}

	bool IsDynamicXmlBone(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name)
	{
		if (a_name.empty()) {
			return false;
		}
		if (const auto* descriptor = FindBoneDescriptor(a_summary, a_name)) {
			return descriptor->mass > 0.0F;
		}
		return a_summary.defaultBoneDescriptor && a_summary.defaultBoneDescriptor->mass > 0.0F &&
			std::ranges::find_if(a_summary.boneNames, [a_name](const std::string& a_boneName) {
				return Smp::PhysicsNamesEqual(a_boneName, a_name);
			}) != a_summary.boneNames.end();
	}

	std::uint32_t CountDynamicDecodedSkinBones(
		const Smp::Fo4MeshExtractionResult& a_extraction,
		const Smp::PhysicsXmlSummary& a_summary)
	{
		std::uint32_t count = 0;
		for (const auto& mesh : a_extraction.meshes) {
			for (const auto& bone : mesh.bones) {
				if (!bone.name.empty() && IsDynamicXmlBone(a_summary, bone.name)) {
					++count;
				}
			}
		}
		return count;
	}

	bool HasDynamicXmlDescendant(RE::NiNode* a_node, const Smp::PhysicsXmlSummary& a_summary)
	{
		if (!a_node) {
			return false;
		}

		for (auto& child : a_node->children) {
			auto* childNode = child ? child->IsNode() : nullptr;
			if (!childNode) {
				continue;
			}

			const auto childName = childNode->GetName();
			if (!childName.empty()) {
				if (IsDynamicXmlBone(a_summary, childName)) {
					return true;
				}
			}

			if (HasDynamicXmlDescendant(childNode, a_summary)) {
				return true;
			}
		}

		return false;
	}

	bool HasExplicitXmlDescendant(RE::NiNode* a_node, const Smp::PhysicsXmlSummary& a_summary)
	{
		if (!a_node) {
			return false;
		}

		for (auto& child : a_node->children) {
			auto* childNode = child ? child->IsNode() : nullptr;
			if (!childNode) {
				continue;
			}

			const auto childName = childNode->GetName();
			if (!childName.empty() && IsExplicitXmlBone(a_summary, childName)) {
				return true;
			}
			if (HasExplicitXmlDescendant(childNode, a_summary)) {
				return true;
			}
		}

		return false;
	}

	bool HasRelevantXmlDescendant(RE::NiNode* a_node, const Smp::PhysicsXmlSummary& a_summary)
	{
		if (!a_node) {
			return false;
		}

		for (auto& child : a_node->children) {
			auto* childNode = child ? child->IsNode() : nullptr;
			if (!childNode) {
				continue;
			}

			const auto childName = childNode->GetName();
			if (!childName.empty() && IsReferencedXmlBoneName(a_summary, childName)) {
				return true;
			}
			if (HasRelevantXmlDescendant(childNode, a_summary)) {
				return true;
			}
		}

		return HasDynamicXmlDescendant(a_node, a_summary) || HasExplicitXmlDescendant(a_node, a_summary);
	}

	bool IsIgnoredSourceSkeletonNodeName(const std::string_view a_name)
	{
		return a_name.empty() ||
			Smp::PhysicsNamesEqual(a_name, "BSFaceGenNiNodeSkinned") ||
			StartsWithInsensitive(a_name, "VHW");
	}

	bool IsReferencedXmlSourceBone(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name)
	{
		return IsReferencedXmlBoneName(a_summary, a_name) ||
			IsExplicitXmlBone(a_summary, a_name) ||
			IsDynamicXmlBone(a_summary, a_name);
	}

	RE::NiNode* FindTrustedActorSkeletonNodeForSource(
		RE::NiAVObject* a_actorRoot,
		RE::NiNode* a_sourceNode,
		const ActorSkeletonLookup& a_actorSkeletonLookup,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		if (!a_actorRoot || !a_sourceNode) {
			return nullptr;
		}

		const auto sourceName = a_sourceNode->GetName();
		auto* actorNode = FindActorSkeletonLookupNode(a_actorSkeletonLookup, sourceName);
		if (!actorNode) {
			actorNode = FindNodeByNameExcludingKnownNodes(
				a_actorRoot,
				sourceName,
				a_excludedObjects,
				a_knownArmorNodes);
		}
		if (!actorNode) {
			return nullptr;
		}
		if (!IsTrustedActorSkeletonCandidate(actorNode, a_trustedActorSkeletonNodes)) {
			spdlog::debug(
				"trusted actor skeleton lookup for '{}' rejected actorNode={} because it was not present in the pre-attach trusted actor skeleton set",
				sourceName,
				static_cast<void*>(actorNode));
			return nullptr;
		}
		return actorNode;
	}

	RE::NiNode* FindTrustedActorSkeletonNodeForSourceName(
		RE::NiAVObject* a_actorRoot,
		const std::string_view a_name,
		const ActorSkeletonLookup& a_actorSkeletonLookup,
		RE::NiNode* a_sourceRoot,
		RE::NiAVObject* a_excludedRootA,
		RE::NiAVObject* a_excludedRootB,
		RE::NiAVObject* a_excludedRootC,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		(void)a_sourceRoot;
		(void)a_excludedRootA;
		(void)a_excludedRootB;
		(void)a_excludedRootC;
		auto* actorNode = FindActorSkeletonLookupNode(a_actorSkeletonLookup, a_name);
		if (!actorNode) {
			actorNode = FindNodeByNameExcludingKnownNodes(
				a_actorRoot,
				a_name,
				a_excludedObjects,
				a_knownArmorNodes);
		}
		if (!actorNode) {
			return nullptr;
		}
		if (!IsTrustedActorSkeletonCandidate(actorNode, a_trustedActorSkeletonNodes)) {
			spdlog::debug(
				"trusted actor skeleton lookup for '{}' rejected actorNode={} because it was not present in the pre-attach trusted actor skeleton set",
				a_name,
				static_cast<void*>(actorNode));
			return nullptr;
		}
		return actorNode;
	}

	SourceBoneOwnership ClassifySourceBoneOwnership(
		RE::NiAVObject* a_actorRoot,
		RE::NiNode* a_sourceNode,
		const ActorSkeletonLookup& a_actorSkeletonLookup,
		const std::vector<RE::NiAVObject*>& a_actorSkeletonSearchExclusions,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes,
		const std::vector<Smp::MergeParentBinding>& a_mergeParentBindings)
	{
		if (!a_sourceNode) {
			return SourceBoneOwnership::kIgnored;
		}

		const auto sourceName = a_sourceNode->GetName();
		if (IsIgnoredSourceSkeletonNodeName(sourceName)) {
			return SourceBoneOwnership::kIgnored;
		}

		auto* actorNode = FindTrustedActorSkeletonNodeForSource(
				a_actorRoot,
				a_sourceNode,
				a_actorSkeletonLookup,
				a_actorSkeletonSearchExclusions,
				a_knownArmorNodes,
				a_trustedActorSkeletonNodes);
		if (actorNode) {
			const auto storedParent = std::ranges::find_if(a_mergeParentBindings, [sourceName](const Smp::MergeParentBinding& a_binding) {
				return Smp::PhysicsNamesEqual(a_binding.sourceName, sourceName);
			});
			if (storedParent != a_mergeParentBindings.end() && !storedParent->parentName.empty()) {
				auto* expectedParent = FindActorSkeletonLookupNode(a_actorSkeletonLookup, storedParent->parentName);
				if (!expectedParent) {
					expectedParent = FindNodeByNameExcludingKnownNodes(
						a_actorRoot,
						storedParent->parentName,
						a_actorSkeletonSearchExclusions,
						a_knownArmorNodes);
				}
				if (expectedParent &&
					IsTrustedActorSkeletonCandidate(expectedParent, a_trustedActorSkeletonNodes) &&
					actorNode != expectedParent &&
					!IsObjectDescendantOf(actorNode, expectedParent)) {
					spdlog::debug(
						"trusted actor skeleton lookup for '{}' rejected actorNode={} parent={} parentName='{}' because stored source parent binding expects descendant of {} parentName='{}'",
						sourceName,
						static_cast<void*>(actorNode),
						static_cast<void*>(actorNode->parent),
						actorNode->parent ? std::string_view(actorNode->parent->GetName()) : std::string_view{},
						static_cast<void*>(expectedParent),
						std::string_view(expectedParent->GetName()));
					return SourceBoneOwnership::kArmorOwned;
				}
			}
			return SourceBoneOwnership::kActorSkeleton;
		}

		return SourceBoneOwnership::kArmorOwned;
	}

	void CloneSourceSkeletonIntoPartTree(
		RE::NiNode* a_cloneParent,
		RE::NiNode* a_source,
		RE::NiAVObject* a_actorRoot,
		const ActorSkeletonLookup& a_actorSkeletonLookup,
		const std::vector<RE::NiAVObject*>& a_actorSkeletonSearchExclusions,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes,
		const std::string& a_prefix,
		const Smp::PhysicsXmlSummary& a_summary,
		const std::vector<Smp::MergeParentBinding>& a_mergeParentBindings,
		std::vector<MergedSkeletonNode>& a_renamedNodes,
		std::vector<MergedRootNode>& a_mergedRoots)
	{
		if (!a_cloneParent || !a_source) {
			return;
		}

		for (auto& child : a_source->children) {
			auto* sourceChild = child ? child->IsNode() : nullptr;
			if (!sourceChild) {
				continue;
			}

			const auto childName = sourceChild->GetName();
			if (Smp::PhysicsNamesEqual(childName, "BSFaceGenNiNodeSkinned")) {
				continue;
			}
			if (childName.empty()) {
				CloneSourceSkeletonIntoPartTree(a_cloneParent, sourceChild, a_actorRoot, a_actorSkeletonLookup, a_actorSkeletonSearchExclusions, a_knownArmorNodes, a_trustedActorSkeletonNodes, a_prefix, a_summary, a_mergeParentBindings, a_renamedNodes, a_mergedRoots);
				continue;
			}

			if (auto* actorNode = FindTrustedActorSkeletonNodeForSource(
					a_actorRoot,
					sourceChild,
					a_actorSkeletonLookup,
					a_actorSkeletonSearchExclusions,
					a_knownArmorNodes,
					a_trustedActorSkeletonNodes)) {
				if (IsReferencedXmlSourceBone(a_summary, childName) || HasRelevantXmlDescendant(sourceChild, a_summary)) {
					spdlog::debug(
						"armor skeleton merge recursing into actor node '{}' sourceNode={} actorNode={} parent={} parentName='{}'",
						childName,
						static_cast<void*>(sourceChild),
						static_cast<void*>(actorNode),
						static_cast<void*>(actorNode->parent),
						actorNode->parent ? std::string_view(actorNode->parent->GetName()) : std::string_view{});
				}
				CloneSourceSkeletonIntoPartTree(actorNode, sourceChild, a_actorRoot, a_actorSkeletonLookup, a_actorSkeletonSearchExclusions, a_knownArmorNodes, a_trustedActorSkeletonNodes, a_prefix, a_summary, a_mergeParentBindings, a_renamedNodes, a_mergedRoots);
				continue;
			}

			if (!IsReferencedXmlSourceBone(a_summary, childName) && !HasRelevantXmlDescendant(sourceChild, a_summary)) {
				continue;
			}

			auto* clonedNode = CloneMergedNodeTree(sourceChild, a_prefix, a_renamedNodes);
			if (!clonedNode) {
				continue;
			}

			auto* attachParent = a_cloneParent;
			const auto* parentBinding = FindMergeParentBinding(a_mergeParentBindings, childName);
			if (parentBinding) {
				if (auto* boundParent = FindTrustedBoundActorParent(
						a_actorRoot,
						a_actorSkeletonLookup,
						a_actorSkeletonSearchExclusions,
						a_knownArmorNodes,
						a_trustedActorSkeletonNodes,
						*parentBinding)) {
					if (boundParent != clonedNode && !IsObjectDescendantOf(boundParent, clonedNode)) {
						attachParent = boundParent;
					}
				}
			}

			attachParent->AttachChild(clonedNode, false);
			if (parentBinding && parentBinding->hasLocalToParent && attachParent != a_cloneParent) {
				clonedNode->local = parentBinding->localToParent;
			}
			UpdateNodeWorldFromLocal(clonedNode);
			a_mergedRoots.push_back({
				.parent = attachParent,
				.node = clonedNode,
				.sourceNode = sourceChild,
				.originalName = std::string(childName),
				.recordParentName = parentBinding && !parentBinding->parentName.empty() ? parentBinding->parentName : (attachParent->GetName().empty() ? std::string{} : std::string(attachParent->GetName())),
				.localToParent = clonedNode->local,
				.recordLocalToParent = parentBinding && parentBinding->hasLocalToParent ? parentBinding->localToParent : clonedNode->local,
				.hasLocalToParent = true,
				.hasRecordLocalToParent = true,
				.recordMergeParentBinding = parentBinding && !parentBinding->parentName.empty(),
			});
			spdlog::debug(
				"cloned armor skeleton subtree '{}' as plugin-owned prefixed node='{}' node={} under merge parent={} parentName='{}' recordParent='{}' prefix='{}'",
				childName,
				std::string_view(clonedNode->GetName()),
				static_cast<void*>(clonedNode),
				static_cast<void*>(attachParent),
				std::string_view(attachParent->GetName()),
				parentBinding ? std::string_view(parentBinding->parentName) : std::string_view{},
				a_prefix);
		}
	}

	RE::NiNode* FindCurrentRenameMapSourceNode(
		RE::NiAVObject* a_root,
		const std::string_view a_name,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		if (!a_root || a_name.empty()) {
			return nullptr;
		}

		if (const auto name = a_root->GetName(); !name.empty() && Smp::PhysicsNamesEqual(name, a_name)) {
			auto* node = a_root->IsNode();
			if (node && (a_trustedActorSkeletonNodes.empty() || !a_trustedActorSkeletonNodes.contains(node))) {
				return node;
			}
		}

		auto* rootNode = a_root->IsNode();
		if (!rootNode) {
			return nullptr;
		}

		for (auto& child : rootNode->children) {
			if (auto* found = FindCurrentRenameMapSourceNode(child.get(), a_name, a_trustedActorSkeletonNodes)) {
				return found;
			}
		}

		return nullptr;
	}

	void RegisterMergedRenameMapNodes(
		RE::NiAVObject* a_actorRoot,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes,
		const std::vector<Smp::MergeRename>& a_mergeRenameMap,
		const std::vector<Smp::MergeParentBinding>& a_mergeParentBindings,
		std::vector<MergedSkeletonNode>& a_renamedNodes,
		std::vector<MergedRootNode>& a_mergedRoots)
	{
		if (!a_actorRoot || a_mergeRenameMap.empty()) {
			return;
		}

		auto findParentBinding = [&a_mergeParentBindings](const std::string_view a_sourceName) -> const Smp::MergeParentBinding* {
			if (a_sourceName.empty()) {
				return nullptr;
			}

			const auto found = std::ranges::find_if(a_mergeParentBindings, [a_sourceName](const Smp::MergeParentBinding& a_binding) {
				return Smp::PhysicsNamesEqual(a_binding.sourceName, a_sourceName);
			});
			return found == a_mergeParentBindings.end() ? nullptr : std::addressof(*found);
		};
		auto findOriginalNameFromRenamedName = [&a_mergeRenameMap](const std::string_view a_renamedName) -> std::string_view {
			if (a_renamedName.empty()) {
				return {};
			}

			const auto found = std::ranges::find_if(a_mergeRenameMap, [a_renamedName](const Smp::MergeRename& a_entry) {
				return Smp::PhysicsNamesEqual(a_entry.renamedName, a_renamedName);
			});
			return found != a_mergeRenameMap.end() ? std::string_view(found->sourceName) : std::string_view{};
		};

		std::unordered_set<std::string> registeredNames;
		for (const auto& entry : a_mergeRenameMap) {
			if (entry.sourceName.empty() || entry.renamedName.empty()) {
				continue;
			}
			if (StartsWithInsensitive(entry.sourceName, "VHW")) {
				spdlog::debug(
					"skipping merged armor rename map source='{}' renamed='{}' because it is a Classic Holstered Weapon helper bone",
					entry.sourceName,
					entry.renamedName);
				continue;
			}

			const auto sourceKey = NormalizeActorLookupName(entry.sourceName);
			if (sourceKey.empty() || registeredNames.contains(sourceKey)) {
				continue;
			}

			auto* node = FindNodeByName(a_actorRoot, entry.renamedName);
			if (!node) {
				auto* sourceNode = FindCurrentRenameMapSourceNode(
					a_actorRoot,
					entry.sourceName,
					a_trustedActorSkeletonNodes);
				if (!sourceNode) {
					spdlog::debug(
						"skipping merged armor rename map source='{}' renamed='{}' because renamed/source nodes were not found under actor root={}",
						entry.sourceName,
						entry.renamedName,
						static_cast<void*>(a_actorRoot));
					continue;
				}

				sourceNode->name = entry.renamedName.c_str();
				node = sourceNode;
				spdlog::debug(
					"re-applied pre-merged armor rename source='{}' renamed='{}' node={} parent={} parentName='{}'",
					entry.sourceName,
					entry.renamedName,
					static_cast<void*>(node),
					static_cast<void*>(node->parent),
					node->parent ? std::string_view(node->parent->GetName()) : std::string_view{});
			}

			a_renamedNodes.push_back({
				.originalName = entry.sourceName,
				.renamedName = entry.renamedName,
				.node = node,
			});

			const auto* parentBinding = findParentBinding(entry.sourceName);
			const auto hasRecordBinding = parentBinding && !parentBinding->parentName.empty();
			const auto hasRecordLocal = parentBinding && parentBinding->hasLocalToParent;
			if (hasRecordBinding) {
				auto* expectedParent = FindNodeByName(a_actorRoot, parentBinding->parentName);
				if (expectedParent &&
					expectedParent != node &&
					(a_trustedActorSkeletonNodes.empty() || a_trustedActorSkeletonNodes.contains(expectedParent)) &&
					!IsObjectDescendantOf(expectedParent, node) &&
					(node->parent == expectedParent || !IsObjectDescendantOf(node, expectedParent))) {
					RE::NiPointer<RE::NiAVObject> keepAlive{ node };
					if (node->parent != expectedParent) {
						if (node->parent) {
							node->parent->DetachChild(node);
						}
						expectedParent->AttachChild(node, false);
					}
					if (hasRecordLocal) {
						node->local = parentBinding->localToParent;
					}
					UpdateNodeWorldFromLocal(node);
					spdlog::debug(
						"realigned pre-merged armor node source='{}' renamed='{}' node={} to actor parent={} parentName='{}' local=({:.3f},{:.3f},{:.3f})",
						entry.sourceName,
						entry.renamedName,
						static_cast<void*>(node),
						static_cast<void*>(expectedParent),
						std::string_view(expectedParent->GetName()),
						node->local.translate.x,
						node->local.translate.y,
						node->local.translate.z);
				} else if (expectedParent &&
					hasRecordLocal &&
					node->parent &&
					node->parent != expectedParent &&
					IsObjectDescendantOf(node, expectedParent)) {
					const auto parentOriginalName = findOriginalNameFromRenamedName(node->parent->GetName());
					const auto* parentRecordBinding = findParentBinding(parentOriginalName);
					if (parentRecordBinding &&
						parentRecordBinding->hasLocalToParent &&
						Smp::PhysicsNamesEqual(parentRecordBinding->parentName, parentBinding->parentName)) {
						node->local = parentRecordBinding->localToParent.Invert() * parentBinding->localToParent;
						UpdateNodeWorldFromLocal(node);
						spdlog::debug(
							"restored pre-merged armor descendant reference pose source='{}' renamed='{}' node={} parent={} parentName='{}' recordParent='{}' local=({:.3f},{:.3f},{:.3f})",
							entry.sourceName,
							entry.renamedName,
							static_cast<void*>(node),
							static_cast<void*>(node->parent),
							std::string_view(node->parent->GetName()),
							std::string_view(parentBinding->parentName),
							node->local.translate.x,
							node->local.translate.y,
							node->local.translate.z);
					}
				}
			}
			const auto recordLocal = hasRecordLocal ? parentBinding->localToParent : node->local;
			a_mergedRoots.push_back({
				.parent = node->parent,
				.node = node,
				.sourceNode = nullptr,
				.originalName = entry.sourceName,
				.recordParentName = hasRecordBinding ? parentBinding->parentName : (node->parent ? std::string(node->parent->GetName()) : std::string{}),
				.localToParent = node->local,
				.recordLocalToParent = recordLocal,
				.hasLocalToParent = true,
				.hasRecordLocalToParent = true,
				.recordMergeParentBinding = hasRecordBinding,
			});
			registeredNames.insert(sourceKey);

			spdlog::debug(
				"registered pre-merged armor rename source='{}' renamed='{}' node={} parent={} parentName='{}' recordBinding={} recordParent='{}' local=({:.3f},{:.3f},{:.3f}) recordLocal=({:.3f},{:.3f},{:.3f})",
				entry.sourceName,
				entry.renamedName,
				static_cast<void*>(node),
				static_cast<void*>(node->parent),
				node->parent ? std::string_view(node->parent->GetName()) : std::string_view{},
				hasRecordBinding,
				hasRecordBinding ? std::string_view(parentBinding->parentName) : std::string_view{},
				node->local.translate.x,
				node->local.translate.y,
				node->local.translate.z,
				recordLocal.translate.x,
				recordLocal.translate.y,
				recordLocal.translate.z);
		}
	}

	void RestorePreMergedRenameMapLocalPoseFromSource(
		const std::vector<MergedSkeletonNode>& a_renamedNodes,
		RE::NiAVObject* a_sourceRoot,
		const std::vector<Smp::MergeParentBinding>& a_mergeParentBindings)
	{
		if (!a_sourceRoot || a_renamedNodes.empty()) {
			return;
		}

		auto hasExplicitParentBinding = [&a_mergeParentBindings](const std::string_view a_sourceName) {
			return std::ranges::any_of(a_mergeParentBindings, [a_sourceName](const Smp::MergeParentBinding& a_binding) {
				return !a_binding.parentName.empty() && a_binding.hasLocalToParent && Smp::PhysicsNamesEqual(a_binding.sourceName, a_sourceName);
			});
		};

		std::uint32_t restored = 0;
		for (const auto& entry : a_renamedNodes) {
			if (!entry.node || entry.originalName.empty() || hasExplicitParentBinding(entry.originalName)) {
				continue;
			}

			auto* sourceNode = FindNodeByName(a_sourceRoot, entry.originalName);
			if (!sourceNode) {
				continue;
			}

			const auto previousLocal = entry.node->local;
			entry.node->local = sourceNode->local;
			++restored;
			if (previousLocal.translate != entry.node->local.translate) {
				spdlog::debug(
					"restored pre-merged armor local pose from source source='{}' renamed='{}' node={} local=({:.3f},{:.3f},{:.3f}) previous=({:.3f},{:.3f},{:.3f})",
					entry.originalName,
					entry.renamedName,
					static_cast<void*>(entry.node),
					entry.node->local.translate.x,
					entry.node->local.translate.y,
					entry.node->local.translate.z,
					previousLocal.translate.x,
					previousLocal.translate.y,
					previousLocal.translate.z);
			}
		}

		if (restored > 0) {
			spdlog::debug("restored {} pre-merged armor renamed node local poses from source root={}", restored, static_cast<void*>(a_sourceRoot));
		}
	}

	RE::NiNode* FindCurrentMergedSkeletonNode(
		const std::vector<MergedSkeletonNode>& a_renamedNodes,
		const std::string_view a_name,
		RE::NiAVObject* a_skeletonRoot)
	{
		const auto found = std::ranges::find_if(a_renamedNodes, [a_name](const MergedSkeletonNode& a_entry) {
			return Smp::PhysicsNamesEqual(a_entry.originalName, a_name);
		});
		if (found == a_renamedNodes.end()) {
			return nullptr;
		}

		if (!a_skeletonRoot) {
			return found->node;
		}

		if (auto* currentNode = FindNodeByName(a_skeletonRoot, found->renamedName)) {
			return currentNode;
		}

		return nullptr;
	}

	void DetachMergedRootNodes(std::vector<MergedRootNode>& a_roots)
	{
		for (auto& root : a_roots) {
			if (root.parent && root.node) {
				root.parent->DetachChild(root.node.get());
			}
		}
		a_roots.clear();
	}

	std::vector<RE::NiAVObject*> BuildSkeletonLookupExclusions(
		const std::vector<RE::NiAVObject*>& a_baseExclusions,
		const std::vector<MergedRootNode>& a_mergedRoots)
	{
		auto exclusions = a_baseExclusions;
		exclusions.reserve(exclusions.size() + a_mergedRoots.size());
		for (const auto& root : a_mergedRoots) {
			if (root.node) {
				exclusions.push_back(root.node.get());
			}
		}
		return exclusions;
	}

	std::string NormalizeXmlKey(const std::string_view a_path)
	{
		return Smp::ConfigPaths::LowerString(Smp::ConfigPaths::Trim(std::string(a_path)));
	}

	std::string MakeArmorModelCacheKey(const RE::BIPED_OBJECT a_bipedObject, const std::string_view a_model)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return {};
		}

		auto model = Smp::ConfigPaths::LowerString(Smp::ConfigPaths::Trim(std::string(a_model)));
		if (model.empty()) {
			return {};
		}

		std::ranges::replace(model, '/', '\\');
		return std::to_string(std::to_underlying(a_bipedObject)) + "|" + model;
	}

	std::string MakeArmorModelCacheKey(RE::BIPOBJECT* a_bipObject, const RE::BIPED_OBJECT a_bipedObject)
	{
		if (!a_bipObject || !a_bipObject->part) {
			return {};
		}
		return MakeArmorModelCacheKey(a_bipedObject, a_bipObject->part->GetModel());
	}

	void ResolveExplicitXmlBonesFromMergedSkeleton(
		std::vector<MatchedSkinBone>& a_matchedBones,
		const Smp::PhysicsXmlSummary& a_summary,
		const std::vector<std::string>& a_boneNames,
		const std::vector<MergedSkeletonNode>& a_renamedNodes,
		RE::NiAVObject* a_skeletonRoot,
		const ActorSkeletonLookup& a_actorSkeletonLookup,
		RE::NiNode* a_sourceRoot,
		RE::NiAVObject* a_attachedObject,
		RE::NiAVObject* a_sourceObject,
		RE::NiAVObject* a_mergeSourceObject,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		(void)a_summary;
		if (!a_skeletonRoot) {
			return;
		}

		for (const auto& boneName : a_boneNames) {
			if (boneName.empty()) {
				continue;
			}

			bool resolvedFromMergedNode = false;
			RE::NiNode* skeletonNode = nullptr;
			skeletonNode = FindCurrentMergedSkeletonNode(a_renamedNodes, boneName, a_skeletonRoot);
			resolvedFromMergedNode = skeletonNode != nullptr;
			if (!skeletonNode) {
				skeletonNode = FindTrustedActorSkeletonNodeForSourceName(
					a_skeletonRoot,
					boneName,
					a_actorSkeletonLookup,
					a_sourceRoot,
					a_attachedObject,
					a_sourceObject,
					a_mergeSourceObject,
					a_excludedObjects,
					a_knownArmorNodes,
					a_trustedActorSkeletonNodes);
				resolvedFromMergedNode = false;
			}
			if (!skeletonNode) {
				continue;
			}

			if (auto* existingByNode = FindMatchedSkinBone(a_matchedBones, skeletonNode)) {
				existingByNode->name = boneName;
				existingByNode->resolvedFromSkeleton = true;
				existingByNode->useActorKinematicBody = !resolvedFromMergedNode;
				continue;
			}

			if (auto* existingByName = FindMatchedSkinBoneByName(a_matchedBones, boneName)) {
				if (existingByName->node != skeletonNode) {
					spdlog::debug(
						"explicit XML bone '{}' resolved from {} node={} instead of skinned runtime node={}",
						boneName,
						resolvedFromMergedNode ? "merged attachment" : "actor skeleton",
						static_cast<void*>(skeletonNode),
						static_cast<void*>(existingByName->node));
					if (!existingByName->sourceNode) {
						existingByName->sourceNode = existingByName->node;
					}
					existingByName->node = skeletonNode;
				}
				existingByName->name = boneName;
				existingByName->resolvedFromSkeleton = true;
				existingByName->useActorKinematicBody = !resolvedFromMergedNode;
				continue;
			}

			a_matchedBones.push_back({
				.node = skeletonNode,
				.name = boneName,
				.resolvedFromSkeleton = true,
				.useActorKinematicBody = !resolvedFromMergedNode,
			});
			spdlog::debug(
				"explicit XML bone '{}' resolved from {} node={} nodeName='{}'",
				boneName,
				resolvedFromMergedNode ? "plugin-owned armor clone" : "actor skeleton",
				static_cast<void*>(skeletonNode),
				std::string_view(skeletonNode->GetName()));
		}
	}

	void ResolveMatchedSkinBonesFromSkeleton(
		std::vector<MatchedSkinBone>& a_matchedBones,
		const Smp::PhysicsXmlSummary& a_summary,
		const std::vector<MergedSkeletonNode>& a_renamedNodes,
		RE::NiAVObject* a_skeletonRoot,
		const ActorSkeletonLookup& a_actorSkeletonLookup,
		RE::NiNode* a_sourceRoot,
		RE::NiAVObject* a_attachedObject,
		RE::NiAVObject* a_sourceObject,
		RE::NiAVObject* a_mergeSourceObject,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		(void)a_summary;
		if (!a_skeletonRoot) {
			return;
		}

		for (auto& matchedBone : a_matchedBones) {
			if (matchedBone.name.empty()) {
				continue;
			}

			bool resolvedFromMergedNode = false;
			RE::NiNode* skeletonNode = nullptr;
			skeletonNode = FindCurrentMergedSkeletonNode(a_renamedNodes, matchedBone.name, a_skeletonRoot);
			resolvedFromMergedNode = skeletonNode != nullptr;
			if (!skeletonNode) {
				skeletonNode = FindTrustedActorSkeletonNodeForSourceName(
					a_skeletonRoot,
					matchedBone.name,
					a_actorSkeletonLookup,
					a_sourceRoot,
					a_attachedObject,
					a_sourceObject,
					a_mergeSourceObject,
					a_excludedObjects,
					a_knownArmorNodes,
					a_trustedActorSkeletonNodes);
				resolvedFromMergedNode = false;
			}
			if (!skeletonNode) {
				continue;
			}
			if (skeletonNode == matchedBone.node) {
				matchedBone.resolvedFromSkeleton = true;
				matchedBone.useActorKinematicBody = !resolvedFromMergedNode;
				continue;
			}

			spdlog::debug(
				"matched skin bone '{}' resolved from {} node={} nodeName='{}' instead of attached runtime node={} rawName='{}'",
				matchedBone.name,
				resolvedFromMergedNode ? "plugin-owned armor clone" : "actor skeleton",
				static_cast<void*>(skeletonNode),
				std::string_view(skeletonNode->GetName()),
				static_cast<void*>(matchedBone.node),
				matchedBone.node ? std::string_view(matchedBone.node->GetName()) : std::string_view{});
			if (!matchedBone.sourceNode) {
				matchedBone.sourceNode = matchedBone.node;
			}
			matchedBone.node = skeletonNode;
			matchedBone.resolvedFromSkeleton = true;
			matchedBone.useActorKinematicBody = !resolvedFromMergedNode;
		}
	}

	bool IsUnresolvedArmorOwnedMatchedBone(
		const MatchedSkinBone& a_matchedBone,
		RE::NiAVObject* a_actorRoot,
		RE::NiNode* a_sourceRoot,
		RE::NiAVObject* a_attachedObject,
		RE::NiAVObject* a_sourceObject,
		RE::NiAVObject* a_mergeSourceObject,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes)
	{
		if (!a_matchedBone.node || a_matchedBone.resolvedFromSkeleton) {
			return false;
		}

		auto* object = static_cast<RE::NiAVObject*>(a_matchedBone.node);
		if (a_knownArmorNodes.contains(object)) {
			return true;
		}

		if (IsObjectInTree(a_attachedObject, object) ||
			IsObjectInTree(a_sourceObject, object) ||
			IsObjectInTree(a_mergeSourceObject, object) ||
			IsObjectInTree(a_sourceRoot, object)) {
			return true;
		}

		for (auto* excluded : a_excludedObjects) {
			if (IsObjectInTree(excluded, object)) {
				return true;
			}
		}

		if (a_actorRoot && IsObjectInTree(a_actorRoot, object)) {
			return false;
		}

		return false;
	}

	btTransform ToBulletTransform(const Smp::XmlTransform& a_transform)
	{
		btQuaternion rotation(a_transform.rotation.x, a_transform.rotation.y, a_transform.rotation.z, a_transform.rotation.w);
		if (rotation.length2() <= FLT_EPSILON) {
			rotation = btQuaternion::getIdentity();
		} else {
			rotation.normalize();
		}

		return btTransform(rotation, btVector3(a_transform.origin.x, a_transform.origin.y, a_transform.origin.z));
	}

	std::unique_ptr<btCollisionShape> CreateCollisionShape(const Smp::PhysicsShapeDescriptor& a_descriptor);

	void LogPrototypeTransformDiagnostic(
		const std::string_view a_phase,
		RE::Actor* a_actor,
		const std::string& a_boneName,
		RE::NiNode* a_node,
		const btRigidBody& a_body)
	{
		const auto bodyTransform = a_body.getWorldTransform();
		const auto bodyOrigin = bodyTransform.getOrigin();
		const auto bodyRotation = bodyTransform.getRotation();
		const auto nodeTranslate = a_node ? a_node->world.translate : RE::NiPoint3{};
		const auto nodeRotation = a_node ? a_node->world.rotate : RE::NiMatrix3::IDENTITY;
		const auto nodeScale = a_node ? a_node->world.scale : 0.0F;
		spdlog::info(
			"prototype bone diagnostic {} actor={} bone='{}' node={} nodeWorld=({:.3f},{:.3f},{:.3f}) nodeRot=(({:.3f},{:.3f},{:.3f}),({:.3f},{:.3f},{:.3f}),({:.3f},{:.3f},{:.3f})) nodeScale={:.3f} bodyWorld=({:.3f},{:.3f},{:.3f}) bodyRot=({:.3f},{:.3f},{:.3f},{:.3f}) linearVel=({:.3f},{:.3f},{:.3f}) angularVel=({:.3f},{:.3f},{:.3f})",
			a_phase,
			static_cast<void*>(a_actor),
			a_boneName,
			static_cast<void*>(a_node),
			nodeTranslate.x,
			nodeTranslate.y,
			nodeTranslate.z,
			nodeRotation[0].x,
			nodeRotation[0].y,
			nodeRotation[0].z,
			nodeRotation[1].x,
			nodeRotation[1].y,
			nodeRotation[1].z,
			nodeRotation[2].x,
			nodeRotation[2].y,
			nodeRotation[2].z,
			nodeScale,
			bodyOrigin.x(),
			bodyOrigin.y(),
			bodyOrigin.z(),
			bodyRotation.x(),
			bodyRotation.y(),
			bodyRotation.z(),
			bodyRotation.w(),
			a_body.getLinearVelocity().x(),
			a_body.getLinearVelocity().y(),
			a_body.getLinearVelocity().z(),
			a_body.getAngularVelocity().x(),
			a_body.getAngularVelocity().y(),
			a_body.getAngularVelocity().z());
	}

	void LogNodeHierarchyRecursive(RE::NiAVObject* a_object, const std::uint32_t a_depth)
	{
		if (!a_object) {
			return;
		}

		const auto* parent = a_object->parent;
		const auto* node = a_object->IsNode();
		const auto& local = a_object->local.translate;
		const auto& world = a_object->world.translate;
		const auto childCount = node ? node->children.size() : 0;
		spdlog::debug(
			"actor skeleton hierarchy depth={} object={} parent={} name='{}' isNode={} children={} local=({:.3f},{:.3f},{:.3f}) world=({:.3f},{:.3f},{:.3f}) localScale={:.3f} worldScale={:.3f}",
			a_depth,
			static_cast<void*>(a_object),
			static_cast<const void*>(parent),
			std::string_view(a_object->GetName()),
			node != nullptr,
			childCount,
			local.x,
			local.y,
			local.z,
			world.x,
			world.y,
			world.z,
			a_object->local.scale,
			a_object->world.scale);

		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			LogNodeHierarchyRecursive(child.get(), a_depth + 1);
		}
	}

	void LogActorSkeletonHierarchy(RE::Actor* a_actor, const bool a_firstPerson, const std::string_view a_reason)
	{
		if (!a_actor) {
			return;
		}

		auto* root = a_actor->Get3D(a_firstPerson);
		if (!root && !a_firstPerson) {
			root = a_actor->Get3D();
		}
		spdlog::debug(
			"begin full actor skeleton hierarchy dump actor={} root={} firstPerson={} reason={}",
			static_cast<void*>(a_actor),
			static_cast<void*>(root),
			a_firstPerson,
			a_reason);
		LogNodeHierarchyRecursive(root, 0);
		spdlog::debug(
			"end full actor skeleton hierarchy dump actor={} root={} firstPerson={} reason={}",
			static_cast<void*>(a_actor),
			static_cast<void*>(root),
			a_firstPerson,
			a_reason);
	}

	bool IsFiniteTransform(const btTransform& a_transform)
	{
		const auto origin = a_transform.getOrigin();
		if (!std::isfinite(origin.x()) || !std::isfinite(origin.y()) || !std::isfinite(origin.z())) {
			return false;
		}

		const auto& basis = a_transform.getBasis();
		for (int row = 0; row < 3; ++row) {
			for (int column = 0; column < 3; ++column) {
				if (!std::isfinite(basis[row][column])) {
					return false;
				}
			}
		}
		return true;
	}

	btVector3 ApplyTranslationOffset(btDiscreteDynamicsWorld& a_world)
	{
		btVector3 center(0.0F, 0.0F, 0.0F);
		int count = 0;
		int localSpaceCount = 0;
		int worldSpaceCount = 0;
		for (int index = 0; index < a_world.getNumCollisionObjects(); ++index) {
			auto* body = btRigidBody::upcast(a_world.getCollisionObjectArray()[index]);
			if (!body) {
				continue;
			}

			const auto origin = body->getWorldTransform().getOrigin();
			if (origin.length2() < 2048.0F * 2048.0F) {
				++localSpaceCount;
			} else {
				++worldSpaceCount;
			}
			center += origin;
			++count;
		}

		if (count <= 0 || localSpaceCount > 0 || worldSpaceCount == 0) {
			return btVector3(0.0F, 0.0F, 0.0F);
		}

		center /= static_cast<btScalar>(count);
		for (int index = 0; index < a_world.getNumCollisionObjects(); ++index) {
			auto* body = btRigidBody::upcast(a_world.getCollisionObjectArray()[index]);
			if (!body) {
				continue;
			}

			body->getWorldTransform().getOrigin() -= center;
			body->getInterpolationWorldTransform().getOrigin() -= center;
		}
		return center;
	}

	void RestoreTranslationOffset(btDiscreteDynamicsWorld& a_world, const btVector3& a_offset)
	{
		if (a_offset.fuzzyZero()) {
			return;
		}

		for (int index = 0; index < a_world.getNumCollisionObjects(); ++index) {
			auto* body = btRigidBody::upcast(a_world.getCollisionObjectArray()[index]);
			if (!body) {
				continue;
			}

			body->getWorldTransform().getOrigin() += a_offset;
			body->getInterpolationWorldTransform().getOrigin() += a_offset;
		}
	}

	void RefreshSkinnedMeshWorldState(btDiscreteDynamicsWorld& a_world)
	{
		for (int index = 0; index < a_world.getNumCollisionObjects(); ++index) {
			auto* body = btRigidBody::upcast(a_world.getCollisionObjectArray()[index]);
			if (!body || !body->getUserPointer()) {
				continue;
			}

			auto* bone = static_cast<hdt::SkinnedMeshBone*>(body->getUserPointer());
			bone->internalUpdate();
		}

		for (int index = 0; index < a_world.getNumCollisionObjects(); ++index) {
			auto* object = a_world.getCollisionObjectArray()[index];
			if (!object || !object->getCollisionShape() || object->getCollisionShape()->getShapeType() != CUSTOM_CONCAVE_SHAPE_TYPE) {
				continue;
			}

			auto* meshBody = static_cast<hdt::SkinnedMeshBody*>(object);
			meshBody->internalUpdate();
			meshBody->updateBoundingSphereAabb();
		}
	}

	std::unique_ptr<btCollisionShape> CreateCollisionShape(const Smp::PhysicsShapeDescriptor& a_descriptor)
	{
		switch (a_descriptor.kind) {
		case Smp::PhysicsShapeKind::kBox:
		{
			const btVector3 halfExtents(
				std::max(a_descriptor.halfExtents.x, kMinimumShapeExtent),
				std::max(a_descriptor.halfExtents.y, kMinimumShapeExtent),
				std::max(a_descriptor.halfExtents.z, kMinimumShapeExtent));
			auto shape = std::make_unique<btBoxShape>(halfExtents);
			shape->setMargin(std::max(a_descriptor.margin, 0.0F));
			return shape;
		}
		case Smp::PhysicsShapeKind::kCapsule:
			return std::make_unique<btCapsuleShape>(
				std::max(a_descriptor.radius, kMinimumShapeExtent),
				std::max(a_descriptor.height, kMinimumShapeExtent));
		case Smp::PhysicsShapeKind::kCylinder:
		{
			auto shape = std::make_unique<btCylinderShape>(btVector3(
				std::max(a_descriptor.radius, kMinimumShapeExtent),
				std::max(a_descriptor.height, kMinimumShapeExtent),
				std::max(a_descriptor.radius, kMinimumShapeExtent)));
			shape->setMargin(std::max(a_descriptor.margin, 0.0F));
			return shape;
		}
		case Smp::PhysicsShapeKind::kHull:
		{
			auto shape = std::make_unique<btConvexHullShape>();
			for (const auto& point : a_descriptor.hullPoints) {
				shape->addPoint(btVector3(point.x, point.y, point.z), false);
			}
			if (shape->getNumPoints() == 0) {
				return std::make_unique<btEmptyShape>();
			}
			shape->recalcLocalAabb();
			shape->setMargin(std::max(a_descriptor.margin, 0.0F));
			return shape;
		}
		case Smp::PhysicsShapeKind::kCompound:
		{
			auto shape = std::make_unique<OwnedCompoundShape>();
			for (const auto& [transform, childDescriptor] : a_descriptor.compoundChildren) {
				auto child = CreateCollisionShape(childDescriptor);
				if (!child) {
					continue;
				}
				shape->addChildShape(ToBulletTransform(transform), child.get());
				shape->children.push_back(std::move(child));
			}
			if (shape->getNumChildShapes() == 0) {
				return std::make_unique<btEmptyShape>();
			}
			return shape;
		}
		case Smp::PhysicsShapeKind::kSphere:
		default:
			return std::make_unique<btSphereShape>(std::max(a_descriptor.radius, kMinimumShapeExtent));
		}
	}

	std::unique_ptr<btCollisionShape> CreateCollisionShape(const Smp::PhysicsBoneDescriptor& a_descriptor)
	{
		return a_descriptor.hasShape ? CreateCollisionShape(a_descriptor.shape) : std::make_unique<btEmptyShape>();
	}

	std::optional<hdt::BoundingSphere> CalculateBoneSphere(const Smp::Fo4DecodedSkinnedMesh& a_mesh, const std::size_t a_boneIndex)
	{
		hdt::Aabb bounds;
		bool hasVertex = false;
		for (const auto& vertex : a_mesh.vertices) {
			for (int influence = 0; influence < 4; ++influence) {
				if (vertex.weight_[influence] > FLT_EPSILON && vertex.getBoneIdx(influence) == a_boneIndex) {
					bounds.merge(vertex.skinPos_);
					hasVertex = true;
					break;
				}
			}
		}

		if (!hasVertex) {
			return std::nullopt;
		}

		const auto min = hdt::vectorFromM128(bounds.min_);
		const auto max = hdt::vectorFromM128(bounds.max_);
		const auto center = (min + max) * 0.5F;
		return hdt::BoundingSphere(center, center.distance(max));
	}

	struct PointCloudStats
	{
		btVector3 center{ 0.0F, 0.0F, 0.0F };
		btVector3 aabbCenter{ 0.0F, 0.0F, 0.0F };
		std::uint32_t samples{ 0 };
	};

	PointCloudStats CalculateSkinVertexStats(const std::vector<hdt::Vertex>& a_vertices)
	{
		PointCloudStats result;
		if (a_vertices.empty()) {
			return result;
		}

		hdt::Aabb bounds;
		for (const auto& vertex : a_vertices) {
			result.center += vertex.skinPos_;
			bounds.merge(vertex.skinPos_);
			++result.samples;
		}

		result.center /= static_cast<float>(result.samples);
		const auto min = hdt::vectorFromM128(bounds.min_);
		const auto max = hdt::vectorFromM128(bounds.max_);
		result.aabbCenter = (min + max) * 0.5F;
		return result;
	}

	PointCloudStats CalculateVertexPositionStats(const std::vector<hdt::VertexPos>& a_vertices)
	{
		PointCloudStats result;
		if (a_vertices.empty()) {
			return result;
		}

		hdt::Aabb bounds;
		for (const auto& vertex : a_vertices) {
			const auto position = vertex.pos();
			result.center += position;
			bounds.merge(position);
			++result.samples;
		}

		result.center /= static_cast<float>(result.samples);
		const auto min = hdt::vectorFromM128(bounds.min_);
		const auto max = hdt::vectorFromM128(bounds.max_);
		result.aabbCenter = (min + max) * 0.5F;
		return result;
	}

	PointCloudStats CalculateSkinnedBoneStats(const hdt::SkinnedMeshBody& a_body)
	{
		PointCloudStats result;
		if (a_body.skinnedBones_.empty()) {
			return result;
		}

		hdt::Aabb bounds;
		for (const auto& bone : a_body.skinnedBones_) {
			if (!bone.ptr) {
				continue;
			}

			const auto origin = bone.ptr->m_currentTransform.getOrigin();
			result.center += origin;
			bounds.merge(origin);
			++result.samples;
		}

		if (result.samples == 0) {
			return result;
		}

		result.center /= static_cast<float>(result.samples);
		const auto min = hdt::vectorFromM128(bounds.min_);
		const auto max = hdt::vectorFromM128(bounds.max_);
		result.aabbCenter = (min + max) * 0.5F;
		return result;
	}

	btVector3 ToBulletVector(const Smp::XmlVector3& a_value)
	{
		return btVector3(a_value.x, a_value.y, a_value.z);
	}

	float CurrentBoneScale(const Smp::Fo4SkinnedMeshBone* a_bone)
	{
		if (!a_bone) {
			return 1.0F;
		}
		const auto scale = a_bone->m_currentTransform.getScale();
		return std::isfinite(scale) && scale > FLT_EPSILON ? scale : 1.0F;
	}

	float WeightedScaleFactor(
		const float a_oldScaleA,
		const float a_oldScaleB,
		const float a_newScaleA,
		const float a_newScaleB,
		const float a_invMassA,
		const float a_invMassB)
	{
		const auto factorA = a_newScaleA / std::max(a_oldScaleA, FLT_EPSILON);
		const auto factorB = a_newScaleB / std::max(a_oldScaleB, FLT_EPSILON);
		const auto weight = a_invMassA + a_invMassB;
		if (weight <= FLT_EPSILON) {
			return (factorA + factorB) * 0.5F;
		}
		return (factorA * a_invMassA + factorB * a_invMassB) / weight;
	}

	void IncrementWritebackCounter(
		const Smp::WritebackSource a_source,
		std::uint32_t& a_cellJobsCounter,
		std::uint32_t& a_postAnimationCounter)
	{
		switch (a_source) {
		case Smp::WritebackSource::kCellJobs:
			++a_cellJobsCounter;
			break;
		case Smp::WritebackSource::kPostAnimationGraph:
			++a_postAnimationCounter;
			break;
		case Smp::WritebackSource::kUnknown:
		default:
			break;
		}
	}

	const char* WritebackSourceName(const Smp::WritebackSource a_source)
	{
		switch (a_source) {
		case Smp::WritebackSource::kMainSync:
			return "MainSync";
		case Smp::WritebackSource::kCellJobs:
			return "CellJobs";
		case Smp::WritebackSource::kPostAnimationGraph:
			return "PostAnimationGraph";
		case Smp::WritebackSource::kUnknown:
		default:
			return "Unknown";
		}
	}

	bool CanWriteBackFrame(const std::uint64_t a_lastFrame, const Smp::WritebackSource a_source, const std::uint64_t a_simulationFrame)
	{
		return a_source == Smp::WritebackSource::kMainSync || a_lastFrame != a_simulationFrame;
	}

	std::string WritebackPhaseName(const char* a_prefix, const Smp::WritebackSource a_source)
	{
		std::string result{ a_prefix };
		result.push_back('-');
		result += WritebackSourceName(a_source);
		return result;
	}

	float AxisValue(const Smp::XmlVector3& a_value, const int a_axis)
	{
		switch (a_axis) {
		case 0:
			return a_value.x;
		case 1:
			return a_value.y;
		case 2:
			return a_value.z;
		default:
			return 0.0F;
		}
	}

	btVector3 ReversedLimitLowerToBullet([[maybe_unused]] const Smp::XmlVector3& a_lower, const Smp::XmlVector3& a_upper)
	{
		return btVector3(-a_upper.x, -a_upper.y, -a_upper.z);
	}

	btVector3 ReversedLimitUpperToBullet(const Smp::XmlVector3& a_lower, [[maybe_unused]] const Smp::XmlVector3& a_upper)
	{
		return btVector3(-a_lower.x, -a_lower.y, -a_lower.z);
	}

	float AxisValue(const Smp::XmlVector3& a_value, const int a_axis, const bool a_reverse)
	{
		const auto value = AxisValue(a_value, a_axis);
		return a_reverse ? -value : value;
	}

	bool AxisLocked(const Smp::XmlVector3& a_lower, const Smp::XmlVector3& a_upper, const int a_axis)
	{
		return AxisValue(a_lower, a_axis) == AxisValue(a_upper, a_axis);
	}

	btQuaternion RotFromAToB(const btVector3& a_from, const btVector3& a_to)
	{
		const auto axis = a_from.cross(a_to);
		if (axis.fuzzyZero()) {
			return btQuaternion::getIdentity();
		}

		const auto sinAngle = axis.length();
		const auto cosAngle = a_from.dot(a_to);
		return btQuaternion(axis, btAtan2(cosAngle, sinAngle));
	}

	std::pair<btTransform, btTransform> CalculateConstraintFrames(
		const Smp::PhysicsConstraintDescriptor& a_descriptor,
		const hdt::btQsTransform& a_transformA,
		const hdt::btQsTransform& a_transformB)
	{
		btTransform frameA = btTransform::getIdentity();
		btTransform frameB = btTransform::getIdentity();
		const auto frame = ToBulletTransform(a_descriptor.frame);
		const auto frameQs = hdt::btQsTransform(frame);

		switch (a_descriptor.frameMode) {
		case Smp::PhysicsFrameMode::kAWithXPointToB:
		case Smp::PhysicsFrameMode::kAWithYPointToB:
		case Smp::PhysicsFrameMode::kAWithZPointToB:
		{
			int axis = 0;
			if (a_descriptor.frameMode == Smp::PhysicsFrameMode::kAWithYPointToB) {
				axis = 1;
			} else if (a_descriptor.frameMode == Smp::PhysicsFrameMode::kAWithZPointToB) {
				axis = 2;
			}

			auto frameWorld = a_transformA;
			auto oldAxis = btMatrix3x3(a_transformA.getBasis()).getColumn(axis);
			auto axisToB = a_transformB.getOrigin() - a_transformA.getOrigin();
			if (!oldAxis.fuzzyZero() && !axisToB.fuzzyZero()) {
				oldAxis.normalize();
				axisToB.normalize();
				const auto rotation = RotFromAToB(oldAxis, axisToB);
				frameWorld.getBasis() *= rotation;
			}
			frameA = (a_transformA.inverse() * frameWorld).asTransform();
			frameB = (a_transformB.inverse() * frameWorld).asTransform();
			break;
		}
		case Smp::PhysicsFrameMode::kFrameInA:
		{
			frameA = frame;
			const auto frameWorld = a_transformA * frameQs;
			frameB = (a_transformB.inverse() * frameWorld).asTransform();
			break;
		}
		case Smp::PhysicsFrameMode::kFrameInLerp:
		{
			const auto translationLerp = std::clamp(a_descriptor.translationLerp, 0.0F, 1.0F);
			const auto rotationLerp = std::clamp(a_descriptor.rotationLerp, 0.0F, 1.0F);
			const auto origin = a_transformA.getOrigin().lerp(a_transformB.getOrigin(), translationLerp);
			const auto rotation = a_transformA.getBasis().slerp(a_transformB.getBasis(), rotationLerp);
			const hdt::btQsTransform frameWorld(rotation, origin);
			frameA = (a_transformA.inverse() * frameWorld).asTransform();
			frameB = (a_transformB.inverse() * frameWorld).asTransform();
			break;
		}
		case Smp::PhysicsFrameMode::kFrameInB:
		default:
		{
			frameB = frame;
			const auto frameWorld = a_transformB * frameQs;
			frameA = (a_transformA.inverse() * frameWorld).asTransform();
			break;
		}
		}

		return { frameA, frameB };
	}

	class PrototypeStiffSpringConstraint :
		public btTypedConstraint
	{
	public:
		PrototypeStiffSpringConstraint(
			btRigidBody& a_bodyA,
			btRigidBody& a_bodyB,
			const btTransform& a_rigToLocalA,
			const btTransform& a_rigToLocalB,
			const float a_minDistanceFactor,
			const float a_maxDistanceFactor,
			const float a_stiffness,
			const float a_damping,
			const float a_equilibriumFactor) :
			btTypedConstraint(MAX_CONSTRAINT_TYPE, a_bodyA, a_bodyB),
			rigToLocalA_(a_rigToLocalA),
			rigToLocalB_(a_rigToLocalB),
			stiffness_(std::max(a_stiffness, 0.0F)),
			damping_(std::max(a_damping, 0.0F))
		{
			const auto localA = rigToLocalA_ * m_rbA.getWorldTransform();
			const auto localB = rigToLocalB_ * m_rbB.getWorldTransform();
			const auto distance = localA.getOrigin().distance(localB.getOrigin());
			minDistance_ = distance * std::max(a_minDistanceFactor, 0.0F);
			maxDistance_ = distance * std::max(a_maxDistanceFactor, 0.0F);
			const auto equilibriumFactor = std::clamp(a_equilibriumFactor, 0.0F, 1.0F);
			equilibriumPoint_ = minDistance_ * equilibriumFactor + maxDistance_ * (1.0F - equilibriumFactor);
		}

		void getInfo1(btConstraintInfo1* a_info) override
		{
			const auto localA = rigToLocalA_ * m_rbA.getWorldTransform();
			const auto localB = rigToLocalB_ * m_rbB.getWorldTransform();
			const auto distance = (localA.getOrigin() - localB.getOrigin()).length();

			a_info->m_numConstraintRows = btFuzzyZero(distance) ? 0 : 1;
			a_info->nub = 0;
		}

		void getInfo2(btConstraintInfo2* a_info) override
		{
			const auto localA = rigToLocalA_ * m_rbA.getWorldTransform();
			const auto localB = rigToLocalB_ * m_rbB.getWorldTransform();
			const auto deltaVector = localA.getOrigin() - localB.getOrigin();
			const auto distance = deltaVector.length();
			if (btFuzzyZero(distance)) {
				return;
			}

			const auto direction = deltaVector.normalized();
			a_info->m_J1linearAxis[0] = direction[0];
			a_info->m_J1linearAxis[1] = direction[1];
			a_info->m_J1linearAxis[2] = direction[2];
			a_info->m_J2linearAxis[0] = -direction[0];
			a_info->m_J2linearAxis[1] = -direction[1];
			a_info->m_J2linearAxis[2] = -direction[2];

			int currentLimit = 0;
			float currentLimitError = 0.0F;
			if (distance < minDistance_) {
				currentLimit = 2;
				currentLimitError = distance - minDistance_;
			} else if (distance > maxDistance_) {
				currentLimit = 1;
				currentLimitError = distance - maxDistance_;
			}

			if (currentLimit == 0) {
				const auto delta = distance - equilibriumPoint_;
				const auto velocity = (delta - oldDiff_) * a_info->fps;
				auto force = (delta + oldDiff_) * 0.5F * stiffness_;
				const auto friction = damping_ * velocity;
				force += force * friction < 0.0F ? btClamped(friction, -std::abs(force), std::abs(force)) : friction;

				const auto targetVelocity = (a_info->fps / static_cast<btScalar>(a_info->m_numIterations)) * force;
				const auto maxMotorForce = btFabs(force) / a_info->fps;
				const auto motorFactor = getMotorFactor(distance, minDistance_, maxDistance_, targetVelocity, a_info->fps * a_info->erp);
				a_info->m_constraintError[0] = motorFactor * targetVelocity * (m_rbA.getInvMass() + m_rbB.getInvMass());
				a_info->m_lowerLimit[0] = -maxMotorForce;
				a_info->m_upperLimit[0] = maxMotorForce;
				oldDiff_ = delta;
				return;
			}

			const auto k = a_info->fps * a_info->erp;
			a_info->m_constraintError[0] = k * currentLimitError;
			if (minDistance_ == maxDistance_) {
				a_info->m_lowerLimit[0] = -SIMD_INFINITY;
				a_info->m_upperLimit[0] = SIMD_INFINITY;
			} else if (currentLimit == 1) {
				a_info->m_lowerLimit[0] = -SIMD_INFINITY;
				a_info->m_upperLimit[0] = 0.0F;
			} else {
				a_info->m_lowerLimit[0] = 0.0F;
				a_info->m_upperLimit[0] = SIMD_INFINITY;
			}
		}

		void setParam([[maybe_unused]] int a_num, [[maybe_unused]] btScalar a_value, [[maybe_unused]] int a_axis = -1) override {}
		btScalar getParam([[maybe_unused]] int a_num, [[maybe_unused]] int a_axis = -1) const override { return 0.0F; }

	private:
		btTransform rigToLocalA_{ btTransform::getIdentity() };
		btTransform rigToLocalB_{ btTransform::getIdentity() };
		float minDistance_{ 0.0F };
		float maxDistance_{ 0.0F };
		float stiffness_{ 0.0F };
		float damping_{ 0.0F };
		float equilibriumPoint_{ 0.0F };
		float oldDiff_{ 0.0F };

	public:
		void ScaleConstraint(
			const float a_oldScaleA,
			const float a_oldScaleB,
			const float a_newScaleA,
			const float a_newScaleB)
		{
			const auto factor = WeightedScaleFactor(
				a_oldScaleA,
				a_oldScaleB,
				a_newScaleA,
				a_newScaleB,
				m_rbA.getInvMass(),
				m_rbB.getInvMass());
			const auto factorCubed = factor * factor * factor;

			minDistance_ *= factor;
			maxDistance_ *= factor;
			equilibriumPoint_ *= factor;
			stiffness_ *= factorCubed;
			damping_ *= factorCubed;
		}
	};

	void ScaleGenericConstraint(
		btGeneric6DofSpring2Constraint& a_constraint,
		const float a_oldScaleA,
		const float a_oldScaleB,
		const float a_newScaleA,
		const float a_newScaleB)
	{
		const auto factorA = a_newScaleA / std::max(a_oldScaleA, FLT_EPSILON);
		const auto factorB = a_newScaleB / std::max(a_oldScaleB, FLT_EPSILON);
		const auto factor = WeightedScaleFactor(
			a_oldScaleA,
			a_oldScaleB,
			a_newScaleA,
			a_newScaleB,
			a_constraint.getRigidBodyA().getInvMass(),
			a_constraint.getRigidBodyB().getInvMass());
		const auto factorSquared = factor * factor;
		const auto factorCubed = factorSquared * factor;
		const auto factorFifth = factorCubed * factorSquared;

		auto frameA = a_constraint.getFrameOffsetA();
		auto frameB = a_constraint.getFrameOffsetB();
		frameA.setOrigin(frameA.getOrigin() * factorA);
		frameB.setOrigin(frameB.getOrigin() * factorB);
		a_constraint.setFrames(frameA, frameB);

		auto* linear = a_constraint.getTranslationalLimitMotor();
		linear->m_equilibriumPoint *= factor;
		linear->m_springStiffness *= factorCubed;
		linear->m_lowerLimit *= factor;
		linear->m_upperLimit *= factor;

		for (int axis = 0; axis < 3; ++axis) {
			if (auto* motor = a_constraint.getRotationalLimitMotor(axis)) {
				motor->m_springStiffness *= factorFifth;
			}
		}
	}

	void ScaleConeTwistConstraint(
		btConeTwistConstraint& a_constraint,
		const float a_oldScaleA,
		const float a_oldScaleB,
		const float a_newScaleA,
		const float a_newScaleB)
	{
		const auto factorA = a_newScaleA / std::max(a_oldScaleA, FLT_EPSILON);
		const auto factorB = a_newScaleB / std::max(a_oldScaleB, FLT_EPSILON);
		auto frameA = a_constraint.getFrameOffsetA();
		auto frameB = a_constraint.getFrameOffsetB();
		frameA.setOrigin(frameA.getOrigin() * factorA);
		frameB.setOrigin(frameB.getOrigin() * factorB);
		a_constraint.setFrames(frameA, frameB);
	}

	std::unique_ptr<btTypedConstraint> CreatePrototypeConstraint(
		const Smp::PhysicsConstraintDescriptor& a_descriptor,
		btRigidBody& a_bodyA,
		btRigidBody& a_bodyB,
		const btTransform& a_rigToLocalA,
		const btTransform& a_rigToLocalB,
		const hdt::btQsTransform& a_nodeTransformA,
		const hdt::btQsTransform& a_nodeTransformB)
	{
		auto [nodeFrameA, nodeFrameB] = CalculateConstraintFrames(a_descriptor, a_nodeTransformA, a_nodeTransformB);
		const auto frameA = a_rigToLocalA * nodeFrameA;
		const auto frameB = a_rigToLocalB * nodeFrameB;

		switch (a_descriptor.kind) {
		case Smp::PhysicsConstraintKind::kConeTwist:
		{
			auto constraint = std::make_unique<btConeTwistConstraint>(a_bodyA, a_bodyB, frameA, frameB);
			constraint->setLimit(
				a_descriptor.swingSpan1,
				a_descriptor.swingSpan2,
				a_descriptor.twistSpan,
				a_descriptor.limitSoftness,
				a_descriptor.biasFactor,
				a_descriptor.relaxationFactor);
			return constraint;
		}
		case Smp::PhysicsConstraintKind::kStiffSpring:
			return std::make_unique<PrototypeStiffSpringConstraint>(
				a_bodyA,
				a_bodyB,
				a_rigToLocalA,
				a_rigToLocalB,
				a_descriptor.minDistanceFactor,
				a_descriptor.maxDistanceFactor,
				a_descriptor.stiffness,
				a_descriptor.damping,
				a_descriptor.equilibriumFactor);
		case Smp::PhysicsConstraintKind::kGeneric:
		default:
		{
			const bool reverseFrames = a_descriptor.useLinearReferenceFrameA;
			auto constraint = reverseFrames ?
				std::make_unique<btGeneric6DofSpring2Constraint>(a_bodyB, a_bodyA, btTransform::getIdentity(), btTransform::getIdentity(), RO_XYZ) :
				std::make_unique<btGeneric6DofSpring2Constraint>(a_bodyA, a_bodyB, btTransform::getIdentity(), btTransform::getIdentity(), RO_XYZ);
			if (reverseFrames) {
				constraint->setFrames(frameB, frameA);
			} else {
				constraint->setFrames(frameA, frameB);
			}
			constraint->setLinearLowerLimit(reverseFrames ? ReversedLimitLowerToBullet(a_descriptor.linearLowerLimit, a_descriptor.linearUpperLimit) : ToBulletVector(a_descriptor.linearLowerLimit));
			constraint->setLinearUpperLimit(reverseFrames ? ReversedLimitUpperToBullet(a_descriptor.linearLowerLimit, a_descriptor.linearUpperLimit) : ToBulletVector(a_descriptor.linearUpperLimit));
			constraint->setAngularLowerLimit(reverseFrames ? ReversedLimitLowerToBullet(a_descriptor.angularLowerLimit, a_descriptor.angularUpperLimit) : ToBulletVector(a_descriptor.angularLowerLimit));
			constraint->setAngularUpperLimit(reverseFrames ? ReversedLimitUpperToBullet(a_descriptor.angularLowerLimit, a_descriptor.angularUpperLimit) : ToBulletVector(a_descriptor.angularUpperLimit));
			for (int axis = 0; axis < 3; ++axis) {
				const auto linearStiffness = AxisValue(a_descriptor.linearStiffness, axis);
				const auto linearDamping = AxisValue(a_descriptor.linearDamping, axis);
				const bool hasLinearSpring = a_descriptor.enableLinearSprings &&
					(linearStiffness > 0.0F || linearDamping > 0.0F);
				const bool hasAngularSpring = a_descriptor.enableAngularSprings &&
					(AxisValue(a_descriptor.angularStiffness, axis) > 0.0F || AxisValue(a_descriptor.angularDamping, axis) > 0.0F);
				constraint->enableSpring(axis, hasLinearSpring);
				constraint->enableSpring(axis + 3, hasAngularSpring);
				constraint->setStiffness(axis, linearStiffness, a_descriptor.linearStiffnessLimited);
				constraint->setStiffness(axis + 3, AxisValue(a_descriptor.angularStiffness, axis), a_descriptor.angularStiffnessLimited);
				constraint->setDamping(axis, linearDamping, a_descriptor.springDampingLimited);
				constraint->setDamping(axis + 3, AxisValue(a_descriptor.angularDamping, axis), a_descriptor.springDampingLimited);
				constraint->setNonHookeanDamping(axis, AxisValue(a_descriptor.linearNonHookeanDamping, axis));
				constraint->setNonHookeanDamping(axis + 3, AxisValue(a_descriptor.angularNonHookeanDamping, axis));
				constraint->setNonHookeanStiffness(axis, AxisValue(a_descriptor.linearNonHookeanStiffness, axis));
				constraint->setNonHookeanStiffness(axis + 3, AxisValue(a_descriptor.angularNonHookeanStiffness, axis));
				constraint->setEquilibriumPoint(axis, AxisValue(a_descriptor.linearEquilibrium, axis, reverseFrames));
				constraint->setEquilibriumPoint(axis + 3, AxisValue(a_descriptor.angularEquilibrium, axis, reverseFrames));
				constraint->setBounce(axis, AxisValue(a_descriptor.linearBounce, axis));
				constraint->setBounce(axis + 3, AxisValue(a_descriptor.angularBounce, axis));
				constraint->enableMotor(axis, a_descriptor.linearMotors);
				constraint->enableMotor(axis + 3, a_descriptor.angularMotors);
				constraint->setServo(axis, a_descriptor.linearServoMotors);
				constraint->setServo(axis + 3, a_descriptor.angularServoMotors);
				constraint->setServoTarget(axis, AxisValue(a_descriptor.linearEquilibrium, axis, reverseFrames));
				constraint->setServoTarget(axis + 3, AxisValue(a_descriptor.angularEquilibrium, axis, reverseFrames));
				constraint->setTargetVelocity(axis, AxisValue(a_descriptor.linearTargetVelocity, axis, reverseFrames));
				constraint->setTargetVelocity(axis + 3, AxisValue(a_descriptor.angularTargetVelocity, axis, reverseFrames));
				constraint->setMaxMotorForce(axis, AxisValue(a_descriptor.linearMaxMotorForce, axis));
				constraint->setMaxMotorForce(axis + 3, AxisValue(a_descriptor.angularMaxMotorForce, axis));
				constraint->setParam(BT_CONSTRAINT_ERP, a_descriptor.motorErp, axis);
				constraint->setParam(BT_CONSTRAINT_ERP, a_descriptor.motorErp, axis + 3);
				constraint->setParam(BT_CONSTRAINT_CFM, a_descriptor.motorCfm, axis);
				constraint->setParam(BT_CONSTRAINT_CFM, a_descriptor.motorCfm, axis + 3);
				constraint->setParam(BT_CONSTRAINT_STOP_ERP, AxisLocked(a_descriptor.linearLowerLimit, a_descriptor.linearUpperLimit, axis) ? 1.0F : a_descriptor.stopErp, axis);
				constraint->setParam(BT_CONSTRAINT_STOP_ERP, AxisLocked(a_descriptor.angularLowerLimit, a_descriptor.angularUpperLimit, axis) ? 1.0F : a_descriptor.stopErp, axis + 3);
				constraint->setParam(BT_CONSTRAINT_STOP_CFM, a_descriptor.stopCfm, axis);
				constraint->setParam(BT_CONSTRAINT_STOP_CFM, a_descriptor.stopCfm, axis + 3);
				if (auto* motor = constraint->getRotationalLimitMotor(axis)) {
					motor->m_motorERP = a_descriptor.motorErp;
					motor->m_motorCFM = a_descriptor.motorCfm;
					motor->m_stopERP = AxisLocked(a_descriptor.angularLowerLimit, a_descriptor.angularUpperLimit, axis) ? 1.0F : a_descriptor.stopErp;
					motor->m_stopCFM = a_descriptor.stopCfm;
					motor->m_bounce = AxisValue(a_descriptor.angularBounce, axis);
				}
			}
			constraint->getTranslationalLimitMotor()->m_bounce = ToBulletVector(a_descriptor.linearBounce);
			return constraint;
		}
		}
	}
}


#include "Fo4PhysicsWorld/Core.inl"
#include "Fo4PhysicsWorld/Simulation.inl"
#include "Fo4PhysicsWorld/Events.inl"
#include "Fo4PhysicsWorld/Lifecycle.inl"
#include "Fo4PhysicsWorld/Rebuilds.inl"
#include "Fo4PhysicsWorld/RuntimeState.inl"
#include "Fo4PhysicsWorld/Build.inl"
