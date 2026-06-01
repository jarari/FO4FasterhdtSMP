#include "Fo4PhysicsWorld.h"

#include "BSSkin.h"
#include "ConfigPaths.h"
#include "DefaultBBP.h"
#include "Fo4MeshExtractor.h"
#include "Fo4NiObjectUtils.h"
#include "Fo4SkinnedMeshBone.h"
#include "Fo4TransformConversion.h"
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
	constexpr std::uint32_t kAttachResetReadFrames = 8;
	constexpr std::uint32_t kHeadInitializedRebuildDelayFrames = 2;
	constexpr std::uint32_t kArmorChangeRebuildDelayTasks = 0;
	constexpr std::uint32_t kCpuCopyPendingRetryDelayTasks = 10;
	constexpr std::uint32_t kCpuCopyPendingMaxRetries = 3;
	constexpr std::uint64_t kPendingRebuildRetryIntervalFrames = 15;
	std::atomic<std::uint32_t> PrototypeArmorRenameId{ 0 };
	std::atomic<std::uint32_t> PrototypeHeadRenameId{ 0 };
	using Clock = std::chrono::steady_clock;

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
		RE::NiAVObject* sourceObject{ nullptr };
		RE::NiNode* sourceRoot{ nullptr };
		bool preserveMergeSourceNames{ false };
		Smp::PrototypeBuildDomain domain{ Smp::PrototypeBuildDomain::kHead };
	};

	std::optional<ArmorPhysicsXmlSelection> FindArmorPhysicsXml(RE::NiAVObject* a_object);
	RE::NiNode* FindNodeByName(RE::NiAVObject* a_root, std::string_view a_name);
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

	bool RebindSkinBoneSlot(RE::BSSkin::Instance* a_skin, const std::uint32_t a_index, RE::NiNode* a_node)
	{
		if (!a_skin || !a_node || a_index >= a_skin->bones.size()) {
			return false;
		}

		a_skin->bones[a_index] = a_node;
		if (a_index < a_skin->worldTransforms.size()) {
			a_skin->worldTransforms[a_index] = std::addressof(a_node->world);
		}
		return true;
	}

	RE::NiNode* FindNodeByName(RE::NiAVObject* a_root, const std::string_view a_name)
	{
		return Smp::NiObject::FindNodeByName(a_root, a_name);
	}

	bool IsExcludedMergeSearchObject(
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_excludedRootA,
		RE::NiAVObject* a_excludedRootB,
		RE::NiAVObject* a_excludedRootC,
		const std::vector<RE::NiAVObject*>& a_excludedObjects)
	{
		if (!a_object) {
			return true;
		}

		if (a_object == a_excludedRootA || a_object == a_excludedRootB || a_object == a_excludedRootC) {
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
			if (IsProbablyValidNiObject(object)) {
				result.insert(object);
			}
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
		auto* node = a_object ? a_object->IsNode() : nullptr;
		if (!node) {
			return;
		}

		const auto name = node->GetName();
		if (name.empty()) {
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
			IsExcludedMergeSearchObject(a_object, a_excludedRootA, a_excludedRootB, a_excludedRootC, a_excludedObjects) ||
			a_knownArmorNodes.contains(a_object)) {
			return;
		}

		AddActorSkeletonLookupNode(a_lookup, a_object);

		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}

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

	RE::NiNode* FindNodeByNameExcludingKnownNodes(
		RE::NiAVObject* a_root,
		const std::string_view a_name,
		RE::NiAVObject* a_excludedRootA,
		RE::NiAVObject* a_excludedRootB,
		RE::NiAVObject* a_excludedRootC,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes)
	{
		if (!a_root ||
			a_name.empty() ||
			IsExcludedMergeSearchObject(a_root, a_excludedRootA, a_excludedRootB, a_excludedRootC, a_excludedObjects) ||
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
			if (auto* found = FindNodeByNameExcludingKnownNodes(child.get(), a_name, a_excludedRootA, a_excludedRootB, a_excludedRootC, a_excludedObjects, a_knownArmorNodes)) {
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
				nullptr,
				nullptr,
				nullptr,
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
		return cloneObject ? static_cast<RE::NiAVObject*>(cloneObject->IsNode()) : nullptr;
	}

	RE::NiNode* CloneMergedNodeOnly(RE::NiNode* a_source, const std::string& a_prefix, std::vector<MergedSkeletonNode>& a_renamedNodes)
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

		std::vector<RE::NiAVObject*> clonedChildren;
		for (auto& child : cloneNode->children) {
			if (child) {
				clonedChildren.push_back(child.get());
			}
		}
		for (auto* child : clonedChildren) {
			cloneNode->DetachChild(child);
		}

		const auto originalName = a_source->GetName();
		if (!originalName.empty()) {
			auto renamed = a_prefix;
			renamed += std::string_view(originalName);
			a_renamedNodes.push_back({
				.originalName = std::string(originalName),
				.renamedName = renamed,
				.node = cloneNode,
			});
			cloneNode->name = renamed.c_str();
		}

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
		return a_name.empty() || Smp::PhysicsNamesEqual(a_name, "BSFaceGenNiNodeSkinned");
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
		RE::NiAVObject* a_excludedRootA,
		RE::NiAVObject* a_excludedRootB,
		RE::NiAVObject* a_excludedRootC,
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
				a_excludedRootA,
				a_excludedRootB,
				a_excludedRootC,
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
		auto* actorNode = FindActorSkeletonLookupNode(a_actorSkeletonLookup, a_name);
		if (!actorNode) {
			actorNode = FindNodeByNameExcludingKnownNodes(
				a_actorRoot,
				a_name,
				a_excludedRootA,
				a_excludedRootB,
				a_excludedRootC,
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
				nullptr,
				nullptr,
				nullptr,
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
						nullptr,
						nullptr,
						nullptr,
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

	struct MergeParentResolution
	{
		RE::NiNode* parent{ nullptr };
		std::string parentName;
		RE::NiTransform localToParent{ RE::NiTransform::IDENTITY };
		bool hasLocalToParent{ false };
		bool recordBinding{ false };
		bool fromStoredBinding{ false };
		bool fromConstraint{ false };
	};

	RE::NiTransform BuildLocalToSourceAncestor(RE::NiNode* a_node, RE::NiNode* a_ancestor)
	{
		return Smp::NiObject::BuildLocalToAncestor(a_node, a_ancestor);
	}

	RE::NiNode* FindTrustedActorSkeletonNodeForName(
		RE::NiAVObject* a_actorRoot,
		const std::string_view a_name,
		const ActorSkeletonLookup& a_actorSkeletonLookup,
		const std::vector<RE::NiAVObject*>& a_actorSkeletonSearchExclusions,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		auto* actorNode = FindActorSkeletonLookupNode(a_actorSkeletonLookup, a_name);
		if (!actorNode) {
			actorNode = FindNodeByNameExcludingKnownNodes(
				a_actorRoot,
				a_name,
				nullptr,
				nullptr,
				nullptr,
				a_actorSkeletonSearchExclusions,
				a_knownArmorNodes);
		}
		if (actorNode && !IsTrustedActorSkeletonCandidate(actorNode, a_trustedActorSkeletonNodes)) {
			spdlog::debug(
				"trusted actor parent lookup for '{}' rejected actorNode={} because it was not present in the pre-attach trusted actor skeleton set",
				a_name,
				static_cast<void*>(actorNode));
			return nullptr;
		}
		return actorNode;
	}

	MergeParentResolution ResolveMergeParentForArmorBone(
		RE::NiNode* a_defaultParent,
		RE::NiNode* a_source,
		RE::NiAVObject* a_actorRoot,
		const ActorSkeletonLookup& a_actorSkeletonLookup,
		const Smp::PhysicsXmlSummary& a_summary,
		const std::vector<Smp::MergeParentBinding>& a_mergeParentBindings,
		const std::vector<RE::NiAVObject*>& a_actorSkeletonSearchExclusions,
		const std::unordered_set<RE::NiAVObject*>& a_knownArmorNodes,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		MergeParentResolution result;
		result.parent = a_defaultParent;
		if (!a_source) {
			return result;
		}

		const auto sourceName = a_source->GetName();
		for (auto* sourceParent = a_source->parent; sourceParent; sourceParent = sourceParent->parent) {
			const auto sourceParentName = sourceParent->GetName();
			if (sourceParentName.empty()) {
				continue;
			}
			if (auto* actorParent = FindTrustedActorSkeletonNodeForName(a_actorRoot, sourceParentName, a_actorSkeletonLookup, a_actorSkeletonSearchExclusions, a_knownArmorNodes, a_trustedActorSkeletonNodes)) {
				result.parent = actorParent;
				result.parentName = std::string(sourceParentName);
				result.localToParent = BuildLocalToSourceAncestor(a_source, sourceParent);
				result.hasLocalToParent = true;
				result.recordBinding = true;
				return result;
			}
		}

		if (!sourceName.empty()) {
			const auto storedParent = std::ranges::find_if(a_mergeParentBindings, [sourceName](const Smp::MergeParentBinding& a_binding) {
				return Smp::PhysicsNamesEqual(a_binding.sourceName, sourceName);
			});
			if (storedParent != a_mergeParentBindings.end() && !storedParent->parentName.empty()) {
				if (auto* actorParent = FindTrustedActorSkeletonNodeForName(a_actorRoot, storedParent->parentName, a_actorSkeletonLookup, a_actorSkeletonSearchExclusions, a_knownArmorNodes, a_trustedActorSkeletonNodes)) {
					result.parent = actorParent;
					result.parentName = storedParent->parentName;
					result.localToParent = storedParent->localToParent;
					result.hasLocalToParent = storedParent->hasLocalToParent;
					result.recordBinding = true;
					result.fromStoredBinding = true;
					return result;
				}
				spdlog::debug(
					"stored merge parent binding for source '{}' requested actor parent '{}' but it was not found under current actor skeleton",
					sourceName,
					storedParent->parentName);
			}
		}

		if (!sourceName.empty()) {
			for (const auto& constraint : a_summary.constraintDescriptors) {
				std::string_view candidateName;
				if (Smp::PhysicsNamesEqual(constraint.bodyB, sourceName)) {
					candidateName = constraint.bodyA;
				} else if (Smp::PhysicsNamesEqual(constraint.bodyA, sourceName)) {
					candidateName = constraint.bodyB;
				}
				if (candidateName.empty()) {
					continue;
				}
				if (auto* actorParent = FindTrustedActorSkeletonNodeForName(a_actorRoot, candidateName, a_actorSkeletonLookup, a_actorSkeletonSearchExclusions, a_knownArmorNodes, a_trustedActorSkeletonNodes)) {
					result.parent = actorParent;
					result.parentName = std::string(candidateName);
					result.fromConstraint = true;
					return result;
				}
			}
		}

		return result;
	}

	bool IsCurrentMergeCloneNode(const std::vector<MergedSkeletonNode>& a_renamedNodes, RE::NiNode* a_node)
	{
		return a_node &&
			std::ranges::any_of(a_renamedNodes, [a_node](const MergedSkeletonNode& a_entry) {
				return a_entry.node == a_node;
			});
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

		const auto sourceName = a_source->GetName();
		const auto ownership = ClassifySourceBoneOwnership(a_actorRoot, a_source, a_actorSkeletonLookup, a_actorSkeletonSearchExclusions, a_knownArmorNodes, a_trustedActorSkeletonNodes, a_mergeParentBindings);
		const auto currentRelevant = IsReferencedXmlSourceBone(a_summary, sourceName);
		const auto descendantRelevant = HasRelevantXmlDescendant(a_source, a_summary);
		if (ownership == SourceBoneOwnership::kActorSkeleton) {
			auto* actorNode = FindTrustedActorSkeletonNodeForSource(
				a_actorRoot,
				a_source,
				a_actorSkeletonLookup,
				nullptr,
				nullptr,
				nullptr,
				a_actorSkeletonSearchExclusions,
				a_knownArmorNodes,
				a_trustedActorSkeletonNodes);
			auto* nextCloneParent = actorNode ? actorNode : a_cloneParent;
			if (!sourceName.empty() && (currentRelevant || descendantRelevant)) {
				if (actorNode) {
					spdlog::debug(
						"armor skeleton clone skipped actor-owned source bone '{}' sourceNode={} actorNode={}; actor skeleton node will be used directly if XML/skin needs it",
						sourceName,
						static_cast<void*>(a_source),
						static_cast<void*>(actorNode));
				}
			}
			for (auto& child : a_source->children) {
				if (auto* sourceChild = child ? child->IsNode() : nullptr) {
					CloneSourceSkeletonIntoPartTree(nextCloneParent, sourceChild, a_actorRoot, a_actorSkeletonLookup, a_actorSkeletonSearchExclusions, a_knownArmorNodes, a_trustedActorSkeletonNodes, a_prefix, a_summary, a_mergeParentBindings, a_renamedNodes, a_mergedRoots);
				}
			}
			return;
		}

		if (ownership == SourceBoneOwnership::kArmorOwned && currentRelevant) {
			const auto parentIsPluginClone = IsCurrentMergeCloneNode(a_renamedNodes, a_cloneParent);
			auto* clonedNode = CloneMergedNodeOnly(a_source, a_prefix, a_renamedNodes);
			if (clonedNode) {
				auto resolution = ResolveMergeParentForArmorBone(
					a_cloneParent,
					a_source,
					a_actorRoot,
					a_actorSkeletonLookup,
					a_summary,
					a_mergeParentBindings,
					a_actorSkeletonSearchExclusions,
					a_knownArmorNodes,
					a_trustedActorSkeletonNodes);
				auto* cloneParent = parentIsPluginClone ? a_cloneParent : (resolution.parent ? resolution.parent : a_cloneParent);
				cloneParent->AttachChild(clonedNode, false);
				if (parentIsPluginClone) {
					clonedNode->local = a_source->local;
					UpdateNodeWorldFromLocal(clonedNode);
				} else if (resolution.hasLocalToParent) {
					clonedNode->local = resolution.localToParent;
					UpdateNodeWorldFromLocal(clonedNode);
				} else if (resolution.fromStoredBinding || resolution.fromConstraint || (resolution.recordBinding && cloneParent != a_cloneParent)) {
					clonedNode->local = a_source->local;
					UpdateNodeWorldFromLocal(clonedNode);
				} else {
					UpdateNodeWorldFromLocal(clonedNode);
				}
				const auto actualLocal = clonedNode->local;
				const auto recordLocal = resolution.hasLocalToParent ? resolution.localToParent : actualLocal;
				a_mergedRoots.push_back({
					.parent = cloneParent,
					.node = clonedNode,
					.sourceNode = a_source,
					.originalName = std::string(sourceName),
					.recordParentName = resolution.recordBinding ? resolution.parentName : std::string{},
					.localToParent = actualLocal,
					.recordLocalToParent = recordLocal,
					.hasLocalToParent = true,
					.hasRecordLocalToParent = true,
					.recordMergeParentBinding = resolution.recordBinding,
				});
				spdlog::debug(
					"cloned armor-owned XML source bone '{}' as plugin-owned prefixed node='{}' node={} under merge parent={} parentName='{}' prefix='{}' storedParent={} constraintParent={} recordBinding={} recordParent='{}' actualLocal=({:.3f},{:.3f},{:.3f}) recordLocal=({:.3f},{:.3f},{:.3f})",
					sourceName,
					std::string_view(clonedNode->GetName()),
					static_cast<void*>(clonedNode),
					static_cast<void*>(cloneParent),
					cloneParent ? std::string_view(cloneParent->GetName()) : std::string_view{},
					a_prefix,
					resolution.fromStoredBinding,
					resolution.fromConstraint,
					resolution.recordBinding,
					resolution.recordBinding ? std::string_view(resolution.parentName) : std::string_view{},
					actualLocal.translate.x,
					actualLocal.translate.y,
					actualLocal.translate.z,
					recordLocal.translate.x,
					recordLocal.translate.y,
					recordLocal.translate.z);
				for (auto& child : a_source->children) {
					if (auto* sourceChild = child ? child->IsNode() : nullptr) {
						CloneSourceSkeletonIntoPartTree(clonedNode, sourceChild, a_actorRoot, a_actorSkeletonLookup, a_actorSkeletonSearchExclusions, a_knownArmorNodes, a_trustedActorSkeletonNodes, a_prefix, a_summary, a_mergeParentBindings, a_renamedNodes, a_mergedRoots);
					}
				}
				return;
			}
		}

		if (ownership == SourceBoneOwnership::kArmorOwned && !descendantRelevant) {
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

			CloneSourceSkeletonIntoPartTree(a_cloneParent, sourceChild, a_actorRoot, a_actorSkeletonLookup, a_actorSkeletonSearchExclusions, a_knownArmorNodes, a_trustedActorSkeletonNodes, a_prefix, a_summary, a_mergeParentBindings, a_renamedNodes, a_mergedRoots);
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

	void RebindMatchedSkinSlots(
		std::vector<MatchedSkinBone>& a_matchedBones,
		const std::vector<MergedSkeletonNode>& a_renamedNodes,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_skeletonRoot)
	{
		if (!a_object) {
			return;
		}

		if (auto* geometry = a_object->IsGeometry()) {
			auto* skin = geometry->skinInstance ? geometry->skinInstance.get() : nullptr;
			if (!skin || skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				return;
			}

			for (std::uint32_t index = 0; index < skin->bones.size(); ++index) {
				auto* skinBoneObject = skin->bones[index];
				if (skinBoneObject && !IsProbablyValidNiObject(skinBoneObject)) {
					spdlog::warn(
						"skipping invalid skin bone pointer={} slot={} while rebinding geometry '{}'",
						static_cast<void*>(skinBoneObject),
						index,
						geometry->GetName());
					continue;
				}
				auto* skinBone = skinBoneObject ? skinBoneObject->IsNode() : nullptr;
				if (!skinBone) {
					continue;
				}

				const auto name = skinBoneObject->GetName();
				if (name.empty()) {
					continue;
				}

				auto* matched = FindMatchedSkinBoneByMergedName(a_matchedBones, a_renamedNodes, name);
				if (!matched) {
					continue;
				}

				if (matched->node) {
					AddSkinWorldTransformSlot(*matched, skin, index);
					const auto rebound = RebindSkinBoneSlot(skin, index, matched->node);
					if (rebound && a_skeletonRoot) {
						skin->rootNode = a_skeletonRoot;
					}
					if (rebound && skinBone != matched->node) {
						spdlog::debug(
							"prototype rebound skin bone '{}' slot={} oldNode={} newNode={} skin={} root={}",
							matched->name,
							index,
							static_cast<void*>(skinBone),
							static_cast<void*>(matched->node),
							static_cast<void*>(skin),
							static_cast<void*>(a_skeletonRoot));
					}
				}
			}
			return;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			RebindMatchedSkinSlots(a_matchedBones, a_renamedNodes, child.get(), a_skeletonRoot);
		}
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
		for (int index = 0; index < a_world.getNumCollisionObjects(); ++index) {
			auto* body = btRigidBody::upcast(a_world.getCollisionObjectArray()[index]);
			if (!body) {
				continue;
			}

			center += body->getWorldTransform().getOrigin();
			++count;
		}

		if (count <= 0) {
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
			auto constraint = a_descriptor.useLinearReferenceFrameA ?
				std::make_unique<btGeneric6DofSpring2Constraint>(a_bodyB, a_bodyA, btTransform::getIdentity(), btTransform::getIdentity(), RO_XYZ) :
				std::make_unique<btGeneric6DofSpring2Constraint>(a_bodyA, a_bodyB, btTransform::getIdentity(), btTransform::getIdentity(), RO_XYZ);
			if (a_descriptor.useLinearReferenceFrameA) {
				constraint->setFrames(frameB, frameA);
			} else {
				constraint->setFrames(frameA, frameB);
			}
			constraint->setLinearLowerLimit(ToBulletVector(a_descriptor.linearLowerLimit));
			constraint->setLinearUpperLimit(ToBulletVector(a_descriptor.linearUpperLimit));
			constraint->setAngularLowerLimit(ToBulletVector(a_descriptor.angularLowerLimit));
			constraint->setAngularUpperLimit(ToBulletVector(a_descriptor.angularUpperLimit));
			for (int axis = 0; axis < 3; ++axis) {
				constraint->enableSpring(axis, a_descriptor.enableLinearSprings);
				constraint->enableSpring(axis + 3, a_descriptor.enableAngularSprings);
				constraint->setStiffness(axis, AxisValue(a_descriptor.linearStiffness, axis), a_descriptor.linearStiffnessLimited);
				constraint->setStiffness(axis + 3, AxisValue(a_descriptor.angularStiffness, axis), a_descriptor.angularStiffnessLimited);
				constraint->setDamping(axis, AxisValue(a_descriptor.linearDamping, axis), a_descriptor.springDampingLimited);
				constraint->setDamping(axis + 3, AxisValue(a_descriptor.angularDamping, axis), a_descriptor.springDampingLimited);
				constraint->setNonHookeanDamping(axis, AxisValue(a_descriptor.linearNonHookeanDamping, axis));
				constraint->setNonHookeanDamping(axis + 3, AxisValue(a_descriptor.angularNonHookeanDamping, axis));
				constraint->setNonHookeanStiffness(axis, AxisValue(a_descriptor.linearNonHookeanStiffness, axis));
				constraint->setNonHookeanStiffness(axis + 3, AxisValue(a_descriptor.angularNonHookeanStiffness, axis));
				constraint->setEquilibriumPoint(axis, AxisValue(a_descriptor.linearEquilibrium, axis));
				constraint->setEquilibriumPoint(axis + 3, AxisValue(a_descriptor.angularEquilibrium, axis));
				constraint->setBounce(axis, AxisValue(a_descriptor.linearBounce, axis));
				constraint->setBounce(axis + 3, AxisValue(a_descriptor.angularBounce, axis));
				constraint->enableMotor(axis, a_descriptor.linearMotors);
				constraint->enableMotor(axis + 3, a_descriptor.angularMotors);
				constraint->setServo(axis, a_descriptor.linearServoMotors);
				constraint->setServo(axis + 3, a_descriptor.angularServoMotors);
				constraint->setServoTarget(axis, AxisValue(a_descriptor.linearEquilibrium, axis));
				constraint->setServoTarget(axis + 3, AxisValue(a_descriptor.angularEquilibrium, axis));
				constraint->setTargetVelocity(axis, AxisValue(a_descriptor.linearTargetVelocity, axis));
				constraint->setTargetVelocity(axis + 3, AxisValue(a_descriptor.angularTargetVelocity, axis));
				constraint->setMaxMotorForce(axis, AxisValue(a_descriptor.linearMaxMotorForce, axis));
				constraint->setMaxMotorForce(axis + 3, AxisValue(a_descriptor.angularMaxMotorForce, axis));
				constraint->setParam(BT_CONSTRAINT_ERP, a_descriptor.motorErp, axis);
				constraint->setParam(BT_CONSTRAINT_ERP, a_descriptor.motorErp, axis + 3);
				constraint->setParam(BT_CONSTRAINT_CFM, a_descriptor.motorCfm, axis);
				constraint->setParam(BT_CONSTRAINT_CFM, a_descriptor.motorCfm, axis + 3);
				constraint->setParam(BT_CONSTRAINT_STOP_ERP, a_descriptor.stopErp, axis);
				constraint->setParam(BT_CONSTRAINT_STOP_ERP, a_descriptor.stopErp, axis + 3);
				constraint->setParam(BT_CONSTRAINT_STOP_CFM, a_descriptor.stopCfm, axis);
				constraint->setParam(BT_CONSTRAINT_STOP_CFM, a_descriptor.stopCfm, axis + 3);
				if (auto* motor = constraint->getRotationalLimitMotor(axis)) {
					motor->m_motorERP = a_descriptor.motorErp;
					motor->m_motorCFM = a_descriptor.motorCfm;
					motor->m_stopERP = a_descriptor.stopErp;
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

namespace Smp
{
	Fo4PhysicsWorld::~Fo4PhysicsWorld()
	{
		Reset();
	}

	Fo4PhysicsWorld* Fo4PhysicsWorld::GetSingleton()
	{
		static Fo4PhysicsWorld singleton;
		return std::addressof(singleton);
	}

	void Fo4PhysicsWorld::Register()
	{
		if (registered_) {
			return;
		}
		if (Initialize()) {
			GetLifecycleEventSource().RegisterSink(static_cast<RE::BSTEventSink<LifecycleEvent>*>(this));
			if (auto* ui = RE::UI::GetSingleton()) {
				ui->RegisterSink<RE::MenuOpenCloseEvent>(static_cast<RE::BSTEventSink<RE::MenuOpenCloseEvent>*>(this));
			} else {
				spdlog::warn("FO4 Faster HDT-SMP could not register UI menu sink; LooksMenu rebuild deferral will be unavailable");
			}
			registered_ = true;
		}
	}

	bool Fo4PhysicsWorld::Initialize()
	{
		std::scoped_lock lock(lock_);
		return InitializeLocked();
	}

	void Fo4PhysicsWorld::ApplyConfig(const RuntimeSettings& a_settings)
	{
		std::scoped_lock lock(lock_);
		solverIterations_ = a_settings.solver.numIterations;
		solverErp_ = a_settings.solver.erp;
		maxSubSteps_ = a_settings.solver.maxSubSteps;
		fixedStepSeconds_ = 1.0F / static_cast<float>(a_settings.solver.minFps);
		ResetStepClockLocked();
		useRealTime_ = a_settings.smp.useRealTime;
		budgetMs_ = std::max(a_settings.smp.budgetMs, 0.1F);
		sampleSize_ = std::max(a_settings.smp.sampleSize, 1);
		metricFrameInterval_ = static_cast<std::uint32_t>(std::max(a_settings.solver.minFps, 1));
		Fo4SkinnedMeshBone::ApplyStabilityConfig(
			a_settings.smp.clampRotations,
			a_settings.smp.rotationSpeedLimit,
			a_settings.smp.unclampedResets,
			a_settings.smp.unclampedResetAngle);
		disableFirstPersonViewPhysics_ = a_settings.smp.disableFirstPersonViewPhysics;
		disableSMPHairWhenWigEquipped_ = a_settings.smp.disableSMPHairWhenWigEquipped;
		enableNpcPhysics_ = a_settings.smp.enableNpcPhysics;
		autoAdjustMaxActors_ = a_settings.smp.autoAdjustMaxActors;
		maxActiveActors_ = static_cast<std::size_t>(std::max(a_settings.smp.maxActiveActors, 1));
		currentMaxActiveActors_ = maxActiveActors_;
		maxActorDistance_ = std::max(a_settings.smp.maxActorDistance, 0.0F);
		prototypePhysicsXml_.clear();
		if (!a_settings.smp.prototypePhysicsXml.empty()) {
			if (auto resolved = ConfigPaths::ResolveExistingConfigPath(a_settings.smp.prototypePhysicsXml, true)) {
				prototypePhysicsXml_ = resolved->string();
			} else {
				spdlog::warn("prototype physics XML fallback disabled because configured path is missing or not XML: {}", a_settings.smp.prototypePhysicsXml);
			}
		}
		windEnabled_ = a_settings.wind.enabled;
		windUseWeather_ = a_settings.wind.useWeather;
		windStrength_ = std::max(a_settings.wind.windStrength, 0.0F);
		windDistanceForNoWind_ = std::max(a_settings.wind.distanceForNoWind, 0.0F);
		windDistanceForMaxWind_ = std::max(a_settings.wind.distanceForMaxWind, windDistanceForNoWind_);
		windWeatherShortCooldownSeconds_ = std::max(a_settings.wind.weatherShortCooldownSeconds, 0.0F);
		windWeatherLongCooldownSeconds_ = std::max(a_settings.wind.weatherLongCooldownSeconds, windWeatherShortCooldownSeconds_);
		windSmoothingSamples_ = std::max(a_settings.wind.smoothingSamples, 1);
		randomizePerBoneWind_ = a_settings.wind.randomizePerBoneWind;
		windDirection_ = btVector3(a_settings.wind.directionX, a_settings.wind.directionY, a_settings.wind.directionZ);
		if (windDirection_.length2() > SIMD_EPSILON) {
			windDirection_.normalize();
		} else {
			windDirection_ = btVector3(1.0F, 0.0F, 0.0F);
		}

		if (dynamicsWorld_) {
			auto& solverInfo = dynamicsWorld_->getSolverInfo();
			solverInfo.m_numIterations = solverIterations_;
			solverInfo.m_erp = solverErp_;
		}
		EnforceActorBudgetLocked();
	}

	void Fo4PhysicsWorld::Reset()
	{
		std::scoped_lock lock(lock_);
		ResetLocked();
	}

	void Fo4PhysicsWorld::ResetStepClockLocked()
	{
		averageInterval_ = fixedStepSeconds_;
		accumulatedInterval_ = 0.0F;
		currentStepSeconds_ = fixedStepSeconds_;
	}

	void Fo4PhysicsWorld::StepFrame()
	{
		auto delta = fixedStepSeconds_;
		if (const auto timer = RE::BSTimer::GetSingleton()) {
			delta = useRealTime_ ? timer->realTimeDelta : timer->delta;
		}
		if (delta <= 0.0F || !std::isfinite(delta)) {
			return;
		}

		if (IsGamePaused()) {
			std::scoped_lock lock(lock_);
			ResetStepClockLocked();
			return;
		}

		float remainingTimeStep = 0.0F;
		{
			std::scoped_lock lock(lock_);
			if (loadingPhysicsSuspended_) {
				ResumeFromLoadingMenuLocked();
				return;
			}
			if (characterCustomizationMenuDepth_ > 0) {
				ResetStepClockLocked();
				return;
			}

			accumulatedInterval_ += delta;
			averageInterval_ += (delta - averageInterval_) * 0.125F;
			currentStepSeconds_ = std::min(averageInterval_, fixedStepSeconds_);

			if (accumulatedInterval_ * 2.0F <= currentStepSeconds_) {
				return;
			}

			remainingTimeStep = std::min(
				accumulatedInterval_,
				currentStepSeconds_ * static_cast<float>(std::max(maxSubSteps_, 1)));
			accumulatedInterval_ = 0.0F;
		}

		const auto start = Clock::now();
		Step(remainingTimeStep);
		RecordFrameMetrics(ElapsedMs(start, Clock::now()));
	}

	void Fo4PhysicsWorld::Step(const float a_deltaSeconds)
	{
		std::scoped_lock lock(lock_);
		if (!dynamicsWorld_ || a_deltaSeconds <= 0.0F) {
			return;
		}

		PruneInvalidPrototypeStatesLocked();
		TryReactivateSuspendedActorsLocked();
		TryReactivateSuspendedPrototypeStatesLocked();
		if ((!pendingActorRebuilds_.empty() || !pendingHeadRebuilds_.empty()) && simulationFrame_ >= nextPendingRebuildFrame_) {
			nextPendingRebuildFrame_ = simulationFrame_ + kPendingRebuildRetryIntervalFrames;
			SchedulePendingRebuildTaskLocked();
		}

		if (disableFirstPersonViewPhysics_ && IsPlayerFirstPersonView()) {
			const auto* player = RE::PlayerCharacter::GetSingleton();
			bool suspendedPlayer = false;
			for (auto& actorState : prototypeActors_) {
				if (actorState.actor == player) {
					actorState.resetReadFrames = std::max(actorState.resetReadFrames, kAttachResetReadFrames);
					suspendedPlayer = true;
				}
			}
			if (suspendedPlayer) {
				ResetStepClockLocked();
				return;
			}
		}

		auto phaseStart = Clock::now();
		for (auto& actorState : prototypeActors_) {
			if (actorState.runtimeSoftSuspended) {
				continue;
			}
			UpdateMeshDisableStatesLocked(actorState);
			const auto resettingRead = actorState.resetReadFrames > 0;
			const auto readDelta = PreparePrototypeActorForReadLocked(actorState, actorState.resetReadFrames > 0 ? 0.0F : a_deltaSeconds);
			if (!actorState.runtimes.empty()) {
				for (const auto& runtime : actorState.runtimes) {
					for (auto* bone : runtime.bones) {
						if (bone) {
							bone->readTransform(readDelta);
						}
					}
					ScalePrototypeConstraintsLocked(actorState, runtime);
				}
			} else {
				for (auto& prototypeBody : actorState.bodies) {
					if (prototypeBody.bone) {
						prototypeBody.bone->readTransform(readDelta);
					}
				}
				ScalePrototypeConstraintsLocked(actorState);
			}
			if (actorState.resetReadFrames > 0) {
				--actorState.resetReadFrames;
			}
			if (resettingRead) {
				if (dynamicsWorld_) {
					dynamicsWorld_->clearForces();
				}
				ResetStepClockLocked();
				return;
			}
		}
		const auto readMs = ElapsedMs(phaseStart, Clock::now());

		phaseStart = Clock::now();
		UpdateWindLocked();
		ApplyWindForcesLocked();
		const auto windMs = ElapsedMs(phaseStart, Clock::now());

		const auto fixedStepSeconds = std::clamp(currentStepSeconds_, kMinimumStepSeconds, fixedStepSeconds_);
		const auto maximumStepSeconds = std::max(fixedStepSeconds, fixedStepSeconds * static_cast<float>(std::max(maxSubSteps_, 1)));
		const auto clampedDelta = std::clamp(a_deltaSeconds, kMinimumStepSeconds, maximumStepSeconds);

		ResetFrameCollisionProfile();
		phaseStart = Clock::now();
		const auto translationOffset = ApplyTranslationOffset(*dynamicsWorld_);
		if (auto* world = static_cast<PrototypeDynamicsWorld*>(dynamicsWorld_.get())) {
			world->StepReference(clampedDelta, fixedStepSeconds);
		}
		RestoreTranslationOffset(*dynamicsWorld_, translationOffset);
		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}
		const auto bulletMs = ElapsedMs(phaseStart, Clock::now());
		std::uint32_t collisionCalls = 0;
		const auto collisionMs = ConsumeFrameCollisionProfile(collisionCalls);

		pendingStepReadMs_ += readMs;
		pendingStepWindMs_ += windMs;
		pendingStepBulletMs_ += bulletMs;
		pendingStepCollisionMs_ += collisionMs;
		pendingStepCollisionCalls_ += collisionCalls;

		++simulationFrame_;
		if (simulationFrame_ == 0) {
			simulationFrame_ = 1;
		}
	}

	void Fo4PhysicsWorld::LogRootConstraintDiagnosticsLocked(const std::string_view a_phase, const PrototypeActorState& a_state)
	{
		const PrototypeConstraint* selected = nullptr;
		for (const auto& prototypeConstraint : a_state.constraints) {
			if (!prototypeConstraint.constraint || prototypeConstraint.kind != PhysicsConstraintKind::kGeneric) {
				continue;
			}

			const auto& bodyA = prototypeConstraint.constraint->getRigidBodyA();
			const auto& bodyB = prototypeConstraint.constraint->getRigidBodyB();
			if (bodyA.isStaticOrKinematicObject() != bodyB.isStaticOrKinematicObject()) {
				selected = std::addressof(prototypeConstraint);
				break;
			}
			if (!selected) {
				selected = std::addressof(prototypeConstraint);
			}
		}
		if (!selected || !selected->constraint) {
			return;
		}

		auto* generic = static_cast<btGeneric6DofSpring2Constraint*>(selected->constraint.get());
		const auto& bodyA = generic->getRigidBodyA();
		const auto& bodyB = generic->getRigidBodyB();
		const auto bodyATransform = bodyA.getWorldTransform();
		const auto bodyBTransform = bodyB.getWorldTransform();
		const auto anchorA = bodyATransform * generic->getFrameOffsetA();
		const auto anchorB = bodyBTransform * generic->getFrameOffsetB();
		const auto bodyAOrigin = bodyATransform.getOrigin();
		const auto bodyBOrigin = bodyBTransform.getOrigin();
		const auto anchorAOrigin = anchorA.getOrigin();
		const auto anchorBOrigin = anchorB.getOrigin();
		const auto anchorDelta = anchorBOrigin - anchorAOrigin;

		spdlog::info(
			"prototype root constraint diagnostic {} actor={} bodies='{}'/'{}' enabled={} bodyAkin={} bodyBkin={} bodyA=({:.3f},{:.3f},{:.3f}) bodyB=({:.3f},{:.3f},{:.3f}) anchorA=({:.3f},{:.3f},{:.3f}) anchorB=({:.3f},{:.3f},{:.3f}) anchorDelta=({:.3f},{:.3f},{:.3f}) velA=({:.3f},{:.3f},{:.3f}) velB=({:.3f},{:.3f},{:.3f})",
			a_phase,
			static_cast<void*>(a_state.actor),
			selected->bodyA,
			selected->bodyB,
			generic->isEnabled(),
			bodyA.isStaticOrKinematicObject(),
			bodyB.isStaticOrKinematicObject(),
			bodyAOrigin.x(),
			bodyAOrigin.y(),
			bodyAOrigin.z(),
			bodyBOrigin.x(),
			bodyBOrigin.y(),
			bodyBOrigin.z(),
			anchorAOrigin.x(),
			anchorAOrigin.y(),
			anchorAOrigin.z(),
			anchorBOrigin.x(),
			anchorBOrigin.y(),
			anchorBOrigin.z(),
			anchorDelta.x(),
			anchorDelta.y(),
			anchorDelta.z(),
			bodyA.getLinearVelocity().x(),
			bodyA.getLinearVelocity().y(),
			bodyA.getLinearVelocity().z(),
			bodyB.getLinearVelocity().x(),
			bodyB.getLinearVelocity().y(),
			bodyB.getLinearVelocity().z());
	}

	void Fo4PhysicsWorld::UpdateMeshDisableStatesLocked(PrototypeActorState& a_state)
	{
		struct DisableGroup
		{
			RE::BSFixedString tag;
			std::vector<hdt::SkinnedMeshBody*> bodies;
		};

		std::vector<RE::BSFixedString> activeTags;
		std::vector<DisableGroup> disableGroups;
		const auto disableHairForWig = disableSMPHairWhenWigEquipped_ && HasEquippedHairSlotObject(a_state.actor);

		for (auto& prototypeMesh : a_state.meshes) {
			auto* body = prototypeMesh.body.get();
			if (!body) {
				continue;
			}

			body->disabled_ = false;
			if (disableHairForWig && prototypeMesh.domain == PrototypeBuildDomain::kHair) {
				body->disabled_ = true;
				continue;
			}
			if (body->disableTag_.empty()) {
				for (const auto& tag : body->tags_) {
					if (std::ranges::find(activeTags, tag) == activeTags.end()) {
						activeTags.push_back(tag);
					}
				}
				continue;
			}

			const auto foundGroup = std::ranges::find_if(disableGroups, [body](const DisableGroup& a_group) {
				return a_group.tag == body->disableTag_;
			});
			if (foundGroup != disableGroups.end()) {
				foundGroup->bodies.push_back(body);
			} else {
				auto& group = disableGroups.emplace_back();
				group.tag = body->disableTag_;
				group.bodies.push_back(body);
			}
		}

		for (auto& group : disableGroups) {
			if (std::ranges::find(activeTags, group.tag) != activeTags.end()) {
				for (auto* body : group.bodies) {
					body->disabled_ = true;
				}
				continue;
			}

			std::ranges::sort(group.bodies, [](const hdt::SkinnedMeshBody* a_lhs, const hdt::SkinnedMeshBody* a_rhs) {
				if (a_lhs->disablePriority_ != a_rhs->disablePriority_) {
					return a_lhs->disablePriority_ > a_rhs->disablePriority_;
				}
				return a_lhs < a_rhs;
			});
			for (auto* body : group.bodies) {
				body->disabled_ = true;
			}
			if (!group.bodies.empty()) {
				group.bodies.front()->disabled_ = false;
			}
		}
	}

	void Fo4PhysicsWorld::UpdateWindLocked()
	{
		if (!windEnabled_ || windStrength_ <= 0.0F) {
			currentWind_.setZero();
			targetWind_.setZero();
			return;
		}

		auto direction = windDirection_;
		auto strength = windStrength_;
		if (windUseWeather_) {
			const auto* sky = RE::Sky::GetSingleton();
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* cell = player ? player->GetParentCell() : nullptr;
			if (!IsWeatherWindSkyValid(sky) || !player || !cell || !cell->IsExterior() || !cell->worldSpace) {
				ClearWindState(currentWind_, targetWind_, windWeatherCooldown_, windWeatherLongCooldownSeconds_);
				return;
			}

			windWeatherCooldown_ -= currentStepSeconds_;
			if (windWeatherCooldown_ <= 0.0F) {
				direction = WindDirectionFromFo4SkyAngle(sky->windAngle);
				strength *= std::max(sky->windSpeed, 0.0F);
				targetWind_ = direction * strength * kGameUnitsPerMeter;
				windWeatherCooldown_ = windWeatherShortCooldownSeconds_;
			}
		} else {
			targetWind_ = direction * strength * kGameUnitsPerMeter;
		}

		const auto smoothingSamples = static_cast<float>(std::max(windSmoothingSamples_, 1));
		if (smoothingSamples <= 1.0F) {
			currentWind_ = targetWind_;
		} else {
			currentWind_ += (targetWind_ - currentWind_) / smoothingSamples;
		}
	}

	void Fo4PhysicsWorld::ApplyWindForcesLocked()
	{
		if (currentWind_.length2() <= SIMD_EPSILON) {
			for (auto& actorState : prototypeActors_) {
				actorState.currentWindFactor = 0.0F;
			}
			return;
		}

		for (auto& actorState : prototypeActors_) {
			if (actorState.runtimeSoftSuspended) {
				actorState.currentWindFactor = 0.0F;
				continue;
			}
			if (windUseWeather_ && !IsActorWeatherWindCellValid(actorState.actor)) {
				actorState.currentWindFactor = 0.0F;
				continue;
			}

			const auto targetActorWindScale = ResolveActorWindObstructionFactor(actorState.actor, currentWind_, windDistanceForNoWind_, windDistanceForMaxWind_);
			actorState.currentWindFactor += (targetActorWindScale - actorState.currentWindFactor) / static_cast<float>(std::max(windSmoothingSamples_, 1));
			if (std::abs(actorState.currentWindFactor - targetActorWindScale) <= 0.001F) {
				actorState.currentWindFactor = targetActorWindScale;
			}
			const auto actorWindScale = std::clamp(actorState.currentWindFactor, 0.0F, 1.0F);
			if (actorWindScale <= 0.0F) {
				continue;
			}

			const auto actorWind = currentWind_ * actorWindScale;
			for (auto& prototypeBody : actorState.bodies) {
				if (!prototypeBody.bone || prototypeBody.bone->m_windFactor <= 0.0F || prototypeBody.bone->m_rig.isStaticOrKinematicObject()) {
					continue;
				}

				const auto boneWind = randomizePerBoneWind_ ?
					actorWind * StableWindVariation(prototypeBody.boneName) :
					actorWind;
				prototypeBody.bone->m_rig.applyCentralForce(boneWind * prototypeBody.bone->m_windFactor);
			}
		}
	}

	void Fo4PhysicsWorld::RecordFrameMetrics(const float a_stepMs)
	{
		std::scoped_lock lock(lock_);
		if (prototypeActors_.empty()) {
			pendingWritebackMs_ = 0.0F;
			pendingMainSyncMs_ = 0.0F;
			pendingStepReadMs_ = 0.0F;
			pendingStepWindMs_ = 0.0F;
			pendingStepBulletMs_ = 0.0F;
			pendingStepCollisionMs_ = 0.0F;
			pendingStepCollisionCalls_ = 0;
			currentMaxActiveActors_ = maxActiveActors_;
			metricFrameCounter_ = 0;
			averageStepMs_ = 0.0F;
			averageWritebackMs_ = 0.0F;
			averageMainSyncMs_ = 0.0F;
			averageStepReadMs_ = 0.0F;
			averageStepWindMs_ = 0.0F;
			averageStepBulletMs_ = 0.0F;
			averageStepCollisionMs_ = 0.0F;
			return;
		}

		const auto sampleWeight = static_cast<float>(sampleSize_);
		averageStepMs_ = ((averageStepMs_ * (sampleWeight - 1.0F)) + std::max(a_stepMs, 0.0F)) / sampleWeight;
		averageWritebackMs_ = ((averageWritebackMs_ * (sampleWeight - 1.0F)) + std::max(pendingWritebackMs_, 0.0F)) / sampleWeight;
		averageMainSyncMs_ = ((averageMainSyncMs_ * (sampleWeight - 1.0F)) + std::max(pendingMainSyncMs_, 0.0F)) / sampleWeight;
		averageStepReadMs_ = ((averageStepReadMs_ * (sampleWeight - 1.0F)) + std::max(pendingStepReadMs_, 0.0F)) / sampleWeight;
		averageStepWindMs_ = ((averageStepWindMs_ * (sampleWeight - 1.0F)) + std::max(pendingStepWindMs_, 0.0F)) / sampleWeight;
		averageStepBulletMs_ = ((averageStepBulletMs_ * (sampleWeight - 1.0F)) + std::max(pendingStepBulletMs_, 0.0F)) / sampleWeight;
		averageStepCollisionMs_ = ((averageStepCollisionMs_ * (sampleWeight - 1.0F)) + std::max(pendingStepCollisionMs_, 0.0F)) / sampleWeight;
		pendingWritebackMs_ = 0.0F;
		pendingMainSyncMs_ = 0.0F;
		pendingStepReadMs_ = 0.0F;
		pendingStepWindMs_ = 0.0F;
		pendingStepBulletMs_ = 0.0F;
		pendingStepCollisionMs_ = 0.0F;

		++metricFrameCounter_;
		if (metricFrameCounter_ < metricFrameInterval_) {
			return;
		}
		metricFrameCounter_ = 0;
		const auto collisionCalls = pendingStepCollisionCalls_;
		pendingStepCollisionCalls_ = 0;

		std::size_t bodyCount = 0;
		std::size_t meshCount = 0;
		std::size_t activeActors = 0;
		for (const auto& actorState : prototypeActors_) {
			bodyCount += actorState.bodies.size();
			meshCount += actorState.meshes.size();
			if (actorState.HasActiveRuntime()) {
				++activeActors;
			}
		}

		const auto totalMs = averageStepMs_ + averageWritebackMs_;
		if (autoAdjustMaxActors_) {
			if (totalMs > budgetMs_ && currentMaxActiveActors_ > 1) {
				--currentMaxActiveActors_;
			} else if (totalMs < budgetMs_ && currentMaxActiveActors_ < maxActiveActors_) {
				const auto averagePerActor = activeActors > 0 ? totalMs / static_cast<float>(activeActors) : 0.0F;
				const auto headroom = budgetMs_ - totalMs;
				const auto canAdd = averagePerActor > 0.0F ? static_cast<std::size_t>(std::max(headroom / averagePerActor, 0.0F)) : 2U;
				currentMaxActiveActors_ += std::clamp<std::size_t>(canAdd, 0, 2);
				currentMaxActiveActors_ = std::min(currentMaxActiveActors_, maxActiveActors_);
			}
		} else {
			currentMaxActiveActors_ = maxActiveActors_;
		}
		EnforceActorBudgetLocked();

		if (totalMs > budgetMs_) {
			spdlog::debug(
				"[SMP Metrics] activeActors={} actorCap={}/{} bodies={} meshes={} avgFrameImpact={:.3f}ms budget={:.3f}ms step={:.3f}ms sync={:.3f}ms writeback={:.3f}ms stepRead={:.3f}ms stepWind={:.3f}ms stepBullet={:.3f}ms collision={:.3f}ms collisionCalls={} writes(mainSync/cellJobs/postAnim)={}/{}/{} duplicateSkips(cellJobs/postAnim)={}/{}",
				activeActors,
				currentMaxActiveActors_,
				maxActiveActors_,
				bodyCount,
				meshCount,
				totalMs,
				budgetMs_,
				averageStepMs_,
				averageMainSyncMs_,
				averageWritebackMs_,
				averageStepReadMs_,
				averageStepWindMs_,
				averageStepBulletMs_,
				averageStepCollisionMs_,
				collisionCalls,
				mainSyncWritebacks_,
				cellJobsWritebacks_,
				postAnimationWritebacks_,
				duplicateCellJobsWritebacks_,
				duplicatePostAnimationWritebacks_);
		} else {
			spdlog::trace(
				"[SMP Metrics] activeActors={} actorCap={}/{} bodies={} meshes={} avgFrameImpact={:.3f}ms budget={:.3f}ms step={:.3f}ms sync={:.3f}ms writeback={:.3f}ms stepRead={:.3f}ms stepWind={:.3f}ms stepBullet={:.3f}ms collision={:.3f}ms collisionCalls={} writes(mainSync/cellJobs/postAnim)={}/{}/{} duplicateSkips(cellJobs/postAnim)={}/{}",
				activeActors,
				currentMaxActiveActors_,
				maxActiveActors_,
				bodyCount,
				meshCount,
				totalMs,
				budgetMs_,
				averageStepMs_,
				averageMainSyncMs_,
				averageWritebackMs_,
				averageStepReadMs_,
				averageStepWindMs_,
				averageStepBulletMs_,
				averageStepCollisionMs_,
				collisionCalls,
				mainSyncWritebacks_,
				cellJobsWritebacks_,
				postAnimationWritebacks_,
				duplicateCellJobsWritebacks_,
				duplicatePostAnimationWritebacks_);
		}

		mainSyncWritebacks_ = 0;
		cellJobsWritebacks_ = 0;
		postAnimationWritebacks_ = 0;
		duplicateCellJobsWritebacks_ = 0;
		duplicatePostAnimationWritebacks_ = 0;
	}

	void Fo4PhysicsWorld::RecordWritebackMetric(
		const float a_writebackMs,
		const WritebackSource a_source,
		const bool a_wroteAny,
		const bool a_skippedDuplicate)
	{
		std::scoped_lock lock(lock_);
		if (a_wroteAny) {
			pendingWritebackMs_ += std::max(a_writebackMs, 0.0F);
			if (a_source == WritebackSource::kMainSync) {
				pendingMainSyncMs_ += std::max(a_writebackMs, 0.0F);
				++mainSyncWritebacks_;
			}
			IncrementWritebackCounter(a_source, cellJobsWritebacks_, postAnimationWritebacks_);
		}
		if (a_skippedDuplicate) {
			IncrementWritebackCounter(a_source, duplicateCellJobsWritebacks_, duplicatePostAnimationWritebacks_);
		}
	}

	void Fo4PhysicsWorld::WriteBackPrototypeBodies(const WritebackSource a_source)
	{
		const auto start = Clock::now();
		bool wroteAny = false;
		bool skippedDuplicate = false;
		{
			std::scoped_lock lock(lock_);
			if (loadingPhysicsSuspended_ || characterCustomizationMenuDepth_ > 0) {
				ResetStepClockLocked();
				return;
			}

			PruneInvalidPrototypeStatesLocked();

			for (auto& actorState : prototypeActors_) {
				if (actorState.runtimeSoftSuspended) {
					continue;
				}
				if (!CanWriteBackFrame(actorState.lastWritebackFrame, a_source, simulationFrame_)) {
					skippedDuplicate = true;
					continue;
				}
				actorState.lastWritebackFrame = simulationFrame_;
				actorState.lastWritebackSource = a_source;
				std::vector<RE::NiNode*> writtenNodes;
				const auto writeBody = [&](PrototypeBody& prototypeBody) {
					if (!prototypeBody.bone || prototypeBody.bone->m_rig.isKinematicObject()) {
						return;
					}
					if (prototypeBody.node && std::ranges::find(writtenNodes, prototypeBody.node) != writtenNodes.end()) {
						skippedDuplicate = true;
						spdlog::warn(
							"skipping duplicate dynamic prototype writeback actor={} node={} nodeName='{}' buildGroup={} bone='{}' source={}",
							static_cast<void*>(actorState.actor),
							static_cast<void*>(prototypeBody.node),
							std::string_view(prototypeBody.node->GetName()),
							prototypeBody.buildGroup,
							prototypeBody.boneName,
							WritebackSourceName(a_source));
						return;
					}
					prototypeBody.bone->writeTransform();
					if (prototypeBody.node) {
						writtenNodes.push_back(prototypeBody.node);
					}
					wroteAny = true;
				};
				if (!actorState.runtimes.empty()) {
					for (const auto& runtime : actorState.runtimes) {
						for (auto* bone : runtime.bones) {
							auto body = std::ranges::find_if(actorState.bodies, [bone](const PrototypeBody& a_body) {
								return a_body.bone.get() == bone;
							});
							if (body == actorState.bodies.end()) {
								continue;
							}
							writeBody(*body);
						}
					}
				} else {
					for (auto& prototypeBody : actorState.bodies) {
						writeBody(prototypeBody);
					}
				}
			}
		}
		RecordWritebackMetric(ElapsedMs(start, Clock::now()), a_source, wroteAny, skippedDuplicate);
	}

	void Fo4PhysicsWorld::WriteBackPrototypeBodies(RE::Actor* a_actor, const WritebackSource a_source)
	{
		const auto start = Clock::now();
		bool wroteAny = false;
		bool skippedDuplicate = false;
		{
			std::scoped_lock lock(lock_);
			if (loadingPhysicsSuspended_ || characterCustomizationMenuDepth_ > 0) {
				ResetStepClockLocked();
				return;
			}

			PruneInvalidPrototypeStatesLocked();

			for (auto& actorState : prototypeActors_) {
				if (actorState.actor != a_actor) {
					continue;
				}
				if (actorState.runtimeSoftSuspended) {
					continue;
				}

				if (CanWriteBackFrame(actorState.lastWritebackFrame, a_source, simulationFrame_)) {
					actorState.lastWritebackFrame = simulationFrame_;
					actorState.lastWritebackSource = a_source;
					std::vector<RE::NiNode*> writtenNodes;
					const auto writeBody = [&](PrototypeBody& prototypeBody) {
						if (!prototypeBody.bone || prototypeBody.bone->m_rig.isKinematicObject()) {
							return;
						}
						if (prototypeBody.node && std::ranges::find(writtenNodes, prototypeBody.node) != writtenNodes.end()) {
							skippedDuplicate = true;
							spdlog::warn(
								"skipping duplicate dynamic prototype writeback actor={} node={} nodeName='{}' buildGroup={} bone='{}' source={}",
								static_cast<void*>(actorState.actor),
								static_cast<void*>(prototypeBody.node),
								std::string_view(prototypeBody.node->GetName()),
								prototypeBody.buildGroup,
								prototypeBody.boneName,
								WritebackSourceName(a_source));
							return;
						}
						prototypeBody.bone->writeTransform();
						if (prototypeBody.node) {
							writtenNodes.push_back(prototypeBody.node);
						}
						wroteAny = true;
					};
					if (!actorState.runtimes.empty()) {
						for (const auto& runtime : actorState.runtimes) {
							for (auto* bone : runtime.bones) {
								auto body = std::ranges::find_if(actorState.bodies, [bone](const PrototypeBody& a_body) {
									return a_body.bone.get() == bone;
								});
								if (body == actorState.bodies.end()) {
									continue;
								}
								writeBody(*body);
							}
						}
					} else {
						for (auto& prototypeBody : actorState.bodies) {
							writeBody(prototypeBody);
						}
					}
				} else {
					skippedDuplicate = true;
				}
			}
		}
		RecordWritebackMetric(ElapsedMs(start, Clock::now()), a_source, wroteAny, skippedDuplicate);
	}

	void Fo4PhysicsWorld::ProcessPendingRebuilds()
	{
		std::scoped_lock lock(lock_);
		pendingRebuildTaskQueued_ = false;
		if (characterCustomizationMenuDepth_ > 0) {
			return;
		}

		if (pendingActorRebuilds_.empty() && pendingHeadRebuilds_.empty()) {
			return;
		}

		TryRebuildPendingActorsLocked();
		TryRebuildPendingHeadsLocked();
	}

	RE::BSEventNotifyControl Fo4PhysicsWorld::ProcessEvent(const LifecycleEvent& a_event, RE::BSTEventSource<LifecycleEvent>*)
	{
		if (IsAttachCandidate(a_event.type)) {
			if (a_event.type == LifecycleEventType::kActorSet3D && !a_event.object) {
				std::scoped_lock lock(lock_);
				bool clearedActorState = false;
				bool preservedForCustomization = false;
				for (auto& actorState : prototypeActors_) {
					if (actorState.actor != a_event.actor) {
						continue;
					}
					if (characterCustomizationMenuDepth_ > 0 || !actorState.armorRecords.empty()) {
						SuspendPrototypeRuntimeLocked(actorState);
						preservedForCustomization = true;
						continue;
					}
					clearedActorState = true;
					ClearPrototypeStateLocked(actorState);
					actorState.actor = nullptr;
					actorState.actorHandle.reset();
				}
				std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
					return !a_state.actor && !a_state.HasRuntime();
				});
				if (preservedForCustomization) {
					spdlog::debug("physics world preserved actor state for null Set3D actor={} because SMP armor records are tracked or customization is active", static_cast<void*>(a_event.actor));
				} else if (clearedActorState) {
					spdlog::debug("physics world cleared actor state for null Set3D actor={}", static_cast<void*>(a_event.actor));
				} else {
					spdlog::trace("physics world ignored null Set3D for untracked actor={}", static_cast<void*>(a_event.actor));
				}
				return RE::BSEventNotifyControl::kContinue;
			}
			NoteLifecycleCandidate(a_event);
		} else if (IsArmorDetachCandidate(a_event.type)) {
			if (!a_event.actor) {
				spdlog::trace(
					"skipping unactionable armor detach candidate {} actor={} biped={} object={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.biped),
					static_cast<void*>(a_event.object));
				return RE::BSEventNotifyControl::kContinue;
			}

			if (IsIgnoredFirstPersonEvent(a_event, disableFirstPersonViewPhysics_)) {
				spdlog::debug("skipping first-person prototype physics detach candidate {}", ToString(a_event.type));
				return RE::BSEventNotifyControl::kContinue;
			}

			std::scoped_lock lock(lock_);
			if (a_event.firstPerson) {
				spdlog::debug(
					"ignored first-person armor detach full prototype rebuild candidate {} actor={} object={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object));
				return RE::BSEventNotifyControl::kContinue;
			}
			auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson);
			if (!actorState) {
				spdlog::trace(
					"ignored untracked armor detach candidate {} actor={} object={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object));
				return RE::BSEventNotifyControl::kContinue;
			}

			bool cleared = false;
			const auto bipedObject = ResolveEventBipedObject(a_event);
			if (a_event.object) {
				cleared = ClearPrototypeGroupsForObjectLocked(*actorState, a_event.object);
			}
			if (!cleared && bipedObject != RE::BIPED_OBJECT::kTotal) {
				cleared = ClearPrototypeGroupsForBipedObjectLocked(*actorState, bipedObject);
			}
			std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return !a_state.runtimeSuspended && !a_state.HasRuntime() && a_state.armorRecords.empty();
			});
			ResetStepClockLocked();
			const auto detachLogLevel = cleared ? spdlog::level::debug : spdlog::level::trace;
			spdlog::log(
				detachLogLevel,
				"processed scoped armor prototype physics detach after {} actor={} object={} cleared={} customizationActive={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				cleared,
				characterCustomizationMenuDepth_ > 0);
			spdlog::log(
				detachLogLevel,
				"physics world observed armor detach candidate {} actor={} object={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
		} else if (IsResetCandidate(a_event.type)) {
			std::scoped_lock lock(lock_);
			bool clearedActorState = false;
			bool preservedForCustomization = false;
			for (auto& actorState : prototypeActors_) {
				if (actorState.actor != a_event.actor) {
					continue;
				}
				if (characterCustomizationMenuDepth_ > 0 || !actorState.armorRecords.empty()) {
					SuspendPrototypeRuntimeLocked(actorState);
					preservedForCustomization = true;
					continue;
				}
				clearedActorState = true;
				ClearPrototypeStateLocked(actorState);
				actorState.actor = nullptr;
				actorState.actorHandle.reset();
			}
			std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return !a_state.actor && !a_state.HasRuntime();
			});
			const auto deferForCustomization = characterCustomizationMenuDepth_ > 0;
			if (!deferForCustomization) {
				PruneInvalidPrototypeStatesLocked();
			}
			bool rebuilt = false;
			if (!deferForCustomization && InitializeLocked() && IsPrototypeCandidateLocked(a_event, true)) {
				BuildPrototypeForEventLocked(a_event);
				rebuilt = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson) != nullptr;
			}
			if (!rebuilt && clearedActorState) {
				if (characterCustomizationMenuDepth_ == 0) {
					MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson);
				}
			}
			spdlog::debug(
				"physics world observed rebuild/reset candidate {} actor={} object={} preservedForCustomization={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				preservedForCustomization);
		} else if (a_event.type == LifecycleEventType::kActorUpdate3DModel) {
			std::scoped_lock lock(lock_);
			if (characterCustomizationMenuDepth_ == 0 && (HasActiveOrPendingActorRebuildLocked(a_event.actor) || !pendingHeadRebuilds_.empty())) {
				SchedulePendingRebuildTaskLocked();
			}
			spdlog::trace(
				"physics world observed per-frame update candidate {} actor={} object={}; pending rebuilds run through the F4SE task queue",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
		} else if (IsHeadCandidate(a_event.type)) {
			std::scoped_lock lock(lock_);
			PruneInvalidPrototypeStatesLocked();
			MarkPendingHeadRebuildLocked(a_event);
			ResetStepClockLocked();
			spdlog::debug(
				"queued head physics rebuild candidate {} actor={} object={} for deferred main-frame processing",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl Fo4PhysicsWorld::ProcessEvent(const RE::MenuOpenCloseEvent& a_event, RE::BSTEventSource<RE::MenuOpenCloseEvent>*)
	{
		const auto menuName = LowerMenuName(a_event.menuName);
		if (menuName == "loadingmenu") {
			std::scoped_lock lock(lock_);
			if (a_event.opening) {
				++loadingMenuDepth_;
				loadingPhysicsSuspended_ = true;
				ResetStepClockLocked();
				if (dynamicsWorld_) {
					dynamicsWorld_->clearForces();
				}
				spdlog::debug("loading menu '{}' opened; prototype physics suspended until game resumes", std::string_view(a_event.menuName));
			} else {
				if (loadingMenuDepth_ > 0) {
					--loadingMenuDepth_;
				}
				spdlog::debug(
					"loading menu '{}' closed; prototype physics reset will run when game resumes depth={}",
					std::string_view(a_event.menuName),
					loadingMenuDepth_);
			}
			return RE::BSEventNotifyControl::kContinue;
		}

		if (menuName != "looksmenu") {
			return RE::BSEventNotifyControl::kContinue;
		}

		std::scoped_lock lock(lock_);
		if (a_event.opening) {
			const auto wasClosed = characterCustomizationMenuDepth_ == 0;
			++characterCustomizationMenuDepth_;
			if (wasClosed) {
				pendingActorRebuilds_.clear();
				pendingHeadRebuilds_.clear();
				pendingRebuildTaskQueued_ = false;
				SuspendPrototypeStatesForCustomizationMenuLocked();
			}
			spdlog::debug(
				"character customization menu '{}' opened; prototype physics suspended for active actors",
				std::string_view(a_event.menuName));
		} else {
			if (characterCustomizationMenuDepth_ > 0) {
				--characterCustomizationMenuDepth_;
			}
			if (characterCustomizationMenuDepth_ == 0) {
				ReloadPrototypeStatesForCustomizationMenuLocked();
				std::vector<RE::Actor*> actors;
				for (const auto& actorState : prototypeActors_) {
					if (actorState.actor && std::ranges::find(actors, actorState.actor) == actors.end()) {
						actors.push_back(actorState.actor);
					}
				}
				for (auto* actor : actors) {
					MarkPendingHeadRebuildLocked(LifecycleEvent{
						.type = LifecycleEventType::kActorHeadInitialized,
						.actor = actor,
						.object = actor->GetFaceNodeSkinned() ? reinterpret_cast<RE::NiAVObject*>(actor->GetFaceNodeSkinned()) : nullptr,
						.firstPerson = false,
					});
				}
			}
			ResetStepClockLocked();
			spdlog::debug(
				"character customization menu '{}' closed; prototype physics reloaded from tracked armor records",
				std::string_view(a_event.menuName));
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	bool Fo4PhysicsWorld::InitializeLocked()
	{
		if (dynamicsWorld_) {
			return true;
		}

		collisionConfiguration_ = std::make_unique<btDefaultCollisionConfiguration>();
		dispatcher_ = std::make_unique<hdt::CollisionDispatcher>(collisionConfiguration_.get());
		broadphase_ = std::make_unique<btDbvtBroadphase>();
		solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
		dynamicsWorld_ = std::make_unique<PrototypeDynamicsWorld>(dispatcher_.get(), broadphase_.get(), solver_.get(), collisionConfiguration_.get());
		dynamicsWorld_->setGravity(btVector3(0.0F, 0.0F, kGravityAcceleration));
		auto& solverInfo = dynamicsWorld_->getSolverInfo();
		solverInfo.m_numIterations = solverIterations_;
		solverInfo.m_erp = solverErp_;
		solverInfo.m_friction = 0.0F;
		solverInfo.m_splitImpulse = true;
		solverInfo.m_splitImpulsePenetrationThreshold = -0.01F;
		solverInfo.m_erp2 = 0.15F;
		solverInfo.m_globalCfm = 0.001F;
		solverInfo.m_restitutionVelocityThreshold = 0.2F;
		solverInfo.m_solverMode = SOLVER_SIMD;
		solverInfo.m_leastSquaresResidualThreshold = 0.0001F;

		spdlog::info("initialized FO4 Faster HDT-SMP Bullet physics world");
		return true;
	}

	void Fo4PhysicsWorld::ResetLocked()
	{
		ClearAllPrototypeStatesLocked();

		if (dynamicsWorld_) {
			for (auto index = dynamicsWorld_->getNumCollisionObjects() - 1; index >= 0; --index) {
				const auto object = dynamicsWorld_->getCollisionObjectArray()[index];
				dynamicsWorld_->removeCollisionObject(object);
			}
		}

		dynamicsWorld_.reset();
		solver_.reset();
		broadphase_.reset();
		dispatcher_.reset();
		collisionConfiguration_.reset();
		suspendedActors_.clear();
		pendingActorRebuilds_.clear();
		pendingHeadRebuilds_.clear();
		pendingRebuildTaskQueued_ = false;
		nextPendingRebuildFrame_ = 1;
		loadingMenuDepth_ = 0;
		loadingPhysicsSuspended_ = false;
		candidateEvents_ = 0;
		simulationFrame_ = 1;
		currentMaxActiveActors_ = maxActiveActors_;
		ResetStepClockLocked();
		metricFrameCounter_ = 0;
		averageStepMs_ = 0.0F;
		averageWritebackMs_ = 0.0F;
		averageMainSyncMs_ = 0.0F;
		averageStepReadMs_ = 0.0F;
		averageStepWindMs_ = 0.0F;
		averageStepBulletMs_ = 0.0F;
		averageStepCollisionMs_ = 0.0F;
		pendingWritebackMs_ = 0.0F;
		pendingMainSyncMs_ = 0.0F;
		pendingStepReadMs_ = 0.0F;
		pendingStepWindMs_ = 0.0F;
		pendingStepBulletMs_ = 0.0F;
		pendingStepCollisionMs_ = 0.0F;
		pendingStepCollisionCalls_ = 0;
		mainSyncWritebacks_ = 0;
		cellJobsWritebacks_ = 0;
		postAnimationWritebacks_ = 0;
		duplicateCellJobsWritebacks_ = 0;
		duplicatePostAnimationWritebacks_ = 0;
		currentWind_.setZero();
		targetWind_.setZero();
		windWeatherCooldown_ = 0.0F;
		characterCustomizationMenuDepth_ = 0;
	}

	void Fo4PhysicsWorld::NoteLifecycleCandidate(const LifecycleEvent& a_event)
	{
		std::scoped_lock lock(lock_);
		if (!InitializeLocked()) {
			return;
		}

		if ((a_event.type == LifecycleEventType::kActorLoad3D || a_event.type == LifecycleEventType::kActorSet3D) && a_event.actor) {
			if (const auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson);
				actorState && actorState->HasActiveRuntime()) {
				spdlog::debug(
					"skipping generic {} rebuild for actor={} firstPerson={} because direct armor physics is already active",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					a_event.firstPerson);
				return;
			}
		}

		PruneInvalidPrototypeStatesLocked();

		const auto armorAttach = IsArmorAttachCandidate(a_event.type);
		const auto actorArmorAttach = armorAttach && a_event.actor && !a_event.firstPerson;
		if (actorArmorAttach && loadingPhysicsSuspended_) {
			auto armorRecords = CollectQueuedArmorRecordsForAttachLocked(a_event);
			const auto queuedRecords = armorRecords.size();
			MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson, std::move(armorRecords), false, false);
			ResetStepClockLocked();
			spdlog::debug(
				"queued scoped armor prototype physics resume for loading-screen attach {} actor={} object={} armorRecords={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				queuedRecords);
			return;
		}

		if (characterCustomizationMenuDepth_ > 0) {
			if (actorArmorAttach) {
				auto armorRecords = CollectQueuedArmorRecordsForAttachLocked(a_event);
				const auto queuedRecords = armorRecords.size();
				MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson, std::move(armorRecords), false, false);
				spdlog::debug(
					"queued scoped armor prototype physics resume for customization attach {} actor={} object={} armorRecords={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object),
					queuedRecords);
			} else {
				spdlog::debug(
					"deferred prototype physics attach candidate {} actor={} object={} while customization is active",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object));
			}
			ResetStepClockLocked();
			return;
		}

		++candidateEvents_;
		const auto candidateLogLevel =
			armorAttach && a_event.physicsXmlPath.empty() ?
				spdlog::level::trace :
				spdlog::level::debug;
		spdlog::log(
			candidateLogLevel,
			"physics world observed attach candidate #{} {} actor={} object={}",
			candidateEvents_,
			ToString(a_event.type),
			static_cast<void*>(a_event.actor),
			static_cast<void*>(a_event.object));

		if (!IsPrototypeCandidateLocked(a_event, true)) {
			auto armorRecords = CollectSuspendedArmorRecordsLocked(a_event);
			if (!armorRecords.empty()) {
				spdlog::debug(
					"captured {} suspended armor records for skipped prototype physics candidate {} actor={}",
					armorRecords.size(),
					ToString(a_event.type),
					static_cast<void*>(a_event.actor));
				SuspendActorCandidateLocked(a_event.actor, a_event.firstPerson, std::move(armorRecords));
			}
			return;
		}

		BuildPrototypeForEventLocked(a_event);
		if ((a_event.type == LifecycleEventType::kActorLoad3D || a_event.type == LifecycleEventType::kActorSet3D) && a_event.actor) {
			const auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson);
			if (!actorState || !actorState->HasActiveRuntime()) {
				auto armorRecords = actorState ? actorState->armorRecords : std::vector<PrototypeArmorRecord>{};
				if (armorRecords.empty()) {
					armorRecords = CollectSuspendedArmorRecordsLocked(a_event);
				}
				const auto hasBiped = ResolveEventBiped(a_event) != nullptr;
				if (!armorRecords.empty() || !hasBiped) {
					MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson, std::move(armorRecords));
					spdlog::debug(
						"queued retry for {} actor={} firstPerson={} because initial generic actor build found no active runtime",
						ToString(a_event.type),
						static_cast<void*>(a_event.actor),
						a_event.firstPerson);
				} else {
					spdlog::debug(
						"skipping pending retry for {} actor={} firstPerson={} because ready biped scan found no SMP armor records",
						ToString(a_event.type),
						static_cast<void*>(a_event.actor),
						a_event.firstPerson);
				}
			}
		}
	}

	void Fo4PhysicsWorld::BuildPrototypeForEventLocked(const LifecycleEvent& a_event)
	{
		auto* loader = PhysicsXmlLoader::GetSingleton();
		const auto armorAttach = IsArmorAttachCandidate(a_event.type);

		const auto buildSelection = [&](const ArmorPhysicsXmlBuildCandidate& a_candidate) {
			auto* a_object = a_candidate.object;
			const auto& a_selection = a_candidate.selection;
			const auto selectedXml = a_selection.path.string();
			if (!a_object || selectedXml.empty()) {
				return false;
			}

			spdlog::info(
				"loading prototype physics XML {} for actor={} object={}",
				selectedXml,
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_object));
			const auto selectedSummary = loader->LoadSummary(selectedXml);
			if (!selectedSummary) {
				spdlog::warn("skipping prototype physics candidate because selected XML failed to load: {}", selectedXml);
				return false;
			}

			auto scopedEvent = a_event;
			scopedEvent.object = a_object;
			scopedEvent.physicsXmlPath = selectedXml;
			if (a_candidate.sourceObject) {
				scopedEvent.sourceObject = a_candidate.sourceObject;
			}
			if (a_candidate.sourceRoot) {
				scopedEvent.sourceRoot = a_candidate.sourceRoot;
			}
			if (a_candidate.bipObject) {
				scopedEvent.bipObject = a_candidate.bipObject;
			}
			if (a_candidate.bipedObject != RE::BIPED_OBJECT::kTotal) {
				scopedEvent.bipedObject = a_candidate.bipedObject;
			}
			if (!scopedEvent.biped) {
				scopedEvent.biped = ResolveEventBiped(scopedEvent);
			}
			auto& actorState = GetOrCreatePrototypeStateLocked(a_event.actor, a_event.firstPerson);
			std::vector<std::uint64_t> staleArmorBuildGroups;
			if (armorAttach) {
				if (IsPrototypeAttachmentCurrentLocked(actorState, scopedEvent.bipedObject, a_object, scopedEvent.sourceObject, selectedXml)) {
					spdlog::debug(
						"skipping duplicate armor prototype attach actor={} bipedObject={} object={} xml='{}'",
						static_cast<void*>(a_event.actor),
						std::to_underlying(scopedEvent.bipedObject),
						static_cast<void*>(a_object),
						selectedXml);
					return false;
				}
				if (const auto* attachment = FindPrototypeAttachmentLocked(actorState, scopedEvent.bipedObject, a_object, scopedEvent.sourceObject, selectedXml)) {
					for (const auto buildGroup : attachment->buildGroups) {
						if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
							staleArmorBuildGroups.push_back(buildGroup);
						}
					}
				}
				for (const auto buildGroup : CollectPrototypeGroupsForObjectLocked(actorState, a_object)) {
					if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
						staleArmorBuildGroups.push_back(buildGroup);
					}
				}
				if (!staleArmorBuildGroups.empty()) {
					std::uint32_t resetBodies = 0;
					for (const auto buildGroup : staleArmorBuildGroups) {
						resetBodies += ResetPrototypeBuildGroupToReferencePoseLocked(actorState, buildGroup);
					}
					spdlog::debug(
						"clearing stale armor prototype groups before rebuild actor={} bipedObject={} object={} count={} resetBodies={}",
						static_cast<void*>(a_event.actor),
						std::to_underlying(scopedEvent.bipedObject),
						static_cast<void*>(a_object),
						staleArmorBuildGroups.size(),
						resetBodies);
					ClearPrototypeGroupsLocked(actorState, staleArmorBuildGroups);
					staleArmorBuildGroups.clear();
				}
			}
			const auto buildResult = BuildPrototypeBodiesLocked(actorState, scopedEvent, *selectedSummary, a_selection.meshNameMap, PrototypeBuildDomain::kArmor);
			if (buildResult.succeeded) {
				RecordPrototypeAttachmentLocked(actorState, scopedEvent.bipedObject, a_object, scopedEvent.sourceObject, selectedXml, buildResult.buildGroup);
				RecordPrototypeArmorLocked(
					actorState,
					scopedEvent.bipedObject,
					selectedXml,
					a_selection.meshNameMap,
					a_object,
					scopedEvent.sourceObject,
					scopedEvent.mergeSourceObject,
					scopedEvent.trustedActorSkeletonNodes,
					buildResult.buildGroup);
			} else if (buildResult.buildGroup != 0 && PrototypeBuildGroupIsRecordableLocked(actorState, buildResult.buildGroup, PrototypeBuildDomain::kArmor)) {
				ClearPrototypeGroupsLocked(actorState, std::vector<std::uint64_t>{ buildResult.buildGroup });
				spdlog::debug(
					"rolled back incomplete armor prototype build group actor={} bipedObject={} object={} buildGroup={} xml='{}' pendingCpuCopy={}",
					static_cast<void*>(a_event.actor),
					std::to_underlying(scopedEvent.bipedObject),
					static_cast<void*>(a_object),
					buildResult.buildGroup,
					selectedXml,
					buildResult.cpuCopyPending);
			}
			if (armorAttach && buildResult.cpuCopyPending && a_event.actor) {
				MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson, std::vector<PrototypeArmorRecord>{
					PrototypeArmorRecord{
						.bipedObject = scopedEvent.bipedObject,
						.physicsXmlPath = selectedXml,
						.meshNameMap = a_selection.meshNameMap,
						.attachedObject = a_object,
						.sourceObject = scopedEvent.sourceObject,
						.mergeSourceObject = scopedEvent.mergeSourceObject,
						.trustedActorSkeletonNodes = scopedEvent.trustedActorSkeletonNodes,
						.mergeParentBindings = scopedEvent.mergeParentBindings,
						.cpuCopyRetryCount = 1,
					},
				});
			}
			if (armorAttach) {
				SoftSuspendBuiltRuntimeIfOutOfRangeLocked(actorState, scopedEvent);
			}
			return buildResult.succeeded;
		};

		if (!armorAttach) {
			std::vector<ArmorPhysicsXmlBuildCandidate> candidates;
			CollectDirectArmorPhysicsXmlSelections(a_event.object, candidates);
			CollectEquippedArmorPhysicsXmlSelections(a_event, candidates);
			if (!candidates.empty()) {
				if (auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson)) {
					ClearPrototypeStateLocked(*actorState);
					std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
						return !a_state.runtimeSuspended && !a_state.HasRuntime();
					});
				}

				std::uint32_t built = 0;
				for (const auto& candidate : candidates) {
					if (buildSelection(candidate)) {
						++built;
					}
				}
				spdlog::debug(
					"rebuilt prototype physics for {} armor subtrees/equipped clones actor={} root={} built={}",
					candidates.size(),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object),
					built);
				std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
					return !a_state.runtimeSuspended && !a_state.HasRuntime();
				});
				return;
			}
		}

		const auto discoveredXml = FindArmorPhysicsXml(a_event);
		const auto selectedXml = discoveredXml ? discoveredXml->path.string() : prototypePhysicsXml_;
		if (selectedXml.empty()) {
			if (armorAttach) {
				spdlog::trace(
					"armor attach candidate has no direct XML/defaultBBP match actor={} object={} sourceObject={} sourceRoot={} destinationRoot={} bipObject={} model='{}' armorAddon={} preScannedXml='{}'",
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object),
					static_cast<void*>(a_event.sourceObject),
					static_cast<void*>(a_event.sourceRoot),
					static_cast<void*>(a_event.destinationRoot),
					static_cast<void*>(a_event.bipObject),
					(a_event.bipObject && a_event.bipObject->part) ? a_event.bipObject->part->GetModel() : "",
					static_cast<void*>(a_event.bipObject ? a_event.bipObject->armorAddon : nullptr),
					a_event.physicsXmlPath);
			}
			if (loader->HasPrototype()) {
				loader->LoadPrototype({});
			}
			return;
		}

		if (!armorAttach) {
			if (auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson)) {
				ClearPrototypeStateLocked(*actorState);
				std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
					return !a_state.runtimeSuspended && !a_state.HasRuntime();
				});
			}
		}

		const Smp::DefaultBBP::NameMap emptyMeshNameMap;
		buildSelection(ArmorPhysicsXmlBuildCandidate{
			.object = a_event.object,
			.selection = ArmorPhysicsXmlSelection{
				.path = selectedXml,
				.meshNameMap = discoveredXml ? discoveredXml->meshNameMap : emptyMeshNameMap,
			},
			.bipObject = a_event.bipObject,
			.bipedObject = a_event.bipedObject,
			.sourceObject = a_event.sourceObject,
			.sourceRoot = a_event.sourceRoot,
		});
		std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return !a_state.runtimeSuspended && !a_state.HasRuntime();
		});
	}

	void Fo4PhysicsWorld::BuildHeadPrototypeForEventLocked(const LifecycleEvent& a_event)
	{
		auto* faceNode = a_event.actor ? a_event.actor->GetFaceNodeSkinned() : nullptr;
		if (!faceNode) {
			spdlog::debug("skipping head physics candidate {} actor={} because no skinned face node is available", ToString(a_event.type), static_cast<void*>(a_event.actor));
			return;
		}
		auto* faceObject = reinterpret_cast<RE::NiAVObject*>(faceNode);

		auto& actorState = GetOrCreatePrototypeStateLocked(a_event.actor, a_event.firstPerson);
		const auto faceNodeChanged = actorState.faceNode && actorState.faceNode.get() != faceObject;
		if (faceNodeChanged) {
			ClearPrototypeGroupsByDomainLocked(actorState, PrototypeBuildDomain::kHead);
			ClearPrototypeGroupsByDomainLocked(actorState, PrototypeBuildDomain::kHair);
			spdlog::debug(
				"cleared stale head/hair prototype physics after face node replacement actor={} oldFaceNode={} newFaceNode={}",
				static_cast<void*>(a_event.actor),
				static_cast<void*>(actorState.faceNode.get()),
				static_cast<void*>(faceObject));
		}
		actorState.faceNode = faceObject;

		const auto touchedHeadPart = a_event.type == LifecycleEventType::kHeadPrepareHeadPart;
		const auto touchedObjectValid = !touchedHeadPart || !a_event.object || IsObjectInTree(faceObject, a_event.object);
		if (!touchedHeadPart) {
			ClearPrototypeGroupsByDomainLocked(actorState, PrototypeBuildDomain::kHead);
			ClearPrototypeGroupsByDomainLocked(actorState, PrototypeBuildDomain::kHair);
		} else if (touchedObjectValid && a_event.object) {
			ClearPrototypeGroupsForObjectLocked(actorState, a_event.object);
		} else {
			ClearPrototypeGroupsByDomainLocked(actorState, PrototypeBuildDomain::kHead);
			ClearPrototypeGroupsByDomainLocked(actorState, PrototypeBuildDomain::kHair);
			spdlog::debug(
				"discarded stale touched headpart object for actor={} object={} faceNode={}; rebuilding full current face node",
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				static_cast<void*>(faceObject));
		}

		const auto hairKeys = BuildHairHeadpartKeys(a_event.actor);
		std::vector<HeadPhysicsXmlBuildCandidate> candidates;
		const auto headPartIsHair = a_event.headPart && a_event.headPart->type.get() == RE::BGSHeadPart::HeadPartType::kHair;
		CollectHeadPhysicsXmlSelections(touchedHeadPart && touchedObjectValid && a_event.object ? a_event.object : faceObject, hairKeys, candidates, headPartIsHair);
		if (candidates.empty()) {
			spdlog::debug(
				"head physics candidate {} actor={} faceNode={} object={} found no XML/defaultBBP head/hair subtrees hairKeys={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(faceNode),
				static_cast<void*>(a_event.object),
				hairKeys.size());
			std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return !a_state.runtimeSuspended && !a_state.HasRuntime();
			});
			return;
		}

		auto* loader = PhysicsXmlLoader::GetSingleton();
		std::uint32_t built = 0;
		for (const auto& candidate : candidates) {
			const auto selectedXml = candidate.path.string();
			if (selectedXml.empty()) {
				continue;
			}

			spdlog::info(
				"loading {} prototype physics XML {} for actor={} object={}",
				PrototypeDomainName(candidate.domain),
				selectedXml,
				static_cast<void*>(a_event.actor),
				static_cast<void*>(candidate.object));
			const auto selectedSummary = loader->LoadSummary(selectedXml);
			if (!selectedSummary) {
				spdlog::warn("skipping {} physics candidate because selected XML failed to load: {}", PrototypeDomainName(candidate.domain), selectedXml);
				continue;
			}

			auto scopedEvent = a_event;
			scopedEvent.object = candidate.object;
			scopedEvent.sourceObject = candidate.sourceObject;
			scopedEvent.sourceRoot = candidate.sourceRoot;
			scopedEvent.mergeSourceObject = candidate.sourceObject;
			scopedEvent.preserveMergeSourceNames = candidate.preserveMergeSourceNames;
			scopedEvent.mergeRenamePrefix = MakeReferenceHeadRenamePrefix(PrototypeHeadRenameId.fetch_add(1, std::memory_order_relaxed));
			const auto clearedOverlappingBones = ClearPrototypeGroupsForBoneNamesLocked(actorState, selectedSummary->boneNames, candidate.domain);
			if (clearedOverlappingBones) {
				spdlog::debug(
					"cleared stale {} headpart build groups for actor={} object={} before rebuilding XML {}",
					PrototypeDomainName(candidate.domain),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(candidate.object),
					selectedXml);
			}
			const auto buildResult = BuildPrototypeBodiesLocked(actorState, scopedEvent, *selectedSummary, candidate.meshNameMap, candidate.domain);
			if (buildResult.succeeded) {
				++built;
			}
		}

		spdlog::debug(
			"processed head physics candidate actor={} faceNode={} candidates={} built={} hairKeys={}",
			static_cast<void*>(a_event.actor),
			static_cast<void*>(faceNode),
			candidates.size(),
			built,
			hairKeys.size());
		std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return !a_state.runtimeSuspended && !a_state.HasRuntime();
		});
	}

	bool Fo4PhysicsWorld::IsPrototypeCandidateLocked(const LifecycleEvent& a_event, const bool a_requireObject)
	{
		if (a_requireObject && !a_event.object) {
			spdlog::trace("skipping prototype physics candidate {} with null object", ToString(a_event.type));
			return false;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (!a_event.actor) {
			spdlog::trace("skipping prototype physics candidate {} with null actor", ToString(a_event.type));
			return false;
		}

		if (IsIgnoredFirstPersonEvent(a_event, disableFirstPersonViewPhysics_)) {
			spdlog::debug("skipping first-person prototype physics candidate {}", ToString(a_event.type));
			return false;
		}

		if (a_event.actor == player) {
			return true;
		}

		if (!enableNpcPhysics_) {
			spdlog::trace("skipping prototype physics candidate {} for non-player actor={} because NPC physics is disabled", ToString(a_event.type), static_cast<void*>(a_event.actor));
			return false;
		}

		const auto buildSuspendedArmorCandidate = ShouldBuildSuspendedArmorCandidateLocked(a_event);
		const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return a_state.HasActiveRuntime();
		}));
		if (FindPrototypeStateLocked(a_event.actor, a_event.firstPerson) == nullptr && activeActors >= currentMaxActiveActors_) {
			if (buildSuspendedArmorCandidate) {
				spdlog::debug(
					"allowing out-of-budget SMP armor candidate {} for actor={} to build directly into soft suspension ({}/{})",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					activeActors,
					currentMaxActiveActors_);
			} else {
				spdlog::debug(
					"skipping prototype physics candidate {} for actor={} because active actor budget is full ({}/{})",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					activeActors,
					currentMaxActiveActors_);
				SuspendActorCandidateLocked(a_event.actor, a_event.firstPerson);
				return false;
			}
		}

		if (player && maxActorDistance_ > 0.0F) {
			const auto distanceSquared = DistanceSquared(a_event.actor->GetPosition(), player->GetPosition());
			const auto maxDistanceSquared = maxActorDistance_ * maxActorDistance_;
			if (distanceSquared > maxDistanceSquared) {
				if (buildSuspendedArmorCandidate) {
					spdlog::debug(
						"allowing out-of-range SMP armor candidate {} for actor={} to build directly into soft suspension distanceSq={} maxDistanceSq={}",
						ToString(a_event.type),
						static_cast<void*>(a_event.actor),
						distanceSquared,
						maxDistanceSquared);
					return true;
				}
				spdlog::trace(
					"skipping prototype physics candidate {} for actor={} beyond distance budget distanceSq={} maxDistanceSq={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					distanceSquared,
					maxDistanceSquared);
				SuspendActorCandidateLocked(a_event.actor, a_event.firstPerson);
				return false;
			}
		}

		return true;
	}

	Fo4PhysicsWorld::PrototypeActorState* Fo4PhysicsWorld::FindPrototypeStateLocked(RE::Actor* a_actor, const bool a_firstPerson)
	{
		const auto found = std::ranges::find_if(prototypeActors_, [a_actor, a_firstPerson](const PrototypeActorState& a_state) {
			return a_state.actor == a_actor && a_state.firstPerson == a_firstPerson;
		});
		return found == prototypeActors_.end() ? nullptr : std::addressof(*found);
	}

	Fo4PhysicsWorld::PrototypeActorState& Fo4PhysicsWorld::GetOrCreatePrototypeStateLocked(RE::Actor* a_actor, const bool a_firstPerson)
	{
		if (auto* state = FindPrototypeStateLocked(a_actor, a_firstPerson)) {
			return *state;
		}

		auto& state = prototypeActors_.emplace_back();
		state.actor = a_actor;
		state.actorHandle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		state.firstPerson = a_firstPerson;
		return state;
	}

	bool Fo4PhysicsWorld::IsPrototypeStateValidLocked(PrototypeActorState& a_state)
	{
		if (!a_state.actor || !a_state.actorHandle) {
			spdlog::debug("dropping prototype physics state with missing actor handle actor={}", static_cast<void*>(a_state.actor));
			return false;
		}

		auto resolvedActor = a_state.actorHandle.get();
		if (!resolvedActor || resolvedActor.get() != a_state.actor) {
			spdlog::debug("dropping prototype physics state with stale actor handle actor={}", static_cast<void*>(a_state.actor));
			return false;
		}

		auto* root = resolvedActor->Get3D(a_state.firstPerson);
		if (!root && !a_state.firstPerson) {
			root = resolvedActor->Get3D();
		}
		if (!root) {
			if (!a_state.armorRecords.empty()) {
				if (!a_state.runtimeSuspended || a_state.HasRuntime()) {
					spdlog::debug(
						"suspending prototype physics state for actor={} firstPerson={} with no current 3D; preserved armorRecords={}",
						static_cast<void*>(a_state.actor),
						a_state.firstPerson,
						a_state.armorRecords.size());
					SuspendPrototypeRuntimeLocked(a_state);
				}
				return true;
			}
			spdlog::debug(
				"dropping prototype physics state for actor={} firstPerson={} with no current 3D",
				static_cast<void*>(a_state.actor),
				a_state.firstPerson);
			return false;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (a_state.actor != player) {
			if (!enableNpcPhysics_) {
				spdlog::debug("dropping prototype physics state for actor={} because NPC physics is disabled", static_cast<void*>(a_state.actor));
				return false;
			}

			if (player && maxActorDistance_ > 0.0F) {
				const auto distanceSquared = DistanceSquared(a_state.actor->GetPosition(), player->GetPosition());
				const auto maxDistanceSquared = maxActorDistance_ * maxActorDistance_;
				if (distanceSquared > maxDistanceSquared) {
					if (a_state.HasRuntime() && !a_state.runtimeSoftSuspended) {
						spdlog::debug(
							"soft-suspending prototype physics state for actor={} beyond distance budget distanceSq={} maxDistanceSq={}",
							static_cast<void*>(a_state.actor),
							distanceSquared,
							maxDistanceSquared);
						SoftSuspendPrototypeRuntimeLocked(a_state);
					}
					return true;
				}
			}
		}

		return true;
	}

	void Fo4PhysicsWorld::PruneInvalidPrototypeStatesLocked()
	{
		for (auto& actorState : prototypeActors_) {
			if (!IsPrototypeStateValidLocked(actorState)) {
				ClearPrototypeStateLocked(actorState);
				actorState.actor = nullptr;
				actorState.actorHandle.reset();
			}
		}

		std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return !a_state.actor && !a_state.HasRuntime();
		});
	}

	void Fo4PhysicsWorld::EnforceActorBudgetLocked()
	{
		auto activeActors = static_cast<std::size_t>(std::ranges::count_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return a_state.HasActiveRuntime();
		}));
		if (activeActors <= currentMaxActiveActors_) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		while (activeActors > currentMaxActiveActors_) {
			auto victim = prototypeActors_.end();
			auto victimDistanceSquared = -1.0F;
			for (auto it = prototypeActors_.begin(); it != prototypeActors_.end(); ++it) {
				if (!it->HasActiveRuntime()) {
					continue;
				}
				auto resolvedActor = it->actorHandle.get();
				if (!resolvedActor || resolvedActor.get() != it->actor) {
					continue;
				}
				if (player && it->actor == player) {
					continue;
				}

				auto distanceSquared = std::numeric_limits<float>::max();
				if (player) {
					distanceSquared = DistanceSquared(resolvedActor->GetPosition(), player->GetPosition());
				}

				if (victim == prototypeActors_.end() || distanceSquared > victimDistanceSquared) {
					victim = it;
					victimDistanceSquared = distanceSquared;
				}
			}

			if (victim == prototypeActors_.end()) {
				return;
			}

			spdlog::debug(
				"soft-suspending prototype physics state for actor={} because active actor budget shrank to {}",
				static_cast<void*>(victim->actor),
				currentMaxActiveActors_);
			SoftSuspendPrototypeRuntimeLocked(*victim);
			--activeActors;
		}
	}

	bool Fo4PhysicsWorld::ShouldBuildSuspendedArmorCandidateLocked(const LifecycleEvent& a_event) const
	{
		return IsArmorAttachCandidate(a_event.type) && !a_event.physicsXmlPath.empty();
	}

	void Fo4PhysicsWorld::SoftSuspendBuiltRuntimeIfOutOfRangeLocked(PrototypeActorState& a_state, const LifecycleEvent& a_event)
	{
		if (!a_state.HasActiveRuntime()) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (a_state.actor == player) {
			return;
		}

		bool shouldSuspend = false;
		const char* reason = nullptr;
		if (player && maxActorDistance_ > 0.0F) {
			const auto distanceSquared = DistanceSquared(a_event.actor->GetPosition(), player->GetPosition());
			const auto maxDistanceSquared = maxActorDistance_ * maxActorDistance_;
			if (distanceSquared > maxDistanceSquared) {
				shouldSuspend = true;
				reason = "distance";
			}
		}

		const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(prototypeActors_, [](const PrototypeActorState& a_candidate) {
			return a_candidate.HasActiveRuntime();
		}));
		if (!shouldSuspend && activeActors > currentMaxActiveActors_) {
			shouldSuspend = true;
			reason = "budget";
		}

		if (!shouldSuspend) {
			return;
		}

		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.runtimes) {
			if (runtime.buildGroup != 0 && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}

		std::uint32_t resetBodies = 0;
		for (const auto buildGroup : buildGroups) {
			resetBodies += ResetPrototypeBuildGroupToReferencePoseLocked(a_state, buildGroup);
		}

		spdlog::debug(
			"soft-suspending freshly built armor prototype runtime after reference-pose node/Bullet reset actor={} reason={} activeActors={} actorCap={} event={} buildGroups={} resetBodies={}",
			static_cast<void*>(a_state.actor),
			reason ? reason : "unknown",
			activeActors,
			currentMaxActiveActors_,
			ToString(a_event.type),
			buildGroups.size(),
			resetBodies);
		SoftSuspendPrototypeRuntimeLocked(a_state);
	}

	void Fo4PhysicsWorld::SuspendActorCandidateLocked(
		RE::Actor* a_actor,
		const bool a_firstPerson,
		std::vector<PrototypeArmorRecord> a_armorRecords)
	{
		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (!a_actor || a_actor == player || !enableNpcPhysics_) {
			return;
		}

		for (auto& candidate : suspendedActors_) {
			const auto resolvedActor = candidate.actorHandle.get();
			if (resolvedActor && resolvedActor.get() == a_actor) {
				if (!a_armorRecords.empty()) {
					candidate.firstPerson = a_firstPerson;
					candidate.armorRecords = std::move(a_armorRecords);
				}
				return;
			}
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		if (!handle) {
			return;
		}

		suspendedActors_.push_back({
			.actorHandle = handle,
			.firstPerson = a_firstPerson,
			.armorRecords = std::move(a_armorRecords),
		});
		spdlog::debug(
			"suspended prototype physics candidate actor={} firstPerson={} armorRecords={} until distance/budget allows rebuild",
			static_cast<void*>(a_actor),
			a_firstPerson,
			suspendedActors_.back().armorRecords.size());
	}

	void Fo4PhysicsWorld::TryReactivateSuspendedActorsLocked()
	{
		if (suspendedActors_.empty()) {
			return;
		}

		if (!enableNpcPhysics_) {
			suspendedActors_.clear();
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		for (auto it = suspendedActors_.begin(); it != suspendedActors_.end();) {
			const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return a_state.HasActiveRuntime();
			}));
			if (activeActors >= currentMaxActiveActors_) {
				return;
			}

			auto resolvedActor = it->actorHandle.get();
			if (!resolvedActor) {
				it = suspendedActors_.erase(it);
				continue;
			}

			auto* actor = resolvedActor.get();
			auto* existingState = actor ? FindPrototypeStateLocked(actor, it->firstPerson) : nullptr;
			if (!actor) {
				it = suspendedActors_.erase(it);
				continue;
			}
			if (existingState) {
				if ((existingState->runtimeSuspended || existingState->runtimeSoftSuspended) && !it->armorRecords.empty()) {
					for (auto& record : it->armorRecords) {
						MergePrototypeArmorRecord(existingState->armorRecords, std::move(record));
					}
					spdlog::debug(
						"merged suspended armor records into soft-suspended prototype state actor={} firstPerson={} armorRecords={}",
						static_cast<void*>(actor),
						it->firstPerson,
						existingState->armorRecords.size());
				}
				it = suspendedActors_.erase(it);
				continue;
			}

			auto* root = actor->Get3D(it->firstPerson);
			if (!root && !it->firstPerson) {
				root = actor->Get3D();
			}
			if (!root) {
				++it;
				continue;
			}

			if (player && maxActorDistance_ > 0.0F) {
				const auto distanceSquared = DistanceSquared(actor->GetPosition(), player->GetPosition());
				const auto maxDistanceSquared = maxActorDistance_ * maxActorDistance_;
				if (distanceSquared > maxDistanceSquared) {
					++it;
					continue;
				}
			}

			if (!it->armorRecords.empty()) {
				PendingActorRebuild pending{
					.actorHandle = it->actorHandle,
					.firstPerson = it->firstPerson,
					.armorRecords = std::move(it->armorRecords),
				};
				if (!RebuildPendingArmorRecordsLocked(actor, pending)) {
					it->armorRecords = std::move(pending.armorRecords);
					++it;
					continue;
				}

				spdlog::debug(
					"reactivated suspended prototype physics candidate actor={} firstPerson={} from preserved armor records",
					static_cast<void*>(actor),
					it->firstPerson);
				it = suspendedActors_.erase(it);
				continue;
			}

			spdlog::debug(
				"reactivating suspended prototype physics candidate actor={} root={} firstPerson={}",
				static_cast<void*>(actor),
				static_cast<void*>(root),
				it->firstPerson);
			BuildPrototypeForEventLocked({
				.type = LifecycleEventType::kActorSet3D,
				.actor = actor,
				.biped = actor->GetBiped(it->firstPerson).get(),
				.object = root,
				.firstPerson = it->firstPerson,
			});
			if (FindPrototypeStateLocked(actor, it->firstPerson)) {
				it = suspendedActors_.erase(it);
			} else {
				++it;
			}
		}
	}

	void Fo4PhysicsWorld::TryReactivateSuspendedPrototypeStatesLocked()
	{
		if (!enableNpcPhysics_) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		for (auto& actorState : prototypeActors_) {
			const auto needsSoftResume = actorState.runtimeSoftSuspended;
			const auto needsRebuild = actorState.runtimeSuspended && !actorState.armorRecords.empty();
			if ((!needsSoftResume && !needsRebuild) || actorState.actor == player) {
				continue;
			}

			const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return a_state.HasActiveRuntime();
			}));
			if (activeActors >= currentMaxActiveActors_) {
				return;
			}

			auto resolvedActor = actorState.actorHandle.get();
			if (!resolvedActor || resolvedActor.get() != actorState.actor) {
				continue;
			}

			auto* actor = resolvedActor.get();
			auto* root = actor->Get3D(actorState.firstPerson);
			if (!root && !actorState.firstPerson) {
				root = actor->Get3D();
			}
			if (!root) {
				continue;
			}

			if (player && maxActorDistance_ > 0.0F) {
				const auto distanceSquared = DistanceSquared(actor->GetPosition(), player->GetPosition());
				const auto maxDistanceSquared = maxActorDistance_ * maxActorDistance_;
				if (distanceSquared > maxDistanceSquared) {
					continue;
				}
			}

			if (needsSoftResume && ResumeSoftSuspendedPrototypeRuntimeLocked(actorState)) {
				continue;
			}
			if (needsSoftResume && !actorState.armorRecords.empty()) {
				actorState.runtimeSuspended = true;
			}

			if (!actorState.runtimeSuspended || actorState.armorRecords.empty()) {
				continue;
			}

			auto armorRecords = actorState.armorRecords;
			PendingActorRebuild pending{
				.actorHandle = actorState.actorHandle,
				.firstPerson = actorState.firstPerson,
				.armorRecords = std::move(armorRecords),
			};
			if (!RebuildPendingArmorRecordsLocked(actor, pending)) {
				if (actorState.HasRuntime()) {
					actorState.runtimeSuspended = false;
				}
				continue;
			}

			actorState.runtimeSuspended = false;
			spdlog::debug(
				"reactivated soft-suspended prototype physics state actor={} firstPerson={} from preserved armor records",
				static_cast<void*>(actor),
				actorState.firstPerson);
		}
	}

	void Fo4PhysicsWorld::MergePrototypeArmorRecord(std::vector<PrototypeArmorRecord>& a_records, PrototypeArmorRecord a_record)
	{
		if (a_record.bipedObject == RE::BIPED_OBJECT::kTotal || a_record.physicsXmlPath.empty()) {
			return;
		}
		auto appendBuildGroups = [](std::vector<std::uint64_t>& a_target, const std::vector<std::uint64_t>& a_source) {
			for (const auto buildGroup : a_source) {
				if (buildGroup != 0 && std::ranges::find(a_target, buildGroup) == a_target.end()) {
					a_target.push_back(buildGroup);
				}
			}
		};
		const auto normalizedXml = ConfigPaths::LowerString(a_record.physicsXmlPath);
		auto existing = std::ranges::find_if(a_records, [&a_record, &normalizedXml](const PrototypeArmorRecord& a_existing) {
			return a_existing.bipedObject == a_record.bipedObject &&
				a_existing.attachedObject.get() == a_record.attachedObject.get() &&
				a_existing.sourceObject.get() == a_record.sourceObject.get() &&
				ConfigPaths::LowerString(a_existing.physicsXmlPath) == normalizedXml;
		});
		if (existing != a_records.end()) {
			if (!a_record.attachedObject && existing->attachedObject) {
				a_record.attachedObject = existing->attachedObject;
			}
			if (!a_record.sourceObject && existing->sourceObject) {
				a_record.sourceObject = existing->sourceObject;
			}
			if (!a_record.mergeSourceObject && existing->mergeSourceObject) {
				a_record.mergeSourceObject = existing->mergeSourceObject;
			}
			if (a_record.trustedActorSkeletonNodes.empty() && !existing->trustedActorSkeletonNodes.empty()) {
				a_record.trustedActorSkeletonNodes = existing->trustedActorSkeletonNodes;
			}
			if (a_record.mergeParentBindings.empty() && !existing->mergeParentBindings.empty()) {
				a_record.mergeParentBindings = existing->mergeParentBindings;
			}
			appendBuildGroups(a_record.buildGroups, existing->buildGroups);
			*existing = std::move(a_record);
		} else {
			a_records.push_back(std::move(a_record));
		}
	}

	Fo4PhysicsWorld::PendingActorRebuild* Fo4PhysicsWorld::FindPendingActorRebuildLocked(RE::Actor* a_actor, const bool a_firstPerson)
	{
		if (!a_actor) {
			return nullptr;
		}

		const auto found = std::ranges::find_if(pendingActorRebuilds_, [a_actor, a_firstPerson](const PendingActorRebuild& a_pending) {
			const auto resolvedActor = a_pending.actorHandle.get();
			return resolvedActor && resolvedActor.get() == a_actor && a_pending.firstPerson == a_firstPerson;
		});
		return found != pendingActorRebuilds_.end() ? std::addressof(*found) : nullptr;
	}

	std::vector<Fo4PhysicsWorld::PrototypeArmorRecord> Fo4PhysicsWorld::CollectQueuedArmorRecordsForAttachLocked(const LifecycleEvent& a_event)
	{
		std::vector<PrototypeArmorRecord> records;
		if (auto* pending = FindPendingActorRebuildLocked(a_event.actor, a_event.firstPerson)) {
			for (auto& record : pending->armorRecords) {
				MergePrototypeArmorRecord(records, record);
			}
		} else if (auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson)) {
			for (auto& record : actorState->armorRecords) {
				MergePrototypeArmorRecord(records, record);
			}
		}

		for (auto& record : CollectSuspendedArmorRecordsLocked(a_event)) {
			MergePrototypeArmorRecord(records, std::move(record));
		}
		return records;
	}

	std::vector<Fo4PhysicsWorld::PrototypeArmorRecord> Fo4PhysicsWorld::CollectQueuedArmorRecordsForDetachLocked(const LifecycleEvent& a_event)
	{
		std::vector<PrototypeArmorRecord> records;
		if (auto* pending = FindPendingActorRebuildLocked(a_event.actor, a_event.firstPerson)) {
			records = pending->armorRecords;
		} else if (auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson)) {
			records = actorState->armorRecords;
		}
		const auto detachedBipedObject = ResolveEventBipedObject(a_event);
		if (detachedBipedObject != RE::BIPED_OBJECT::kTotal) {
			std::erase_if(records, [detachedBipedObject, object = a_event.object](const PrototypeArmorRecord& a_record) {
				if (a_record.bipedObject != detachedBipedObject) {
					return false;
				}
				if (!object) {
					return true;
				}
				const auto matchesAttached =
					a_record.attachedObject &&
					(a_record.attachedObject.get() == object ||
						IsObjectInTree(a_record.attachedObject.get(), object) ||
						IsObjectInTree(object, a_record.attachedObject.get()));
				const auto matchesSource =
					a_record.sourceObject &&
					(a_record.sourceObject.get() == object ||
						IsObjectInTree(a_record.sourceObject.get(), object) ||
						IsObjectInTree(object, a_record.sourceObject.get()));
				return matchesAttached || matchesSource;
			});
		}
		return records;
	}

	void Fo4PhysicsWorld::MarkPendingActorRebuildLocked(
		RE::Actor* a_actor,
		const bool a_firstPerson,
		std::vector<PrototypeArmorRecord> a_armorRecords,
		const bool a_forceArmorRescan,
		const bool a_scheduleImmediately,
		const bool a_replaceArmorRecords)
	{
		if (!a_actor) {
			return;
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		if (!handle) {
			return;
		}

		const auto requestedDelay = std::ranges::any_of(a_armorRecords, [](const PrototypeArmorRecord& a_record) {
			return a_record.cpuCopyRetryCount > 0;
		}) ? kCpuCopyPendingRetryDelayTasks : 0U;
		const auto rebuildDelay = a_forceArmorRescan ? std::max(requestedDelay, kArmorChangeRebuildDelayTasks) : requestedDelay;
		if (auto* pending = FindPendingActorRebuildLocked(a_actor, a_firstPerson)) {
			if (a_replaceArmorRecords) {
				pending->armorRecords = std::move(a_armorRecords);
			} else {
				for (auto& record : a_armorRecords) {
					MergePrototypeArmorRecord(pending->armorRecords, std::move(record));
				}
			}
			pending->frameDelay = std::max(pending->frameDelay, rebuildDelay);
			pending->forceArmorRescan = pending->forceArmorRescan || a_forceArmorRescan;
			if (a_scheduleImmediately) {
				SchedulePendingRebuildTaskLocked();
			} else {
				nextPendingRebuildFrame_ = std::min(nextPendingRebuildFrame_, simulationFrame_ + 1);
			}
			spdlog::debug(
				"updated pending prototype physics rebuild for actor={} firstPerson={} armorRecords={} forceArmorRescan={} scheduleImmediately={} replaceArmorRecords={}",
				static_cast<void*>(a_actor),
				a_firstPerson,
				pending->armorRecords.size(),
				pending->forceArmorRescan,
				a_scheduleImmediately,
				a_replaceArmorRecords);
			return;
		}

		pendingActorRebuilds_.push_back({
			.actorHandle = handle,
			.firstPerson = a_firstPerson,
			.armorRecords = std::move(a_armorRecords),
			.frameDelay = rebuildDelay,
			.forceArmorRescan = a_forceArmorRescan,
		});
		if (a_scheduleImmediately) {
			SchedulePendingRebuildTaskLocked();
		} else {
			nextPendingRebuildFrame_ = std::min(nextPendingRebuildFrame_, simulationFrame_ + 1);
		}
		spdlog::debug(
			"queued pending prototype physics rebuild for actor={} firstPerson={} armorRecords={} forceArmorRescan={} scheduleImmediately={} replaceArmorRecords={}",
			static_cast<void*>(a_actor),
			a_firstPerson,
			pendingActorRebuilds_.back().armorRecords.size(),
			a_forceArmorRescan,
			a_scheduleImmediately,
			a_replaceArmorRecords);
	}

	void Fo4PhysicsWorld::MarkPendingHeadRebuildLocked(const LifecycleEvent& a_event)
	{
		if (!a_event.actor) {
			return;
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_event.actor);
		if (!handle) {
			return;
		}

		for (auto& pending : pendingHeadRebuilds_) {
			auto resolvedActor = pending.actorHandle.get();
			if (resolvedActor && resolvedActor.get() == a_event.actor && pending.type == a_event.type && pending.object.get() == a_event.object) {
				pending.frameDelay = std::max(pending.frameDelay, kHeadInitializedRebuildDelayFrames);
				pending.headPart = a_event.headPart;
				SchedulePendingRebuildTaskLocked();
				return;
			}
		}

		pendingHeadRebuilds_.push_back({
			.actorHandle = handle,
			.type = a_event.type,
			.object = a_event.object,
			.headPart = a_event.headPart,
			.frameDelay = kHeadInitializedRebuildDelayFrames,
		});
		SchedulePendingRebuildTaskLocked();
	}

	void Fo4PhysicsWorld::SchedulePendingRebuildTaskLocked()
	{
		if (pendingRebuildTaskQueued_ || (pendingActorRebuilds_.empty() && pendingHeadRebuilds_.empty())) {
			return;
		}

		const auto taskInterface = F4SE::GetTaskInterface();
		if (!taskInterface) {
			spdlog::warn("unable to queue pending prototype physics rebuild task because F4SE task interface is unavailable");
			return;
		}

		pendingRebuildTaskQueued_ = true;
		taskInterface->AddTask([] {
			Fo4PhysicsWorld::GetSingleton()->ProcessPendingRebuilds();
		});
	}

	bool Fo4PhysicsWorld::HasActiveOrPendingActorRebuildLocked(RE::Actor* a_actor)
	{
		if (!a_actor) {
			return false;
		}

		if (std::ranges::any_of(prototypeActors_, [a_actor](const PrototypeActorState& a_state) {
			return a_state.actor == a_actor;
		})) {
			return true;
		}

		return std::ranges::any_of(pendingActorRebuilds_, [a_actor](const PendingActorRebuild& a_pending) {
			const auto resolvedActor = a_pending.actorHandle.get();
			return resolvedActor && resolvedActor.get() == a_actor;
		});
	}

	std::vector<Fo4PhysicsWorld::PrototypeArmorRecord> Fo4PhysicsWorld::CollectSuspendedArmorRecordsLocked(const LifecycleEvent& a_event)
	{
		std::vector<PrototypeArmorRecord> records;
		auto appendRecord = [&](
								const RE::BIPED_OBJECT a_bipedObject,
								const ArmorPhysicsXmlSelection& a_selection,
								RE::NiAVObject* a_attachedObject,
								RE::NiAVObject* a_sourceObject,
								RE::NiAVObject* a_mergeSourceObject = nullptr) {
			if (a_bipedObject == RE::BIPED_OBJECT::kTotal || a_selection.path.empty()) {
				return;
			}
			const auto normalizedXml = ConfigPaths::LowerString(a_selection.path.string());
			const auto duplicate = std::ranges::any_of(records, [a_bipedObject, a_attachedObject, a_sourceObject, &normalizedXml](const PrototypeArmorRecord& a_record) {
				return a_record.bipedObject == a_bipedObject &&
					a_record.attachedObject.get() == a_attachedObject &&
					a_record.sourceObject.get() == a_sourceObject &&
					ConfigPaths::LowerString(a_record.physicsXmlPath) == normalizedXml;
			});
			if (duplicate) {
				return;
			}
			records.push_back({
				.bipedObject = a_bipedObject,
				.physicsXmlPath = a_selection.path.string(),
				.meshNameMap = a_selection.meshNameMap,
				.attachedObject = a_attachedObject,
				.sourceObject = a_sourceObject,
				.mergeSourceObject = a_mergeSourceObject,
				.trustedActorSkeletonNodes = a_event.trustedActorSkeletonNodes,
				.mergeParentBindings = a_event.mergeParentBindings,
			});
		};

		if (IsArmorAttachCandidate(a_event.type)) {
			if (auto selection = FindArmorPhysicsXml(a_event)) {
				appendRecord(a_event.bipedObject, *selection, a_event.object, a_event.sourceObject, a_event.mergeSourceObject);
			}
			return records;
		}

		std::vector<ArmorPhysicsXmlBuildCandidate> candidates;
		CollectEquippedArmorPhysicsXmlSelections(a_event, candidates);
		for (const auto& candidate : candidates) {
			appendRecord(candidate.bipedObject, candidate.selection, candidate.object, candidate.sourceObject);
		}
		return records;
	}

	void Fo4PhysicsWorld::RecordPrototypeArmorLocked(
		PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		std::string a_physicsXmlPath,
		const DefaultBBP::NameMap& a_meshNameMap,
		RE::NiAVObject* a_attachedObject,
		RE::NiAVObject* a_sourceObject,
		RE::NiAVObject* a_mergeSourceObject,
		std::vector<RE::NiAVObject*> a_trustedActorSkeletonNodes,
		const std::uint64_t a_buildGroup)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal || a_physicsXmlPath.empty()) {
			return;
		}

		std::vector<MergeParentBinding> mergeParentBindings;
		if (a_buildGroup != 0) {
			for (const auto& mergedNode : a_state.mergedNodes) {
				auto* node = mergedNode.node ? mergedNode.node->IsNode() : nullptr;
				const auto parentName = !mergedNode.recordParentName.empty() ?
					mergedNode.recordParentName :
					(mergedNode.parent ? std::string(mergedNode.parent->GetName()) : std::string{});
				if (mergedNode.buildGroup != a_buildGroup || !mergedNode.recordMergeParentBinding || !node || parentName.empty()) {
					continue;
				}

				auto sourceName = mergedNode.sourceName;
				if (sourceName.empty()) {
					continue;
				}

				const auto& recordLocal = mergedNode.hasRecordLocalToParent ?
					mergedNode.recordLocalToParent :
					(mergedNode.hasLocalToParent ? mergedNode.localToParent : node->local);
				mergeParentBindings.push_back({
					.sourceName = std::move(sourceName),
					.parentName = parentName,
					.localToParent = recordLocal,
					.hasLocalToParent = true,
				});
			}
		}
		for (const auto& binding : mergeParentBindings) {
			spdlog::debug(
				"recorded armor merge parent binding actor={} bipedObject={} xml='{}' source='{}' parent='{}' buildGroup={} localToParent=({:.3f},{:.3f},{:.3f})",
				static_cast<void*>(a_state.actor),
				std::to_underlying(a_bipedObject),
				a_physicsXmlPath,
				binding.sourceName,
				binding.parentName,
				a_buildGroup,
				binding.localToParent.translate.x,
				binding.localToParent.translate.y,
				binding.localToParent.translate.z);
		}

		const auto normalizedXml = ConfigPaths::LowerString(a_physicsXmlPath);
		const auto appendBuildGroup = [](std::vector<std::uint64_t>& a_buildGroups, const std::uint64_t a_buildGroup) {
			if (a_buildGroup != 0 && std::ranges::find(a_buildGroups, a_buildGroup) == a_buildGroups.end()) {
				a_buildGroups.push_back(a_buildGroup);
			}
		};
		auto existing = std::ranges::find_if(a_state.armorRecords, [&](const PrototypeArmorRecord& a_record) {
			return a_record.bipedObject == a_bipedObject &&
				a_record.attachedObject.get() == a_attachedObject &&
				a_record.sourceObject.get() == a_sourceObject &&
				ConfigPaths::LowerString(a_record.physicsXmlPath) == normalizedXml;
		});
		if (existing != a_state.armorRecords.end()) {
			existing->physicsXmlPath = std::move(a_physicsXmlPath);
			existing->meshNameMap = a_meshNameMap;
			existing->attachedObject = a_attachedObject;
			existing->sourceObject = a_sourceObject;
			if (a_mergeSourceObject) {
				existing->mergeSourceObject = a_mergeSourceObject;
			}
			if (!a_trustedActorSkeletonNodes.empty()) {
				existing->trustedActorSkeletonNodes = std::move(a_trustedActorSkeletonNodes);
			}
			existing->mergeParentBindings = std::move(mergeParentBindings);
			appendBuildGroup(existing->buildGroups, a_buildGroup);
		} else {
			std::vector<std::uint64_t> buildGroups;
			appendBuildGroup(buildGroups, a_buildGroup);
			a_state.armorRecords.push_back({
				.bipedObject = a_bipedObject,
				.physicsXmlPath = std::move(a_physicsXmlPath),
				.meshNameMap = a_meshNameMap,
				.attachedObject = a_attachedObject,
				.sourceObject = a_sourceObject,
				.mergeSourceObject = a_mergeSourceObject,
				.trustedActorSkeletonNodes = std::move(a_trustedActorSkeletonNodes),
				.mergeParentBindings = std::move(mergeParentBindings),
				.buildGroups = std::move(buildGroups),
			});
		}
	}

	Fo4PhysicsWorld::PrototypeAttachmentRecord* Fo4PhysicsWorld::FindPrototypeAttachmentLocked(
		PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		const std::string_view a_physicsXmlPath)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return nullptr;
		}

		const auto normalizedXml = a_physicsXmlPath.empty() ? std::string{} : ConfigPaths::LowerString(std::string(a_physicsXmlPath));
		const auto found = std::ranges::find_if(a_state.attachmentRecords, [&](const PrototypeAttachmentRecord& a_record) {
			if (a_record.bipedObject != a_bipedObject) {
				return false;
			}
			if (a_object && a_record.attachedObject.get() != a_object) {
				return false;
			}
			if (a_sourceObject && a_record.sourceObject && a_record.sourceObject.get() != a_sourceObject) {
				return false;
			}
			if (!normalizedXml.empty() && ConfigPaths::LowerString(a_record.physicsXmlPath) != normalizedXml) {
				return false;
			}
			return true;
		});
		return found != a_state.attachmentRecords.end() ? std::addressof(*found) : nullptr;
	}

	const Fo4PhysicsWorld::PrototypeAttachmentRecord* Fo4PhysicsWorld::FindPrototypeAttachmentLocked(
		const PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		const std::string_view a_physicsXmlPath)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return nullptr;
		}

		const auto normalizedXml = a_physicsXmlPath.empty() ? std::string{} : ConfigPaths::LowerString(std::string(a_physicsXmlPath));
		const auto found = std::ranges::find_if(a_state.attachmentRecords, [&](const PrototypeAttachmentRecord& a_record) {
			if (a_record.bipedObject != a_bipedObject) {
				return false;
			}
			if (a_object && a_record.attachedObject.get() != a_object) {
				return false;
			}
			if (a_sourceObject && a_record.sourceObject && a_record.sourceObject.get() != a_sourceObject) {
				return false;
			}
			if (!normalizedXml.empty() && ConfigPaths::LowerString(a_record.physicsXmlPath) != normalizedXml) {
				return false;
			}
			return true;
		});
		return found != a_state.attachmentRecords.end() ? std::addressof(*found) : nullptr;
	}

	bool Fo4PhysicsWorld::IsPrototypeAttachmentCurrentLocked(
		const PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		const std::string_view a_physicsXmlPath)
	{
		const auto* record = FindPrototypeAttachmentLocked(a_state, a_bipedObject, a_object, a_sourceObject, a_physicsXmlPath);
		if (!record || record->buildGroups.empty() || record->physicsXmlPath.empty()) {
			return false;
		}

		const auto sameXml = ConfigPaths::LowerString(record->physicsXmlPath) == ConfigPaths::LowerString(std::string(a_physicsXmlPath));
		if (!sameXml) {
			return false;
		}
		if (a_object && record->attachedObject.get() != a_object) {
			return false;
		}
		if (a_sourceObject && record->sourceObject && record->sourceObject.get() != a_sourceObject) {
			return false;
		}

		return std::ranges::any_of(record->buildGroups, [&](const std::uint64_t a_buildGroup) {
			if (std::ranges::any_of(a_state.meshes, [a_buildGroup](const PrototypeMesh& a_mesh) {
					return a_mesh.buildGroup == a_buildGroup;
				})) {
				return true;
			}
			return std::ranges::any_of(a_state.bodies, [a_buildGroup](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, a_buildGroup);
			});
		});
	}

	void Fo4PhysicsWorld::RecordPrototypeAttachmentLocked(
		PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		std::string a_physicsXmlPath,
		const std::uint64_t a_buildGroup)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal || a_physicsXmlPath.empty() || a_buildGroup == 0) {
			return;
		}

		auto* record = FindPrototypeAttachmentLocked(a_state, a_bipedObject, a_object, a_sourceObject, a_physicsXmlPath);
		if (!record) {
			a_state.attachmentRecords.push_back({
				.bipedObject = a_bipedObject,
				.generation = ++a_state.nextAttachmentGeneration,
			});
			record = std::addressof(a_state.attachmentRecords.back());
		}

		record->physicsXmlPath = std::move(a_physicsXmlPath);
		record->attachedObject = a_object;
		record->sourceObject = a_sourceObject;
		if (std::ranges::find(record->buildGroups, a_buildGroup) == record->buildGroups.end()) {
			record->buildGroups.push_back(a_buildGroup);
		}
	}

	bool Fo4PhysicsWorld::RebuildPendingArmorRecordsLocked(RE::Actor* a_actor, PendingActorRebuild& a_pending)
	{
		if (!a_actor) {
			a_pending.armorRecords.clear();
			return true;
		}

		auto* biped = a_actor->GetBiped(a_pending.firstPerson).get();
		if (!biped && !a_pending.firstPerson) {
			biped = a_actor->GetBiped().get();
		}
		if (!biped) {
			return false;
		}

		auto* loader = PhysicsXmlLoader::GetSingleton();
		for (auto it = a_pending.armorRecords.begin(); it != a_pending.armorRecords.end();) {
			auto& record = *it;
			if (record.bipedObject == RE::BIPED_OBJECT::kTotal || record.physicsXmlPath.empty()) {
				it = a_pending.armorRecords.erase(it);
				continue;
			}

			auto* bipObject = biped->GetBipObject(record.bipedObject);
			auto* partClone = bipObject ? bipObject->partClone.get() : nullptr;
			if (!partClone) {
				++it;
				continue;
			}
			auto* rebuildObject = record.attachedObject && IsProbablyValidNiObject(record.attachedObject.get()) ? record.attachedObject.get() : partClone;
			auto* rebuildSourceObject = record.mergeSourceObject ? partClone : (record.sourceObject && IsProbablyValidNiObject(record.sourceObject.get()) ? record.sourceObject.get() : partClone);
			auto* rebuildSourceRoot = rebuildSourceObject ? rebuildSourceObject->IsNode() : partClone->IsNode();

			const auto selectedSummary = loader->LoadSummary(record.physicsXmlPath);
			if (!selectedSummary) {
				spdlog::warn(
					"dropping pending customization resume armor slot because XML failed to load actor={} bipedObject={} xml='{}'",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					record.physicsXmlPath);
				it = a_pending.armorRecords.erase(it);
				continue;
			}

			const LifecycleEvent resumeEvent{
				.type = LifecycleEventType::kArmorApplySkinnedObjects,
				.actor = a_actor,
				.biped = biped,
				.bipObject = bipObject,
				.bipedObject = record.bipedObject,
				.object = rebuildObject,
				.sourceObject = rebuildSourceObject,
				.mergeSourceObject = record.mergeSourceObject.get(),
				.trustedActorSkeletonNodes = record.trustedActorSkeletonNodes,
				.mergeParentBindings = record.mergeParentBindings,
				.sourceRoot = rebuildSourceRoot,
				.physicsXmlPath = record.physicsXmlPath,
				.firstPerson = a_pending.firstPerson,
			};

			auto& actorState = GetOrCreatePrototypeStateLocked(a_actor, a_pending.firstPerson);
			std::vector<std::uint64_t> staleArmorBuildGroups;
			if (const auto* attachment = FindPrototypeAttachmentLocked(actorState, record.bipedObject, rebuildObject, rebuildSourceObject, record.physicsXmlPath)) {
				for (const auto buildGroup : attachment->buildGroups) {
					if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
						staleArmorBuildGroups.push_back(buildGroup);
					}
				}
			}
			for (const auto buildGroup : CollectPrototypeGroupsForObjectLocked(actorState, rebuildObject)) {
				if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
					staleArmorBuildGroups.push_back(buildGroup);
				}
			}
			if (!staleArmorBuildGroups.empty()) {
				std::uint32_t resetBodies = 0;
				for (const auto buildGroup : staleArmorBuildGroups) {
					resetBodies += ResetPrototypeBuildGroupToReferencePoseLocked(actorState, buildGroup);
				}
				ClearPrototypeGroupsLocked(actorState, staleArmorBuildGroups);
				const auto clearedCount = staleArmorBuildGroups.size();
				staleArmorBuildGroups.clear();
				spdlog::debug(
					"cleared stale prototype groups before pending armor rebuild actor={} bipedObject={} object={} xml='{}' groups={} resetBodies={}",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					static_cast<void*>(rebuildObject),
					record.physicsXmlPath,
					clearedCount,
					resetBodies);
			}
			spdlog::debug(
				"rebuilding pending armor prototype physics actor={} bipedObject={} object={} xml='{}' stagedStaleGroups={}",
				static_cast<void*>(a_actor),
				std::to_underlying(record.bipedObject),
				static_cast<void*>(rebuildObject),
				record.physicsXmlPath,
				staleArmorBuildGroups.size());
			if (record.mergeSourceObject) {
				spdlog::debug(
					"pending armor rebuild will use preserved merge source actor={} bipedObject={} source={} sourceName='{}'",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					static_cast<void*>(record.mergeSourceObject.get()),
					std::string_view(record.mergeSourceObject->GetName()));
			}
			for (const auto& binding : record.mergeParentBindings) {
				spdlog::debug(
					"pending armor rebuild merge parent binding actor={} bipedObject={} xml='{}' source='{}' parent='{}' hasLocal={} localToParent=({:.3f},{:.3f},{:.3f})",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					record.physicsXmlPath,
					binding.sourceName,
					binding.parentName,
					binding.hasLocalToParent,
					binding.localToParent.translate.x,
					binding.localToParent.translate.y,
					binding.localToParent.translate.z);
			}
			const auto buildResult = BuildPrototypeBodiesLocked(actorState, resumeEvent, *selectedSummary, record.meshNameMap, PrototypeBuildDomain::kArmor);
			if (buildResult.succeeded) {
				if (!staleArmorBuildGroups.empty()) {
					ClearPrototypeGroupsLocked(actorState, staleArmorBuildGroups);
				}
				RecordPrototypeAttachmentLocked(actorState, record.bipedObject, rebuildObject, rebuildSourceObject, record.physicsXmlPath, buildResult.buildGroup);
				RecordPrototypeArmorLocked(
					actorState,
					record.bipedObject,
					record.physicsXmlPath,
					record.meshNameMap,
					rebuildObject,
					rebuildSourceObject,
					record.mergeSourceObject.get(),
					record.trustedActorSkeletonNodes,
					buildResult.buildGroup);
				SoftSuspendBuiltRuntimeIfOutOfRangeLocked(actorState, resumeEvent);
			} else if (buildResult.buildGroup != 0 && PrototypeBuildGroupIsRecordableLocked(actorState, buildResult.buildGroup, PrototypeBuildDomain::kArmor)) {
				ClearPrototypeGroupsLocked(actorState, std::vector<std::uint64_t>{ buildResult.buildGroup });
				spdlog::debug(
					"rolled back incomplete pending armor prototype build group actor={} bipedObject={} object={} buildGroup={} xml='{}' pendingCpuCopy={}",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					static_cast<void*>(rebuildObject),
					buildResult.buildGroup,
					record.physicsXmlPath,
					buildResult.cpuCopyPending);
			}
			if (buildResult.cpuCopyPending) {
				if (record.cpuCopyRetryCount < kCpuCopyPendingMaxRetries) {
					++record.cpuCopyRetryCount;
					a_pending.frameDelay = std::max(a_pending.frameDelay, kCpuCopyPendingRetryDelayTasks);
					spdlog::debug(
						"retrying pending prototype mesh CPU copy actor={} bipedObject={} object={} attempt={}/{} delayTasks={}",
						static_cast<void*>(a_actor),
						std::to_underlying(record.bipedObject),
						static_cast<void*>(rebuildObject),
						record.cpuCopyRetryCount,
						kCpuCopyPendingMaxRetries,
						a_pending.frameDelay);
					++it;
					continue;
				}
				spdlog::warn(
					"giving up pending prototype mesh CPU copy actor={} bipedObject={} object={} attempts={} xml='{}'",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					static_cast<void*>(rebuildObject),
					record.cpuCopyRetryCount,
					record.physicsXmlPath);
			}
			it = a_pending.armorRecords.erase(it);
		}

		return a_pending.armorRecords.empty();
	}

	void Fo4PhysicsWorld::TryRebuildPendingActorsLocked(RE::Actor* a_actor)
	{
		if (characterCustomizationMenuDepth_ > 0) {
			return;
		}
		if (pendingActorRebuilds_.empty()) {
			return;
		}
		if (!InitializeLocked()) {
			return;
		}

		for (auto it = pendingActorRebuilds_.begin(); it != pendingActorRebuilds_.end();) {
			auto resolvedActor = it->actorHandle.get();
			if (!resolvedActor) {
				it = pendingActorRebuilds_.erase(it);
				continue;
			}

			auto* actor = resolvedActor.get();
			if (!actor) {
				it = pendingActorRebuilds_.erase(it);
				continue;
			}
			if (a_actor && actor != a_actor) {
				++it;
				continue;
			}

			if (it->frameDelay > 0) {
				--it->frameDelay;
				++it;
				continue;
			}

			bool fullActorRebuild = false;
			if (it->forceArmorRescan) {
				auto* root = actor->Get3D(it->firstPerson);
				if (!root && !it->firstPerson) {
					root = actor->Get3D();
				}
				auto* biped = actor->GetBiped(it->firstPerson).get();
				if (!biped && !it->firstPerson) {
					biped = actor->GetBiped().get();
				}
				if (!root || !biped) {
					++it;
					continue;
				}

				std::vector<RE::NiAVObject*> trackedMergedObjects;
				if (auto* existingState = FindPrototypeStateLocked(actor, it->firstPerson)) {
					trackedMergedObjects.reserve(existingState->mergedNodes.size());
					for (const auto& mergedNode : existingState->mergedNodes) {
						if (mergedNode.node) {
							trackedMergedObjects.push_back(mergedNode.node.get());
						}
					}
					const auto prunedStaleMergedNodes = DetachStaleArmorMergedNodes(root, trackedMergedObjects, actor, "full-actor-rebuild-before-clear");
					if (prunedStaleMergedNodes > 0) {
						spdlog::debug(
							"pruned {} stale untracked armor merge nodes before full actor prototype rebuild actor={} firstPerson={}",
							prunedStaleMergedNodes,
							static_cast<void*>(actor),
							it->firstPerson);
					}
					ClearPrototypeStateLocked(*existingState);
					std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
						return !a_state.runtimeSuspended && !a_state.HasRuntime() && a_state.armorRecords.empty();
					});
				} else {
					const auto prunedStaleMergedNodes = DetachStaleArmorMergedNodes(root, trackedMergedObjects, actor, "full-actor-rebuild-no-state");
					if (prunedStaleMergedNodes > 0) {
						spdlog::debug(
							"pruned {} stale armor merge nodes before stateless full actor prototype rebuild actor={} firstPerson={}",
							prunedStaleMergedNodes,
							static_cast<void*>(actor),
							it->firstPerson);
					}
				}

				const LifecycleEvent rebuildEvent{
					.type = LifecycleEventType::kActorSet3D,
					.actor = actor,
					.biped = biped,
					.object = root,
					.firstPerson = it->firstPerson,
				};
				auto liveRecords = CollectSuspendedArmorRecordsLocked(rebuildEvent);
				for (auto& record : liveRecords) {
					MergePrototypeArmorRecord(it->armorRecords, std::move(record));
				}
				it->forceArmorRescan = false;
				if (it->armorRecords.empty()) {
					spdlog::debug(
						"dropping full actor prototype physics rebuild for actor={} firstPerson={} because current biped scan found no SMP armor records",
						static_cast<void*>(actor),
						it->firstPerson);
					it = pendingActorRebuilds_.erase(it);
					continue;
				}
				spdlog::debug(
					"processing full actor prototype physics rebuild for actor={} root={} firstPerson={} armorRecords={}",
					static_cast<void*>(actor),
					static_cast<void*>(root),
					it->firstPerson,
					it->armorRecords.size());
				fullActorRebuild = true;
			}

			if (!it->armorRecords.empty()) {
				if (!RebuildPendingArmorRecordsLocked(actor, *it)) {
					++it;
					continue;
				}
				if (fullActorRebuild) {
					LogActorSkeletonHierarchy(actor, it->firstPerson, "after-full-actor-prototype-rebuild");
					if (const auto* rebuiltState = FindPrototypeStateLocked(actor, it->firstPerson)) {
						LogPrototypeActorBulletObjectsLocked(*rebuiltState, "after-full-actor-prototype-rebuild");
					}
				}
				it = pendingActorRebuilds_.erase(it);
				continue;
			}

			if (auto* existingState = FindPrototypeStateLocked(actor, it->firstPerson);
				existingState && existingState->HasActiveRuntime()) {
				it = pendingActorRebuilds_.erase(it);
				continue;
			}

			auto* root = actor->Get3D(it->firstPerson);
			if (!root && !it->firstPerson) {
				root = actor->Get3D();
			}
			if (!root) {
				++it;
				continue;
			}
			auto* biped = actor->GetBiped(it->firstPerson).get();
			if (!biped && !it->firstPerson) {
				biped = actor->GetBiped().get();
			}
			const LifecycleEvent rebuildEvent{
				.type = LifecycleEventType::kActorSet3D,
				.actor = actor,
				.biped = biped,
				.object = root,
				.firstPerson = it->firstPerson,
			};
			auto discoveredRecords = rebuildEvent.biped ? CollectSuspendedArmorRecordsLocked(rebuildEvent) : std::vector<PrototypeArmorRecord>{};
			if (IsPrototypeCandidateLocked(rebuildEvent, true)) {
				spdlog::debug(
					"processing pending prototype physics rebuild for actor={} root={} firstPerson={}",
					static_cast<void*>(actor),
					static_cast<void*>(root),
					it->firstPerson);
				BuildPrototypeForEventLocked(rebuildEvent);
			}
			const auto* rebuiltState = FindPrototypeStateLocked(actor, it->firstPerson);
			const auto rebuiltRuntime = rebuiltState && rebuiltState->HasActiveRuntime();
			if (rebuiltRuntime) {
				it = pendingActorRebuilds_.erase(it);
				continue;
			}
			if (!discoveredRecords.empty()) {
				it->armorRecords = std::move(discoveredRecords);
				++it;
				continue;
			}
			if (rebuildEvent.biped) {
				spdlog::debug(
					"dropping pending prototype physics rebuild for actor={} firstPerson={} because ready biped scan found no SMP armor records",
					static_cast<void*>(actor),
					it->firstPerson);
				it = pendingActorRebuilds_.erase(it);
				continue;
			}

			++it;
		}
	}

	void Fo4PhysicsWorld::TryRebuildPendingHeadsLocked()
	{
		if (characterCustomizationMenuDepth_ > 0 || pendingHeadRebuilds_.empty()) {
			return;
		}
		if (!InitializeLocked()) {
			return;
		}

		for (auto it = pendingHeadRebuilds_.begin(); it != pendingHeadRebuilds_.end();) {
			auto resolvedActor = it->actorHandle.get();
			if (!resolvedActor) {
				it = pendingHeadRebuilds_.erase(it);
				continue;
			}

			auto* actor = resolvedActor.get();
			if (!actor) {
				it = pendingHeadRebuilds_.erase(it);
				continue;
			}

			if (it->frameDelay > 0) {
				--it->frameDelay;
				++it;
				continue;
			}

			auto* root = actor->Get3D(false);
			if (!root) {
				root = actor->Get3D();
			}
			auto* faceNode = actor->GetFaceNodeSkinned();
			if (!root || !faceNode) {
				++it;
				continue;
			}

			const LifecycleEvent headEvent{
				.type = it->type,
				.actor = actor,
				.object = it->object ? it->object.get() : reinterpret_cast<RE::NiAVObject*>(faceNode),
				.headPart = it->headPart,
				.firstPerson = false,
			};
			if (IsPrototypeCandidateLocked(headEvent, false)) {
				spdlog::debug(
					"processing pending head physics rebuild for actor={} root={} faceNode={}",
					static_cast<void*>(actor),
					static_cast<void*>(root),
					static_cast<void*>(faceNode));
				BuildHeadPrototypeForEventLocked(headEvent);
			}
			it = pendingHeadRebuilds_.erase(it);
		}
	}

	void Fo4PhysicsWorld::SuspendPrototypeStatesForCustomizationMenuLocked()
	{
		if (prototypeActors_.empty()) {
			return;
		}

		std::uint32_t suspendedStates = 0;
		std::uint32_t resetBodies = 0;
		for (auto& actorState : prototypeActors_) {
			if (!actorState.HasRuntime()) {
				continue;
			}
			resetBodies += ResetPrototypeRuntimeToReferencePoseLocked(actorState, "customization-suspend");
			SuspendPrototypeRuntimeLocked(actorState);
			++suspendedStates;
		}

		pendingActorRebuilds_.clear();
		pendingHeadRebuilds_.clear();
		suspendedActors_.clear();
		ResetStepClockLocked();
		spdlog::debug(
			"suspended {} prototype actor states for character customization menu; trackedStates={} resetBodies={}",
			suspendedStates,
			prototypeActors_.size(),
			resetBodies);
	}

	void Fo4PhysicsWorld::ReloadPrototypeStatesForCustomizationMenuLocked()
	{
		if (prototypeActors_.empty()) {
			return;
		}

		if (!InitializeLocked()) {
			return;
		}

		std::uint32_t reloadedStates = 0;
		std::uint32_t skippedFirstPerson = 0;
		std::uint32_t skippedEmpty = 0;
		std::uint32_t queuedPending = 0;
		for (auto& actorState : prototypeActors_) {
			if (actorState.firstPerson) {
				++skippedFirstPerson;
				continue;
			}
			if (actorState.armorRecords.empty()) {
				++skippedEmpty;
				continue;
			}

			auto* actor = actorState.actor;
			if (!actor) {
				if (auto resolved = actorState.actorHandle.get()) {
					actor = resolved.get();
				}
			}
			if (!actor) {
				continue;
			}

			auto records = actorState.armorRecords;
			PendingActorRebuild pending{
				.actorHandle = actorState.actorHandle,
				.firstPerson = actorState.firstPerson,
				.armorRecords = std::move(records),
			};
			if (RebuildPendingArmorRecordsLocked(actor, pending)) {
				++reloadedStates;
			} else if (!pending.armorRecords.empty()) {
				MarkPendingActorRebuildLocked(actor, actorState.firstPerson, std::move(pending.armorRecords));
				++queuedPending;
			}
		}

		std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return !a_state.actor && !a_state.HasRuntime() && a_state.armorRecords.empty();
		});
		ResetStepClockLocked();
		spdlog::debug(
			"reloaded prototype physics after character customization; reloadedStates={} queuedPending={} skippedFirstPerson={} skippedEmpty={}",
			reloadedStates,
			queuedPending,
			skippedFirstPerson,
			skippedEmpty);
	}

	void Fo4PhysicsWorld::SuspendPrototypeRuntimeLocked(PrototypeActorState& a_state)
	{
		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}

		if (dynamicsWorld_) {
			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.constraint && prototypeConstraint.inBulletWorld) {
					dynamicsWorld_->removeConstraint(prototypeConstraint.constraint.get());
					prototypeConstraint.inBulletWorld = false;
				}
			}

			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.body && prototypeMesh.inBulletWorld) {
					dynamicsWorld_->removeCollisionObject(prototypeMesh.body.get());
					prototypeMesh.inBulletWorld = false;
				}
			}

			for (auto& prototypeBody : a_state.bodies) {
				if (prototypeBody.bone && prototypeBody.inBulletWorld) {
					dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
					prototypeBody.inBulletWorld = false;
				}
			}
		}

		const auto meshCount = a_state.meshes.size();
		const auto constraintCount = a_state.constraints.size();
		const auto bodyCount = a_state.bodies.size();
		std::uint32_t capturedSkinSlots = 0;
		for (auto& prototypeBody : a_state.bodies) {
			if (!prototypeBody.bone) {
				continue;
			}
			const auto before = a_state.suspendedSkinSlots.size();
			prototypeBody.bone->CollectSkinWorldTransformRestoreSlots(a_state.suspendedSkinSlots);
			capturedSkinSlots += static_cast<std::uint32_t>(a_state.suspendedSkinSlots.size() - before);
		}
		a_state.meshes.clear();
		a_state.constraints.clear();
		a_state.bodies.clear();
		a_state.runtimes.clear();
		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.resetReadFrames = 0;
		a_state.runtimeSuspended = true;
		a_state.runtimeSoftSuspended = false;
		spdlog::debug(
			"suspended prototype runtime for actor={} bodies={} meshes={} constraints={} capturedSkinSlots={} preservedMergedNodes={} armorRecords={}",
			static_cast<void*>(a_state.actor),
			bodyCount,
			meshCount,
			constraintCount,
			capturedSkinSlots,
			a_state.mergedNodes.size(),
			a_state.armorRecords.size());
	}

	void Fo4PhysicsWorld::SoftSuspendPrototypeRuntimeLocked(PrototypeActorState& a_state)
	{
		if (!a_state.HasRuntime() || a_state.runtimeSoftSuspended) {
			return;
		}

		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}

		std::uint32_t removedConstraints = 0;
		std::uint32_t removedMeshes = 0;
		std::uint32_t removedBodies = 0;
		if (dynamicsWorld_) {
			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.constraint && prototypeConstraint.inBulletWorld) {
					dynamicsWorld_->removeConstraint(prototypeConstraint.constraint.get());
					prototypeConstraint.inBulletWorld = false;
					++removedConstraints;
				}
			}

			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.body && prototypeMesh.inBulletWorld) {
					dynamicsWorld_->removeCollisionObject(prototypeMesh.body.get());
					prototypeMesh.inBulletWorld = false;
					++removedMeshes;
				}
			}

			for (auto& prototypeBody : a_state.bodies) {
				if (prototypeBody.bone && prototypeBody.inBulletWorld) {
					dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
					prototypeBody.inBulletWorld = false;
					++removedBodies;
				}
			}
		}

		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.resetReadFrames = 0;
		a_state.currentWindFactor = 0.0F;
		a_state.runtimeSuspended = false;
		a_state.runtimeSoftSuspended = true;
		spdlog::debug(
			"soft-suspended prototype runtime for actor={} removedBodies={} removedMeshes={} removedConstraints={} retainedBodies={} retainedMeshes={} retainedConstraints={} runtimes={} armorRecords={}",
			static_cast<void*>(a_state.actor),
			removedBodies,
			removedMeshes,
			removedConstraints,
			a_state.bodies.size(),
			a_state.meshes.size(),
			a_state.constraints.size(),
			a_state.runtimes.size(),
			a_state.armorRecords.size());
	}

	bool Fo4PhysicsWorld::ResumeSoftSuspendedPrototypeRuntimeLocked(PrototypeActorState& a_state)
	{
		if (!a_state.runtimeSoftSuspended || !dynamicsWorld_) {
			return false;
		}

		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.runtimes) {
			if (runtime.buildGroup != 0 && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}
		if (buildGroups.empty()) {
			return false;
		}

		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}

		for (const auto buildGroup : buildGroups) {
			CommitPrototypeBuildGroupToBulletLocked(a_state, buildGroup);
		}

		const auto hasResumedObject = [&a_state, &buildGroups](const auto& a_collection) {
			return std::ranges::any_of(a_collection, [&buildGroups](const auto& a_object) {
				if (!a_object.inBulletWorld) {
					return false;
				}
				if constexpr (requires { a_object.buildGroup; }) {
					return std::ranges::find(buildGroups, a_object.buildGroup) != buildGroups.end();
				} else {
					return std::ranges::any_of(a_object.buildGroups, [&buildGroups](const std::uint64_t a_buildGroup) {
						return std::ranges::find(buildGroups, a_buildGroup) != buildGroups.end();
					});
				}
			});
		};
		if (!hasResumedObject(a_state.bodies) && !hasResumedObject(a_state.meshes) && !hasResumedObject(a_state.constraints)) {
			return false;
		}

		a_state.runtimeSoftSuspended = false;
		a_state.runtimeSuspended = false;
		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.resetReadFrames = 0;
		a_state.currentWindFactor = 1.0F;
		spdlog::debug(
			"resumed soft-suspended prototype runtime for actor={} buildGroups={} bodies={} meshes={} constraints={}",
			static_cast<void*>(a_state.actor),
			buildGroups.size(),
			a_state.bodies.size(),
			a_state.meshes.size(),
			a_state.constraints.size());
		return true;
	}

	std::uint32_t Fo4PhysicsWorld::RestoreSuspendedSkinSlotsLocked(
		PrototypeActorState& a_state,
		const std::span<const std::uint64_t> a_buildGroups,
		const std::span<const Fo4SkinnedMeshBone::ActiveSkinSlot> a_activeSlots)
	{
		if (a_buildGroups.empty() || a_state.suspendedSkinSlots.empty()) {
			return 0;
		}

		const auto containsGroup = [&a_buildGroups](const std::uint64_t a_buildGroup) {
			return a_buildGroup != 0 && std::ranges::find(a_buildGroups, a_buildGroup) != a_buildGroups.end();
		};
		const auto hasActiveSlot = [&a_activeSlots](RE::BSSkin::Instance* a_skin, const std::uint32_t a_index) {
			return std::ranges::any_of(a_activeSlots, [a_skin, a_index](const Fo4SkinnedMeshBone::ActiveSkinSlot& a_slot) {
				return a_slot.skin == a_skin && a_slot.index == a_index;
			});
		};
		const auto hasActiveSkin = [&a_activeSlots](RE::BSSkin::Instance* a_skin) {
			return std::ranges::any_of(a_activeSlots, [a_skin](const Fo4SkinnedMeshBone::ActiveSkinSlot& a_slot) {
				return a_slot.skin == a_skin;
			});
		};
		const auto hasRetainedSuspendedSlot = [&a_state, &containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_slot) {
			return std::ranges::any_of(a_state.suspendedSkinSlots, [&a_slot, &containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_other) {
				return a_other.buildGroup != a_slot.buildGroup &&
					!containsGroup(a_other.buildGroup) &&
					a_other.skin.get() == a_slot.skin.get() &&
					a_other.index == a_slot.index;
			});
		};
		const auto hasRetainedSuspendedSkin = [&a_state, &containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_slot) {
			return std::ranges::any_of(a_state.suspendedSkinSlots, [&a_slot, &containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_other) {
				return a_other.buildGroup != a_slot.buildGroup &&
					!containsGroup(a_other.buildGroup) &&
					a_other.skin.get() == a_slot.skin.get();
			});
		};

		std::uint32_t restored = 0;
		for (const auto& slot : a_state.suspendedSkinSlots) {
			if (!containsGroup(slot.buildGroup) || !slot.skin) {
				continue;
			}

			if (!hasActiveSlot(slot.skin.get(), slot.index) && !hasRetainedSuspendedSlot(slot)) {
				if (slot.index < slot.skin->bones.size() &&
					slot.skin->bones[slot.index] == slot.reboundBone.get() &&
					slot.originalBone) {
					slot.skin->bones[slot.index] = slot.originalBone.get();
					++restored;
				}
				if (slot.index < slot.skin->worldTransforms.size() &&
					slot.skin->worldTransforms[slot.index] == slot.reboundWorldTransform &&
					slot.originalWorldTransform) {
					slot.skin->worldTransforms[slot.index] = slot.originalWorldTransform;
					++restored;
				}
			}

			if (!hasActiveSkin(slot.skin.get()) &&
				!hasRetainedSuspendedSkin(slot) &&
				slot.originalRootNode &&
				slot.skin->rootNode != slot.originalRootNode.get()) {
				slot.skin->rootNode = slot.originalRootNode.get();
				++restored;
			}
		}

		const auto erased = std::erase_if(a_state.suspendedSkinSlots, [&containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_slot) {
			return containsGroup(a_slot.buildGroup);
		});
		if (restored > 0 || erased > 0) {
			spdlog::debug(
				"restored {} suspended prototype skin slot fields and erased {} cached slots for actor={}",
				restored,
				erased,
				static_cast<void*>(a_state.actor));
		}
		return restored;
	}

	std::uint32_t Fo4PhysicsWorld::RestoreAllSuspendedSkinSlotsLocked(PrototypeActorState& a_state)
	{
		std::vector<std::uint64_t> buildGroups;
		for (const auto& slot : a_state.suspendedSkinSlots) {
			if (slot.buildGroup != 0 && std::ranges::find(buildGroups, slot.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(slot.buildGroup);
			}
		}
		return RestoreSuspendedSkinSlotsLocked(a_state, buildGroups);
	}

	void Fo4PhysicsWorld::ClearPrototypeStateLocked(PrototypeActorState& a_state, const bool a_restoreSkinSlots)
	{
		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}

		if (dynamicsWorld_) {
			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.constraint && prototypeConstraint.inBulletWorld) {
					dynamicsWorld_->removeConstraint(prototypeConstraint.constraint.get());
					prototypeConstraint.inBulletWorld = false;
				}
			}

			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.body && prototypeMesh.inBulletWorld) {
					dynamicsWorld_->removeCollisionObject(prototypeMesh.body.get());
					prototypeMesh.inBulletWorld = false;
				}
			}
		}
		if (!a_state.meshes.empty()) {
			spdlog::debug("cleared {} prototype physics mesh bodies for actor={}", a_state.meshes.size(), static_cast<void*>(a_state.actor));
		}
		a_state.meshes.clear();

		if (!a_state.constraints.empty()) {
			spdlog::debug("cleared {} prototype physics constraints for actor={}", a_state.constraints.size(), static_cast<void*>(a_state.actor));
		}
		a_state.constraints.clear();
		a_state.runtimes.clear();

		if (dynamicsWorld_) {
			for (auto& prototypeBody : a_state.bodies) {
				if (prototypeBody.bone && prototypeBody.inBulletWorld) {
					dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
					prototypeBody.inBulletWorld = false;
				}
			}
		}

		if (a_restoreSkinSlots) {
			for (auto& prototypeBody : a_state.bodies) {
				if (!prototypeBody.bone) {
					continue;
				}
				for (const auto buildGroup : prototypeBody.buildGroups) {
					prototypeBody.bone->RemoveSkinWorldTransformsForBuildGroup(buildGroup);
				}
				if (prototypeBody.buildGroup != 0) {
					prototypeBody.bone->RemoveSkinWorldTransformsForBuildGroup(prototypeBody.buildGroup);
				}
			}
			RestoreAllSuspendedSkinSlotsLocked(a_state);
		} else if (!a_state.bodies.empty()) {
			spdlog::debug(
				"skipped restoring prototype skin slots while clearing actor={} for model rebuild",
				static_cast<void*>(a_state.actor));
			a_state.suspendedSkinSlots.clear();
		}

		if (!a_state.bodies.empty()) {
			spdlog::debug("cleared {} prototype physics bodies for actor={}", a_state.bodies.size(), static_cast<void*>(a_state.actor));
		}
		a_state.bodies.clear();
		for (auto& mergedNode : a_state.mergedNodes) {
			if (mergedNode.parent && mergedNode.node) {
				mergedNode.parent->DetachChild(mergedNode.node.get());
			}
		}
		a_state.mergedNodes.clear();
		a_state.nextBuildGroup = 0;
		a_state.nextAttachmentGeneration = 0;
		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.resetReadFrames = 0;
		a_state.currentWindFactor = 1.0F;
		a_state.runtimeSuspended = false;
		a_state.runtimeSoftSuspended = false;
		a_state.faceNode = nullptr;
		a_state.attachmentRecords.clear();
		a_state.runtimes.clear();
		a_state.suspendedSkinSlots.clear();
	}

	std::vector<std::uint64_t> Fo4PhysicsWorld::CollectPrototypeGroupsForObjectLocked(const PrototypeActorState& a_state, RE::NiAVObject* a_object) const
	{
		std::vector<std::uint64_t> buildGroups;
		if (!a_object) {
			return buildGroups;
		}
		if (a_state.actor) {
			auto* primaryRoot = a_state.actor->Get3D(a_state.firstPerson);
			auto* thirdPersonRoot = a_state.actor->Get3D(false);
			auto* firstPersonRoot = a_state.actor->Get3D(true);
			if (a_object == primaryRoot || a_object == thirdPersonRoot || a_object == firstPersonRoot || a_object == a_state.faceNode.get()) {
				spdlog::debug(
					"refusing object-scoped prototype clear from broad actor object={} actor={}; waiting for attachment/biped scoped clear",
					static_cast<void*>(a_object),
					static_cast<void*>(a_state.actor));
				return buildGroups;
			}
		}

		for (const auto& record : a_state.attachmentRecords) {
			const auto matchesAttachment =
				record.attachedObject &&
				(record.attachedObject.get() == a_object ||
					IsObjectInTree(record.attachedObject.get(), a_object) ||
					IsObjectInTree(a_object, record.attachedObject.get()));
			const auto matchesSource =
				record.sourceObject &&
				(record.sourceObject.get() == a_object ||
					IsObjectInTree(record.sourceObject.get(), a_object) ||
					IsObjectInTree(a_object, record.sourceObject.get()));
			if (!matchesAttachment && !matchesSource) {
				continue;
			}

			for (const auto buildGroup : record.buildGroups) {
				if (buildGroup != 0 && std::ranges::find(buildGroups, buildGroup) == buildGroups.end()) {
					buildGroups.push_back(buildGroup);
				}
			}
		}
		if (!buildGroups.empty()) {
			return buildGroups;
		}

		for (const auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.buildGroup == 0 || !prototypeMesh.geometry || !IsObjectInTree(a_object, prototypeMesh.geometry)) {
				continue;
			}

			if (std::ranges::find(buildGroups, prototypeMesh.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeMesh.buildGroup);
			}
		}

		for (const auto& prototypeBody : a_state.bodies) {
			if (prototypeBody.buildGroup == 0 || !prototypeBody.node || !IsNodeInTree(a_object, prototypeBody.node)) {
				continue;
			}

			if (std::ranges::find(buildGroups, prototypeBody.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeBody.buildGroup);
			}
		}

		return buildGroups;
	}

	bool Fo4PhysicsWorld::ClearPrototypeGroupsForObjectLocked(PrototypeActorState& a_state, RE::NiAVObject* a_object)
	{
		auto buildGroups = CollectPrototypeGroupsForObjectLocked(a_state, a_object);
		if (buildGroups.empty()) {
			return false;
		}

		ClearPrototypeGroupsLocked(a_state, buildGroups);
		return true;
	}

	bool Fo4PhysicsWorld::ClearPrototypeGroupsForBipedObjectLocked(PrototypeActorState& a_state, const RE::BIPED_OBJECT a_bipedObject)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return false;
		}

		const auto attachmentCount = static_cast<std::size_t>(std::ranges::count_if(a_state.attachmentRecords, [a_bipedObject](const PrototypeAttachmentRecord& a_record) {
			return a_record.bipedObject == a_bipedObject && !a_record.buildGroups.empty();
		}));
		const auto armorRecordCount = static_cast<std::size_t>(std::ranges::count_if(a_state.armorRecords, [a_bipedObject](const PrototypeArmorRecord& a_record) {
			return a_record.bipedObject == a_bipedObject;
		}));
		if (attachmentCount > 1 || armorRecordCount > 1) {
			spdlog::warn(
				"refusing biped-wide prototype clear actor={} bipedObject={} attachments={} armorRecords={} because multiple same-slot systems are tracked",
				static_cast<void*>(a_state.actor),
				std::to_underlying(a_bipedObject),
				attachmentCount,
				armorRecordCount);
			return false;
		}

		std::vector<std::uint64_t> buildGroups;
		for (const auto& attachment : a_state.attachmentRecords) {
			if (attachment.bipedObject == a_bipedObject) {
				buildGroups.insert(buildGroups.end(), attachment.buildGroups.begin(), attachment.buildGroups.end());
			}
		}
		const auto appendGroup = [&buildGroups](const std::uint64_t a_buildGroup) {
			if (a_buildGroup != 0 && std::ranges::find(buildGroups, a_buildGroup) == buildGroups.end()) {
				buildGroups.push_back(a_buildGroup);
			}
		};

		for (const auto& runtime : a_state.runtimes) {
			if (runtime.bipedObject == a_bipedObject) {
				appendGroup(runtime.buildGroup);
			}
		}

		for (const auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.bipedObject == a_bipedObject) {
				appendGroup(prototypeMesh.buildGroup);
			}
		}

		for (const auto& prototypeBody : a_state.bodies) {
			if (!prototypeBody.buildGroupBipedObjects.empty()) {
				for (const auto& [buildGroup, bipedObject] : prototypeBody.buildGroupBipedObjects) {
					if (bipedObject == a_bipedObject) {
						appendGroup(buildGroup);
					}
				}
			} else if (prototypeBody.bipedObject == a_bipedObject) {
				appendGroup(prototypeBody.buildGroup);
			}
		}

		if (buildGroups.empty()) {
			return false;
		}

		ClearPrototypeGroupsLocked(a_state, buildGroups);
		std::erase_if(a_state.armorRecords, [a_bipedObject](const PrototypeArmorRecord& a_record) {
			return a_record.bipedObject == a_bipedObject;
		});
		return true;
	}

	bool Fo4PhysicsWorld::ClearPrototypeGroupsForBoneNamesLocked(PrototypeActorState& a_state, const std::span<const std::string> a_boneNames, const PrototypeBuildDomain a_domain)
	{
		if (a_boneNames.empty()) {
			return false;
		}
		if (a_domain == PrototypeBuildDomain::kArmor) {
			spdlog::warn(
				"refusing to clear armor prototype groups by bone names actor={} names={} because actor skeleton bones may be shared across armor XMLs",
				static_cast<void*>(a_state.actor),
				a_boneNames.size());
			return false;
		}

		std::vector<std::uint64_t> buildGroups;
		const auto appendGroup = [&buildGroups](const std::uint64_t a_buildGroup) {
			if (a_buildGroup != 0 && std::ranges::find(buildGroups, a_buildGroup) == buildGroups.end()) {
				buildGroups.push_back(a_buildGroup);
			}
		};
		for (const auto& prototypeBody : a_state.bodies) {
			if (prototypeBody.boneName.empty()) {
				continue;
			}
			const auto nameMatched = std::ranges::any_of(a_boneNames, [&prototypeBody](const std::string& a_boneName) {
				return PhysicsNamesEqual(prototypeBody.boneName, a_boneName);
			});
			if (!nameMatched) {
				continue;
			}

			if (!prototypeBody.buildGroupDomains.empty()) {
				for (const auto& [buildGroup, domain] : prototypeBody.buildGroupDomains) {
					if (domain == a_domain) {
						appendGroup(buildGroup);
					}
				}
			} else if (prototypeBody.buildGroup != 0) {
				appendGroup(prototypeBody.buildGroup);
			}
		}

		if (buildGroups.empty()) {
			return false;
		}

		ClearPrototypeGroupsLocked(a_state, buildGroups);
		return true;
	}

	bool Fo4PhysicsWorld::ClearPrototypeGroupsByDomainLocked(PrototypeActorState& a_state, const PrototypeBuildDomain a_domain)
	{
		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.runtimes) {
			if (runtime.buildGroup != 0 && runtime.domain == a_domain && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}
		for (const auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.buildGroup != 0 && prototypeMesh.domain == a_domain && std::ranges::find(buildGroups, prototypeMesh.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeMesh.buildGroup);
			}
		}
		for (const auto& prototypeConstraint : a_state.constraints) {
			if (prototypeConstraint.buildGroup != 0 && prototypeConstraint.domain == a_domain && std::ranges::find(buildGroups, prototypeConstraint.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeConstraint.buildGroup);
			}
		}
		for (const auto& prototypeBody : a_state.bodies) {
			for (const auto& [buildGroup, domain] : prototypeBody.buildGroupDomains) {
				if (buildGroup != 0 && domain == a_domain && std::ranges::find(buildGroups, buildGroup) == buildGroups.end()) {
					buildGroups.push_back(buildGroup);
				}
			}
		}

		if (buildGroups.empty()) {
			return false;
		}

		ClearPrototypeGroupsLocked(a_state, buildGroups);
		return true;
	}

	void Fo4PhysicsWorld::ClearPrototypeGroupsLocked(PrototypeActorState& a_state, const std::vector<std::uint64_t>& a_buildGroups)
	{
		if (a_buildGroups.empty()) {
			return;
		}

		const auto containsGroup = [&a_buildGroups](const std::uint64_t a_buildGroup) {
			return std::ranges::find(a_buildGroups, a_buildGroup) != a_buildGroups.end();
		};
		const auto allGroupsRemoved = [&containsGroup](const PrototypeBody& a_body) {
			return !a_body.buildGroups.empty() ?
				std::ranges::all_of(a_body.buildGroups, containsGroup) :
				containsGroup(a_body.buildGroup);
		};
		if (dynamicsWorld_) {
			for (const auto& runtime : a_state.runtimes) {
				if (!containsGroup(runtime.buildGroup)) {
					continue;
				}

				for (auto* constraint : runtime.constraints) {
					if (!constraint) {
						continue;
					}
					dynamicsWorld_->removeConstraint(constraint);
					for (auto& prototypeConstraint : a_state.constraints) {
						if (prototypeConstraint.constraint.get() == constraint) {
							prototypeConstraint.inBulletWorld = false;
							break;
						}
					}
				}

				for (auto* mesh : runtime.meshes) {
					if (!mesh) {
						continue;
					}
					dynamicsWorld_->removeCollisionObject(mesh);
					for (auto& prototypeMesh : a_state.meshes) {
						if (prototypeMesh.body.get() == mesh) {
							prototypeMesh.inBulletWorld = false;
							break;
						}
					}
				}

				for (auto& prototypeBody : a_state.bodies) {
					if (prototypeBody.bone &&
						prototypeBody.inBulletWorld &&
						allGroupsRemoved(prototypeBody) &&
						std::ranges::find(runtime.bones, prototypeBody.bone.get()) != runtime.bones.end()) {
						dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
						prototypeBody.inBulletWorld = false;
					}
				}
			}

			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.constraint && prototypeConstraint.inBulletWorld && containsGroup(prototypeConstraint.buildGroup)) {
					dynamicsWorld_->removeConstraint(prototypeConstraint.constraint.get());
					prototypeConstraint.inBulletWorld = false;
				}
			}

			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.body && prototypeMesh.inBulletWorld && containsGroup(prototypeMesh.buildGroup)) {
					dynamicsWorld_->removeCollisionObject(prototypeMesh.body.get());
					prototypeMesh.inBulletWorld = false;
				}
			}

			for (auto& prototypeBody : a_state.bodies) {
				if (prototypeBody.bone && prototypeBody.inBulletWorld && allGroupsRemoved(prototypeBody)) {
					dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
					prototypeBody.inBulletWorld = false;
				}
			}
		}

		std::vector<Fo4SkinnedMeshBone::ActiveSkinSlot> activeSkinSlots;
		for (const auto& prototypeBody : a_state.bodies) {
			if (allGroupsRemoved(prototypeBody) || !prototypeBody.bone) {
				continue;
			}
			prototypeBody.bone->CollectSkinWorldTransformSlots(activeSkinSlots);
		}
		for (const auto& suspendedSlot : a_state.suspendedSkinSlots) {
			if (!suspendedSlot.skin || containsGroup(suspendedSlot.buildGroup)) {
				continue;
			}
			activeSkinSlots.push_back({
				.skin = suspendedSlot.skin.get(),
				.index = suspendedSlot.index,
				.buildGroup = suspendedSlot.buildGroup,
			});
		}

		for (auto& prototypeBody : a_state.bodies) {
			if (!prototypeBody.bone) {
				continue;
			}

			for (const auto buildGroup : a_buildGroups) {
				prototypeBody.bone->RemoveSkinWorldTransformsForBuildGroup(buildGroup, activeSkinSlots);
			}
		}
		RestoreSuspendedSkinSlotsLocked(a_state, a_buildGroups, activeSkinSlots);

		const auto meshCount = std::erase_if(a_state.meshes, [&containsGroup](const PrototypeMesh& a_mesh) {
			return containsGroup(a_mesh.buildGroup);
		});
		const auto constraintCount = std::erase_if(a_state.constraints, [&containsGroup](const PrototypeConstraint& a_constraint) {
			return containsGroup(a_constraint.buildGroup);
		});
		const auto runtimeCount = std::erase_if(a_state.runtimes, [&containsGroup](const PrototypeBuildGroupRuntime& a_runtime) {
			return containsGroup(a_runtime.buildGroup);
		});

		for (auto& body : a_state.bodies) {
			if (body.buildGroups.empty() && body.buildGroup != 0) {
				body.buildGroups.push_back(body.buildGroup);
			}
			if (body.buildGroupDomains.empty() && body.buildGroup != 0) {
				body.buildGroupDomains.push_back({ body.buildGroup, PrototypeBuildDomain::kArmor });
			}
			if (body.buildGroupBipedObjects.empty() && body.buildGroup != 0) {
				body.buildGroupBipedObjects.push_back({ body.buildGroup, body.bipedObject });
			}
			std::erase_if(body.buildGroups, [&containsGroup](const std::uint64_t a_buildGroup) {
				return containsGroup(a_buildGroup);
			});
			std::erase_if(body.buildGroupDomains, [&containsGroup](const auto& a_entry) {
				return containsGroup(a_entry.first);
			});
			std::erase_if(body.buildGroupBipedObjects, [&containsGroup](const auto& a_entry) {
				return containsGroup(a_entry.first);
			});
			if (!body.buildGroups.empty()) {
				body.buildGroup = body.buildGroups.front();
				const auto biped = std::ranges::find_if(body.buildGroupBipedObjects, [&body](const auto& a_entry) {
					return a_entry.first == body.buildGroup;
				});
				body.bipedObject = biped != body.buildGroupBipedObjects.end() ? biped->second : RE::BIPED_OBJECT::kTotal;
			}
		}
		const auto bodyCount = std::erase_if(a_state.bodies, [](const PrototypeBody& a_body) {
			return a_body.buildGroups.empty();
		});
		const auto mergedNodeCount = std::erase_if(a_state.mergedNodes, [&containsGroup](PrototypeMergedNode& a_node) {
			if (!containsGroup(a_node.buildGroup)) {
				return false;
			}
			auto* node = a_node.node ? a_node.node->IsNode() : nullptr;
			if (a_node.parent && node && node->parent == a_node.parent) {
				a_node.parent->DetachChild(a_node.node.get());
			}
			return true;
		});
		std::erase_if(a_state.attachmentRecords, [&containsGroup](PrototypeAttachmentRecord& a_record) {
			std::erase_if(a_record.buildGroups, [&containsGroup](const std::uint64_t a_buildGroup) {
				return containsGroup(a_buildGroup);
			});
			if (!a_record.buildGroups.empty()) {
				return false;
			}
			a_record.attachedObject = nullptr;
			a_record.sourceObject = nullptr;
			return true;
		});
		const auto armorRecordCount = std::erase_if(a_state.armorRecords, [&containsGroup](PrototypeArmorRecord& a_record) {
			if (a_record.buildGroups.empty()) {
				return false;
			}
			std::erase_if(a_record.buildGroups, [&containsGroup](const std::uint64_t a_buildGroup) {
				return containsGroup(a_buildGroup);
			});
			if (!a_record.buildGroups.empty()) {
				return false;
			}
			a_record.attachedObject = nullptr;
			a_record.sourceObject = nullptr;
			a_record.mergeSourceObject = nullptr;
			return true;
		});
		if (!a_state.HasRuntime()) {
			a_state.runtimeSoftSuspended = false;
		}

		spdlog::debug(
			"cleared prototype physics groups={} runtimes={} bodies={} meshes={} constraints={} mergedNodes={} armorRecords={} for actor={}",
			a_buildGroups.size(),
			runtimeCount,
			bodyCount,
			meshCount,
			constraintCount,
			mergedNodeCount,
			armorRecordCount,
			static_cast<void*>(a_state.actor));
	}

	void Fo4PhysicsWorld::ClearAllPrototypeStatesLocked()
	{
		for (auto& actorState : prototypeActors_) {
			ClearPrototypeStateLocked(actorState);
		}
		prototypeActors_.clear();
		suspendedActors_.clear();
	}

	void Fo4PhysicsWorld::ResumeFromLoadingMenuLocked()
	{
		std::size_t resetBodies = 0;
		for (auto& actorState : prototypeActors_) {
			if (actorState.runtimeSoftSuspended) {
				continue;
			}
			actorState.lastWritebackFrame = 0;
			actorState.lastWritebackSource = WritebackSource::kUnknown;
			actorState.resetReadFrames = std::max(actorState.resetReadFrames, kAttachResetReadFrames);
			actorState.currentWindFactor = 1.0F;
			if (!actorState.runtimes.empty()) {
				for (const auto& runtime : actorState.runtimes) {
					for (auto* bone : runtime.bones) {
						if (!bone) {
							continue;
						}
						bone->readTransform(0.0F);
						++resetBodies;
					}
					ScalePrototypeConstraintsLocked(actorState, runtime);
				}
			} else {
				for (auto& prototypeBody : actorState.bodies) {
					if (!prototypeBody.bone) {
						continue;
					}
					prototypeBody.bone->readTransform(0.0F);
					++resetBodies;
				}
				ScalePrototypeConstraintsLocked(actorState);
			}
		}
		if (dynamicsWorld_) {
			dynamicsWorld_->clearForces();
		}
		loadingPhysicsSuspended_ = false;
		loadingMenuDepth_ = 0;
		ResetStepClockLocked();
		spdlog::debug("loading menu resume reset {} prototype physics bodies to current node poses", resetBodies);
	}

	float Fo4PhysicsWorld::PreparePrototypeActorForReadLocked(PrototypeActorState& a_state, float a_timeStep)
	{
		auto* actorRoot = a_state.actor ? a_state.actor->Get3D(a_state.firstPerson) : nullptr;
		if (!actorRoot && a_state.actor && !a_state.firstPerson) {
			actorRoot = a_state.actor->Get3D();
		}
		auto* skeletonRoot = actorRoot ? actorRoot->IsNode() : nullptr;
		if (!skeletonRoot) {
			return a_timeStep;
		}

		auto* topRoot = static_cast<RE::NiAVObject*>(skeletonRoot);
		while (topRoot && topRoot->parent) {
			topRoot = topRoot->parent;
		}

		if (a_state.lastReadRoot && a_state.lastReadRoot.get() != topRoot) {
			a_timeStep = 0.0F;
		}
		if (!a_state.readInitialized) {
			a_timeStep = 0.0F;
			a_state.readInitialized = true;
		}

		if (a_timeStep <= 0.0F) {
			UpdateTransformUpDown(skeletonRoot, true);
			a_state.lastReadRoot = topRoot;
			return 0.0F;
		}

		a_state.lastReadRoot = topRoot;
		return a_timeStep;
	}

	bool Fo4PhysicsWorld::PrototypeBuildGroupHasMeshLocked(const PrototypeActorState& a_state, const std::uint64_t a_buildGroup) const
	{
		if (a_buildGroup == 0) {
			return false;
		}

		return std::ranges::any_of(a_state.meshes, [a_buildGroup](const PrototypeMesh& a_mesh) {
			return a_mesh.buildGroup == a_buildGroup && a_mesh.body;
		});
	}

	bool Fo4PhysicsWorld::PrototypeBuildGroupHasBodyLocked(const PrototypeActorState& a_state, const std::uint64_t a_buildGroup) const
	{
		if (a_buildGroup == 0) {
			return false;
		}

		return std::ranges::any_of(a_state.bodies, [a_buildGroup](const PrototypeBody& a_body) {
			return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && a_body.bone;
		});
	}

	bool Fo4PhysicsWorld::PrototypeBuildGroupIsRecordableLocked(
		const PrototypeActorState& a_state,
		const std::uint64_t a_buildGroup,
		const PrototypeBuildDomain a_domain) const
	{
		if (!PrototypeBuildGroupHasBodyLocked(a_state, a_buildGroup)) {
			return false;
		}

		if (a_domain == PrototypeBuildDomain::kArmor) {
			return PrototypeBuildGroupHasMeshLocked(a_state, a_buildGroup);
		}

		return true;
	}

	void Fo4PhysicsWorld::CommitPrototypeBuildGroupToBulletLocked(PrototypeActorState& a_state, const std::uint64_t a_buildGroup)
	{
		if (!dynamicsWorld_ || a_buildGroup == 0) {
			return;
		}

		std::uint32_t committedMeshes = 0;
		for (auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.inBulletWorld || !prototypeMesh.body || prototypeMesh.buildGroup != a_buildGroup) {
				continue;
			}

			dynamicsWorld_->addCollisionObject(prototypeMesh.body.get(), 1, 1);
			prototypeMesh.inBulletWorld = true;
			++committedMeshes;
		}

		std::uint32_t committedBodies = 0;
		for (auto& prototypeBody : a_state.bodies) {
			if (prototypeBody.inBulletWorld || !prototypeBody.bone || !PrototypeBodyHasBuildGroup(prototypeBody, a_buildGroup)) {
				continue;
			}

			// hdtSMP bones are solver/constraint bodies; mesh collisions are handled by the custom dispatcher.
			dynamicsWorld_->addRigidBody(std::addressof(prototypeBody.bone->m_rig), 0, 0);
			prototypeBody.inBulletWorld = true;
			++committedBodies;
		}

		std::uint32_t committedConstraints = 0;
		for (auto& prototypeConstraint : a_state.constraints) {
			if (prototypeConstraint.inBulletWorld || !prototypeConstraint.constraint || prototypeConstraint.buildGroup != a_buildGroup) {
				continue;
			}

			dynamicsWorld_->addConstraint(prototypeConstraint.constraint.get(), true);
			prototypeConstraint.inBulletWorld = true;
			++committedConstraints;
		}

		if (committedBodies > 0 || committedMeshes > 0 || committedConstraints > 0) {
			auto runtime = std::ranges::find_if(a_state.runtimes, [a_buildGroup](const PrototypeBuildGroupRuntime& a_runtime) {
				return a_runtime.buildGroup == a_buildGroup;
			});
			if (runtime == a_state.runtimes.end()) {
				PrototypeBuildGroupRuntime newRuntime;
				newRuntime.buildGroup = a_buildGroup;
				if (const auto mesh = std::ranges::find_if(a_state.meshes, [a_buildGroup](const PrototypeMesh& a_mesh) {
						return a_mesh.buildGroup == a_buildGroup;
					});
					mesh != a_state.meshes.end()) {
					newRuntime.domain = mesh->domain;
					newRuntime.bipedObject = mesh->bipedObject;
				} else if (const auto constraint = std::ranges::find_if(a_state.constraints, [a_buildGroup](const PrototypeConstraint& a_constraint) {
						return a_constraint.buildGroup == a_buildGroup;
					});
					constraint != a_state.constraints.end()) {
					newRuntime.domain = constraint->domain;
				}
				a_state.runtimes.push_back(newRuntime);
				runtime = std::prev(a_state.runtimes.end());
			}
			runtime->meshes.clear();
			runtime->bones.clear();
			runtime->constraints.clear();
			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.buildGroup != a_buildGroup || !prototypeMesh.body) {
					continue;
				}
				runtime->meshes.push_back(prototypeMesh.body.get());
				runtime->domain = prototypeMesh.domain;
				runtime->bipedObject = prototypeMesh.bipedObject;
			}
			for (auto& prototypeBody : a_state.bodies) {
				if (!PrototypeBodyHasBuildGroup(prototypeBody, a_buildGroup) || !prototypeBody.bone) {
					continue;
				}
				runtime->bones.push_back(prototypeBody.bone.get());
				if (runtime->bipedObject == RE::BIPED_OBJECT::kTotal) {
					const auto biped = std::ranges::find_if(prototypeBody.buildGroupBipedObjects, [a_buildGroup](const auto& a_entry) {
						return a_entry.first == a_buildGroup;
					});
					runtime->bipedObject = biped != prototypeBody.buildGroupBipedObjects.end() ? biped->second : prototypeBody.bipedObject;
				}
				const auto domain = std::ranges::find_if(prototypeBody.buildGroupDomains, [a_buildGroup](const auto& a_entry) {
					return a_entry.first == a_buildGroup;
				});
				if (domain != prototypeBody.buildGroupDomains.end()) {
					runtime->domain = domain->second;
				}
			}
			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.buildGroup != a_buildGroup || !prototypeConstraint.constraint) {
					continue;
				}
				runtime->constraints.push_back(prototypeConstraint.constraint.get());
				runtime->domain = prototypeConstraint.domain;
			}
			ResetPrototypeBuildGroupToCurrentPoseLocked(a_state, a_buildGroup);
			spdlog::debug(
				"committed prototype build group to Bullet actor={} buildGroup={} domain={} bipedObject={} bodies={} meshes={} constraints={}",
				static_cast<void*>(a_state.actor),
				a_buildGroup,
				PrototypeDomainName(runtime->domain),
				std::to_underlying(runtime->bipedObject),
				runtime->bones.size(),
				runtime->meshes.size(),
				runtime->constraints.size());
		}
	}

	Fo4PhysicsWorld::PrototypeBuildResult Fo4PhysicsWorld::BuildPrototypeBodiesLocked(PrototypeActorState& a_state, const LifecycleEvent& a_event, const PhysicsXmlSummary& a_summary, const DefaultBBP::NameMap& a_meshNameMap, const PrototypeBuildDomain a_domain)
	{
		struct BuildTiming
		{
			float actorTreePrepMs{ 0.0F };
			float cloneMergeMs{ 0.0F };
			float referencePoseMs{ 0.0F };
			float xmlSkinResolveMs{ 0.0F };
			float bulletBodyMs{ 0.0F };
			float meshBuildMs{ 0.0F };
			float bindCommitConstraintMs{ 0.0F };
		};

		const auto buildTimingStart = Clock::now();
		BuildTiming timing;
		auto logBuildTiming = [&](const char* a_reason, const std::uint64_t a_buildGroup = 0) {
			spdlog::debug(
				"prototype build timing actor={} domain={} bipedObject={} buildGroup={} reason={} totalMs={:.3f} actorTreePrepMs={:.3f} cloneMergeMs={:.3f} referencePoseMs={:.3f} xmlSkinResolveMs={:.3f} bulletBodyMs={:.3f} meshBuildMs={:.3f} bindCommitConstraintMs={:.3f}",
				static_cast<void*>(a_state.actor),
				PrototypeDomainName(a_domain),
				std::to_underlying(a_event.bipedObject),
				a_buildGroup,
				a_reason,
				ElapsedMs(buildTimingStart, Clock::now()),
				timing.actorTreePrepMs,
				timing.cloneMergeMs,
				timing.referencePoseMs,
				timing.xmlSkinResolveMs,
				timing.bulletBodyMs,
				timing.meshBuildMs,
				timing.bindCommitConstraintMs);
		};

		a_state.runtimeSuspended = false;
		a_state.runtimeSoftSuspended = false;
		auto meshNames = BuildMeshMatchNames(a_summary, a_meshNameMap);
		if (a_summary.boneNames.empty() && meshNames.empty()) {
			spdlog::debug("prototype physics XML has no named bones or mesh descriptors to match");
			logBuildTiming("empty-xml");
			return {};
		}

		std::vector<MatchedSkinBone> matchedBones;
		auto phaseStart = Clock::now();
		auto* skeletonSearchRoot = ResolveSkeletonSearchRoot(a_event);
		const auto actorSkeletonSearchExclusions = BuildBipedPartCloneExclusions(a_event);
		const auto knownArmorNodes = BuildKnownArmorNodeSet(a_event);
		auto* actorRoot = a_event.actor ? a_event.actor->Get3D(a_event.firstPerson) : nullptr;
		if (!actorRoot && a_event.actor) {
			actorRoot = a_event.actor->Get3D();
		}
		auto* actorRootNode = actorRoot ? actorRoot->IsNode() : nullptr;
		if (actorRootNode) {
			UpdateTransformUpDown(actorRootNode, true);
		}
		timing.actorTreePrepMs += ElapsedMs(phaseStart, Clock::now());
		std::vector<MergedSkeletonNode> mergedSkeletonNodes;
		std::vector<MergedRootNode> mergedRootNodes;
		std::vector<SavedNodeLocalPose> savedBuildPoses;
		RE::NiPointer<RE::NiAVObject> preservedSourceClone;
		phaseStart = Clock::now();
		const auto trustedActorSkeletonNodes = BuildTrustedActorSkeletonNodeSet(a_event);
		if (!trustedActorSkeletonNodes.empty()) {
			spdlog::debug(
				"using pre-attach trusted actor skeleton node set actor={} nodes={}",
				static_cast<void*>(a_event.actor),
				trustedActorSkeletonNodes.size());
		}
		const auto actorSkeletonLookup = BuildActorSkeletonLookup(actorRoot, actorSkeletonSearchExclusions, knownArmorNodes, trustedActorSkeletonNodes);
		const auto mergePrefix = !a_event.mergeRenamePrefix.empty() ? a_event.mergeRenamePrefix :
			(a_domain == PrototypeBuildDomain::kHead || a_domain == PrototypeBuildDomain::kHair) ?
				MakeReferenceHeadRenamePrefix(PrototypeHeadRenameId.fetch_add(1, std::memory_order_relaxed)) :
				MakeReferenceArmorRenamePrefix(PrototypeArmorRenameId.fetch_add(1, std::memory_order_relaxed));
		auto* mergeSourceObject = a_event.mergeSourceObject ? a_event.mergeSourceObject : a_event.sourceObject;
		auto* sourceRoot = mergeSourceObject ? mergeSourceObject->IsNode() : a_event.sourceRoot;
		const auto smpClonedPrefix = MakeReferenceSmpClonedPrefix(mergePrefix);
		if (sourceRoot && a_event.preserveMergeSourceNames) {
			preservedSourceClone = CloneNodeExact(sourceRoot);
			if (auto* clonedRoot = preservedSourceClone ? preservedSourceClone->IsNode() : nullptr) {
				sourceRoot = clonedRoot;
			}
		}
		if (sourceRoot) {
			UpdateNodeWorldFromLocal(sourceRoot);
			auto* liveCloneParent = actorRootNode;
			if (!liveCloneParent) {
				liveCloneParent = a_event.object ? a_event.object->IsNode() : nullptr;
			}
			if (!liveCloneParent) {
				liveCloneParent = sourceRoot->parent ? sourceRoot->parent : sourceRoot;
			}

			const auto sourceRootName = sourceRoot->GetName();
			const auto sourceRootIsActorBone = FindTrustedActorSkeletonNodeForSource(actorRoot, sourceRoot, actorSkeletonLookup, nullptr, nullptr, nullptr, actorSkeletonSearchExclusions, knownArmorNodes, trustedActorSkeletonNodes) != nullptr;
			if (sourceRoot == liveCloneParent && !sourceRootIsActorBone && !sourceRootName.empty() && IsReferencedXmlBoneName(a_summary, sourceRootName) && sourceRoot->parent) {
				liveCloneParent = sourceRoot->parent;
			}
			if (!sourceRootName.empty() && !sourceRootIsActorBone && HasRelevantXmlDescendant(sourceRoot, a_summary)) {
				spdlog::debug(
					"selectively cloning relevant armor bones from source root '{}' node={} under merge destination parent={} parentName='{}' prefix='{}'",
					sourceRootName,
					static_cast<void*>(sourceRoot),
					static_cast<void*>(liveCloneParent),
					std::string_view(liveCloneParent->GetName()),
					smpClonedPrefix);
			}
			CloneSourceSkeletonIntoPartTree(
				liveCloneParent,
				sourceRoot,
				actorRoot,
				actorSkeletonLookup,
				actorSkeletonSearchExclusions,
				knownArmorNodes,
				trustedActorSkeletonNodes,
				smpClonedPrefix,
				a_summary,
				a_event.mergeParentBindings,
				mergedSkeletonNodes,
				mergedRootNodes);
		}
		if (actorRootNode) {
			UpdateTransformUpDown(actorRootNode, true);
		}
		auto* skeletonLookupRoot = actorRootNode ? static_cast<RE::NiAVObject*>(actorRootNode) : skeletonSearchRoot;
		const auto skeletonLookupExclusions = BuildSkeletonLookupExclusions(actorSkeletonSearchExclusions, mergedRootNodes);
		const auto skeletonLookupKnownArmorNodes = BuildKnownArmorNodeSet(a_event, std::addressof(mergedRootNodes));
		timing.cloneMergeMs += ElapsedMs(phaseStart, Clock::now());
		if (actorRootNode) {
			phaseStart = Clock::now();
			if (!ApplyHavokReferencePose(
					a_event.actor,
					actorRootNode,
					skeletonLookupExclusions,
					skeletonLookupKnownArmorNodes,
					trustedActorSkeletonNodes,
					savedBuildPoses)) {
				UpdateTransformUpDown(actorRootNode, true);
				spdlog::debug(
					"prototype physics build used current actor pose because Havok reference pose was unavailable actor={} root={}",
					static_cast<void*>(a_event.actor),
					static_cast<void*>(actorRootNode));
			}
			timing.referencePoseMs += ElapsedMs(phaseStart, Clock::now());
		}
		phaseStart = Clock::now();
		CollectMatchedSkinBones(a_event.object, a_summary.boneNames, meshNames, matchedBones);
		ResolveExplicitXmlBonesFromMergedSkeleton(
			matchedBones,
			a_summary,
			a_summary.boneNames,
			mergedSkeletonNodes,
			skeletonLookupRoot,
			actorSkeletonLookup,
			sourceRoot,
			a_event.object,
			a_event.sourceObject,
			mergeSourceObject,
			skeletonLookupExclusions,
			skeletonLookupKnownArmorNodes,
			trustedActorSkeletonNodes);
		ResolveMatchedSkinBonesFromSkeleton(
			matchedBones,
			a_summary,
			mergedSkeletonNodes,
			skeletonLookupRoot,
			actorSkeletonLookup,
			sourceRoot,
			a_event.object,
			a_event.sourceObject,
			mergeSourceObject,
			skeletonLookupExclusions,
			skeletonLookupKnownArmorNodes,
			trustedActorSkeletonNodes);
		if (a_domain == PrototypeBuildDomain::kArmor) {
			const auto removedRawArmorBones = std::erase_if(matchedBones, [&](const MatchedSkinBone& a_matchedBone) {
				const auto remove = IsUnresolvedArmorOwnedMatchedBone(
					a_matchedBone,
					actorRoot,
					sourceRoot,
					a_event.object,
					a_event.sourceObject,
					mergeSourceObject,
					skeletonLookupExclusions,
					skeletonLookupKnownArmorNodes);
				if (remove) {
					spdlog::debug(
						"dropping unresolved armor-owned skin bone '{}' node={} nodeName='{}'; no actor-root fallback will target raw partClone/source nodes",
						a_matchedBone.name,
						static_cast<void*>(a_matchedBone.node),
						a_matchedBone.node ? std::string_view(a_matchedBone.node->GetName()) : std::string_view{});
				}
				return remove;
			});
			if (removedRawArmorBones > 0) {
				spdlog::debug("dropped {} unresolved armor-owned skin bones before prototype body creation", removedRawArmorBones);
			}
		}
		timing.xmlSkinResolveMs += ElapsedMs(phaseStart, Clock::now());
		if (matchedBones.empty()) {
			spdlog::debug(
				"prototype physics XML matched no skin bones for {} actor={} object={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
			RestoreSavedLocalPoses(savedBuildPoses, actorRootNode);
			DetachMergedRootNodes(mergedRootNodes);
			logBuildTiming("no-matched-bones");
			return {};
		}

		std::uint32_t created = 0;
		std::uint32_t dynamicBodies = 0;
		std::uint32_t kinematicBodies = 0;
		std::uint32_t matchedUnderActorRoot = 0;
		std::uint32_t matchedUnderAttachedObject = 0;
		std::uint64_t buildGroup = 0;
		const auto hadActorRuntimeBeforeBuild = a_state.HasRuntime();
		for (const auto& matchedBone : matchedBones) {
			if (IsNodeInTree(actorRoot, matchedBone.node)) {
				++matchedUnderActorRoot;
			}
			if (IsNodeInTree(a_event.object, matchedBone.node)) {
				++matchedUnderAttachedObject;
			}
		}
		if (a_domain != PrototypeBuildDomain::kArmor) {
			for (const auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.domain == a_domain && prototypeMesh.geometry && IsObjectInTree(a_event.object, prototypeMesh.geometry)) {
					buildGroup = prototypeMesh.buildGroup;
					break;
				}
			}
			for (const auto& prototypeBody : a_state.bodies) {
				if (buildGroup != 0) {
					break;
				}
				if (prototypeBody.node && IsNodeInTree(a_event.object, prototypeBody.node)) {
					const auto existingGroup = std::ranges::find_if(prototypeBody.buildGroupDomains, [a_domain](const auto& a_entry) {
						return a_entry.second == a_domain;
					});
					if (existingGroup != prototypeBody.buildGroupDomains.end()) {
						buildGroup = existingGroup->first;
						break;
					}
				}
			}
			for (const auto& matchedBone : matchedBones) {
				if (buildGroup != 0) {
					break;
				}
				const auto existing = std::ranges::find_if(a_state.bodies, [&matchedBone](const PrototypeBody& a_body) {
					return a_body.node == matchedBone.node;
				});
				if (existing == a_state.bodies.end()) {
					continue;
				}
				const auto existingGroup = std::ranges::find_if(existing->buildGroupDomains, [a_domain](const auto& a_entry) {
					return a_entry.second == a_domain;
				});
				if (existingGroup != existing->buildGroupDomains.end()) {
					buildGroup = existingGroup->first;
				}
			}
		}
		bool createdBuildGroup = false;
		if (buildGroup == 0) {
			buildGroup = ++a_state.nextBuildGroup;
			createdBuildGroup = true;
		}
		PrototypeBuildResult result;
		result.buildGroup = buildGroup;
		struct BuildGroupRollbackGuard
		{
			Fo4PhysicsWorld* world{ nullptr };
			PrototypeActorState* state{ nullptr };
			std::vector<MergedRootNode>* mergedRoots{ nullptr };
			std::uint64_t buildGroup{ 0 };
			bool active{ false };

			~BuildGroupRollbackGuard()
			{
				if (!active || !world || !state || buildGroup == 0) {
					return;
				}

				if (mergedRoots) {
					DetachMergedRootNodes(*mergedRoots);
				}
				world->ClearPrototypeGroupsLocked(*state, std::vector<std::uint64_t>{ buildGroup });
			}

			void Dismiss()
			{
				active = false;
			}
		};
		BuildGroupRollbackGuard rollbackGuard{
			.world = this,
			.state = std::addressof(a_state),
			.mergedRoots = std::addressof(mergedRootNodes),
			.buildGroup = buildGroup,
			.active = createdBuildGroup,
		};
		auto* skinRoot = actorRootNode ? static_cast<RE::NiAVObject*>(actorRootNode) : skeletonSearchRoot;
		std::vector<PrototypeBody> stagedBodies;

		phaseStart = Clock::now();
		for (auto& matchedBone : matchedBones) {
			const auto sameBuildBody = std::ranges::find_if(a_state.bodies, [&matchedBone, buildGroup](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, buildGroup) && a_body.node == matchedBone.node && a_body.bone;
			});
			if (sameBuildBody != a_state.bodies.end()) {
				continue;
			}
			const auto sameStagedBody = std::ranges::find_if(stagedBodies, [&matchedBone, buildGroup](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, buildGroup) && a_body.node == matchedBone.node && a_body.bone;
			});
			if (sameStagedBody != stagedBodies.end()) {
				continue;
			}

			if (a_domain != PrototypeBuildDomain::kArmor) {
				const auto existing = std::ranges::find_if(a_state.bodies, [&matchedBone](const PrototypeBody& a_body) {
					return a_body.node == matchedBone.node && a_body.bone;
				});
				if (existing != a_state.bodies.end()) {
					AddPrototypeBodyBuildGroup(*existing, buildGroup, a_domain, a_event.bipedObject);
					continue;
				}
			}

			const auto* descriptor = FindBoneDescriptor(a_summary, matchedBone.name);
			auto fallbackDescriptor = a_summary.defaultBoneDescriptor.value_or(PhysicsBoneDescriptor{});
			fallbackDescriptor.name = matchedBone.name;
			const auto& boneDescriptor = descriptor ? *descriptor : fallbackDescriptor;
			auto shape = CreateCollisionShape(boneDescriptor);
			btVector3 localInertia(0.0F, 0.0F, 0.0F);
			const auto mass = matchedBone.useActorKinematicBody ? 0.0F : std::max(boneDescriptor.mass, 0.0F);
			if (boneDescriptor.hasLocalInertia) {
				localInertia = btVector3(
					std::max(boneDescriptor.localInertia.x, 0.0F),
					std::max(boneDescriptor.localInertia.y, 0.0F),
					std::max(boneDescriptor.localInertia.z, 0.0F));
			} else if (mass > 0.0F) {
				shape->calculateLocalInertia(mass, localInertia);
			}

			const auto localToRig = ToBulletTransform(boneDescriptor.centerOfMassTransform);
			const auto& initialWorld = matchedBone.node->world;
			auto motionState = std::make_unique<btDefaultMotionState>(Smp::Fo4Transform::ToBulletTransform(initialWorld) * localToRig);
			btRigidBody::btRigidBodyConstructionInfo constructionInfo(mass, motionState.get(), shape.get(), localInertia);
			auto bone = std::make_unique<Fo4SkinnedMeshBone>(RE::BSFixedString(matchedBone.name), matchedBone.node, constructionInfo);
			bone->m_localToRig = localToRig;
			bone->m_rigToLocal = localToRig.inverse();
			bone->m_marginMultipler = std::max(boneDescriptor.marginMultiplier, 0.0F);
			bone->m_gravityFactor = std::clamp(boneDescriptor.gravityFactor, 0.0F, 1.0F);
			bone->m_windFactor = std::max(boneDescriptor.windFactor, 0.0F);
			for (const auto& boneName : boneDescriptor.canCollideWithBones) {
				bone->m_canCollideWithBone.emplace_back(boneName.c_str());
			}
			for (const auto& boneName : boneDescriptor.noCollideWithBones) {
				bone->m_noCollideWithBone.emplace_back(boneName.c_str());
			}
			bone->m_rig.setDamping(std::max(boneDescriptor.linearDamping, 0.0F), std::max(boneDescriptor.angularDamping, 0.0F));
			bone->m_rig.setFriction(std::max(boneDescriptor.friction, 0.0F));
			bone->m_rig.setRollingFriction(std::max(boneDescriptor.rollingFriction, 0.0F));
			bone->m_rig.setRestitution(std::max(boneDescriptor.restitution, 0.0F));
			bone->m_rig.setGravity(btVector3(0.0F, 0.0F, kGravityAcceleration * bone->m_gravityFactor));
			if (mass <= 0.0F) {
				++kinematicBodies;
			} else {
				++dynamicBodies;
			}
			bone->readTransform(0.0F);
			bone->m_rig.setActivationState(DISABLE_DEACTIVATION);
			spdlog::debug(
				"staged prototype body writeback target actor={} bone='{}' node={} nodeName='{}' sourceNode={} sourceName='{}' buildGroup={} mass={:.4f}",
				static_cast<void*>(a_event.actor),
				matchedBone.name,
				static_cast<void*>(matchedBone.node),
				matchedBone.node ? std::string_view(matchedBone.node->GetName()) : std::string_view{},
				static_cast<void*>(matchedBone.sourceNode),
				matchedBone.sourceNode ? std::string_view(matchedBone.sourceNode->GetName()) : std::string_view{},
				buildGroup,
				mass);

			PrototypeBody prototypeBody;
			prototypeBody.actor = a_event.actor;
			prototypeBody.node = matchedBone.node;
			prototypeBody.buildGroup = buildGroup;
			prototypeBody.bipedObject = a_event.bipedObject;
			AddPrototypeBodyBuildGroup(prototypeBody, buildGroup, a_domain, a_event.bipedObject);
			prototypeBody.boneName = std::move(matchedBone.name);
			prototypeBody.shape = std::move(shape);
			prototypeBody.motionState = std::move(motionState);
			prototypeBody.bone = std::move(bone);
			stagedBodies.push_back(std::move(prototypeBody));
			++created;
		}
		timing.bulletBodyMs += ElapsedMs(phaseStart, Clock::now());

		std::ranges::stable_sort(stagedBodies, [](const PrototypeBody& a_lhs, const PrototypeBody& a_rhs) {
			const auto lhsDepth = a_lhs.bone ? a_lhs.bone->GetDepth() : 0x7fffffff;
			const auto rhsDepth = a_rhs.bone ? a_rhs.bone->GetDepth() : 0x7fffffff;
			return lhsDepth < rhsDepth;
		});

		ResetPrototypeBuildGroupToCurrentPoseLocked(a_state, buildGroup, stagedBodies);
		std::vector<PrototypeMesh> stagedMeshes;
		phaseStart = Clock::now();
		const auto cpuCopyPending = BuildPrototypeMeshesLocked(a_state, a_summary, a_event, a_meshNameMap, buildGroup, a_domain, stagedBodies, stagedMeshes);
		timing.meshBuildMs += ElapsedMs(phaseStart, Clock::now());
		if (cpuCopyPending) {
			spdlog::debug(
				"prototype build group actor={} buildGroup={} is waiting for mesh CPU copy; staged Bullet rigid bodies were not committed",
				static_cast<void*>(a_state.actor),
				buildGroup);
			result.cpuCopyPending = true;
			RestoreSavedLocalPoses(savedBuildPoses, actorRootNode);
			logBuildTiming("cpu-copy-pending", buildGroup);
			return result;
		}
		if (a_domain == PrototypeBuildDomain::kArmor && stagedMeshes.empty() && !PrototypeBuildGroupHasMeshLocked(a_state, buildGroup)) {
			spdlog::debug(
				"prototype armor build group actor={} buildGroup={} produced no staged meshes; skin rebinding and Bullet commit skipped",
				static_cast<void*>(a_state.actor),
				buildGroup);
			RestoreSavedLocalPoses(savedBuildPoses, actorRootNode);
			logBuildTiming("no-staged-meshes", buildGroup);
			return result;
		}
		phaseStart = Clock::now();
		for (auto& stagedBody : stagedBodies) {
			a_state.bodies.push_back(std::move(stagedBody));
		}
		std::ranges::stable_sort(a_state.bodies, [](const PrototypeBody& a_lhs, const PrototypeBody& a_rhs) {
			const auto lhsDepth = a_lhs.bone ? a_lhs.bone->GetDepth() : 0x7fffffff;
			const auto rhsDepth = a_rhs.bone ? a_rhs.bone->GetDepth() : 0x7fffffff;
			return lhsDepth < rhsDepth;
		});
		for (auto& stagedMesh : stagedMeshes) {
			a_state.meshes.push_back(std::move(stagedMesh));
		}
		if (a_domain == PrototypeBuildDomain::kArmor && !PrototypeBuildGroupHasMeshLocked(a_state, buildGroup)) {
			spdlog::debug(
				"prototype armor build group actor={} buildGroup={} produced no committed meshes; skin rebinding and Bullet commit skipped",
				static_cast<void*>(a_state.actor),
				buildGroup);
			RestoreSavedLocalPoses(savedBuildPoses, actorRootNode);
			timing.bindCommitConstraintMs += ElapsedMs(phaseStart, Clock::now());
			logBuildTiming("no-committed-meshes", buildGroup);
			return result;
		}
		for (auto& mergedRoot : mergedRootNodes) {
			a_state.mergedNodes.push_back({
				.buildGroup = buildGroup,
				.parent = mergedRoot.parent,
				.node = mergedRoot.node,
				.sourceName = mergedRoot.originalName,
				.recordParentName = mergedRoot.recordParentName,
				.localToParent = mergedRoot.localToParent,
				.recordLocalToParent = mergedRoot.recordLocalToParent,
				.hasLocalToParent = mergedRoot.hasLocalToParent,
				.hasRecordLocalToParent = mergedRoot.hasRecordLocalToParent,
				.recordMergeParentBinding = mergedRoot.recordMergeParentBinding,
			});
		}
		mergedRootNodes.clear();
		RebindMatchedSkinSlots(matchedBones, mergedSkeletonNodes, a_event.object, skinRoot);
		for (auto& matchedBone : matchedBones) {
			auto body = std::ranges::find_if(a_state.bodies, [&matchedBone, buildGroup](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, buildGroup) && a_body.node == matchedBone.node && a_body.bone;
			});
			if (body == a_state.bodies.end() && !matchedBone.name.empty()) {
				body = std::ranges::find_if(a_state.bodies, [&matchedBone, buildGroup](const PrototypeBody& a_body) {
					return PrototypeBodyHasBuildGroup(a_body, buildGroup) && PhysicsNamesEqual(a_body.boneName, matchedBone.name) && a_body.bone;
				});
			}
			if (body == a_state.bodies.end()) {
				continue;
			}

			for (auto& skinWorld : matchedBone.skinWorldTransforms) {
				body->bone->AddSkinWorldTransform(
					skinWorld.skin.get(),
					skinWorld.index,
					buildGroup,
					skinWorld.originalBone.get(),
					skinWorld.originalWorldTransform,
					skinWorld.originalRootNode.get());
			}
		}
		RestoreSavedLocalPoses(savedBuildPoses, actorRootNode);
		ResetPrototypeBuildGroupToCurrentPoseLocked(a_state, buildGroup);
		std::vector<PrototypeConstraint> stagedConstraints;
		BuildPrototypeConstraintsLocked(a_state, a_summary, buildGroup, a_domain, {}, stagedConstraints);
		for (auto& stagedConstraint : stagedConstraints) {
			a_state.constraints.push_back(std::move(stagedConstraint));
		}
		CommitPrototypeBuildGroupToBulletLocked(a_state, buildGroup);
		if (a_domain == PrototypeBuildDomain::kArmor) {
			ResetPrototypeBuildGroupToReferencePoseLocked(a_state, buildGroup);
		}
		timing.bindCommitConstraintMs += ElapsedMs(phaseStart, Clock::now());
		if (actorRoot && spdlog::default_logger_raw() && spdlog::default_logger_raw()->should_log(spdlog::level::trace)) {
			LogObjectHierarchy(actorRoot, "actor-skeleton-after-prototype-build");
		}
		LogPrototypeActorBulletObjectsLocked(a_state, "after-prototype-build-commit");
		result.committed = true;
		result.recordable = PrototypeBuildGroupIsRecordableLocked(a_state, buildGroup, a_domain);
		result.succeeded = result.recordable;
		rollbackGuard.Dismiss();
		if (a_domain != PrototypeBuildDomain::kArmor || !hadActorRuntimeBeforeBuild) {
			a_state.resetReadFrames = std::max(a_state.resetReadFrames, kAttachResetReadFrames);
			ResetStepClockLocked();
		} else {
			spdlog::debug(
				"skipped actor-wide reset-read after hot armor build actor={} buildGroup={} because existing prototype runtime is active",
				static_cast<void*>(a_state.actor),
				buildGroup);
		}
		spdlog::debug(
			"prototype matched bone placement actorRoot={} attachedObject={} underActorRoot={} underAttachedObject={} matchedXMLBones={}",
			static_cast<void*>(actorRoot),
			static_cast<void*>(a_event.object),
			matchedUnderActorRoot,
			matchedUnderAttachedObject,
			matchedBones.size());
		logBuildTiming(result.succeeded ? "success" : "not-recordable", buildGroup);
		return result;
	}

	void Fo4PhysicsWorld::LogPrototypeActorBulletObjectsLocked(const PrototypeActorState& a_state, const std::string_view a_reason) const
	{
		spdlog::debug(
			"begin actor bullet physics object dump actor={} firstPerson={} bodies={} meshes={} constraints={} reason={}",
			static_cast<void*>(a_state.actor),
			a_state.firstPerson,
			a_state.bodies.size(),
			a_state.meshes.size(),
			a_state.constraints.size(),
			a_reason);

		for (const auto& body : a_state.bodies) {
			if (!body.bone) {
				continue;
			}

			const auto& rig = body.bone->m_rig;
			const auto transform = rig.getWorldTransform();
			const auto origin = transform.getOrigin();
			const auto rotation = transform.getRotation();
			const auto* node = body.node;
			spdlog::debug(
				"actor bullet rigid body actor={} body={} buildGroup={} bipedObject={} boneName='{}' rigidBody={} motionState={} invMass={:.6f} kinematic={} pos=({:.3f},{:.3f},{:.3f}) rot=({:.6f},{:.6f},{:.6f},{:.6f}) writeTargetNode={} writeTargetWorld={} nodeName='{}'",
				static_cast<void*>(a_state.actor),
				static_cast<const void*>(std::addressof(body)),
				body.buildGroup,
				std::to_underlying(body.bipedObject),
				body.boneName,
				static_cast<const void*>(std::addressof(rig)),
				static_cast<const void*>(rig.getMotionState()),
				rig.getInvMass(),
				rig.isStaticOrKinematicObject(),
				origin.x(),
				origin.y(),
				origin.z(),
				rotation.x(),
				rotation.y(),
				rotation.z(),
				rotation.w(),
				static_cast<const void*>(node),
				node ? static_cast<const void*>(std::addressof(node->world)) : nullptr,
				node ? std::string_view(node->GetName()) : std::string_view{});
		}

		for (const auto& mesh : a_state.meshes) {
			if (!mesh.body) {
				continue;
			}

			const auto transform = mesh.body->getWorldTransform();
			const auto origin = transform.getOrigin();
			const auto rotation = transform.getRotation();
			const auto* geometry = mesh.geometry;
			spdlog::debug(
				"actor bullet mesh body actor={} meshBody={} buildGroup={} bipedObject={} meshName='{}' geometry={} geometryName='{}' pos=({:.3f},{:.3f},{:.3f}) rot=({:.6f},{:.6f},{:.6f},{:.6f})",
				static_cast<void*>(a_state.actor),
				static_cast<const void*>(mesh.body.get()),
				mesh.buildGroup,
				std::to_underlying(mesh.bipedObject),
				mesh.name,
				static_cast<const void*>(geometry),
				geometry ? std::string_view(geometry->GetName()) : std::string_view{},
				origin.x(),
				origin.y(),
				origin.z(),
				rotation.x(),
				rotation.y(),
				rotation.z(),
				rotation.w());
		}

		for (const auto& constraint : a_state.constraints) {
			if (!constraint.constraint) {
				continue;
			}

			const auto& bodyA = constraint.constraint->getRigidBodyA();
			const auto& bodyB = constraint.constraint->getRigidBodyB();
			const auto transformA = bodyA.getWorldTransform();
			const auto transformB = bodyB.getWorldTransform();
			const auto originA = transformA.getOrigin();
			const auto originB = transformB.getOrigin();
			spdlog::debug(
				"actor bullet constraint actor={} constraint={} buildGroup={} kind={} bodyA='{}' bodyB='{}' enabled={} bodyAkin={} bodyBkin={} bodyApos=({:.3f},{:.3f},{:.3f}) bodyBpos=({:.3f},{:.3f},{:.3f})",
				static_cast<void*>(a_state.actor),
				static_cast<const void*>(constraint.constraint.get()),
				constraint.buildGroup,
				std::to_underlying(constraint.kind),
				constraint.bodyA,
				constraint.bodyB,
				constraint.constraint->isEnabled(),
				bodyA.isStaticOrKinematicObject(),
				bodyB.isStaticOrKinematicObject(),
				originA.x(),
				originA.y(),
				originA.z(),
				originB.x(),
				originB.y(),
				originB.z());
		}

		spdlog::debug(
			"end actor bullet physics object dump actor={} firstPerson={} reason={}",
			static_cast<void*>(a_state.actor),
			a_state.firstPerson,
			a_reason);
	}

	void Fo4PhysicsWorld::ResetPrototypeBuildGroupToCurrentPoseLocked(
		PrototypeActorState& a_state,
		const std::uint64_t a_buildGroup,
		const std::span<PrototypeBody> a_stagedBodies)
	{
		auto isInBuildGroup = [a_buildGroup](const PrototypeBody& a_body) {
			return PrototypeBodyHasBuildGroup(a_body, a_buildGroup);
		};

		for (auto& body : a_state.bodies) {
			if (!isInBuildGroup(body) || !body.bone || !body.node) {
				continue;
			}

			body.bone->readTransform(0.0F);
			body.bone->RefreshSkinWorldTransforms();
		}
		for (auto& body : a_stagedBodies) {
			if (!isInBuildGroup(body) || !body.bone || !body.node) {
				continue;
			}

			body.bone->readTransform(0.0F);
			body.bone->RefreshSkinWorldTransforms();
		}
	}

	std::uint32_t Fo4PhysicsWorld::ResetPrototypeBuildGroupToReferencePoseLocked(
		PrototypeActorState& a_state,
		const std::uint64_t a_buildGroup)
	{
		if (a_buildGroup == 0) {
			return 0;
		}

		std::uint32_t resetBodies = 0;
		std::uint32_t syncedNodes = 0;
		std::uint32_t skippedNonMergedBodies = 0;
		std::uint32_t skippedKinematicBodies = 0;
		std::vector<RE::NiNode*> dynamicResetNodes;
		std::vector<RE::NiNode*> updateRoots;
		const auto isMergedNodeInBuildGroup = [&a_state, a_buildGroup](const RE::NiNode* a_node) {
			return a_node &&
				std::ranges::any_of(a_state.mergedNodes, [a_node, a_buildGroup](const PrototypeMergedNode& a_mergedNode) {
					return a_mergedNode.buildGroup == a_buildGroup && a_mergedNode.node.get() == a_node;
				});
		};
		for (auto& body : a_state.bodies) {
			if (!PrototypeBodyHasBuildGroup(body, a_buildGroup) || !body.bone) {
				continue;
			}
			if (!isMergedNodeInBuildGroup(body.node)) {
				++skippedNonMergedBodies;
				continue;
			}
			if (body.bone->m_rig.getInvMass() <= 0.0F) {
				++skippedKinematicBodies;
				continue;
			}
			if (body.node && std::ranges::find(dynamicResetNodes, body.node) == dynamicResetNodes.end()) {
				dynamicResetNodes.push_back(body.node);
			}
		}
		const auto shouldResetMergedNode = [&dynamicResetNodes](const RE::NiNode* a_node) {
			return a_node && std::ranges::find(dynamicResetNodes, a_node) != dynamicResetNodes.end();
		};
		for (auto& mergedNode : a_state.mergedNodes) {
			auto* node = mergedNode.node ? mergedNode.node->IsNode() : nullptr;
			if (mergedNode.buildGroup != a_buildGroup || !node || !shouldResetMergedNode(node)) {
				continue;
			}

			if (mergedNode.hasLocalToParent) {
				mergedNode.node->local = mergedNode.localToParent;
			}

			if (!shouldResetMergedNode(node->parent) &&
				std::ranges::find(updateRoots, node) == updateRoots.end()) {
				updateRoots.push_back(node);
			}
			++syncedNodes;
		}
		for (auto* updateRoot : updateRoots) {
			UpdateNodeWorldFromLocal(updateRoot);
		}

		for (auto& body : a_state.bodies) {
			if (!PrototypeBodyHasBuildGroup(body, a_buildGroup) || !body.bone) {
				continue;
			}
			if (!isMergedNodeInBuildGroup(body.node)) {
				continue;
			}
			if (body.bone->m_rig.getInvMass() <= 0.0F) {
				continue;
			}

			body.bone->readTransform(0.0F);
			body.bone->RefreshSkinWorldTransforms();
			++resetBodies;
		}

		if (resetBodies > 0 || syncedNodes > 0 || skippedNonMergedBodies > 0 || skippedKinematicBodies > 0) {
			spdlog::debug(
				"reset prototype build group to reference pose actor={} buildGroup={} dynamicBodies={} syncedPluginNodes={} skippedNonMergedBodies={} skippedKinematicBodies={}",
				static_cast<void*>(a_state.actor),
				a_buildGroup,
				resetBodies,
				syncedNodes,
				skippedNonMergedBodies,
				skippedKinematicBodies);
		}
		return resetBodies;
	}

	std::uint32_t Fo4PhysicsWorld::ResetPrototypeRuntimeToReferencePoseLocked(
		PrototypeActorState& a_state,
		const std::string_view a_reason)
	{
		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.runtimes) {
			if (runtime.buildGroup != 0 && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}
		for (const auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.buildGroup != 0 && std::ranges::find(buildGroups, prototypeMesh.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeMesh.buildGroup);
			}
		}
		for (const auto& prototypeConstraint : a_state.constraints) {
			if (prototypeConstraint.buildGroup != 0 && std::ranges::find(buildGroups, prototypeConstraint.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeConstraint.buildGroup);
			}
		}
		for (const auto& prototypeBody : a_state.bodies) {
			if (!prototypeBody.buildGroups.empty()) {
				for (const auto buildGroup : prototypeBody.buildGroups) {
					if (buildGroup != 0 && std::ranges::find(buildGroups, buildGroup) == buildGroups.end()) {
						buildGroups.push_back(buildGroup);
					}
				}
			} else if (prototypeBody.buildGroup != 0 && std::ranges::find(buildGroups, prototypeBody.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeBody.buildGroup);
			}
		}

		std::uint32_t resetBodies = 0;
		for (const auto buildGroup : buildGroups) {
			resetBodies += ResetPrototypeBuildGroupToReferencePoseLocked(a_state, buildGroup);
		}
		if (resetBodies > 0) {
			spdlog::debug(
				"reset prototype runtime to reference pose actor={} reason={} buildGroups={} resetBodies={}",
				static_cast<void*>(a_state.actor),
				a_reason,
				buildGroups.size(),
				resetBodies);
		}
		return resetBodies;
	}

	bool Fo4PhysicsWorld::BuildPrototypeMeshesLocked(
		PrototypeActorState& a_state,
		const PhysicsXmlSummary& a_summary,
		const LifecycleEvent& a_event,
		const DefaultBBP::NameMap& a_meshNameMap,
		const std::uint64_t a_buildGroup,
		const PrototypeBuildDomain a_domain,
		const std::span<PrototypeBody> a_stagedBodies,
		std::vector<PrototypeMesh>& a_stagedMeshes)
	{
		if (!dynamicsWorld_ || a_summary.meshDescriptors.empty()) {
			return false;
		}

		auto meshNames = BuildMeshMatchNames(a_summary, a_meshNameMap);

		auto extraction = ExtractSkinnedMeshes(a_event.object, meshNames);
		auto cpuCopyPending = HasPendingCpuCopyExtraction(extraction);
		auto pendingMatchedGeometries = cpuCopyPending ? extraction.stats.matchedGeometries : 0U;
		auto pendingVertexCopies = cpuCopyPending ? extraction.stats.pendingVertexCopies : 0U;
		auto pendingIndexCopies = cpuCopyPending ? extraction.stats.pendingIndexCopies : 0U;
		const char* extractionSource = "attached-object";
		if (extraction.meshes.empty()) {
			const std::array fallbackRoots{
				std::pair{ "merge-source", a_event.mergeSourceObject },
				std::pair{ "source-object", a_event.sourceObject },
				std::pair{ "source-root", static_cast<RE::NiAVObject*>(a_event.sourceRoot) },
			};
			for (const auto& [sourceName, root] : fallbackRoots) {
				if (!root || root == a_event.object) {
					continue;
				}

				auto fallbackExtraction = ExtractSkinnedMeshes(root, meshNames);
				const auto fallbackCpuCopyPending = HasPendingCpuCopyExtraction(fallbackExtraction);
				if (fallbackCpuCopyPending) {
					cpuCopyPending = true;
					pendingMatchedGeometries += fallbackExtraction.stats.matchedGeometries;
					pendingVertexCopies += fallbackExtraction.stats.pendingVertexCopies;
					pendingIndexCopies += fallbackExtraction.stats.pendingIndexCopies;
				}
				if (fallbackExtraction.meshes.empty()) {
					spdlog::debug(
						"prototype mesh extraction fallback {} root={} produced no meshes for actor={} geometries={} skinned={} matched={} missingCpuVertexData={} invalidCpuVertexData={} pendingVertexCopies={} pendingIndexCopies={}",
						sourceName,
						static_cast<void*>(root),
						static_cast<void*>(a_state.actor),
						fallbackExtraction.stats.geometries,
						fallbackExtraction.stats.skinnedGeometries,
						fallbackExtraction.stats.matchedGeometries,
						fallbackExtraction.stats.missingCpuVertexData,
						fallbackExtraction.stats.invalidCpuVertexData,
						fallbackExtraction.stats.pendingVertexCopies,
						fallbackExtraction.stats.pendingIndexCopies);
					continue;
				}

				extraction = std::move(fallbackExtraction);
				extractionSource = sourceName;
				spdlog::debug(
					"prototype mesh extraction using {} root={} for actor={} decodedMeshes={}",
					sourceName,
					static_cast<void*>(root),
					static_cast<void*>(a_state.actor),
					extraction.stats.decodedMeshes);
				break;
			}
		}
		if (extraction.meshes.empty() && cpuCopyPending) {
			spdlog::debug(
				"prototype mesh extraction delayed for pending CPU copy actor={} object={} matched={} pendingVertexCopies={} pendingIndexCopies={}",
				static_cast<void*>(a_state.actor),
				static_cast<void*>(a_event.object),
				pendingMatchedGeometries,
				pendingVertexCopies,
				pendingIndexCopies);
			return true;
		}
		if (a_domain == PrototypeBuildDomain::kArmor && cpuCopyPending) {
			spdlog::debug(
				"prototype armor mesh extraction delayed for partial pending CPU copy actor={} object={} decodedMeshes={} matched={} pendingVertexCopies={} pendingIndexCopies={}",
				static_cast<void*>(a_state.actor),
				static_cast<void*>(a_event.object),
				extraction.stats.decodedMeshes,
				pendingMatchedGeometries,
				pendingVertexCopies,
				pendingIndexCopies);
			return true;
		}
		std::uint32_t created = 0;
		std::uint32_t skippedMissingBones = 0;
		std::uint32_t skippedMissingBoneData = 0;
		std::uint32_t sanitizedBadBoneMeshes = 0;
		std::uint32_t skippedMissingTriangleIndices = 0;
		std::uint32_t skippedInvalidTriangleIndices = 0;
		std::uint32_t skippedEmpty = 0;
		std::uint32_t skippedNoColliders = 0;
		std::uint32_t skippedExisting = 0;
		std::uint32_t unresolvedCanCollideBones = 0;
		std::uint32_t unresolvedNoCollideBones = 0;
		std::uint32_t unresolvedWeightThresholds = 0;
		for (const auto& decodedMesh : extraction.meshes) {
			if (decodedMesh.vertices.empty()) {
				++skippedEmpty;
				continue;
			}

			const auto existingMesh = std::ranges::find_if(a_state.meshes, [&decodedMesh, a_buildGroup](const PrototypeMesh& a_mesh) {
				return a_mesh.geometry == decodedMesh.geometry && a_mesh.buildGroup == a_buildGroup;
			});
			if (existingMesh != a_state.meshes.end()) {
				++skippedExisting;
				continue;
			}
			const auto existingStagedMesh = std::ranges::find_if(a_stagedMeshes, [&decodedMesh, a_buildGroup](const PrototypeMesh& a_mesh) {
				return a_mesh.geometry == decodedMesh.geometry && a_mesh.buildGroup == a_buildGroup;
			});
			if (existingStagedMesh != a_stagedMeshes.end()) {
				++skippedExisting;
				continue;
			}

			const auto* meshDescriptor = FindMeshDescriptor(a_summary, decodedMesh.name, a_meshNameMap);
			if (decodedMesh.badBoneIndices > 0) {
				++sanitizedBadBoneMeshes;
				spdlog::debug("mesh '{}' discarded {} unusable vertex bone influences during decode", decodedMesh.name, decodedMesh.badBoneIndices);
			}
			const auto weightedBoneWithoutBindData = [&decodedMesh]() {
				std::size_t vertexIndex = 0;
				for (const auto& vertex : decodedMesh.vertices) {
					for (int influence = 0; influence < 4; ++influence) {
						const auto boneIndex = static_cast<std::size_t>(vertex.getBoneIdx(influence));
						if (vertex.weight_[influence] > FLT_EPSILON &&
							boneIndex < decodedMesh.bones.size() &&
							!decodedMesh.bones[boneIndex].hasSkinToBone) {
							const auto& decodedBone = decodedMesh.bones[boneIndex];
							spdlog::warn(
								"mesh '{}' bind-pose miss vertex={} influence={} weight={} boneIndex={} boneCount={} boneNode={} boneName='{}' hasBoneData={}",
								decodedMesh.name,
								vertexIndex,
								influence,
								vertex.weight_[influence],
								boneIndex,
								decodedMesh.bones.size(),
								static_cast<void*>(decodedBone.node),
								decodedBone.name,
								decodedBone.hasBoneData);
							return true;
						}
					}
					++vertexIndex;
				}
				return false;
			}();
			if (weightedBoneWithoutBindData) {
				++skippedMissingBoneData;
				spdlog::warn("skipping mesh '{}' because a weighted skin bone is missing bind-pose data", decodedMesh.name);
				continue;
			}
			if (meshDescriptor && meshDescriptor->kind == PhysicsMeshShapeKind::kPerTriangle && decodedMesh.indices.size() < 3) {
				++skippedMissingTriangleIndices;
				spdlog::warn("skipping per-triangle mesh '{}' because no usable CPU index buffer was decoded", decodedMesh.name);
				continue;
			}

			auto meshBody = RE::make_smart<hdt::SkinnedMeshBody>();
			meshBody->name_ = RE::BSFixedString(decodedMesh.name);
			meshBody->actor_ = a_state.actor;
			meshBody->buildGroup_ = a_buildGroup;
			meshBody->vertices_ = decodedMesh.vertices;

			for (std::size_t boneIndex = 0; boneIndex < decodedMesh.bones.size(); ++boneIndex) {
				const auto& decodedBone = decodedMesh.bones[boneIndex];
				PrototypeBody* matchedBody = nullptr;
				auto committedBody = std::ranges::find_if(a_state.bodies, [&decodedBone, a_buildGroup](const PrototypeBody& a_body) {
					return decodedBone.node && PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && a_body.node == decodedBone.node;
				});
				if (committedBody == a_state.bodies.end() && !decodedBone.name.empty()) {
					committedBody = std::ranges::find_if(a_state.bodies, [&decodedBone, a_buildGroup](const PrototypeBody& a_body) {
						return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, decodedBone.name);
					});
				}
				if (committedBody != a_state.bodies.end()) {
					matchedBody = std::addressof(*committedBody);
				}
				if (!matchedBody) {
					auto stagedBody = std::ranges::find_if(a_stagedBodies, [&decodedBone, a_buildGroup](const PrototypeBody& a_body) {
						return decodedBone.node && PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && a_body.node == decodedBone.node;
					});
					if (stagedBody == a_stagedBodies.end() && !decodedBone.name.empty()) {
						stagedBody = std::ranges::find_if(a_stagedBodies, [&decodedBone, a_buildGroup](const PrototypeBody& a_body) {
							return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, decodedBone.name);
						});
					}
					if (stagedBody != a_stagedBodies.end()) {
						matchedBody = std::addressof(*stagedBody);
					}
				}

				const auto weightedBone = std::ranges::any_of(decodedMesh.vertices, [boneIndex](const hdt::Vertex& a_vertex) {
					for (int influence = 0; influence < 4; ++influence) {
						if (a_vertex.weight_[influence] > FLT_EPSILON && a_vertex.getBoneIdx(influence) == boneIndex) {
							return true;
						}
					}
					return false;
				});

				if (!matchedBody || !matchedBody->bone) {
					if (weightedBone) {
						++skippedMissingBones;
						spdlog::debug(
							"mesh '{}' did not create fallback Bullet body for weighted skin bone '{}' node={} because it was not resolved through the current merged/actor-safe build group",
							decodedMesh.name,
							decodedBone.name,
							static_cast<void*>(decodedBone.node));
					}
					meshBody->addBone(
						nullptr,
						decodedBone.hasSkinToBone ? decodedBone.skinToBone : hdt::btQsTransform::getIdentity(),
						decodedBone.hasBoneData ? decodedBone.boundingSphere : hdt::BoundingSphere(btVector3(0.0F, 0.0F, 0.0F), 0.0F));
					continue;
				}

				const auto sphere = decodedBone.hasBoneData ?
					decodedBone.boundingSphere :
					CalculateBoneSphere(decodedMesh, boneIndex).value_or(hdt::BoundingSphere(btVector3(0.0F, 0.0F, 0.0F), 0.0F));
				meshBody->addBone(matchedBody->bone.get(), decodedBone.hasSkinToBone ? decodedBone.skinToBone : hdt::btQsTransform::getIdentity(), sphere);
			}

			if (meshDescriptor) {
				switch (meshDescriptor->shared) {
				case PhysicsMeshSharedScope::kInternal:
					meshBody->shared_ = hdt::SkinnedMeshBody::SharedType::kInternal;
					break;
				case PhysicsMeshSharedScope::kExternal:
					meshBody->shared_ = hdt::SkinnedMeshBody::SharedType::kExternal;
					break;
				case PhysicsMeshSharedScope::kPrivate:
					meshBody->shared_ = hdt::SkinnedMeshBody::SharedType::kPrivate;
					break;
				case PhysicsMeshSharedScope::kPublic:
				default:
					meshBody->shared_ = hdt::SkinnedMeshBody::SharedType::kPublic;
					break;
				}
				for (const auto& tag : meshDescriptor->tags) {
					meshBody->tags_.emplace_back(tag.c_str());
				}
				for (const auto& tag : meshDescriptor->canCollideWithTags) {
					meshBody->canCollideWithTags_.emplace_back(tag.c_str());
				}
				for (const auto& tag : meshDescriptor->noCollideWithTags) {
					meshBody->noCollideWithTags_.emplace_back(tag.c_str());
				}
				meshBody->disableTag_ = meshDescriptor->disableTag;
				meshBody->disablePriority_ = meshDescriptor->disablePriority;
				for (const auto& boneName : meshDescriptor->canCollideWithBones) {
					auto matchedBody = std::ranges::find_if(a_state.bodies, [&boneName, a_buildGroup](const PrototypeBody& a_body) {
						return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, boneName);
					});
					PrototypeBody* resolvedBody = matchedBody != a_state.bodies.end() ? std::addressof(*matchedBody) : nullptr;
					if (!resolvedBody) {
						const auto stagedBody = std::ranges::find_if(a_stagedBodies, [&boneName, a_buildGroup](const PrototypeBody& a_body) {
							return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, boneName);
						});
						if (stagedBody != a_stagedBodies.end()) {
							resolvedBody = std::addressof(*stagedBody);
						}
					}
					if (resolvedBody && resolvedBody->bone) {
						meshBody->canCollideWithBones_.push_back(resolvedBody->bone.get());
					} else {
						++unresolvedCanCollideBones;
						spdlog::debug("mesh '{}' could not resolve can-collide-with-bone '{}'", decodedMesh.name, boneName);
					}
				}
				for (const auto& boneName : meshDescriptor->noCollideWithBones) {
					auto matchedBody = std::ranges::find_if(a_state.bodies, [&boneName, a_buildGroup](const PrototypeBody& a_body) {
						return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, boneName);
					});
					PrototypeBody* resolvedBody = matchedBody != a_state.bodies.end() ? std::addressof(*matchedBody) : nullptr;
					if (!resolvedBody) {
						const auto stagedBody = std::ranges::find_if(a_stagedBodies, [&boneName, a_buildGroup](const PrototypeBody& a_body) {
							return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, boneName);
						});
						if (stagedBody != a_stagedBodies.end()) {
							resolvedBody = std::addressof(*stagedBody);
						}
					}
					if (resolvedBody && resolvedBody->bone) {
						meshBody->noCollideWithBones_.push_back(resolvedBody->bone.get());
					} else {
						++unresolvedNoCollideBones;
						spdlog::debug("mesh '{}' could not resolve no-collide-with-bone '{}'", decodedMesh.name, boneName);
					}
				}
				for (const auto& [boneName, threshold] : meshDescriptor->weightThresholds) {
					bool appliedThreshold = false;
					for (std::size_t boneIndex = 0; boneIndex < decodedMesh.bones.size() && boneIndex < meshBody->skinnedBones_.size(); ++boneIndex) {
						if (PhysicsNamesEqual(decodedMesh.bones[boneIndex].name, boneName)) {
							meshBody->skinnedBones_[boneIndex].weightThreshold = threshold;
							appliedThreshold = true;
							break;
						}
					}
					if (!appliedThreshold) {
						++unresolvedWeightThresholds;
						spdlog::debug("mesh '{}' could not resolve weight-threshold bone '{}'", decodedMesh.name, boneName);
					}
				}
			}

			if (meshDescriptor && meshDescriptor->kind == PhysicsMeshShapeKind::kPerTriangle && decodedMesh.indices.size() >= 3) {
				auto* shape = new hdt::PerTriangleShape(meshBody.get());
				if (meshDescriptor->hasMargin) {
					shape->shapeProp_.margin = std::max(meshDescriptor->margin, 0.0F);
				}
				if (meshDescriptor->hasPenetration) {
					shape->shapeProp_.penetration = std::max(meshDescriptor->penetration, 0.0F);
				}
				for (std::size_t index = 0; index + 2 < decodedMesh.indices.size(); index += 3) {
					if (decodedMesh.indices[index] >= decodedMesh.vertices.size() ||
						decodedMesh.indices[index + 1] >= decodedMesh.vertices.size() ||
						decodedMesh.indices[index + 2] >= decodedMesh.vertices.size()) {
						++skippedInvalidTriangleIndices;
						continue;
					}
					shape->addTriangle(
						static_cast<int>(decodedMesh.indices[index]),
						static_cast<int>(decodedMesh.indices[index + 1]),
						static_cast<int>(decodedMesh.indices[index + 2]));
				}
			} else {
				auto* shape = new hdt::PerVertexShape(meshBody.get());
				if (meshDescriptor && meshDescriptor->hasMargin) {
					shape->shapeProp_.margin = std::max(meshDescriptor->margin, 0.0F);
				}
				shape->autoGen();
			}

			meshBody->finishBuild();
			if (!meshBody->shape_ || meshBody->shape_->colliders_.empty() || meshBody->vertices_.empty()) {
				++skippedNoColliders;
				continue;
			}

			meshBody->internalUpdate();

			PrototypeMesh prototypeMesh;
			prototypeMesh.name = decodedMesh.name;
			prototypeMesh.geometry = decodedMesh.geometry;
			prototypeMesh.buildGroup = a_buildGroup;
			prototypeMesh.bipedObject = a_event.bipedObject;
			prototypeMesh.domain = a_domain;
			prototypeMesh.body = std::move(meshBody);
			a_stagedMeshes.push_back(std::move(prototypeMesh));
			++created;
		}

		if (a_domain != PrototypeBuildDomain::kArmor) {
			spdlog::info(
				"created {} {} prototype skinned mesh bodies for actor={} extractionSource={} from decodedMeshes={} geometries={} skinnedGeometries={} matchedGeometries={} decodedVertices={} decodedTriangles={} skippedExisting={} skippedEmpty={} skippedMissingBones={} skippedMissingBoneData={} sanitizedBadBoneMeshes={} skippedMissingTriangleIndices={} skippedInvalidTriangleIndices={} skippedNoColliders={} unresolvedCanCollideBones={} unresolvedNoCollideBones={} unresolvedWeightThresholds={} nullBones={} nonNodeBones={} missingBoneData={} unsupportedGeometryClasses={} missingRendererData={} missingVertexBuffer={} missingIndexBuffer={} missingCpuVertexData={} invalidCpuVertexData={} pendingVertexCopies={} missingCpuIndexData={} invalidCpuIndexData={} pendingIndexCopies={} undersizedVertexBuffers={} undersizedIndexBuffers={} badBoneIndices={}",
				created,
				PrototypeDomainName(a_domain),
				static_cast<void*>(a_state.actor),
				extractionSource,
				extraction.stats.decodedMeshes,
				extraction.stats.geometries,
				extraction.stats.skinnedGeometries,
				extraction.stats.matchedGeometries,
				extraction.stats.decodedVertices,
				extraction.stats.decodedTriangles,
				skippedExisting,
				skippedEmpty,
				skippedMissingBones,
				skippedMissingBoneData,
				sanitizedBadBoneMeshes,
				skippedMissingTriangleIndices,
				skippedInvalidTriangleIndices,
				skippedNoColliders,
				unresolvedCanCollideBones,
				unresolvedNoCollideBones,
				unresolvedWeightThresholds,
				extraction.stats.nullBones,
				extraction.stats.nonNodeBones,
				extraction.stats.missingBoneData,
				extraction.stats.unsupportedGeometryClasses,
				extraction.stats.missingRendererData,
				extraction.stats.missingVertexBuffer,
				extraction.stats.missingIndexBuffer,
				extraction.stats.missingCpuVertexData,
				extraction.stats.invalidCpuVertexData,
				extraction.stats.pendingVertexCopies,
				extraction.stats.missingCpuIndexData,
				extraction.stats.invalidCpuIndexData,
				extraction.stats.pendingIndexCopies,
				extraction.stats.undersizedVertexBuffers,
				extraction.stats.undersizedIndexBuffers,
				extraction.stats.badBoneIndices);
		}
		return false;
	}

	void Fo4PhysicsWorld::BuildPrototypeConstraintsLocked(
		PrototypeActorState& a_state,
		const PhysicsXmlSummary& a_summary,
		const std::uint64_t a_buildGroup,
		const PrototypeBuildDomain a_domain,
		const std::span<PrototypeBody> a_stagedBodies,
		std::vector<PrototypeConstraint>& a_stagedConstraints)
	{
		std::uint32_t created = 0;
		std::uint32_t skippedMissingBodies = 0;
		std::uint32_t skippedExisting = 0;
		std::uint32_t skippedSelfConstraints = 0;
		std::uint32_t kinematicPairsAllowed = 0;
		std::uint32_t skippedInvalid = 0;
		const auto findBodyForConstraint = [&](const std::string_view a_name) {
			auto body = std::ranges::find_if(a_state.bodies, [a_buildGroup, a_name](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, a_name);
			});
			if (body != a_state.bodies.end()) {
				return std::addressof(*body);
			}
			const auto stagedBody = std::ranges::find_if(a_stagedBodies, [a_buildGroup, a_name](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, a_name);
			});
			return stagedBody != a_stagedBodies.end() ? std::addressof(*stagedBody) : nullptr;
		};
		for (const auto& descriptor : a_summary.constraintDescriptors) {
			const auto bodyA = findBodyForConstraint(descriptor.bodyA);
			const auto bodyB = findBodyForConstraint(descriptor.bodyB);
			if (!bodyA || !bodyB || !bodyA->bone || !bodyB->bone) {
				++skippedMissingBodies;
				spdlog::debug("skipping constraint '{}' because bodies '{}'/'{}' were not both resolved", descriptor.name, descriptor.bodyA, descriptor.bodyB);
				continue;
			}
			if (bodyA == bodyB || bodyA->bone.get() == bodyB->bone.get()) {
				++skippedSelfConstraints;
				spdlog::warn("skipping constraint '{}' between same body '{}'", descriptor.name, descriptor.bodyA);
				continue;
			}
			if (bodyA->bone->m_rig.isStaticOrKinematicObject() && bodyB->bone->m_rig.isStaticOrKinematicObject()) {
				++kinematicPairsAllowed;
				spdlog::debug("allowing FO4 kinematic-to-kinematic constraint '{}' between '{}'/'{}'", descriptor.name, descriptor.bodyA, descriptor.bodyB);
			}
			const auto existing = std::ranges::find_if(a_state.constraints, [&descriptor, a_buildGroup](const PrototypeConstraint& a_constraint) {
					return a_constraint.buildGroup == a_buildGroup && PhysicsNamesEqual(a_constraint.bodyA, descriptor.bodyA) && PhysicsNamesEqual(a_constraint.bodyB, descriptor.bodyB);
			});
			if (existing != a_state.constraints.end()) {
				++skippedExisting;
				continue;
			}
			const auto existingStaged = std::ranges::find_if(a_stagedConstraints, [&descriptor, a_buildGroup](const PrototypeConstraint& a_constraint) {
					return a_constraint.buildGroup == a_buildGroup && PhysicsNamesEqual(a_constraint.bodyA, descriptor.bodyA) && PhysicsNamesEqual(a_constraint.bodyB, descriptor.bodyB);
			});
			if (existingStaged != a_stagedConstraints.end()) {
				++skippedExisting;
				continue;
			}

			auto constraint = CreatePrototypeConstraint(
				descriptor,
				bodyA->bone->m_rig,
				bodyB->bone->m_rig,
				bodyA->bone->m_rigToLocal,
				bodyB->bone->m_rigToLocal,
				bodyA->bone->m_currentTransform,
				bodyB->bone->m_currentTransform);
			if (!constraint) {
				++skippedInvalid;
				continue;
			}

			PrototypeConstraint prototypeConstraint;
			prototypeConstraint.buildGroup = a_buildGroup;
			prototypeConstraint.domain = a_domain;
			prototypeConstraint.bodyA = descriptor.bodyA;
			prototypeConstraint.bodyB = descriptor.bodyB;
			prototypeConstraint.kind = descriptor.kind;
			const auto reversedGeneric = descriptor.kind == PhysicsConstraintKind::kGeneric && descriptor.useLinearReferenceFrameA;
			prototypeConstraint.boneA = reversedGeneric ? bodyB->bone.get() : bodyA->bone.get();
			prototypeConstraint.boneB = reversedGeneric ? bodyA->bone.get() : bodyB->bone.get();
			prototypeConstraint.constraint = std::move(constraint);
			a_stagedConstraints.push_back(std::move(prototypeConstraint));
			++created;
		}

		if (created > 0 || skippedMissingBodies > 0 || skippedExisting > 0 || skippedSelfConstraints > 0 || kinematicPairsAllowed > 0 || skippedInvalid > 0) {
			spdlog::info(
				"created {} {} prototype Bullet constraints for actor={} buildGroup={}; actor constraints={} skippedMissingBodies={} skippedExisting={} skippedSelfConstraints={} kinematicPairsAllowed={} skippedInvalid={}",
				created,
				PrototypeDomainName(a_domain),
				static_cast<void*>(a_state.actor),
				a_buildGroup,
				a_state.constraints.size(),
				skippedMissingBodies,
				skippedExisting,
				skippedSelfConstraints,
				kinematicPairsAllowed,
				skippedInvalid);
		}
	}

	void Fo4PhysicsWorld::ScalePrototypeConstraintsLocked(PrototypeActorState& a_state)
	{
		if (!a_state.runtimes.empty()) {
			for (const auto& runtime : a_state.runtimes) {
				ScalePrototypeConstraintsLocked(a_state, runtime);
			}
			return;
		}

		for (auto& prototypeConstraint : a_state.constraints) {
			if (!prototypeConstraint.constraint || !prototypeConstraint.boneA || !prototypeConstraint.boneB) {
				continue;
			}

			const auto newScaleA = CurrentBoneScale(prototypeConstraint.boneA);
			const auto newScaleB = CurrentBoneScale(prototypeConstraint.boneB);
			if (btFuzzyZero(newScaleA - prototypeConstraint.scaleA) && btFuzzyZero(newScaleB - prototypeConstraint.scaleB)) {
				continue;
			}

			switch (prototypeConstraint.kind) {
			case PhysicsConstraintKind::kConeTwist:
				ScaleConeTwistConstraint(
					*static_cast<btConeTwistConstraint*>(prototypeConstraint.constraint.get()),
					prototypeConstraint.scaleA,
					prototypeConstraint.scaleB,
					newScaleA,
					newScaleB);
				break;
			case PhysicsConstraintKind::kStiffSpring:
				static_cast<PrototypeStiffSpringConstraint*>(prototypeConstraint.constraint.get())
					->ScaleConstraint(prototypeConstraint.scaleA, prototypeConstraint.scaleB, newScaleA, newScaleB);
				break;
			case PhysicsConstraintKind::kGeneric:
			default:
				ScaleGenericConstraint(
					*static_cast<btGeneric6DofSpring2Constraint*>(prototypeConstraint.constraint.get()),
					prototypeConstraint.scaleA,
					prototypeConstraint.scaleB,
					newScaleA,
					newScaleB);
				break;
			}

			prototypeConstraint.scaleA = newScaleA;
			prototypeConstraint.scaleB = newScaleB;
		}
	}

	void Fo4PhysicsWorld::ScalePrototypeConstraintsLocked(PrototypeActorState& a_state, const PrototypeBuildGroupRuntime& a_runtime)
	{
		for (auto* constraint : a_runtime.constraints) {
			if (!constraint) {
				continue;
			}
			auto prototypeConstraint = std::ranges::find_if(a_state.constraints, [constraint](const PrototypeConstraint& a_constraint) {
				return a_constraint.constraint.get() == constraint;
			});
			if (prototypeConstraint == a_state.constraints.end() ||
				!prototypeConstraint->constraint ||
				!prototypeConstraint->boneA ||
				!prototypeConstraint->boneB) {
				continue;
			}

			const auto newScaleA = CurrentBoneScale(prototypeConstraint->boneA);
			const auto newScaleB = CurrentBoneScale(prototypeConstraint->boneB);
			if (btFuzzyZero(newScaleA - prototypeConstraint->scaleA) && btFuzzyZero(newScaleB - prototypeConstraint->scaleB)) {
				continue;
			}

			switch (prototypeConstraint->kind) {
			case PhysicsConstraintKind::kConeTwist:
				ScaleConeTwistConstraint(
					*static_cast<btConeTwistConstraint*>(prototypeConstraint->constraint.get()),
					prototypeConstraint->scaleA,
					prototypeConstraint->scaleB,
					newScaleA,
					newScaleB);
				break;
			case PhysicsConstraintKind::kStiffSpring:
				static_cast<PrototypeStiffSpringConstraint*>(prototypeConstraint->constraint.get())
					->ScaleConstraint(prototypeConstraint->scaleA, prototypeConstraint->scaleB, newScaleA, newScaleB);
				break;
			case PhysicsConstraintKind::kGeneric:
			default:
				ScaleGenericConstraint(
					*static_cast<btGeneric6DofSpring2Constraint*>(prototypeConstraint->constraint.get()),
					prototypeConstraint->scaleA,
					prototypeConstraint->scaleB,
					newScaleA,
					newScaleB);
				break;
			}

			prototypeConstraint->scaleA = newScaleA;
			prototypeConstraint->scaleB = newScaleB;
		}
	}
}
