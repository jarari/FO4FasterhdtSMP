#include "Fo4PhysicsWorld.h"

#include "Address.h"
#include "ArmorBoneReference.h"
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
#include "PapyrusFunctions.h"
#include "PhysicsProfiler.h"
#include "SmpConfig.h"
#include "hdtSkinnedMesh/hdtDispatcher.h"
#include "hdtSkinnedMesh/hdtConeTwistConstraint.h"
#include "hdtSkinnedMesh/hdtGeneric6DofConstraint.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"
#include "hdtSkinnedMesh/hdtStiffSpringConstraint.h"
#include "RE/B/BipedAnim.h"
#include "RE/B/BGSHeadPart.h"
#include "RE/B/bhkPickData.h"
#include "RE/B/BSFlattenedBoneTree.h"
#include "RE/B/BSModelDB.h"
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
#include "RE/N/NiStringExtraData.h"
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

#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>
#include <LinearMath/btThreads.h>
#include <btBulletDynamicsCommon.h>

#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <tbb/parallel_for.h>
#include <tbb/task_group.h>
#include <unordered_map>
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
	constexpr std::uint64_t kPapyrusArmorPoseCacheBuildGroup = std::numeric_limits<std::uint64_t>::max();
	constexpr std::uint64_t kPapyrusHeadPoseCacheBuildGroup = kPapyrusArmorPoseCacheBuildGroup - 1;
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

	int InitializeBulletTaskSchedulerAndGetThreadCount()
	{
		auto* scheduler = btGetTBBTaskScheduler();
		if (!scheduler) {
			scheduler = btGetSequentialTaskScheduler();
		}
		btSetTaskScheduler(scheduler);

		const auto concurrency = std::max(1, scheduler->getMaxNumThreads());
		spdlog::info("FO4 Faster HDT-SMP constraint solving is using {} threads", concurrency);
		return concurrency;
	}

	class OwnedCompoundShape :
		public btCompoundShape
	{
	public:
		std::vector<std::unique_ptr<btCollisionShape>> children;
	};

	class Fo4SkinnedMeshWorld :
		public hdt::SkinnedMeshWorld
	{
	public:
		using hdt::SkinnedMeshWorld::SkinnedMeshWorld;

		int StepReference(const btScalar a_remainingTimeStep, const btScalar a_fixedTimeStep)
		{
			BT_PROFILE("FO4FasterHdtSMP_StepReference");
			return stepReference(a_remainingTimeStep, a_fixedTimeStep);
		}

		void performDiscreteCollisionDetection() override
		{
			const auto profileStart = Clock::now();
			hdt::SkinnedMeshWorld::performDiscreteCollisionDetection();
			AddFrameCollisionProfile(ElapsedMs(profileStart, Clock::now()));
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
		std::vector<RE::NiPointer<RE::NiAVObject>> meshSourceRoots;
		RE::NiPointer<RE::NiAVObject> sourceObject;
		RE::NiPointer<RE::NiAVObject> sourceRoot;
		RE::NiPointer<RE::NiNode> destinationRoot;
		std::vector<Smp::ArmorBoneReference> boneReferences;
		Smp::BuildDomain domain{ Smp::BuildDomain::kHead };
	};

	std::optional<ArmorPhysicsXmlSelection> FindArmorPhysicsXml(RE::NiAVObject* a_object);
	RE::NiPoint3 ResolveWindRayStart(RE::Actor* a_actor);
	bool IsReadableMemory(const void* a_address, std::size_t a_minSize);
	bool IsProbablyValidNiObject(const RE::NiObject* a_object);
	RE::BSFlattenedBoneTree* FindFlattenedBoneTreeInScene(RE::NiAVObject* a_object);
	RE::BSFlattenedBoneTree::FlattenedBone* FindFlattenedBoneByName(
		RE::BSFlattenedBoneTree* a_tree,
		std::string_view a_name);
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
		return Smp::Address::ActorCalculateLOS(a_actor, a_targetPosition, a_hitPosition, 6.28F);
	}

	RE::NiPoint3 ResolveActorCullPosition(RE::Actor* a_actor, RE::NiAVObject* a_root)
	{
		if (a_root) {
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

		if (!a_root) {
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

	const char* BuildDomainName(const Smp::BuildDomain a_domain)
	{
		switch (a_domain) {
		case Smp::BuildDomain::kArmor:
			return "armor";
		case Smp::BuildDomain::kHead:
			return "head";
		case Smp::BuildDomain::kHair:
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

	const char* GetHeadPartModelPath(RE::BGSHeadPart* a_headPart)
	{
		return a_headPart ?
			static_cast<RE::BGSModelMaterialSwap*>(a_headPart)->GetModel() :
			nullptr;
	}

	void AddHeadpartClosureKeys(RE::BGSHeadPart* a_headPart, std::vector<std::string>& a_keys)
	{
		if (!a_headPart) {
			return;
		}

		AddHeadpartKey(a_keys, GetHeadPartModelPath(a_headPart));
		AddHeadpartKey(a_keys, std::string(std::string_view(a_headPart->formEditorID)));
		for (auto* extraPart : a_headPart->extraParts) {
			AddHeadpartClosureKeys(extraPart, a_keys);
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
			if (headPart && headPart->type.get() == RE::BGSHeadPart::HeadPartType::kHair) {
				AddHeadpartClosureKeys(headPart, keys);
			}
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
				if (auto* headNode = Smp::NiObject::GetObjectNodeByName(root, "HEAD")) {
					return headNode->world.translate;
				}
			}
			if (auto* root = a_actor->Get3D(); root) {
				if (auto* headNode = Smp::NiObject::GetObjectNodeByName(root, "HEAD")) {
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
			a_type == Smp::LifecycleEventType::kHeadSkinAllGeometry ||
			a_type == Smp::LifecycleEventType::kHeadSkinSingleGeometry;
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

	void AppendHeadCandidate(
		std::vector<HeadPhysicsXmlBuildCandidate>& a_candidates,
		RE::NiAVObject* a_object,
		std::filesystem::path a_path,
		Smp::DefaultBBP::NameMap a_meshNameMap,
		RE::NiAVObject* a_sourceObject,
		RE::NiNode* a_sourceRoot,
		const Smp::BuildDomain a_domain,
		RE::NiNode* a_destinationRoot = nullptr,
		std::vector<Smp::ArmorBoneReference> a_boneReferences = {})
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
			.destinationRoot = a_destinationRoot,
			.boneReferences = std::move(a_boneReferences),
			.domain = a_domain,
		});
	}

	struct HeadPartSourceSelection
	{
		std::filesystem::path path;
		RE::BSGeometry* geometry{ nullptr };
	};

	RE::BSGeometry* FindFirstGeometry(RE::NiAVObject* a_object)
	{
		if (!a_object) {
			return nullptr;
		}
		if (auto* triShape = a_object->IsTriShape()) {
			return triShape;
		}
		if (auto* geometry = a_object->IsGeometry()) {
			return geometry;
		}
		auto* node = a_object->IsNode();
		if (!node) {
			return nullptr;
		}
		for (auto& child : node->children) {
			if (auto* geometry = FindFirstGeometry(child.get())) {
				return geometry;
			}
		}
		return nullptr;
	}

	std::optional<HeadPartSourceSelection> FindHeadPartSourceSelection(RE::NiAVObject* a_object)
	{
		if (!a_object) {
			return std::nullopt;
		}
		if (auto path = FindDirectPhysicsXmlExtraData(a_object)) {
			return HeadPartSourceSelection{
				.path = std::move(*path),
				.geometry = FindFirstGeometry(a_object),
			};
		}
		auto* node = a_object->IsNode();
		if (!node) {
			return std::nullopt;
		}
		for (auto& child : node->children) {
			if (auto selection = FindHeadPartSourceSelection(child.get())) {
				return selection;
			}
		}
		return std::nullopt;
	}

	RE::BSGeometry* ResolveLiveHeadPartGeometry(
		RE::NiAVObject* a_faceObject,
		RE::BGSHeadPart* a_headPart,
		const std::string_view a_sourceGeometryName)
	{
		if (!a_faceObject || !a_headPart) {
			return nullptr;
		}
		if (!a_headPart->formEditorID.empty()) {
			if (auto* object = a_faceObject->GetObjectByName(a_headPart->formEditorID)) {
				if (auto* geometry = FindFirstGeometry(object)) {
					return geometry;
				}
			}
		}
		if (!a_sourceGeometryName.empty()) {
			if (auto* object = a_faceObject->GetObjectByName(RE::BSFixedString(std::string(a_sourceGeometryName)))) {
				return FindFirstGeometry(object);
			}
		}
		return nullptr;
	}

	void AppendLiveHairHeadPartCandidates(
		RE::Actor* a_actor,
		RE::NiAVObject* a_faceObject,
		RE::NiNode* a_destinationRoot,
		RE::BGSHeadPart* a_headPart,
		std::unordered_set<RE::BGSHeadPart*>& a_visited,
		Smp::DefaultBBP::NameMap& a_meshNameMap,
		std::vector<RE::NiPointer<RE::NiAVObject>>& a_meshSourceRoots,
		std::vector<HeadPhysicsXmlBuildCandidate>& a_candidates)
	{
		if (!a_actor || !a_faceObject || !a_destinationRoot || !a_headPart || !a_visited.insert(a_headPart).second) {
			return;
		}

		const auto* modelPath = GetHeadPartModelPath(a_headPart);
		if (modelPath && *modelPath) {
			// BSModelDB caches by the prepared resource ID. Load the complete model
			// before requesting FaceGen processing so an uncached extra-part NIF
			// does not enter the cache as a stripped FaceGen source.
			RE::BSModelDB::DBTraits::ArgsType meshArgs{};
			meshArgs.loadLevel = 3;
			meshArgs.performProcess = true;
			meshArgs.loadTextures = true;
			RE::NiPointer<RE::NiNode> meshSourceRoot;
			const auto meshError = RE::BSModelDB::Demand(modelPath, std::addressof(meshSourceRoot), meshArgs);
			if (meshError != RE::BSResource::ErrorCode::kNone || !meshSourceRoot) {
				spdlog::warn(
					"failed to load current hair headpart mesh source actor={} headPart={:08X} nif='{}' error={}",
					static_cast<void*>(a_actor),
					a_headPart->GetFormID(),
					modelPath,
					std::to_underlying(meshError));
			}

			auto faceGenArgs = meshArgs;
			faceGenArgs.faceGenModel = true;
			RE::NiPointer<RE::NiNode> sourceRoot;
			const auto faceGenError = RE::BSModelDB::Demand(modelPath, std::addressof(sourceRoot), faceGenArgs);
			if (faceGenError != RE::BSResource::ErrorCode::kNone || !sourceRoot) {
				if (meshSourceRoot) {
					spdlog::warn(
						"failed to load current hair headpart FaceGen source actor={} headPart={:08X} nif='{}' error={}; using complete model source",
						static_cast<void*>(a_actor),
						a_headPart->GetFormID(),
						modelPath,
						std::to_underlying(faceGenError));
					sourceRoot = meshSourceRoot;
				} else {
					spdlog::warn(
						"failed to load current hair headpart model actor={} headPart={:08X} nif='{}' error={}",
						static_cast<void*>(a_actor),
						a_headPart->GetFormID(),
						modelPath,
						std::to_underlying(faceGenError));
				}
			}
			if (!meshSourceRoot) {
				meshSourceRoot = sourceRoot;
			}

			if (sourceRoot && meshSourceRoot) {
				auto selection = FindHeadPartSourceSelection(sourceRoot.get());
				auto* sourceGeometry = selection && selection->geometry ?
					selection->geometry :
					FindFirstGeometry(meshSourceRoot.get());
				if (std::ranges::none_of(a_meshSourceRoots, [&](const auto& a_root) {
						return a_root.get() == meshSourceRoot.get();
					})) {
					a_meshSourceRoots.emplace_back(meshSourceRoot.get());
				}
				const auto sourceGeometryName = sourceGeometry ?
					std::string(std::string_view(sourceGeometry->GetName())) :
					std::string{};
				auto* liveGeometry = ResolveLiveHeadPartGeometry(a_faceObject, a_headPart, sourceGeometryName);
				if (!liveGeometry) {
					spdlog::debug(
						"current hair headpart source has no live FaceGen geometry actor={} headPart={:08X} editorID='{}' nif='{}' sourceGeometry='{}' xml='{}' supplementalMeshSource={}",
						static_cast<void*>(a_actor),
						a_headPart->GetFormID(),
						std::string_view(a_headPart->formEditorID),
						modelPath,
						sourceGeometryName,
						selection ? selection->path.string() : std::string{},
						true);
				} else {
					const auto liveGeometryName = std::string(std::string_view(liveGeometry->GetName()));
					const auto selectedXml = selection ? selection->path.string() : std::string{};
					if (!sourceGeometryName.empty() && !liveGeometryName.empty()) {
						a_meshNameMap[sourceGeometryName].emplace(liveGeometryName);
					}

					std::size_t referenceCount = 0;
					if (selection) {
						auto references = Smp::CaptureArmorBoneReferences(sourceRoot.get(), a_destinationRoot, modelPath);
						auto* actorBoneTree = FindFlattenedBoneTreeInScene(a_destinationRoot);
						for (auto& reference : references) {
							reference.isArmorOnly = FindFlattenedBoneByName(actorBoneTree, reference.name) == nullptr;
						}
						referenceCount = references.size();
						AppendHeadCandidate(
							a_candidates,
							liveGeometry,
							std::move(selection->path),
							{},
							liveGeometry,
							nullptr,
							Smp::BuildDomain::kHair,
							a_destinationRoot,
							std::move(references));
					}
					spdlog::debug(
						"resolved current hair headpart component actor={} headPart={:08X} nif='{}' xml='{}' sourceGeometry='{}' liveGeometry={} name='{}' references={}",
						static_cast<void*>(a_actor),
						a_headPart->GetFormID(),
						modelPath,
						selectedXml,
						sourceGeometryName,
						static_cast<void*>(liveGeometry),
						liveGeometryName,
						referenceCount);
				}
			}
		}

		for (auto* extraPart : a_headPart->extraParts) {
			AppendLiveHairHeadPartCandidates(
				a_actor,
				a_faceObject,
				a_destinationRoot,
				extraPart,
				a_visited,
				a_meshNameMap,
				a_meshSourceRoots,
				a_candidates);
		}
	}

	void CollectLiveHairHeadPartCandidates(
		RE::Actor* a_actor,
		RE::NiAVObject* a_faceObject,
		std::vector<HeadPhysicsXmlBuildCandidate>& a_candidates)
	{
		auto* npc = a_actor ? a_actor->GetNPC() : nullptr;
		auto* actorObject = a_actor ? a_actor->Get3D(false) : nullptr;
		auto* destinationRoot = actorObject ? actorObject->IsNode() : nullptr;
		if (!npc || !a_faceObject || !destinationRoot) {
			return;
		}

		for (auto* headPart : npc->GetHeadParts(true)) {
			if (!headPart || headPart->type.get() != RE::BGSHeadPart::HeadPartType::kHair) {
				continue;
			}

			std::unordered_set<RE::BGSHeadPart*> visited;
			Smp::DefaultBBP::NameMap meshNameMap;
			std::vector<RE::NiPointer<RE::NiAVObject>> meshSourceRoots;
			const auto firstCandidate = a_candidates.size();
			AppendLiveHairHeadPartCandidates(
				a_actor,
				a_faceObject,
				destinationRoot,
				headPart,
				visited,
				meshNameMap,
				meshSourceRoots,
				a_candidates);

			for (auto candidateIndex = firstCandidate; candidateIndex < a_candidates.size(); ++candidateIndex) {
				auto& candidate = a_candidates[candidateIndex];
				candidate.meshSourceRoots = meshSourceRoots;
				for (const auto& [sourceName, liveNames] : meshNameMap) {
					auto& candidateNames = candidate.meshNameMap[sourceName];
					for (const auto& liveName : liveNames) {
						candidateNames.emplace(liveName);
					}
				}
			}
		}
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

	std::vector<RE::NiAVObject*> BuildBipedPartExclusions(const Smp::LifecycleEvent& a_event)
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
		addArmorExclusion(a_event.sourceObject);
		addArmorExclusion(a_event.sourceRoot);

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
		if (auto directXml = FindDirectPhysicsXmlExtraData(a_object)) {
			AppendHeadCandidate(
				a_candidates,
				a_object,
				*directXml,
				{},
				a_object,
				a_object->IsNode(),
				isHair ? Smp::BuildDomain::kHair : Smp::BuildDomain::kHead);
			return;
		}

		if (auto defaultBbp = Smp::DefaultBBP::GetSingleton()->Find(a_object)) {
			AppendHeadCandidate(
				a_candidates,
				a_object,
				defaultBbp->physicsXml,
				std::move(defaultBbp->meshNameMap),
				a_object,
				a_object->IsNode(),
				isHair ? Smp::BuildDomain::kHair : Smp::BuildDomain::kHead);
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

	bool IsNamedPhysicsMesh(const std::vector<std::string>& a_meshNames, RE::BSGeometry* a_geometry)
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
			RE::NiAVObject* originalBone{ nullptr };
			RE::NiTransform* originalWorldTransform{ nullptr };
			RE::NiAVObject* originalRootNode{ nullptr };
		};

		RE::NiNode* node{ nullptr };
		RE::NiTransform* transform{ nullptr };
		RE::NiNode* sourceNode{ nullptr };
		RE::NiTransform canonicalWorld{ RE::NiTransform::IDENTITY };
		std::string name;
		bool hasCanonicalWorld{ false };
		bool isSharedActorBone{ false };
		bool meshOnlySkinBoneCandidate{ false };
		std::vector<SkinWorldTransformSlot> skinWorldTransforms;
	};

	struct SavedNodeLocalPose
	{
		RE::NiNode* node{ nullptr };
		RE::NiTransform local;
	};

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

	bool IsExcludedSkeletonSearchObject(
		RE::NiAVObject* a_object,
		const std::vector<RE::NiAVObject*>& a_excludedObjects)
	{
		if (!a_object) {
			return true;
		}

		return std::ranges::find(a_excludedObjects, a_object) != a_excludedObjects.end();
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

	std::unordered_set<RE::NiAVObject*> BuildKnownAttachmentNodeSet(const Smp::LifecycleEvent& a_event)
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
		auto addAttachmentRoot = [&](RE::NiAVObject* a_object) {
			if (!a_object || a_object == primaryActorRoot || a_object == thirdPersonActorRoot || a_object == firstPersonActorRoot) {
				return;
			}
			addRoot(a_object);
		};

		addAttachmentRoot(a_event.object);
		addAttachmentRoot(a_event.sourceObject);
		addAttachmentRoot(a_event.sourceRoot);
		if (auto* biped = ResolveEventBiped(a_event)) {
			for (auto index = 0; index < std::to_underlying(RE::BIPED_OBJECT::kTotal); ++index) {
				const auto bipedObject = static_cast<RE::BIPED_OBJECT>(index);
				auto* bipObject = biped->GetBipObject(bipedObject);
				if (bipObject && bipObject->partClone) {
					addRoot(bipObject->partClone.get());
				}
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

	RE::NiNode* FindNodeByNameExcludingKnownNodes(
		RE::NiAVObject* a_root,
		const std::string_view a_name,
		const std::vector<RE::NiAVObject*>& a_excludedObjects,
		const std::unordered_set<RE::NiAVObject*>& a_knownAttachmentNodes)
	{
		if (!a_root ||
			a_name.empty() ||
			IsExcludedSkeletonSearchObject(a_root, a_excludedObjects) ||
			a_knownAttachmentNodes.contains(a_root)) {
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
			if (auto* found = FindNodeByNameExcludingKnownNodes(child.get(), a_name, a_excludedObjects, a_knownAttachmentNodes)) {
				return found;
			}
		}

		return nullptr;
	}

	const Smp::PhysicsBoneDescriptor* FindBoneDescriptor(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name)
	{
		const auto found = std::ranges::find_if(a_summary.boneDescriptors, [a_name](const Smp::PhysicsBoneDescriptor& a_descriptor) {
			return Smp::PhysicsNamesEqual(a_descriptor.name, a_name);
		});
		return found == a_summary.boneDescriptors.end() ? nullptr : std::addressof(*found);
	}

	template <class Body>
	void AddBodyBuildGroup(
		Body& a_body,
		const std::uint64_t a_buildGroup,
		const Smp::BuildDomain a_domain,
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
	bool BodyHasBuildGroup(const Body& a_body, const std::uint64_t a_buildGroup)
	{
		if (a_buildGroup == 0) {
			return false;
		}
		return a_body.buildGroup == a_buildGroup || std::ranges::find(a_body.buildGroups, a_buildGroup) != a_body.buildGroups.end();
	}

	std::vector<std::pair<std::size_t, const Smp::PhysicsMeshShapeDescriptor*>> FindMeshDescriptors(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name, const Smp::DefaultBBP::NameMap& a_meshNameMap)
	{
		std::vector<std::pair<std::size_t, const Smp::PhysicsMeshShapeDescriptor*>> result;
		for (std::size_t index = 0; index < a_summary.meshDescriptors.size(); ++index) {
			const auto& descriptor = a_summary.meshDescriptors[index];
			if (MeshNameMatches(descriptor.name, a_name, a_meshNameMap)) {
				result.emplace_back(index, std::addressof(descriptor));
			}
		}
		return result;
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

			const auto meshMatched = IsNamedPhysicsMesh(a_meshNames, geometry);
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
			std::vector<RE::NiNode*> skinBones;
			skinBones.reserve(skin->bones.size());
			for (std::uint32_t index = 0; index < skin->bones.size(); ++index) {
				auto* boneObject = skin->bones[index];
				if (!boneObject) {
					skinBones.push_back(nullptr);
					continue;
				}
				if (!IsProbablyValidNiObject(boneObject)) {
					spdlog::warn(
						"skipping geometry '{}' because skin bone pointer={} slot={} is invalid",
						geometry->GetName(),
						static_cast<void*>(boneObject),
						index);
					return;
				}

				auto* bone = boneObject->IsNode();
				if (!bone) {
					spdlog::warn("skipping geometry '{}' because skin bone '{}' slot={} is not a node", geometry->GetName(), boneObject->GetName(), index);
					return;
				}
				skinBones.push_back(bone);
			}

			for (std::uint32_t index = 0; index < skinBones.size(); ++index) {
				auto* bone = skinBones[index];
				if (!bone) {
					continue;
				}

				const auto name = bone->GetName();
				const auto matchedName = Smp::FindMatchingPhysicsName(a_boneNames, name);
				if (!name.empty() && (includeAllSkinBones || includeAllWhenUnfiltered || matchedName)) {
					if (auto* existing = FindMatchedSkinBone(a_result, bone)) {
						if (matchedName) {
							existing->name = std::string(*matchedName);
							existing->meshOnlySkinBoneCandidate = false;
						}
						AddSkinWorldTransformSlot(*existing, skin, index);
						continue;
					}

					auto& matchedBone = a_result.emplace_back(MatchedSkinBone{
						.node = bone,
						.transform = std::addressof(bone->world),
						.sourceNode = bone,
						.name = matchedName ? std::string(*matchedName) : std::string(name),
						.meshOnlySkinBoneCandidate = !matchedName && includeAllSkinBones,
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

	void CollectMatchedArmorReferenceBones(
		const std::vector<Smp::ArmorBoneReference>& a_references,
		const std::vector<std::string>& a_boneNames,
		RE::NiAVObject* a_actorRoot,
		std::vector<MatchedSkinBone>& a_result)
	{
		if (!a_actorRoot || a_boneNames.empty()) {
			return;
		}

		for (const auto& reference : a_references) {
			auto* node = reference.resolvedNode.get();
			if (!node || !IsNodeInTree(a_actorRoot, node)) {
				continue;
			}

			const auto matchedName = Smp::FindMatchingPhysicsName(a_boneNames, reference.name);
			if (!matchedName) {
				continue;
			}

			if (auto* existing = FindMatchedSkinBone(a_result, node)) {
				existing->name = std::string(*matchedName);
				existing->meshOnlySkinBoneCandidate = false;
				continue;
			}
			if (auto* existing = FindMatchedSkinBoneByName(a_result, *matchedName)) {
				spdlog::warn(
					"resolved armor node '{}' differs from existing matched node actorRoot={} resolved={} existing={}",
					*matchedName,
					static_cast<void*>(a_actorRoot),
					static_cast<void*>(node),
					static_cast<void*>(existing->node));
				continue;
			}

			a_result.push_back({
				.node = node,
				.transform = std::addressof(node->world),
				.sourceNode = node,
				.name = std::string(*matchedName),
				.isSharedActorBone = !reference.isArmorOnly,
			});
			spdlog::debug(
				"matched XML bone '{}' from persistent armor hierarchy node={} nodeName='{}' createdByUs={}",
				*matchedName,
				static_cast<void*>(node),
				std::string_view(node->GetName()),
				reference.createdByUs);
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
		const std::unordered_set<RE::NiAVObject*>& a_knownAttachmentNodes,
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
				a_knownAttachmentNodes);
			if (!boneNode) {
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
			Smp::NiObject::UpdateWorldData(a_root, true);
			spdlog::debug(
				"applied Havok reference pose for system build actor={} root={} bones={} matched={}",
				static_cast<void*>(a_actor),
				static_cast<void*>(a_root),
				count,
				applied);
			return true;
		}

		return false;
	}

	RE::NiTransform ComposeFo4NiTransform(
		const RE::NiTransform& a_parentWorld,
		const RE::NiTransform& a_local)
	{
		// Fallout 4 stores Ni rotations transposed relative to Bullet. Its
		// NiTransform composition therefore multiplies the stored matrices in
		// local-to-parent order. This is the math used by both
		// NiAVObject::UpdateWorldData and BSFlattenedBoneTree::UpdateBoneArray.
		RE::NiTransform result;
		result.rotate = a_local.rotate * a_parentWorld.rotate;
		result.translate =
			a_parentWorld.translate +
			(a_parentWorld.rotate.Transpose() * a_local.translate) * a_parentWorld.scale;
		result.scale = a_parentWorld.scale * a_local.scale;
		return result;
	}

	bool BuildDetachedHavokReferencePose(
		RE::Actor* a_actor,
		RE::BSFlattenedBoneTree* a_flattened,
		const std::vector<Smp::ArmorBoneReference>& a_armorBoneReferences,
		std::vector<MatchedSkinBone>& a_matchedBones)
	{
		// Constraint frames still need a canonical pose, but hot armor attachment
		// must not expose that pose through the actor's live scene graph.
		const auto* skeleton = GetHavokReferenceSkeleton(a_actor);
		if (!skeleton ||
			!a_flattened ||
			!a_flattened->bone ||
			a_flattened->boneCountExpanded <= 0 ||
			skeleton->bones.size <= 0 ||
			skeleton->referencePose.size <= 0) {
			return false;
		}

		const auto havokBoneCount = std::min(skeleton->bones.size, skeleton->referencePose.size);
		std::unordered_map<std::string, RE::NiTransform> referenceLocals;
		referenceLocals.reserve(static_cast<std::size_t>(havokBoneCount));
		for (std::int32_t index = 0; index < havokBoneCount; ++index) {
			const auto* boneName = HkStringPtrData(skeleton->bones.data[index].name);
			if (!boneName || *boneName == '\0') {
				continue;
			}
			referenceLocals.insert_or_assign(
				Smp::NormalizePhysicsName(boneName),
				ToNiTransform(skeleton->referencePose.data[index]));
		}
		if (referenceLocals.empty()) {
			return false;
		}
		const auto hasHavokReference = [&](const std::string_view a_name) {
			return referenceLocals.contains(Smp::NormalizePhysicsName(a_name));
		};

		// Emulate BSFlattenedBoneTree::UpdateBoneArray without mutating the live
		// tree. The reference implementation replaces matching Ni locals with
		// Havok reference locals and then lets the game skeleton hierarchy compute
		// worlds; composing through hkaSkeleton::parentIndices is not equivalent
		// for merged armor/hair skeletons.
		const auto flattenedBoneCount = static_cast<std::size_t>(a_flattened->boneCountExpanded);
		std::unordered_map<std::string, std::size_t> flattenedIndices;
		flattenedIndices.reserve(flattenedBoneCount);
		for (std::size_t index = 0; index < flattenedBoneCount; ++index) {
			const auto name = std::string_view(a_flattened->bone[index].name);
			if (!name.empty()) {
				flattenedIndices.try_emplace(Smp::NormalizePhysicsName(name), index);
			}
		}

		std::vector<RE::NiTransform> canonicalActorWorlds(flattenedBoneCount, RE::NiTransform::IDENTITY);
		std::vector<std::uint8_t> actorWorldState(flattenedBoneCount, 0);
		std::uint32_t appliedReferenceLocals = 0;
		std::uint32_t retainedFlattenedLocals = 0;
		const auto computeActorWorld = [&](this auto&& a_self, const std::size_t a_index, RE::NiTransform& a_world) -> bool {
			if (a_index >= canonicalActorWorlds.size()) {
				return false;
			}
			if (actorWorldState[a_index] == 2) {
				a_world = canonicalActorWorlds[a_index];
				return true;
			}
			if (actorWorldState[a_index] == 1) {
				return false;
			}

			actorWorldState[a_index] = 1;
			const auto& flattenedBone = a_flattened->bone[a_index];
			const auto referenceLocal = referenceLocals.find(Smp::NormalizePhysicsName(flattenedBone.name.c_str()));
			const auto& local = referenceLocal != referenceLocals.end() ? referenceLocal->second : flattenedBone.local;
			if (referenceLocal != referenceLocals.end()) {
				++appliedReferenceLocals;
			} else {
				++retainedFlattenedLocals;
			}

			RE::NiTransform parentWorld = a_flattened->world;
			const auto parentIndex = flattenedBone.parent;
			if (parentIndex >= 0) {
				if (static_cast<std::size_t>(parentIndex) == a_index ||
					!a_self(static_cast<std::size_t>(parentIndex), parentWorld)) {
					actorWorldState[a_index] = 0;
					return false;
				}
			}

			a_world = ComposeFo4NiTransform(parentWorld, local);
			canonicalActorWorlds[a_index] = a_world;
			actorWorldState[a_index] = 2;
			return true;
		};

		const auto findActorWorld = [&](const std::string_view a_name, RE::NiTransform& a_world) {
			// The flattened tree object itself is the named skeleton root; it is
			// not duplicated in the flattened-bone array.
			if (Smp::PhysicsNamesEqual(a_name, a_flattened->GetName())) {
				a_world = a_flattened->world;
				return true;
			}
			const auto found = flattenedIndices.find(Smp::NormalizePhysicsName(a_name));
			return found != flattenedIndices.end() && computeActorWorld(found->second, a_world);
		};

		std::unordered_map<std::string, const Smp::ArmorBoneReference*> capturedReferences;
		capturedReferences.reserve(a_armorBoneReferences.size());
		for (const auto& reference : a_armorBoneReferences) {
			if (!reference.name.empty()) {
				capturedReferences.try_emplace(Smp::NormalizePhysicsName(reference.name), std::addressof(reference));
			}
		}
		std::unordered_map<std::string, RE::NiTransform> canonicalCapturedWorlds;
		canonicalCapturedWorlds.reserve(capturedReferences.size());
		std::unordered_set<std::string> visitingCapturedBones;
		const auto computeCapturedWorld = [&](this auto&& a_self, const Smp::ArmorBoneReference& a_reference, RE::NiTransform& a_world) -> bool {
			const auto normalizedName = Smp::NormalizePhysicsName(a_reference.name);
			if (const auto cached = canonicalCapturedWorlds.find(normalizedName); cached != canonicalCapturedWorlds.end()) {
				a_world = cached->second;
				return true;
			}

			// Shared bones belong to the actor pose whenever FO4 exposes them
			// through the flattened tree. The captured transform is only the
			// compatibility fallback for shared attachment nodes omitted from
			// that tree (notably some twist bones).
			if (!a_reference.isArmorOnly && findActorWorld(a_reference.name, a_world)) {
				canonicalCapturedWorlds.emplace(normalizedName, a_world);
				return true;
			}
			if (!visitingCapturedBones.insert(normalizedName).second) {
				return false;
			}

			RE::NiTransform parentWorld;
			bool foundParent = false;
			if (!a_reference.parentBoneName.empty()) {
				const auto normalizedParent = Smp::NormalizePhysicsName(a_reference.parentBoneName);
				const auto parentReference = capturedReferences.find(normalizedParent);
				if (parentReference != capturedReferences.end() && parentReference->second->isArmorOnly) {
					foundParent = a_self(*parentReference->second, parentWorld);
				} else {
					foundParent = findActorWorld(a_reference.parentBoneName, parentWorld);
					if (!foundParent && parentReference != capturedReferences.end()) {
						foundParent = a_self(*parentReference->second, parentWorld);
					}
				}
			} else {
				parentWorld = a_flattened->world;
				foundParent = true;
			}
			visitingCapturedBones.erase(normalizedName);
			if (!foundParent) {
				return false;
			}

			a_world = ComposeFo4NiTransform(parentWorld, a_reference.localToParentBone);
			canonicalCapturedWorlds.emplace(normalizedName, a_world);
			return true;
		};

		std::uint32_t appliedArmorBones = 0;
		std::uint32_t appliedActorBones = 0;
		std::uint32_t appliedCapturedSharedFallbackBones = 0;
		std::uint32_t resolvedMatchedBones = 0;
		std::uint32_t requiredMatchedBones = 0;
		std::vector<std::string_view> unresolvedRequiredBones;
		for (auto& matchedBone : a_matchedBones) {
			if (!matchedBone.meshOnlySkinBoneCandidate) {
				++requiredMatchedBones;
			}
			RE::NiTransform canonicalWorld;
			bool resolved = false;
			const auto reference = capturedReferences.find(Smp::NormalizePhysicsName(matchedBone.name));
			if (hasHavokReference(matchedBone.name)) {
				resolved = findActorWorld(matchedBone.name, canonicalWorld);
				if (resolved) {
					++appliedActorBones;
				}
			} else if (reference != capturedReferences.end() && reference->second->isArmorOnly) {
				resolved = computeCapturedWorld(*reference->second, canonicalWorld);
				if (resolved) {
					++appliedArmorBones;
				}
			} else {
				resolved = findActorWorld(matchedBone.name, canonicalWorld);
				if (resolved) {
					++appliedActorBones;
				} else if (reference != capturedReferences.end()) {
					resolved = computeCapturedWorld(*reference->second, canonicalWorld);
					if (resolved) {
						++appliedCapturedSharedFallbackBones;
					}
				}
			}
			if (!resolved) {
				if (!matchedBone.meshOnlySkinBoneCandidate) {
					unresolvedRequiredBones.push_back(matchedBone.name);
				}
				continue;
			}
			matchedBone.canonicalWorld = canonicalWorld;
			matchedBone.hasCanonicalWorld = true;
			++resolvedMatchedBones;
		}

		spdlog::debug(
			"built detached armor reference pose for system actor={} flattened={} havokBones={} flattenedBones={} appliedReferenceLocals={} retainedFlattenedLocals={} capturedArmorOnlyBones={} actorBones={} capturedSharedFallbackBones={} matchedBones={}/{} requiredBones={} unresolvedRequired={}",
			static_cast<void*>(a_actor),
			static_cast<void*>(a_flattened),
			havokBoneCount,
			flattenedBoneCount,
			appliedReferenceLocals,
			retainedFlattenedLocals,
			appliedArmorBones,
			appliedActorBones,
			appliedCapturedSharedFallbackBones,
			resolvedMatchedBones,
			a_matchedBones.size(),
			requiredMatchedBones,
			unresolvedRequiredBones.size());
		for (const auto name : unresolvedRequiredBones) {
			spdlog::warn(
				"skipping armor physics bone '{}' because it has neither an actor reference transform nor a captured NIF-local armor hierarchy actor={}",
				name,
				static_cast<void*>(a_actor));
		}
		std::erase_if(a_matchedBones, [](const MatchedSkinBone& a_bone) {
			return !a_bone.hasCanonicalWorld;
		});
		return requiredMatchedBones > unresolvedRequiredBones.size();
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
		Smp::NiObject::UpdateWorldData(a_updateRoot, true);
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

	RE::BSFlattenedBoneTree* FindFlattenedBoneTreeInScene(RE::NiAVObject* a_object)
	{
		if (!a_object) {
			return nullptr;
		}
		if (auto* flattened = netimmerse_cast<RE::BSFlattenedBoneTree*>(a_object)) {
			return flattened;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return nullptr;
		}
		for (auto& child : node->children) {
			if (auto* flattened = FindFlattenedBoneTreeInScene(child.get())) {
				return flattened;
			}
		}
		return nullptr;
	}

	RE::BSFlattenedBoneTree::FlattenedBone* FindFlattenedBoneByName(
		RE::BSFlattenedBoneTree* a_tree,
		const std::string_view a_name)
	{
		if (!a_tree || !a_tree->bone || a_name.empty()) {
			return nullptr;
		}

		const auto found = a_tree->boneMap.find(RE::BSFixedString(std::string(a_name)));
		if (found == a_tree->boneMap.end()) {
			return nullptr;
		}
		const auto index = found->second;
		if (index < 0 || index >= a_tree->boneCountExpanded) {
			return nullptr;
		}
		return std::addressof(a_tree->bone[index]);
	}

	void ResolveEnginePreparedBones(
		std::vector<MatchedSkinBone>& a_matchedBones,
		const std::vector<std::string>& a_boneNames,
		RE::NiAVObject* a_actorRoot,
		RE::NiNode* a_skeletonRoot)
	{
		auto* flattened = FindFlattenedBoneTreeInScene(a_skeletonRoot);
		if (!flattened) {
			flattened = FindFlattenedBoneTreeInScene(a_actorRoot);
		}

		for (auto& matchedBone : a_matchedBones) {
			matchedBone.transform = matchedBone.node ? std::addressof(matchedBone.node->world) : matchedBone.transform;
			if (auto* flattenedBone = FindFlattenedBoneByName(flattened, matchedBone.name)) {
				matchedBone.node = flattenedBone->node.get();
				matchedBone.transform = matchedBone.node ? std::addressof(matchedBone.node->world) : std::addressof(flattenedBone->world);
				matchedBone.isSharedActorBone = true;
				matchedBone.meshOnlySkinBoneCandidate = false;
			} else if (matchedBone.node) {
				matchedBone.isSharedActorBone = false;
			}
		}

		for (const auto& boneName : a_boneNames) {
			if (boneName.empty() || FindMatchedSkinBoneByName(a_matchedBones, boneName)) {
				continue;
			}
			auto* flattenedBone = FindFlattenedBoneByName(flattened, boneName);
			if (!flattenedBone) {
				continue;
			}

			auto* node = flattenedBone->node.get();
			a_matchedBones.push_back({
				.node = node,
				.transform = node ? std::addressof(node->world) : std::addressof(flattenedBone->world),
				.name = boneName,
				.isSharedActorBone = true,
			});
			spdlog::debug(
				"resolved XML anchor '{}' directly from the engine BSFlattenedBoneTree node={} transform={}",
				boneName,
				static_cast<void*>(node),
				static_cast<void*>(a_matchedBones.back().transform));
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
			const auto origin = a_transformA.getOrigin().lerp(a_transformB.getOrigin(), a_descriptor.translationLerp);
			const auto rotation = a_transformA.getBasis().slerp(a_transformB.getBasis(), a_descriptor.rotationLerp);
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

	RE::BSTSmartPointer<hdt::BoneScaleConstraint> CreateConstraint(
		const Smp::PhysicsConstraintDescriptor& a_descriptor,
		hdt::SkinnedMeshBone* a_boneA,
		hdt::SkinnedMeshBone* a_boneB,
		const hdt::btQsTransform& a_nodeTransformA,
		const hdt::btQsTransform& a_nodeTransformB)
	{
		if (!a_boneA || !a_boneB) {
			return nullptr;
		}

		auto [nodeFrameA, nodeFrameB] = CalculateConstraintFrames(a_descriptor, a_nodeTransformA, a_nodeTransformB);

		switch (a_descriptor.kind) {
		case Smp::PhysicsConstraintKind::kConeTwist:
		{
			auto constraint = RE::make_smart<hdt::ConeTwistConstraint>(
				a_boneA,
				a_boneB,
				nodeFrameA,
				nodeFrameB);
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
		{
			auto constraint = RE::make_smart<hdt::StiffSpringConstraint>(a_boneA, a_boneB);
			const auto initialDistance = a_nodeTransformA.getOrigin().distance(a_nodeTransformB.getOrigin());
			constraint->m_minDistance = initialDistance * std::max(a_descriptor.minDistanceFactor, 0.0F);
			constraint->m_maxDistance = initialDistance * std::max(a_descriptor.maxDistanceFactor, 0.0F);
			const auto equilibriumFactor = std::clamp(a_descriptor.equilibriumFactor, 0.0F, 1.0F);
			constraint->m_equilibriumPoint =
				constraint->m_minDistance * equilibriumFactor +
				constraint->m_maxDistance * (1.0F - equilibriumFactor);
			constraint->m_stiffness = std::max(a_descriptor.stiffness, 0.0F);
			constraint->m_damping = std::max(a_descriptor.damping, 0.0F);
			return constraint;
		}
		case Smp::PhysicsConstraintKind::kGeneric:
		default:
		{
			const bool swapBodies = a_descriptor.useLinearReferenceFrameA;
			auto constraint = swapBodies ?
				RE::make_smart<hdt::Generic6DofConstraint>(a_boneB, a_boneA, nodeFrameB, nodeFrameA) :
				RE::make_smart<hdt::Generic6DofConstraint>(a_boneA, a_boneB, nodeFrameA, nodeFrameB);
			constraint->setLinearLowerLimit(ToBulletVector(a_descriptor.linearLowerLimit));
			constraint->setLinearUpperLimit(ToBulletVector(a_descriptor.linearUpperLimit));
			constraint->setAngularLowerLimit(ToBulletVector(a_descriptor.angularLowerLimit));
			constraint->setAngularUpperLimit(ToBulletVector(a_descriptor.angularUpperLimit));
			for (int axis = 0; axis < 3; ++axis) {
				constraint->setStiffness(axis, AxisValue(a_descriptor.linearStiffness, axis), a_descriptor.linearStiffnessLimited);
				constraint->setStiffness(axis + 3, AxisValue(a_descriptor.angularStiffness, axis), a_descriptor.angularStiffnessLimited);
				constraint->setDamping(axis, AxisValue(a_descriptor.linearDamping, axis), a_descriptor.springDampingLimited);
				constraint->setDamping(axis + 3, AxisValue(a_descriptor.angularDamping, axis), a_descriptor.springDampingLimited);
				constraint->setEquilibriumPoint(axis, AxisValue(a_descriptor.linearEquilibrium, axis));
				constraint->setEquilibriumPoint(axis + 3, AxisValue(a_descriptor.angularEquilibrium, axis));
				constraint->setNonHookeanDamping(axis, AxisValue(a_descriptor.linearNonHookeanDamping, axis));
				constraint->setNonHookeanDamping(axis + 3, AxisValue(a_descriptor.angularNonHookeanDamping, axis));
				constraint->setNonHookeanStiffness(axis, AxisValue(a_descriptor.linearNonHookeanStiffness, axis));
				constraint->setNonHookeanStiffness(axis + 3, AxisValue(a_descriptor.angularNonHookeanStiffness, axis));
				constraint->enableSpring(axis, a_descriptor.enableLinearSprings);
				constraint->enableSpring(axis + 3, a_descriptor.enableAngularSprings);
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
				constraint->setParam(BT_CONSTRAINT_CFM, a_descriptor.motorCfm, axis);
				constraint->setParam(BT_CONSTRAINT_STOP_ERP, a_descriptor.stopErp, axis);
				constraint->setParam(BT_CONSTRAINT_STOP_CFM, a_descriptor.stopCfm, axis);
				if (auto* motor = constraint->getRotationalLimitMotor(axis)) {
					motor->m_motorERP = a_descriptor.motorErp;
					motor->m_motorCFM = a_descriptor.motorCfm;
					motor->m_stopERP = a_descriptor.stopErp;
					motor->m_stopCFM = a_descriptor.stopCfm;
				}
			}
			constraint->getTranslationalLimitMotor()->m_bounce = ToBulletVector(a_descriptor.linearBounce);
			for (int axis = 0; axis < 3; ++axis) {
				if (auto* motor = constraint->getRotationalLimitMotor(axis)) {
					motor->m_bounce = AxisValue(a_descriptor.angularBounce, axis);
				}
			}
			return constraint;
		}
		}
	}
}


#include "Fo4PhysicsWorld/Core.inl"
#include "Fo4PhysicsWorld/Console.inl"
#include "Fo4PhysicsWorld/Simulation.inl"
#include "Fo4PhysicsWorld/Events.inl"
#include "Fo4PhysicsWorld/Lifecycle.inl"
#include "Fo4PhysicsWorld/Rebuilds.inl"
#include "Fo4PhysicsWorld/RuntimeState.inl"
#include "Fo4PhysicsWorld/Build.inl"
#include "Fo4PhysicsWorld/Papyrus.inl"
