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
#include "HeadPartModelSourceCache.h"
#include "BulletVisualization.h"
#include "PhysicsName.h"
#include "PhysicsXml.h"
#include "PhysicsXmlSelection.h"
#include "PapyrusFunctions.h"
#include "PhysicsProfiler.h"
#include "ResourceFile.h"
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
#include "RE/B/BSTimer.h"
#include "RE/A/AIProcess.h"
#include "RE/C/CFilter.h"
#include "RE/C/COL_LAYER.h"
#include "RE/H/hkArray.h"
#include "RE/H/hkQsTransformf.h"
#include "RE/H/hkReferencedObject.h"
#include "RE/H/hkRefPtr.h"
#include "RE/H/HighProcessData.h"
#include "RE/M/Main.h"
#include "RE/M/MenuOpenCloseEvent.h"
#include "RE/N/NiStringExtraData.h"
#include "RE/P/PlayerCamera.h"
#include "RE/P/PlayerCharacter.h"
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
#include <cstring>
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
	constexpr std::uint32_t kFaceGenStableIdleObservations = 2;
	constexpr std::uint32_t kArmorChangeRebuildDelayTasks = 0;
	constexpr std::uint32_t kCpuCopyPendingRetryDelayTasks = 10;
	constexpr std::uint32_t kCpuCopyPendingMaxRetries = 3;
	constexpr std::uint32_t kSkeletonTransitionLoadTimeoutFrames = 120;
	constexpr std::uint32_t kRetainedFaceSkinningTimeoutFrames = 30;
	constexpr std::uint32_t kSkeletonTransitionResolutionTimeoutFrames = 120;
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
		struct PathVisibilitySources
		{
			std::filesystem::path path;
			std::vector<RE::NiPointer<RE::NiAVObject>> objects;
		};

		RE::NiAVObject* object{ nullptr };
		std::vector<std::filesystem::path> paths;
		std::vector<PathVisibilitySources> pathVisibilitySources;
		Smp::DefaultBBP::NameMap meshNameMap;
		std::vector<RE::NiPointer<RE::NiAVObject>> meshSourceRoots;
		std::vector<RE::NiPointer<RE::NiAVObject>> liveBindingObjects;
		RE::NiPointer<RE::NiAVObject> sourceObject;
		RE::NiPointer<RE::NiAVObject> sourceRoot;
		RE::NiPointer<RE::NiNode> destinationRoot;
		std::vector<Smp::ArmorBoneReference> boneReferences;
		Smp::BuildDomain domain{ Smp::BuildDomain::kHead };
		bool isHeadPartClosure{ false };
	};

	template <class T>
	T ReadSegmentField(const void* a_base, const std::size_t a_offset)
	{
		T value{};
		std::memcpy(std::addressof(value), static_cast<const std::byte*>(a_base) + a_offset, sizeof(value));
		return value;
	}

	bool SegmentEntryHasEnabledDrawRange(
		const std::byte* a_entries,
		const std::uint32_t a_entryIndex,
		const std::uint32_t a_depth,
		std::uint32_t& a_visited)
	{
		constexpr std::size_t kEntryStride = 0x14;
		constexpr std::uint32_t kTraversalLimit = 4096;
		constexpr std::uint32_t kDepthLimit = 16;
		if (!a_entries || a_entryIndex >= kTraversalLimit || ++a_visited > kTraversalLimit || a_depth > kDepthLimit) {
			// Unknown/corrupt segment layouts are kept active. Only a proven fully
			// hidden source is allowed to suppress physics.
			return true;
		}

		const auto* entry = a_entries + static_cast<std::size_t>(a_entryIndex) * kEntryStride;
		const auto primitiveCount = ReadSegmentField<std::uint32_t>(entry, 0x4);
		const auto childCount = ReadSegmentField<std::uint32_t>(entry, 0xC);
		const auto disableCount = ReadSegmentField<std::uint8_t>(entry, 0x10);
		if (disableCount == 0 && primitiveCount != 0) {
			return true;
		}
		if (childCount >= kTraversalLimit || a_entryIndex >= kTraversalLimit - childCount) {
			return true;
		}
		for (std::uint32_t child = 0; child < childCount; ++child) {
			if (SegmentEntryHasEnabledDrawRange(a_entries, a_entryIndex + child + 1, a_depth + 1, a_visited)) {
				return true;
			}
		}
		return false;
	}

	bool GeometryHasEnabledDrawRange(RE::BSGeometry* a_geometry)
	{
		if (!a_geometry) {
			return false;
		}
		auto* segmentData = a_geometry->GetSegmentData();
		if (!segmentData) {
			return true;
		}

		// HeadPart::HideShowHair mutates BSGeometrySegmentData's disable counts.
		// These are the OG/AE-stable fields consumed by UpdateDrawData: the shared
		// segment map, flat 0x14-byte draw entries, top-level count/remap, and the
		// one-segment flag. Unknown layouts deliberately resolve as visible.
		constexpr std::uint32_t kSegmentLimit = 4096;
		const auto segmentCount = ReadSegmentField<std::uint8_t>(segmentData, 0x3D) == 0 ?
			ReadSegmentField<std::uint32_t>(segmentData, 0x2C) :
			1U;
		if (segmentCount > kSegmentLimit) {
			return true;
		}
		const auto entriesAddress = ReadSegmentField<std::uintptr_t>(segmentData, 0x18);
		if (!entriesAddress) {
			return true;
		}

		const auto mappingAddress = ReadSegmentField<std::uintptr_t>(segmentData, 0x10);
		const auto mappingIndicesAddress = mappingAddress ?
			ReadSegmentField<std::uintptr_t>(reinterpret_cast<const void*>(mappingAddress), 0x18) :
			0;
		const auto remappedSegment = ReadSegmentField<std::uint32_t>(segmentData, 0x38);
		const auto* entries = reinterpret_cast<const std::byte*>(entriesAddress);
		for (std::uint32_t segment = 0; segment < segmentCount; ++segment) {
			const auto logicalSegment = segment == remappedSegment ? 0U : segment;
			std::uint32_t entryIndex = logicalSegment;
			if (mappingAddress) {
				if (!mappingIndicesAddress) {
					return true;
				}
				entryIndex = ReadSegmentField<std::uint32_t>(
					reinterpret_cast<const void*>(mappingIndicesAddress),
					static_cast<std::size_t>(logicalSegment) * sizeof(std::uint32_t));
			}
			if (entryIndex == std::numeric_limits<std::uint32_t>::max()) {
				continue;
			}
			if (entryIndex >= kSegmentLimit) {
				return true;
			}

			std::uint32_t visited = 0;
			if (SegmentEntryHasEnabledDrawRange(entries, entryIndex, 0, visited)) {
				return true;
			}
		}
		return false;
	}

	struct GeometryVisibility
	{
		bool foundGeometry{ false };
		bool foundVisibleGeometry{ false };
	};

	void CollectGeometryVisibility(
		RE::NiAVObject* a_object,
		GeometryVisibility& a_result)
	{
		if (!a_object || a_result.foundVisibleGeometry) {
			return;
		}
		if (auto* geometry = a_object->IsGeometry()) {
			a_result.foundGeometry = true;
			a_result.foundVisibleGeometry = !geometry->GetAppCulled() && GeometryHasEnabledDrawRange(geometry);
			return;
		}
		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}
		for (auto& child : node->children) {
			CollectGeometryVisibility(child.get(), a_result);
			if (a_result.foundVisibleGeometry) {
				return;
			}
		}
	}

	bool IsPhysicsSourceEntirelyHidden(RE::NiAVObject* a_object)
	{
		if (!a_object) {
			return false;
		}

		// Parent nodes are view-dependent: the player head is app-culled in first
		// person even though its armor-controlled hair segments remain enabled.
		// Only geometry-local state is evidence that the hair source itself is hidden.
		GeometryVisibility visibility;
		CollectGeometryVisibility(a_object, visibility);
		return visibility.foundGeometry && !visibility.foundVisibleGeometry;
	}

	bool IsHeadPhysicsPathEntirelyHidden(
		const HeadPhysicsXmlBuildCandidate& a_candidate,
		const std::filesystem::path& a_path)
	{
		const auto pathKey = Smp::ResourceFile::ComparisonKey(a_path.string());
		const auto sources = std::ranges::find_if(a_candidate.pathVisibilitySources, [&](const auto& a_source) {
			return Smp::ResourceFile::ComparisonKey(a_source.path.string()) == pathKey;
		});
		if (sources != a_candidate.pathVisibilitySources.end() && !sources->objects.empty()) {
			return std::ranges::all_of(sources->objects, [](const auto& a_object) {
				return IsPhysicsSourceEntirelyHidden(a_object.get());
			});
		}

		// Direct candidates pre-dating source metadata, or candidates whose live
		// association could not be resolved, stay active unless their fallback
		// source is itself proven wholly hidden.
		auto* fallback = a_candidate.sourceObject ? a_candidate.sourceObject.get() : a_candidate.object;
		return IsPhysicsSourceEntirelyHidden(fallback);
	}

	std::uint32_t CountUnresolvedRequiredPhysicsBones(
		RE::BSFlattenedBoneTree* a_root,
		const std::span<const std::string> a_requiredBoneNames)
	{
		if (!a_root) {
			return static_cast<std::uint32_t>(a_requiredBoneNames.size());
		}
		return static_cast<std::uint32_t>(std::ranges::count_if(a_requiredBoneNames, [a_root](const std::string& a_name) {
			return a_name.empty() || !a_root->GetObjectByName(RE::BSFixedString{ a_name });
		}));
	}

	void ResetBoneReferenceRuntimeState(Smp::ArmorBoneReference& a_reference)
	{
		a_reference.resolvedNode.reset();
		a_reference.parentBoneIsArmorOnly = false;
		a_reference.createdByUs = false;
	}

	void MergeBoneReferenceRecipe(
		std::vector<Smp::ArmorBoneReference>& a_references,
		Smp::ArmorBoneReference a_reference)
	{
		ResetBoneReferenceRuntimeState(a_reference);
		const auto existing = std::ranges::find_if(
			a_references,
			[&](const Smp::ArmorBoneReference& a_current) {
				return Smp::PhysicsNamesEqual(a_current.name, a_reference.name);
			});
		if (existing == a_references.end()) {
			a_references.push_back(std::move(a_reference));
			return;
		}

		existing->isSkinned = existing->isSkinned || a_reference.isSkinned;
		if (existing->parentBoneName.empty() && !a_reference.parentBoneName.empty()) {
			existing->parentBoneName = std::move(a_reference.parentBoneName);
			existing->localToParentBone = a_reference.localToParentBone;
		}
	}

	void MergePhysicsName(std::vector<std::string>& a_names, const std::string_view a_name)
	{
		if (a_name.empty() ||
			std::ranges::any_of(a_names, [&](const std::string& a_existing) {
				return Smp::PhysicsNamesEqual(a_existing, a_name);
			})) {
			return;
		}
		a_names.emplace_back(a_name);
	}

	std::optional<ArmorPhysicsXmlSelection> FindArmorPhysicsXml(RE::NiAVObject* a_object);
	RE::NiPoint3 ResolveWindRayStart(RE::Actor* a_actor);
	bool IsReadableMemory(const void* a_address, std::size_t a_minSize);
	bool IsProbablyValidNiObject(const RE::NiObject* a_object);
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

	bool IsSupportedHeadPartClosureRoot(RE::BGSHeadPart* a_headPart)
	{
		return a_headPart &&
			a_headPart->type.get() != RE::BGSHeadPart::HeadPartType::kMisc;
	}

	Smp::BuildDomain GetHeadPartBuildDomain(RE::BGSHeadPart* a_headPart)
	{
		return a_headPart &&
				a_headPart->type.get() == RE::BGSHeadPart::HeadPartType::kHair ?
			Smp::BuildDomain::kHair :
			Smp::BuildDomain::kHead;
	}

	std::span<RE::BGSHeadPart*> GetRuntimeHeadParts(const RE::TESNPC* a_npc)
	{
		if (!a_npc) {
			return {};
		}

		// The published map relocation points at different fields of the scatter
		// table on OG and AE. Let the engine perform the lookup so the plugin does
		// not depend on either global's address or container base offset.
		const auto getAlternate = [a_npc]() {
			auto** headParts = Smp::Address::TESNPCGetAlternateHeadPartList(a_npc);
			const auto count = Smp::Address::TESNPCGetAlternateHeadPartListSize(a_npc);
			return headParts && count > 0 ?
				std::span<RE::BGSHeadPart*>{ headParts, count } :
				std::span<RE::BGSHeadPart*>{};
		};

		if (a_npc->IsPlayer()) {
			const auto* player = RE::PlayerCharacter::GetSingleton();
			if (player && player->charGenRace && player->charGenRace != a_npc->formRace) {
				if (auto alternate = getAlternate(); !alternate.empty()) {
					return alternate;
				}
			}
		} else if (a_npc->originalRace && a_npc->originalRace != a_npc->formRace) {
			return getAlternate();
		}

		return a_npc->headParts && a_npc->numHeadParts > 0 ?
			std::span<RE::BGSHeadPart*>{ a_npc->headParts, static_cast<std::size_t>(a_npc->numHeadParts) } :
			std::span<RE::BGSHeadPart*>{};
	}

	template <class Visitor>
	void VisitHeadPartClosure(
		RE::BGSHeadPart* a_headPart,
		std::unordered_set<RE::BGSHeadPart*>& a_visited,
		Visitor&& a_visitor,
		const std::unordered_set<RE::BGSHeadPart*>* a_allowedParts = nullptr)
	{
		if (!a_headPart ||
			(a_allowedParts && !a_allowedParts->contains(a_headPart)) ||
			!a_visited.insert(a_headPart).second) {
			return;
		}

		a_visitor(a_headPart);
		for (auto* extraPart : a_headPart->extraParts) {
			VisitHeadPartClosure(extraPart, a_visited, a_visitor, a_allowedParts);
		}
	}

	bool MustHeadPartBeUnique(const RE::BGSHeadPart::HeadPartType a_type)
	{
		const auto value = std::to_underlying(a_type);
		return (value >= 1 && value <= 4) || value == 6 || (value >= 8 && value <= 9);
	}

	std::vector<RE::BGSHeadPart*> CollectMergedRuntimeHeadParts(const RE::TESNPC* a_npc)
	{
		std::vector<RE::BGSHeadPart*> acceptedParts;
		std::unordered_set<RE::BGSHeadPart*> visitedParts;
		std::unordered_set<std::underlying_type_t<RE::BGSHeadPart::HeadPartType>> claimedUniqueTypes;
		const auto mergePart = [&](this const auto& a_self, RE::BGSHeadPart* a_headPart) -> void {
			if (!a_headPart || !visitedParts.insert(a_headPart).second) {
				return;
			}

			const auto type = a_headPart->type.get();
			const auto typeValue = std::to_underlying(type);
			if (MustHeadPartBeUnique(type) && !claimedUniqueTypes.insert(typeValue).second) {
				// Fallout does not merge the rejected part's extraParts either.
				return;
			}

			acceptedParts.push_back(a_headPart);
			for (auto* extraPart : a_headPart->extraParts) {
				a_self(extraPart);
			}
		};

		for (auto* headPart : GetRuntimeHeadParts(a_npc)) {
			mergePart(headPart);
		}
		return acceptedParts;
	}

	std::vector<RE::BGSHeadPart*> CollectRuntimeHeadPartClosureRoots(
		const std::vector<RE::BGSHeadPart*>& a_runtimeHeadParts)
	{
		const std::unordered_set<RE::BGSHeadPart*> acceptedParts(
			a_runtimeHeadParts.begin(),
			a_runtimeHeadParts.end());
		std::unordered_set<RE::BGSHeadPart*> referencedExtraParts;
		for (auto* headPart : a_runtimeHeadParts) {
			for (auto* extraPart : headPart->extraParts) {
				if (acceptedParts.contains(extraPart)) {
					referencedExtraParts.insert(extraPart);
				}
			}
		}

		std::vector<RE::BGSHeadPart*> roots;
		std::unordered_set<RE::BGSHeadPart*> claimedParts;
		const auto claimRoot = [&](RE::BGSHeadPart* a_headPart) {
			if (!IsSupportedHeadPartClosureRoot(a_headPart) || claimedParts.contains(a_headPart)) {
				return;
			}
			roots.push_back(a_headPart);
			VisitHeadPartClosure(a_headPart, claimedParts, [](RE::BGSHeadPart*) {}, std::addressof(acceptedParts));
		};

		for (auto* headPart : a_runtimeHeadParts) {
			if (!referencedExtraParts.contains(headPart)) {
				claimRoot(headPart);
			}
		}
		// Cyclic graphs, or supported parts referenced only by a misc root, have
		// no eligible unreferenced root. Claim the first unowned supported part.
		for (auto* headPart : a_runtimeHeadParts) {
			claimRoot(headPart);
		}
		return roots;
	}

	void AddHeadpartClosureKeys(
		RE::BGSHeadPart* a_headPart,
		const std::unordered_set<RE::BGSHeadPart*>& a_allowedParts,
		std::vector<std::string>& a_keys)
	{
		std::unordered_set<RE::BGSHeadPart*> visited;
		VisitHeadPartClosure(
			a_headPart,
			visited,
			[&](RE::BGSHeadPart* a_part) {
				AddHeadpartKey(a_keys, GetHeadPartModelPath(a_part));
				AddHeadpartKey(a_keys, std::string(std::string_view(a_part->formEditorID)));
			},
			std::addressof(a_allowedParts));
	}

	std::vector<std::string> BuildHairHeadpartKeys(RE::Actor* a_actor)
	{
		std::vector<std::string> keys;
		auto* npc = a_actor ? a_actor->GetNPC() : nullptr;
		if (!npc) {
			return keys;
		}

		const auto runtimeHeadParts = CollectMergedRuntimeHeadParts(npc);
		const std::unordered_set<RE::BGSHeadPart*> allowedParts(
			runtimeHeadParts.begin(),
			runtimeHeadParts.end());
		for (auto* headPart : CollectRuntimeHeadPartClosureRoots(runtimeHeadParts)) {
			if (GetHeadPartBuildDomain(headPart) == Smp::BuildDomain::kHair) {
				AddHeadpartClosureKeys(headPart, allowedParts, keys);
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
			a_type == Smp::LifecycleEventType::kHeadPartVisibilityUpdated ||
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

		if (const auto* main = RE::Main::GetSingleton(); main && (main->inMenuMode || main->freezeTime)) {
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

	void AppendHeadPathVisibilitySource(
		std::vector<HeadPhysicsXmlBuildCandidate::PathVisibilitySources>& a_sources,
		const std::filesystem::path& a_path,
		RE::NiAVObject* a_object)
	{
		if (a_path.empty() || !a_object) {
			return;
		}

		const auto pathKey = Smp::ResourceFile::ComparisonKey(a_path.string());
		auto source = std::ranges::find_if(a_sources, [&](const auto& a_existing) {
			return Smp::ResourceFile::ComparisonKey(a_existing.path.string()) == pathKey;
		});
		if (source == a_sources.end()) {
			a_sources.push_back({ .path = a_path });
			source = std::prev(a_sources.end());
		}
		if (std::ranges::none_of(source->objects, [a_object](const auto& a_existing) {
			return a_existing.get() == a_object;
		})) {
			source->objects.emplace_back(a_object);
		}
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

		const auto normalizedPath = Smp::ResourceFile::ComparisonKey(a_path.string());
		const auto duplicate = std::ranges::any_of(a_candidates, [&](const HeadPhysicsXmlBuildCandidate& a_candidate) {
			return a_candidate.object == a_object &&
				a_candidate.domain == a_domain &&
				std::ranges::any_of(a_candidate.paths, [&](const std::filesystem::path& a_existingPath) {
					return Smp::ResourceFile::ComparisonKey(a_existingPath.string()) == normalizedPath;
				});
		});
		if (duplicate) {
			return;
		}

		std::vector<HeadPhysicsXmlBuildCandidate::PathVisibilitySources> visibilitySources;
		AppendHeadPathVisibilitySource(visibilitySources, a_path, a_sourceObject ? a_sourceObject : a_object);
		a_candidates.push_back({
			.object = a_object,
			.paths = { std::move(a_path) },
			.pathVisibilitySources = std::move(visibilitySources),
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

	struct HeadPartClosureSources
	{
		struct ModelSource
		{
			RE::NiPointer<RE::NiAVObject> root;
			std::string nifPath;
		};

		RE::NiAVObject* object{ nullptr };
		RE::NiNode* destinationRoot{ nullptr };
		Smp::BuildDomain domain{ Smp::BuildDomain::kHead };
		Smp::DefaultBBP::NameMap meshNameMap;
		std::vector<RE::NiPointer<RE::NiAVObject>> meshSourceRoots;
		std::vector<ModelSource> modelSources;
		std::vector<RE::NiPointer<RE::NiAVObject>> liveBindingObjects;
		std::vector<std::filesystem::path> physicsXmlPaths;
		std::vector<HeadPhysicsXmlBuildCandidate::PathVisibilitySources> pathVisibilitySources;
		std::vector<Smp::ArmorBoneReference> boneReferences;
	};

	void AppendHeadPartClosurePhysicsXml(
		std::vector<std::filesystem::path>& a_paths,
		std::vector<HeadPhysicsXmlBuildCandidate::PathVisibilitySources>& a_visibilitySources,
		const std::filesystem::path& a_path,
		RE::NiAVObject* a_liveObject)
	{
		if (a_path.empty()) {
			return;
		}

		const auto key = Smp::ResourceFile::ComparisonKey(a_path.string());
		if (std::ranges::none_of(a_paths, [&](const auto& a_existing) {
			return Smp::ResourceFile::ComparisonKey(a_existing.string()) == key;
		})) {
			a_paths.push_back(a_path);
		}
		AppendHeadPathVisibilitySource(a_visibilitySources, a_path, a_liveObject);
	}

	void MergePhysicsSummary(
		Smp::PhysicsXmlSummary& a_target,
		const Smp::PhysicsXmlSummary& a_source)
	{
		if (a_target.path.empty()) {
			a_target.path = a_source.path;
		}
		a_target.bones += a_source.bones;
		a_target.boneDefaults += a_source.boneDefaults;
		a_target.perVertexShapes += a_source.perVertexShapes;
		a_target.perTriangleShapes += a_source.perTriangleShapes;
		a_target.shapes += a_source.shapes;
		a_target.constraintGroups += a_source.constraintGroups;
		a_target.constraints += a_source.constraints;
		a_target.validSystemRoot = a_target.validSystemRoot || a_source.validSystemRoot;

		for (const auto& boneName : a_source.boneNames) {
			if (boneName.empty()) {
				continue;
			}
			if (std::ranges::none_of(a_target.boneNames, [&](const std::string& a_existing) {
					return Smp::PhysicsNamesEqual(a_existing, boneName);
				})) {
				a_target.boneNames.push_back(boneName);
			}

			const auto explicitDescriptor = std::ranges::find_if(
				a_source.boneDescriptors,
				[&](const Smp::PhysicsBoneDescriptor& a_descriptor) {
					return Smp::PhysicsNamesEqual(a_descriptor.name, boneName);
				});
			const auto alreadyDefined = std::ranges::any_of(
				a_target.boneDescriptors,
				[&](const Smp::PhysicsBoneDescriptor& a_descriptor) {
					return Smp::PhysicsNamesEqual(a_descriptor.name, boneName);
				});
			if (alreadyDefined) {
				continue;
			}
			if (explicitDescriptor != a_source.boneDescriptors.end()) {
				a_target.boneDescriptors.push_back(*explicitDescriptor);
			} else if (a_source.defaultBoneDescriptor) {
				auto descriptor = *a_source.defaultBoneDescriptor;
				descriptor.name = boneName;
				a_target.boneDescriptors.push_back(std::move(descriptor));
			}
		}

		a_target.meshDescriptors.insert(
			a_target.meshDescriptors.end(),
			a_source.meshDescriptors.begin(),
			a_source.meshDescriptors.end());
		a_target.constraintDescriptors.insert(
			a_target.constraintDescriptors.end(),
			a_source.constraintDescriptors.begin(),
			a_source.constraintDescriptors.end());
	}

	void MergeHeadPartClosureBoneReferences(
		std::vector<Smp::ArmorBoneReference>& a_target,
		std::vector<Smp::ArmorBoneReference> a_source)
	{
		for (auto& reference : a_source) {
			const auto key = Smp::ConfigPaths::LowerString(reference.name);
			const auto existing = std::ranges::find_if(a_target, [&](const auto& a_current) {
				return Smp::ConfigPaths::LowerString(a_current.name) == key;
			});
			if (existing == a_target.end()) {
				a_target.push_back(std::move(reference));
			} else {
				existing->isSkinned = existing->isSkinned || reference.isSkinned;
			}
		}
	}

	void AppendLiveHeadPartClosureComponents(
		RE::Actor* a_actor,
		RE::NiAVObject* a_faceObject,
		RE::BGSHeadPart* a_headPart,
		const std::unordered_set<RE::BGSHeadPart*>& a_allowedParts,
		std::unordered_set<RE::BGSHeadPart*>& a_visited,
		HeadPartClosureSources& a_closure)
	{
		if (!a_actor || !a_faceObject || !a_closure.destinationRoot || !a_headPart ||
			!a_allowedParts.contains(a_headPart) || !a_visited.insert(a_headPart).second) {
			return;
		}

		const auto* modelPath = GetHeadPartModelPath(a_headPart);
		if (modelPath && *modelPath) {
			auto meshSourceRoot = Smp::HeadPartModelSourceCache::Find(modelPath);
			if (meshSourceRoot) {
				auto selection = FindHeadPartSourceSelection(meshSourceRoot.get());
				auto* sourceGeometry = selection && selection->geometry ?
					selection->geometry :
					FindFirstGeometry(meshSourceRoot.get());
				const auto sourceGeometryName = sourceGeometry ?
					std::string(std::string_view(sourceGeometry->GetName())) :
					std::string{};
				auto* liveGeometry = ResolveLiveHeadPartGeometry(a_faceObject, a_headPart, sourceGeometryName);
				// extraParts are dependencies of their owning closure. They may provide
				// meshes and bone layout for a sibling's XML without owning XML themselves.
				const auto contributesToClosure = selection.has_value() || a_headPart->IsExtraPart();
				if (liveGeometry && contributesToClosure && !a_closure.object) {
					a_closure.object = liveGeometry;
				}
				if (liveGeometry && contributesToClosure &&
					std::ranges::none_of(a_closure.meshSourceRoots, [&](const auto& a_root) {
						return a_root.get() == meshSourceRoot.get();
					})) {
					a_closure.meshSourceRoots.emplace_back(meshSourceRoot.get());
					a_closure.modelSources.push_back({
						.root = meshSourceRoot.get(),
						.nifPath = modelPath,
					});
				}
				if (liveGeometry && contributesToClosure &&
					std::ranges::none_of(a_closure.liveBindingObjects, [&](const auto& a_object) {
						return a_object.get() == liveGeometry;
					})) {
					a_closure.liveBindingObjects.emplace_back(liveGeometry);
				}

				if (liveGeometry && selection) {
					AppendHeadPartClosurePhysicsXml(
						a_closure.physicsXmlPaths,
						a_closure.pathVisibilitySources,
						selection->path,
						liveGeometry);
				}
				if (!liveGeometry) {
					spdlog::debug(
						"current headpart source has no live FaceGen geometry actor={} headPart={:08X} type={} editorID='{}' nif='{}' sourceGeometry='{}' xml='{}' supplementalMeshSource={}",
						static_cast<void*>(a_actor),
						a_headPart->GetFormID(),
						std::to_underlying(a_headPart->type.get()),
						std::string_view(a_headPart->formEditorID),
						modelPath,
						sourceGeometryName,
						selection ? selection->path.string() : std::string{},
						true);
				} else {
					const auto liveGeometryName = std::string(std::string_view(liveGeometry->GetName()));
					const auto selectedXml = selection ? selection->path.string() : std::string{};
					if (contributesToClosure && !sourceGeometryName.empty() && !liveGeometryName.empty()) {
						a_closure.meshNameMap[sourceGeometryName].emplace(liveGeometryName);
					}
					spdlog::debug(
						"resolved current headpart component actor={} headPart={:08X} type={} domain={} nif='{}' xml='{}' sourceGeometry='{}' liveGeometry={} name='{}'",
						static_cast<void*>(a_actor),
						a_headPart->GetFormID(),
						std::to_underlying(a_headPart->type.get()),
						BuildDomainName(a_closure.domain),
						modelPath,
						selectedXml,
						sourceGeometryName,
						static_cast<void*>(liveGeometry),
						liveGeometryName);
				}
			} else if (a_headPart->IsExtraPart()) {
				spdlog::debug(
					"deferred extraPart head physics source because Fallout has not exposed its loaded NIF yet actor={} headPart={:08X} nif='{}'",
					static_cast<void*>(a_actor),
					a_headPart->GetFormID(),
					modelPath);
			}
		}

		for (auto* extraPart : a_headPart->extraParts) {
			AppendLiveHeadPartClosureComponents(
				a_actor,
				a_faceObject,
				extraPart,
				a_allowedParts,
				a_visited,
				a_closure);
		}
	}

	void CollectLiveHeadPartCandidates(
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

		const auto runtimeHeadParts = CollectMergedRuntimeHeadParts(npc);
		const std::unordered_set<RE::BGSHeadPart*> allowedParts(
			runtimeHeadParts.begin(),
			runtimeHeadParts.end());
		const auto closureRoots = CollectRuntimeHeadPartClosureRoots(runtimeHeadParts);
		std::unordered_set<RE::BGSHeadPart*> visited;
		const auto appendClosure = [&](RE::BGSHeadPart* a_headPart) {
			if (!a_headPart ||
				!IsSupportedHeadPartClosureRoot(a_headPart) ||
				visited.contains(a_headPart)) {
				return;
			}

			HeadPartClosureSources closure{
				.destinationRoot = destinationRoot,
				.domain = GetHeadPartBuildDomain(a_headPart),
			};
			AppendLiveHeadPartClosureComponents(
				a_actor,
				a_faceObject,
				a_headPart,
				allowedParts,
				visited,
				closure);

			if (!closure.object || closure.physicsXmlPaths.empty()) {
				return;
			}

			for (const auto& source : closure.modelSources) {
				auto references = Smp::CaptureArmorBoneReferences(
					source.root.get(),
					closure.destinationRoot,
					source.nifPath);
				MergeHeadPartClosureBoneReferences(closure.boneReferences, std::move(references));
			}

			HeadPhysicsXmlBuildCandidate candidate;
			candidate.object = closure.object;
			candidate.paths = std::move(closure.physicsXmlPaths);
			candidate.pathVisibilitySources = std::move(closure.pathVisibilitySources);
			candidate.meshNameMap = std::move(closure.meshNameMap);
			candidate.meshSourceRoots = std::move(closure.meshSourceRoots);
			candidate.liveBindingObjects = std::move(closure.liveBindingObjects);
			candidate.sourceObject = closure.object;
			candidate.destinationRoot = destinationRoot;
			candidate.boneReferences = std::move(closure.boneReferences);
			candidate.domain = closure.domain;
			candidate.isHeadPartClosure = true;
			a_candidates.push_back(std::move(candidate));
		};

		for (auto* headPart : closureRoots) {
			appendClosure(headPart);
		}
	}

	bool FinalizeLiveHeadPartCandidate(
		RE::Actor* a_actor,
		HeadPhysicsXmlBuildCandidate& a_candidate)
	{
		if (!a_actor || !a_candidate.object || !a_candidate.destinationRoot || a_candidate.boneReferences.empty()) {
			return false;
		}

		return Smp::FinalizeArmorSkinBindings(
			a_actor,
			a_candidate.object,
			a_candidate.destinationRoot.get(),
			false,
			a_candidate.boneReferences,
			a_candidate.liveBindingObjects,
			false);
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
		const std::vector<Smp::ArmorBoneReference>& a_boneReferences,
		std::vector<MatchedSkinBone>& a_matchedBones)
	{
		// Constraint frames need a canonical pose, but system construction must
		// never expose that pose through the actor's live scene graph.
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
		std::unordered_map<std::string, std::int32_t> referenceIndices;
		referenceIndices.reserve(static_cast<std::size_t>(havokBoneCount));
		for (std::int32_t index = 0; index < havokBoneCount; ++index) {
			const auto* boneName = HkStringPtrData(skeleton->bones.data[index].name);
			if (!boneName || *boneName == '\0') {
				continue;
			}
			const auto normalizedName = Smp::NormalizePhysicsName(boneName);
			referenceIndices.insert_or_assign(std::move(normalizedName), index);
		}
		if (referenceIndices.empty()) {
			return false;
		}
		const auto hasHavokReference = [&](const std::string_view a_name) {
			return referenceIndices.contains(Smp::NormalizePhysicsName(a_name));
		};

		// Emulate the non-realized-bone path in
		// BSFlattenedBoneTree::UpdateBoneArray without mutating the live tree. Both
		// OG and AE compose each FlattenedBone::local through its flattened parent
		// index. Mixing same-name hkaSkeleton reference locals into this hierarchy
		// produces a compressed hybrid pose because the two skeleton
		// representations do not share interchangeable local transforms.
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
		std::uint32_t appliedFlattenedLocals = 0;
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
			const auto& local = flattenedBone.local;
			++appliedFlattenedLocals;

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

		std::vector<RE::NiTransform> canonicalHavokWorlds(
			static_cast<std::size_t>(havokBoneCount),
			RE::NiTransform::IDENTITY);
		std::vector<std::uint8_t> havokWorldState(static_cast<std::size_t>(havokBoneCount), 0);
		const auto computeHavokWorld = [&](this auto&& a_self, const std::int32_t a_index, RE::NiTransform& a_world) -> bool {
			if (a_index < 0 || a_index >= havokBoneCount) {
				return false;
			}
			const auto index = static_cast<std::size_t>(a_index);
			if (havokWorldState[index] == 2) {
				a_world = canonicalHavokWorlds[index];
				return true;
			}
			if (havokWorldState[index] == 1) {
				return false;
			}

			havokWorldState[index] = 1;
			RE::NiTransform parentWorld = a_flattened->world;
			const auto parentIndex =
				a_index < skeleton->parentIndices.size ?
					skeleton->parentIndices.data[a_index] :
					static_cast<std::int16_t>(-1);
			if (parentIndex >= 0 && !a_self(parentIndex, parentWorld)) {
				havokWorldState[index] = 0;
				return false;
			}

			a_world = ComposeFo4NiTransform(
				parentWorld,
				ToNiTransform(skeleton->referencePose.data[a_index]));
			canonicalHavokWorlds[index] = a_world;
			havokWorldState[index] = 2;
			return true;
		};
		const auto findHavokWorld = [&](const std::string_view a_name, RE::NiTransform& a_world) {
			const auto found = referenceIndices.find(Smp::NormalizePhysicsName(a_name));
			return found != referenceIndices.end() && computeHavokWorld(found->second, a_world);
		};
		const auto findSharedCanonicalWorld = [&](const std::string_view a_name, RE::NiTransform& a_world) {
			// Keep every shared actor bone in one canonical hierarchy. A complete
			// Havok world is the fallback when FO4 omits the bone from the
			// flattened array; captured NIF transforms are only a compatibility
			// fallback after both actor reference sources fail.
			return findActorWorld(a_name, a_world) || findHavokWorld(a_name, a_world);
		};

		std::unordered_map<std::string, const Smp::ArmorBoneReference*> capturedReferences;
		capturedReferences.reserve(a_boneReferences.size());
		for (const auto& reference : a_boneReferences) {
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

			// Shared bones belong to the actor pose: prefer FO4's flattened
			// hierarchy, then its complete Havok reference hierarchy. The
			// captured transform is only a compatibility fallback for attachment
			// nodes omitted from both representations (notably some twist bones).
			if (!a_reference.isArmorOnly && findSharedCanonicalWorld(a_reference.name, a_world)) {
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
					foundParent = findSharedCanonicalWorld(a_reference.parentBoneName, parentWorld);
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
		std::uint32_t appliedHavokHierarchyFallbackBones = 0;
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
				} else {
					resolved = findHavokWorld(matchedBone.name, canonicalWorld);
					if (resolved) {
						++appliedHavokHierarchyFallbackBones;
					}
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
			"built detached Havok reference pose for system actor={} flattened={} havokBones={} flattenedBones={} appliedFlattenedLocals={} capturedArmorOnlyBones={} actorBones={} havokHierarchyFallbackBones={} capturedSharedFallbackBones={} matchedBones={}/{} requiredBones={} unresolvedRequired={}",
			static_cast<void*>(a_actor),
			static_cast<void*>(a_flattened),
			havokBoneCount,
			flattenedBoneCount,
			appliedFlattenedLocals,
			appliedArmorBones,
			appliedActorBones,
			appliedHavokHierarchyFallbackBones,
			appliedCapturedSharedFallbackBones,
			resolvedMatchedBones,
			a_matchedBones.size(),
			requiredMatchedBones,
			unresolvedRequiredBones.size());
		for (const auto name : unresolvedRequiredBones) {
			spdlog::debug(
				"skipping system physics bone '{}' because it has neither an actor reference transform nor a captured NIF-local attachment hierarchy actor={}",
				name,
				static_cast<void*>(a_actor));
		}
		std::erase_if(a_matchedBones, [](const MatchedSkinBone& a_bone) {
			return !a_bone.hasCanonicalWorld;
		});
		return requiredMatchedBones > unresolvedRequiredBones.size();
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

	void ResolveEnginePreparedBones(
		std::vector<MatchedSkinBone>& a_matchedBones,
		const std::vector<std::string>& a_boneNames,
		RE::NiAVObject* a_actorRoot,
		RE::NiNode* a_skeletonRoot)
	{
		auto* flattened = Smp::NiObject::FindFlattenedBoneTree(a_skeletonRoot);
		if (!flattened) {
			flattened = Smp::NiObject::FindFlattenedBoneTree(a_actorRoot);
		}
		const auto resolveActorBone = [&](const std::string_view a_name) {
			auto* node = Smp::NiObject::ResolveActorSkeletonBoneNode(a_skeletonRoot, a_name);
			return node ? node : Smp::NiObject::ResolveActorSkeletonBoneNode(a_actorRoot, a_name);
		};

		for (auto& matchedBone : a_matchedBones) {
			matchedBone.transform = matchedBone.node ? std::addressof(matchedBone.node->world) : matchedBone.transform;
			if (auto* actorBone = resolveActorBone(matchedBone.name)) {
				matchedBone.node = actorBone;
				matchedBone.transform = std::addressof(actorBone->world);
				matchedBone.isSharedActorBone = true;
				matchedBone.meshOnlySkinBoneCandidate = false;
			} else if (auto* flattenedBone = Smp::NiObject::FindFlattenedBoneByName(flattened, matchedBone.name)) {
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
			auto* node = resolveActorBone(boneName);
			auto* flattenedBone = node ? nullptr : Smp::NiObject::FindFlattenedBoneByName(flattened, boneName);
			if (!node && !flattenedBone) {
				continue;
			}

			if (!node) {
				node = flattenedBone->node.get();
			}
			a_matchedBones.push_back({
				.node = node,
				.transform = node ? std::addressof(node->world) : std::addressof(flattenedBone->world),
				.name = boneName,
				.isSharedActorBone = true,
			});
			spdlog::debug(
				"resolved XML anchor '{}' from the actor's default skeleton node={} transform={}",
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
