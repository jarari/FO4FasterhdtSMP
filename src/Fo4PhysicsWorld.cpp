#include "Fo4PhysicsWorld.h"

#include "BSSkin.h"
#include "ConfigPaths.h"
#include "DefaultBBP.h"
#include "Fo4MeshExtractor.h"
#include "Fo4SkinnedMeshBone.h"
#include "PhysicsName.h"
#include "PhysicsXml.h"
#include "SmpConfig.h"
#include "hdtSkinnedMesh/hdtDispatcher.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"
#include "RE/B/BGSHeadPart.h"
#include "RE/B/BSUtilities.h"
#include "RE/B/BSTimer.h"
#include "RE/N/NiStringExtraData.h"
#include "RE/S/Sky.h"
#include "RE/T/TESNPC.h"
#include "RE/T/TESWeather.h"

#include <btBulletDynamicsCommon.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace
{
	constexpr float kMinimumStepSeconds = 1.0F / 240.0F;
	constexpr float kMinimumShapeExtent = 0.01F;
	constexpr float kPi = 3.14159265358979323846F;
	constexpr std::uint32_t kMaxAttachAncestorScanDepth = 2;
	constexpr std::string_view kPhysicsXmlExtraName = "HDT Skinned Mesh Physics Object";
	using Clock = std::chrono::steady_clock;

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
	};

	struct ArmorPhysicsXmlSelection
	{
		std::filesystem::path path;
		Smp::DefaultBBP::NameMap meshNameMap;
	};

	struct ArmorPhysicsXmlBuildCandidate
	{
		RE::NiAVObject* object{ nullptr };
		ArmorPhysicsXmlSelection selection;
	};

	struct HeadPhysicsXmlBuildCandidate
	{
		RE::NiAVObject* object{ nullptr };
		std::filesystem::path path;
		Smp::PrototypeBuildDomain domain{ Smp::PrototypeBuildDomain::kHead };
	};

	std::optional<ArmorPhysicsXmlSelection> FindArmorPhysicsXml(RE::NiAVObject* a_object);

	float DistanceSquared(const RE::NiPoint3& a_lhs, const RE::NiPoint3& a_rhs)
	{
		const auto dx = a_lhs.x - a_rhs.x;
		const auto dy = a_lhs.y - a_rhs.y;
		const auto dz = a_lhs.z - a_rhs.z;
		return dx * dx + dy * dy + dz * dz;
	}

	float ElapsedMs(const Clock::time_point a_start, const Clock::time_point a_end)
	{
		return std::chrono::duration<float, std::milli>(a_end - a_start).count();
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

		std::uint32_t localRemaining = 96;
		if (!a_remaining) {
			a_remaining = std::addressof(localRemaining);
		}
		if (*a_remaining == 0) {
			return;
		}
		--(*a_remaining);

		auto* geometry = a_object->IsGeometry();
		auto* node = a_object->IsNode();
		spdlog::debug(
			"{} hierarchy depth={} object={} kind={} name='{}'",
			a_label,
			a_depth,
			static_cast<void*>(a_object),
			geometry ? "geometry" : node ? "node" : "object",
			std::string_view(a_object->GetName()));

		if (a_object->extra) {
			for (auto* extra : *a_object->extra) {
				if (auto* stringExtra = netimmerse_cast<RE::NiStringExtraData*>(extra)) {
					spdlog::debug(
						"{} hierarchy depth={} object={} stringExtra name='{}' data='{}'",
						a_label,
						a_depth,
						static_cast<void*>(a_object),
						std::string_view(stringExtra->name),
						std::string_view(stringExtra->data));
				}
			}
		}

		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			LogObjectHierarchy(child.get(), a_label, a_depth + 1, a_remaining);
			if (*a_remaining == 0) {
				spdlog::debug("{} hierarchy truncated after 96 objects", a_label);
				return;
			}
		}
	}

	btVector3 WindDirectionFromGameAngle(const float a_angle)
	{
		const auto theta = ((a_angle * 180.0F / 256.0F) - 90.0F) * (kPi / 180.0F);
		return btVector3(-std::sin(theta), std::cos(theta), 0.0F);
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
		return a_type == Smp::LifecycleEventType::kActorHeadInitialized;
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

	std::optional<std::filesystem::path> FindPhysicsXmlExtraData(RE::NiAVObject* a_object)
	{
		if (!a_object || !a_object->extra) {
			return std::nullopt;
		}

		for (auto* extra : *a_object->extra) {
			auto* stringExtra = netimmerse_cast<RE::NiStringExtraData*>(extra);
			if (!stringExtra) {
				continue;
			}

			const std::string_view name(stringExtra->name);
			const auto data = Smp::ConfigPaths::Trim(std::string(std::string_view(stringExtra->data)));
			if (data.empty()) {
				continue;
			}

			if (!Smp::PhysicsNamesEqual(name, kPhysicsXmlExtraName)) {
				continue;
			}

			if (auto path = Smp::ConfigPaths::ResolveExistingConfigPath(data, true)) {
				return path;
			}
			spdlog::warn("found '{}' NiStringExtraData on object={} but XML path could not be resolved: {}", kPhysicsXmlExtraName, static_cast<void*>(a_object), data);
		}

		return std::nullopt;
	}

	std::optional<std::filesystem::path> FindDirectPhysicsXmlExtraData(RE::NiAVObject* a_object)
	{
		if (!a_object || !a_object->extra) {
			return std::nullopt;
		}

		for (auto* extra : *a_object->extra) {
			auto* stringExtra = netimmerse_cast<RE::NiStringExtraData*>(extra);
			if (!stringExtra) {
				continue;
			}

			const std::string_view name(stringExtra->name);
			const auto data = Smp::ConfigPaths::Trim(std::string(std::string_view(stringExtra->data)));
			if (data.empty()) {
				continue;
			}

			if (!Smp::PhysicsNamesEqual(name, kPhysicsXmlExtraName)) {
				continue;
			}

			if (auto path = Smp::ConfigPaths::ResolveExistingConfigPath(data, true)) {
				return path;
			}
			spdlog::warn("found '{}' NiStringExtraData on object={} but XML path could not be resolved: {}", kPhysicsXmlExtraName, static_cast<void*>(a_object), data);
		}

		return std::nullopt;
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
		if (ancestorDepth >= kMaxAttachAncestorScanDepth) {
			spdlog::warn("stopped armor XML ancestor scan after {} parents for attached object={}", kMaxAttachAncestorScanDepth, static_cast<void*>(a_event.object));
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
			a_candidates.push_back({
				.object = a_object,
				.path = *directXml,
				.domain = isHair ? Smp::PrototypeBuildDomain::kHair : Smp::PrototypeBuildDomain::kHead,
			});
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
		if (auto directXml = FindPhysicsXmlExtraData(a_object)) {
			return ArmorPhysicsXmlSelection{ .path = *directXml };
		}

		if (auto defaultBbp = Smp::DefaultBBP::GetSingleton()->Find(a_object)) {
			return ArmorPhysicsXmlSelection{
				.path = defaultBbp->physicsXml,
				.meshNameMap = std::move(defaultBbp->meshNameMap),
			};
		}

		return std::nullopt;
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
		const auto found = std::ranges::find_if(a_meshNameMap, [a_name](const auto& a_entry) {
			return Smp::PhysicsNamesEqual(a_entry.first, a_name);
		});
		return found == a_meshNameMap.end() ? nullptr : std::addressof(found->second);
	}

	bool MeshNameMatches(const std::string_view a_descriptorName, const std::string_view a_geometryName, const Smp::DefaultBBP::NameMap& a_meshNameMap)
	{
		if (Smp::PhysicsNamesEqual(a_descriptorName, a_geometryName)) {
			return true;
		}

		const auto* aliases = FindMeshAliases(a_meshNameMap, a_descriptorName);
		if (!aliases) {
			return false;
		}

		return std::ranges::any_of(*aliases, [a_geometryName](const std::string& a_alias) {
			return Smp::PhysicsNamesEqual(a_alias, a_geometryName);
		});
	}

	std::vector<std::string> BuildMeshMatchNames(const Smp::PhysicsXmlSummary& a_summary, const Smp::DefaultBBP::NameMap& a_meshNameMap)
	{
		std::vector<std::string> result;
		for (const auto& meshDescriptor : a_summary.meshDescriptors) {
			result.push_back(meshDescriptor.name);
			if (const auto* aliases = FindMeshAliases(a_meshNameMap, meshDescriptor.name)) {
				for (const auto& alias : *aliases) {
					if (std::ranges::find_if(result, [&alias](const std::string& a_existing) {
							return Smp::PhysicsNamesEqual(a_existing, alias);
						}) == result.end()) {
						result.push_back(alias);
					}
				}
			}
		}
		return result;
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

	bool ContainsNode(const std::vector<std::pair<RE::NiNode*, std::string>>& a_nodes, RE::NiNode* a_node)
	{
		return std::ranges::find_if(a_nodes, [a_node](const auto& a_entry) {
			return a_entry.first == a_node;
		}) != a_nodes.end();
	}

	struct MatchedSkinBone
	{
		struct SkinWorldTransformSlot
		{
			RE::NiPointer<RE::BSSkin::Instance> skin;
			std::uint32_t index{ 0 };
		};

		RE::NiNode* node{ nullptr };
		std::string name;
		bool resolvedFromSkeleton{ false };
		std::vector<RE::NiPointer<RE::NiNode>> drivenTargets;
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

	void AddDrivenTarget(MatchedSkinBone& a_bone, RE::NiNode* a_target)
	{
		if (!a_target || a_target == a_bone.node) {
			return;
		}

		const auto found = std::ranges::find_if(a_bone.drivenTargets, [a_target](const RE::NiPointer<RE::NiNode>& a_existing) {
			return a_existing.get() == a_target;
		});
		if (found != a_bone.drivenTargets.end()) {
			return;
		}

		a_bone.drivenTargets.emplace_back(a_target);
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
		});
	}

	RE::NiTransform* ResolveSkinWorldTransform(const MatchedSkinBone::SkinWorldTransformSlot& a_slot)
	{
		if (!a_slot.skin || a_slot.index >= a_slot.skin->worldTransforms.size()) {
			return nullptr;
		}

		return a_slot.skin->worldTransforms[a_slot.index];
	}

	RE::NiNode* FindNodeByName(RE::NiAVObject* a_root, const std::string_view a_name)
	{
		if (!a_root || a_name.empty()) {
			return nullptr;
		}

		auto* object = RE::BSUtilities::GetObjectByName(a_root, RE::BSFixedString(a_name), true, true);
		return object ? object->IsNode() : nullptr;
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

	std::uint32_t AddSkeletonDrivenAncestors(
		Smp::Fo4SkinnedMeshBone& a_bone,
		RE::NiNode* a_node,
		RE::NiAVObject* a_searchRoot,
		RE::NiAVObject* a_stopRoot)
	{
		std::uint32_t added = 0;
		for (auto* parent = a_node ? a_node->parent : nullptr; parent && parent != a_stopRoot; parent = parent->parent) {
			const auto name = parent->GetName();
			if (name.empty()) {
				continue;
			}

			auto* driver = FindNodeByName(a_searchRoot, name);
			if (!driver || driver == parent) {
				continue;
			}

			a_bone.AddDrivenAncestor(parent, driver);
			++added;
		}
		return added;
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

	bool IsKinematicXmlBone(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name)
	{
		if (const auto* descriptor = FindBoneDescriptor(a_summary, a_name)) {
			return descriptor->mass <= 0.0F;
		}

		return a_summary.defaultBoneDescriptor && a_summary.defaultBoneDescriptor->mass <= 0.0F;
	}

	const Smp::PhysicsMeshShapeDescriptor* FindMeshDescriptor(const Smp::PhysicsXmlSummary& a_summary, const std::string_view a_name, const Smp::DefaultBBP::NameMap& a_meshNameMap)
	{
		const auto found = std::ranges::find_if(a_summary.meshDescriptors, [a_name, &a_meshNameMap](const Smp::PhysicsMeshShapeDescriptor& a_descriptor) {
			return MeshNameMatches(a_descriptor.name, a_name, a_meshNameMap);
		});
		return found == a_summary.meshDescriptors.end() ? nullptr : std::addressof(*found);
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

	void ResolveExplicitXmlBonesFromSkeleton(
		std::vector<MatchedSkinBone>& a_matchedBones,
		const std::vector<std::string>& a_boneNames,
		RE::NiAVObject* a_skeletonRoot)
	{
		if (!a_skeletonRoot) {
			return;
		}

		for (const auto& boneName : a_boneNames) {
			if (boneName.empty()) {
				continue;
			}

			auto* skeletonNode = FindNodeByName(a_skeletonRoot, boneName);
			if (!skeletonNode) {
				continue;
			}

			if (auto* existingByNode = FindMatchedSkinBone(a_matchedBones, skeletonNode)) {
				existingByNode->name = boneName;
				existingByNode->resolvedFromSkeleton = true;
				continue;
			}

			if (auto* existingByName = FindMatchedSkinBoneByName(a_matchedBones, boneName)) {
				if (existingByName->node != skeletonNode) {
					spdlog::debug(
						"explicit XML bone '{}' resolved from actor skeleton node={} instead of attached skin node={}",
						boneName,
						static_cast<void*>(skeletonNode),
						static_cast<void*>(existingByName->node));
					AddDrivenTarget(*existingByName, existingByName->node);
					existingByName->node = skeletonNode;
				}
				existingByName->name = boneName;
				existingByName->resolvedFromSkeleton = true;
				continue;
			}

			a_matchedBones.push_back({
				.node = skeletonNode,
				.name = boneName,
				.resolvedFromSkeleton = true,
			});
			spdlog::debug("explicit XML bone '{}' resolved from actor skeleton node={}", boneName, static_cast<void*>(skeletonNode));
		}
	}

	void AttachSkeletonDrivenSkinSlots(
		std::vector<MatchedSkinBone>& a_matchedBones,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_skeletonRoot)
	{
		if (!a_object || !a_skeletonRoot) {
			return;
		}

		if (auto* geometry = a_object->IsGeometry()) {
			auto* skin = geometry->skinInstance ? geometry->skinInstance.get() : nullptr;
			if (!skin || skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				return;
			}

			for (std::uint32_t index = 0; index < skin->bones.size(); ++index) {
				auto* skinBoneObject = skin->bones[index];
				auto* skinBone = skinBoneObject ? skinBoneObject->IsNode() : nullptr;
				if (!skinBone) {
					continue;
				}

				const auto name = skinBoneObject->GetName();
				if (name.empty()) {
					continue;
				}

				auto* matched = FindMatchedSkinBoneByName(a_matchedBones, name);
				if (!matched || !matched->resolvedFromSkeleton) {
					continue;
				}

			if (auto* skeletonNode = FindNodeByName(a_skeletonRoot, name); skeletonNode && skeletonNode == matched->node) {
					AddSkinWorldTransformSlot(*matched, skin, index);
					if (skinBone != skeletonNode) {
						AddDrivenTarget(*matched, skinBone);
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
			AttachSkeletonDrivenSkinSlots(a_matchedBones, child.get(), a_skeletonRoot);
		}
	}

	void AttachExplicitSkeletonDrivenSourceNodes(
		std::vector<MatchedSkinBone>& a_matchedBones,
		const Smp::LifecycleEvent& a_event,
		const Smp::PhysicsXmlSummary& a_summary)
	{
		for (auto& matched : a_matchedBones) {
			if (!matched.resolvedFromSkeleton || !matched.node || matched.name.empty()) {
				continue;
			}
			if (!IsKinematicXmlBone(a_summary, matched.name)) {
				continue;
			}

			for (auto* root : { a_event.sourceObject, static_cast<RE::NiAVObject*>(a_event.sourceRoot) }) {
				if (root && root->IsNode() && root->IsNode()->children.empty()) {
					continue;
				}
				auto* sourceNode = FindNodeByName(root, matched.name);
				if (!sourceNode || sourceNode == matched.node) {
					continue;
				}

				AddDrivenTarget(matched, sourceNode);
				spdlog::debug(
					"explicit XML bone '{}' will drive same-name source node={} from actor skeleton node={}",
					matched.name,
					static_cast<void*>(sourceNode),
					static_cast<void*>(matched.node));
			}

			if (matched.drivenTargets.empty()) {
				spdlog::debug(
					"explicit XML kinematic bone '{}' has no cloned same-name source node to drive sourceObject={} sourceRoot={} attachedObject={}",
					matched.name,
					static_cast<void*>(a_event.sourceObject),
					static_cast<void*>(a_event.sourceRoot),
					static_cast<void*>(a_event.object));
				if (a_event.sourceObject && a_event.sourceObject->IsNode() && !a_event.sourceObject->IsNode()->children.empty()) {
					LogObjectHierarchy(a_event.sourceObject, "xml-source-root");
				}
			}
		}
	}

	btTransform ToBulletTransform(const RE::NiTransform& a_transform)
	{
		const btMatrix3x3 basis(
			a_transform.rotate[0].x,
			a_transform.rotate[0].y,
			a_transform.rotate[0].z,
			a_transform.rotate[1].x,
			a_transform.rotate[1].y,
			a_transform.rotate[1].z,
			a_transform.rotate[2].x,
			a_transform.rotate[2].y,
			a_transform.rotate[2].z);

		return btTransform(basis, btVector3(a_transform.translate.x, a_transform.translate.y, a_transform.translate.z));
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

	RE::NiTransform ToNiTransform(const btTransform& a_transform, const float a_scale)
	{
		const auto& basis = a_transform.getBasis();
		RE::NiTransform result;
		result.rotate = RE::NiMatrix3(
			basis[0].x(), basis[0].y(), basis[0].z(), 0.0F,
			basis[1].x(), basis[1].y(), basis[1].z(), 0.0F,
			basis[2].x(), basis[2].y(), basis[2].z(), 0.0F);
		const auto origin = a_transform.getOrigin();
		result.translate = RE::NiPoint3(origin.x(), origin.y(), origin.z());
		result.scale = a_scale;
		return result;
	}

	void LogPrototypeTransformDiagnostic(
		const std::string_view a_phase,
		RE::Actor* a_actor,
		const std::string& a_boneName,
		RE::NiNode* a_node,
		const btRigidBody& a_body)
	{
		const auto bodyTransform = a_body.getWorldTransform();
		const auto bodyOrigin = bodyTransform.getOrigin();
		const auto nodeTranslate = a_node ? a_node->world.translate : RE::NiPoint3{};
		const auto nodeScale = a_node ? a_node->world.scale : 0.0F;
		spdlog::info(
			"prototype bone diagnostic {} actor={} bone='{}' node={} nodeWorld=({:.3f},{:.3f},{:.3f}) nodeScale={:.3f} bodyWorld=({:.3f},{:.3f},{:.3f}) linearVel=({:.3f},{:.3f},{:.3f})",
			a_phase,
			static_cast<void*>(a_actor),
			a_boneName,
			static_cast<void*>(a_node),
			nodeTranslate.x,
			nodeTranslate.y,
			nodeTranslate.z,
			nodeScale,
			bodyOrigin.x(),
			bodyOrigin.y(),
			bodyOrigin.z(),
			a_body.getLinearVelocity().x(),
			a_body.getLinearVelocity().y(),
			a_body.getLinearVelocity().z());
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

	bool HasAnyNonZero(const Smp::XmlVector3& a_value)
	{
		return std::abs(a_value.x) > FLT_EPSILON ||
			std::abs(a_value.y) > FLT_EPSILON ||
			std::abs(a_value.z) > FLT_EPSILON;
	}

	bool HasNonHookeanTerms(const Smp::PhysicsConstraintDescriptor& a_descriptor)
	{
		return HasAnyNonZero(a_descriptor.linearNonHookeanDamping) ||
			HasAnyNonZero(a_descriptor.angularNonHookeanDamping) ||
			HasAnyNonZero(a_descriptor.linearNonHookeanStiffness) ||
			HasAnyNonZero(a_descriptor.angularNonHookeanStiffness);
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

	std::pair<btTransform, btTransform> CalculateConstraintFrames(
		const Smp::PhysicsConstraintDescriptor& a_descriptor,
		const btTransform& a_transformA,
		const btTransform& a_transformB)
	{
		btTransform frameA = btTransform::getIdentity();
		btTransform frameB = btTransform::getIdentity();
		const auto frame = ToBulletTransform(a_descriptor.frame);

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
			auto oldAxis = a_transformA.getBasis().getColumn(axis);
			auto axisToB = a_transformB.getOrigin() - a_transformA.getOrigin();
			if (!oldAxis.fuzzyZero() && !axisToB.fuzzyZero()) {
				oldAxis.normalize();
				axisToB.normalize();
				const auto rotation = shortestArcQuat(oldAxis, axisToB);
				frameWorld.setBasis(frameWorld.getBasis() * btMatrix3x3(rotation));
			}
			frameA = a_transformA.inverse() * frameWorld;
			frameB = a_transformB.inverse() * frameWorld;
			break;
		}
		case Smp::PhysicsFrameMode::kFrameInA:
		{
			frameA = frame;
			const auto frameWorld = a_transformA * frameA;
			frameB = a_transformB.inverse() * frameWorld;
			break;
		}
		case Smp::PhysicsFrameMode::kFrameInLerp:
		{
			const auto translationLerp = std::clamp(a_descriptor.translationLerp, 0.0F, 1.0F);
			const auto rotationLerp = std::clamp(a_descriptor.rotationLerp, 0.0F, 1.0F);
			const auto origin = a_transformA.getOrigin().lerp(a_transformB.getOrigin(), translationLerp);
			const auto rotation = a_transformA.getRotation().slerp(a_transformB.getRotation(), rotationLerp);
			const btTransform frameWorld(rotation, origin);
			frameA = a_transformA.inverse() * frameWorld;
			frameB = a_transformB.inverse() * frameWorld;
			break;
		}
		case Smp::PhysicsFrameMode::kFrameInB:
		default:
		{
			frameB = frame;
			const auto frameWorld = a_transformB * frameB;
			frameA = a_transformA.inverse() * frameWorld;
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
		const btTransform& a_rigToLocalB)
	{
		const auto nodeTransformA = a_bodyA.getWorldTransform() * a_rigToLocalA;
		const auto nodeTransformB = a_bodyB.getWorldTransform() * a_rigToLocalB;
		const auto [nodeFrameA, nodeFrameB] = CalculateConstraintFrames(a_descriptor, nodeTransformA, nodeTransformB);
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
			if (HasNonHookeanTerms(a_descriptor)) {
				spdlog::warn(
					"prototype generic constraint '{}' uses non-Hookean fields; current FO4 Bullet constraint path parses them but cannot apply them yet",
					a_descriptor.name);
			}
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
			}
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
			GetLifecycleEventSource().RegisterSink(this);
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

	void Fo4PhysicsWorld::StepFrame()
	{
		auto delta = fixedStepSeconds_;
		if (const auto timer = RE::BSTimer::GetSingleton()) {
			delta = useRealTime_ ? timer->realTimeDelta : timer->delta;
		}
		if (delta <= 0.0F || !std::isfinite(delta)) {
			delta = fixedStepSeconds_;
		}
		const auto start = Clock::now();
		Step(delta);
		RecordFrameMetrics(ElapsedMs(start, Clock::now()));
	}

	void Fo4PhysicsWorld::Step(const float a_deltaSeconds)
	{
		std::scoped_lock lock(lock_);
		if (!dynamicsWorld_ || a_deltaSeconds <= 0.0F) {
			if (diagnosticFrameBudget_ > 0) {
				spdlog::info(
					"prototype physics step skipped dynamicsWorld={} delta={}",
					static_cast<void*>(dynamicsWorld_.get()),
					a_deltaSeconds);
			}
			return;
		}

		PruneInvalidPrototypeStatesLocked();
		TryReactivateSuspendedActorsLocked();
		if (diagnosticFrameBudget_ > 0 && prototypeActors_.empty()) {
			spdlog::info("prototype physics step has no active actors after pruning");
		}
		UpdateWindLocked();
		ApplyWindForcesLocked();

		for (auto& actorState : prototypeActors_) {
			UpdateMeshDisableStatesLocked(actorState);
			const auto readDelta = actorState.resetReadFrames > 0 ? 0.0F : a_deltaSeconds;
			bool loggedReadSample = false;
			for (auto& prototypeBody : actorState.bodies) {
				if (prototypeBody.bone) {
					const bool logThisBody = diagnosticFrameBudget_ > 0 && !loggedReadSample && !prototypeBody.bone->m_rig.isStaticOrKinematicObject();
					if (logThisBody) {
						LogPrototypeTransformDiagnostic(
							"pre-read",
							actorState.actor,
							prototypeBody.boneName,
							prototypeBody.node,
							prototypeBody.bone->m_rig);
					}
					prototypeBody.bone->readTransform(readDelta);
					if (logThisBody) {
						LogPrototypeTransformDiagnostic(
							"post-read",
							actorState.actor,
							prototypeBody.boneName,
							prototypeBody.node,
							prototypeBody.bone->m_rig);
						loggedReadSample = true;
					}
				}
			}
			if (actorState.resetReadFrames > 0) {
				--actorState.resetReadFrames;
			}
			ScalePrototypeConstraintsLocked(actorState);
			for (auto& prototypeMesh : actorState.meshes) {
				if (prototypeMesh.body) {
					prototypeMesh.body->updateBoundingSphereAabb();
				}
			}
		}

		const auto maximumStepSeconds = std::max(fixedStepSeconds_, fixedStepSeconds_ * static_cast<float>(std::max(maxSubSteps_, 1)));
		const auto clampedDelta = std::clamp(a_deltaSeconds, kMinimumStepSeconds, maximumStepSeconds);
		for (const auto& actorState : prototypeActors_) {
			if (diagnosticFrameBudget_ > 115) {
				LogRootConstraintDiagnosticsLocked("pre-step", actorState);
			}
		}

		const auto translationOffset = ApplyTranslationOffset(*dynamicsWorld_);
		if (auto* world = static_cast<PrototypeDynamicsWorld*>(dynamicsWorld_.get())) {
			world->StepReference(clampedDelta, fixedStepSeconds_);
		}
		RestoreTranslationOffset(*dynamicsWorld_, translationOffset);
		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}
		for (auto& actorState : prototypeActors_) {
			if (diagnosticFrameBudget_ > 115) {
				LogRootConstraintDiagnosticsLocked("post-step", actorState);
			}
			bool loggedPostStep = false;
			for (auto& prototypeBody : actorState.bodies) {
				if (prototypeBody.bone) {
					prototypeBody.bone->internalUpdate();
					if (diagnosticFrameBudget_ > 0 && !loggedPostStep && !prototypeBody.bone->m_rig.isStaticOrKinematicObject()) {
						LogPrototypeTransformDiagnostic(
							"post-step",
							actorState.actor,
							prototypeBody.boneName,
							prototypeBody.node,
							prototypeBody.bone->m_rig);
						loggedPostStep = true;
					}
				}
			}
			for (auto& prototypeMesh : actorState.meshes) {
				if (prototypeMesh.body) {
					prototypeMesh.body->internalUpdate();
				}
			}
		}

		++simulationFrame_;
		if (simulationFrame_ == 0) {
			simulationFrame_ = 1;
		}
		if (diagnosticFrameBudget_ > 0) {
			if (diagnosticFrameBudget_ == 120) {
				spdlog::info(
					"prototype physics step entered delta={} activeActors={} dynamicsWorld={}",
					a_deltaSeconds,
					prototypeActors_.size(),
					static_cast<void*>(dynamicsWorld_.get()));
			}
			--diagnosticFrameBudget_;
			if ((diagnosticFrameBudget_ % 30) == 0) {
				std::size_t dynamicBodies = 0;
				std::size_t kinematicBodies = 0;
				std::size_t constraints = 0;
				for (const auto& actorState : prototypeActors_) {
					constraints += actorState.constraints.size();
					for (const auto& prototypeBody : actorState.bodies) {
						if (!prototypeBody.bone) {
							continue;
						}
						if (prototypeBody.bone->m_rig.isStaticOrKinematicObject()) {
							++kinematicBodies;
						} else {
							++dynamicBodies;
						}
					}
				}
				spdlog::info(
					"prototype physics step diagnostic frame={} dynamicBodies={} kinematicBodies={} constraints={}",
					simulationFrame_,
					dynamicBodies,
					kinematicBodies,
					constraints);
			}
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

		for (auto& prototypeMesh : a_state.meshes) {
			auto* body = prototypeMesh.body.get();
			if (!body) {
				continue;
			}

			body->disabled_ = false;
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
		currentWind_.setZero();
		if (!windEnabled_ || windStrength_ <= 0.0F) {
			return;
		}

		auto direction = windDirection_;
		auto strength = windStrength_;
		if (windUseWeather_) {
			const auto* sky = RE::Sky::GetSingleton();
			if (!sky) {
				return;
			}

			auto angle = sky->windAngle;
			if (sky->currentWeather) {
				const auto index = static_cast<std::size_t>(RE::TESWeather::WeatherData::kWindDir);
				angle = static_cast<float>(sky->currentWeather->weatherData[index]);
			}

			direction = WindDirectionFromGameAngle(angle);
			strength *= std::max(sky->windSpeed, 0.0F);
		}

		currentWind_ = direction * strength;
	}

	void Fo4PhysicsWorld::ApplyWindForcesLocked()
	{
		if (currentWind_.length2() <= SIMD_EPSILON) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		for (auto& actorState : prototypeActors_) {
			float actorWindScale = 1.0F;
			if (player && actorState.actor) {
				const auto distance = std::sqrt(DistanceSquared(actorState.actor->GetPosition(), player->GetPosition()));
				if (distance <= windDistanceForNoWind_) {
					actorWindScale = 0.0F;
				} else if (windDistanceForMaxWind_ > windDistanceForNoWind_ && distance < windDistanceForMaxWind_) {
					actorWindScale = (distance - windDistanceForNoWind_) / (windDistanceForMaxWind_ - windDistanceForNoWind_);
				}
			}
			if (actorWindScale <= 0.0F) {
				continue;
			}

			const auto actorWind = currentWind_ * actorWindScale;
			for (auto& prototypeBody : actorState.bodies) {
				if (!prototypeBody.bone || prototypeBody.bone->m_windFactor <= 0.0F || prototypeBody.bone->m_rig.isStaticOrKinematicObject()) {
					continue;
				}

				prototypeBody.bone->m_rig.applyCentralForce(actorWind * prototypeBody.bone->m_windFactor);
			}
		}
	}

	void Fo4PhysicsWorld::RecordFrameMetrics(const float a_stepMs)
	{
		std::scoped_lock lock(lock_);
		if (prototypeActors_.empty()) {
			pendingWritebackMs_ = 0.0F;
			currentMaxActiveActors_ = maxActiveActors_;
			metricFrameCounter_ = 0;
			averageStepMs_ = 0.0F;
			averageWritebackMs_ = 0.0F;
			return;
		}

		const auto sampleWeight = static_cast<float>(sampleSize_);
		averageStepMs_ = ((averageStepMs_ * (sampleWeight - 1.0F)) + std::max(a_stepMs, 0.0F)) / sampleWeight;
		averageWritebackMs_ = ((averageWritebackMs_ * (sampleWeight - 1.0F)) + std::max(pendingWritebackMs_, 0.0F)) / sampleWeight;
		pendingWritebackMs_ = 0.0F;

		++metricFrameCounter_;
		if (metricFrameCounter_ < metricFrameInterval_) {
			return;
		}
		metricFrameCounter_ = 0;

		std::size_t bodyCount = 0;
		std::size_t meshCount = 0;
		for (const auto& actorState : prototypeActors_) {
			bodyCount += actorState.bodies.size();
			meshCount += actorState.meshes.size();
		}

		const auto totalMs = averageStepMs_ + averageWritebackMs_;
		if (autoAdjustMaxActors_) {
			if (totalMs > budgetMs_ && currentMaxActiveActors_ > 1) {
				--currentMaxActiveActors_;
			} else if (totalMs < budgetMs_ && currentMaxActiveActors_ < maxActiveActors_) {
				const auto averagePerActor = prototypeActors_.empty() ? 0.0F : totalMs / static_cast<float>(prototypeActors_.size());
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
				"[SMP Metrics] activeActors={} actorCap={}/{} bodies={} meshes={} avgFrameImpact={:.3f}ms budget={:.3f}ms step={:.3f}ms writeback={:.3f}ms writes(cellJobs/postAnim)={}/{} duplicateSkips(cellJobs/postAnim)={}/{}",
				prototypeActors_.size(),
				currentMaxActiveActors_,
				maxActiveActors_,
				bodyCount,
				meshCount,
				totalMs,
				budgetMs_,
				averageStepMs_,
				averageWritebackMs_,
				cellJobsWritebacks_,
				postAnimationWritebacks_,
				duplicateCellJobsWritebacks_,
				duplicatePostAnimationWritebacks_);
		} else {
			spdlog::trace(
				"[SMP Metrics] activeActors={} actorCap={}/{} bodies={} meshes={} avgFrameImpact={:.3f}ms budget={:.3f}ms step={:.3f}ms writeback={:.3f}ms writes(cellJobs/postAnim)={}/{} duplicateSkips(cellJobs/postAnim)={}/{}",
				prototypeActors_.size(),
				currentMaxActiveActors_,
				maxActiveActors_,
				bodyCount,
				meshCount,
				totalMs,
				budgetMs_,
				averageStepMs_,
				averageWritebackMs_,
				cellJobsWritebacks_,
				postAnimationWritebacks_,
				duplicateCellJobsWritebacks_,
				duplicatePostAnimationWritebacks_);
		}

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
			PruneInvalidPrototypeStatesLocked();

			for (auto& actorState : prototypeActors_) {
				if (!CanWriteBackFrame(actorState.lastWritebackFrame, a_source, simulationFrame_)) {
					skippedDuplicate = true;
					if (diagnosticFrameBudget_ > 0) {
						spdlog::info(
							"prototype writeback skipped duplicate source={} frame={} actor={} previousSource={}",
							WritebackSourceName(a_source),
							simulationFrame_,
							static_cast<void*>(actorState.actor),
							WritebackSourceName(actorState.lastWritebackSource));
					}
					continue;
				}
				actorState.lastWritebackFrame = simulationFrame_;
				actorState.lastWritebackSource = a_source;
				bool loggedWriteback = false;
				for (auto& prototypeBody : actorState.bodies) {
					if (!prototypeBody.bone || prototypeBody.bone->m_rig.isKinematicObject()) {
						continue;
					}

					const bool logThisBody = diagnosticFrameBudget_ > 0 && !loggedWriteback;
					if (logThisBody) {
						const auto phase = WritebackPhaseName("pre-writeback", a_source);
						LogPrototypeTransformDiagnostic(
							phase,
							actorState.actor,
							prototypeBody.boneName,
							prototypeBody.node,
							prototypeBody.bone->m_rig);
					}
					prototypeBody.bone->writeTransform();
					if (logThisBody) {
						const auto phase = WritebackPhaseName("post-writeback", a_source);
						LogPrototypeTransformDiagnostic(
							phase,
							actorState.actor,
							prototypeBody.boneName,
							prototypeBody.node,
							prototypeBody.bone->m_rig);
						loggedWriteback = true;
					}
					wroteAny = true;
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
			PruneInvalidPrototypeStatesLocked();

			if (auto* actorState = FindPrototypeStateLocked(a_actor)) {
				if (CanWriteBackFrame(actorState->lastWritebackFrame, a_source, simulationFrame_)) {
					actorState->lastWritebackFrame = simulationFrame_;
					actorState->lastWritebackSource = a_source;
					bool loggedWriteback = false;
					for (auto& prototypeBody : actorState->bodies) {
						if (!prototypeBody.bone || prototypeBody.bone->m_rig.isKinematicObject()) {
							continue;
						}

						const bool logThisBody = diagnosticFrameBudget_ > 0 && !loggedWriteback;
						if (logThisBody) {
							const auto phase = WritebackPhaseName("pre-writeback", a_source);
							LogPrototypeTransformDiagnostic(
								phase,
								actorState->actor,
								prototypeBody.boneName,
								prototypeBody.node,
								prototypeBody.bone->m_rig);
						}
						prototypeBody.bone->writeTransform();
						if (logThisBody) {
							const auto phase = WritebackPhaseName("post-writeback", a_source);
							LogPrototypeTransformDiagnostic(
								phase,
								actorState->actor,
								prototypeBody.boneName,
								prototypeBody.node,
								prototypeBody.bone->m_rig);
							loggedWriteback = true;
						}
						wroteAny = true;
					}
				} else {
					skippedDuplicate = true;
					if (diagnosticFrameBudget_ > 0) {
						spdlog::info(
							"prototype writeback skipped duplicate source={} frame={} actor={} previousSource={}",
							WritebackSourceName(a_source),
							simulationFrame_,
							static_cast<void*>(actorState->actor),
							WritebackSourceName(actorState->lastWritebackSource));
					}
				}
			}
		}
		RecordWritebackMetric(ElapsedMs(start, Clock::now()), a_source, wroteAny, skippedDuplicate);
	}

	RE::BSEventNotifyControl Fo4PhysicsWorld::ProcessEvent(const LifecycleEvent& a_event, RE::BSTEventSource<LifecycleEvent>*)
	{
		if (IsAttachCandidate(a_event.type)) {
			if (a_event.type == LifecycleEventType::kActorSet3D && !a_event.object) {
				std::scoped_lock lock(lock_);
				if (auto* actorState = FindPrototypeStateLocked(a_event.actor)) {
					ClearPrototypeStateLocked(*actorState);
					std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
						return a_state.bodies.empty() && a_state.meshes.empty() && a_state.constraints.empty();
					});
					spdlog::debug("physics world cleared actor state for null Set3D actor={}", static_cast<void*>(a_event.actor));
				}
				return RE::BSEventNotifyControl::kContinue;
			}
			NoteLifecycleCandidate(a_event);
		} else if (IsArmorDetachCandidate(a_event.type)) {
			if (!a_event.actor || (!a_event.object && a_event.type == LifecycleEventType::kArmorDetachBegin)) {
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
			if (auto* actorState = FindPrototypeStateLocked(a_event.actor)) {
				const auto clearedGroups = ClearPrototypeGroupsForObjectLocked(*actorState, a_event.object);
				if (!clearedGroups && a_event.type == LifecycleEventType::kArmorDetachBegin) {
					spdlog::debug(
						"ignored armor detach candidate {} actor={} object={} because it did not match an active prototype group",
						ToString(a_event.type),
						static_cast<void*>(a_event.actor),
						static_cast<void*>(a_event.object));
				}
				std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
					return a_state.bodies.empty() && a_state.meshes.empty() && a_state.constraints.empty();
				});
			}
			spdlog::debug(
				"physics world observed armor detach candidate {} actor={} object={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
		} else if (IsResetCandidate(a_event.type)) {
			std::scoped_lock lock(lock_);
			if (auto* actorState = FindPrototypeStateLocked(a_event.actor)) {
				ClearPrototypeStateLocked(*actorState);
				std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
					return a_state.bodies.empty() && a_state.meshes.empty() && a_state.constraints.empty();
				});
			}
			PruneInvalidPrototypeStatesLocked();
			if (InitializeLocked() && IsPrototypeCandidateLocked(a_event, true)) {
				BuildPrototypeForEventLocked(a_event);
			}
			spdlog::debug(
				"physics world observed rebuild/reset candidate {} actor={} object={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
		} else if (a_event.type == LifecycleEventType::kActorUpdate3DModel) {
			spdlog::trace(
				"physics world observed per-frame update candidate {} actor={} object={}; ignored as rebuild trigger",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
		} else if (IsHeadCandidate(a_event.type)) {
			std::scoped_lock lock(lock_);
			PruneInvalidPrototypeStatesLocked();
			if (InitializeLocked() && IsPrototypeCandidateLocked(a_event, false)) {
				BuildHeadPrototypeForEventLocked(a_event);
			}
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
		dynamicsWorld_->setGravity(btVector3(0.0F, 0.0F, -9.80665F));
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
		candidateEvents_ = 0;
		simulationFrame_ = 1;
		currentMaxActiveActors_ = maxActiveActors_;
		metricFrameCounter_ = 0;
		averageStepMs_ = 0.0F;
		averageWritebackMs_ = 0.0F;
		pendingWritebackMs_ = 0.0F;
		cellJobsWritebacks_ = 0;
		postAnimationWritebacks_ = 0;
		duplicateCellJobsWritebacks_ = 0;
		duplicatePostAnimationWritebacks_ = 0;
		diagnosticFrameBudget_ = 0;
	}

	void Fo4PhysicsWorld::NoteLifecycleCandidate(const LifecycleEvent& a_event)
	{
		std::scoped_lock lock(lock_);
		if (!InitializeLocked()) {
			return;
		}
		PruneInvalidPrototypeStatesLocked();

		++candidateEvents_;
		spdlog::debug(
			"physics world observed attach candidate #{} {} actor={} object={}",
			candidateEvents_,
			ToString(a_event.type),
			static_cast<void*>(a_event.actor),
			static_cast<void*>(a_event.object));

		if (!IsPrototypeCandidateLocked(a_event, true)) {
			return;
		}

		BuildPrototypeForEventLocked(a_event);
	}

	void Fo4PhysicsWorld::BuildPrototypeForEventLocked(const LifecycleEvent& a_event)
	{
		auto* loader = PhysicsXmlLoader::GetSingleton();
		const auto armorAttach = IsArmorAttachCandidate(a_event.type);

		const auto buildSelection = [&](RE::NiAVObject* a_object, const ArmorPhysicsXmlSelection& a_selection) {
			const auto selectedXml = a_selection.path.string();
			if (selectedXml.empty()) {
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
			auto& actorState = GetOrCreatePrototypeStateLocked(a_event.actor);
			BuildPrototypeBodiesLocked(actorState, scopedEvent, *selectedSummary, a_selection.meshNameMap, PrototypeBuildDomain::kArmor);
			return true;
		};

		if (!armorAttach) {
			std::vector<ArmorPhysicsXmlBuildCandidate> candidates;
			CollectDirectArmorPhysicsXmlSelections(a_event.object, candidates);
			if (!candidates.empty()) {
				if (auto* actorState = FindPrototypeStateLocked(a_event.actor)) {
					ClearPrototypeStateLocked(*actorState);
					std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
						return a_state.bodies.empty() && a_state.meshes.empty() && a_state.constraints.empty();
					});
				}

				std::uint32_t built = 0;
				for (const auto& candidate : candidates) {
					if (buildSelection(candidate.object, candidate.selection)) {
						++built;
					}
				}
				spdlog::debug(
					"rebuilt prototype physics for {} directly XML-tagged armor subtrees actor={} root={} built={}",
					candidates.size(),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object),
					built);
				std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
					return a_state.bodies.empty() && a_state.meshes.empty() && a_state.constraints.empty();
				});
				return;
			}
		}

		const auto discoveredXml = FindArmorPhysicsXml(a_event);
		const auto selectedXml = discoveredXml ? discoveredXml->path.string() : prototypePhysicsXml_;
		if (selectedXml.empty()) {
			if (armorAttach) {
			spdlog::debug(
				"armor attach candidate has no direct XML/defaultBBP match actor={} object={} sourceObject={} sourceRoot={} destinationRoot={} bipObject={} model='{}' armorAddon={} preScannedXml='{}'; dumping hierarchy",
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				static_cast<void*>(a_event.sourceObject),
				static_cast<void*>(a_event.sourceRoot),
				static_cast<void*>(a_event.destinationRoot),
				static_cast<void*>(a_event.bipObject),
				(a_event.bipObject && a_event.bipObject->part) ? a_event.bipObject->part->GetModel() : "",
				static_cast<void*>(a_event.bipObject ? a_event.bipObject->armorAddon : nullptr),
				a_event.physicsXmlPath);
				LogObjectHierarchy(a_event.object, "attached-object");
				LogObjectHierarchy(a_event.sourceObject, "source-object");
				LogObjectHierarchy(a_event.sourceRoot, "source-root");
				LogObjectHierarchy(a_event.destinationRoot, "destination-root");
			}
			if (loader->HasPrototype()) {
				loader->LoadPrototype({});
			}
			return;
		}

		if (!armorAttach) {
			if (auto* actorState = FindPrototypeStateLocked(a_event.actor)) {
				ClearPrototypeStateLocked(*actorState);
				std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
					return a_state.bodies.empty() && a_state.meshes.empty() && a_state.constraints.empty();
				});
			}
		}

		const Smp::DefaultBBP::NameMap emptyMeshNameMap;
		buildSelection(a_event.object, ArmorPhysicsXmlSelection{
			.path = selectedXml,
			.meshNameMap = discoveredXml ? discoveredXml->meshNameMap : emptyMeshNameMap,
		});
		std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return a_state.bodies.empty() && a_state.meshes.empty() && a_state.constraints.empty();
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

		auto& actorState = GetOrCreatePrototypeStateLocked(a_event.actor);
		ClearPrototypeGroupsByDomainLocked(actorState, PrototypeBuildDomain::kHead);
		ClearPrototypeGroupsByDomainLocked(actorState, PrototypeBuildDomain::kHair);

		const auto hairKeys = BuildHairHeadpartKeys(a_event.actor);
		std::vector<HeadPhysicsXmlBuildCandidate> candidates;
		CollectHeadPhysicsXmlSelections(faceObject, hairKeys, candidates);
		if (candidates.empty()) {
			spdlog::debug(
				"head physics candidate {} actor={} faceNode={} found no direct XML-tagged head/hair subtrees hairKeys={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(faceNode),
				hairKeys.size());
			std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return a_state.bodies.empty() && a_state.meshes.empty() && a_state.constraints.empty();
			});
			return;
		}

		auto* loader = PhysicsXmlLoader::GetSingleton();
		const Smp::DefaultBBP::NameMap emptyMeshNameMap;
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
			BuildPrototypeBodiesLocked(actorState, scopedEvent, *selectedSummary, emptyMeshNameMap, candidate.domain);
			++built;
		}

		spdlog::debug(
			"processed head physics candidate actor={} faceNode={} candidates={} built={} hairKeys={}",
			static_cast<void*>(a_event.actor),
			static_cast<void*>(faceNode),
			candidates.size(),
			built,
			hairKeys.size());
		std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return a_state.bodies.empty() && a_state.meshes.empty() && a_state.constraints.empty();
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

		if (FindPrototypeStateLocked(a_event.actor) == nullptr && prototypeActors_.size() >= currentMaxActiveActors_) {
			spdlog::debug(
				"skipping prototype physics candidate {} for actor={} because active actor budget is full ({}/{})",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				prototypeActors_.size(),
				currentMaxActiveActors_);
			return false;
		}

		if (player && maxActorDistance_ > 0.0F) {
			const auto distanceSquared = DistanceSquared(a_event.actor->GetPosition(), player->GetPosition());
			const auto maxDistanceSquared = maxActorDistance_ * maxActorDistance_;
			if (distanceSquared > maxDistanceSquared) {
				spdlog::trace(
					"skipping prototype physics candidate {} for actor={} beyond distance budget distanceSq={} maxDistanceSq={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					distanceSquared,
					maxDistanceSquared);
				return false;
			}
		}

		return true;
	}

	Fo4PhysicsWorld::PrototypeActorState* Fo4PhysicsWorld::FindPrototypeStateLocked(RE::Actor* a_actor)
	{
		const auto found = std::ranges::find_if(prototypeActors_, [a_actor](const PrototypeActorState& a_state) {
			return a_state.actor == a_actor;
		});
		return found == prototypeActors_.end() ? nullptr : std::addressof(*found);
	}

	Fo4PhysicsWorld::PrototypeActorState& Fo4PhysicsWorld::GetOrCreatePrototypeStateLocked(RE::Actor* a_actor)
	{
		if (auto* state = FindPrototypeStateLocked(a_actor)) {
			return *state;
		}

		auto& state = prototypeActors_.emplace_back();
		state.actor = a_actor;
		state.actorHandle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
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

		auto* root = resolvedActor->Get3D();
		if (!root) {
			spdlog::debug("dropping prototype physics state for actor={} with no current 3D", static_cast<void*>(a_state.actor));
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
					spdlog::debug(
						"dropping prototype physics state for actor={} beyond distance budget distanceSq={} maxDistanceSq={}",
						static_cast<void*>(a_state.actor),
						distanceSquared,
						maxDistanceSquared);
					SuspendActorCandidateLocked(a_state.actor);
					return false;
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
			return !a_state.actor && a_state.bodies.empty() && a_state.meshes.empty() && a_state.constraints.empty();
		});
	}

	void Fo4PhysicsWorld::EnforceActorBudgetLocked()
	{
		if (prototypeActors_.size() <= currentMaxActiveActors_) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		while (prototypeActors_.size() > currentMaxActiveActors_) {
			auto victim = prototypeActors_.end();
			auto victimDistanceSquared = -1.0F;
			for (auto it = prototypeActors_.begin(); it != prototypeActors_.end(); ++it) {
				if (player && it->actor == player) {
					continue;
				}

				auto distanceSquared = std::numeric_limits<float>::max();
				if (player && it->actor) {
					distanceSquared = DistanceSquared(it->actor->GetPosition(), player->GetPosition());
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
				"dropping prototype physics state for actor={} because active actor budget shrank to {}",
				static_cast<void*>(victim->actor),
				currentMaxActiveActors_);
			SuspendActorCandidateLocked(victim->actor);
			ClearPrototypeStateLocked(*victim);
			prototypeActors_.erase(victim);
		}
	}

	void Fo4PhysicsWorld::SuspendActorCandidateLocked(RE::Actor* a_actor)
	{
		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (!a_actor || a_actor == player || !enableNpcPhysics_) {
			return;
		}

		for (auto& candidate : suspendedActors_) {
			const auto resolvedActor = candidate.actorHandle.get();
			if (resolvedActor && resolvedActor.get() == a_actor) {
				return;
			}
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		if (!handle) {
			return;
		}

		suspendedActors_.push_back({ .actorHandle = handle });
		spdlog::debug("suspended prototype physics candidate actor={} until distance/budget allows rebuild", static_cast<void*>(a_actor));
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
			if (prototypeActors_.size() >= currentMaxActiveActors_) {
				return;
			}

			auto resolvedActor = it->actorHandle.get();
			if (!resolvedActor) {
				it = suspendedActors_.erase(it);
				continue;
			}

			auto* actor = resolvedActor.get();
			if (!actor || FindPrototypeStateLocked(actor)) {
				it = suspendedActors_.erase(it);
				continue;
			}

			auto* root = actor->Get3D();
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

			spdlog::debug("reactivating suspended prototype physics candidate actor={} root={}", static_cast<void*>(actor), static_cast<void*>(root));
			BuildPrototypeForEventLocked({
				.type = LifecycleEventType::kActorSet3D,
				.actor = actor,
				.object = root,
			});
			it = suspendedActors_.erase(it);
		}
	}

	void Fo4PhysicsWorld::ClearPrototypeStateLocked(PrototypeActorState& a_state)
	{
		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}

		if (dynamicsWorld_) {
			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.body) {
					dynamicsWorld_->removeCollisionObject(prototypeMesh.body.get());
				}
			}

			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.constraint) {
					dynamicsWorld_->removeConstraint(prototypeConstraint.constraint.get());
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

		if (dynamicsWorld_) {
			for (auto& prototypeBody : a_state.bodies) {
				if (prototypeBody.bone) {
					dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
				}
			}
		}

		if (!a_state.bodies.empty()) {
			spdlog::debug("cleared {} prototype physics bodies for actor={}", a_state.bodies.size(), static_cast<void*>(a_state.actor));
		}
		a_state.bodies.clear();
		a_state.nextBuildGroup = 0;
		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.resetReadFrames = 0;
	}

	bool Fo4PhysicsWorld::ClearPrototypeGroupsForObjectLocked(PrototypeActorState& a_state, RE::NiAVObject* a_object)
	{
		if (!a_object) {
			return false;
		}

		std::vector<std::uint64_t> buildGroups;
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

		if (buildGroups.empty()) {
			return false;
		}

		ClearPrototypeGroupsLocked(a_state, buildGroups);
		return true;
	}

	bool Fo4PhysicsWorld::ClearPrototypeGroupsByDomainLocked(PrototypeActorState& a_state, const PrototypeBuildDomain a_domain)
	{
		std::vector<std::uint64_t> buildGroups;
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

		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}

		if (dynamicsWorld_) {
			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.body && containsGroup(prototypeMesh.buildGroup)) {
					dynamicsWorld_->removeCollisionObject(prototypeMesh.body.get());
				}
			}

			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.constraint && containsGroup(prototypeConstraint.buildGroup)) {
					dynamicsWorld_->removeConstraint(prototypeConstraint.constraint.get());
				}
			}

			for (auto& prototypeBody : a_state.bodies) {
				const auto allBodyGroupsRemoved = !prototypeBody.buildGroups.empty() ?
					std::ranges::all_of(prototypeBody.buildGroups, containsGroup) :
					containsGroup(prototypeBody.buildGroup);
				if (prototypeBody.bone && allBodyGroupsRemoved) {
					dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
				}
			}
		}

		for (auto& prototypeBody : a_state.bodies) {
			if (!prototypeBody.bone) {
				continue;
			}

			for (const auto buildGroup : a_buildGroups) {
				prototypeBody.bone->RemoveSkinWorldTransformsForBuildGroup(buildGroup);
			}
		}

		const auto meshCount = std::erase_if(a_state.meshes, [&containsGroup](const PrototypeMesh& a_mesh) {
			return containsGroup(a_mesh.buildGroup);
		});
		const auto constraintCount = std::erase_if(a_state.constraints, [&containsGroup](const PrototypeConstraint& a_constraint) {
			return containsGroup(a_constraint.buildGroup);
		});

		for (auto& body : a_state.bodies) {
			if (body.buildGroups.empty() && body.buildGroup != 0) {
				body.buildGroups.push_back(body.buildGroup);
			}
			if (body.buildGroupDomains.empty() && body.buildGroup != 0) {
				body.buildGroupDomains.push_back({ body.buildGroup, PrototypeBuildDomain::kArmor });
			}
			std::erase_if(body.buildGroups, [&containsGroup](const std::uint64_t a_buildGroup) {
				return containsGroup(a_buildGroup);
			});
			std::erase_if(body.buildGroupDomains, [&containsGroup](const auto& a_entry) {
				return containsGroup(a_entry.first);
			});
			if (!body.buildGroups.empty()) {
				body.buildGroup = body.buildGroups.front();
			}
		}
		const auto bodyCount = std::erase_if(a_state.bodies, [](const PrototypeBody& a_body) {
			return a_body.buildGroups.empty();
		});

		spdlog::debug(
			"cleared prototype physics groups={} bodies={} meshes={} constraints={} for actor={}",
			a_buildGroups.size(),
			bodyCount,
			meshCount,
			constraintCount,
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

	void Fo4PhysicsWorld::BuildPrototypeBodiesLocked(PrototypeActorState& a_state, const LifecycleEvent& a_event, const PhysicsXmlSummary& a_summary, const DefaultBBP::NameMap& a_meshNameMap, const PrototypeBuildDomain a_domain)
	{
		auto meshNames = BuildMeshMatchNames(a_summary, a_meshNameMap);
		if (a_summary.boneNames.empty() && meshNames.empty()) {
			spdlog::debug("prototype physics XML has no named bones or mesh descriptors to match");
			return;
		}

		std::vector<MatchedSkinBone> matchedBones;
		auto* skeletonSearchRoot = ResolveSkeletonSearchRoot(a_event);
		CollectMatchedSkinBones(a_event.object, a_summary.boneNames, meshNames, matchedBones);
		ResolveExplicitXmlBonesFromSkeleton(matchedBones, a_summary.boneNames, skeletonSearchRoot);
		AttachSkeletonDrivenSkinSlots(matchedBones, a_event.object, skeletonSearchRoot);
		AttachExplicitSkeletonDrivenSourceNodes(matchedBones, a_event, a_summary);
		if (matchedBones.empty()) {
			spdlog::debug(
				"prototype physics XML matched no skin bones for {} actor={} object={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
			return;
		}

		std::uint32_t created = 0;
		std::uint32_t dynamicBodies = 0;
		std::uint32_t kinematicBodies = 0;
		std::uint32_t sameNameSkeletonDrivers = 0;
		std::uint32_t drivenAncestors = 0;
		std::uint32_t matchedUnderActorRoot = 0;
		std::uint32_t matchedUnderAttachedObject = 0;
		std::uint64_t buildGroup = 0;
		auto* actorRoot = a_event.actor ? a_event.actor->Get3D(a_event.firstPerson) : nullptr;
		if (!actorRoot && a_event.actor) {
			actorRoot = a_event.actor->Get3D();
		}
		for (const auto& matchedBone : matchedBones) {
			if (IsNodeInTree(actorRoot, matchedBone.node)) {
				++matchedUnderActorRoot;
			}
			if (IsNodeInTree(a_event.object, matchedBone.node)) {
				++matchedUnderAttachedObject;
			}
		}
		for (const auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.geometry && IsObjectInTree(a_event.object, prototypeMesh.geometry)) {
				buildGroup = prototypeMesh.buildGroup;
				break;
			}
		}
		for (const auto& prototypeBody : a_state.bodies) {
			if (buildGroup != 0) {
				break;
			}
			if (prototypeBody.node && IsNodeInTree(a_event.object, prototypeBody.node)) {
				buildGroup = prototypeBody.buildGroup;
				break;
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
			} else if (!existing->buildGroups.empty()) {
				buildGroup = existing->buildGroups.front();
			}
		}
		if (buildGroup == 0) {
			buildGroup = ++a_state.nextBuildGroup;
		}

		for (auto& matchedBone : matchedBones) {
			const auto existing = std::ranges::find_if(a_state.bodies, [&matchedBone](const PrototypeBody& a_body) {
				return a_body.node == matchedBone.node;
			});
			if (existing != a_state.bodies.end()) {
				if (std::ranges::find(existing->buildGroups, buildGroup) == existing->buildGroups.end()) {
					existing->buildGroups.push_back(buildGroup);
				}
				if (std::ranges::find(existing->buildGroupDomains, std::pair{ buildGroup, a_domain }) == existing->buildGroupDomains.end()) {
					existing->buildGroupDomains.push_back({ buildGroup, a_domain });
				}
				if (existing->bone) {
					for (auto& skinWorld : matchedBone.skinWorldTransforms) {
						existing->bone->AddSkinWorldTransform(skinWorld.skin.get(), skinWorld.index, buildGroup);
					}
					for (auto& drivenTarget : matchedBone.drivenTargets) {
						if (drivenTarget) {
							existing->bone->AddDrivenAncestor(drivenTarget.get(), matchedBone.node);
							++drivenAncestors;
						}
					}
					drivenAncestors += AddSkeletonDrivenAncestors(*existing->bone, matchedBone.node, skeletonSearchRoot, a_event.object);
				}
				continue;
			}

			const auto* descriptor = FindBoneDescriptor(a_summary, matchedBone.name);
			auto fallbackDescriptor = a_summary.defaultBoneDescriptor.value_or(PhysicsBoneDescriptor{});
			fallbackDescriptor.name = matchedBone.name;
			const auto& boneDescriptor = descriptor ? *descriptor : fallbackDescriptor;
			auto shape = CreateCollisionShape(boneDescriptor);
			btVector3 localInertia(0.0F, 0.0F, 0.0F);
			const auto mass = std::max(boneDescriptor.mass, 0.0F);
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
			auto motionState = std::make_unique<btDefaultMotionState>(ToBulletTransform(initialWorld) * localToRig);
			btRigidBody::btRigidBodyConstructionInfo constructionInfo(mass, motionState.get(), shape.get(), localInertia);
			auto bone = std::make_unique<Fo4SkinnedMeshBone>(RE::BSFixedString(matchedBone.name), matchedBone.node, constructionInfo);
			for (auto& skinWorld : matchedBone.skinWorldTransforms) {
				bone->AddSkinWorldTransform(skinWorld.skin.get(), skinWorld.index, buildGroup);
			}
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
			bone->m_rig.setGravity(btVector3(0.0F, 0.0F, -9.80665F * bone->m_gravityFactor));
			if (mass <= 0.0F) {
				++kinematicBodies;
				if (auto* driver = FindNodeByName(skeletonSearchRoot, matchedBone.name); driver && driver != matchedBone.node) {
					bone->SetDriverNode(driver);
					++sameNameSkeletonDrivers;
					spdlog::debug(
						"prototype kinematic bone '{}' will read transform from actor skeleton node={} instead of cloned node={}",
						matchedBone.name,
						static_cast<void*>(driver),
						static_cast<void*>(matchedBone.node));
				}
			} else {
				++dynamicBodies;
			}
			for (auto& drivenTarget : matchedBone.drivenTargets) {
				if (drivenTarget) {
					bone->AddDrivenAncestor(drivenTarget.get(), matchedBone.node);
					++drivenAncestors;
				}
			}
			drivenAncestors += AddSkeletonDrivenAncestors(*bone, matchedBone.node, skeletonSearchRoot, a_event.object);
			bone->readTransform(0.0F);
			bone->m_rig.setActivationState(DISABLE_DEACTIVATION);
			// hdtSMP bones are solver/constraint bodies; mesh collisions are handled by the custom dispatcher.
			dynamicsWorld_->addRigidBody(std::addressof(bone->m_rig), 0, 0);

			PrototypeBody prototypeBody;
			prototypeBody.actor = a_event.actor;
			prototypeBody.node = matchedBone.node;
			prototypeBody.buildGroup = buildGroup;
			prototypeBody.buildGroups.push_back(buildGroup);
			prototypeBody.buildGroupDomains.push_back({ buildGroup, a_domain });
			prototypeBody.boneName = std::move(matchedBone.name);
			prototypeBody.shape = std::move(shape);
			prototypeBody.motionState = std::move(motionState);
			prototypeBody.bone = std::move(bone);
			a_state.bodies.push_back(std::move(prototypeBody));
			++created;
		}

		std::ranges::stable_sort(a_state.bodies, [](const PrototypeBody& a_lhs, const PrototypeBody& a_rhs) {
			const auto lhsDepth = a_lhs.bone ? a_lhs.bone->GetDepth() : 0x7fffffff;
			const auto rhsDepth = a_rhs.bone ? a_rhs.bone->GetDepth() : 0x7fffffff;
			return lhsDepth < rhsDepth;
		});

		BuildPrototypeMeshesLocked(a_state, a_summary, a_event, a_meshNameMap, buildGroup, a_domain);
		BuildPrototypeConstraintsLocked(a_state, a_summary, buildGroup, a_domain);
		a_state.resetReadFrames = std::max<std::uint32_t>(a_state.resetReadFrames, 1);
		spdlog::info(
			"prepared {} prototype physics buildGroup={} createdBodies={} dynamicBodies={} kinematicBodies={} matchedXMLBones={} skeletonDrivers={} drivenAncestors={} actor={} actorBodies={} activeActors={}",
			PrototypeDomainName(a_domain),
			buildGroup,
			created,
			dynamicBodies,
			kinematicBodies,
			matchedBones.size(),
			sameNameSkeletonDrivers,
			drivenAncestors,
			static_cast<void*>(a_state.actor),
			a_state.bodies.size(),
			prototypeActors_.size());
		spdlog::debug(
			"prototype matched bone placement actorRoot={} attachedObject={} underActorRoot={} underAttachedObject={} matchedXMLBones={}",
			static_cast<void*>(actorRoot),
			static_cast<void*>(a_event.object),
			matchedUnderActorRoot,
			matchedUnderAttachedObject,
			matchedBones.size());
		diagnosticFrameBudget_ = 120;
	}

	void Fo4PhysicsWorld::BuildPrototypeMeshesLocked(PrototypeActorState& a_state, const PhysicsXmlSummary& a_summary, const LifecycleEvent& a_event, const DefaultBBP::NameMap& a_meshNameMap, const std::uint64_t a_buildGroup, const PrototypeBuildDomain a_domain)
	{
		if (!dynamicsWorld_ || a_summary.meshDescriptors.empty()) {
			return;
		}

		auto meshNames = BuildMeshMatchNames(a_summary, a_meshNameMap);

		auto extraction = ExtractSkinnedMeshes(a_event.object, meshNames);
		std::uint32_t created = 0;
		std::uint32_t skippedMissingBones = 0;
		std::uint32_t skippedMissingBoneData = 0;
		std::uint32_t skippedBadBoneIndices = 0;
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

			const auto existingMesh = std::ranges::find_if(a_state.meshes, [&decodedMesh](const PrototypeMesh& a_mesh) {
				return a_mesh.geometry == decodedMesh.geometry;
			});
			if (existingMesh != a_state.meshes.end()) {
				++skippedExisting;
				continue;
			}

			const auto* meshDescriptor = FindMeshDescriptor(a_summary, decodedMesh.name, a_meshNameMap);
			if (decodedMesh.badBoneIndices > 0) {
				++skippedBadBoneIndices;
				spdlog::warn("skipping mesh '{}' because decoded vertex weights reference {} out-of-range bone indices", decodedMesh.name, decodedMesh.badBoneIndices);
				continue;
			}
			const auto weightedBoneWithoutBindData = std::ranges::any_of(decodedMesh.vertices, [&decodedMesh](const hdt::Vertex& a_vertex) {
				for (int influence = 0; influence < 4; ++influence) {
					const auto boneIndex = static_cast<std::size_t>(a_vertex.getBoneIdx(influence));
					if (a_vertex.weight_[influence] > FLT_EPSILON &&
						boneIndex < decodedMesh.bones.size() &&
						!decodedMesh.bones[boneIndex].hasBoneData) {
						return true;
					}
				}
				return false;
			});
			if (weightedBoneWithoutBindData) {
				++skippedMissingBoneData;
				spdlog::warn("skipping mesh '{}' because a weighted skin bone is missing FO4 bind-pose data", decodedMesh.name);
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

			bool missingWeightedBone = false;
			for (std::size_t boneIndex = 0; boneIndex < decodedMesh.bones.size(); ++boneIndex) {
				const auto& decodedBone = decodedMesh.bones[boneIndex];
				const auto matchedBody = std::ranges::find_if(a_state.bodies, [&decodedBone, a_buildGroup](const PrototypeBody& a_body) {
					return decodedBone.node &&
						a_body.node == decodedBone.node &&
						std::ranges::find(a_body.buildGroups, a_buildGroup) != a_body.buildGroups.end();
				});

				if (matchedBody == a_state.bodies.end() || !matchedBody->bone) {
					for (const auto& vertex : decodedMesh.vertices) {
						for (int influence = 0; influence < 4; ++influence) {
							if (vertex.weight_[influence] > FLT_EPSILON && vertex.getBoneIdx(influence) == boneIndex) {
								missingWeightedBone = true;
								break;
							}
						}
						if (missingWeightedBone) {
							break;
						}
					}
					meshBody->addBone(
						nullptr,
						decodedBone.hasBoneData ? decodedBone.skinToBone : hdt::btQsTransform::getIdentity(),
						decodedBone.hasBoneData ? decodedBone.boundingSphere : hdt::BoundingSphere(btVector3(0.0F, 0.0F, 0.0F), 0.0F));
					continue;
				}

				const auto sphere = decodedBone.hasBoneData ?
					decodedBone.boundingSphere :
					CalculateBoneSphere(decodedMesh, boneIndex).value_or(hdt::BoundingSphere(btVector3(0.0F, 0.0F, 0.0F), 0.0F));
				meshBody->addBone(matchedBody->bone.get(), decodedBone.hasBoneData ? decodedBone.skinToBone : hdt::btQsTransform::getIdentity(), sphere);
			}

			if (missingWeightedBone) {
				++skippedMissingBones;
				continue;
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
					const auto matchedBody = std::ranges::find_if(a_state.bodies, [&boneName, a_buildGroup](const PrototypeBody& a_body) {
						return std::ranges::find(a_body.buildGroups, a_buildGroup) != a_body.buildGroups.end() && PhysicsNamesEqual(a_body.boneName, boneName);
					});
					if (matchedBody != a_state.bodies.end() && matchedBody->bone) {
						meshBody->canCollideWithBones_.push_back(matchedBody->bone.get());
					} else {
						++unresolvedCanCollideBones;
						spdlog::debug("mesh '{}' could not resolve can-collide-with-bone '{}'", decodedMesh.name, boneName);
					}
				}
				for (const auto& boneName : meshDescriptor->noCollideWithBones) {
					const auto matchedBody = std::ranges::find_if(a_state.bodies, [&boneName, a_buildGroup](const PrototypeBody& a_body) {
						return std::ranges::find(a_body.buildGroups, a_buildGroup) != a_body.buildGroups.end() && PhysicsNamesEqual(a_body.boneName, boneName);
					});
					if (matchedBody != a_state.bodies.end() && matchedBody->bone) {
						meshBody->noCollideWithBones_.push_back(matchedBody->bone.get());
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
			dynamicsWorld_->addCollisionObject(meshBody.get(), 1, 1);

			PrototypeMesh prototypeMesh;
			prototypeMesh.name = decodedMesh.name;
			prototypeMesh.geometry = decodedMesh.geometry;
			prototypeMesh.buildGroup = a_buildGroup;
			prototypeMesh.domain = a_domain;
			prototypeMesh.body = std::move(meshBody);
			a_state.meshes.push_back(std::move(prototypeMesh));
			++created;
		}

		spdlog::info(
			"created {} {} prototype skinned mesh bodies for actor={} from decodedMeshes={} skippedExisting={} skippedEmpty={} skippedMissingBones={} skippedMissingBoneData={} skippedBadBoneIndices={} skippedMissingTriangleIndices={} skippedInvalidTriangleIndices={} skippedNoColliders={} unresolvedCanCollideBones={} unresolvedNoCollideBones={} unresolvedWeightThresholds={} nullBones={} nonNodeBones={} missingBoneData={} unsupportedGeometryClasses={} badBoneIndices={}",
			created,
			PrototypeDomainName(a_domain),
			static_cast<void*>(a_state.actor),
			extraction.stats.decodedMeshes,
			skippedExisting,
			skippedEmpty,
			skippedMissingBones,
			skippedMissingBoneData,
			skippedBadBoneIndices,
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
			extraction.stats.badBoneIndices);
	}

	void Fo4PhysicsWorld::BuildPrototypeConstraintsLocked(PrototypeActorState& a_state, const PhysicsXmlSummary& a_summary, const std::uint64_t a_buildGroup, const PrototypeBuildDomain a_domain)
	{
		std::uint32_t created = 0;
		std::uint32_t skippedMissingBodies = 0;
		std::uint32_t skippedExisting = 0;
		std::uint32_t skippedSelfConstraints = 0;
		std::uint32_t skippedKinematicPairs = 0;
		std::uint32_t skippedInvalid = 0;
		for (const auto& descriptor : a_summary.constraintDescriptors) {
			const auto bodyA = std::ranges::find_if(a_state.bodies, [&descriptor, a_buildGroup](const PrototypeBody& a_body) {
				return std::ranges::find(a_body.buildGroups, a_buildGroup) != a_body.buildGroups.end() && PhysicsNamesEqual(a_body.boneName, descriptor.bodyA);
			});
			const auto bodyB = std::ranges::find_if(a_state.bodies, [&descriptor, a_buildGroup](const PrototypeBody& a_body) {
				return std::ranges::find(a_body.buildGroups, a_buildGroup) != a_body.buildGroups.end() && PhysicsNamesEqual(a_body.boneName, descriptor.bodyB);
			});
			if (bodyA == a_state.bodies.end() || bodyB == a_state.bodies.end() || !bodyA->bone || !bodyB->bone) {
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
				++skippedKinematicPairs;
				spdlog::warn("skipping constraint '{}' between two kinematic bodies '{}'/'{}'", descriptor.name, descriptor.bodyA, descriptor.bodyB);
				continue;
			}

			const auto existing = std::ranges::find_if(a_state.constraints, [&descriptor, a_buildGroup](const PrototypeConstraint& a_constraint) {
					return a_constraint.buildGroup == a_buildGroup && PhysicsNamesEqual(a_constraint.bodyA, descriptor.bodyA) && PhysicsNamesEqual(a_constraint.bodyB, descriptor.bodyB);
			});
			if (existing != a_state.constraints.end()) {
				++skippedExisting;
				if (diagnosticFrameBudget_ > 0) {
					spdlog::info(
						"skipping duplicate {} prototype constraint '{}' for buildGroup={} bodies '{}'/'{}'",
						PrototypeDomainName(a_domain),
						descriptor.name,
						a_buildGroup,
						descriptor.bodyA,
						descriptor.bodyB);
				}
				continue;
			}

			auto constraint = CreatePrototypeConstraint(descriptor, bodyA->bone->m_rig, bodyB->bone->m_rig, bodyA->bone->m_rigToLocal, bodyB->bone->m_rigToLocal);
			if (!constraint) {
				++skippedInvalid;
				continue;
			}

			dynamicsWorld_->addConstraint(constraint.get(), true);
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
			a_state.constraints.push_back(std::move(prototypeConstraint));
			++created;
		}

		if (created > 0 || skippedMissingBodies > 0 || skippedExisting > 0 || skippedSelfConstraints > 0 || skippedKinematicPairs > 0 || skippedInvalid > 0) {
		spdlog::info(
			"created {} {} prototype Bullet constraints for actor={} buildGroup={}; actor constraints={} skippedMissingBodies={} skippedExisting={} skippedSelfConstraints={} skippedKinematicPairs={} skippedInvalid={}",
			created,
			PrototypeDomainName(a_domain),
			static_cast<void*>(a_state.actor),
				a_buildGroup,
				a_state.constraints.size(),
				skippedMissingBodies,
				skippedExisting,
				skippedSelfConstraints,
				skippedKinematicPairs,
				skippedInvalid);
		}
	}

	void Fo4PhysicsWorld::ScalePrototypeConstraintsLocked(PrototypeActorState& a_state)
	{
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
}
