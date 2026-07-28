#pragma once

#include "RE/B/BSFixedString.h"
#include "RE/N/NiNode.h"
#include "RE/N/NiPointer.h"
#include "RE/N/NiTransform.h"
#include "BSSkin.h"

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace RE
{
	class Actor;
	class NiAVObject;
	class BSFaceGenNiNode;
	class BSFlattenedBoneTree;
}

namespace Smp
{
	struct ArmorBoneReference
	{
		std::string name;
		std::string parentBoneName;
		RE::NiTransform localToParentBone{ RE::NiTransform::IDENTITY };
		RE::NiPointer<RE::NiNode> resolvedNode;
		bool isSkinned{ false };
		bool isArmorOnly{ false };
		bool parentBoneIsArmorOnly{ false };
		bool createdByUs{ false };
	};

	struct RetainedSkinBinding
	{
		RE::NiPointer<RE::BSSkin::Instance> skin;
		std::vector<RE::BSFixedString> boneNames;
	};

	struct RetainedSkinRebindResult
	{
		std::uint32_t instances{ 0 };
		std::uint32_t boneSlots{ 0 };
		std::uint32_t reboundSlots{ 0 };
		std::uint32_t unresolvedSlots{ 0 };
		std::uint32_t unnamedSlots{ 0 };
		std::uint32_t transformSizeMismatches{ 0 };
		std::uint32_t boneSizeMismatches{ 0 };
	};

	std::vector<ArmorBoneReference> CaptureArmorBoneReferences(
		RE::NiAVObject* a_modelRoot,
		RE::NiAVObject* a_skeletonRoot,
		std::string_view a_nifPath);
	std::vector<RetainedSkinBinding> CaptureRetainedSkinBindings(RE::NiAVObject* a_retainedFace);
	RetainedSkinRebindResult RebindRetainedSkinBindings(
		RE::BSFlattenedBoneTree* a_root,
		std::span<const RetainedSkinBinding> a_bindings);
	std::uint32_t MaterializeRetainedHeadPartBones(
		RE::Actor* a_actor,
		RE::BSFaceGenNiNode* a_retainedFace,
		RE::BSFlattenedBoneTree* a_root,
		std::vector<ArmorBoneReference>& a_references,
		std::span<const std::string> a_requiredBoneNames);
	bool FinalizeArmorSkinBindings(
		RE::Actor* a_actor,
		RE::NiAVObject* a_attachedObject,
		RE::NiNode* a_skeletonRoot,
		bool a_firstPerson,
		std::vector<ArmorBoneReference>& a_references,
		std::span<const RE::NiPointer<RE::NiAVObject>> a_additionalAttachedObjects = {},
		bool a_moveBoundBonesToSkeleton = true);
}
