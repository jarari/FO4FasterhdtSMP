#pragma once

#include "RE/N/NiNode.h"
#include "RE/N/NiPointer.h"
#include "RE/N/NiTransform.h"

#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace RE
{
	class Actor;
	class NiAVObject;
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

	std::vector<ArmorBoneReference> CaptureArmorBoneReferences(
		RE::NiAVObject* a_modelRoot,
		RE::NiAVObject* a_skeletonRoot,
		std::string_view a_nifPath);
	void FinalizeArmorSkinBindings(
		RE::Actor* a_actor,
		RE::NiAVObject* a_attachedObject,
		RE::NiNode* a_skeletonRoot,
		bool a_firstPerson,
		std::vector<ArmorBoneReference>& a_references,
		std::span<const RE::NiPointer<RE::NiAVObject>> a_additionalAttachedObjects = {});
}
