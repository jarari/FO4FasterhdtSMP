#pragma once

#include "RE/N/NiTransform.h"

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
		bool isArmorOnly{ false };
		bool parentBoneIsArmorOnly{ false };
	};

	std::vector<ArmorBoneReference> CaptureArmorBoneReferences(
		RE::NiAVObject* a_modelRoot,
		RE::NiAVObject* a_skeletonRoot,
		std::string_view a_nifPath);
	void FinalizeArmorSkinBindings(
		RE::Actor* a_actor,
		RE::NiAVObject* a_attachedObject,
		bool a_firstPerson,
		std::vector<ArmorBoneReference>& a_references);
	void RestoreArmorBoneLocalPose(
		RE::NiAVObject* a_attachedObject,
		const std::vector<ArmorBoneReference>& a_references);
}
