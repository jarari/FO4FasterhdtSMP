#pragma once

#include <string>
#include <vector>

namespace Smp
{
	enum class LifecycleEventType
	{
		kArmorApplySkinnedObjects,
		kArmorAttachSkinnedObject,
		kArmorAttachToParent,
		kArmorDetachBegin,
		kArmorDetachEnd,
		kActorLoad3D,
		kActorSet3D,
		kActorHeadInitialized,
		kHeadPrepareHeadPart,
		kActorUpdate3DModel,
		kActorReset3D
	};

	struct MergeParentBinding
	{
		std::string sourceName;
		std::string parentName;
		RE::NiTransform localToParent{ RE::NiTransform::IDENTITY };
		bool hasLocalToParent{ false };
	};

	struct LifecycleEvent
	{
		LifecycleEventType type;
		RE::Actor*         actor{ nullptr };
		RE::BipedAnim*     biped{ nullptr };
		RE::BIPOBJECT*     bipObject{ nullptr };
		RE::BIPED_OBJECT   bipedObject{ RE::BIPED_OBJECT::kTotal };
		RE::NiAVObject*    object{ nullptr };
		RE::NiAVObject*    sourceObject{ nullptr };
		RE::NiAVObject*    mergeSourceObject{ nullptr };
		RE::BGSHeadPart*   headPart{ nullptr };
		std::vector<RE::NiAVObject*> trustedActorSkeletonNodes;
		std::vector<RE::NiAVObject*> mergeSearchExclusions;
		std::vector<MergeParentBinding> mergeParentBindings;
		RE::NiNode*        destinationRoot{ nullptr };
		RE::NiNode*        sourceRoot{ nullptr };
		std::string        mergeRenamePrefix;
		std::string        physicsXmlPath;
		bool               firstPerson{ false };
		bool               queue3DTasks{ false };
		bool               queueDetach{ false };
		bool               preserveMergeSourceNames{ false };
	};

	RE::BSTEventSource<LifecycleEvent>& GetLifecycleEventSource();
	void NotifyLifecycleEvent(const LifecycleEvent& a_event);
	const char* ToString(LifecycleEventType a_type);
}
