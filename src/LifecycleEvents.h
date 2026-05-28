#pragma once

#include <string>

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
		kActorUpdate3DModel,
		kActorReset3D
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
		RE::NiNode*        destinationRoot{ nullptr };
		RE::NiNode*        sourceRoot{ nullptr };
		std::string        physicsXmlPath;
		bool               firstPerson{ false };
		bool               queue3DTasks{ false };
		bool               queueDetach{ false };
	};

	RE::BSTEventSource<LifecycleEvent>& GetLifecycleEventSource();
	void NotifyLifecycleEvent(const LifecycleEvent& a_event);
	const char* ToString(LifecycleEventType a_type);
}
