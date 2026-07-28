#pragma once

#include "ArmorBoneReference.h"
#include "RE/B/BSPointerHandle.h"
#include "RE/N/NiPointer.h"

#include <cstddef>
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
		kHeadSkinAllGeometry,
		kHeadSkinSingleGeometry,
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
		std::vector<ArmorBoneReference> armorBoneReferences;
		RE::NiNode*        destinationRoot{ nullptr };
		RE::NiNode*        sourceRoot{ nullptr };
		std::string        physicsXmlPath;
		bool               firstPerson{ false };
		bool               queue3DTasks{ false };
		bool               queueDetach{ false };
		RE::NiPointer<RE::NiAVObject> retainedObject;
		RE::NiPointer<RE::NiAVObject> retainedSourceObject;
		RE::NiPointer<RE::NiNode>     retainedDestinationRoot;
		RE::NiPointer<RE::NiNode>     retainedSourceRoot;
		RE::ActorHandle               retainedActor;
	};

	RE::BSTEventSource<LifecycleEvent>& GetLifecycleEventSource();
	void NotifyLifecycleEvent(const LifecycleEvent& a_event);
	std::size_t DiscardQueuedLifecycleEvents(RE::Actor* a_actor);
	std::size_t DiscardAllQueuedLifecycleEvents();
	void DrainQueuedLifecycleEvents();
	const char* ToString(LifecycleEventType a_type);
}
