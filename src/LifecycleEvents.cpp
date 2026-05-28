#include "LifecycleEvents.h"

namespace Smp
{
	RE::BSTEventSource<LifecycleEvent>& GetLifecycleEventSource()
	{
		static RE::BSTEventSource<LifecycleEvent> source;
		return source;
	}

	void NotifyLifecycleEvent(const LifecycleEvent& a_event)
	{
		GetLifecycleEventSource().Notify(a_event);
	}

	const char* ToString(const LifecycleEventType a_type)
	{
		switch (a_type) {
		case LifecycleEventType::kArmorApplySkinnedObjects:
			return "ArmorApplySkinnedObjects";
		case LifecycleEventType::kArmorAttachSkinnedObject:
			return "ArmorAttachSkinnedObject";
		case LifecycleEventType::kArmorAttachToParent:
			return "ArmorAttachToParent";
		case LifecycleEventType::kArmorDetachBegin:
			return "ArmorDetachBegin";
		case LifecycleEventType::kArmorDetachEnd:
			return "ArmorDetachEnd";
		case LifecycleEventType::kActorLoad3D:
			return "ActorLoad3D";
		case LifecycleEventType::kActorSet3D:
			return "ActorSet3D";
		case LifecycleEventType::kActorHeadInitialized:
			return "ActorHeadInitialized";
		case LifecycleEventType::kActorUpdate3DModel:
			return "ActorUpdate3DModel";
		case LifecycleEventType::kActorReset3D:
			return "ActorReset3D";
		default:
			return "Unknown";
		}
	}
}
