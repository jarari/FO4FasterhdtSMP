#include "LifecycleEvents.h"

#include "RE/P/PlayerCharacter.h"

#include <spdlog/spdlog.h>

#include <mutex>
#include <vector>

namespace Smp
{
	namespace
	{
		std::mutex& GetLifecycleEventQueueLock()
		{
			static std::mutex lock;
			return lock;
		}

		std::vector<LifecycleEvent>& GetLifecycleEventQueue()
		{
			static std::vector<LifecycleEvent> queue;
			return queue;
		}

		RE::ActorHandle GetRetainedActorHandle(RE::Actor* a_actor)
		{
			if (!a_actor) {
				return {};
			}

			if (a_actor == RE::PlayerCharacter::GetSingleton()) {
				return RE::PlayerCharacter::GetPlayerHandle();
			}
			return RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		}

		LifecycleEvent RetainLifecyclePointers(const LifecycleEvent& a_event)
		{
			auto retained = a_event;
			retained.retainedObject = a_event.object;
			retained.retainedSourceObject = a_event.sourceObject;
			retained.retainedMergeSourceObject = a_event.mergeSourceObject;
			retained.retainedDestinationRoot = a_event.destinationRoot;
			retained.retainedSourceRoot = a_event.sourceRoot;
			retained.retainedActor = GetRetainedActorHandle(a_event.actor);
			return retained;
		}
	}

	RE::BSTEventSource<LifecycleEvent>& GetLifecycleEventSource()
	{
		static RE::BSTEventSource<LifecycleEvent> source;
		return source;
	}

	void NotifyLifecycleEvent(const LifecycleEvent& a_event)
	{
		auto retained = RetainLifecyclePointers(a_event);
		std::scoped_lock lock(GetLifecycleEventQueueLock());
		GetLifecycleEventQueue().push_back(std::move(retained));
	}

	void DrainQueuedLifecycleEvents()
	{
		std::vector<LifecycleEvent> events;
		{
			std::scoped_lock lock(GetLifecycleEventQueueLock());
			events.swap(GetLifecycleEventQueue());
		}

		for (auto& event : events) {
			if (event.actor) {
				if (!event.retainedActor) {
					spdlog::trace(
						"dropped queued lifecycle {} because actor handle could not be retained actor={}",
						ToString(event.type),
						static_cast<void*>(event.actor));
					continue;
				}

				auto actor = event.retainedActor.get();
				if (!actor) {
					spdlog::trace(
						"dropped queued lifecycle {} because retained actor handle expired actor={}",
						ToString(event.type),
						static_cast<void*>(event.actor));
					continue;
				}
				event.actor = actor.get();
			}

			GetLifecycleEventSource().Notify(event);
		}
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
		case LifecycleEventType::kHeadSkinAllGeometry:
			return "HeadSkinAllGeometry";
		case LifecycleEventType::kHeadSkinSingleGeometry:
			return "HeadSkinSingleGeometry";
		case LifecycleEventType::kActorUpdate3DModel:
			return "ActorUpdate3DModel";
		case LifecycleEventType::kActorReset3D:
			return "ActorReset3D";
		default:
			return "Unknown";
		}
	}
}
