#pragma once

#include "LifecycleEvents.h"

namespace Smp
{
	class PrototypePhysicsSystem :
		public RE::BSTEventSink<LifecycleEvent>
	{
	public:
		static PrototypePhysicsSystem* GetSingleton();
		void Register();

		RE::BSEventNotifyControl ProcessEvent(const LifecycleEvent& a_event, RE::BSTEventSource<LifecycleEvent>* a_source) override;

	private:
		void InspectAttachedObject(const LifecycleEvent& a_event);
		bool registered_{ false };
	};
}
