#include "PluginInterfaceImpl.h"

#include <F4SE/API.h>
#include <F4SE/Interfaces.h>
#include <LinearMath/btScalar.h>

static_assert(BT_BULLET_VERSION == 325, "PluginAPI.h's advertised Bullet version must match the linked Bullet ABI");

namespace hdt
{
	PluginInterfaceImpl g_pluginInterface;

	void PluginInterfaceImpl::addListener(IPreStepListener* a_listener)
	{
		preStepDispatcher_.RegisterSink(a_listener);
	}

	void PluginInterfaceImpl::removeListener(IPreStepListener* a_listener)
	{
		preStepDispatcher_.UnregisterSink(a_listener);
	}

	void PluginInterfaceImpl::addListener(IPostStepListener* a_listener)
	{
		postStepDispatcher_.RegisterSink(a_listener);
	}

	void PluginInterfaceImpl::removeListener(IPostStepListener* a_listener)
	{
		postStepDispatcher_.UnregisterSink(a_listener);
	}

	void PluginInterfaceImpl::Initialize()
	{
		messagingInterface_ = F4SE::GetMessagingInterface();
		if (!messagingInterface_) {
			spdlog::warn("failed to get the F4SE messaging interface; the HDT-SMP plugin API will be unavailable");
		}
	}

	void PluginInterfaceImpl::OnPostPostLoad()
	{
		if (messagingInterface_) {
			messagingInterface_->Dispatch(MSG_STARTUP, static_cast<PluginInterface*>(this), 0, nullptr);
		}
	}
}
