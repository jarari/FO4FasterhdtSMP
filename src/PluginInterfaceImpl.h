#pragma once

#include "PluginAPI.h"

namespace F4SE
{
	class MessagingInterface;
}

namespace hdt
{
	class PluginInterfaceImpl final :
		public PluginInterface
	{
	public:
		PluginInterfaceImpl() = default;
		~PluginInterfaceImpl() override = default;

		PluginInterfaceImpl(const PluginInterfaceImpl&) = delete;
		PluginInterfaceImpl& operator=(const PluginInterfaceImpl&) = delete;

		[[nodiscard]] const VersionInfo& getVersionInfo() const override { return versionInfo_; }

		void addListener(IPreStepListener* a_listener) override;
		void removeListener(IPreStepListener* a_listener) override;

		void addListener(IPostStepListener* a_listener) override;
		void removeListener(IPostStepListener* a_listener) override;

		void Initialize();
		void OnPostPostLoad();
		void OnPreStep(const PreStepEvent& a_event) { preStepDispatcher_.Notify(a_event); }
		void OnPostStep(const PostStepEvent& a_event) { postStepDispatcher_.Notify(a_event); }

	private:
		VersionInfo versionInfo_{ INTERFACE_VERSION, BULLET_VERSION };
		RE::BSTEventSource<PreStepEvent> preStepDispatcher_;
		RE::BSTEventSource<PostStepEvent> postStepDispatcher_;
		const F4SE::MessagingInterface* messagingInterface_{ nullptr };
	};

	extern PluginInterfaceImpl g_pluginInterface;
}
