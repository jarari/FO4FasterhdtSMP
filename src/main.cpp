#include "Hooks.h"
#include "DefaultBBP.h"
#include "Fo4PhysicsWorld.h"
#include "ImguiLayer.h"
#include "LifecycleEvents.h"
#include "PhysicsXml.h"
#include "PrototypePhysicsSystem.h"
#include "SmpConfig.h"

#include <version.h>

namespace
{
	void DrawBulletVisualization()
	{
		Smp::Fo4PhysicsWorld::GetSingleton()->DrawBulletVisualization();
	}

	const char* RuntimeName()
	{
		if (REX::FModule::IsRuntimeOG()) {
			return "og";
		}
		if (REX::FModule::IsRuntimeAE()) {
			return "ae";
		}
		if (REX::FModule::IsRuntimeNG()) {
			return "ng";
		}
		return "unknown";
	}

	const char* SimdVariantName()
	{
#if defined(FO4_FASTER_HDTSMP_AVX512)
		return "AVX512";
#elif defined(FO4_FASTER_HDTSMP_AVX2)
		return "AVX2";
#else
		return "No AVX";
#endif
	}

	void LoadRuntimeConfig()
	{
		Smp::PhysicsXmlLoader::GetSingleton()->ClearCache();
		Smp::DefaultBBP::GetSingleton()->Reload();
		Smp::Config::GetSingleton()->Load();
		Smp::Config::GetSingleton()->Log();
		const auto& settings = Smp::Config::GetSingleton()->GetSettings();
		if (!Smp::PhysicsXmlLoader::GetSingleton()->LoadPrototype(settings.smp.prototypePhysicsXml) && !settings.smp.prototypePhysicsXml.empty()) {
			Smp::PhysicsXmlLoader::GetSingleton()->LoadPrototype({});
		}
		Hooks::ApplyConfig(settings);
		Smp::Fo4PhysicsWorld::GetSingleton()->ApplyConfig(settings);
		Smp::ImguiLayer::SetEnabled(settings.smp.enableBulletVisualization);
		Smp::ImguiLayer::SetDrawCallback(settings.smp.enableBulletVisualization ? &DrawBulletVisualization : nullptr);
		if (settings.smp.enablePrototypeDiagnostics) {
			Smp::PrototypePhysicsSystem::GetSingleton()->Register();
		}
	}

	void ResetRuntimePhysics(std::string_view a_reason)
	{
		spdlog::info("resetting FO4 Faster HDT-SMP physics state: {}", a_reason);
		Smp::Fo4PhysicsWorld::GetSingleton()->Reset();
	}

	class LifecycleLogSink :
		public RE::BSTEventSink<Smp::LifecycleEvent>
	{
	public:
		static LifecycleLogSink* GetSingleton()
		{
			static LifecycleLogSink singleton;
			return std::addressof(singleton);
		}

		RE::BSEventNotifyControl ProcessEvent(const Smp::LifecycleEvent& a_event, RE::BSTEventSource<Smp::LifecycleEvent>*) override
		{
			spdlog::trace(
				"lifecycle event {} actor={} object={}",
				Smp::ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
			return RE::BSEventNotifyControl::kContinue;
		}
	};

	bool InitializeRuntime()
	{
		static bool initialized = false;
		if (initialized) {
			return true;
		}

		Smp::GetLifecycleEventSource().RegisterSink(LifecycleLogSink::GetSingleton());
		LoadRuntimeConfig();
		Smp::Fo4PhysicsWorld::GetSingleton()->Register();
		initialized = true;
		return true;
	}
}

void OnInit(F4SE::MessagingInterface::Message* a_msg)
{
	switch (a_msg->type) {
		case F4SE::MessagingInterface::kPostPostLoad:
			break;
		case F4SE::MessagingInterface::kPreLoadGame:
			ResetRuntimePhysics("pre-load-game");
			break;
		case F4SE::MessagingInterface::kPostLoadGame:
		case F4SE::MessagingInterface::kGameLoaded:
			InitializeRuntime();
			break;
		case F4SE::MessagingInterface::kNewGame:
			ResetRuntimePhysics("new-game");
			InitializeRuntime();
			break;
		case F4SE::MessagingInterface::kGameDataReady:
			InitializeRuntime();
			break;
		default:
			break;
	}
}

F4SEPluginLoad(const F4SE::LoadInterface* a_f4se)
{
	F4SE::Init(a_f4se, {
		.log = true,
        .logName = "FO4FasterHdtSMP",
		.trampoline = true,
		.trampolineSize = 1 << 15,
    });

	spdlog::info("{} v{} ({}) loading", Version::PROJECT, Version::NAME, SimdVariantName());
	const auto runtimeVersion = a_f4se->RuntimeVersion();
	const auto executableVersion = REX::FModule::GetExecutingModule().GetFileVersion();
	spdlog::info(
		"detected Fallout 4 runtime={} f4seRuntimeVersion={} executableVersion={}",
		RuntimeName(),
		runtimeVersion.string(),
		executableVersion.string());

	if (!Hooks::InstallLifecycleHooks()) {
		spdlog::error("failed to install FO4 Faster HDT-SMP lifecycle hooks");
		return false;
	}

	const auto messaging = F4SE::GetMessagingInterface();
	if (!messaging || !messaging->RegisterListener(OnInit)) {
		spdlog::error("failed to register F4SE messaging listener");
		return false;
	}

    return true;
}

extern "C"
{
    F4SE_EXPORT bool F4SEPlugin_Query(const F4SE::QueryInterface*, F4SE::PluginInfo* a_info)
    {
		a_info->name = Version::PROJECT.data();
		a_info->infoVersion = F4SE::PluginInfo::kVersion;
        a_info->version = Version::MAJOR;
        return true;
    }
}
