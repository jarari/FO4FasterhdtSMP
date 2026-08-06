#include "Hooks.h"
#include "BSSkin.h"
#include "DefaultBBP.h"
#include "Fo4PhysicsWorld.h"
#include "ImguiLayer.h"
#include "LifecycleEvents.h"
#include "PapyrusFunctions.h"
#include "PhysicsXml.h"
#include "SmpConfig.h"

#include <array>
#include <charconv>
#include <cctype>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

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
		Smp::Fo4PhysicsWorld::GetSingleton()->ApplyConfig(settings);
		Smp::ImguiLayer::SetEnabled(settings.smp.enableBulletVisualization);
		Smp::ImguiLayer::SetDrawCallback(settings.smp.enableBulletVisualization ? &DrawBulletVisualization : nullptr);
	}

	void ResetRuntimePhysics(std::string_view a_reason)
	{
		spdlog::info("resetting FO4 Faster HDT-SMP physics state: {}", a_reason);
		Smp::Fo4PhysicsWorld::GetSingleton()->Reset();
	}

	std::uint64_t ParsePositiveDecimal(const std::string_view a_value, const std::uint64_t a_fallback)
	{
		if (a_value.empty()) {
			return a_fallback;
		}

		std::uint64_t parsed = 0;
		const auto [end, error] = std::from_chars(a_value.data(), a_value.data() + a_value.size(), parsed);
		return error == std::errc{} && end == a_value.data() + a_value.size() && parsed != 0 ?
			parsed :
			a_fallback;
	}

	std::string_view TrimConsoleArgument(std::string_view a_value)
	{
		while (!a_value.empty() && std::isspace(static_cast<unsigned char>(a_value.front()))) {
			a_value.remove_prefix(1);
		}
		while (!a_value.empty() && std::isspace(static_cast<unsigned char>(a_value.back()))) {
			a_value.remove_suffix(1);
		}
		return a_value;
	}

	bool ConsoleArgumentsEqual(const std::string_view a_lhs, const std::string_view a_rhs)
	{
		return a_lhs.size() == a_rhs.size() &&
			std::ranges::equal(a_lhs, a_rhs, [](const char a_lhsCharacter, const char a_rhsCharacter) {
				return std::tolower(static_cast<unsigned char>(a_lhsCharacter)) ==
					std::tolower(static_cast<unsigned char>(a_rhsCharacter));
			});
	}

	std::optional<bool> ParseConsoleBoolean(
		const std::string_view a_value,
		const std::string_view a_trueValue,
		const std::string_view a_falseValue)
	{
		const auto value = TrimConsoleArgument(a_value);
		if (ConsoleArgumentsEqual(value, a_trueValue)) {
			return true;
		}
		if (ConsoleArgumentsEqual(value, a_falseValue)) {
			return false;
		}
		return std::nullopt;
	}

	std::optional<RE::TESFormID> ParseConsoleFormID(std::string_view a_value)
	{
		a_value = TrimConsoleArgument(a_value);
		if (a_value.starts_with("0x") || a_value.starts_with("0X")) {
			a_value.remove_prefix(2);
		}
		if (a_value.empty()) {
			return std::nullopt;
		}

		RE::TESFormID formID = 0;
		const auto [end, error] = std::from_chars(
			a_value.data(),
			a_value.data() + a_value.size(),
			formID,
			16);
		if (error != std::errc{} || end != a_value.data() + a_value.size()) {
			return std::nullopt;
		}
		return formID;
	}

	std::vector<std::string> ParseConsoleBoneNames(const std::string_view a_value)
	{
		std::vector<std::string> boneNames;
		for (std::size_t begin = 0; begin <= a_value.size();) {
			const auto separator = a_value.find(',', begin);
			const auto end = separator == std::string_view::npos ? a_value.size() : separator;
			const auto boneName = TrimConsoleArgument(a_value.substr(begin, end - begin));
			if (!boneName.empty()) {
				boneNames.emplace_back(boneName);
			}
			if (separator == std::string_view::npos) {
				break;
			}
			begin = separator + 1;
		}
		return boneNames;
	}

	RE::Actor* ResolveConsoleActor(RE::TESObjectREFR* a_refObject)
	{
		auto* actor = a_refObject ? a_refObject->As<RE::Actor>() : nullptr;
		if (!actor) {
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				console->Log("[HDT-SMP] error: select an actor before running this command");
			}
		}
		return actor;
	}

	void DumpNodeChildren(RE::NiAVObject* a_node)
	{
		if (!a_node) {
			spdlog::info("targeted reference has no loaded 3D");
			return;
		}

		const auto* rtti = a_node->GetRTTI();
		spdlog::info(
			"{} {} [{:.2f}, {:.2f}, {:.2f}]",
			rtti ? rtti->GetName() : "unk_type",
			a_node->name.c_str(),
			a_node->world.translate.x,
			a_node->world.translate.y,
			a_node->world.translate.z);

		if (a_node->extra) {
			for (const auto* extraData : *a_node->extra) {
				if (!extraData) {
					continue;
				}
				const auto* extraRtti = extraData->GetRTTI();
				spdlog::info(
					"{} {}",
					extraRtti ? extraRtti->GetName() : "unk_type",
					extraData->name.c_str());
			}
		}

		const auto* node = a_node->IsNode();
		if (!node) {
			return;
		}

		for (const auto& child : node->children) {
			auto* object = child.get();
			if (!object) {
				continue;
			}

			if (const auto* geometry = object->IsGeometry()) {
				const auto* geometryRtti = geometry->GetRTTI();
				spdlog::info(
					"{} {} [{:.2f}, {:.2f}, {:.2f}] - Geometry",
					geometryRtti ? geometryRtti->GetName() : "unk_type",
					geometry->name.c_str(),
					geometry->world.translate.x,
					geometry->world.translate.y,
					geometry->world.translate.z);

				if (geometry->skinInstance && geometry->skinInstance->bones.size() <= RE::BSSkin::kMaxExpectedBones) {
					for (std::uint32_t boneIndex = 0; boneIndex < geometry->skinInstance->bones.size(); ++boneIndex) {
						const auto* bone = geometry->skinInstance->bones[boneIndex];
						if (!bone) {
							continue;
						}
						const auto* boneRtti = bone->GetRTTI();
						spdlog::info(
							"Bone {} - {} {} [{:.2f}, {:.2f}, {:.2f}]",
							boneIndex,
							boneRtti ? boneRtti->GetName() : "unk_type",
							bone->name.c_str(),
							bone->world.translate.x,
							bone->world.translate.y,
							bone->world.translate.z);
					}
				}

				for (const auto& property : geometry->properties) {
					auto* lightingShader = property ?
						netimmerse_cast<RE::BSLightingShaderProperty*>(property.get()) :
						nullptr;
					if (!lightingShader || !lightingShader->material ||
						lightingShader->material->GetType() != RE::BSShaderMaterial::Type::kLighting) {
						continue;
					}

					auto* material = static_cast<RE::BSLightingShaderMaterialBase*>(lightingShader->material);
					std::array<RE::NiTexture*, std::to_underlying(RE::BSShaderProperty::TextureTypeEnum::kTotal)> textures{};
					const auto textureCount = material->GetTextures(textures.data(), static_cast<std::uint32_t>(textures.size()));
					if (material->textureSet) {
						for (std::uint32_t textureIndex = 0; textureIndex < textures.size(); ++textureIndex) {
							const auto textureType = static_cast<RE::BSShaderProperty::TextureTypeEnum>(textureIndex);
							const auto* texturePath = material->textureSet->GetTextureFilename(textureType);
							if (!texturePath || texturePath[0] == '\0') {
								continue;
							}

							const auto* texture = textureIndex < textureCount ? textures[textureIndex] : nullptr;
							spdlog::info(
								"Texture {} - {} ({})",
								textureIndex,
								texturePath,
								texture ? texture->name.c_str() : "");
						}
					}
					spdlog::info("Flags - {:016X}", lightingShader->flags.underlying());
				}
			} else if (object->IsNode()) {
				DumpNodeChildren(object);
			} else {
				const auto* objectRtti = object->GetRTTI();
				spdlog::info(
					"{} {} [{:.2f}, {:.2f}, {:.2f}]",
					objectRtti ? objectRtti->GetName() : "unk_type",
					object->name.c_str(),
					object->world.translate.x,
					object->world.translate.y,
					object->world.translate.z);
			}
		}
	}

	bool ExecuteSmpConsoleCommand(
		const RE::SCRIPT_PARAMETER* a_parameters,
		const char* a_compiledParams,
		RE::TESObjectREFR* a_refObject,
		RE::TESObjectREFR* a_container,
		RE::Script* a_script,
		RE::ScriptLocals* a_scriptLocals,
		[[maybe_unused]] float& a_returnValue,
		std::uint32_t& a_offset)
	{
		constexpr std::size_t argumentBufferSize = 260;
		std::array<char, argumentBufferSize> subcommand{};
		std::array<char, argumentBufferSize> argument1{};
		std::array<char, argumentBufferSize> argument2{};
		std::array<char, argumentBufferSize> argument3{};
		if (!RE::Script::ParseParameters(
				a_parameters,
				a_compiledParams,
				a_offset,
				a_refObject,
				a_container,
				a_script,
				a_scriptLocals,
				subcommand.data(),
				argument1.data(),
				argument2.data(),
				argument3.data())) {
			return false;
		}

		spdlog::debug(
			"smp console command: subcommand='{}' argument1='{}' argument2='{}' argument3='{}'",
			subcommand.data(),
			argument1.data(),
			argument2.data(),
			argument3.data());

		if (_stricmp(subcommand.data(), "reset") == 0) {
			spdlog::debug("smp reset: reloading config and resetting physics world");
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				console->Log("running full smp reset");
			}

			LoadRuntimeConfig();
			Smp::Fo4PhysicsWorld::GetSingleton()->ResetSystems();
			return true;
		}

		if (_stricmp(subcommand.data(), "dumptree") == 0) {
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				if (a_refObject) {
					console->Log("dumping targeted reference's node tree");
					DumpNodeChildren(a_refObject->Get3D());
				} else {
					console->Log("error: you must target a reference to dump their node tree");
				}
			}
			return true;
		}

		auto* physicsWorld = Smp::Fo4PhysicsWorld::GetSingleton();
		if (_stricmp(subcommand.data(), "ReloadPhysicsFile") == 0) {
			auto* actor = ResolveConsoleActor(a_refObject);
			const auto armorAddonFormID = ParseConsoleFormID(argument1.data());
			const auto persist = ParseConsoleBoolean(argument3.data(), "true", "false");
			auto* armorAddon = armorAddonFormID ?
				RE::TESForm::GetFormByID<RE::TESObjectARMA>(*armorAddonFormID) :
				nullptr;
			if (!actor || !armorAddon || argument2[0] == '\0' || !persist) {
				if (auto* console = RE::ConsoleLog::GetSingleton()) {
					console->Log(
						"[HDT-SMP] usage: smp ReloadPhysicsFile <ARMA form id> <xml path> true/false");
					if (armorAddonFormID && !armorAddon) {
						console->Log("[HDT-SMP] error: {:08X} is not a loaded ArmorAddon form", *armorAddonFormID);
					}
				}
				a_returnValue = 0.0F;
				return true;
			}

			const auto succeeded = physicsWorld->ReloadPhysicsFile(
				actor,
				armorAddon,
				argument2.data(),
				*persist,
				true);
			a_returnValue = succeeded ? 1.0F : 0.0F;
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				console->Log(
					"[HDT-SMP] ReloadPhysicsFile {}",
					succeeded ? "succeeded" : "failed");
			}
			return true;
		}

		if (_stricmp(subcommand.data(), "SwapPhysicsFile") == 0) {
			auto* actor = ResolveConsoleActor(a_refObject);
			const auto persist = ParseConsoleBoolean(argument3.data(), "true", "false");
			if (!actor || argument1[0] == '\0' || argument2[0] == '\0' || !persist) {
				if (auto* console = RE::ConsoleLog::GetSingleton()) {
					console->Log(
						"[HDT-SMP] usage: smp SwapPhysicsFile <old path> <new path> true/false");
				}
				a_returnValue = 0.0F;
				return true;
			}

			const auto succeeded = physicsWorld->SwapPhysicsFile(
				actor,
				argument1.data(),
				argument2.data(),
				*persist,
				true);
			a_returnValue = succeeded ? 1.0F : 0.0F;
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				console->Log(
					"[HDT-SMP] SwapPhysicsFile {}",
					succeeded ? "succeeded" : "failed");
			}
			return true;
		}

		if (_stricmp(subcommand.data(), "QueryCurrentPhysicsFile") == 0) {
			auto* actor = ResolveConsoleActor(a_refObject);
			const auto armorAddonFormID = ParseConsoleFormID(argument1.data());
			auto* armorAddon = armorAddonFormID ?
				RE::TESForm::GetFormByID<RE::TESObjectARMA>(*armorAddonFormID) :
				nullptr;
			if (!actor || !armorAddon) {
				if (auto* console = RE::ConsoleLog::GetSingleton()) {
					console->Log(
						"[HDT-SMP] usage: smp QueryCurrentPhysicsFile <ARMA form id>");
					if (armorAddonFormID && !armorAddon) {
						console->Log("[HDT-SMP] error: {:08X} is not a loaded ArmorAddon form", *armorAddonFormID);
					}
				}
				a_returnValue = 0.0F;
				return true;
			}

			const auto physicsFilePath = physicsWorld->QueryCurrentPhysicsFile(actor, armorAddon, true);
			a_returnValue = physicsFilePath.empty() ? 0.0F : 1.0F;
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				console->Log(
					"[HDT-SMP] current physics file: {}",
					physicsFilePath.empty() ? "<none>" : physicsFilePath);
			}
			return true;
		}

		if (_stricmp(subcommand.data(), "TogglePhysics") == 0) {
			auto* actor = ResolveConsoleActor(a_refObject);
			auto boneNames = ParseConsoleBoneNames(argument1.data());
			const auto on = ParseConsoleBoolean(argument2.data(), "on", "off");
			if (!actor || boneNames.empty() || !on) {
				if (auto* console = RE::ConsoleLog::GetSingleton()) {
					console->Log(
						"[HDT-SMP] usage: smp TogglePhysics <comma separated bone names> on/off");
				}
				a_returnValue = 0.0F;
				return true;
			}

			const auto previousStates = physicsWorld->TogglePhysics(actor, boneNames, *on);
			a_returnValue = 1.0F;
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				for (std::size_t index = 0; index < boneNames.size(); ++index) {
					const auto previousState =
						index < previousStates.size() && previousStates[index];
					console->Log(
						"[HDT-SMP] {}: previous state {}",
						boneNames[index],
						previousState ? "on" : "off/not found");
				}
			}
			return true;
		}

		if (_stricmp(subcommand.data(), "ResetPhysics") == 0) {
			auto* actor = ResolveConsoleActor(a_refObject);
			const auto full = ParseConsoleBoolean(argument1.data(), "true", "false");
			if (!actor || !full) {
				if (auto* console = RE::ConsoleLog::GetSingleton()) {
					console->Log("[HDT-SMP] usage: smp ResetPhysics true/false");
				}
				a_returnValue = 0.0F;
				return true;
			}

			Smp::Papyrus::QueueResetPhysics(actor, *full);
			a_returnValue = 1.0F;
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				console->Log(
					"[HDT-SMP] queued {} physics reset for selected actor",
					*full ? "full" : "soft");
			}
			return true;
		}

		if (_stricmp(subcommand.data(), "detail") == 0) {
			physicsWorld->PrintConsoleDetails(true);
			return true;
		}

		if (_stricmp(subcommand.data(), "list") == 0) {
			physicsWorld->PrintConsoleDetails(false);
			return true;
		}

		if (_stricmp(subcommand.data(), "profile") == 0) {
			static bool profilerCaptureRequested = false;
			profilerCaptureRequested = !profilerCaptureRequested;

			const auto sampleFrames = ParsePositiveDecimal(argument1.data(), 240);
			const auto printFrames = ParsePositiveDecimal(argument2.data(), 240);
			physicsWorld->SetProfilerCapture(profilerCaptureRequested, sampleFrames, printFrames);

			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				if (profilerCaptureRequested) {
					console->Log(
						"HDT-SMP physics profiler enabled: sample {} frames, print every {} frames",
						sampleFrames,
						printFrames);
					console->Log("Check your FO4FasterHdtSMP.log file for results.");
				} else {
					console->Log("HDT-SMP physics profiler disabled");
				}
			}
			return true;
		}

		if (_stricmp(subcommand.data(), "on") == 0) {
			physicsWorld->SetDisabled(false);
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				console->Log("HDT-SMP enabled");
			}
			return true;
		}

		if (_stricmp(subcommand.data(), "off") == 0) {
			physicsWorld->SetDisabled(true);
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				console->Log("HDT-SMP disabled");
			}
			return true;
		}

		if (_stricmp(subcommand.data(), "QueryOverride") == 0) {
			if (auto* console = RE::ConsoleLog::GetSingleton()) {
				const auto overrides = Smp::Papyrus::QueryPhysicsFileOverrides();
				console->AddString(overrides.c_str());
			}
			return true;
		}

		physicsWorld->PrintConsoleSummary();
		return true;
	}

	bool InstallSmpConsoleCommand()
	{
		auto* command = RE::SCRIPT_FUNCTION::LocateConsoleCommand("ShowRenderPasses");
		if (!command) {
			spdlog::error("could not locate the ShowRenderPasses console command slot");
			return false;
		}

		static RE::SCRIPT_PARAMETER parameters[4]{};
		for (auto& parameter : parameters) {
			parameter.paramType = RE::SCRIPT_PARAM_TYPE::kChar;
			parameter.paramName = "String (optional)";
			parameter.optional = true;
		}

		command->functionName = "SMPDebug";
		command->shortName = "smp";
		command->helpString = "smp <command> [arguments]";
		command->referenceFunction = false;
		command->SetParameters(parameters);
		command->executeFunction = ExecuteSmpConsoleCommand;
		command->editorFilter = false;
		command->invalidatesCellList = false;

		spdlog::info("registered SMPDebug (smp) using the ShowRenderPasses console command slot");
		return true;
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
			Smp::Fo4PhysicsWorld::GetSingleton()->BeginSaveLoad();
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

	const auto runtimeVersion = a_f4se->RuntimeVersion();
	const auto executableVersion = REX::FModule::GetExecutingModule().GetFileVersion();
	spdlog::info(
		"detected Fallout 4 runtime={} f4seRuntimeVersion={} executableVersion={}",
		RuntimeName(),
		runtimeVersion.string(),
		executableVersion.string());

	if (!Smp::Papyrus::Register()) {
		spdlog::error("failed to register DynamicHDT Papyrus functions");
		return false;
	}
	if (!Smp::Papyrus::RegisterSerialization()) {
		spdlog::error("failed to register DynamicHDT F4SE serialization");
		return false;
	}

	if (!Hooks::InstallLifecycleHooks()) {
		spdlog::error("failed to install FO4 Faster HDT-SMP lifecycle hooks");
		return false;
	}
	if (!InstallSmpConsoleCommand()) {
		spdlog::warn("FO4 Faster HDT-SMP console commands will be unavailable");
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
		const auto* versionData = F4SE::PluginVersionData::GetSingleton();
		if (!versionData) {
			return false;
		}

		a_info->name = versionData->GetPluginName().data();
		a_info->infoVersion = F4SE::PluginInfo::kVersion;
		a_info->version = versionData->pluginVersion;
        return true;
    }
}
