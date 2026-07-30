#include "PapyrusFunctions.h"

#include "Fo4PhysicsWorld.h"
#include "ResourceFile.h"

#include <limits>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
	constexpr std::string_view kPapyrusClass = "DynamicHDT";
	constexpr std::uint32_t kSerializationVersion = 1;
	constexpr std::uint32_t kMaximumSerializedPathLength = 32U * 1024U;
	constexpr std::uint32_t kMaximumSerializedActorCount = 64U * 1024U;
	constexpr std::uint32_t kMaximumSerializedOverridesPerActor = 64U * 1024U;

	constexpr std::uint32_t MakeFourCC(const char a_a, const char a_b, const char a_c, const char a_d)
	{
		return static_cast<std::uint32_t>(static_cast<unsigned char>(a_a)) |
			(static_cast<std::uint32_t>(static_cast<unsigned char>(a_b)) << 8U) |
			(static_cast<std::uint32_t>(static_cast<unsigned char>(a_c)) << 16U) |
			(static_cast<std::uint32_t>(static_cast<unsigned char>(a_d)) << 24U);
	}

	constexpr std::uint32_t kSerializationID = MakeFourCC('F', 'H', 'S', 'P');
	constexpr std::uint32_t kOverrideRecord = MakeFourCC('O', 'V', 'R', 'D');

	using ActorOverrideMap = std::unordered_map<std::string, std::string>;
	using OverrideMap = std::unordered_map<std::uint32_t, ActorOverrideMap>;

	std::mutex overrideLock;
	OverrideMap physicsFileOverrides;

	void VerboseLog(const bool a_verbose, const std::string_view a_message)
	{
		if (!a_verbose) {
			return;
		}

		spdlog::info("[DynamicHDT] {}", a_message);
		if (auto* console = RE::ConsoleLog::GetSingleton()) {
			console->Log("[DynamicHDT] {}", a_message);
		}
	}

	bool ReloadPhysicsFile(
		std::monostate,
		RE::Actor* a_actor,
		RE::TESObjectARMA* a_armorAddon,
		const RE::BSFixedString a_physicsFilePath,
		const bool a_persist,
		const bool a_verbose)
	{
		if (!a_actor || !a_armorAddon) {
			VerboseLog(a_verbose, "ReloadPhysicsFile() could not parse Actor or ArmorAddon.");
			return false;
		}

		const auto succeeded = Smp::Fo4PhysicsWorld::GetSingleton()->ReloadPhysicsFile(
			a_actor,
			a_armorAddon,
			a_physicsFilePath.c_str(),
			a_persist,
			a_verbose);
		if (succeeded) {
			VerboseLog(a_verbose, "ReloadPhysicsFile() succeeded.");
		}
		return succeeded;
	}

	bool SwapPhysicsFile(
		std::monostate,
		RE::Actor* a_actor,
		const RE::BSFixedString a_oldPhysicsFilePath,
		const RE::BSFixedString a_newPhysicsFilePath,
		const bool a_persist,
		const bool a_verbose)
	{
		if (!a_actor) {
			VerboseLog(a_verbose, "SwapPhysicsFile() could not parse Actor.");
			return false;
		}

		const auto succeeded = Smp::Fo4PhysicsWorld::GetSingleton()->SwapPhysicsFile(
			a_actor,
			a_oldPhysicsFilePath.c_str(),
			a_newPhysicsFilePath.c_str(),
			a_persist,
			a_verbose);
		if (succeeded) {
			VerboseLog(a_verbose, "SwapPhysicsFile() succeeded.");
		}
		return succeeded;
	}

	RE::BSFixedString QueryCurrentPhysicsFile(
		std::monostate,
		RE::Actor* a_actor,
		RE::TESObjectARMA* a_armorAddon,
		const bool a_verbose)
	{
		if (!a_actor || !a_armorAddon) {
			VerboseLog(a_verbose, "QueryCurrentPhysicsFile() could not parse Actor or ArmorAddon.");
			return {};
		}

		auto path = Smp::Fo4PhysicsWorld::GetSingleton()->QueryCurrentPhysicsFile(a_actor, a_armorAddon, a_verbose);
		if (!path.empty()) {
			VerboseLog(a_verbose, "QueryCurrentPhysicsFile() succeeded.");
		}
		return RE::BSFixedString(path);
	}

	std::vector<bool> TogglePhysics(
		std::monostate,
		RE::Actor* a_actor,
		const std::vector<RE::BSFixedString> a_boneNames,
		const bool a_on)
	{
		if (!a_actor || a_boneNames.empty()) {
			return {};
		}

		std::vector<std::string> boneNames;
		boneNames.reserve(a_boneNames.size());
		for (const auto& name : a_boneNames) {
			boneNames.emplace_back(name.c_str());
		}
		return Smp::Fo4PhysicsWorld::GetSingleton()->TogglePhysics(a_actor, boneNames, a_on);
	}

	void ResetPhysics(std::monostate, RE::Actor* a_actor, const bool a_full)
	{
		Smp::Papyrus::QueueResetPhysics(a_actor, a_full);
	}

	bool F4SEAPI RegisterPapyrusFunctions(RE::BSScript::IVirtualMachine* a_vm)
	{
		if (!a_vm) {
			return false;
		}

		a_vm->BindNativeMethod(kPapyrusClass, "ReloadPhysicsFile", ReloadPhysicsFile);
		a_vm->BindNativeMethod(kPapyrusClass, "SwapPhysicsFile", SwapPhysicsFile);
		a_vm->BindNativeMethod(kPapyrusClass, "QueryCurrentPhysicsFile", QueryCurrentPhysicsFile);
		a_vm->BindNativeMethod(kPapyrusClass, "TogglePhysics", TogglePhysics);
		a_vm->BindNativeMethod(kPapyrusClass, "ResetPhysics", ResetPhysics);
		spdlog::info("registered DynamicHDT Papyrus functions");
		return true;
	}

	template <class T>
	bool WriteValue(const F4SE::SerializationInterface* a_intfc, const T& a_value)
	{
		return a_intfc && a_intfc->WriteRecordData(a_value);
	}

	bool WriteString(const F4SE::SerializationInterface* a_intfc, const std::string& a_value)
	{
		if (a_value.size() > std::numeric_limits<std::uint32_t>::max()) {
			return false;
		}

		const auto size = static_cast<std::uint32_t>(a_value.size());
		return WriteValue(a_intfc, size) &&
			(size == 0 || a_intfc->WriteRecordData(a_value.data(), size));
	}

	template <class T>
	bool ReadValue(const F4SE::SerializationInterface* a_intfc, T& a_value)
	{
		return a_intfc && a_intfc->ReadRecordData(a_value) == sizeof(T);
	}

	bool ReadString(const F4SE::SerializationInterface* a_intfc, std::string& a_value)
	{
		std::uint32_t size = 0;
		if (!ReadValue(a_intfc, size) || size > kMaximumSerializedPathLength) {
			return false;
		}

		a_value.resize(size);
		return size == 0 || a_intfc->ReadRecordData(a_value.data(), size) == size;
	}

	void F4SEAPI SaveOverrides(const F4SE::SerializationInterface* a_intfc)
	{
		OverrideMap snapshot;
		{
			std::scoped_lock lock(overrideLock);
			snapshot = physicsFileOverrides;
		}

		if (!a_intfc->OpenRecord(kOverrideRecord, kSerializationVersion)) {
			spdlog::error("failed to open DynamicHDT override serialization record");
			return;
		}

		const auto actorCount = static_cast<std::uint32_t>(snapshot.size());
		if (!WriteValue(a_intfc, actorCount)) {
			spdlog::error("failed to serialize DynamicHDT override actor count");
			return;
		}

		for (const auto& [actorFormID, overrides] : snapshot) {
			const auto overrideCount = static_cast<std::uint32_t>(overrides.size());
			if (!WriteValue(a_intfc, actorFormID) || !WriteValue(a_intfc, overrideCount)) {
				spdlog::error("failed to serialize DynamicHDT override header for actor {:08X}", actorFormID);
				return;
			}
			for (const auto& [originalPath, overridePath] : overrides) {
				if (!WriteString(a_intfc, originalPath) || !WriteString(a_intfc, overridePath)) {
					spdlog::error("failed to serialize DynamicHDT override paths for actor {:08X}", actorFormID);
					return;
				}
			}
		}
	}

	void F4SEAPI LoadOverrides(const F4SE::SerializationInterface* a_intfc)
	{
		OverrideMap loadedOverrides;
		std::size_t loadedActorCount = 0;
		std::uint32_t type = 0;
		std::uint32_t version = 0;
		std::uint32_t length = 0;
		while (a_intfc->GetNextRecordInfo(type, version, length)) {
			if (type != kOverrideRecord) {
				spdlog::warn("skipping unknown FO4 Faster HDT-SMP serialization record {:08X}", type);
				continue;
			}
			if (version != kSerializationVersion) {
				spdlog::warn("skipping unsupported DynamicHDT override serialization version {}", version);
				continue;
			}

			std::uint32_t actorCount = 0;
			if (!ReadValue(a_intfc, actorCount) || actorCount > kMaximumSerializedActorCount) {
				spdlog::error("failed to deserialize DynamicHDT override actor count");
				return;
			}

			for (std::uint32_t actorIndex = 0; actorIndex < actorCount; ++actorIndex) {
				std::uint32_t savedActorFormID = 0;
				std::uint32_t overrideCount = 0;
				if (!ReadValue(a_intfc, savedActorFormID) ||
					!ReadValue(a_intfc, overrideCount) ||
					overrideCount > kMaximumSerializedOverridesPerActor) {
					spdlog::error("failed to deserialize DynamicHDT override actor header");
					return;
				}

				const auto resolvedActorFormID = a_intfc->ResolveFormID(savedActorFormID);
				for (std::uint32_t overrideIndex = 0; overrideIndex < overrideCount; ++overrideIndex) {
					std::string originalPath;
					std::string overridePath;
					if (!ReadString(a_intfc, originalPath) || !ReadString(a_intfc, overridePath)) {
						spdlog::error("failed to deserialize DynamicHDT override paths");
						return;
					}
					if (resolvedActorFormID && !originalPath.empty() && !overridePath.empty()) {
						auto& actorOverrides = loadedOverrides[*resolvedActorFormID];
						const auto existing = std::ranges::find_if(actorOverrides, [&](const auto& a_entry) {
							return Smp::ResourceFile::PathsEqual(a_entry.first, originalPath);
						});
						if (existing != actorOverrides.end()) {
							existing->second = std::move(overridePath);
						} else {
							actorOverrides[std::move(originalPath)] = std::move(overridePath);
						}
					}
				}
			}
		}

		{
			std::scoped_lock lock(overrideLock);
			physicsFileOverrides = std::move(loadedOverrides);
			loadedActorCount = physicsFileOverrides.size();
		}
		spdlog::info("loaded DynamicHDT physics-file overrides for {} actors", loadedActorCount);
	}

	void F4SEAPI RevertOverrides(const F4SE::SerializationInterface*)
	{
		std::scoped_lock lock(overrideLock);
		physicsFileOverrides.clear();
	}
}

namespace Smp::Papyrus
{
	void QueueResetPhysics(RE::Actor* a_actor, const bool a_full)
	{
		if (!a_actor) {
			return;
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		if (!handle) {
			return;
		}

		if (const auto* tasks = F4SE::GetTaskInterface()) {
			tasks->AddTask([handle, a_full]() mutable {
				if (auto actor = handle.get()) {
					Fo4PhysicsWorld::GetSingleton()->ResetActorPhysics(actor.get(), a_full);
				}
			});
		}
	}

	bool Register()
	{
		const auto* papyrus = F4SE::GetPapyrusInterface();
		return papyrus && papyrus->Register(RegisterPapyrusFunctions);
	}

	bool RegisterSerialization()
	{
		const auto* serialization = F4SE::GetSerializationInterface();
		if (!serialization) {
			return false;
		}

		serialization->SetUniqueID(kSerializationID);
		serialization->SetSaveCallback(SaveOverrides);
		serialization->SetLoadCallback(LoadOverrides);
		serialization->SetRevertCallback(RevertOverrides);
		return true;
	}

	bool RegisterPhysicsFileOverride(
		const std::uint32_t a_actorFormID,
		std::string a_oldPhysicsFilePath,
		std::string a_newPhysicsFilePath)
	{
		if (a_actorFormID == 0 || a_oldPhysicsFilePath.empty()) {
			return false;
		}

		std::scoped_lock lock(overrideLock);
		auto& actorOverrides = physicsFileOverrides[a_actorFormID];
		auto storedOriginalPath = std::move(a_oldPhysicsFilePath);
		auto existing = std::ranges::find_if(actorOverrides, [&](const auto& a_entry) {
			return ResourceFile::PathsEqual(a_entry.first, storedOriginalPath);
		});
		if (existing == actorOverrides.end()) {
			existing = std::ranges::find_if(actorOverrides, [&](const auto& a_entry) {
				return ResourceFile::PathsEqual(a_entry.second, storedOriginalPath);
			});
		}
		if (existing != actorOverrides.end()) {
			storedOriginalPath = existing->first;
		}
		actorOverrides[std::move(storedOriginalPath)] = std::move(a_newPhysicsFilePath);
		return true;
	}

	std::optional<std::string> ResolvePhysicsFileOverride(
		const std::uint32_t a_actorFormID,
		const std::string_view a_physicsFilePath)
	{
		std::scoped_lock lock(overrideLock);
		const auto actor = physicsFileOverrides.find(a_actorFormID);
		if (actor == physicsFileOverrides.end()) {
			return std::nullopt;
		}

		const auto override = std::ranges::find_if(actor->second, [&](const auto& a_entry) {
			return ResourceFile::PathsEqual(a_entry.first, a_physicsFilePath);
		});
		if (override == actor->second.end() || override->second.empty()) {
			return std::nullopt;
		}
		return override->second;
	}

	std::string QueryPhysicsFileOverrides()
	{
		std::scoped_lock lock(overrideLock);

		std::string result("[DynamicHDT] -- Querying existing override data...\n");
		for (const auto& [actorFormID, overrides] : physicsFileOverrides) {
			result += std::format("Actor formID: {:08X}\t{}\n", actorFormID, overrides.size());
			for (const auto& [originalPath, overridePath] : overrides) {
				result += std::format("\tOriginal file: {}\n\t\t| Override: {}\n", originalPath, overridePath);
			}
		}
		result += "[DynamicHDT] -- Query finished...\n";
		return result;
	}
}
