#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace Smp::ConfigPaths
{
	inline constexpr auto kFo4PluginConfigPath = "Data/F4SE/Plugins/FO4FasterHdtSMP";
	inline constexpr auto kLegacySkyrimConfigPath = "Data/SKSE/Plugins/hdtSkinnedMeshConfigs";

	std::string Trim(std::string a_value);
	std::string LowerString(std::string a_value);
	bool PathExists(const std::filesystem::path& a_path);
	bool IsXmlPath(const std::filesystem::path& a_path);

	std::vector<std::filesystem::path> MakeConfigPathCandidates(std::filesystem::path a_raw, bool a_includeLegacy = true);
	std::optional<std::filesystem::path> ResolveExistingConfigPath(std::string_view a_value, bool a_requireXml = false, bool a_includeLegacy = true);
	std::filesystem::path ResolveConfigPath(std::string_view a_value, bool a_requireXml = false, bool a_includeLegacy = true);
}
