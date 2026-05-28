#include "ConfigPaths.h"

#include <algorithm>
#include <cctype>

namespace Smp::ConfigPaths
{
	std::string Trim(std::string a_value)
	{
		a_value.erase(a_value.begin(), std::find_if(a_value.begin(), a_value.end(), [](const unsigned char a_character) {
			return !std::isspace(a_character);
		}));
		a_value.erase(std::find_if(a_value.rbegin(), a_value.rend(), [](const unsigned char a_character) {
			return !std::isspace(a_character);
		}).base(), a_value.end());
		return a_value;
	}

	std::string LowerString(std::string a_value)
	{
		std::ranges::transform(a_value, a_value.begin(), [](const unsigned char a_character) {
			return static_cast<char>(std::tolower(a_character));
		});
		return a_value;
	}

	bool PathExists(const std::filesystem::path& a_path)
	{
		std::error_code error;
		return std::filesystem::exists(a_path, error) && !error;
	}

	bool IsXmlPath(const std::filesystem::path& a_path)
	{
		return LowerString(a_path.extension().string()) == ".xml";
	}

	std::vector<std::filesystem::path> MakeConfigPathCandidates(std::filesystem::path a_raw, const bool a_includeLegacy)
	{
		std::vector<std::filesystem::path> candidates;
		candidates.push_back(a_raw);
		if (a_raw.is_relative()) {
			candidates.emplace_back(std::filesystem::path("Data") / a_raw);
			candidates.emplace_back(std::filesystem::path(kFo4PluginConfigPath) / a_raw);
			if (a_includeLegacy) {
				candidates.emplace_back(std::filesystem::path(kLegacySkyrimConfigPath) / a_raw);
			}
		}
		return candidates;
	}

	std::optional<std::filesystem::path> ResolveExistingConfigPath(const std::string_view a_value, const bool a_requireXml, const bool a_includeLegacy)
	{
		if (a_value.empty()) {
			return std::nullopt;
		}

		const std::filesystem::path raw(a_value);
		if (a_requireXml && !IsXmlPath(raw)) {
			return std::nullopt;
		}

		for (const auto& candidate : MakeConfigPathCandidates(raw, a_includeLegacy)) {
			if (PathExists(candidate)) {
				return candidate.lexically_normal();
			}
		}

		return std::nullopt;
	}

	std::filesystem::path ResolveConfigPath(const std::string_view a_value, const bool a_requireXml, const bool a_includeLegacy)
	{
		if (auto resolved = ResolveExistingConfigPath(a_value, a_requireXml, a_includeLegacy)) {
			return *resolved;
		}

		const std::filesystem::path raw(a_value);
		if (a_requireXml && !IsXmlPath(raw)) {
			return {};
		}

		return raw.lexically_normal();
	}
}
