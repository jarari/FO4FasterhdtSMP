#include "PhysicsName.h"

#include <algorithm>
#include <cctype>

namespace
{
	constexpr std::string_view kAutoRenamePrefix = "hdtSSEPhysics_AutoRename_";

	bool StartsWithInsensitive(const std::string_view a_value, const std::string_view a_prefix)
	{
		if (a_value.size() < a_prefix.size()) {
			return false;
		}

		for (std::size_t index = 0; index < a_prefix.size(); ++index) {
			const auto lhs = static_cast<unsigned char>(a_value[index]);
			const auto rhs = static_cast<unsigned char>(a_prefix[index]);
			if (std::tolower(lhs) != std::tolower(rhs)) {
				return false;
			}
		}

		return true;
	}
}

namespace Smp
{
	bool IsAutoRenamedPhysicsName(const std::string_view a_name)
	{
		return StartsWithInsensitive(a_name, kAutoRenamePrefix);
	}

	std::string NormalizePhysicsName(const std::string_view a_name)
	{
		auto value = a_name;
		if (IsAutoRenamedPhysicsName(value)) {
			if (const auto separator = value.find(' '); separator != std::string_view::npos && separator + 1 < value.size()) {
				value = value.substr(separator + 1);
			}
		}

		std::string result(value);
		std::ranges::transform(result, result.begin(), [](const unsigned char a_character) {
			return static_cast<char>(std::tolower(a_character));
		});
		return result;
	}

	bool PhysicsNamesEqual(const std::string_view a_lhs, const std::string_view a_rhs)
	{
		if (a_lhs.empty() || a_rhs.empty()) {
			return false;
		}

		return NormalizePhysicsName(a_lhs) == NormalizePhysicsName(a_rhs);
	}

	std::optional<std::string_view> FindMatchingPhysicsName(const std::span<const std::string> a_names, const std::string_view a_name)
	{
		const auto found = std::ranges::find_if(a_names, [a_name](const std::string& a_candidate) {
			return PhysicsNamesEqual(a_candidate, a_name);
		});
		if (found == a_names.end()) {
			return std::nullopt;
		}

		return std::string_view(*found);
	}
}
