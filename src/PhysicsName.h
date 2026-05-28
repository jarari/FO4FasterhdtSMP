#pragma once

#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace Smp
{
	std::string NormalizePhysicsName(std::string_view a_name);
	bool PhysicsNamesEqual(std::string_view a_lhs, std::string_view a_rhs);
	std::optional<std::string_view> FindMatchingPhysicsName(std::span<const std::string> a_names, std::string_view a_name);
}
