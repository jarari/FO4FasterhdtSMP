#pragma once

#include <optional>
#include <functional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>

namespace Smp
{
	std::string NormalizePhysicsName(std::string_view a_name);
	bool IsAutoRenamedPhysicsName(std::string_view a_name);
	bool PhysicsNamesEqual(std::string_view a_lhs, std::string_view a_rhs);
	std::optional<std::string_view> FindMatchingPhysicsName(std::span<const std::string> a_names, std::string_view a_name);

	template <class TRange, class TProjection>
	auto FindByPhysicsName(TRange&& a_range, std::string_view a_name, TProjection a_projection)
	{
		return std::ranges::find_if(a_range, [a_name, a_projection](const auto& a_entry) {
			return PhysicsNamesEqual(std::invoke(a_projection, a_entry), a_name);
		});
	}

	template <class TRange, class TProjection>
	bool ContainsPhysicsName(TRange&& a_range, std::string_view a_name, TProjection a_projection)
	{
		return FindByPhysicsName(a_range, a_name, a_projection) != std::ranges::end(a_range);
	}
}
