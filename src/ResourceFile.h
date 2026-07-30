#pragma once

#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace Smp::ResourceFile
{
	struct Contents
	{
		std::string resourcePath;
		std::string bytes;
		bool fromArchive{ false };
	};

	[[nodiscard]] std::string NormalizePath(std::string_view a_path);
	[[nodiscard]] std::string ComparisonKey(std::string_view a_path);
	[[nodiscard]] bool PathsEqual(std::string_view a_left, std::string_view a_right);
	[[nodiscard]] std::optional<Contents> Read(std::string_view a_path);
	[[nodiscard]] std::optional<Contents> ReadFirst(std::span<const std::filesystem::path> a_candidates);
}
