#include "ResourceFile.h"

#include "ConfigPaths.h"

#include "RE/B/BSResourceNiBinaryStream.h"

#include <algorithm>
#include <cctype>
#include <unordered_set>

namespace
{
	bool StartsWithDataDirectory(const std::string_view a_path)
	{
		return a_path.size() >= 5 &&
			std::tolower(static_cast<unsigned char>(a_path[0])) == 'd' &&
			std::tolower(static_cast<unsigned char>(a_path[1])) == 'a' &&
			std::tolower(static_cast<unsigned char>(a_path[2])) == 't' &&
			std::tolower(static_cast<unsigned char>(a_path[3])) == 'a' &&
			(a_path[4] == '\\' || a_path[4] == '/');
	}
}

namespace Smp::ResourceFile
{
	std::string NormalizePath(const std::string_view a_path)
	{
		auto normalized = ConfigPaths::Trim(std::string(a_path));
		std::ranges::replace(normalized, '/', '\\');
		while (normalized.starts_with(".\\")) {
			normalized.erase(0, 2);
		}
		if (StartsWithDataDirectory(normalized)) {
			normalized.erase(0, 5);
		}
		return normalized;
	}

	std::string ComparisonKey(const std::string_view a_path)
	{
		auto key = NormalizePath(a_path);
		std::ranges::replace(key, '\\', '/');
		return ConfigPaths::LowerString(std::move(key));
	}

	bool PathsEqual(const std::string_view a_left, const std::string_view a_right)
	{
		return ComparisonKey(a_left) == ComparisonKey(a_right);
	}

	std::optional<Contents> Read(const std::string_view a_path)
	{
		auto resourcePath = NormalizePath(a_path);
		if (resourcePath.empty()) {
			return std::nullopt;
		}

		RE::BSResourceNiBinaryStream stream(resourcePath.c_str(), false, nullptr, true);
		if (!stream || !stream.stream) {
			return std::nullopt;
		}

		Contents contents{
			.resourcePath = std::move(resourcePath),
			.fromArchive = stream.stream->DoGetIsFromArchive(),
		};
		contents.bytes.resize(stream.stream->totalSize);
		if (!contents.bytes.empty() && !stream.read(contents.bytes.data(), contents.bytes.size())) {
			return std::nullopt;
		}
		return contents;
	}

	std::optional<Contents> ReadFirst(const std::span<const std::filesystem::path> a_candidates)
	{
		std::unordered_set<std::string> attempted;
		for (const auto& candidate : a_candidates) {
			const auto path = candidate.string();
			if (!attempted.insert(ComparisonKey(path)).second) {
				continue;
			}
			if (auto contents = Read(path)) {
				return contents;
			}
		}
		return std::nullopt;
	}
}
