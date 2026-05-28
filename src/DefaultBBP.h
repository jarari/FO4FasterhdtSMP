#pragma once

#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace RE
{
	class NiAVObject;
}

namespace Smp
{
	class DefaultBBP
	{
	public:
		using NameSet = std::unordered_set<std::string>;
		using NameMap = std::unordered_map<std::string, NameSet>;

		struct Match
		{
			std::filesystem::path physicsXml;
			NameMap meshNameMap;
		};

		static DefaultBBP* GetSingleton();

		void Reload();
		std::optional<Match> Find(RE::NiAVObject* a_object);

	private:
		using RemapEntry = std::pair<int, std::string>;

		struct Remap
		{
			std::string target;
			std::set<RemapEntry> sources;
			std::unordered_set<std::string> required;
		};

		void Load();
		bool loaded_{ false };
		std::unordered_map<std::string, std::string> maps_;
		std::vector<Remap> remaps_;
	};
}
