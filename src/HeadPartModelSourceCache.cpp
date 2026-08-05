#include "HeadPartModelSourceCache.h"

#include "ResourceFile.h"

#include "RE/N/NiCloningProcess.h"

#include <mutex>
#include <unordered_map>

namespace Smp::HeadPartModelSourceCache
{
	namespace
	{
		std::mutex CacheLock;
		std::unordered_map<std::string, RE::NiPointer<RE::NiNode>> Cache;

		RE::NiPointer<RE::NiNode> CloneRoot(RE::NiNode* a_source)
		{
			if (!a_source) {
				return {};
			}

			RE::NiCloningProcess cloning{};
			cloning.copyType = RE::NiCloningProcess::CopyType::kCopyExact;
			cloning.scale = { 1.0F, 1.0F, 1.0F };
			auto* clonedObject = a_source->CreateClone(cloning);
			if (!clonedObject) {
				return {};
			}
			a_source->ProcessClone(cloning);
			return RE::NiPointer<RE::NiNode>(static_cast<RE::NiNode*>(clonedObject));
		}
	}

	RE::NiPointer<RE::NiNode> Capture(const std::string_view a_nifPath, RE::NiNode* a_loadedRoot)
	{
		const auto key = ResourceFile::ComparisonKey(a_nifPath);
		if (key.empty() || !a_loadedRoot) {
			return {};
		}

		std::scoped_lock lock(CacheLock);
		if (const auto found = Cache.find(key); found != Cache.end()) {
			return found->second;
		}

		auto snapshot = CloneRoot(a_loadedRoot);
		if (!snapshot) {
			return {};
		}

		Cache.emplace(key, snapshot);
		return snapshot;
	}

	RE::NiPointer<RE::NiNode> Find(const std::string_view a_nifPath)
	{
		const auto key = ResourceFile::ComparisonKey(a_nifPath);
		if (key.empty()) {
			return {};
		}

		std::scoped_lock lock(CacheLock);
		const auto found = Cache.find(key);
		return found != Cache.end() ? found->second : RE::NiPointer<RE::NiNode>{};
	}
}
