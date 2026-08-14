#include "DefaultBBP.h"

#include "ConfigPaths.h"
#include "PhysicsName.h"
#include "ResourceFile.h"

#include "RE/N/NiStringExtraData.h"

#include <tinyxml2.h>

#include <vector>

namespace
{
	std::string ReadAttribute(tinyxml2::XMLElement* a_element, const char* a_name)
	{
		if (const auto value = a_element ? a_element->Attribute(a_name) : nullptr) {
			return value;
		}
		return {};
	}

	std::string ReadText(tinyxml2::XMLElement* a_element)
	{
		if (const auto text = a_element ? a_element->GetText() : nullptr) {
			return text;
		}
		return {};
	}

	void AddGeometryName(RE::NiAVObject* a_object, Smp::DefaultBBP::NameMap& a_names)
	{
		if (!a_object) {
			return;
		}

		if (const auto* geometry = a_object->IsGeometry()) {
			const std::string_view name(geometry->GetName());
			if (!name.empty()) {
				a_names[std::string(name)].insert(std::string(name));
			}
		}
	}

	void CollectDirectGeometryNames(RE::NiAVObject* a_parent, Smp::DefaultBBP::NameMap& a_names)
	{
		auto* node = a_parent ? a_parent->IsNode() : nullptr;
		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			AddGeometryName(child.get(), a_names);
		}
	}

	void CollectDefaultGeometryNames(RE::NiAVObject* a_root, Smp::DefaultBBP::NameMap& a_names)
	{
		CollectDirectGeometryNames(a_root, a_names);

		if (auto* faceGenSkinned = a_root ? a_root->GetObjectByName("BSFaceGenNiNodeSkinned") : nullptr;
			faceGenSkinned && faceGenSkinned != a_root) {
			CollectDirectGeometryNames(faceGenSkinned, a_names);
		}
	}

	auto FindNameMapEntry(Smp::DefaultBBP::NameMap& a_names, const std::string_view a_name)
	{
		return std::ranges::find_if(a_names, [a_name](const auto& a_entry) {
			return Smp::PhysicsNamesEqual(a_entry.first, a_name);
		});
	}

	auto FindNameMapEntry(const Smp::DefaultBBP::NameMap& a_names, const std::string_view a_name)
	{
		return std::ranges::find_if(a_names, [a_name](const auto& a_entry) {
			return Smp::PhysicsNamesEqual(a_entry.first, a_name);
		});
	}
}

namespace Smp
{
	DefaultBBP* DefaultBBP::GetSingleton()
	{
		static DefaultBBP singleton;
		return std::addressof(singleton);
	}

	void DefaultBBP::Reload()
	{
		loaded_ = false;
		maps_.clear();
		remaps_.clear();
		Load();
	}

	std::optional<DefaultBBP::Match> DefaultBBP::Find(RE::NiAVObject* a_object)
	{
		Load();
		if (!a_object || maps_.empty()) {
			return std::nullopt;
		}

		NameMap nameMap;
		CollectDefaultGeometryNames(a_object, nameMap);
		if (nameMap.empty()) {
			return std::nullopt;
		}

		for (const auto& remap : remaps_) {
			bool requirementsMet = true;
			for (const auto& required : remap.required) {
				if (FindNameMapEntry(nameMap, required) == nameMap.end()) {
					requirementsMet = false;
					break;
				}
			}
			if (!requirementsMet) {
				continue;
			}

			const auto source = std::find_if(remap.sources.rbegin(), remap.sources.rend(), [&](const auto& a_entry) {
				return FindNameMapEntry(nameMap, a_entry.second) != nameMap.end();
			});
			if (source == remap.sources.rend()) {
				continue;
			}

			auto& targetSet = nameMap[remap.target];
			for (auto it = source; it != remap.sources.rend() && it->first == source->first; ++it) {
				const auto mappedSource = FindNameMapEntry(nameMap, it->second);
				if (mappedSource != nameMap.end()) {
					targetSet.insert(mappedSource->second.begin(), mappedSource->second.end());
				}
			}
		}

		for (const auto& [shape, file] : maps_) {
			if (FindNameMapEntry(nameMap, shape) == nameMap.end()) {
				continue;
			}

			spdlog::info("defaultBBP selected {} for shape {}", file, shape);
			return Match{
				.physicsXml = std::filesystem::path(file),
				.meshNameMap = std::move(nameMap),
			};
		}

		return std::nullopt;
	}

	void DefaultBBP::Load()
	{
		if (loaded_) {
			return;
		}
		loaded_ = true;

		const auto candidates = Smp::ConfigPaths::MakeConfigPathCandidates("defaultBBPs.xml", true);
		const auto resource = Smp::ResourceFile::ReadFirst(candidates);
		if (!resource) {
			spdlog::debug("defaultBBP map not found at Data/F4SE/Plugins/FO4FasterHdtSMP/defaultBBPs.xml or Data/SKSE/Plugins/hdtSkinnedMeshConfigs/defaultBBPs.xml; direct XML/config matching only");
			return;
		}

		tinyxml2::XMLDocument document;
		const auto error = document.Parse(resource->bytes.data(), resource->bytes.size());
		if (error != tinyxml2::XML_SUCCESS) {
			spdlog::warn("failed to parse defaultBBP map {}: {}", resource->resourcePath, document.ErrorStr());
			return;
		}

		const auto root = document.FirstChildElement("default-bbps");
		if (!root) {
			spdlog::warn("defaultBBP map {} does not contain a <default-bbps> root", resource->resourcePath);
			return;
		}

		for (auto* child = root->FirstChildElement(); child; child = child->NextSiblingElement()) {
			const std::string_view name(child->Name());
			if (name == "map") {
				const auto shape = ReadAttribute(child, "shape");
				const auto file = ReadAttribute(child, "file");
				if (!shape.empty() && !file.empty()) {
					maps_[shape] = file;
				}
			} else if (name == "remap") {
				Remap remap;
				remap.target = ReadAttribute(child, "target");
				if (remap.target.empty()) {
					continue;
				}

				for (auto* remapChild = child->FirstChildElement(); remapChild; remapChild = remapChild->NextSiblingElement()) {
					const std::string_view childName(remapChild->Name());
					if (childName == "source") {
						int priority = 0;
						remapChild->QueryIntAttribute("priority", std::addressof(priority));
						const auto source = ReadText(remapChild);
						if (!source.empty()) {
							remap.sources.insert({ priority, source });
						}
					} else if (childName == "requires") {
						const auto required = ReadText(remapChild);
						if (!required.empty()) {
							remap.required.insert(required);
						}
					}
				}

				if (!remap.sources.empty()) {
					remaps_.push_back(std::move(remap));
				}
			}
		}

		spdlog::info(
			"loaded defaultBBP map {} maps={} remaps={} source={}",
			resource->resourcePath,
			maps_.size(),
			remaps_.size(),
			resource->fromArchive ? "archive" : "loose");
	}
}
