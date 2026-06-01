#pragma once

#include "DefaultBBP.h"

#include <filesystem>
#include <optional>
#include <string_view>
#include <vector>

namespace RE
{
	class BSGeometry;
	class NiAVObject;
}

namespace Smp
{
	struct PhysicsXmlSummary;
}

namespace Smp::PhysicsXmlSelection
{
	enum class DirectXmlLogContext
	{
		kObject,
		kOriginalModelObject
	};

	struct ArmorSelection
	{
		std::filesystem::path path;
		DefaultBBP::NameMap meshNameMap;
	};

	std::optional<std::filesystem::path> FindDirectPhysicsXmlExtraData(
		RE::NiAVObject* a_object,
		DirectXmlLogContext a_context = DirectXmlLogContext::kObject);
	std::optional<ArmorSelection> FindArmorPhysicsXml(RE::NiAVObject* a_object);
	const DefaultBBP::NameSet* FindMeshAliases(const DefaultBBP::NameMap& a_meshNameMap, std::string_view a_name);
	bool MeshNameMatches(std::string_view a_descriptorName, std::string_view a_geometryName, const DefaultBBP::NameMap& a_meshNameMap);
	std::vector<std::string> BuildMeshMatchNames(const PhysicsXmlSummary& a_summary, const DefaultBBP::NameMap& a_meshNameMap);
}
