#include "PhysicsXmlSelection.h"

#include "Fo4NiObjectUtils.h"
#include "PhysicsName.h"
#include "PhysicsXml.h"
#include "RE/B/BSGeometry.h"
#include "RE/N/NiAVObject.h"

namespace Smp::PhysicsXmlSelection
{
	namespace
	{
		constexpr std::string_view kPhysicsXmlExtraName = "HDT Skinned Mesh Physics Object";
	}

	std::optional<std::filesystem::path> FindDirectPhysicsXmlExtraData(
		RE::NiAVObject* a_object,
		const DirectXmlLogContext)
	{
		auto data = NiObject::FindStringExtraData(a_object, kPhysicsXmlExtraName);
		if (!data) {
			return std::nullopt;
		}

		// Keep the NIF-authored, Data-relative resource path intact. Existence is
		// checked by the resource-backed XML loader so missing paths remain
		// addressable by DynamicHDT and archived XML can be found in BA2 files.
		return std::filesystem::path(std::move(*data));
	}

	std::optional<ArmorSelection> FindArmorPhysicsXml(RE::NiAVObject* a_object)
	{
		if (auto directXml = FindDirectPhysicsXmlExtraData(a_object)) {
			return ArmorSelection{ .path = *directXml };
		}

		if (auto defaultBbp = DefaultBBP::GetSingleton()->Find(a_object)) {
			return ArmorSelection{
				.path = defaultBbp->physicsXml,
				.meshNameMap = std::move(defaultBbp->meshNameMap),
			};
		}

		return std::nullopt;
	}

	const DefaultBBP::NameSet* FindMeshAliases(const DefaultBBP::NameMap& a_meshNameMap, const std::string_view a_name)
	{
		const auto found = std::ranges::find_if(a_meshNameMap, [a_name](const auto& a_entry) {
			return PhysicsNamesEqual(a_entry.first, a_name);
		});
		return found == a_meshNameMap.end() ? nullptr : std::addressof(found->second);
	}

	bool MeshNameMatches(const std::string_view a_descriptorName, const std::string_view a_geometryName, const DefaultBBP::NameMap& a_meshNameMap)
	{
		if (PhysicsNamesEqual(a_descriptorName, a_geometryName)) {
			return true;
		}

		const auto* aliases = FindMeshAliases(a_meshNameMap, a_descriptorName);
		if (!aliases) {
			return false;
		}

		return std::ranges::any_of(*aliases, [a_geometryName](const std::string& a_alias) {
			return PhysicsNamesEqual(a_alias, a_geometryName);
		});
	}

	std::vector<std::string> BuildMeshMatchNames(const PhysicsXmlSummary& a_summary, const DefaultBBP::NameMap& a_meshNameMap)
	{
		std::vector<std::string> result;
		for (const auto& meshDescriptor : a_summary.meshDescriptors) {
			result.push_back(meshDescriptor.name);
			if (const auto* aliases = FindMeshAliases(a_meshNameMap, meshDescriptor.name)) {
				for (const auto& alias : *aliases) {
					if (!std::ranges::any_of(result, [&alias](const std::string& a_existing) {
							return PhysicsNamesEqual(a_existing, alias);
						})) {
						result.push_back(alias);
					}
				}
			}
		}
		return result;
	}
}
