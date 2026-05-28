#include "Fo4MeshExtractor.h"

#include "BSSkin.h"
#include "PhysicsName.h"
#include "RE/B/BSGraphics.h"
#include "RE/B/BSTriShape.h"

#include <array>

namespace
{
	hdt::btQsTransform ToQsTransform(const RE::NiTransform& a_transform)
	{
		btMatrix3x3 basis(
			a_transform.rotate[0].x,
			a_transform.rotate[0].y,
			a_transform.rotate[0].z,
			a_transform.rotate[1].x,
			a_transform.rotate[1].y,
			a_transform.rotate[1].z,
			a_transform.rotate[2].x,
			a_transform.rotate[2].y,
			a_transform.rotate[2].z);

		btQuaternion rotation = btQuaternion::getIdentity();
		basis.getRotation(rotation);
		if (rotation.length2() <= FLT_EPSILON) {
			rotation = btQuaternion::getIdentity();
		} else {
			rotation.normalize();
		}

		return hdt::btQsTransform(
			rotation,
			btVector3(a_transform.translate.x, a_transform.translate.y, a_transform.translate.z),
			std::max(a_transform.scale, FLT_EPSILON));
	}

	hdt::BoundingSphere ToBoundingSphere(const RE::NiBound& a_bound)
	{
		return hdt::BoundingSphere(
			btVector3(a_bound.center.x, a_bound.center.y, a_bound.center.z),
			std::max(a_bound.fRadius, 0.0F));
	}

	bool MatchesMeshName(RE::BSGeometry* a_geometry, std::span<const std::string> a_meshNames)
	{
		if (a_meshNames.empty()) {
			return true;
		}

		const auto name = a_geometry ? a_geometry->GetName() : "";
		if (name.empty()) {
			return false;
		}

		const std::string_view geometryName(name);
		return Smp::FindMatchingPhysicsName(a_meshNames, geometryName).has_value();
	}

	void DecodeSkinBones(RE::BSSkin::Instance* a_skin, std::vector<Smp::Fo4DecodedSkinBone>& a_bones, Smp::Fo4MeshExtractionStats& a_stats)
	{
		if (!a_skin) {
			return;
		}

		if (a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
			spdlog::warn("skipping suspicious FO4 skin instance with {} bones", a_skin->bones.size());
			return;
		}
		if (!a_skin->worldTransforms.empty() && a_skin->worldTransforms.size() != a_skin->bones.size()) {
			spdlog::warn("FO4 skin instance has {} bones but {} world transforms", a_skin->bones.size(), a_skin->worldTransforms.size());
		}
		const bool useBoneData = a_skin->boneData && a_skin->boneData->transforms.size() <= RE::BSSkin::kMaxExpectedBones;
		if (a_skin->boneData && !useBoneData) {
			spdlog::warn("ignoring suspicious FO4 skin bone data with {} transforms", a_skin->boneData->transforms.size());
		}

		a_bones.reserve(a_skin->bones.size());
		for (std::uint32_t index = 0; index < a_skin->bones.size(); ++index) {
			auto* boneObject = a_skin->bones[index];
			Smp::Fo4DecodedSkinBone decoded;
			if (!boneObject) {
				++a_stats.nullBones;
				a_bones.push_back(decoded);
				continue;
			}

			auto* bone = boneObject->IsNode();
			if (!bone) {
				++a_stats.nonNodeBones;
				a_bones.push_back(decoded);
				continue;
			}

			const auto name = boneObject->GetName();
			decoded.node = bone;
			decoded.name = name.empty() ? std::string{} : std::string(name);

			if (useBoneData && index < a_skin->boneData->transforms.size()) {
				const auto& transform = a_skin->boneData->transforms[index];
				decoded.skinToBone = ToQsTransform(transform.transform);
				decoded.boundingSphere = ToBoundingSphere(transform.bound);
				decoded.hasBoneData = true;
			} else {
				++a_stats.missingBoneData;
			}

			a_bones.push_back(std::move(decoded));
		}
	}

	std::uint32_t CountBadBoneIndex(
		const std::array<std::uint8_t, 4>& a_indices,
		const std::array<float, 4>& a_weights,
		const std::size_t a_boneCount,
		Smp::Fo4MeshExtractionStats& a_stats)
	{
		std::uint32_t count = 0;
		for (std::size_t index = 0; index < a_indices.size(); ++index) {
			if (a_weights[index] > 0.0F && a_indices[index] >= a_boneCount) {
				++a_stats.badBoneIndices;
				++count;
			}
		}
		return count;
	}

	bool DecodeGeometry(RE::BSGeometry* a_geometry, Smp::Fo4MeshExtractionResult& a_result)
	{
		auto* triShape = a_geometry ? a_geometry->IsTriShape() : nullptr;
		auto* skin = a_geometry && a_geometry->skinInstance ? a_geometry->skinInstance.get() : nullptr;
		if (!skin) {
			return false;
		}
		if (!triShape) {
			++a_result.stats.unsupportedGeometryClasses;
			return false;
		}

		auto* renderer = static_cast<RE::BSGraphics::TriShape*>(a_geometry->rendererData);
		if (!renderer) {
			++a_result.stats.missingRendererData;
			return false;
		}

		auto* vertexBuffer = renderer->vertexBuffer;
		if (!vertexBuffer) {
			++a_result.stats.missingVertexBuffer;
			return false;
		}

		if (vertexBuffer->invalidCpuData) {
			++a_result.stats.invalidCpuVertexData;
		}
		if (vertexBuffer->pendingCopy) {
			++a_result.stats.pendingVertexCopies;
		}
		if (!vertexBuffer->data) {
			++a_result.stats.missingCpuVertexData;
		}

		auto vertexDesc = renderer->vertexDesc;
		const auto vertexStride = vertexDesc.GetSize();
		if (vertexStride == 0) {
			++a_result.stats.badVertexStride;
			return false;
		}

		const auto requiredVertexBytes = static_cast<std::uint64_t>(vertexStride) * triShape->numVertices;
		const auto availableVertexBytes = vertexBuffer->dataSize > vertexBuffer->dataOffset ? vertexBuffer->dataSize - vertexBuffer->dataOffset : 0;
		if (availableVertexBytes < requiredVertexBytes) {
			++a_result.stats.undersizedVertexBuffers;
			return false;
		}

		if (!vertexBuffer->data || vertexBuffer->invalidCpuData || vertexBuffer->pendingCopy || triShape->numVertices == 0) {
			return false;
		}

		Smp::Fo4DecodedSkinnedMesh mesh;
		mesh.geometry = a_geometry;
		const auto name = a_geometry->GetName();
		mesh.name = name.empty() ? std::string{} : std::string(name);
		DecodeSkinBones(skin, mesh.bones, a_result.stats);
		mesh.vertices.reserve(triShape->numVertices);

		const auto* vertexData = static_cast<const std::uint8_t*>(vertexBuffer->data) + vertexBuffer->dataOffset;
		for (std::uint16_t vertexIndex = 0; vertexIndex < triShape->numVertices; ++vertexIndex) {
			RE::NiPoint3 position{};
			RE::NiPoint2 texCoord0{};
			RE::NiPoint2 texCoord1{};
			RE::NiPoint3 normal{};
			RE::NiPoint3 binormal{};
			RE::NiPoint3 tangent{};
			RE::NiColorA color{};
			RE::NiColorA skinWeights{};
			std::uint8_t boneIndex0{ 0 };
			std::uint8_t boneIndex1{ 0 };
			std::uint8_t boneIndex2{ 0 };
			std::uint8_t boneIndex3{ 0 };

			RE::BSGraphics::Utility::UnpackVertexData(
				vertexData,
				vertexIndex,
				vertexDesc.desc,
				std::addressof(position),
				std::addressof(texCoord0),
				std::addressof(texCoord1),
				std::addressof(normal),
				std::addressof(binormal),
				std::addressof(tangent),
				std::addressof(color),
				std::addressof(skinWeights),
				std::addressof(boneIndex0),
				std::addressof(boneIndex1),
				std::addressof(boneIndex2),
				std::addressof(boneIndex3));

			hdt::Vertex vertex(position.x, position.y, position.z);
			const std::array weights{ skinWeights.r, skinWeights.g, skinWeights.b, skinWeights.a };
			const std::array indices{ boneIndex0, boneIndex1, boneIndex2, boneIndex3 };
			for (std::size_t index = 0; index < weights.size(); ++index) {
				vertex.weight_[index] = weights[index];
				vertex.boneIdx_[index] = indices[index];
			}
			vertex.sortWeight();
			mesh.badBoneIndices += CountBadBoneIndex(indices, weights, mesh.bones.size(), a_result.stats);
			mesh.vertices.push_back(vertex);
		}

		auto* indexBuffer = a_geometry->GetCustomIndexBuffer();
		if (!indexBuffer) {
			indexBuffer = renderer->indexBuffer;
		}

		if (indexBuffer) {
			if (indexBuffer->invalidCpuData) {
				++a_result.stats.invalidCpuIndexData;
			}
			if (indexBuffer->pendingCopy) {
				++a_result.stats.pendingIndexCopies;
			}
			if (!indexBuffer->data) {
				++a_result.stats.missingCpuIndexData;
			}

			const auto requiredIndexBytes = static_cast<std::uint64_t>(triShape->numTriangles) * 3 * sizeof(std::uint16_t);
			const auto availableIndexBytes = indexBuffer->dataSize > indexBuffer->dataOffset ? indexBuffer->dataSize - indexBuffer->dataOffset : 0;
			if (availableIndexBytes < requiredIndexBytes) {
				++a_result.stats.undersizedIndexBuffers;
			} else if (indexBuffer->data && !indexBuffer->invalidCpuData && !indexBuffer->pendingCopy) {
				const auto* indexData = reinterpret_cast<const std::uint16_t*>(static_cast<const std::uint8_t*>(indexBuffer->data) + indexBuffer->dataOffset);
				const auto indexCount = static_cast<std::uint32_t>(triShape->numTriangles) * 3;
				mesh.indices.reserve(indexCount);
				for (std::uint32_t index = 0; index < indexCount; ++index) {
					mesh.indices.push_back(indexData[index]);
				}
			}
		} else {
			++a_result.stats.missingIndexBuffer;
		}

		++a_result.stats.decodedMeshes;
		a_result.stats.decodedVertices += static_cast<std::uint32_t>(mesh.vertices.size());
		a_result.stats.decodedTriangles += static_cast<std::uint32_t>(mesh.indices.size() / 3);
		a_result.meshes.push_back(std::move(mesh));
		return true;
	}

	void Collect(RE::NiAVObject* a_object, std::span<const std::string> a_meshNames, Smp::Fo4MeshExtractionResult& a_result)
	{
		if (!a_object) {
			return;
		}

		if (auto* geometry = a_object->IsGeometry()) {
			++a_result.stats.geometries;
			if (!geometry->skinInstance) {
				return;
			}

			++a_result.stats.skinnedGeometries;
			if (!MatchesMeshName(geometry, a_meshNames)) {
				return;
			}

			++a_result.stats.matchedGeometries;
			DecodeGeometry(geometry, a_result);
			return;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}

		++a_result.stats.nodes;
		for (auto& child : node->children) {
			Collect(child.get(), a_meshNames, a_result);
		}
	}
}

namespace Smp
{
	Fo4MeshExtractionResult ExtractSkinnedMeshes(RE::NiAVObject* a_root, std::span<const std::string> a_meshNames)
	{
		Fo4MeshExtractionResult result;
		Collect(a_root, a_meshNames, result);
		return result;
	}
}
