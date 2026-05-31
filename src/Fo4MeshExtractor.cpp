#include "Fo4MeshExtractor.h"

#include "BSSkin.h"
#include "Fo4TransformConversion.h"
#include "PhysicsName.h"
#include "RE/B/BSGraphics.h"
#include "RE/B/BSTriShape.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <array>
#include <bit>
#include <cstring>
#include <limits>

namespace
{

	hdt::BoundingSphere ToBoundingSphere(const RE::NiBound& a_bound)
	{
		return hdt::BoundingSphere(
			btVector3(a_bound.center.x, a_bound.center.y, a_bound.center.z),
			std::max(a_bound.fRadius, 0.0F));
	}

	bool IsReadableRange(const void* a_data, const std::size_t a_size)
	{
		if (!a_data || a_size == 0) {
			return false;
		}

		const auto* current = static_cast<const std::byte*>(a_data);
		const auto* const end = current + a_size;
		while (current < end) {
			MEMORY_BASIC_INFORMATION info{};
			if (VirtualQuery(current, std::addressof(info), sizeof(info)) == 0) {
				return false;
			}
			if (info.State != MEM_COMMIT || (info.Protect & (PAGE_NOACCESS | PAGE_GUARD)) != 0) {
				return false;
			}

			const auto* regionEnd = static_cast<const std::byte*>(info.BaseAddress) + info.RegionSize;
			if (regionEnd <= current) {
				return false;
			}
			current = regionEnd;
		}
		return true;
	}

	struct ResolvedCpuBuffer
	{
		const std::uint8_t* data{ nullptr };
		std::uint64_t availableBytes{ 0 };
		bool usedRawDataFallback{ false };
	};

	bool ResolveReadableCpuBuffer(
		const std::string& a_meshName,
		const char* a_kind,
		const void* a_data,
		const std::uint64_t a_dataSize,
		const std::uint64_t a_dataOffset,
		const std::uint64_t a_requiredBytes,
		ResolvedCpuBuffer& a_result)
	{
		if (!a_data || a_requiredBytes == 0 || a_requiredBytes > std::numeric_limits<std::size_t>::max()) {
			return false;
		}

		const auto* rawData = static_cast<const std::uint8_t*>(a_data);
		const auto primaryAvailableBytes = a_dataSize > a_dataOffset ? a_dataSize - a_dataOffset : 0;
		const auto requiredSize = static_cast<std::size_t>(a_requiredBytes);
		if (primaryAvailableBytes >= a_requiredBytes) {
			const auto* primaryData = rawData + a_dataOffset;
			if (IsReadableRange(primaryData, requiredSize)) {
				a_result.data = primaryData;
				a_result.availableBytes = primaryAvailableBytes;
				a_result.usedRawDataFallback = false;
				return true;
			}
		}

		if (a_dataOffset != 0 && IsReadableRange(rawData, requiredSize)) {
			spdlog::warn(
				"mesh '{}' using CPU {} data without dataOffset fallback data={} badOffset={} dataSize={} requiredBytes={}",
				a_meshName,
				a_kind,
				a_data,
				a_dataOffset,
				a_dataSize,
				a_requiredBytes);
			a_result.data = rawData;
			a_result.availableBytes = a_dataSize;
			a_result.usedRawDataFallback = true;
			return true;
		}

		return false;
	}

	float HalfToFloat(const std::uint16_t a_value)
	{
		const std::uint32_t sign = static_cast<std::uint32_t>(a_value & 0x8000U) << 16;
		std::uint32_t exponent = (a_value >> 10) & 0x1FU;
		std::uint32_t mantissa = a_value & 0x03FFU;

		std::uint32_t bits = sign;
		if (exponent == 0) {
			if (mantissa != 0) {
				exponent = 127 - 15 + 1;
				while ((mantissa & 0x0400U) == 0) {
					mantissa <<= 1;
					--exponent;
				}
				mantissa &= 0x03FFU;
				bits |= (exponent << 23) | (mantissa << 13);
			}
		} else if (exponent == 0x1FU) {
			bits |= 0x7F800000U | (mantissa << 13);
		} else {
			bits |= ((exponent + (127 - 15)) << 23) | (mantissa << 13);
		}

		return std::bit_cast<float>(bits);
	}

	template <class T>
	T ReadUnaligned(const std::uint8_t* a_data)
	{
		T value{};
		std::memcpy(std::addressof(value), a_data, sizeof(T));
		return value;
	}

	bool DecodePosition(
		const std::uint8_t* a_vertex,
		const std::uint32_t a_vertexStride,
		const RE::BSGraphics::VertexDesc& a_vertexDesc,
		RE::NiPoint3& a_position)
	{
		if (a_vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_FULLPREC)) {
			if (a_vertexStride < sizeof(float) * 3) {
				return false;
			}
			a_position.x = ReadUnaligned<float>(a_vertex);
			a_position.y = ReadUnaligned<float>(a_vertex + sizeof(float));
			a_position.z = ReadUnaligned<float>(a_vertex + sizeof(float) * 2);
			return true;
		}

		if (a_vertexStride < sizeof(std::uint16_t) * 3) {
			return false;
		}
		a_position.x = HalfToFloat(ReadUnaligned<std::uint16_t>(a_vertex));
		a_position.y = HalfToFloat(ReadUnaligned<std::uint16_t>(a_vertex + sizeof(std::uint16_t)));
		a_position.z = HalfToFloat(ReadUnaligned<std::uint16_t>(a_vertex + sizeof(std::uint16_t) * 2));
		return true;
	}

	void DecodeSkinning(
		const std::uint8_t* a_vertex,
		const std::uint32_t a_vertexStride,
		const RE::BSGraphics::VertexDesc& a_vertexDesc,
		hdt::Vertex& a_output)
	{
		const auto skinOffset = a_vertexDesc.GetAttributeOffset(RE::BSGraphics::Vertex::VA_SKINNING);
		if (!a_vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_SKINNED) || skinOffset == 0 || skinOffset + 12 > a_vertexStride) {
			a_output.weight_[0] = 1.0F;
			a_output.boneIdx_[0] = 0;
			for (std::size_t index = 1; index < 4; ++index) {
				a_output.weight_[index] = 0.0F;
				a_output.boneIdx_[index] = 0;
			}
			return;
		}

		const auto* skinData = a_vertex + skinOffset;
		for (std::size_t index = 0; index < 4; ++index) {
			a_output.weight_[index] = HalfToFloat(ReadUnaligned<std::uint16_t>(skinData + index * sizeof(std::uint16_t)));
			a_output.boneIdx_[index] = skinData[8 + index];
		}
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

	bool IsValidNiObjectForIsNode(const RE::NiAVObject* a_object)
	{
		constexpr std::uintptr_t kCanonicalUserSpaceMax = 0x00007FFFFFFFFFFFULL;
		if (!a_object || reinterpret_cast<std::uintptr_t>(a_object) > kCanonicalUserSpaceMax) {
			return false;
		}

		const auto vtable = *reinterpret_cast<void* const* const*>(a_object);
		if (!vtable || reinterpret_cast<std::uintptr_t>(vtable) > kCanonicalUserSpaceMax) {
			return false;
		}

		const auto isNode = vtable[4];
		return isNode && reinterpret_cast<std::uintptr_t>(isNode) <= kCanonicalUserSpaceMax;
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
			if (!IsValidNiObjectForIsNode(boneObject)) {
				++a_stats.nonNodeBones;
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
				decoded.skinToBone = Smp::Fo4Transform::ToBulletQsTransform(transform.transform);
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

	bool SanitizeVertexSkinning(hdt::Vertex& a_vertex, const std::size_t a_boneCount, Smp::Fo4MeshExtractionStats& a_stats)
	{
		constexpr std::size_t kMaxSkinInfluences = 4;
		float totalWeight = 0.0F;
		for (std::size_t index = 0; index < kMaxSkinInfluences; ++index) {
			if (a_vertex.weight_[index] <= FLT_EPSILON) {
				a_vertex.weight_[index] = 0.0F;
				continue;
			}
			if (a_vertex.boneIdx_[index] >= a_boneCount) {
				++a_stats.badBoneIndices;
				a_vertex.weight_[index] = 0.0F;
				a_vertex.boneIdx_[index] = 0;
				continue;
			}
			totalWeight += a_vertex.weight_[index];
		}

		if (totalWeight <= FLT_EPSILON) {
			if (a_boneCount == 0) {
				return false;
			}
			a_vertex.weight_[0] = 1.0F;
			a_vertex.boneIdx_[0] = 0;
			for (std::size_t index = 1; index < kMaxSkinInfluences; ++index) {
				a_vertex.weight_[index] = 0.0F;
				a_vertex.boneIdx_[index] = 0;
			}
			return true;
		}

		const auto invTotalWeight = 1.0F / totalWeight;
		for (auto& weight : a_vertex.weight_) {
			weight *= invTotalWeight;
		}
		return true;
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

		ResolvedCpuBuffer vertexBufferData;
		if (!ResolveReadableCpuBuffer(
				mesh.name,
				"vertex",
				vertexBuffer->data,
				vertexBuffer->dataSize,
				vertexBuffer->dataOffset,
				requiredVertexBytes,
				vertexBufferData)) {
			spdlog::warn(
				"skipping mesh '{}' because CPU vertex range is not readable data={} offset={} stride={} vertices={} availableBytes={} requiredBytes={}",
				mesh.name,
				vertexBuffer->data,
				vertexBuffer->dataOffset,
				vertexStride,
				triShape->numVertices,
				availableVertexBytes,
				requiredVertexBytes);
			return false;
		}
		const auto* vertexData = vertexBufferData.data;

		std::uint32_t skippedUnusableVertices = 0;
		for (std::uint16_t vertexIndex = 0; vertexIndex < triShape->numVertices; ++vertexIndex) {
			RE::NiPoint3 position{};
			const auto* vertexBase = vertexData + static_cast<std::size_t>(vertexIndex) * vertexStride;
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
			if (vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_FULLPREC)) {
				if (vertexStride < sizeof(float) * 3) {
					++skippedUnusableVertices;
					continue;
				}
				position.x = ReadUnaligned<float>(vertexBase);
				position.y = ReadUnaligned<float>(vertexBase + sizeof(float));
				position.z = ReadUnaligned<float>(vertexBase + sizeof(float) * 2);
			}

			hdt::Vertex vertex(position.x, position.y, position.z);
			const std::array weights{ skinWeights.r, skinWeights.g, skinWeights.b, skinWeights.a };
			const std::array indices{ boneIndex0, boneIndex1, boneIndex2, boneIndex3 };
			for (std::size_t index = 0; index < weights.size(); ++index) {
				vertex.weight_[index] = weights[index];
				vertex.boneIdx_[index] = indices[index];
			}
			const auto beforeBadBoneIndices = a_result.stats.badBoneIndices;
			if (!SanitizeVertexSkinning(vertex, mesh.bones.size(), a_result.stats)) {
				++skippedUnusableVertices;
				continue;
			}
			mesh.badBoneIndices += a_result.stats.badBoneIndices - beforeBadBoneIndices;
			vertex.sortWeight();
			mesh.vertices.push_back(vertex);
		}
		if (skippedUnusableVertices > 0) {
			spdlog::debug("mesh '{}' skipped {} vertices with no usable skinning", mesh.name, skippedUnusableVertices);
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

			const auto indexCount = static_cast<std::uint32_t>(triShape->numTriangles) * 3;
			const auto requiredIndexBytes = static_cast<std::uint64_t>(indexCount) * sizeof(std::uint16_t);
			const auto availableIndexBytes = indexBuffer->dataSize > indexBuffer->dataOffset ? indexBuffer->dataSize - indexBuffer->dataOffset : 0;
			if (availableIndexBytes < requiredIndexBytes) {
				++a_result.stats.undersizedIndexBuffers;
			}
			if (indexBuffer->data && !indexBuffer->invalidCpuData && !indexBuffer->pendingCopy && indexCount > 0) {
				ResolvedCpuBuffer indexBufferData;
				if (!ResolveReadableCpuBuffer(
						mesh.name,
						"index",
						indexBuffer->data,
						indexBuffer->dataSize,
						indexBuffer->dataOffset,
						requiredIndexBytes,
						indexBufferData)) {
					spdlog::warn(
						"mesh '{}' has unreadable CPU index range data={} offset={} indices={} availableBytes={} requiredBytes={}",
						mesh.name,
						indexBuffer->data,
						indexBuffer->dataOffset,
						indexCount,
						availableIndexBytes,
						requiredIndexBytes);
				} else {
					const auto* indexData = reinterpret_cast<const std::uint16_t*>(indexBufferData.data);
					mesh.indices.reserve(indexCount);
					for (std::uint32_t index = 0; index < indexCount; ++index) {
						mesh.indices.push_back(indexData[index]);
					}
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
