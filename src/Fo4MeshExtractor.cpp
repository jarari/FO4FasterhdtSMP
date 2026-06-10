#include "Fo4MeshExtractor.h"

#include "BSSkin.h"
#include "Fo4CpuBuffer.h"
#include "Fo4TransformConversion.h"
#include "PhysicsName.h"
#include "RE/B/BSGraphics.h"
#include "RE/B/BSTriShape.h"


#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>

#if defined(__AVX2__) || defined(__AVX512F__) || defined(FO4_FASTER_HDTSMP_AVX2) || defined(FO4_FASTER_HDTSMP_AVX512)
#	include <immintrin.h>
#	define FO4SMP_USE_F16C_SKINNING 1
#endif

namespace
{
	struct Fo4BSFaceGenObjectData :
		public RE::NiExtraData
	{
		RE::NiPoint3* positions{ nullptr };
		std::uint32_t positionCount{ 0 };
		std::uint32_t vertexCount{ 0 };
	};
	static_assert(offsetof(Fo4BSFaceGenObjectData, positions) == 0x18);
	static_assert(offsetof(Fo4BSFaceGenObjectData, positionCount) == 0x20);
	static_assert(offsetof(Fo4BSFaceGenObjectData, vertexCount) == 0x24);

	struct Fo4BSFaceGenModelMeshData
	{
		std::byte pad00[0x08]{};
		RE::NiPointer<RE::NiAVObject> faceNode;
		RE::NiPointer<RE::NiAVObject> geometry;
		std::byte pad18[0x10]{};
	};
	static_assert(offsetof(Fo4BSFaceGenModelMeshData, faceNode) == 0x08);
	static_assert(offsetof(Fo4BSFaceGenModelMeshData, geometry) == 0x10);
	static_assert(sizeof(Fo4BSFaceGenModelMeshData) == 0x28);

	struct Fo4BSFaceGenModel
	{
		std::byte pad00[0x10]{};
		Fo4BSFaceGenModelMeshData* modelMeshData{ nullptr };
		std::byte pad18[0x08]{};
	};
	static_assert(offsetof(Fo4BSFaceGenModel, modelMeshData) == 0x10);
	static_assert(sizeof(Fo4BSFaceGenModel) == 0x20);

	struct Fo4BSFaceGenModelExtraData :
		public RE::NiExtraData
	{
		Fo4BSFaceGenModel* model{ nullptr };
	};
	static_assert(offsetof(Fo4BSFaceGenModelExtraData, model) == 0x18);

	hdt::BoundingSphere ToBoundingSphere(const RE::NiBound& a_bound)
	{
		return hdt::BoundingSphere(
			btVector3(a_bound.center.x, a_bound.center.y, a_bound.center.z),
			std::max(a_bound.fRadius, 0.0F));
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
		if (!a_vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_VERTEX)) {
			return false;
		}

		if (a_vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_FULLPREC)) {
			if (a_vertexStride < sizeof(float) * 3) {
				return false;
			}
			a_position.x = ReadUnaligned<float>(a_vertex);
			a_position.y = ReadUnaligned<float>(a_vertex + sizeof(float));
			a_position.z = ReadUnaligned<float>(a_vertex + sizeof(float) * 2);
			return true;
		}

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
			a_vertex,
			0,
			a_vertexDesc.desc,
			std::addressof(a_position),
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
		return true;
	}

	struct SplitPositionStream
	{
		const std::uint8_t* data{ nullptr };
		std::uint32_t stride{ 0 };
		std::uint32_t dataSize{ 0 };
		bool fullPrecision{ false };
	};

	std::uint32_t GetSplitPositionStride(const RE::BSGraphics::VertexDesc& a_vertexDesc)
	{
		// FO4 reuses the descriptor's VA_POSITION offset field as the split position-stream stride.
		return a_vertexDesc.GetAttributeOffset(RE::BSGraphics::Vertex::VA_POSITION);
	}

	std::optional<SplitPositionStream> ResolveSplitPositionStream(
		RE::BSGeometry* a_geometry,
		const RE::BSGraphics::VertexDesc& a_rendererVertexDesc,
		const RE::BSGraphics::VertexDesc& a_geometryVertexDesc,
		const std::uint32_t a_vertexCount)
	{
		if (!a_geometry || a_vertexCount == 0 || a_rendererVertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_VERTEX)) {
			return std::nullopt;
		}

		const auto splitStride = GetSplitPositionStride(a_geometryVertexDesc);
		if (splitStride < sizeof(std::uint16_t) * 3 || splitStride > 64) {
			return std::nullopt;
		}

		const auto* geometryBytes = reinterpret_cast<const std::uint8_t*>(a_geometry);
		constexpr std::size_t kSplitPositionDataSizeOffset = 0x170;
		constexpr std::size_t kSplitPositionDataPointerOffset = 0x180;

		const auto dataSize = ReadUnaligned<std::uint32_t>(geometryBytes + kSplitPositionDataSizeOffset);
		const auto data = ReadUnaligned<const std::uint8_t*>(geometryBytes + kSplitPositionDataPointerOffset);
		const auto requiredBytes = static_cast<std::uint64_t>(splitStride) * a_vertexCount;
		if (requiredBytes > std::numeric_limits<std::uint32_t>::max() || dataSize != requiredBytes || !data) {
			return std::nullopt;
		}

		SplitPositionStream result;
		result.data = data;
		result.stride = splitStride;
		result.dataSize = dataSize;
		result.fullPrecision = splitStride >= sizeof(float) * 4 && a_geometryVertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_FULLPREC);
		return result;
	}

	bool DecodeSplitPosition(
		const SplitPositionStream& a_stream,
		const std::uint32_t a_vertexIndex,
		RE::NiPoint3& a_position)
	{
		if (!a_stream.data || a_stream.stride < sizeof(std::uint16_t) * 3) {
			return false;
		}

		const auto* vertex = a_stream.data + static_cast<std::size_t>(a_vertexIndex) * a_stream.stride;
		if (a_stream.fullPrecision) {
			if (a_stream.stride < sizeof(float) * 3) {
				return false;
			}
			a_position.x = ReadUnaligned<float>(vertex);
			a_position.y = ReadUnaligned<float>(vertex + sizeof(float));
			a_position.z = ReadUnaligned<float>(vertex + sizeof(float) * 2);
			return true;
		}

		a_position.x = HalfToFloat(ReadUnaligned<std::uint16_t>(vertex));
		a_position.y = HalfToFloat(ReadUnaligned<std::uint16_t>(vertex + sizeof(std::uint16_t)));
		a_position.z = HalfToFloat(ReadUnaligned<std::uint16_t>(vertex + sizeof(std::uint16_t) * 2));
		return true;
	}

	bool IsFinitePosition(const RE::NiPoint3& a_position)
	{
		return std::isfinite(a_position.x) && std::isfinite(a_position.y) && std::isfinite(a_position.z);
	}

	const RE::NiPoint3* ResolveFaceGenObjectPositions(
		RE::BSGeometry* a_geometry,
		const std::uint32_t a_vertexCount,
		const std::string& a_meshName)
	{
		if (!a_geometry || !a_geometry->extra || a_vertexCount == 0) {
			return nullptr;
		}

		for (auto* extra : *a_geometry->extra) {
			if (!extra || !Smp::PhysicsNamesEqual(std::string_view(extra->name), "FOD")) {
				continue;
			}

			const auto* fod = static_cast<const Fo4BSFaceGenObjectData*>(extra);
			if (!fod->positions || fod->positionCount != a_vertexCount || fod->vertexCount != a_vertexCount) {
				spdlog::debug(
					"mesh '{}' ignored FaceGen object data positions={} positionCount={} vertexCount={} expectedVertices={}",
					a_meshName,
					static_cast<const void*>(fod->positions),
					fod->positionCount,
					fod->vertexCount,
					a_vertexCount);
				continue;
			}

			return fod->positions;
		}

		return nullptr;
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
#ifdef FO4SMP_USE_F16C_SKINNING
		const auto halfWeights = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(skinData));
		_mm_storeu_ps(a_output.weight_, _mm_cvtph_ps(halfWeights));
		for (std::size_t index = 0; index < 4; ++index) {
			a_output.boneIdx_[index] = skinData[8 + index];
		}
#else
		for (std::size_t index = 0; index < 4; ++index) {
			a_output.weight_[index] = HalfToFloat(ReadUnaligned<std::uint16_t>(skinData + index * sizeof(std::uint16_t)));
			a_output.boneIdx_[index] = skinData[8 + index];
		}
#endif
	}

	RE::BSGeometry* ResolveFaceGenOriginalGeometry(RE::BSGeometry* a_geometry)
	{
		if (!a_geometry || !a_geometry->extra) {
			return nullptr;
		}

		for (auto* extra : *a_geometry->extra) {
			if (!extra || !Smp::PhysicsNamesEqual(std::string_view(extra->name), "FMD")) {
				continue;
			}

			const auto* fmd = static_cast<const Fo4BSFaceGenModelExtraData*>(extra);
			const auto* meshData = fmd->model ? fmd->model->modelMeshData : nullptr;
			auto* originalObject = meshData ? meshData->geometry.get() : nullptr;
			return originalObject ? originalObject->IsGeometry() : nullptr;
		}

		return nullptr;
	}

	std::string ResolveGeometryName(RE::BSGeometry* a_geometry)
	{
		if (!a_geometry) {
			return {};
		}

		if (auto* originalGeometry = ResolveFaceGenOriginalGeometry(a_geometry)) {
			const auto originalName = originalGeometry->GetName();
			if (!originalName.empty()) {
				return std::string(originalName);
			}
		}

		const auto name = a_geometry->GetName();
		return name.empty() ? std::string{} : std::string(name);
	}

	bool MatchesMeshName(RE::BSGeometry* a_geometry, std::span<const std::string> a_meshNames)
	{
		if (a_meshNames.empty()) {
			return true;
		}

		const auto name = ResolveGeometryName(a_geometry);
		if (name.empty()) {
			return false;
		}

		return Smp::FindMatchingPhysicsName(a_meshNames, name).has_value();
	}

	std::uint32_t CountNullSkinBones(RE::BSSkin::Instance* a_skin)
	{
		if (!a_skin) {
			return 0;
		}

		std::uint32_t count = 0;
		for (auto* bone : a_skin->bones) {
			if (!bone) {
				++count;
			}
		}
		return count;
	}

	void MakeSkinBonesReal(RE::BSSkin::Instance* a_skin, const std::string& a_meshName)
	{
		if (!a_skin || a_skin->bones.empty()) {
			return;
		}

		const auto beforeNullBones = CountNullSkinBones(a_skin);
		if (beforeNullBones == 0) {
			return;
		}

		using func_t = void (*)(RE::BSSkin::Instance*);
		static REL::Relocation<func_t> makeBonesReal{ REL::ID{ 497936, 2270470 } };
		makeBonesReal(a_skin);

		const auto afterNullBones = CountNullSkinBones(a_skin);
		if (afterNullBones != beforeNullBones) {
			spdlog::debug(
				"mesh '{}' realized BSSkin bones nullBefore={} nullAfter={}",
				a_meshName,
				beforeNullBones,
				afterNullBones);
		}
	}

	bool DecodeSkinBones(RE::BSSkin::Instance* a_skin, std::vector<Smp::Fo4DecodedSkinBone>& a_bones, Smp::Fo4MeshExtractionStats& a_stats)
	{
		if (!a_skin) {
			return false;
		}

		if (a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
			spdlog::warn("skipping suspicious FO4 skin instance with {} bones", a_skin->bones.size());
			return false;
		}
		if (!a_skin->worldTransforms.empty() && a_skin->worldTransforms.size() != a_skin->bones.size()) {
			spdlog::warn("FO4 skin instance has {} bones but {} world transforms", a_skin->bones.size(), a_skin->worldTransforms.size());
			return false;
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
				return false;
			}

			auto* bone = boneObject->IsNode();
			if (!bone) {
				++a_stats.nonNodeBones;
				return false;
			}

			const auto name = boneObject->GetName();
			decoded.node = bone;
			decoded.name = name.empty() ? std::string{} : std::string(name);

			if (useBoneData && index < a_skin->boneData->transforms.size()) {
				const auto& transform = a_skin->boneData->transforms[index];
				auto skinToBone = transform.transform;
				skinToBone.scale = 1.0F;
				decoded.skinToBone = Smp::Fo4Transform::ToBulletQsTransform(skinToBone);
				decoded.boundingSphere = ToBoundingSphere(transform.bound);
				decoded.hasSkinToBone = true;
				decoded.hasBoneData = true;
			} else {
				++a_stats.missingBoneData;
			}

			a_bones.push_back(std::move(decoded));
		}
		return true;
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

	bool HasUsableSkinBone(const std::vector<Smp::Fo4DecodedSkinBone>& a_bones, const std::size_t a_boneIndex)
	{
		return a_boneIndex < a_bones.size() &&
			a_bones[a_boneIndex].node &&
			a_bones[a_boneIndex].hasSkinToBone;
	}

	bool SanitizeVertexSkinning(hdt::Vertex& a_vertex, const std::vector<Smp::Fo4DecodedSkinBone>& a_bones, Smp::Fo4MeshExtractionStats& a_stats)
	{
		constexpr std::size_t kMaxSkinInfluences = 4;
		float totalWeight = 0.0F;
		for (std::size_t index = 0; index < kMaxSkinInfluences; ++index) {
			if (a_vertex.weight_[index] <= FLT_EPSILON) {
				a_vertex.weight_[index] = 0.0F;
				continue;
			}
			if (!HasUsableSkinBone(a_bones, a_vertex.boneIdx_[index])) {
				++a_stats.badBoneIndices;
				a_vertex.weight_[index] = 0.0F;
				a_vertex.boneIdx_[index] = 0;
				continue;
			}
			totalWeight += a_vertex.weight_[index];
		}

		if (totalWeight <= FLT_EPSILON) {
			return false;
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

		auto rendererVertexDesc = renderer->vertexDesc;
		auto geometryVertexDesc = a_geometry->vertexDesc;
		auto vertexDesc = rendererVertexDesc;
		const auto rendererVertexStride = rendererVertexDesc.GetSize();
		const auto geometryVertexStride = geometryVertexDesc.GetSize();
		const auto usingGeometryVertexDesc =
			!rendererVertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_VERTEX) &&
			geometryVertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_VERTEX) &&
			rendererVertexStride == geometryVertexStride;
		if (usingGeometryVertexDesc) {
			vertexDesc = geometryVertexDesc;
			spdlog::debug(
				"mesh '{}' using geometry vertex descriptor for CPU decode because renderer descriptor lacks VF_VERTEX rendererDesc={:#x} rendererFlags={:#x} geometryDesc={:#x} geometryFlags={:#x} stride={} vertices={}",
				ResolveGeometryName(a_geometry),
				rendererVertexDesc.desc,
				std::to_underlying(rendererVertexDesc.GetFlags()),
				geometryVertexDesc.desc,
				std::to_underlying(geometryVertexDesc.GetFlags()),
				rendererVertexStride,
				triShape->numVertices);
		}
		const auto vertexStride = vertexDesc.GetSize();
		if (vertexStride == 0) {
			++a_result.stats.badVertexStride;
			return false;
		}

		const auto requiredVertexBytes = static_cast<std::uint64_t>(vertexStride) * triShape->numVertices;
		const auto availableVertexBytes = Smp::Fo4CpuBuffer::GetAvailableBytes(vertexBuffer);
		if (availableVertexBytes < requiredVertexBytes) {
			++a_result.stats.undersizedVertexBuffers;
		}

		if (!vertexBuffer->data || vertexBuffer->invalidCpuData || vertexBuffer->pendingCopy || triShape->numVertices == 0) {
			return false;
		}

		Smp::Fo4DecodedSkinnedMesh mesh;
		mesh.geometry = a_geometry;
		mesh.skinRootNode = skin->rootNode;
		mesh.name = ResolveGeometryName(a_geometry);
		MakeSkinBonesReal(skin, mesh.name);
		if (!DecodeSkinBones(skin, mesh.bones, a_result.stats)) {
			spdlog::warn("skipping mesh '{}' because its FO4 skin bone array is not coherent", mesh.name);
			return false;
		}
		mesh.vertices.reserve(triShape->numVertices);
		std::vector<std::uint32_t> vertexRemap(triShape->numVertices, std::numeric_limits<std::uint32_t>::max());

		Smp::Fo4CpuBuffer::ResolvedBuffer vertexBufferData;
		if (!Smp::Fo4CpuBuffer::ResolveReadable(mesh.name, "vertex", vertexBuffer, requiredVertexBytes, vertexBufferData)) {
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
		const auto hasDecodablePositionData = vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_VERTEX);
		const auto splitPositions = hasDecodablePositionData ?
			std::optional<SplitPositionStream>{} :
			ResolveSplitPositionStream(a_geometry, rendererVertexDesc, geometryVertexDesc, triShape->numVertices);
		const auto* faceGenPositions = hasDecodablePositionData || splitPositions.has_value() ?
			nullptr :
			ResolveFaceGenObjectPositions(a_geometry, triShape->numVertices, mesh.name);
		if (!hasDecodablePositionData && !splitPositions && !faceGenPositions) {
			++a_result.stats.missingPositionData;
			spdlog::debug(
				"skipping mesh '{}' because vertex descriptor has no position stream rendererDesc={:#x} rendererFlags={:#x} geometryDesc={:#x} geometryFlags={:#x} vertexStride={} splitStride={} vertices={} rendererFullPrecision={} geometryFullPrecision={}",
				a_geometry->GetName(),
				rendererVertexDesc.desc,
				std::to_underlying(rendererVertexDesc.GetFlags()),
				geometryVertexDesc.desc,
				std::to_underlying(geometryVertexDesc.GetFlags()),
				vertexStride,
				GetSplitPositionStride(geometryVertexDesc),
				triShape->numVertices,
				rendererVertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_FULLPREC),
				geometryVertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_FULLPREC));
			return false;
		}
		if (splitPositions) {
			++a_result.stats.splitPositionData;
			spdlog::debug(
				"mesh '{}' using split position stream positions={} stride={} bytes={} vertices={} decode={} rendererDesc={:#x} rendererFlags={:#x} geometryDesc={:#x} geometryFlags={:#x} vertexStride={}",
				mesh.name,
				static_cast<const void*>(splitPositions->data),
				splitPositions->stride,
				splitPositions->dataSize,
				triShape->numVertices,
				splitPositions->fullPrecision ? "float3" : "half3",
				rendererVertexDesc.desc,
				std::to_underlying(rendererVertexDesc.GetFlags()),
				geometryVertexDesc.desc,
				std::to_underlying(geometryVertexDesc.GetFlags()),
				vertexStride);
		}
		if (faceGenPositions) {
			++a_result.stats.faceGenPositionData;
			spdlog::debug(
				"mesh '{}' using FaceGen object data positions={} vertices={} because renderer descriptor has no position stream desc={:#x} flags={:#x} stride={} fullPrecision={}",
				mesh.name,
				static_cast<const void*>(faceGenPositions),
				triShape->numVertices,
				vertexDesc.desc,
				std::to_underlying(vertexDesc.GetFlags()),
				vertexStride,
				vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_FULLPREC));
		}

		std::uint32_t skippedUnusableVertices = 0;
		for (std::uint16_t vertexIndex = 0; vertexIndex < triShape->numVertices; ++vertexIndex) {
			RE::NiPoint3 position{};
			const auto* vertexBase = vertexData + static_cast<std::size_t>(vertexIndex) * vertexStride;
			if (faceGenPositions) {
				position = faceGenPositions[vertexIndex];
			} else if (splitPositions) {
				if (!DecodeSplitPosition(*splitPositions, vertexIndex, position)) {
					++a_result.stats.missingPositionData;
					++skippedUnusableVertices;
					continue;
				}
			} else if (!DecodePosition(vertexBase, vertexStride, vertexDesc, position)) {
				++a_result.stats.missingPositionData;
				++skippedUnusableVertices;
				continue;
			}
			if (!IsFinitePosition(position)) {
				++a_result.stats.nonFinitePositions;
				++skippedUnusableVertices;
				continue;
			}

			hdt::Vertex vertex(position.x, position.y, position.z);
			DecodeSkinning(vertexBase, vertexStride, vertexDesc, vertex);
			const auto beforeBadBoneIndices = a_result.stats.badBoneIndices;
			if (!SanitizeVertexSkinning(vertex, mesh.bones, a_result.stats)) {
				++skippedUnusableVertices;
				continue;
			}
			mesh.badBoneIndices += a_result.stats.badBoneIndices - beforeBadBoneIndices;
			vertex.sortWeight();
			vertexRemap[vertexIndex] = static_cast<std::uint32_t>(mesh.vertices.size());
			mesh.vertices.push_back(vertex);
		}
		if (skippedUnusableVertices > 0) {
			spdlog::debug("mesh '{}' skipped {} vertices with no usable position or skinning", mesh.name, skippedUnusableVertices);
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
			const auto availableIndexBytes = Smp::Fo4CpuBuffer::GetAvailableBytes(indexBuffer);
			if (availableIndexBytes < requiredIndexBytes) {
				++a_result.stats.undersizedIndexBuffers;
			}
			if (indexBuffer->data && !indexBuffer->invalidCpuData && !indexBuffer->pendingCopy && indexCount > 0) {
				Smp::Fo4CpuBuffer::ResolvedBuffer indexBufferData;
				if (!Smp::Fo4CpuBuffer::ResolveReadable(mesh.name, "index", indexBuffer, requiredIndexBytes, indexBufferData)) {
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
					for (std::uint32_t index = 0; index + 2 < indexCount; index += 3) {
						const auto source0 = indexData[index];
						const auto source1 = indexData[index + 1];
						const auto source2 = indexData[index + 2];
						if (source0 >= vertexRemap.size() || source1 >= vertexRemap.size() || source2 >= vertexRemap.size()) {
							continue;
						}

						const auto remapped0 = vertexRemap[source0];
						const auto remapped1 = vertexRemap[source1];
						const auto remapped2 = vertexRemap[source2];
						if (remapped0 != std::numeric_limits<std::uint32_t>::max() &&
							remapped1 != std::numeric_limits<std::uint32_t>::max() &&
							remapped2 != std::numeric_limits<std::uint32_t>::max()) {
							mesh.indices.push_back(remapped0);
							mesh.indices.push_back(remapped1);
							mesh.indices.push_back(remapped2);
						}
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
