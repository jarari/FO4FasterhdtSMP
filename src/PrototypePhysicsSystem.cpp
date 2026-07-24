#include "PrototypePhysicsSystem.h"

#include "BSSkin.h"
#include "Fo4CpuBuffer.h"
#include "Fo4MeshExtractor.h"
#include "RE/B/BSGraphics.h"
#include "RE/B/BSTriShape.h"

namespace
{
	struct GeometryStats
	{
		std::uint32_t nodes{ 0 };
		std::uint32_t geometries{ 0 };
		std::uint32_t skinnedGeometries{ 0 };
		std::uint32_t bones{ 0 };
		std::uint32_t nullBones{ 0 };
		std::uint32_t missingBoneData{ 0 };
		std::uint32_t worldTransformMismatches{ 0 };
		std::uint32_t triShapes{ 0 };
		std::uint32_t unsupportedGeometryClasses{ 0 };
		std::uint32_t missingRendererData{ 0 };
		std::uint32_t missingVertexBuffer{ 0 };
		std::uint32_t missingIndexBuffer{ 0 };
		std::uint32_t invalidCpuVertexData{ 0 };
		std::uint32_t pendingVertexCopies{ 0 };
		std::uint32_t missingCpuVertexData{ 0 };
		std::uint32_t badVertexStride{ 0 };
		std::uint32_t undersizedVertexBuffers{ 0 };
		std::uint32_t undersizedIndexBuffers{ 0 };
		std::uint32_t decodedVertexProbes{ 0 };
	};

	void InspectRendererData(RE::BSGeometry* a_geometry, GeometryStats& a_stats)
	{
		auto* triShape = a_geometry ? a_geometry->IsTriShape() : nullptr;
		if (!triShape) {
			return;
		}

		++a_stats.triShapes;
		auto* renderer = static_cast<RE::BSGraphics::TriShape*>(a_geometry->rendererData);
		if (!renderer) {
			++a_stats.missingRendererData;
			return;
		}

		auto* vertexBuffer = renderer->vertexBuffer;
		if (!vertexBuffer) {
			++a_stats.missingVertexBuffer;
			return;
		}

		if (!renderer->indexBuffer) {
			++a_stats.missingIndexBuffer;
		}

		if (vertexBuffer->invalidCpuData) {
			++a_stats.invalidCpuVertexData;
		}
		if (vertexBuffer->pendingCopy) {
			++a_stats.pendingVertexCopies;
		}
		if (!vertexBuffer->data) {
			++a_stats.missingCpuVertexData;
		}

		auto vertexDesc = renderer->vertexDesc;
		const auto vertexStride = vertexDesc.GetSize();
		if (vertexStride == 0) {
			++a_stats.badVertexStride;
			return;
		}

		const auto vertexBytes = static_cast<std::uint64_t>(vertexStride) * triShape->numVertices;
		const auto availableVertexBytes = Smp::Fo4CpuBuffer::GetAvailableBytes(vertexBuffer);
		if (availableVertexBytes < vertexBytes) {
			++a_stats.undersizedVertexBuffers;
		}

		if (auto* indexBuffer = renderer->indexBuffer) {
			const auto indexBytes = static_cast<std::uint64_t>(triShape->numTriangles) * 3 * sizeof(std::uint16_t);
			const auto availableIndexBytes = Smp::Fo4CpuBuffer::GetAvailableBytes(indexBuffer);
			if (availableIndexBytes < indexBytes) {
				++a_stats.undersizedIndexBuffers;
			}
		}

		if (!a_geometry->skinInstance || triShape->numVertices == 0 || !vertexBuffer->data || vertexBuffer->invalidCpuData || vertexBuffer->pendingCopy || availableVertexBytes < vertexStride) {
			return;
		}

		const auto* vertexData = Smp::Fo4CpuBuffer::GetDataStart(vertexBuffer);
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
			0,
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

		++a_stats.decodedVertexProbes;
		spdlog::debug(
			"decoded prototype vertex geometry={} vertexCount={} stride={} pos=({:.3f},{:.3f},{:.3f}) weights=({:.3f},{:.3f},{:.3f},{:.3f}) bones=({},{},{},{})",
			static_cast<void*>(a_geometry),
			triShape->numVertices,
			vertexStride,
			position.x,
			position.y,
			position.z,
			skinWeights.r,
			skinWeights.g,
			skinWeights.b,
			skinWeights.a,
			boneIndex0,
			boneIndex1,
			boneIndex2,
			boneIndex3);
	}

	void CollectGeometryStats(RE::NiAVObject* a_object, GeometryStats& a_stats)
	{
		if (!a_object) {
			return;
		}

		if (auto* geometry = a_object->IsGeometry()) {
			++a_stats.geometries;
			if (geometry->skinInstance) {
				++a_stats.skinnedGeometries;
				auto* skin = geometry->skinInstance.get();
				a_stats.bones += skin->bones.size();
				if (!skin->boneData) {
					++a_stats.missingBoneData;
				}
				if (!skin->worldTransforms.empty() && skin->worldTransforms.size() != skin->bones.size()) {
					++a_stats.worldTransformMismatches;
				}
				for (auto* bone : skin->bones) {
					if (!bone) {
						++a_stats.nullBones;
					}
				}
				if (!geometry->IsTriShape()) {
					++a_stats.unsupportedGeometryClasses;
				}
			}
			InspectRendererData(geometry, a_stats);
			return;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}

		++a_stats.nodes;
		for (auto& child : node->children) {
			CollectGeometryStats(child.get(), a_stats);
		}
	}
}

namespace Smp
{
	PrototypePhysicsSystem* PrototypePhysicsSystem::GetSingleton()
	{
		static PrototypePhysicsSystem singleton;
		return std::addressof(singleton);
	}

	void PrototypePhysicsSystem::Register()
	{
		if (registered_) {
			return;
		}

		GetLifecycleEventSource().RegisterSink(this);
		registered_ = true;
		spdlog::info("enabled FO4 Faster HDT-SMP prototype geometry diagnostics");
	}

	RE::BSEventNotifyControl PrototypePhysicsSystem::ProcessEvent(const LifecycleEvent& a_event, RE::BSTEventSource<LifecycleEvent>*)
	{
		switch (a_event.type) {
		case LifecycleEventType::kArmorAttachSkinnedObject:
		case LifecycleEventType::kArmorAttachToParent:
		case LifecycleEventType::kActorSet3D:
		case LifecycleEventType::kActorLoad3D:
		case LifecycleEventType::kActorHeadInitialized:
			InspectAttachedObject(a_event);
			break;
		case LifecycleEventType::kArmorDetachBegin:
			spdlog::debug("prototype physics detach begin actor={} object={}", static_cast<void*>(a_event.actor), static_cast<void*>(a_event.object));
			break;
		case LifecycleEventType::kActorReset3D:
			spdlog::debug("prototype physics actor rebuild/reset actor={} object={}", static_cast<void*>(a_event.actor), static_cast<void*>(a_event.object));
			break;
		default:
			break;
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	void PrototypePhysicsSystem::InspectAttachedObject(const LifecycleEvent& a_event)
	{
		GeometryStats stats;
		CollectGeometryStats(a_event.object, stats);

		if (stats.geometries == 0) {
			spdlog::debug("{} has no geometry actor={} object={}", ToString(a_event.type), static_cast<void*>(a_event.actor), static_cast<void*>(a_event.object));
			return;
		}

		spdlog::info(
			"{} geometry inventory actor={} object={} nodes={} geometries={} skinned={} bones={} nullBones={} missingBoneData={} worldXformMismatches={} triShapes={} unsupportedGeometryClasses={} missingRendererData={} decodedVertexProbes={}",
			ToString(a_event.type),
			static_cast<void*>(a_event.actor),
			static_cast<void*>(a_event.object),
			stats.nodes,
			stats.geometries,
			stats.skinnedGeometries,
			stats.bones,
			stats.nullBones,
			stats.missingBoneData,
			stats.worldTransformMismatches,
			stats.triShapes,
			stats.unsupportedGeometryClasses,
			stats.missingRendererData,
			stats.decodedVertexProbes);

		if (stats.skinnedGeometries == 0) {
			spdlog::warn("{} found geometry without public skin instances; real SMP creation will be skipped for this object", ToString(a_event.type));
		}
		if (stats.nullBones > 0 || stats.missingBoneData > 0 || stats.worldTransformMismatches > 0 || stats.unsupportedGeometryClasses > 0) {
			spdlog::warn("{} found suspicious skin layout: nullBones={} missingBoneData={} worldXformMismatches={} unsupportedGeometryClasses={}", ToString(a_event.type), stats.nullBones, stats.missingBoneData, stats.worldTransformMismatches, stats.unsupportedGeometryClasses);
		}
		if (stats.missingRendererData > 0 || stats.missingVertexBuffer > 0 || stats.invalidCpuVertexData > 0 || stats.pendingVertexCopies > 0 || stats.missingCpuVertexData > 0 || stats.badVertexStride > 0 || stats.undersizedVertexBuffers > 0 || stats.undersizedIndexBuffers > 0) {
			spdlog::warn(
				"{} found renderer buffer issues: missingRendererData={} missingVB={} missingIB={} invalidCpuVB={} pendingVBCopy={} missingCpuVBData={} badStride={} undersizedVB={} undersizedIB={}",
				ToString(a_event.type),
				stats.missingRendererData,
				stats.missingVertexBuffer,
				stats.missingIndexBuffer,
				stats.invalidCpuVertexData,
				stats.pendingVertexCopies,
				stats.missingCpuVertexData,
				stats.badVertexStride,
				stats.undersizedVertexBuffers,
				stats.undersizedIndexBuffers);
		}

		const auto extraction = ExtractSkinnedMeshes(a_event.object);
		if (extraction.stats.skinnedGeometries > 0) {
			spdlog::info(
				"{} mesh decode actor={} object={} matched={} decodedMeshes={} vertices={} triangles={} splitPositionData={} faceGenPositionData={} missingPositionData={} unsupportedGeometryClasses={} badBoneIndices={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				extraction.stats.matchedGeometries,
				extraction.stats.decodedMeshes,
				extraction.stats.decodedVertices,
				extraction.stats.decodedTriangles,
				extraction.stats.splitPositionData,
				extraction.stats.faceGenPositionData,
				extraction.stats.missingPositionData,
				extraction.stats.unsupportedGeometryClasses,
				extraction.stats.badBoneIndices);
		}
	}
}
