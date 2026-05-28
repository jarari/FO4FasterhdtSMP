#pragma once

#include "hdtSkinnedMesh/hdtAABB.h"
#include "hdtSkinnedMesh/hdtVertex.h"

#include <span>
#include <string>
#include <vector>

namespace Smp
{
	struct Fo4DecodedSkinBone
	{
		RE::NiNode* node{ nullptr };
		std::string name;
		hdt::btQsTransform skinToBone{ hdt::btQsTransform::getIdentity() };
		hdt::BoundingSphere boundingSphere;
		bool hasBoneData{ false };
	};

	struct Fo4DecodedSkinnedMesh
	{
		RE::BSGeometry* geometry{ nullptr };
		std::string name;
		std::uint32_t badBoneIndices{ 0 };
		std::vector<Fo4DecodedSkinBone> bones;
		std::vector<hdt::Vertex> vertices;
		std::vector<std::uint32_t> indices;
	};

	struct Fo4MeshExtractionStats
	{
		std::uint32_t nodes{ 0 };
		std::uint32_t geometries{ 0 };
		std::uint32_t skinnedGeometries{ 0 };
		std::uint32_t matchedGeometries{ 0 };
		std::uint32_t decodedMeshes{ 0 };
		std::uint32_t decodedVertices{ 0 };
		std::uint32_t decodedTriangles{ 0 };
		std::uint32_t nullBones{ 0 };
		std::uint32_t nonNodeBones{ 0 };
		std::uint32_t missingBoneData{ 0 };
		std::uint32_t unsupportedGeometryClasses{ 0 };
		std::uint32_t missingRendererData{ 0 };
		std::uint32_t missingVertexBuffer{ 0 };
		std::uint32_t missingIndexBuffer{ 0 };
		std::uint32_t invalidCpuVertexData{ 0 };
		std::uint32_t pendingVertexCopies{ 0 };
		std::uint32_t missingCpuVertexData{ 0 };
		std::uint32_t badVertexStride{ 0 };
		std::uint32_t undersizedVertexBuffers{ 0 };
		std::uint32_t invalidCpuIndexData{ 0 };
		std::uint32_t pendingIndexCopies{ 0 };
		std::uint32_t missingCpuIndexData{ 0 };
		std::uint32_t undersizedIndexBuffers{ 0 };
		std::uint32_t badBoneIndices{ 0 };
	};

	struct Fo4MeshExtractionResult
	{
		Fo4MeshExtractionStats stats;
		std::vector<Fo4DecodedSkinnedMesh> meshes;
	};

	Fo4MeshExtractionResult ExtractSkinnedMeshes(RE::NiAVObject* a_root, std::span<const std::string> a_meshNames = {});
}
