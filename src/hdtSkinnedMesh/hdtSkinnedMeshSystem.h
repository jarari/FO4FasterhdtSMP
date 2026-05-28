#pragma once

#include "hdtSkinnedMeshBone.h"

namespace hdt
{
	class SkinnedMeshBody;
	class SkinnedMeshShape;

	class SkinnedMeshSystem :
		public RE::BSIntrusiveRefCounted
	{
	public:
		virtual ~SkinnedMeshSystem() = default;

		virtual float prepareForRead(float a_timeStep) { return a_timeStep; }
		virtual void readTransform(float a_timeStep);
		virtual void writeTransform();

		void internalUpdate();
		void gather(std::vector<SkinnedMeshBody*>& a_bodies, std::vector<SkinnedMeshShape*>& a_shapes);
		bool valid() const { return !bones_.empty(); }
		void addBone(RE::BSTSmartPointer<SkinnedMeshBone> a_bone);
		void addMesh(RE::BSTSmartPointer<SkinnedMeshBody> a_mesh);

		std::vector<RE::BSTSmartPointer<SkinnedMeshBone>>& getBones() { return bones_; }
		const std::vector<RE::BSTSmartPointer<SkinnedMeshBone>>& getBones() const { return bones_; }
		std::vector<RE::BSTSmartPointer<SkinnedMeshBody>>& getMeshes() { return meshes_; }
		const std::vector<RE::BSTSmartPointer<SkinnedMeshBody>>& getMeshes() const { return meshes_; }

		bool blockResetting{ false };

	private:
		std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> bones_;
		std::vector<RE::BSTSmartPointer<SkinnedMeshBody>> meshes_;
	};
}
