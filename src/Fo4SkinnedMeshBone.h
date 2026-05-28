#pragma once

#include "BSSkin.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"

#include <vector>

namespace Smp
{
	class Fo4SkinnedMeshBone :
		public hdt::SkinnedMeshBone
	{
	public:
		Fo4SkinnedMeshBone(const RE::BSFixedString& a_name, RE::NiNode* a_node, btRigidBody::btRigidBodyConstructionInfo& a_constructionInfo);

		static void ApplyStabilityConfig(bool a_clampRotations, float a_rotationSpeedLimit, bool a_unclampedResets, float a_unclampedResetAngle);

		void AddSkinWorldTransform(RE::BSSkin::Instance* a_skin, std::uint32_t a_index, std::uint64_t a_buildGroup);
		void RemoveSkinWorldTransformsForBuildGroup(std::uint64_t a_buildGroup);
		void readTransform(float a_timeStep) override;
		void writeTransform() override;

		RE::NiNode* GetNode() const { return node_.get(); }
		int GetDepth() const { return depth_; }

	private:
		struct SkinWorldTransformSlot
		{
			RE::NiPointer<RE::BSSkin::Instance> skin;
			std::uint32_t index{ 0 };
			std::uint64_t buildGroup{ 0 };
			RE::NiTransform* cached{ nullptr };
		};

		RE::NiTransform* ResolveSkinWorldTransform(SkinWorldTransformSlot& a_slot);

		RE::NiPointer<RE::NiNode> node_;
		std::vector<SkinWorldTransformSlot> skinWorldTransforms_;
		int depth_{ 0 };
	};
}
