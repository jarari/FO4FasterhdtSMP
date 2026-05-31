#pragma once

#include "BSSkin.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"

#include <span>
#include <vector>

namespace Smp
{
	class Fo4SkinnedMeshBone :
		public hdt::SkinnedMeshBone
	{
	public:
		Fo4SkinnedMeshBone(const RE::BSFixedString& a_name, RE::NiNode* a_node, btRigidBody::btRigidBodyConstructionInfo& a_constructionInfo);

		static void ApplyStabilityConfig(bool a_clampRotations, float a_rotationSpeedLimit, bool a_unclampedResets, float a_unclampedResetAngle);

		struct ActiveSkinSlot
		{
			RE::BSSkin::Instance* skin{ nullptr };
			std::uint32_t index{ 0 };
			std::uint64_t buildGroup{ 0 };
		};

		struct SkinSlotRestore
		{
			RE::NiPointer<RE::BSSkin::Instance> skin;
			std::uint32_t index{ 0 };
			std::uint64_t buildGroup{ 0 };
			RE::NiPointer<RE::NiAVObject> reboundBone;
			RE::NiTransform* reboundWorldTransform{ nullptr };
			RE::NiPointer<RE::NiAVObject> originalBone;
			RE::NiTransform* originalWorldTransform{ nullptr };
			RE::NiPointer<RE::NiAVObject> originalRootNode;
		};

		void AddSkinWorldTransform(
			RE::BSSkin::Instance* a_skin,
			std::uint32_t a_index,
			std::uint64_t a_buildGroup,
			RE::NiAVObject* a_originalBone,
			RE::NiTransform* a_originalWorldTransform,
			RE::NiAVObject* a_originalRootNode);
		void CollectSkinWorldTransformSlots(std::vector<ActiveSkinSlot>& a_slots) const;
		void CollectSkinWorldTransformRestoreSlots(std::vector<SkinSlotRestore>& a_slots) const;
		void RemoveSkinWorldTransformsForBuildGroup(std::uint64_t a_buildGroup);
		void RemoveSkinWorldTransformsForBuildGroup(std::uint64_t a_buildGroup, std::span<const ActiveSkinSlot> a_activeSlots);
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
			RE::NiPointer<RE::NiAVObject> originalBone;
			RE::NiTransform* originalWorldTransform{ nullptr };
			RE::NiPointer<RE::NiAVObject> originalRootNode;
			RE::NiTransform* cached{ nullptr };
		};

		RE::NiTransform* ResolveSkinWorldTransform(SkinWorldTransformSlot& a_slot);

		RE::NiPointer<RE::NiNode> node_;
		std::vector<SkinWorldTransformSlot> skinWorldTransforms_;
		int depth_{ 0 };
	};
}
