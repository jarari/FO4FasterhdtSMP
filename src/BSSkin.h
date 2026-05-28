#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace RE::BSSkin
{
	inline constexpr std::uint32_t kMaxExpectedBones = 1024;

	class BoneData :
		public NiObject
	{
	public:
		struct BoneTransforms
		{
			NiBound     bound;      // 00
			NiTransform transform;  // 10
		};
		static_assert(sizeof(BoneTransforms) == 0x50);

		BSTAlignedArray<BoneTransforms, 0x10> transforms;  // 10
	};
	static_assert(sizeof(BoneData) == 0x28);
	static_assert(offsetof(BoneData, transforms) == 0x10);

	class Instance :
		public NiObject
	{
	public:
		// Verified against FO4 1.10.163 BSSkin::Instance::SetNumBones,
		// GetWorldToSkinTransform, AllocBoneData, and MakeBonesReal in Ghidra.
		BSTArray<NiAVObject*>    bones;            // 10
		BSTArray<NiTransform*>   worldTransforms;  // 28
		NiPointer<BoneData>      boneData;         // 40
		NiAVObject*              rootNode;         // 48
		std::array<std::byte, 0x70> pad50;          // 50
	};
	static_assert(sizeof(Instance) == 0xC0);
	static_assert(offsetof(Instance, bones) == 0x10);
	static_assert(offsetof(Instance, worldTransforms) == 0x28);
	static_assert(offsetof(Instance, boneData) == 0x40);
	static_assert(offsetof(Instance, rootNode) == 0x48);
}
