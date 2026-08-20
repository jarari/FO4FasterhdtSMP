#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace RE
{
	struct BSFlattenedBoneRef
	{
		NiPointer<NiAVObject> object;
		std::int32_t           index{ -1 };
		std::uint32_t          pad0C{ 0 };
	};
	static_assert(sizeof(BSFlattenedBoneRef) == 0x10);
}

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
		std::array<std::byte, 0x50> pad50;          // 50
		void*                     currentPalette;  // A0
		void*                     previousPalette; // A8
		std::uint32_t             paletteCount;    // B0
		std::uint32_t             paletteFlags;    // B4
		std::uint32_t             paletteByteSize; // B8
		std::uint32_t             paletteStamp;    // BC
	};
	static_assert(sizeof(Instance) == 0xC0);
	static_assert(offsetof(Instance, bones) == 0x10);
	static_assert(offsetof(Instance, worldTransforms) == 0x28);
	static_assert(offsetof(Instance, boneData) == 0x40);
	static_assert(offsetof(Instance, rootNode) == 0x48);
	static_assert(offsetof(Instance, currentPalette) == 0xA0);
	static_assert(offsetof(Instance, previousPalette) == 0xA8);
	static_assert(offsetof(Instance, paletteStamp) == 0xBC);
}
