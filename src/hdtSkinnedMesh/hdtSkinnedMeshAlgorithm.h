#pragma once

#include "hdtDispatcher.h"
#include "hdtSkinnedMeshShape.h"

namespace hdt
{
	class SkinnedMeshAlgorithm
	{
	public:
		static constexpr int MaxCollisionCount = 512;

		static void processCollision(SkinnedMeshBody* a_body0, SkinnedMeshBody* a_body1, CollisionDispatcher* a_dispatcher);

	private:
		struct CollisionMerge
		{
			btVector3 normal;
			btVector3 pos[2];
			float weight{ 0.0F };

			void reset()
			{
				normal.setZero();
				pos[0].setZero();
				pos[1].setZero();
				weight = 0.0F;
			}
		};

		struct MergeBuffer
		{
			MergeBuffer();
			~MergeBuffer();
			MergeBuffer(const MergeBuffer&) = delete;
			MergeBuffer& operator=(const MergeBuffer&) = delete;
			MergeBuffer(MergeBuffer&&) = delete;
			MergeBuffer& operator=(MergeBuffer&&) = delete;

			void resize(int a_x, int a_y);
			CollisionMerge* getAndTrack(int a_x, int a_y);

			template <class T0, class T1>
			void doMerge(T0* a_shape0, T1* a_shape1, CollisionResult* a_collisions, int a_count);

			void apply(SkinnedMeshBody* a_body0, SkinnedMeshBody* a_body1, CollisionDispatcher* a_dispatcher);

			int mergeStride{ 0 };
			int mergeSize{ 0 };
			std::uint32_t currentGen{ 0 };
			CollisionMerge* buffer{ nullptr };
			std::uint32_t* generations{ nullptr };
			std::vector<int> activeCells;
		};

		template <class T0, class T1>
		static void processCollision(T0* a_shape0, T1* a_shape1, MergeBuffer& a_merge, CollisionResult* a_collision);
	};
}
