#pragma once

#include "hdtAABB.h"
#include "hdtBulletHelper.h"

namespace hdt
{
	class SkinnedMeshBody;

	struct alignas(16) SkinnedMeshBone :
		public RE::BSIntrusiveRefCounted
	{
		BT_DECLARE_ALIGNED_ALLOCATOR();

		SkinnedMeshBone(const RE::BSFixedString& a_name, btRigidBody::btRigidBodyConstructionInfo& a_constructionInfo);
		virtual ~SkinnedMeshBone() = default;

		virtual void readTransform(float a_timeStep) = 0;
		virtual void writeTransform() = 0;

		void internalUpdate();
		bool canCollideWith(SkinnedMeshBone* a_rhs);

		RE::BSFixedString m_name;
		float m_marginMultipler{ 1.0F };
		float m_boudingSphereMultipler{ 1.0F };
		float m_gravityFactor{ 1.0F };
		float m_windFactor{ 1.0F };
		btRigidBody m_rig;
		btTransform m_localToRig{ btTransform::getIdentity() };
		btTransform m_rigToLocal{ btTransform::getIdentity() };
		btQsTransform m_currentTransform{ btQsTransform::getIdentity() };
		std::vector<RE::BSFixedString> m_canCollideWithBone;
		std::vector<RE::BSFixedString> m_noCollideWithBone;
	};
}
