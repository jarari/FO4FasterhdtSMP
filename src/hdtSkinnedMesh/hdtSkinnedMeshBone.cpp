#include "hdtSkinnedMeshBone.h"

#include <algorithm>

namespace hdt
{
	SkinnedMeshBone::SkinnedMeshBone(const RE::BSFixedString& a_name, btRigidBody::btRigidBodyConstructionInfo& a_constructionInfo) :
		m_name(a_name),
		m_rig(a_constructionInfo)
	{
		m_rig.setUserPointer(this);
	}

	void SkinnedMeshBone::internalUpdate()
	{
		const auto transform = m_rigToLocal * m_rig.getInterpolationWorldTransform();
		m_currentTransform.setBasis(transform.getBasis());
		m_currentTransform.setOrigin(transform.getOrigin());
	}

	bool SkinnedMeshBone::canCollideWith(SkinnedMeshBone* a_rhs)
	{
		if (!a_rhs) {
			return false;
		}

		if (!m_canCollideWithBone.empty()) {
			return std::ranges::find(m_canCollideWithBone, a_rhs->m_name) != m_canCollideWithBone.end();
		}
		return std::ranges::find(m_noCollideWithBone, a_rhs->m_name) == m_noCollideWithBone.end();
	}
}
