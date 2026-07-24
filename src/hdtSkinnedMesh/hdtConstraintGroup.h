#pragma once

#include "hdtBoneScaleConstraint.h"

namespace hdt
{
	class ConstraintGroup :
		public RefObject
	{
	public:
		void scaleConstraint()
		{
			for (const auto& constraint : m_constraints) {
				if (constraint) {
					constraint->scaleConstraint();
				}
			}
		}

		std::vector<RE::BSTSmartPointer<BoneScaleConstraint>> m_constraints;
	};
}
