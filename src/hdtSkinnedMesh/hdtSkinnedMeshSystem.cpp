#include "hdtSkinnedMeshSystem.h"

#include "hdtBoneScaleConstraint.h"
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshShape.h"

namespace hdt
{
	void SkinnedMeshSystem::readTransform(const float a_timeStep)
	{
		if (block_resetting) {
			return;
		}

		for (const auto& bone : m_bones) {
			if (bone) {
				bone->readTransform(a_timeStep);
			}
		}
		for (const auto& constraint : m_constraints) {
			if (constraint) {
				constraint->scaleConstraint();
			}
		}
		for (const auto& group : m_constraintGroups) {
			if (group) {
				group->scaleConstraint();
			}
		}
	}

	void SkinnedMeshSystem::writeTransform()
	{
		for (const auto& bone : m_bones) {
			if (!bone || bone->m_rig.isKinematicObject()) {
				continue;
			}

			bone->writeTransform();
		}
	}

	void SkinnedMeshSystem::internalUpdate()
	{
		for (const auto& bone : m_bones) {
			if (bone) {
				bone->internalUpdate();
			}
		}
		for (const auto& mesh : m_meshes) {
			if (mesh) {
				mesh->updateBoundingSphereAabb();
			}
		}
	}

	void SkinnedMeshSystem::gather(std::vector<SkinnedMeshBody*>& a_bodies, std::vector<SkinnedMeshShape*>& a_shapes)
	{
		for (const auto& mesh : m_meshes) {
			if (!mesh) {
				continue;
			}
			a_bodies.push_back(mesh.get());
			if (mesh->shape_) {
				a_shapes.push_back(mesh->shape_.get());
				if (auto* triangleShape = mesh->shape_->asPerTriangleShape()) {
					if (auto* vertexShape = triangleShape->asPerVertexShape()) {
						a_shapes.push_back(vertexShape);
					}
				}
			}
		}
	}
}
