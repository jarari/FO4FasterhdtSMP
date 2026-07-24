#include "hdtSkinnedMeshSystem.h"

#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshShape.h"

namespace hdt
{
	void SkinnedMeshSystem::readTransform(const float a_timeStep)
	{
		if (blockResetting) {
			return;
		}

		for (const auto& bone : bones_) {
			if (bone) {
				bone->readTransform(a_timeStep);
			}
		}
		for (const auto& constraint : constraints_) {
			if (constraint) {
				constraint->scaleConstraint();
			}
		}
		for (const auto& group : constraintGroups_) {
			if (group) {
				group->scaleConstraint();
			}
		}
	}

	void SkinnedMeshSystem::writeTransform()
	{
		for (const auto& bone : bones_) {
			if (!bone || bone->m_rig.isKinematicObject()) {
				continue;
			}

			bone->writeTransform();
		}
	}

	void SkinnedMeshSystem::internalUpdate()
	{

		for (const auto& bone : bones_) {
			if (bone) {
				bone->internalUpdate();
			}
		}
		for (const auto& mesh : meshes_) {
			if (mesh) {
				mesh->updateBoundingSphereAabb();
			}
		}
	}

	void SkinnedMeshSystem::gather(std::vector<SkinnedMeshBody*>& a_bodies, std::vector<SkinnedMeshShape*>& a_shapes)
	{
		for (const auto& mesh : meshes_) {
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

	void SkinnedMeshSystem::addBone(RE::BSTSmartPointer<SkinnedMeshBone> a_bone)
	{
		if (a_bone) {
			bones_.push_back(std::move(a_bone));
		}
	}

	void SkinnedMeshSystem::addMesh(RE::BSTSmartPointer<SkinnedMeshBody> a_mesh)
	{
		if (a_mesh) {
			meshes_.push_back(std::move(a_mesh));
		}
	}

	void SkinnedMeshSystem::addConstraint(RE::BSTSmartPointer<BoneScaleConstraint> a_constraint)
	{
		if (a_constraint) {
			constraints_.push_back(std::move(a_constraint));
		}
	}

	void SkinnedMeshSystem::addConstraintGroup(RE::BSTSmartPointer<ConstraintGroup> a_group)
	{
		if (a_group) {
			constraintGroups_.push_back(std::move(a_group));
		}
	}
}
