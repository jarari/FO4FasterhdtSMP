#include "hdtCollider.h"

#include <algorithm>

namespace hdt
{
	void ColliderTree::insertCollider(const U32* a_keys, std::size_t a_keyCount, const Collider& a_collider)
	{
		auto* node = this;
		for (std::size_t i = 0; i < a_keyCount && i < 4; ++i) {
			const auto child = std::find_if(node->children_.begin(), node->children_.end(), [=](const ColliderTree& a_tree) {
				return a_tree.key_ == a_keys[i];
			});

			if (child == node->children_.end()) {
				node->children_.push_back(ColliderTree(a_keys[i]));
				node = std::addressof(node->children_.back());
			} else {
				node = std::addressof(*child);
			}
		}

		node->colliders_.push_back(a_collider);
	}

	void ColliderTree::checkCollisionL(ColliderTree* a_rhs, std::vector<std::pair<ColliderTree*, ColliderTree*>>& a_result)
	{
		enum class Mode : std::uint8_t
		{
			kLeft,
			kRight
		};

		struct Entry
		{
			ColliderTree* lhs;
			ColliderTree* rhs;
			Mode mode;
		};

		thread_local std::vector<Entry> stack;
		stack.clear();
		stack.push_back({ this, a_rhs, Mode::kLeft });

		while (!stack.empty()) {
			if (a_result.size() > MaxCollisionPairs) {
				break;
			}

			const auto entry = stack.back();
			stack.pop_back();

			if (entry.lhs->isKinematic_ && entry.rhs->isKinematic_) {
				continue;
			}

			if (entry.mode == Mode::kLeft) {
				if (!entry.lhs->aabbAll_.collideWith(entry.rhs->aabbAll_)) {
					continue;
				}

				if (entry.lhs->numCollider_ && entry.lhs->aabbMe_.collideWith(entry.rhs->aabbAll_)) {
					if (entry.lhs->aabbMe_.collideWith(entry.rhs->aabbMe_)) {
						a_result.emplace_back(entry.lhs, entry.rhs);
					}

					const auto begin = entry.rhs->children_.data();
					const auto end = begin + (entry.lhs->isKinematic_ ? entry.rhs->dynChild_ : entry.rhs->children_.size());
					for (auto child = begin; child < end; ++child) {
						stack.push_back({ entry.lhs, child, Mode::kRight });
					}
				}

				const auto begin = entry.lhs->children_.data();
				const auto end = begin + (entry.rhs->isKinematic_ ? entry.lhs->dynChild_ : entry.lhs->children_.size());
				for (auto child = begin; child < end; ++child) {
					stack.push_back({ child, entry.rhs, Mode::kLeft });
				}
			} else {
				if (!entry.lhs->numCollider_ || !entry.lhs->aabbMe_.collideWith(entry.rhs->aabbAll_)) {
					continue;
				}

				if (entry.lhs->aabbMe_.collideWith(entry.rhs->aabbMe_)) {
					a_result.emplace_back(entry.lhs, entry.rhs);
				}

				const auto begin = entry.rhs->children_.data();
				const auto end = begin + (entry.lhs->isKinematic_ ? entry.rhs->dynChild_ : entry.rhs->children_.size());
				for (auto child = begin; child < end; ++child) {
					stack.push_back({ entry.lhs, child, Mode::kRight });
				}
			}
		}
	}

	void ColliderTree::checkCollisionR(ColliderTree* a_rhs, std::vector<std::pair<ColliderTree*, ColliderTree*>>& a_result)
	{
		if (isKinematic_ && a_rhs->isKinematic_) {
			return;
		}

		if (!numCollider_ || !aabbMe_.collideWith(a_rhs->aabbAll_)) {
			return;
		}

		if (aabbMe_.collideWith(a_rhs->aabbMe_)) {
			a_result.emplace_back(this, a_rhs);
		}

		const auto begin = a_rhs->children_.data();
		const auto end = begin + (isKinematic_ ? a_rhs->dynChild_ : a_rhs->children_.size());
		for (auto child = begin; child < end; ++child) {
			checkCollisionR(child, a_result);
		}
	}

	void ColliderTree::clipCollider(const std::function<bool(const Collider&)>& a_func)
	{
		for (auto& child : children_) {
			child.clipCollider(a_func);
		}

		colliders_.erase(std::remove_if(colliders_.begin(), colliders_.end(), a_func), colliders_.end());
		children_.erase(
			std::remove_if(children_.begin(), children_.end(), [](const ColliderTree& a_tree) { return a_tree.empty(); }),
			children_.end());
	}

	void ColliderTree::updateKinematic(const std::function<float(const Collider*)>& a_func)
	{
		U32 kinematic = true;
		for (auto& collider : colliders_) {
			collider.flexible_ = a_func(std::addressof(collider));
			kinematic &= collider.flexible_ < FLT_EPSILON;
		}

		for (auto& child : children_) {
			child.updateKinematic(a_func);
			kinematic &= child.isKinematic_;
		}

		std::sort(colliders_.begin(), colliders_.end(), [](const Collider& a_lhs, const Collider& a_rhs) {
			return a_lhs.flexible_ > a_rhs.flexible_;
		});
		std::sort(children_.begin(), children_.end(), [](const ColliderTree& a_lhs, const ColliderTree& a_rhs) {
			return a_lhs.isKinematic_ < a_rhs.isKinematic_;
		});

		isKinematic_ = kinematic;
		if (kinematic) {
			dynChild_ = 0;
			dynCollider_ = 0;
			return;
		}

		for (dynChild_ = 0; dynChild_ < children_.size(); ++dynChild_) {
			if (children_[dynChild_].isKinematic_) {
				break;
			}
		}

		for (dynCollider_ = 0; dynCollider_ < colliders_.size(); ++dynCollider_) {
			if (colliders_[dynCollider_].flexible_ < FLT_EPSILON) {
				break;
			}
		}
	}

	void ColliderTree::updateAabb()
	{
		struct Frame
		{
			ColliderTree* node;
			std::size_t childIndex;
		};

		thread_local std::vector<Frame> stack;
		stack.clear();
		stack.push_back({ this, 0 });

		while (!stack.empty()) {
			auto& frame = stack.back();
			if (frame.childIndex < frame.node->children_.size()) {
				auto* child = std::addressof(frame.node->children_[frame.childIndex++]);
				stack.push_back({ child, 0 });
				continue;
			}

			auto* node = frame.node;
			stack.pop_back();

			if (node->numCollider_) {
				auto min = node->aabb_[0].min_;
				auto max = node->aabb_[0].max_;
				auto* aabb = node->aabb_ + 1;
				const auto* end = node->aabb_ + node->numCollider_;
				for (; aabb < end; ++aabb) {
					min = _mm_min_ps(min, aabb->min_);
					max = _mm_max_ps(max, aabb->max_);
				}
				node->aabbMe_.min_ = min;
				node->aabbMe_.max_ = max;
			}

			auto allMin = node->aabbMe_.min_;
			auto allMax = node->aabbMe_.max_;
			for (auto& child : node->children_) {
				allMin = _mm_min_ps(allMin, child.aabbAll_.min_);
				allMax = _mm_max_ps(allMax, child.aabbAll_.max_);
			}
			node->aabbAll_.min_ = allMin;
			node->aabbAll_.max_ = allMax;
		}
	}

	void ColliderTree::visitColliders(const std::function<void(Collider*)>& a_func)
	{
		for (auto& collider : colliders_) {
			a_func(std::addressof(collider));
		}

		for (auto& child : children_) {
			child.visitColliders(a_func);
		}
	}

	void ColliderTree::optimize()
	{
		for (auto& child : children_) {
			child.optimize();
		}

		children_.erase(
			std::remove_if(children_.begin(), children_.end(), [](const ColliderTree& a_tree) { return a_tree.empty(); }),
			children_.end());

		while (children_.size() == 1 && children_[0].colliders_.empty()) {
			vectorA16<ColliderTree> temp;
			temp.swap(children_.front().children_);
			children_.swap(temp);
		}

		if (children_.size() == 1 && colliders_.empty()) {
			colliders_ = children_[0].colliders_;
			vectorA16<ColliderTree> temp;
			temp.swap(children_[0].children_);
			children_.swap(temp);
		}
	}

	bool ColliderTree::collapseCollideL(ColliderTree* a_rhs)
	{
		enum class Mode : std::uint8_t
		{
			kLeft,
			kRight
		};

		struct Entry
		{
			ColliderTree* lhs;
			ColliderTree* rhs;
			Mode mode;
		};

		thread_local std::vector<Entry> stack;
		stack.clear();
		stack.push_back({ this, a_rhs, Mode::kLeft });

		while (!stack.empty()) {
			const auto entry = stack.back();
			stack.pop_back();

			if (entry.lhs->isKinematic_ && entry.rhs->isKinematic_) {
				continue;
			}

			if (entry.mode == Mode::kLeft) {
				if (!entry.lhs->aabbAll_.collideWith(entry.rhs->aabbAll_)) {
					continue;
				}

				if (entry.lhs->numCollider_ && entry.lhs->aabbMe_.collideWith(entry.rhs->aabbAll_)) {
					if (entry.lhs->aabbMe_.collideWith(entry.rhs->aabbMe_)) {
						return true;
					}

					const auto begin = entry.rhs->children_.data();
					const auto end = begin + (entry.lhs->isKinematic_ ? entry.rhs->dynChild_ : entry.rhs->children_.size());
					for (auto child = begin; child < end; ++child) {
						stack.push_back({ entry.lhs, child, Mode::kRight });
					}
				}

				const auto begin = entry.lhs->children_.data();
				const auto end = begin + (entry.rhs->isKinematic_ ? entry.lhs->dynChild_ : entry.lhs->children_.size());
				for (auto child = begin; child < end; ++child) {
					stack.push_back({ child, entry.rhs, Mode::kLeft });
				}
			} else {
				if (!entry.lhs->numCollider_ || !entry.lhs->aabbMe_.collideWith(entry.rhs->aabbAll_)) {
					continue;
				}

				if (entry.lhs->aabbMe_.collideWith(entry.rhs->aabbMe_)) {
					return true;
				}

				const auto begin = entry.rhs->children_.data();
				const auto end = begin + (entry.lhs->isKinematic_ ? entry.rhs->dynChild_ : entry.rhs->children_.size());
				for (auto child = begin; child < end; ++child) {
					stack.push_back({ entry.lhs, child, Mode::kRight });
				}
			}
		}

		return false;
	}

	bool ColliderTree::collapseCollideR(ColliderTree* a_rhs)
	{
		if (isKinematic_ && a_rhs->isKinematic_) {
			return false;
		}

		if (!numCollider_ || !aabbMe_.collideWith(a_rhs->aabbAll_)) {
			return false;
		}

		if (aabbMe_.collideWith(a_rhs->aabbMe_)) {
			return true;
		}

		const auto begin = a_rhs->children_.data();
		const auto end = begin + (isKinematic_ ? a_rhs->dynChild_ : a_rhs->children_.size());
		for (auto child = begin; child < end; ++child) {
			if (collapseCollideR(child)) {
				return true;
			}
		}

		return false;
	}

	void ColliderTree::exportColliders(vectorA16<Collider>& a_exportTo)
	{
		numCollider_ = static_cast<U32>(colliders_.size());
		colliderBuffer_ = reinterpret_cast<Collider*>(a_exportTo.size());
		for (auto& collider : colliders_) {
			a_exportTo.push_back(collider);
		}

		for (auto& child : children_) {
			child.exportColliders(a_exportTo);
		}
	}

	void ColliderTree::remapColliders(Collider* a_start, Aabb* a_startAabb)
	{
		vectorA16<Collider> temp;
		colliders_.swap(temp);
		const auto offset = reinterpret_cast<std::size_t>(colliderBuffer_);
		colliderBuffer_ = a_start + offset;
		aabb_ = a_startAabb + offset;

		for (auto& child : children_) {
			child.remapColliders(a_start, a_startAabb);
		}
	}
}
