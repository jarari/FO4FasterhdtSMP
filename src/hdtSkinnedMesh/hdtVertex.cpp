#include "hdtVertex.h"

#include <algorithm>

namespace hdt
{
	void Vertex::sortWeight()
	{
		for (auto i = 0; i < 4; ++i) {
			for (auto j = 0; j < 3; ++j) {
				if (weight_[j] < weight_[j + 1]) {
					std::swap(weight_[j], weight_[j + 1]);
					std::swap(boneIdx_[j], boneIdx_[j + 1]);
				}
			}
		}
	}
}
