#pragma once

#include "RE/N/NiNode.h"

#include <string_view>

namespace Smp::HeadPartModelSourceCache
{
	// Clone a model root that Fallout has already loaded and retain the private
	// snapshot by normalized NIF path. No resource lookup is performed here.
	RE::NiPointer<RE::NiNode> Capture(std::string_view a_nifPath, RE::NiNode* a_loadedRoot);

	// Return a retained private snapshot, or null when Fallout has not exposed a
	// relevant source for this path yet.
	RE::NiPointer<RE::NiNode> Find(std::string_view a_nifPath);
}
