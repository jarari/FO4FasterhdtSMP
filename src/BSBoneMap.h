#pragma once

#include "RE/N/NiAVObject.h"

namespace Smp
{
	inline void RefreshBoneScatterTable(RE::NiAVObject* a_root)
	{
		if (!a_root) {
			return;
		}

		using Create_t = void (*)(RE::NiAVObject*);
		static REL::Relocation<Create_t> create{ REL::ID{ 1131947, 2276147 } };
		create(a_root);
	}
}
