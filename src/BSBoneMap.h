#pragma once

#include "RE/N/NiAVObject.h"

namespace Smp
{
	inline void RefreshBoneScatterTable(RE::NiAVObject* a_root)
	{
		if (!a_root || !REX::FModule::IsRuntimeOG()) {
			return;
		}

		using Create_t = void (*)(RE::NiAVObject*);
		static REL::Relocation<Create_t> create{ REL::ID{ 1131947, 0 } };
		create(a_root);
	}
}
