#pragma once

#include "Address.h"

namespace Smp
{
	inline void RefreshBoneScatterTable(RE::NiAVObject* a_root)
	{
		if (!a_root) {
			return;
		}

		Address::CreateBoneMap(a_root);
	}
}
