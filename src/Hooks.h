#pragma once

namespace Smp
{
	struct RuntimeSettings;
}

namespace Hooks
{
	void ApplyConfig(const Smp::RuntimeSettings& a_settings);
	bool InstallLifecycleHooks();
}
