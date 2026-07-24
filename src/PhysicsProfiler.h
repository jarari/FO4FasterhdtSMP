#pragma once

#include <cstdint>

namespace Smp::PhysicsProfiler
{
	void SetCapture(bool a_enabled, std::uint64_t a_sampleFrames, std::uint64_t a_printFrames);
	void AdvanceFrame();
}
