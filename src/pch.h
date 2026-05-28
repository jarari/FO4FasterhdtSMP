#pragma once

#include <climits>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>

#pragma warning(push)
#include <F4SE/F4SE.h>
#include <RE/Fallout.h>
#include <REL/ASM.h>
#include <REX/REX.h>

#pragma warning(pop)

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/msvc_sink.h>
#include <spdlog/spdlog.h>

using namespace std::literals;
