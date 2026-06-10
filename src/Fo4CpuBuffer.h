#pragma once

#include "RE/B/BSGraphics.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <spdlog/spdlog.h>

namespace Smp::Fo4CpuBuffer
{
	struct ResolvedBuffer
	{
		const std::uint8_t* data{ nullptr };
		std::uint64_t availableBytes{ 0 };
		bool usedRawDataFallback{ false };
	};

	[[nodiscard]] inline std::uint64_t GetAvailableBytes(const RE::BSGraphics::Buffer* a_buffer)
	{
		if (!a_buffer) {
			return 0;
		}
		if (a_buffer->heapAllocated) {
			return a_buffer->dataSize;
		}
		return a_buffer->dataSize > a_buffer->dataOffset ? a_buffer->dataSize - a_buffer->dataOffset : 0;
	}

	[[nodiscard]] inline const std::uint8_t* GetDataStart(const RE::BSGraphics::Buffer* a_buffer)
	{
		if (!a_buffer || !a_buffer->data) {
			return nullptr;
		}

		const auto* data = static_cast<const std::uint8_t*>(a_buffer->data);
		return data + (a_buffer->heapAllocated ? 0 : a_buffer->dataOffset);
	}

	[[nodiscard]] inline bool ResolveReadable(
		const std::string& a_meshName,
		const char* a_kind,
		const RE::BSGraphics::Buffer* a_buffer,
		const std::uint64_t a_requiredBytes,
		ResolvedBuffer& a_result)
	{
		if (!a_buffer || !a_buffer->data || a_requiredBytes == 0 || a_requiredBytes > std::numeric_limits<std::size_t>::max()) {
			return false;
		}

		const auto* rawData = static_cast<const std::uint8_t*>(a_buffer->data);

		if (a_buffer->heapAllocated && a_buffer->dataSize >= a_requiredBytes) {
			a_result.data = rawData;
			a_result.availableBytes = a_buffer->dataSize;
			a_result.usedRawDataFallback = false;
			return true;
		}

		const auto primaryAvailableBytes = a_buffer->dataSize > a_buffer->dataOffset ? a_buffer->dataSize - a_buffer->dataOffset : 0;
		if (primaryAvailableBytes >= a_requiredBytes) {
			const auto* primaryData = rawData + a_buffer->dataOffset;
			a_result.data = primaryData;
			a_result.availableBytes = primaryAvailableBytes;
			a_result.usedRawDataFallback = false;
			return true;
		}

		if (!a_buffer->heapAllocated && a_buffer->dataOffset != 0 && a_buffer->dataSize >= a_requiredBytes) {
			spdlog::warn(
				"mesh '{}' using CPU {} data without dataOffset fallback data={} badOffset={} dataSize={} requiredBytes={}",
				a_meshName,
				a_kind,
				a_buffer->data,
				a_buffer->dataOffset,
				a_buffer->dataSize,
				a_requiredBytes);
			a_result.data = rawData;
			a_result.availableBytes = a_buffer->dataSize;
			a_result.usedRawDataFallback = true;
			return true;
		}

		return false;
	}
}
