#include "PhysicsProfiler.h"

#include <LinearMath/btQuickprof.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Smp::PhysicsProfiler
{
	using Clock = std::chrono::steady_clock;

	namespace
	{
		struct ScopeStat
		{
			std::string name;
			std::string path;
			std::uint64_t totalNs{ 0 };
			std::uint64_t calls{ 0 };
		};

		struct ScopeFrame
		{
			ScopeStat* stat{ nullptr };
			Clock::time_point start;
			bool root{ false };
		};

		struct ThreadState
		{
			explicit ThreadState(const std::uint32_t a_index) :
				index(a_index)
			{}

			std::uint32_t index{ 0 };
			std::unordered_map<std::string, ScopeStat> scopes;
			std::vector<ScopeFrame> stack;
			std::uint64_t rootTotalNs{ 0 };
			std::uint64_t rootCalls{ 0 };
		};

		struct AggregateStat
		{
			std::string name;
			std::string path;
			std::uint64_t totalNs{ 0 };
			std::uint64_t calls{ 0 };
		};

		struct ThreadRow
		{
			std::uint32_t index{ 0 };
			std::uint64_t totalNs{ 0 };
			std::uint64_t calls{ 0 };
		};

		std::mutex g_threadsLock;
		std::vector<std::unique_ptr<ThreadState>> g_threads;
		std::atomic_uint32_t g_nextThreadIndex{ 0 };
		std::atomic_uint32_t g_activeScopes{ 0 };
		std::atomic_bool g_captureEnabled{ false };
		std::uint64_t g_windowFrames{ 0 };
		std::uint64_t g_totalFrames{ 0 };
		std::uint64_t g_sampleWindowFrames{ 0 };
		std::uint64_t g_printIntervalFrames{ 0 };
		Clock::time_point g_windowStart = Clock::now();

		thread_local ThreadState* g_threadState = nullptr;

		double NsToMs(const std::uint64_t a_nanoseconds)
		{
			return static_cast<double>(a_nanoseconds) / 1'000'000.0;
		}

		std::uint64_t ElapsedNs(const Clock::time_point a_start, const Clock::time_point a_end)
		{
			return static_cast<std::uint64_t>(
				std::chrono::duration_cast<std::chrono::nanoseconds>(a_end - a_start).count());
		}

		ThreadState& GetThreadState()
		{
			if (g_threadState) {
				return *g_threadState;
			}

			auto state = std::make_unique<ThreadState>(g_nextThreadIndex.fetch_add(1, std::memory_order_relaxed));
			g_threadState = state.get();
			std::scoped_lock lock(g_threadsLock);
			g_threads.push_back(std::move(state));
			return *g_threadState;
		}

		void Enter(const char* a_name) noexcept
		{
			if (!a_name || !g_captureEnabled.load(std::memory_order_acquire)) {
				return;
			}

			g_activeScopes.fetch_add(1, std::memory_order_acq_rel);
			try {
				auto& thread = GetThreadState();
				const auto root = thread.stack.empty();
				const auto path = root ?
					std::string(a_name) :
					thread.stack.back().stat->path + " / " + a_name;
				auto [it, inserted] = thread.scopes.try_emplace(path);
				auto& stat = it->second;
				if (inserted) {
					stat.name = a_name;
					stat.path = path;
				}
				++stat.calls;
				thread.stack.push_back({
					.stat = std::addressof(stat),
					.start = Clock::now(),
					.root = root,
				});
			} catch (...) {
				g_activeScopes.fetch_sub(1, std::memory_order_acq_rel);
			}
		}

		void Leave() noexcept
		{
			try {
				auto* thread = g_threadState;
				if (!thread || thread->stack.empty()) {
					return;
				}

				const auto frame = thread->stack.back();
				thread->stack.pop_back();
				const auto elapsed = ElapsedNs(frame.start, Clock::now());
				frame.stat->totalNs += elapsed;
				if (frame.root) {
					thread->rootTotalNs += elapsed;
					++thread->rootCalls;
				}
				g_activeScopes.fetch_sub(1, std::memory_order_acq_rel);
			} catch (...) {
			}
		}

		void EmptyEnter(const char*) noexcept
		{}

		void EmptyLeave() noexcept
		{}

		void ResetUnlocked(const bool a_resetTotalFrames)
		{
			for (auto& thread : g_threads) {
				thread->scopes.clear();
				thread->stack.clear();
				thread->rootTotalNs = 0;
				thread->rootCalls = 0;
			}
			g_windowFrames = 0;
			if (a_resetTotalFrames) {
				g_totalFrames = 0;
			}
			g_windowStart = Clock::now();
		}

		void LogRow(
			const std::string_view a_scope,
			const std::uint64_t a_totalNs,
			const std::uint64_t a_parentNs,
			const std::uint64_t a_calls,
			const std::uint64_t a_frames)
		{
			const auto totalMs = NsToMs(a_totalNs);
			const auto msPerFrame = totalMs / static_cast<double>(std::max<std::uint64_t>(a_frames, 1));
			const auto msPerCall = a_calls > 0 ? totalMs / static_cast<double>(a_calls) : 0.0;
			const auto percent = a_parentNs > 0 ?
				static_cast<double>(a_totalNs) * 100.0 / static_cast<double>(a_parentNs) :
				0.0;
			spdlog::info(
				"{:<80} {:>12.3f} ms {:>10.3f} ms/frame {:>10.3f} ms/call {:>8.2f}% {:>8}",
				a_scope,
				totalMs,
				msPerFrame,
				msPerCall,
				percent,
				a_calls);
		}

		bool DumpAndReset()
		{
			const auto activeScopes = g_activeScopes.load(std::memory_order_acquire);
			if (activeScopes != 0) {
				spdlog::warn("Physics profile dump skipped because {} profile scopes are still active", activeScopes);
				return false;
			}

			std::unordered_map<std::string, AggregateStat> tree;
			std::unordered_map<std::string, AggregateStat> flat;
			std::vector<ThreadRow> threads;
			std::uint64_t frames = 0;
			std::uint64_t recordedCpuNs = 0;
			Clock::time_point windowStart;
			{
				std::scoped_lock lock(g_threadsLock);
				frames = std::max<std::uint64_t>(g_windowFrames, 1);
				windowStart = g_windowStart;
				for (const auto& thread : g_threads) {
					if (thread->rootTotalNs != 0 || thread->rootCalls != 0) {
						threads.push_back({ thread->index, thread->rootTotalNs, thread->rootCalls });
						recordedCpuNs += thread->rootTotalNs;
					}
					for (const auto& [path, stat] : thread->scopes) {
						auto& treeRow = tree[path];
						treeRow.name = stat.name;
						treeRow.path = stat.path;
						treeRow.totalNs += stat.totalNs;
						treeRow.calls += stat.calls;

						auto& flatRow = flat[stat.name];
						flatRow.name = stat.name;
						flatRow.path = stat.name;
						flatRow.totalNs += stat.totalNs;
						flatRow.calls += stat.calls;
					}
				}
				ResetUnlocked(false);
			}

			if (recordedCpuNs == 0) {
				spdlog::info("Physics profile had no recorded scopes");
				return true;
			}

			const auto wallNs = ElapsedNs(windowStart, Clock::now());
			const auto parallelism = wallNs > 0 ?
				static_cast<double>(recordedCpuNs) / static_cast<double>(wallNs) :
				0.0;
			spdlog::info(
				"Physics profile: {} frames, {:.3f} ms wall, {:.3f} ms recorded CPU, {:.2f}x recorded parallelism",
				frames,
				NsToMs(wallNs),
				NsToMs(recordedCpuNs),
				parallelism);
			spdlog::info(
				"{:<80} {:>16} {:>18} {:>18} {:>9} {:>8}",
				"Scope",
				"Total CPU",
				"Avg/frame",
				"Avg/call",
				"Parent%",
				"Calls");

			std::vector<const AggregateStat*> treeRows;
			treeRows.reserve(tree.size());
			for (const auto& [path, row] : tree) {
				if (NsToMs(row.totalNs) >= 0.001) {
					treeRows.push_back(std::addressof(row));
				}
			}
			std::ranges::sort(treeRows, [](const auto* a_lhs, const auto* a_rhs) {
				return a_lhs->path < a_rhs->path;
			});
			for (const auto* row : treeRows) {
				const auto separator = row->path.rfind(" / ");
				const auto parentPath = separator == std::string::npos ?
					std::string{} :
					row->path.substr(0, separator);
				const auto parent = parentPath.empty() ? tree.end() : tree.find(parentPath);
				LogRow(
					row->path,
					row->totalNs,
					parent != tree.end() ? parent->second.totalNs : recordedCpuNs,
					row->calls,
					frames);
			}

			std::vector<const AggregateStat*> flatRows;
			flatRows.reserve(flat.size());
			for (const auto& [name, row] : flat) {
				if (NsToMs(row.totalNs) >= 0.001) {
					flatRows.push_back(std::addressof(row));
				}
			}
			std::ranges::sort(flatRows, [](const auto* a_lhs, const auto* a_rhs) {
				return a_lhs->totalNs > a_rhs->totalNs;
			});
			spdlog::info("Physics profile flat hotspots");
			for (std::size_t index = 0; index < std::min<std::size_t>(flatRows.size(), 32); ++index) {
				const auto* row = flatRows[index];
				LogRow(row->name, row->totalNs, recordedCpuNs, row->calls, frames);
			}

			std::ranges::sort(threads, [](const auto& a_lhs, const auto& a_rhs) {
				return a_lhs.totalNs > a_rhs.totalNs;
			});
			spdlog::info("Physics profile threads");
			for (const auto& thread : threads) {
				spdlog::info(
					"T{:<10} {:>10.3f} ms {:>10.3f} ms/frame {:>8} calls",
					thread.index,
					NsToMs(thread.totalNs),
					NsToMs(thread.totalNs) / static_cast<double>(frames),
					thread.calls);
			}
			return true;
		}
	}

	void SetCapture(
		const bool a_enabled,
		const std::uint64_t a_sampleFrames,
		const std::uint64_t a_printFrames)
	{
		btSetProfileEnabled(false);
		g_captureEnabled.store(false, std::memory_order_release);

		if (a_enabled) {
			{
				std::scoped_lock lock(g_threadsLock);
				ResetUnlocked(true);
				g_sampleWindowFrames = std::max<std::uint64_t>(a_sampleFrames, 1);
				g_printIntervalFrames = std::max<std::uint64_t>(a_printFrames, 1);
			}
			btSetCustomEnterProfileZoneFunc(&Enter);
			btSetCustomLeaveProfileZoneFunc(&Leave);
			g_captureEnabled.store(true, std::memory_order_release);
			btSetProfileEnabled(true);
		} else {
			g_sampleWindowFrames = 0;
			g_printIntervalFrames = 0;
			btSetCustomEnterProfileZoneFunc(&EmptyEnter);
			btSetCustomLeaveProfileZoneFunc(&EmptyLeave);
			std::scoped_lock lock(g_threadsLock);
			ResetUnlocked(true);
		}
	}

	void AdvanceFrame()
	{
		if (!g_captureEnabled.load(std::memory_order_acquire)) {
			return;
		}

		++g_windowFrames;
		++g_totalFrames;
		if (g_totalFrames != 0 && g_totalFrames % g_printIntervalFrames == 0) {
			DumpAndReset();
		} else if (g_windowFrames >= g_sampleWindowFrames &&
				   g_activeScopes.load(std::memory_order_acquire) == 0) {
			std::scoped_lock lock(g_threadsLock);
			ResetUnlocked(false);
		}
	}
}
