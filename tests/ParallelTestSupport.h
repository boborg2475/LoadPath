#pragma once

#include "BitwiseTestSupport.h"
#include "EventRegionTestSupport.h"

#include "loadpath/ParallelFor.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>
#include <utility>
#include <vector>

namespace loadpath::test
{
// Runs the indices one at a time on the calling thread, in the order Order(Count) returns.
class OrderedFor final : public IParallelFor
{
public:
	explicit OrderedFor(std::function<std::vector<std::size_t>(std::size_t)> InOrder) : Order(std::move(InOrder)) {}

	void For(std::size_t Count, const std::function<void(std::size_t)>& Body) const override
	{
		++Calls;
		for (const std::size_t I : Order(Count))
		{
			++Ran;
			Body(I);
		}
	}

	mutable std::size_t Calls = 0;
	mutable std::size_t Ran = 0;

private:
	std::function<std::vector<std::size_t>(std::size_t)> Order;
};

inline std::vector<std::size_t> AscendingOrder(std::size_t Count)
{
	std::vector<std::size_t> Order(Count);
	std::iota(Order.begin(), Order.end(), std::size_t{0});
	return Order;
}

inline OrderedFor ReverseFor()
{
	return OrderedFor([](std::size_t Count) {
		std::vector<std::size_t> Order = AscendingOrder(Count);
		std::reverse(Order.begin(), Order.end());
		return Order;
	});
}

// A Fisher-Yates shuffle drawn from SplitMix(Seed); each For call draws a fresh permutation from the same stream.
inline OrderedFor ShuffledFor(std::uint64_t Seed)
{
	auto Rng = std::make_shared<SplitMix>(Seed);
	return OrderedFor([Rng](std::size_t Count) {
		std::vector<std::size_t> Order = AscendingOrder(Count);
		for (std::size_t I = Count; I > 1; --I)
		{
			std::swap(Order[I - 1], Order[static_cast<std::size_t>(Rng->Between(0, static_cast<int>(I) - 1))]);
		}
		return Order;
	});
}

struct ForCall
{
	std::size_t Count = 0;
	std::vector<std::size_t> Ran;
};

// Serial underneath; logs every For call's Count and the indices it ran, in run order.
class RecordingFor final : public IParallelFor
{
public:
	void For(std::size_t Count, const std::function<void(std::size_t)>& Body) const override
	{
		Calls.push_back(ForCall{Count, {}});
		for (std::size_t I = 0; I < Count; ++I)
		{
			Calls.back().Ran.push_back(I);
			Body(I);
		}
	}

	mutable std::vector<ForCall> Calls;
};

// Workers threads pull indices from a shared counter; the calling thread only waits. Concurrent For calls are serialised.
class ThreadPoolFor final : public IParallelFor
{
public:
	explicit ThreadPoolFor(int Workers)
	{
		for (int T = 0; T < Workers; ++T)
		{
			Threads.emplace_back([this] { Work(); });
		}
	}

	~ThreadPoolFor() override
	{
		{
			const std::lock_guard<std::mutex> Lock(Mutex);
			bStop = true;
		}
		Wake.notify_all();
		for (std::thread& T : Threads)
		{
			T.join();
		}
	}

	ThreadPoolFor(const ThreadPoolFor&) = delete;
	ThreadPoolFor& operator=(const ThreadPoolFor&) = delete;

	void For(std::size_t Count, const std::function<void(std::size_t)>& Body) const override
	{
		const std::lock_guard<std::mutex> Serial(CallMutex);
		++Calls;
		if (Count == 0)
		{
			return;
		}
		std::unique_lock<std::mutex> Lock(Mutex);
		Job = &Body;
		JobCount = Count;
		Next = 0;
		Finished = 0;
		++Generation;
		Wake.notify_all();
		Done.wait(Lock, [this] { return Finished == JobCount; });
		Job = nullptr;
	}

	int WorkerCount() const { return static_cast<int>(Threads.size()); }

	mutable std::atomic<std::size_t> Ran{0};
	mutable std::size_t Calls = 0;

private:
	void Work()
	{
		std::uint64_t Seen = 0;
		std::unique_lock<std::mutex> Lock(Mutex);
		for (;;)
		{
			Wake.wait(Lock, [&] { return bStop || Generation != Seen; });
			if (bStop)
			{
				return;
			}
			Seen = Generation;
			while (Job != nullptr && Next < JobCount)
			{
				const std::size_t I = Next++;
				const std::function<void(std::size_t)>* Body = Job;
				Lock.unlock();
				(*Body)(I);
				++Ran;
				Lock.lock();
				if (++Finished == JobCount)
				{
					Done.notify_all();
				}
			}
		}
	}

	mutable std::mutex CallMutex;
	mutable std::mutex Mutex;
	mutable std::condition_variable Wake;
	mutable std::condition_variable Done;
	mutable const std::function<void(std::size_t)>* Job = nullptr;
	mutable std::size_t JobCount = 0;
	mutable std::size_t Next = 0;
	mutable std::size_t Finished = 0;
	mutable std::uint64_t Generation = 0;
	bool bStop = false;
	std::vector<std::thread> Threads;
};

// bitwise::Same plus StaleLoads; Timings are never compared.
inline bool SameParallel(const StandingResult& A, const StandingResult& B) { return bitwise::Same(A, B) && bitwise::Same(A.StaleLoads, B.StaleLoads); }

inline bool SameParallel(const AuthorityResult& A, const AuthorityResult& B) { return bitwise::Same(A, B) && bitwise::Same(A.StaleLoads, B.StaleLoads); }
}
