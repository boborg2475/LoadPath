#pragma once

#include <cstddef>
#include <functional>

namespace loadpath
{
// The host's threads: the library never creates one, and an answer never depends on how the host schedules these calls.
class IParallelFor
{
public:
	virtual ~IParallelFor() = default;
	// Calls Body(I) once for every I in [0, Count), possibly concurrently and in any order, and returns when all have finished.
	// For itself may be called from several threads at once (an authority shares its hook across concurrent solves), and each Body must run with the calling thread's floating-point environment.
	virtual void For(std::size_t Count, const std::function<void(std::size_t)>& Body) const = 0;
};
}
