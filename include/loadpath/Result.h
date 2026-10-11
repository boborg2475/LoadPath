#pragma once

#include <cstdint>

namespace loadpath
{
enum class ModelError : std::uint8_t
{
	None,
	NonFiniteInput,
	NegativeMass,
	DegenerateNormal,
	AxesNotOrthonormal,
	NonPositiveExtent,
	UnknownBody,
	UnknownProfile,
	SameBody,
	BodyRemoved,
	NegativeStrength,
	NonPositiveStiffness
};

// A refused add carries the invalid handle and the reason, and changed nothing.
template <class T>
struct Result
{
	T Value{};
	ModelError Error = ModelError::None;

	bool Ok() const { return Error == ModelError::None; }
};
}
