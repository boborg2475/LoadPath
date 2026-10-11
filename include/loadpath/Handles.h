#pragma once

#include <cstdint>

namespace loadpath
{
// Generation 0 is the invalid handle, so a value-initialised handle is invalid.
struct BodyId
{
	std::uint32_t Index = 0;
	std::uint32_t Generation = 0;

	static constexpr BodyId Invalid() { return {}; }
	bool operator==(const BodyId& Other) const;
};

struct InterfaceId
{
	std::uint32_t Index = 0;
	std::uint32_t Generation = 0;

	static constexpr InterfaceId Invalid() { return {}; }
	bool operator==(const InterfaceId& Other) const;
};

struct ProfileId
{
	std::uint32_t Index = 0;
	std::uint32_t Generation = 0;

	static constexpr ProfileId Invalid() { return {}; }
	bool operator==(const ProfileId& Other) const;
};
}
