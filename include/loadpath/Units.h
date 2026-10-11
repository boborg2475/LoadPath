#pragma once

namespace loadpath
{
inline constexpr double GravityMPerS2 = 9.80665;

// False for NaN and for either infinity.
bool IsFinite(double X);
}
