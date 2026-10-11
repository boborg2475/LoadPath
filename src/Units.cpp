#include "loadpath/Units.h"

#include <cmath>

namespace loadpath
{
bool IsFinite(double X)
{
	return std::isfinite(X);
}
}
