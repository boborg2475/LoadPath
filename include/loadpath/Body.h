#pragma once

#include "loadpath/Vec3.h"

namespace loadpath
{
// Zero mass is a legitimate massless body; a grounded body never moves and absorbs any load.
struct BodyDesc
{
	double MassKg = 0;
	Vec3 CentroidM;
	bool bGrounded = false;
};
}
