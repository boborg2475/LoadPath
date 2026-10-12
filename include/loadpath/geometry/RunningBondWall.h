#pragma once

#include "loadpath/Handles.h"
#include "loadpath/Structure.h"
#include "loadpath/Vec3.h"

#include <vector>

namespace loadpath::geometry
{
struct WallSpec
{
	int Courses = 10;
	int BricksPerCourse = 8;
	// Length along x, thickness along y, height along z.
	Vec3 BrickM = {0.215, 0.1025, 0.065};
	double DensityKgPerM3 = 2000.0;
	// The bed also serves the base course on the ground.
	ProfileId Bed, Perpend;
	// Odd courses offset by half a brick, with half bricks at the ends.
	bool bRunningBond = true;
	double GroundZM = 0.0;
};

struct Wall
{
	std::vector<std::vector<BodyId>> Courses;
	std::vector<InterfaceId> Beds, Perpends;
	BodyId Ground;
	double LengthM = 0, HeightM = 0;
	// False when any AddBody or AddInterface was refused (an Invalid profile): the wall is then not a wall.
	bool bComplete = false;
};

Wall BuildRunningBondWall(Structure& Model, const WallSpec& Spec);
}
