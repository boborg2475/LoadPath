#pragma once

#include "loadpath/Handles.h"
#include "loadpath/Structure.h"
#include "loadpath/Vec3.h"
#include "loadpath/geometry/RunningBondWall.h"

#include <array>
#include <cstddef>
#include <vector>

namespace loadpath::geometry
{
// Wall 0 and 2 are the long walls along x at y = 0 and y = width, 1 and 3 the short walls along y at x = 0 and x = length.
// A brick is removed when its span along the wall lies inside [StartBrick, StartBrick + WidthBricks) brick lengths and its course inside [SillCourse, LintelCourse).
struct Opening
{
	int Wall = 0;
	int StartBrick = 0;
	int WidthBricks = 0;
	int SillCourse = 0;
	int LintelCourse = 0;
};

struct WarehouseSpec
{
	int LengthBricks = 40;
	int WidthBricks = 20;
	int Courses = 30;
	Vec3 BrickM = {0.215, 0.1025, 0.065};
	double DensityKgPerM3 = 2000.0;
	int Bays = 4;
	double PostSideM = 0.2;
	double BeamDepthM = 0.3;
	double BeamWidthM = 0.2;
	double PlankThicknessM = 0.05;
	double PlankWidthM = 0.2;
	int PlanksPerBay = 4;
	double TimberDensityKgPerM3 = 420.0;
	ProfileId Bed, Perpend, Bearing, Plank;
	std::vector<Opening> Openings;
	double GroundZM = 0.0;
	// The open gap between a short wall's inner face and the long walls' ends: the corners are not bonded.
	double CornerGapM = 0.01;
};

struct Warehouse
{
	std::array<Wall, 4> Walls;
	std::vector<BodyId> Posts, Beams, Planks;
	std::vector<InterfaceId> Bearings, PlankJoints;
	BodyId Ground;
	bool bComplete = false;
	// Live bodies, the ground included.
	std::size_t BodyCount() const;
	std::size_t InterfaceCount() const;
};

Warehouse BuildWarehouse(Structure& Model, const WarehouseSpec& Spec);
}
