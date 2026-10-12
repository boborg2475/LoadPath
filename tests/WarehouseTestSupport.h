#pragma once

// Warehouse specs shared by the geometry and scale tests: the 12x8 shed and the half warehouse.

#include "loadpath/geometry/Warehouse.h"

namespace loadpath::test
{

inline geometry::WarehouseSpec ShedSpec()
{
	geometry::WarehouseSpec Spec;
	Spec.LengthBricks = 12;
	Spec.WidthBricks = 8;
	Spec.Courses = 10;
	Spec.Bays = 2;
	Spec.PlanksPerBay = 3;
	Spec.Openings = {geometry::Opening{0, 5, 2, 0, 7}, geometry::Opening{2, 3, 2, 4, 8}};
	return Spec;
}

inline geometry::WarehouseSpec HalfWarehouseSpec()
{
	geometry::WarehouseSpec Spec;
	Spec.LengthBricks = 30;
	Spec.WidthBricks = 15;
	Spec.Courses = 20;
	Spec.Bays = 3;
	Spec.PlanksPerBay = 4;
	Spec.Openings = {geometry::Opening{0, 10, 3, 0, 12}, geometry::Opening{2, 8, 3, 8, 14}};
	return Spec;
}

}
