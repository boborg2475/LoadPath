#pragma once

#include "CatalogueTestSupport.h"
#include "StaticsTestSupport.h"
#include "WarehouseTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/Structure.h"
#include "loadpath/geometry/RunningBondWall.h"
#include "loadpath/geometry/Warehouse.h"

#include "doctest.h"

#include <cstddef>
#include <string>
#include <vector>

namespace loadpath::test
{
struct ScaleSubject
{
	std::string Name;
	Structure Model;
	std::size_t ExpectedBodies = 0;
	std::vector<BodyId> Posts;
	std::vector<BodyId> BaseCourse;
	bool bMustStand = true;
};

struct MortarRows
{
	StrengthProfile Bed, Perpend, Bearing;
	FastenerRow Screw;
};

inline MortarRows LoadMortarRows()
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	return MortarRows{JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, JointOrFail(Loaded.Value, "c24_bearing_perp_to_grain").Profile,
		FastenerOrFail(Loaded.Value, "ec5_screw_8x60_steel_to_glulam")};
}

inline std::size_t WallBricks(int Courses, int BricksPerCourse) { return static_cast<std::size_t>(Courses * BricksPerCourse + Courses / 2); }

inline std::size_t OpeningBricks(const geometry::Opening& O)
{
	std::size_t N = 0;
	for (int C = O.SillCourse; C < O.LintelCourse; ++C)
	{
		N += static_cast<std::size_t>(C % 2 == 0 ? O.WidthBricks : O.WidthBricks - 1);
	}
	return N;
}

inline std::size_t WarehouseBodies(const geometry::WarehouseSpec& S)
{
	std::size_t N = 2 * WallBricks(S.Courses, S.LengthBricks) + 2 * WallBricks(S.Courses, S.WidthBricks);
	for (const geometry::Opening& O : S.Openings)
	{
		N -= OpeningBricks(O);
	}
	const std::size_t Bays = static_cast<std::size_t>(S.Bays);
	return N + 2 * (Bays + 1) + 2 * Bays + Bays * static_cast<std::size_t>(S.PlanksPerBay) + 1;
}

inline ScaleSubject WallSubject(const std::string& Name, int Courses, int BricksPerCourse)
{
	const MortarRows Rows = LoadMortarRows();
	ScaleSubject S;
	S.Name = Name;
	geometry::WallSpec Spec;
	Spec.Courses = Courses;
	Spec.BricksPerCourse = BricksPerCourse;
	Spec.Bed = AddProfileOk(S.Model, Rows.Bed);
	Spec.Perpend = AddProfileOk(S.Model, Rows.Perpend);
	const geometry::Wall W = geometry::BuildRunningBondWall(S.Model, Spec);
	S.ExpectedBodies = WallBricks(Courses, BricksPerCourse) + 1;
	if (!W.Courses.empty())
	{
		S.BaseCourse = W.Courses[0];
	}
	return S;
}

inline ScaleSubject WarehouseSubject(const std::string& Name, geometry::WarehouseSpec Spec, bool bMustStand)
{
	const MortarRows Rows = LoadMortarRows();
	ScaleSubject S;
	S.Name = Name;
	S.bMustStand = bMustStand;
	Spec.Bed = AddProfileOk(S.Model, Rows.Bed);
	Spec.Perpend = AddProfileOk(S.Model, Rows.Perpend);
	Spec.Bearing = AddProfileOk(S.Model, Rows.Bearing);
	Spec.Plank = AddProfileOk(S.Model, FastenerProfile(Rows.Screw, 2, Spec.PlankWidthM * Spec.BeamWidthM));
	const geometry::Warehouse W = geometry::BuildWarehouse(S.Model, Spec);
	S.ExpectedBodies = WarehouseBodies(Spec);
	S.Posts = W.Posts;
	if (!W.Walls[0].Courses.empty())
	{
		S.BaseCourse = W.Walls[0].Courses[0];
	}
	return S;
}
}
