#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "StaticsTestSupport.h"
#include "SlowInDebug.h"

#include "loadpath/Catalogue.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <cstddef>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct BuiltWall
{
	Structure Model;
	geometry::WallSpec Spec;
	geometry::Wall Value;
};

std::size_t BrickCount(const geometry::Wall& W)
{
	std::size_t Count = 0;
	for (const std::vector<BodyId>& Course : W.Courses)
	{
		Count += Course.size();
	}
	return Count;
}

BuiltWall BuildWall(const StrengthProfile& Bed, const StrengthProfile& Perpend, int Courses = 10, int BricksPerCourse = 8)
{
	BuiltWall W;
	W.Spec.Bed = AddProfileOk(W.Model, Bed);
	W.Spec.Perpend = AddProfileOk(W.Model, Perpend);
	W.Spec.Courses = Courses;
	W.Spec.BricksPerCourse = BricksPerCourse;
	W.Value = geometry::BuildRunningBondWall(W.Model, W.Spec);
	MESSAGE(Courses << " x " << BricksPerCourse << " wall: courses " << W.Value.Courses.size() << ", bricks " << BrickCount(W.Value) << ", beds " << W.Value.Beds.size() << ", perpends " << W.Value.Perpends.size() << ", height " << W.Value.HeightM << " m");
	return W;
}

BuiltWall BuildDryWall(int Courses = 10, int BricksPerCourse = 8) { return BuildWall(DryJoint(0.7), DryJoint(0.7), Courses, BricksPerCourse); }

BuiltWall BuildBondedWall(int Courses = 10, int BricksPerCourse = 8)
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	return BuildWall(JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, Courses, BricksPerCourse);
}


double MonolithLambda(const BuiltWall& W) { return W.Spec.BrickM.Y / (W.Spec.Courses * W.Spec.BrickM.Z); }

Vec3 GravityAcrossThickness(double LambdaY) { return Vec3{0.0, LambdaY * GravityMPerS2, -GravityMPerS2}; }

bool RequireBuilt(const BuiltWall& W, std::size_t Bricks = 85u)
{
	INFO("BuildRunningBondWall produced " << BrickCount(W.Value) << " bricks");
	CHECK(BrickCount(W.Value) == Bricks);
	return BrickCount(W.Value) == Bricks;
}

double MaxBedUtilisationUnder(const BuiltWall& W, const AuthorityResult& Result, const std::vector<BodyId>& UpperCourse)
{
	double Max = 0.0;
	for (const InterfaceId& Id : W.Value.Beds)
	{
		if (!W.Model.IsLive(Id) || !HasBody(UpperCourse, W.Model.Interface(Id).BodyB))
		{
			continue;
		}
		for (const InterfaceReading& Reading : Result.Readings)
		{
			if (SameHandle(Reading.Id, Id))
			{
				Max = std::max(Max, Reading.Utilisation);
			}
		}
	}
	return Max;
}
}

TEST_CASE("Slow.Geometry.Anchor.DryRunningBondWallStandsUnderGravity")
{
	const BuiltWall W = BuildDryWall();
	if (!RequireBuilt(W))
	{
		return;
	}
	const StandingResult Elastic = SolveWith(W.Model, StandingOptions{});
	INFO("elastic " << Elastic.State << " after " << Elastic.ActiveSetIterations << " active-set iterations");
	CHECK(Elastic.State == StandingState::Stands);
	const AuthorityResult Combined = Judge(W.Model, AuthorityOptions{});
	INFO("combined " << std::string(VerdictName(Combined.Worst)));
	CHECK(Combined.Worst == ComponentVerdict::Stands);
}

TEST_CASE("Slow.Geometry.Anchor.BondedRunningBondWallStandsPastTOverHAndCollapsesAboveIt")
{
	const BuiltWall W = BuildBondedWall();
	if (!RequireBuilt(W))
	{
		return;
	}
	const double TOverH = MonolithLambda(W);
	const AuthorityResult AtRest = Judge(W.Model, AuthorityOptions{});
	INFO("at rest " << std::string(VerdictName(AtRest.Worst)));
	CHECK(AtRest.Worst == ComponentVerdict::Stands);
	const AuthorityResult Past = Judge(W.Model, AuthorityUnderGravity(GravityAcrossThickness(1.1 * TOverH)));
	INFO("at 1.1 t/H " << std::string(VerdictName(Past.Worst)));
	CHECK((Past.Worst == ComponentVerdict::Stands || Past.Worst == ComponentVerdict::Overstressed));
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Geometry.Anchor.RemovingABaseBrickArchesTheLoadOntoItsNeighbours")
{
	BuiltWall W = BuildBondedWall();
	if (!RequireBuilt(W))
	{
		return;
	}
	const AuthorityResult Before = Judge(W.Model, AuthorityOptions{});
	const double BeforeMax = MaxBedUtilisationUnder(W, Before, W.Value.Courses[1]);
	const BodyId Gap = W.Value.Courses[0][W.Value.Courses[0].size() / 2];
	REQUIRE(W.Model.RemoveBody(Gap));
	const AuthorityResult After = Judge(W.Model, AuthorityOptions{});
	const double AfterMax = MaxBedUtilisationUnder(W, After, W.Value.Courses[1]);
	MESSAGE("max bed utilisation under course 1: before " << BeforeMax << ", after " << AfterMax << ", verdict after " << std::string(VerdictName(After.Worst)));
	CHECK(After.Worst == ComponentVerdict::Stands);
	CHECK(AfterMax > BeforeMax);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Geometry.Anchor.SmallDryWallStandsUnderGravity")
{
	const BuiltWall W = BuildDryWall(4, 5);
	if (!RequireBuilt(W, 22u))
	{
		return;
	}
	const StandingResult Elastic = SolveWith(W.Model, StandingOptions{});
	MESSAGE("4 x 5 dry wall elastic " << Elastic.State << " after " << Elastic.ActiveSetIterations << " active-set iterations");
	const AuthorityResult Combined = Judge(W.Model, AuthorityOptions{});
	INFO(DescribeAuthority(Combined));
	CHECK(Combined.Worst == ComponentVerdict::Stands);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Geometry.Anchor.SmallBondedWallStandsPastThicknessOverHeight")
{
	const BuiltWall W = BuildBondedWall(4, 5);
	if (!RequireBuilt(W, 22u))
	{
		return;
	}
	const double TOverH = MonolithLambda(W);
	const AuthorityResult Past = Judge(W.Model, AuthorityUnderGravity(GravityAcrossThickness(1.1 * TOverH)));
	INFO("combined at 1.1 t/H: " << DescribeAuthority(Past));
	CHECK(Past.Worst == ComponentVerdict::Stands);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Geometry.Anchor.SmallBondedWallOverturnsAtTheBondedMonolithFactor")
{
	const BuiltWall W = BuildBondedWall(4, 5);
	if (!RequireBuilt(W, 22u))
	{
		return;
	}
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	const double TensileStrengthPa = JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile.TensileStrengthPa;
	double MassKg = 0.0;
	for (const std::vector<BodyId>& Course : W.Value.Courses)
	{
		for (const BodyId& Body : Course)
		{
			MassKg += W.Model.Body(Body).MassKg;
		}
	}
	const double WeightN = MassKg * GravityMPerS2;
	const double BaseBedAreaM2 = W.Spec.BricksPerCourse * W.Spec.BrickM.X * W.Spec.BrickM.Y;
	const double LambdaC = MonolithLambda(W) * (1.0 + TensileStrengthPa * BaseBedAreaM2 / WeightN);
	const AuthorityResult CombinedBelow = Judge(W.Model, AuthorityUnderGravity(GravityAcrossThickness(0.99 * LambdaC)));
	const AuthorityResult CombinedAbove = Judge(W.Model, AuthorityUnderGravity(GravityAcrossThickness(1.01 * LambdaC)));
	MESSAGE("4 x 5 bonded wall: ft " << TensileStrengthPa << " Pa, base bed " << BaseBedAreaM2 << " m2, W " << WeightN << " N, lambda_c " << LambdaC << "; authority at 0.99 " << std::string(VerdictName(CombinedBelow.Worst)) << ", at 1.01 " << std::string(VerdictName(CombinedAbove.Worst)));
	CHECK(CombinedBelow.Worst == ComponentVerdict::Stands);
	CHECK(CombinedAbove.Worst == ComponentVerdict::Overstressed);
}
