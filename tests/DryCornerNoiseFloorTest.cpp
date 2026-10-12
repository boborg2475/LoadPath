#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct NoiseFloorWall
{
	Structure Model;
	geometry::WallSpec Spec;
	geometry::Wall Value;
};

std::size_t Bricks(const geometry::Wall& W)
{
	std::size_t Count = 0;
	for (const std::vector<BodyId>& Course : W.Courses)
	{
		Count += Course.size();
	}
	return Count;
}

NoiseFloorWall BuildNoiseFloorWall(const StrengthProfile& Bed, const StrengthProfile& Perpend, int Courses, int BricksPerCourse)
{
	NoiseFloorWall W;
	W.Spec.Bed = AddProfileOk(W.Model, Bed);
	W.Spec.Perpend = AddProfileOk(W.Model, Perpend);
	W.Spec.Courses = Courses;
	W.Spec.BricksPerCourse = BricksPerCourse;
	W.Value = geometry::BuildRunningBondWall(W.Model, W.Spec);
	return W;
}

NoiseFloorWall BuildDry(int Courses, int BricksPerCourse) { return BuildNoiseFloorWall(DryJoint(0.7), DryJoint(0.7), Courses, BricksPerCourse); }

double BrickWeightN(const geometry::WallSpec& Spec) { return Spec.DensityKgPerM3 * Spec.BrickM.X * Spec.BrickM.Y * Spec.BrickM.Z * GravityMPerS2; }

std::vector<const InterfaceReading*> PerpendReadings(const NoiseFloorWall& W, const StandingResult& Result)
{
	std::vector<const InterfaceReading*> Found;
	for (const InterfaceId& Id : W.Value.Perpends)
	{
		const InterfaceReading* Reading = FindReading(Result, Id);
		REQUIRE(Reading != nullptr);
		Found.push_back(Reading);
	}
	return Found;
}

std::string DescribePerpends(const std::vector<const InterfaceReading*>& Readings)
{
	std::ostringstream Out;
	for (const InterfaceReading* R : Readings)
	{
		Out << "[N " << R->NormalForceN << ", |S| " << Length(R->ShearForceN) << ", u " << R->Utilisation << " (" << R->Axis << "), uc " << R->CompressionUtilisation << ", ut " << R->TensionUtilisation << ", us " << R->ShearUtilisation << ", open";
		for (const CornerReading& C : R->Corners)
		{
			Out << " " << C.bOpen;
		}
		Out << "] ";
	}
	return Out.str();
}
}

TEST_CASE("Statics.Solver.TheSmallestDryWallWithPerpendsSettles")
{
	const NoiseFloorWall W = BuildDry(1, 3);
	REQUIRE(W.Value.bComplete);
	REQUIRE(Bricks(W.Value) == 3u);
	REQUIRE(W.Value.Perpends.size() == 2u);
	const StandingResult Elastic = SolveWith(W.Model, StandingOptions{});
	INFO("1 x 3 dry wall elastic " << Elastic.State << " after " << Elastic.ActiveSetIterations << " active-set iterations; perpends " << DescribePerpends(PerpendReadings(W, Elastic)));
	MESSAGE("1 x 3 dry wall elastic " << Elastic.State << " after " << Elastic.ActiveSetIterations << " active-set iterations");
	CHECK(Elastic.State == StandingState::Stands);
}

TEST_CASE("Statics.Solver.ADryPerpendCarryingNothingReadsZeroAtRest")
{
	const NoiseFloorWall W = BuildDry(2, 1);
	REQUIRE(W.Value.bComplete);
	REQUIRE(Bricks(W.Value) == 3u);
	REQUIRE(W.Value.Perpends.size() == 1u);
	const double WeightN = BrickWeightN(W.Spec);
	const StandingResult Elastic = SolveWith(W.Model, StandingOptions{});
	const std::vector<const InterfaceReading*> Perpends = PerpendReadings(W, Elastic);
	const AuthorityResult Combined = Judge(W.Model, AuthorityOptions{});
	INFO("2 x 1 dry wall elastic " << Elastic.State << " after " << Elastic.ActiveSetIterations << " iterations; perpend " << DescribePerpends(Perpends) << "; combined " << DescribeAuthority(Combined));
	CHECK(Elastic.State == StandingState::Stands);
	CHECK(Combined.Worst == ComponentVerdict::Stands);
	for (const InterfaceReading* R : Perpends)
	{
		CHECK(std::isfinite(R->Utilisation));
		CHECK(std::isfinite(R->CompressionUtilisation));
		CHECK(std::isfinite(R->TensionUtilisation));
		CHECK(std::isfinite(R->ShearUtilisation));
		CHECK(R->Utilisation == 0.0);
		CHECK(R->CompressionUtilisation == 0.0);
		CHECK(R->TensionUtilisation == 0.0);
		CHECK(R->ShearUtilisation == 0.0);
		CHECK(std::abs(R->NormalForceN) <= 1e-9 * WeightN);
	}
}

TEST_CASE("Statics.Solver.DryRunningBondWallsWithPerpendsSettleAtRest")
{
	struct Size
	{
		int Courses, BricksPerCourse;
	};
	for (const Size S : {Size{1, 3}, Size{2, 3}, Size{3, 4}, Size{4, 5}})
	{
		CAPTURE(S.Courses);
		CAPTURE(S.BricksPerCourse);
		const NoiseFloorWall W = BuildDry(S.Courses, S.BricksPerCourse);
		REQUIRE(W.Value.bComplete);
		const auto Start = std::chrono::steady_clock::now();
		const StandingResult Elastic = SolveWith(W.Model, StandingOptions{});
		const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
		MESSAGE(S.Courses << " x " << S.BricksPerCourse << " dry wall elastic " << Elastic.State << " after " << Elastic.ActiveSetIterations << " active-set iterations, " << Seconds << " s");
		CHECK(Elastic.State == StandingState::Stands);
		CHECK(Elastic.ActiveSetIterations <= 2);
	}
}

TEST_CASE("Statics.Solver.ADryPerpendUnderAUniformLateralLoadCarriesNothing")
{
	const NoiseFloorWall W = BuildDry(1, 3);
	REQUIRE(W.Value.bComplete);
	const double WeightN = BrickWeightN(W.Spec);
	const StandingResult Elastic = SolveWith(W.Model, LateralLoad(0.2));
	const std::vector<const InterfaceReading*> Perpends = PerpendReadings(W, Elastic);
	INFO("1 x 3 dry wall at lambda 0.2: " << Elastic.State << " after " << Elastic.ActiveSetIterations << " iterations; perpends " << DescribePerpends(Perpends) << "; brick weight " << WeightN << " N");
	CHECK((Elastic.State == StandingState::Stands || Elastic.State == StandingState::Overstressed));
	for (const InterfaceReading* R : Perpends)
	{
		CHECK(std::abs(R->NormalForceN) <= 1e-9 * WeightN);
	}
}

TEST_CASE("Statics.Solver.ADryPerpendUnderAnEndPushStillCarries")
{
	const NoiseFloorWall W = BuildDry(1, 3);
	REQUIRE(W.Value.bComplete);
	const double WeightN = BrickWeightN(W.Spec);
	const BodyId End = W.Value.Courses[0].front();
	const std::vector<BodyLoad> Push = {BodyLoad{End, Vec3{0.2 * WeightN, 0.0, 0.0}, W.Model.Body(End).CentroidM}};
	const StandingResult Elastic = SolveWith(W.Model, StandingOptions{}, Push);
	const std::vector<const InterfaceReading*> Perpends = PerpendReadings(W, Elastic);
	INFO("1 x 3 dry wall, 0.2 W push on the end brick at x " << W.Model.Body(End).CentroidM.X << ": " << Elastic.State << " after " << Elastic.ActiveSetIterations << " iterations; perpends " << DescribePerpends(Perpends) << "; brick weight " << WeightN << " N");
	MESSAGE("1 x 3 end push: " << Elastic.State << " after " << Elastic.ActiveSetIterations << " iterations; perpends " << DescribePerpends(Perpends));
	CHECK((Elastic.State == StandingState::Stands || Elastic.State == StandingState::Overstressed));
	double MaxNormalN = 0.0;
	for (const InterfaceReading* R : Perpends)
	{
		MaxNormalN = std::max(MaxNormalN, std::abs(R->NormalForceN));
	}
	CHECK(MaxNormalN > 1e-3 * WeightN);
}

TEST_CASE("Statics.Solver.ABondedPerpendAtZeroNormalIsNotOpened")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	const NoiseFloorWall W = BuildNoiseFloorWall(JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, 1, 3);
	REQUIRE(W.Value.bComplete);
	const StandingResult Elastic = SolveWith(W.Model, LateralLoad(0.05));
	const std::vector<const InterfaceReading*> Perpends = PerpendReadings(W, Elastic);
	INFO("bonded 1 x 3 wall at lambda 0.05: " << Elastic.State << " after " << Elastic.ActiveSetIterations << " iterations; perpends " << DescribePerpends(Perpends));
	MESSAGE("bonded 1 x 3 wall at lambda 0.05: " << Elastic.State << "; perpends " << DescribePerpends(Perpends));
	CHECK(Elastic.State == StandingState::Stands);
	bool bAnyCarries = false;
	for (const InterfaceReading* R : Perpends)
	{
		for (const CornerReading& C : R->Corners)
		{
			CHECK_FALSE(C.bOpen);
		}
		bAnyCarries = bAnyCarries || Length(R->ShearForceN) > 0.0;
	}
	CHECK(bAnyCarries);
}

TEST_CASE("Statics.Solver.ADryWallStandsUnderAnEndPushNearTheNoiseFloor")
{
	const NoiseFloorWall W = BuildDry(1, 3);
	REQUIRE(W.Value.bComplete);
	const double WeightN = BrickWeightN(W.Spec);
	const BodyId End = W.Value.Courses[0].front();
	int NotStanding = 0;
	for (int K = -260; K <= -140; ++K)
	{
		const double PushN = WeightN * std::pow(10.0, K / 20.0);
		const std::vector<BodyLoad> Push = {BodyLoad{End, Vec3{PushN, 0.0, 0.0}, W.Model.Body(End).CentroidM}};
		const StandingResult Elastic = SolveWith(W.Model, StandingOptions{}, Push);
		if (Elastic.State != StandingState::Stands)
		{
			++NotStanding;
			MESSAGE("push 10^" << K / 20.0 << " W: " << Elastic.State << " after " << Elastic.ActiveSetIterations << " iterations; perpends " << DescribePerpends(PerpendReadings(W, Elastic)));
		}
	}
	CHECK(NotStanding == 0);
}

TEST_CASE("Statics.Solver.ABondedPerpendAtRestStaysClosed")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	const NoiseFloorWall W = BuildNoiseFloorWall(JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, 1, 3);
	REQUIRE(W.Value.bComplete);
	const StandingResult Elastic = SolveWith(W.Model, StandingOptions{});
	const std::vector<const InterfaceReading*> Perpends = PerpendReadings(W, Elastic);
	INFO("bonded 1 x 3 wall at rest: " << Elastic.State << " after " << Elastic.ActiveSetIterations << " iterations; perpends " << DescribePerpends(Perpends));
	CHECK(Elastic.State == StandingState::Stands);
	for (const InterfaceReading* R : Perpends)
	{
		for (const CornerReading& C : R->Corners)
		{
			CHECK_FALSE(C.bOpen);
		}
	}
}

TEST_CASE("Statics.Solver.ACohesiveTensionlessPerpendAtRestStaysClosed")
{
	StrengthProfile Cohesive = DryJoint(0.7);
	Cohesive.ShearCohesionPa = 1.0e5;
	const NoiseFloorWall W = BuildNoiseFloorWall(Cohesive, Cohesive, 1, 3);
	REQUIRE(W.Value.bComplete);
	const StandingResult Elastic = SolveWith(W.Model, StandingOptions{});
	const std::vector<const InterfaceReading*> Perpends = PerpendReadings(W, Elastic);
	INFO("cohesive tensionless 1 x 3 wall at rest: " << Elastic.State << " after " << Elastic.ActiveSetIterations << " iterations; perpends " << DescribePerpends(Perpends));
	CHECK(Elastic.State == StandingState::Stands);
	for (const InterfaceReading* R : Perpends)
	{
		CHECK(std::isfinite(R->TensionUtilisation));
		CHECK(R->TensionUtilisation == 0.0);
		for (const CornerReading& C : R->Corners)
		{
			CHECK_FALSE(C.bOpen);
		}
	}
}

TEST_CASE("Statics.Solver.LimeMortarWallsWithNoTensionStandAtRest")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	const StrengthProfile Lime = JointOrFail(Loaded.Value, "lime_mortar_handmade_brick_upc_mean").Profile;
	struct Size
	{
		int Courses, BricksPerCourse;
	};
	for (const Size S : {Size{2, 1}, Size{1, 3}})
	{
		CAPTURE(S.Courses);
		CAPTURE(S.BricksPerCourse);
		const NoiseFloorWall W = BuildNoiseFloorWall(Lime, Lime, S.Courses, S.BricksPerCourse);
		REQUIRE(W.Value.bComplete);
		const AuthorityResult Combined = Judge(W.Model, AuthorityOptions{});
		INFO("lime mortar wall at rest: combined " << DescribeAuthority(Combined));
		CHECK(Combined.Worst == ComponentVerdict::Stands);
		for (const InterfaceReading& R : Combined.Readings)
		{
			CHECK(std::isfinite(R.TensionUtilisation));
		}
	}
}

TEST_CASE("Statics.Solver.ADryPerpendNormalScalesWithTheEndPush")
{
	const NoiseFloorWall W = BuildDry(1, 3);
	REQUIRE(W.Value.bComplete);
	const double WeightN = BrickWeightN(W.Spec);
	const BodyId End = W.Value.Courses[0].front();
	const auto SolvePush = [&](double PushN) {
		const std::vector<BodyLoad> Push = {BodyLoad{End, Vec3{PushN, 0.0, 0.0}, W.Model.Body(End).CentroidM}};
		return SolveWith(W.Model, StandingOptions{}, Push);
	};
	const double LargeN = 0.2 * WeightN;
	const double SmallN = 1.0e-7 * WeightN;
	const StandingResult Large = SolvePush(LargeN);
	const StandingResult Small = SolvePush(SmallN);
	const std::vector<const InterfaceReading*> LargePerpends = PerpendReadings(W, Large);
	const std::vector<const InterfaceReading*> SmallPerpends = PerpendReadings(W, Small);
	INFO("0.2 W: " << Large.State << ", perpends " << DescribePerpends(LargePerpends));
	INFO("1e-7 W: " << Small.State << ", perpends " << DescribePerpends(SmallPerpends));
	for (const std::vector<const InterfaceReading*>* Set : {&LargePerpends, &SmallPerpends})
	{
		for (const InterfaceReading* R : *Set)
		{
			for (const CornerReading& C : R->Corners)
			{
				CHECK_FALSE(C.bOpen);
			}
		}
	}
	const double Ratio = SmallN / LargeN;
	for (std::size_t I = 0; I < LargePerpends.size(); ++I)
	{
		CAPTURE(I);
		CHECK(SmallPerpends[I]->NormalForceN == doctest::Approx(Ratio * LargePerpends[I]->NormalForceN).epsilon(1e-6));
	}
}
