#include "StaticsTestSupport.h"

#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double Exact = 1e-12;

struct BuiltWall
{
	Structure Model;
	geometry::WallSpec Spec;
	geometry::Wall Value;
};

BuiltWall Build(int Courses, int BricksPerCourse, bool bRunningBond)
{
	BuiltWall W;
	W.Spec.Courses = Courses;
	W.Spec.BricksPerCourse = BricksPerCourse;
	W.Spec.bRunningBond = bRunningBond;
	W.Spec.Bed = AddProfileOk(W.Model, DryJoint(0.7));
	W.Spec.Perpend = AddProfileOk(W.Model, DryJoint(0.6));
	W.Value = geometry::BuildRunningBondWall(W.Model, W.Spec);
	return W;
}

std::size_t ClosedFormBricks(int N, int M, bool bRunningBond) { return static_cast<std::size_t>(N * M + (bRunningBond ? N / 2 : 0)); }
std::size_t ClosedFormPerpends(int N, int M, bool bRunningBond) { return static_cast<std::size_t>(N * (M - 1) + (bRunningBond ? N / 2 : 0)); }
std::size_t ClosedFormBeds(int N, int M, bool bRunningBond) { return static_cast<std::size_t>(bRunningBond ? M + 2 * M * (N - 1) : N * M); }

std::size_t BrickCount(const geometry::Wall& W)
{
	std::size_t Count = 0;
	for (const std::vector<BodyId>& Course : W.Courses)
	{
		Count += Course.size();
	}
	return Count;
}

double FullBrickMassKg(const geometry::WallSpec& Spec) { return Spec.DensityKgPerM3 * Spec.BrickM.X * Spec.BrickM.Y * Spec.BrickM.Z; }

double LengthOfBrickM(const Structure& Model, const geometry::WallSpec& Spec, BodyId Body) { return Spec.BrickM.X * Model.Body(Body).MassKg / FullBrickMassKg(Spec); }

void ReportCounts(const BuiltWall& W)
{
	const int N = W.Spec.Courses;
	const int M = W.Spec.BricksPerCourse;
	const bool R = W.Spec.bRunningBond;
	MESSAGE("wall " << N << " x " << M << std::string(R ? " running bond" : " stack bond") << ": courses " << W.Value.Courses.size() << ", bricks " << BrickCount(W.Value) << " (closed form " << ClosedFormBricks(N, M, R)
		<< "), beds " << W.Value.Beds.size() << " (closed form " << ClosedFormBeds(N, M, R) << "), perpends " << W.Value.Perpends.size() << " (closed form " << ClosedFormPerpends(N, M, R) << "), live bodies " << W.Model.LiveBodyCount());
}
}

TEST_CASE("Geometry.Wall.RunningBondCountsMatchTheClosedForms")
{
	const BuiltWall W = Build(4, 5, true);
	ReportCounts(W);
	REQUIRE(W.Value.Courses.size() == 4);
	if (!(W.Value.Courses.size() == 4))
	{
		return;
	}
	for (std::size_t C = 0; C < W.Value.Courses.size(); ++C)
	{
		INFO("course " << C);
		CHECK(W.Value.Courses[C].size() == (C % 2 == 0 ? 5u : 6u));
	}
	CHECK(BrickCount(W.Value) == ClosedFormBricks(4, 5, true));
	CHECK(BrickCount(W.Value) == 22u);
	CHECK(W.Model.LiveBodyCount() == 23u);
	CHECK(W.Value.Beds.size() == ClosedFormBeds(4, 5, true));
	CHECK(W.Value.Beds.size() == 35u);
	CHECK(W.Value.Perpends.size() == ClosedFormPerpends(4, 5, true));
	CHECK(W.Value.Perpends.size() == 18u);
	CHECK(W.Model.LiveInterfaceCount() == W.Value.Beds.size() + W.Value.Perpends.size());
}

TEST_CASE("Geometry.Wall.EveryBrickWeighsItsDensityTimesItsVolume")
{
	const BuiltWall W = Build(4, 5, true);
	ReportCounts(W);
	REQUIRE(W.Value.Courses.size() == 4);
	if (!(W.Value.Courses.size() == 4))
	{
		return;
	}
	const double FullKg = FullBrickMassKg(W.Spec);
	for (std::size_t C = 0; C < W.Value.Courses.size(); ++C)
	{
		const std::vector<BodyId>& Course = W.Value.Courses[C];
		for (std::size_t K = 0; K < Course.size(); ++K)
		{
			const bool bHalf = C % 2 == 1 && (K == 0 || K + 1 == Course.size());
			INFO("course " << C << " brick " << K);
			CHECK_FALSE(W.Model.Body(Course[K]).bGrounded);
			CHECK(W.Model.Body(Course[K]).MassKg == doctest::Approx(bHalf ? 0.5 * FullKg : FullKg).epsilon(Exact));
		}
	}
}

TEST_CASE("Geometry.Wall.CoursesRunLeftToRightAndTheWallSpansItsComputedSize")
{
	const BuiltWall W = Build(4, 5, true);
	ReportCounts(W);
	REQUIRE(W.Value.Courses.size() == 4);
	if (!(W.Value.Courses.size() == 4))
	{
		return;
	}
	CHECK(W.Value.LengthM == doctest::Approx(5 * W.Spec.BrickM.X).epsilon(Exact));
	CHECK(W.Value.HeightM == doctest::Approx(4 * W.Spec.BrickM.Z).epsilon(Exact));
	for (std::size_t C = 0; C < W.Value.Courses.size(); ++C)
	{
		const std::vector<BodyId>& Course = W.Value.Courses[C];
		double SpanM = 0;
		for (std::size_t K = 0; K < Course.size(); ++K)
		{
			INFO("course " << C << " brick " << K);
			SpanM += LengthOfBrickM(W.Model, W.Spec, Course[K]);
			CHECK(W.Model.Body(Course[K]).CentroidM.Z == doctest::Approx(W.Spec.GroundZM + (static_cast<double>(C) + 0.5) * W.Spec.BrickM.Z).epsilon(Exact));
			if (K > 0)
			{
				CHECK(W.Model.Body(Course[K - 1]).CentroidM.X < W.Model.Body(Course[K]).CentroidM.X);
			}
		}
		INFO("course " << C);
		CHECK(SpanM == doctest::Approx(W.Value.LengthM).epsilon(Exact));
	}
}

TEST_CASE("Geometry.Wall.JointsAreValidAndPointUpOrRight")
{
	const BuiltWall W = Build(4, 5, true);
	ReportCounts(W);
	REQUIRE_FALSE(W.Value.Beds.empty());
	if (W.Value.Beds.empty())
	{
		return;
	}
	REQUIRE_FALSE(W.Value.Perpends.empty());
	if (W.Value.Perpends.empty())
	{
		return;
	}
	Structure Scratch = W.Model;
	for (const InterfaceId& Id : W.Value.Beds)
	{
		const InterfaceRecord& R = W.Model.Interface(Id);
		CHECK(W.Model.IsLive(Id));
		CHECK(SameHandle(R.Profile, W.Spec.Bed));
		CHECK(Length(Minus(R.NormalAToB, Vec3{0, 0, 1})) <= Exact);
		CHECK(W.Model.Body(R.BodyA).CentroidM.Z < W.Model.Body(R.BodyB).CentroidM.Z);
		AddInterfaceOk(Scratch, InterfaceDesc{R.BodyA, R.BodyB, R.NormalAToB, R.CentreM, R.AxisU, R.AxisV, R.HalfExtentUM, R.HalfExtentVM, R.Profile});
	}
	for (const InterfaceId& Id : W.Value.Perpends)
	{
		const InterfaceRecord& R = W.Model.Interface(Id);
		CHECK(W.Model.IsLive(Id));
		CHECK(SameHandle(R.Profile, W.Spec.Perpend));
		CHECK(Length(Minus(R.NormalAToB, Vec3{1, 0, 0})) <= Exact);
		CHECK(W.Model.Body(R.BodyA).CentroidM.X < W.Model.Body(R.BodyB).CentroidM.X);
		CHECK(R.AreaM2 == doctest::Approx(W.Spec.BrickM.Y * W.Spec.BrickM.Z).epsilon(Exact));
		AddInterfaceOk(Scratch, InterfaceDesc{R.BodyA, R.BodyB, R.NormalAToB, R.CentreM, R.AxisU, R.AxisV, R.HalfExtentUM, R.HalfExtentVM, R.Profile});
	}
}

TEST_CASE("Geometry.Wall.BedsJoinEveryBrickToEveryBrickBelowItThatOverlapsInPlan")
{
	const BuiltWall W = Build(4, 5, true);
	ReportCounts(W);
	REQUIRE(W.Value.Courses.size() == 4);
	if (!(W.Value.Courses.size() == 4))
	{
		return;
	}
	std::set<std::pair<std::uint32_t, std::uint32_t>> Expected;
	for (std::size_t C = 1; C < W.Value.Courses.size(); ++C)
	{
		for (const BodyId& Lower : W.Value.Courses[C - 1])
		{
			for (const BodyId& Upper : W.Value.Courses[C])
			{
				double Mid = 0;
				const double Span = OverlapM(W.Model.Body(Lower).CentroidM.X, 0.5 * LengthOfBrickM(W.Model, W.Spec, Lower), W.Model.Body(Upper).CentroidM.X, 0.5 * LengthOfBrickM(W.Model, W.Spec, Upper), Mid);
				if (Span > 1e-9)
				{
					Expected.insert({Lower.Index, Upper.Index});
				}
			}
		}
	}
	for (const BodyId& Base : W.Value.Courses[0])
	{
		Expected.insert({W.Value.Ground.Index, Base.Index});
	}
	std::set<std::pair<std::uint32_t, std::uint32_t>> Got;
	for (const InterfaceId& Id : W.Value.Beds)
	{
		const InterfaceRecord& R = W.Model.Interface(Id);
		Got.insert({R.BodyA.Index, R.BodyB.Index});
	}
	MESSAGE("oracle beds " << Expected.size() << ", built beds " << Got.size());
	CHECK(Expected.size() == ClosedFormBeds(4, 5, true));
	CHECK(Got == Expected);
	CHECK(Got.size() == W.Value.Beds.size());
}

TEST_CASE("Geometry.Wall.TheBaseCourseSitsOnAGroundedBodyThroughBeds")
{
	const BuiltWall W = Build(4, 5, true);
	ReportCounts(W);
	REQUIRE(W.Model.IsLive(W.Value.Ground));
	if (!(W.Model.IsLive(W.Value.Ground)))
	{
		return;
	}
	CHECK(W.Model.Body(W.Value.Ground).bGrounded);
	REQUIRE(W.Value.Courses.size() == 4);
	if (!(W.Value.Courses.size() == 4))
	{
		return;
	}
	std::size_t OnGround = 0;
	for (const InterfaceId& Id : W.Value.Beds)
	{
		const InterfaceRecord& R = W.Model.Interface(Id);
		if (SameHandle(R.BodyA, W.Value.Ground))
		{
			++OnGround;
			CHECK(R.CentreM.Z == doctest::Approx(W.Spec.GroundZM).epsilon(Exact));
			CHECK(R.AreaM2 == doctest::Approx(W.Spec.BrickM.X * W.Spec.BrickM.Y).epsilon(Exact));
		}
	}
	CHECK(OnGround == W.Value.Courses[0].size());
}

TEST_CASE("Geometry.Wall.StackBondHasNTimesMBricksWithAlignedPerpends")
{
	const BuiltWall W = Build(4, 5, false);
	ReportCounts(W);
	REQUIRE(W.Value.Courses.size() == 4);
	if (!(W.Value.Courses.size() == 4))
	{
		return;
	}
	CHECK(BrickCount(W.Value) == 20u);
	CHECK(W.Value.Beds.size() == ClosedFormBeds(4, 5, false));
	CHECK(W.Value.Perpends.size() == ClosedFormPerpends(4, 5, false));
	std::set<double> PerpendX;
	for (const InterfaceId& Id : W.Value.Perpends)
	{
		PerpendX.insert(std::round(W.Model.Interface(Id).CentreM.X * 1e9) / 1e9);
	}
	CHECK(PerpendX.size() == 4u);
	CHECK(W.Model.LiveInterfaceCount() == W.Value.Beds.size() + W.Value.Perpends.size());
}

TEST_CASE("Geometry.Wall.OneCourseHasOnlyGroundBeds")
{
	const BuiltWall W = Build(1, 5, true);
	ReportCounts(W);
	REQUIRE(W.Value.Courses.size() == 1);
	if (!(W.Value.Courses.size() == 1))
	{
		return;
	}
	CHECK(W.Value.Courses[0].size() == 5u);
	CHECK(W.Value.Beds.size() == 5u);
	for (const InterfaceId& Id : W.Value.Beds)
	{
		CHECK(SameHandle(W.Model.Interface(Id).BodyA, W.Value.Ground));
	}
	CHECK(W.Value.Perpends.size() == 4u);
}

TEST_CASE("Geometry.Wall.AWallWithInvalidProfilesIsNotComplete")
{
	Structure Model;
	const geometry::WallSpec Spec;
	const geometry::Wall Value = geometry::BuildRunningBondWall(Model, Spec);
	INFO("default spec (Invalid profiles): beds " << Value.Beds.size() << ", perpends " << Value.Perpends.size() << ", complete " << Value.bComplete);
	CHECK_FALSE(Value.bComplete);
	const BuiltWall Valid = Build(4, 5, true);
	INFO("valid profiles: complete " << Valid.Value.bComplete);
	CHECK(Valid.Value.bComplete);
}

TEST_CASE("Geometry.Wall.AnOriginShiftsEveryCentroidByIt")
{
	const BuiltWall Plain = Build(4, 5, true);
	BuiltWall Moved;
	Moved.Spec = Plain.Spec;
	Moved.Spec.Bed = AddProfileOk(Moved.Model, DryJoint(0.7));
	Moved.Spec.Perpend = AddProfileOk(Moved.Model, DryJoint(0.6));
	Moved.Spec.OriginM = Vec3{1.25, -0.5, 0.3};
	Moved.Value = geometry::BuildRunningBondWall(Moved.Model, Moved.Spec);
	ReportCounts(Moved);
	REQUIRE(BrickCount(Moved.Value) == BrickCount(Plain.Value));
	if (BrickCount(Moved.Value) != BrickCount(Plain.Value))
	{
		return;
	}
	for (std::size_t C = 0; C < Plain.Value.Courses.size(); ++C)
	{
		for (std::size_t K = 0; K < Plain.Value.Courses[C].size(); ++K)
		{
			INFO("course " << C << " brick " << K);
			const Vec3 Want = Plus(Plain.Model.Body(Plain.Value.Courses[C][K]).CentroidM, Moved.Spec.OriginM);
			CHECK(Length(Minus(Moved.Model.Body(Moved.Value.Courses[C][K]).CentroidM, Want)) <= Exact);
		}
	}
}

TEST_CASE("Geometry.Wall.AWallAlongYHasItsLengthAlongYAndItsThicknessAlongX")
{
	const BuiltWall Plain = Build(4, 5, true);
	BuiltWall Turned;
	Turned.Spec = Plain.Spec;
	Turned.Spec.Bed = AddProfileOk(Turned.Model, DryJoint(0.7));
	Turned.Spec.Perpend = AddProfileOk(Turned.Model, DryJoint(0.6));
	Turned.Spec.bAlongY = true;
	Turned.Spec.OriginM = Vec3{2.0, 0.0, 0.0};
	Turned.Value = geometry::BuildRunningBondWall(Turned.Model, Turned.Spec);
	ReportCounts(Turned);
	REQUIRE(BrickCount(Turned.Value) == BrickCount(Plain.Value));
	if (BrickCount(Turned.Value) != BrickCount(Plain.Value))
	{
		return;
	}
	CHECK(Turned.Value.Beds.size() == Plain.Value.Beds.size());
	CHECK(Turned.Value.Perpends.size() == Plain.Value.Perpends.size());
	for (std::size_t C = 0; C < Plain.Value.Courses.size(); ++C)
	{
		for (std::size_t K = 0; K < Plain.Value.Courses[C].size(); ++K)
		{
			INFO("course " << C << " brick " << K);
			const Vec3 P = Plain.Model.Body(Plain.Value.Courses[C][K]).CentroidM;
			const Vec3 Want{2.0 + P.Y, P.X, P.Z};
			CHECK(Length(Minus(Turned.Model.Body(Turned.Value.Courses[C][K]).CentroidM, Want)) <= Exact);
		}
	}
	for (const InterfaceId& Id : Turned.Value.Perpends)
	{
		const InterfaceRecord& R = Turned.Model.Interface(Id);
		CHECK(Length(Minus(R.NormalAToB, Vec3{0, 1, 0})) <= Exact);
		CHECK(R.AreaM2 == doctest::Approx(Turned.Spec.BrickM.Y * Turned.Spec.BrickM.Z).epsilon(Exact));
		CHECK(R.CentreM.X == doctest::Approx(2.0).epsilon(Exact));
	}
	for (const InterfaceId& Id : Turned.Value.Beds)
	{
		const InterfaceRecord& R = Turned.Model.Interface(Id);
		CHECK(Length(Minus(R.NormalAToB, Vec3{0, 0, 1})) <= Exact);
		CHECK(std::abs(R.HalfExtentUM * R.HalfExtentVM * 4.0 - R.AreaM2) <= Exact);
	}
}

TEST_CASE("Geometry.Wall.AWallGivenAGroundStandsOnItAndAddsNoGround")
{
	Structure Model;
	const BodyId Shared = AddGroundPlane(Model);
	geometry::WallSpec Spec;
	Spec.Courses = 4;
	Spec.BricksPerCourse = 5;
	Spec.Bed = AddProfileOk(Model, DryJoint(0.7));
	Spec.Perpend = AddProfileOk(Model, DryJoint(0.6));
	Spec.Ground = Shared;
	const geometry::Wall Value = geometry::BuildRunningBondWall(Model, Spec);
	INFO("live bodies " << Model.LiveBodyCount() << ", bricks " << BrickCount(Value) << ", complete " << Value.bComplete);
	CHECK(Value.bComplete);
	CHECK(SameHandle(Value.Ground, Shared));
	CHECK(Model.LiveBodyCount() == ClosedFormBricks(4, 5, true) + 1);
	std::size_t OnShared = 0;
	for (const InterfaceId& Id : Value.Beds)
	{
		OnShared += SameHandle(Model.Interface(Id).BodyA, Shared) ? 1u : 0u;
	}
	CHECK(OnShared == 5u);
}
