#include "SlowInDebug.h"
#include "StaticsTestSupport.h"

#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/BoxContacts.h"

#include "doctest.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct BlockWall
{
	Structure Model;
	BodyId Ground;
	std::vector<BodyId> Blocks;
	std::vector<geometry::Box> Shapes;
	BodyId Beam;
	Vec3 BeamCentreM;
	int Contacts = 0;
};

void AddBlock(BlockWall& Wall, double X0, double X1, double Z0, double Z1, double DepthM, double DensityKgPerM3)
{
	const geometry::Box Shape{Vec3{0.5 * (X0 + X1), 0.0, 0.5 * (Z0 + Z1)}, Vec3{0.5 * (X1 - X0), 0.5 * DepthM, 0.5 * (Z1 - Z0)}};
	Wall.Blocks.push_back(AddBox(Wall.Model, BoxOfDensity(Shape.CentreM, Shape.HalfExtentsM, DensityKgPerM3)));
	Wall.Shapes.push_back(Shape);
}

void AddCourse(BlockWall& Wall, std::vector<double> Edges, double Z0, double Z1, double DepthM, double DensityKgPerM3, double SkipFromM = 1.0, double SkipToM = -1.0)
{
	std::sort(Edges.begin(), Edges.end());
	Edges.erase(std::unique(Edges.begin(), Edges.end(), [](double A, double B) { return std::abs(A - B) < 1.0e-9; }), Edges.end());
	for (std::size_t Index = 0; Index + 1 < Edges.size(); ++Index)
	{
		const double Mid = 0.5 * (Edges[Index] + Edges[Index + 1]);
		if (Mid > SkipFromM && Mid < SkipToM)
		{
			continue;
		}
		AddBlock(Wall, Edges[Index], Edges[Index + 1], Z0, Z1, DepthM, DensityKgPerM3);
	}
}

std::vector<double> CourseEdges(double LengthM, double BlockM, bool bHalfAtEnds)
{
	std::vector<double> Edges{0.0};
	double X = bHalfAtEnds ? 0.5 * BlockM : BlockM;
	while (X < LengthM - 1.0e-9)
	{
		Edges.push_back(X);
		X += BlockM;
	}
	Edges.push_back(LengthM);
	return Edges;
}

void StartWall(BlockWall& Wall, double LengthM, double DepthM)
{
	Wall.Ground = AddBox(Wall.Model, Box{Vec3{0.5 * LengthM, 0.0, -0.5}, Vec3{LengthM, DepthM, 0.5}, 0.0}, true);
}

void ConnectAll(BlockWall& Wall, ProfileId Profile, double LengthM, double DepthM)
{
	std::vector<BodyId> Bodies{Wall.Ground};
	std::vector<geometry::Box> Shapes{geometry::Box{Vec3{0.5 * LengthM, 0.0, -0.5}, Vec3{LengthM, DepthM, 0.5}}};
	Bodies.insert(Bodies.end(), Wall.Blocks.begin(), Wall.Blocks.end());
	Shapes.insert(Shapes.end(), Wall.Shapes.begin(), Wall.Shapes.end());
	for (std::size_t A = 0; A < Bodies.size(); ++A)
	{
		for (std::size_t B = A + 1; B < Bodies.size(); ++B)
		{
			const std::optional<InterfaceDesc> Contact = geometry::FaceContact(Bodies[A], Shapes[A], Bodies[B], Shapes[B], Profile, 1.0e-9);
			if (Contact)
			{
				AddInterfaceOk(Wall.Model, *Contact);
				++Wall.Contacts;
			}
		}
	}
}

constexpr double FtlBlockM = 4.0;
constexpr double FtlCourseM = 1.75;
constexpr double FtlDepthM = 4.0;
constexpr double FtlLengthM = 20.0;
constexpr int FtlCourses = 6;

BlockWall MakeFerrisTinLoiWall()
{
	BlockWall Wall;
	StartWall(Wall, FtlLengthM, FtlDepthM);
	for (int Course = 0; Course < FtlCourses; ++Course)
	{
		AddCourse(Wall, CourseEdges(FtlLengthM, FtlBlockM, Course % 2 == 0), FtlCourseM * Course, FtlCourseM * (Course + 1), FtlDepthM, 1000.0);
	}
	ConnectAll(Wall, AddProfileOk(Wall.Model, DryJoint(0.65)), FtlLengthM, FtlDepthM);
	return Wall;
}

constexpr double ForLengthM = 3.12;
constexpr double ForDepthM = 0.25;
constexpr double ForBlockM = 0.26;
constexpr double ForCourseM = 0.0756;
constexpr int ForCourses = 34;
constexpr int ForSpandrelCourses = 5;
constexpr int ForOpeningCourses = 16;
constexpr int ForLintelCourses = 3;
constexpr double ForOpeningFromM = 1.04;
constexpr double ForOpeningToM = 2.08;
constexpr double ForSoldierM = 0.065;
constexpr double ForBeamM = 0.194;
constexpr double ForMasonryKgPerM3 = 18000.0 / GravityMPerS2;
constexpr double ForBeamKgPerM3 = 25000.0 / GravityMPerS2;

BlockWall MakeForaboschiWall(double CompressiveStrengthPa)
{
	BlockWall Wall;
	StartWall(Wall, ForLengthM, ForDepthM);
	for (int Course = 0; Course < ForCourses; ++Course)
	{
		std::vector<double> Edges = CourseEdges(ForLengthM, ForBlockM, Course % 2 == 1);
		const double Z0 = ForCourseM * Course;
		const double Z1 = Z0 + ForCourseM;
		const bool bOpening = Course >= ForSpandrelCourses && Course < ForSpandrelCourses + ForOpeningCourses;
		const bool bLintel = Course >= ForSpandrelCourses + ForOpeningCourses && Course < ForSpandrelCourses + ForOpeningCourses + ForLintelCourses;
		if (bOpening || bLintel)
		{
			Edges.push_back(ForOpeningFromM);
			Edges.push_back(ForOpeningToM);
			AddCourse(Wall, Edges, Z0, Z1, ForDepthM, ForMasonryKgPerM3, ForOpeningFromM, ForOpeningToM);
		}
		else
		{
			AddCourse(Wall, Edges, Z0, Z1, ForDepthM, ForMasonryKgPerM3);
		}
		if (Course == ForSpandrelCourses + ForOpeningCourses)
		{
			for (int Soldier = 0; Soldier < 16; ++Soldier)
			{
				const double X = ForOpeningFromM + ForSoldierM * Soldier;
				AddBlock(Wall, X, X + ForSoldierM, Z0, Z0 + ForLintelCourses * ForCourseM, ForDepthM, ForMasonryKgPerM3);
			}
		}
	}
	const double TopM = ForCourseM * ForCourses;
	AddBlock(Wall, 0.0, ForLengthM, TopM, TopM + ForBeamM, ForDepthM, ForBeamKgPerM3);
	Wall.Beam = Wall.Blocks.back();
	Wall.BeamCentreM = Wall.Shapes.back().CentreM;
	StrengthProfile Profile = DryJoint(0.6);
	Profile.CompressiveStrengthPa = CompressiveStrengthPa;
	ConnectAll(Wall, AddProfileOk(Wall.Model, Profile), ForLengthM, ForDepthM);
	return Wall;
}

constexpr double MinhoLengthM = 1.0;
constexpr double MinhoDepthM = 0.2;
constexpr double MinhoBlockM = 0.2;
constexpr double MinhoCourseM = 0.1;
constexpr int MinhoCourses = 10;
constexpr double MinhoBeamM = 0.1;
constexpr double MinhoKgPerM3 = 25000.0 / GravityMPerS2;
constexpr double MinhoFriction = 0.66;
constexpr double MinhoVerticalN = 30000.0;

BlockWall MakeMinhoWall(double BeamKgPerM3)
{
	BlockWall Wall;
	StartWall(Wall, MinhoLengthM, MinhoDepthM);
	for (int Course = 0; Course < MinhoCourses; ++Course)
	{
		AddCourse(Wall, CourseEdges(MinhoLengthM, MinhoBlockM, Course % 2 == 1), MinhoCourseM * Course, MinhoCourseM * (Course + 1), MinhoDepthM, MinhoKgPerM3);
	}
	const double TopM = MinhoCourseM * MinhoCourses;
	AddBlock(Wall, 0.0, MinhoLengthM, TopM, TopM + MinhoBeamM, MinhoDepthM, BeamKgPerM3);
	Wall.Beam = Wall.Blocks.back();
	Wall.BeamCentreM = Wall.Shapes.back().CentreM;
	ConnectAll(Wall, AddProfileOk(Wall.Model, DryJoint(MinhoFriction)), MinhoLengthM, MinhoDepthM);
	return Wall;
}

std::vector<BodyLoad> MinhoLoads(const BlockWall& Wall, double HorizontalN, double PushHeightM)
{
	return {BodyLoad{Wall.Beam, Vec3{0.0, 0.0, -MinhoVerticalN}, Wall.BeamCentreM}, BodyLoad{Wall.Beam, Vec3{HorizontalN, 0.0, 0.0}, Vec3{Wall.BeamCentreM.X, 0.0, PushHeightM}}};
}

std::vector<BodyLoad> BeamLoads(const BlockWall& Wall, double VerticalN, double HorizontalN)
{
	return {BodyLoad{Wall.Beam, Vec3{HorizontalN, 0.0, -VerticalN}, Wall.BeamCentreM}};
}

double SecondsSince(std::chrono::steady_clock::time_point Start)
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Parked.Statics.Anchor.FerrisTinLoiWallStandsBelowTheLoadFactorAndCollapsesAboveIt")
{
	const auto Start = std::chrono::steady_clock::now();
	const BlockWall Wall = MakeFerrisTinLoiWall();
	CHECK(Wall.Blocks.size() == 33u);
	CHECK(Wall.Contacts == 83);
	const auto Judge = [&](double Lambda) {
		AuthorityOptions Options;
		Options.Standing = LateralLoad(Lambda);
		return MakeElasticAuthority(MakeRbsmStandingAuthority())->Judge(Wall.Model, {}, Options, nullptr);
	};
	const AuthorityResult Below = Judge(0.64);
	const AuthorityResult Above = Judge(0.65);
	const StandingResult ElasticBelow = SolveWith(Wall.Model, LateralLoad(0.64));
	const StandingResult ElasticAbove = SolveWith(Wall.Model, LateralLoad(0.65));
	const double Seconds = SecondsSince(Start);
	INFO("elastic " << ElasticBelow.State << " at 0.64, " << ElasticAbove.State << " at 0.65; " << Seconds << " s");
	CHECK(Below.Worst == ComponentVerdict::Stands);
	CHECK(Above.Worst == ComponentVerdict::Collapses);
}

TEST_CASE("Slow.Statics.Anchor.ForaboschiWallAtFortyKilonewtonsStandsAtTheModelCapacityAndCollapsesAboveIt")
{
	const auto Start = std::chrono::steady_clock::now();
	const BlockWall Wall = MakeForaboschiWall(DryJoint(0.6).CompressiveStrengthPa);
	CHECK(Wall.Blocks.size() == 376u);
	CHECK(Wall.Contacts == 1000);
	const double VerticalN = 40000.0;
	const double ModelCapacityN = 23900.0;
	const double CollapsedN = 28000.0;
	const std::vector<BodyLoad> AtModel = BeamLoads(Wall, VerticalN, ModelCapacityN);
	const std::vector<BodyLoad> Beyond = BeamLoads(Wall, VerticalN, CollapsedN);
	const AuthorityResult Below = MakeElasticAuthority(MakeRbsmStandingAuthority())->Judge(Wall.Model, AtModel, AuthorityOptions{}, nullptr);
	const AuthorityResult Above = MakeElasticAuthority(MakeRbsmStandingAuthority())->Judge(Wall.Model, Beyond, AuthorityOptions{}, nullptr);
	const StandingResult ElasticBelow = SolveWith(Wall.Model, StandingOptions{}, AtModel);
	const StandingResult ElasticAbove = SolveWith(Wall.Model, StandingOptions{}, Beyond);
	INFO("elastic " << ElasticBelow.State << " / " << ElasticAbove.State << ", authority " << static_cast<int>(Below.Worst) << " / " << static_cast<int>(Above.Worst) << "; " << SecondsSince(Start) << " s");
	CHECK(Below.Worst != ComponentVerdict::Collapses);
	CHECK(Above.Worst == ComponentVerdict::Collapses);
}

TEST_CASE("Slow.Statics.Anchor.TheMinhoWallStandsBelowItsLimitAndCollapsesAboveIt")
{
	const auto Start = std::chrono::steady_clock::now();
	const BlockWall Wall = MakeMinhoWall(MinhoKgPerM3);
	CHECK(Wall.Blocks.size() == 56u);
	CHECK(Wall.Contacts == 146);
	const double HeightM = 1.0;
	const std::vector<BodyLoad> AtBelow = MinhoLoads(Wall, 16560.0, HeightM);
	const std::vector<BodyLoad> AtAbove = MinhoLoads(Wall, 16700.0, HeightM);
	const AuthorityResult Below = MakeElasticAuthority(MakeRbsmStandingAuthority())->Judge(Wall.Model, AtBelow, AuthorityOptions{}, nullptr);
	const AuthorityResult Above = MakeElasticAuthority(MakeRbsmStandingAuthority())->Judge(Wall.Model, AtAbove, AuthorityOptions{}, nullptr);
	const StandingResult ElasticBelow = SolveWith(Wall.Model, StandingOptions{}, AtBelow);
	const StandingResult ElasticAbove = SolveWith(Wall.Model, StandingOptions{}, AtAbove);
	INFO("elastic " << ElasticBelow.State << "/" << ElasticAbove.State << ", authority " << static_cast<int>(Below.Worst) << "/" << static_cast<int>(Above.Worst) << ", " << SecondsSince(Start) << " s");
	CHECK(Below.Worst == ComponentVerdict::Stands);
	CHECK(Above.Worst == ComponentVerdict::Collapses);
}
