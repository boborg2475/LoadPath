#include "CertificateTestSupport.h"
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
constexpr double FtlLp = 0.64286;
constexpr double FtlMpec = 0.63898;

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

BlockWall MakeForaboschiBondWall(double LengthM, int Courses, double CompressiveStrengthPa)
{
	BlockWall Wall;
	StartWall(Wall, LengthM, ForDepthM);
	for (int Course = 0; Course < Courses; ++Course)
	{
		AddCourse(Wall, CourseEdges(LengthM, ForBlockM, Course % 2 == 1), ForCourseM * Course, ForCourseM * (Course + 1), ForDepthM, ForMasonryKgPerM3);
	}
	const double TopM = ForCourseM * Courses;
	AddBlock(Wall, 0.0, LengthM, TopM, TopM + ForBeamM, ForDepthM, ForBeamKgPerM3);
	Wall.Beam = Wall.Blocks.back();
	Wall.BeamCentreM = Wall.Shapes.back().CentreM;
	StrengthProfile Profile = DryJoint(0.6);
	Profile.CompressiveStrengthPa = CompressiveStrengthPa;
	ConnectAll(Wall, AddProfileOk(Wall.Model, Profile), LengthM, ForDepthM);
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

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Anchor.FerrisTinLoiWallStandsToTheAssociativeLoadFactor")
{
	const auto Start = std::chrono::steady_clock::now();
	const BlockWall Wall = MakeFerrisTinLoiWall();
	CHECK(Wall.Blocks.size() == 33u);
	CHECK(Wall.Contacts == 83);
	const ThresholdBracket Standing = Threshold([&](double Lambda) { return CertificateStands(Wall.Model, CertificateLateral(Lambda)); }, 0.6, 0.7, 1.0e-6);
	const ThresholdBracket Holding = Threshold([&](double Lambda) { return DecideWith(Wall.Model, CertificateLateral(Lambda)).State != CertificateState::Collapses; }, 0.6, 0.7, 1.0e-4);
	const double StandsUpTo = 0.5 * (Standing.LastStanding + Standing.FirstFailing);
	const double CollapsesFrom = 0.5 * (Holding.LastStanding + Holding.FirstFailing);
	INFO("stands up to " << StandsUpTo << ", collapses from " << CollapsesFrom << "; LP " << FtlLp << ", MPEC " << FtlMpec);
	CHECK(Standing.bBracketed);
	CHECK(Holding.bBracketed);
	CHECK(std::abs(StandsUpTo - FtlLp) <= 1.0e-5);
	CHECK(StandsUpTo > FtlMpec);
	CHECK(CollapsesFrom <= 0.65);

	const auto Judge = [&](double Lambda) {
		AuthorityOptions Options;
		Options.Standing = LateralLoad(Lambda);
		Options.Certificate = CertificateLateral(Lambda);
		return MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate())->Judge(Wall.Model, {}, Options, nullptr);
	};
	const AuthorityResult Below = Judge(0.64);
	const AuthorityResult Above = Judge(0.65);
	const StandingResult ElasticBelow = SolveWith(Wall.Model, LateralLoad(0.64));
	const StandingResult ElasticAbove = SolveWith(Wall.Model, LateralLoad(0.65));
	const double Seconds = SecondsSince(Start);
	INFO("elastic " << ElasticBelow.State << " at 0.64, " << ElasticAbove.State << " at 0.65; " << Seconds << " s");
	MESSAGE("Ferris-Tin-Loi lambda stands to " << StandsUpTo << ", collapses from " << CollapsesFrom << ", elastic " << ElasticBelow.State << "/" << ElasticAbove.State << ", " << Seconds << " s");
	CHECK(Below.Worst == ComponentVerdict::Stands);
	CHECK(Above.Worst == ComponentVerdict::Collapses);
	CHECK(Below.Disagreements == 0);
}

TEST_CASE("Slow.Certificate.Anchor.ForaboschiWallAtFortyKilonewtonsStandsAtTheModelCapacityAndCollapsesAboveIt")
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
	const CertificateResult Holds = DecideWith(Wall.Model, CertificateOptions{}, AtModel);
	const CertificateResult Falls = DecideWith(Wall.Model, CertificateOptions{}, Beyond);
	INFO("blocks " << Wall.Blocks.size() << ", contacts " << Wall.Contacts << "; at " << ModelCapacityN << " N " << Holds.State << " [" << Holds.LoadFactorLower << ", " << Holds.LoadFactorUpper << "], at " << CollapsedN << " N " << Falls.State << " [" << Falls.LoadFactorLower << ", " << Falls.LoadFactorUpper << "]");
	CHECK(Holds.State == CertificateState::Stands);
	CHECK(Falls.State == CertificateState::Collapses);

	const AuthorityResult Below = MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate())->Judge(Wall.Model, AtModel, AuthorityOptions{}, nullptr);
	const AuthorityResult Above = MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate())->Judge(Wall.Model, Beyond, AuthorityOptions{}, nullptr);
	const StandingResult ElasticBelow = SolveWith(Wall.Model, StandingOptions{}, AtModel);
	const StandingResult ElasticAbove = SolveWith(Wall.Model, StandingOptions{}, Beyond);
	const double Seconds = SecondsSince(Start);
	INFO("elastic " << ElasticBelow.State << " / " << ElasticAbove.State << ", combined " << static_cast<int>(Below.Worst) << " / " << static_cast<int>(Above.Worst) << ", disagreements " << Below.Disagreements << "; " << Seconds << " s");
	MESSAGE("Foraboschi 40 kN: elastic " << ElasticBelow.State << "/" << ElasticAbove.State << ", combined " << static_cast<int>(Below.Worst) << "/" << static_cast<int>(Above.Worst) << ", " << Seconds << " s");
	CHECK(Below.Worst != ComponentVerdict::Collapses);
	CHECK(Above.Worst == ComponentVerdict::Collapses);
}

TEST_CASE("Slow.Certificate.Solver.TheForaboschiWallStandsInsideItsAssociativeLimit")
{
	struct Case
	{
		double VerticalN;
		double HorizontalN;
		double LpLimitN;
	};
	const Case Cases[] = {{80000.0, 42000.0, 50280.0}, {140000.0, 80000.0, 83610.0}};
	const BlockWall Wall = MakeForaboschiWall(1.0e9);
	REQUIRE(Wall.Blocks.size() == 376u);
	REQUIRE(Wall.Contacts == 1000);
	for (const Case& K : Cases)
	{
		const auto Start = std::chrono::steady_clock::now();
		const CertificateResult Got = DecideWith(Wall.Model, CertificateOptions{}, BeamLoads(Wall, K.VerticalN, K.HorizontalN));
		const double Seconds = SecondsSince(Start);
		INFO("V " << K.VerticalN << " H " << K.HorizontalN << " (2-D LP limit " << K.LpLimitN << "): " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations << ", " << Seconds << " s");
		MESSAGE("Foraboschi V " << K.VerticalN << " H " << K.HorizontalN << ": " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations << ", " << Seconds << " s");
		CHECK(K.HorizontalN < K.LpLimitN);
		CHECK(Got.State == CertificateState::Stands);
		CHECK(Got.LoadFactorUpper < CertificateOptions{}.LoadFactorCap);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Solver.AFiniteLoadFactorBelowTheCapStandsOnTheTwentyThreeBlockWall")
{
	struct Case
	{
		double CompressivePa;
		double VerticalN;
		double HorizontalN;
	};
	const Case Cases[] = {{1.0e8, 80000.0, 43860.0}, {1.0e8, 80000.0, 44880.0}, {1.0e8, 80000.0, 46750.0}, {3.0e7, 40000.0, 24120.0}, {1.0e7, 80000.0, 46920.0}};
	const double BeamWeightN = 25000.0 * 1.04 * ForBeamM * ForDepthM;
	CHECK(std::abs(BeamWeightN - 1261.0) < 1.0e-9);
	const auto Start = std::chrono::steady_clock::now();
	for (const Case& K : Cases)
	{
		const BlockWall Wall = MakeForaboschiBondWall(1.04, 5, K.CompressivePa);
		REQUIRE(Wall.Blocks.size() == 23u);
		REQUIRE(Wall.Contacts == 57);
		CHECK(std::abs(Wall.Model.Body(Wall.Beam).MassKg * GravityMPerS2 - BeamWeightN) < 1.0e-9);
		const double LimitN = 0.6 * (K.VerticalN + BeamWeightN);
		const CertificateResult Below = DecideWith(Wall.Model, CertificateOptions{}, BeamLoads(Wall, K.VerticalN, K.HorizontalN));
		const CertificateResult Past = DecideWith(Wall.Model, CertificateOptions{}, BeamLoads(Wall, K.VerticalN, 1.02 * LimitN));
		INFO("fc " << K.CompressivePa << " V " << K.VerticalN << " H " << K.HorizontalN << ": " << Below.State << " [" << Below.LoadFactorLower << ", " << Below.LoadFactorUpper << "] in " << Below.Iterations << "; at 1.02 H* = " << 1.02 * LimitN << ": " << Past.State << " [" << Past.LoadFactorLower << ", " << Past.LoadFactorUpper << "]");
		CHECK(K.HorizontalN < LimitN);
		CHECK(Below.State == CertificateState::Stands);
		CHECK(Below.LoadFactorUpper < CertificateOptions{}.LoadFactorCap);
		CHECK(Past.State == CertificateState::Collapses);
	}
	MESSAGE("23-block wall: " << SecondsSince(Start) << " s");
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Solver.TheFortySixBlockWallDecidesAtAnIsolatedStallLoad")
{
	const auto Start = std::chrono::steady_clock::now();
	const BlockWall Wall = MakeForaboschiBondWall(1.04, 10, 1.21e6);
	REQUIRE(Wall.Blocks.size() == 46u);
	REQUIRE(Wall.Contacts == 116);
	CHECK(6 * static_cast<int>(Wall.Blocks.size()) <= CertificateOptions{}.DenseRowLimit);
	const double VerticalN = 80000.0;
	for (const double HorizontalN : {47149.0, 47151.0, 47150.0})
	{
		const CertificateResult Got = DecideWith(Wall.Model, CertificateOptions{}, BeamLoads(Wall, VerticalN, HorizontalN));
		INFO("H " << HorizontalN << ": " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations);
		CHECK(Got.State == CertificateState::Collapses);
		CHECK(Got.LoadFactorUpper < 1.0);
		if (HorizontalN == 47150.0)
		{
			CHECK(Got.LoadFactorLower >= 0.55301);
			CHECK(Got.LoadFactorLower <= 0.55307);
		}
	}
	MESSAGE("46-block wall: " << SecondsSince(Start) << " s");
}

TEST_CASE("Slow.Certificate.Solver.TheSparseWallDecidesAtAnIsolatedStallLoad")
{
	const auto Start = std::chrono::steady_clock::now();
	const BlockWall Wall = MakeForaboschiBondWall(2.08, 16, 1.21e6);
	REQUIRE(Wall.Blocks.size() == 137u);
	REQUIRE(Wall.Contacts == 377);
	CHECK(6 * static_cast<int>(Wall.Blocks.size()) > CertificateOptions{}.DenseRowLimit);
	const CertificateResult Got = DecideWith(Wall.Model, CertificateOptions{}, BeamLoads(Wall, 80000.0, 43000.0));
	INFO(std::setprecision(10) << "137 blocks, " << Wall.Contacts << " contacts, H 43000: " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations);
	MESSAGE(std::setprecision(10) << "137-block wall: " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations << ", contacts " << Wall.Contacts << ", " << SecondsSince(Start) << " s");
	CHECK(Got.State == CertificateState::Stands);
	CHECK(Got.LoadFactorLower >= 2.7140);
	CHECK(Got.LoadFactorLower <= 2.7147);
	CHECK(Got.LoadFactorUpper < CertificateOptions{}.LoadFactorCap);
}

namespace
{
BodyId AddPebbleOnBeam(BlockWall& Wall)
{
	const geometry::Box& BeamShape = Wall.Shapes.back();
	const Box BeamBox{BeamShape.CentreM, BeamShape.HalfExtentsM, Wall.Model.Body(Wall.Beam).MassKg};
	const double HalfM = 0.005;
	const Box Pebble = BoxOfDensity(Vec3{Wall.BeamCentreM.X + 0.2, 0.0, BeamShape.CentreM.Z + BeamShape.HalfExtentsM.Z + HalfM}, Vec3{HalfM, HalfM, HalfM}, 2400.0);
	const BodyId Body = AddBox(Wall.Model, Pebble);
	const std::optional<InterfaceDesc> Contact = StackedFaces(Wall.Beam, BeamBox, Body, Pebble, AddProfileOk(Wall.Model, DryJoint(0.6)));
	REQUIRE(Contact.has_value());
	AddInterfaceOk(Wall.Model, *Contact);
	return Body;
}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Solver.APebbleOnTheBeamChangesNoVerdict")
{
	const auto Start = std::chrono::steady_clock::now();
	BlockWall Small = MakeForaboschiBondWall(1.04, 5, 1.0e8);
	REQUIRE(Small.Blocks.size() == 23u);
	const BodyId SmallPebble = AddPebbleOnBeam(Small);
	CHECK(std::abs(Small.Model.Body(SmallPebble).MassKg - 0.0024) < 1.0e-12);
	for (const double HorizontalN : {43860.0, 46750.0, 30000.0})
	{
		const CertificateResult Got = DecideWith(Small.Model, CertificateOptions{}, BeamLoads(Small, 80000.0, HorizontalN));
		INFO("23 blocks and a pebble, H " << HorizontalN << ": " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations);
		CHECK(Got.State == CertificateState::Stands);
		CHECK(Got.LoadFactorUpper < CertificateOptions{}.LoadFactorCap);
	}
	BlockWall Large = MakeForaboschiBondWall(1.04, 10, 1.21e6);
	REQUIRE(Large.Blocks.size() == 46u);
	AddPebbleOnBeam(Large);
	const CertificateResult Got = DecideWith(Large.Model, CertificateOptions{}, BeamLoads(Large, 80000.0, 47149.0));
	INFO("46 blocks and a pebble, H 47149: " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations);
	CHECK(Got.State == CertificateState::Collapses);
	CHECK(Got.LoadFactorUpper < 1.0);
	MESSAGE("pebble walls: " << SecondsSince(Start) << " s");
}

TEST_CASE("Certificate.Solver.ALightBlockOnAHeavyOneReadsItsAnalyticLimit")
{
	const double Lambda = 0.5;
	const double Mu = 0.3;
	const double HeavyKg = 1000.0;
	const double Rho = CircumscribedOverInscribed(16);
	const double Tolerance = 1.0e-3;
	const auto Start = std::chrono::steady_clock::now();
	int Rows = 0;
	for (const double HalfM : {0.005, 0.5})
	{
		const double OverlapFraction = HalfM < 0.1 ? 1.0 : 0.9;
		for (const double Ratio : {1.0, 1.0e2, 1.0e4, 1.0e5, 1.0e6, 1.0e7, 1.0e8, 1.0e10, 1.0e11, 1.0e12})
		{
			const double LightKg = HeavyKg / Ratio;
			const double LightWeightN = LightKg * GravityMPerS2;
			const auto Build = [&](const StrengthProfile& Joint)
			{
				Structure Model;
				const BodyId Ground = AddGroundPlane(Model);
				const Box Heavy{Vec3{0.0, 0.0, 0.5}, Vec3{0.5, 0.5, 0.5}, HeavyKg};
				const BodyId HeavyBody = AddBox(Model, Heavy);
				AddInterfaceOk(Model, OnGroundPlane(Ground, HeavyBody, Heavy, AddProfileOk(Model, DryJoint(0.8))));
				const Box Light{Vec3{-0.1, 0.0, 1.0 + HalfM}, Vec3{HalfM, HalfM, HalfM}, LightKg};
				const BodyId LightBody = AddBox(Model, Light);
				const std::optional<InterfaceDesc> Contact = StackedFaces(HeavyBody, Heavy, LightBody, Light, AddProfileOk(Model, Joint));
				REQUIRE(Contact.has_value());
				AddInterfaceOk(Model, *Contact);
				return Model;
			};
			for (const double Target : {2.0, 0.5})
			{
				StrengthProfile Joint = DryJoint(Mu);
				Joint.ShearCohesionPa = Target * LightWeightN * (Lambda - Mu) / (4.0 * HalfM * HalfM);
				const CertificateResult Got = DecideWith(Build(Joint), CertificateLateral(Lambda));
				const double WantIn = OverlapFraction * Target;
				const double WantOut = WantIn * Rho * (Lambda - Mu) / (Lambda - Mu * Rho);
				INFO("half " << HalfM << " m, ratio " << Ratio << ", target " << Target << ": " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations << "; want [" << WantIn << ", " << WantOut << "]");
				CHECK(Got.State == (Target > 1.0 ? CertificateState::Stands : CertificateState::Collapses));
				CHECK(std::abs(Got.LoadFactorLower - WantIn) <= Tolerance * WantIn);
				CHECK(std::abs(Got.LoadFactorUpper - WantOut) <= Tolerance * WantOut);
				++Rows;
			}
			for (const double Friction : {0.6, 0.4})
			{
				const CertificateResult Got = DecideWith(Build(DryJoint(Friction)), CertificateLateral(Lambda));
				INFO("half " << HalfM << " m, ratio " << Ratio << ", dry mu " << Friction << ": " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "]");
				CHECK(Got.State == (Friction > Lambda ? CertificateState::Stands : CertificateState::Collapses));
				++Rows;
			}
		}
	}
	CHECK(Rows == 80);
	MESSAGE("light block table: " << SecondsSince(Start) << " s");
}

TEST_CASE("Slow.Certificate.Anchor.TheMinhoWallCollapsesJustPastItsAssociativeLimit")
{
	const auto Start = std::chrono::steady_clock::now();
	const BlockWall Wall = MakeMinhoWall(MinhoKgPerM3);
	CHECK(Wall.Blocks.size() == 56u);
	CHECK(Wall.Contacts == 146);
	const double BeamWeightN = Wall.Model.Body(Wall.Beam).MassKg * GravityMPerS2;
	CHECK(std::abs(BeamWeightN - 500.0) < 1.0e-9);
	const double HeightM = 1.0;
	const double DilatancyLift = 0.2 * MinhoFriction / HeightM / (1.0 + MinhoFriction * MinhoFriction);
	const double LimitN = ((MinhoVerticalN + BeamWeightN) * 0.5 * MinhoLengthM + 1210.0) / HeightM + 1800.0 * MinhoFriction * DilatancyLift;
	CHECK(std::abs(LimitN - 16569.2) < 0.05);
	for (const double H : {16560.0, 16575.0, 16600.0, 16700.0})
	{
		const CertificateResult Got = DecideWith(Wall.Model, CertificateOptions{}, MinhoLoads(Wall, H, HeightM));
		INFO("H " << H << ": " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations << ", limit " << LimitN);
		CHECK(Got.State == (H < LimitN ? CertificateState::Stands : CertificateState::Collapses));
	}
	const std::vector<BodyLoad> AtBelow = MinhoLoads(Wall, 16560.0, HeightM);
	const std::vector<BodyLoad> AtAbove = MinhoLoads(Wall, 16700.0, HeightM);
	const AuthorityResult Below = MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate())->Judge(Wall.Model, AtBelow, AuthorityOptions{}, nullptr);
	const AuthorityResult Above = MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate())->Judge(Wall.Model, AtAbove, AuthorityOptions{}, nullptr);
	const StandingResult ElasticBelow = SolveWith(Wall.Model, StandingOptions{}, AtBelow);
	const StandingResult ElasticAbove = SolveWith(Wall.Model, StandingOptions{}, AtAbove);
	MESSAGE("Minho: blocks " << Wall.Blocks.size() << ", contacts " << Wall.Contacts << ", elastic " << ElasticBelow.State << "/" << ElasticAbove.State << ", combined " << static_cast<int>(Below.Worst) << "/" << static_cast<int>(Above.Worst) << ", disagreements " << Below.Disagreements << "/" << Above.Disagreements << ", " << SecondsSince(Start) << " s");
	CHECK(Below.Worst == ComponentVerdict::Stands);
	CHECK(Above.Worst == ComponentVerdict::Collapses);

	const BlockWall Control = MakeMinhoWall(0.0);
	const double ControlHeightM = Control.BeamCentreM.Z;
	const double ControlLift = 0.2 * MinhoFriction / ControlHeightM / (1.0 + MinhoFriction * MinhoFriction);
	const double ControlLimitN = (MinhoVerticalN * 0.5 * MinhoLengthM + 1210.0) / ControlHeightM + 1800.0 * MinhoFriction * ControlLift;
	CHECK(std::abs(ControlLimitN - 15542.1) < 0.05);
	const CertificateResult ControlBelow = DecideWith(Control.Model, CertificateOptions{}, MinhoLoads(Control, 15540.0, ControlHeightM));
	const CertificateResult ControlPast = DecideWith(Control.Model, CertificateOptions{}, MinhoLoads(Control, 15545.0, ControlHeightM));
	INFO("centroid-push control, limit " << ControlLimitN << ": 15540 " << ControlBelow.State << " [" << ControlBelow.LoadFactorLower << ", " << ControlBelow.LoadFactorUpper << "], 15545 " << ControlPast.State << " [" << ControlPast.LoadFactorLower << ", " << ControlPast.LoadFactorUpper << "]");
	CHECK(ControlBelow.State == CertificateState::Stands);
	CHECK(ControlPast.State == CertificateState::Collapses);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Selection.ARockingWallPastItsLimitNamesItsMechanism")
{
	const auto Start = std::chrono::steady_clock::now();
	const BlockWall Wall = MakeForaboschiBondWall(0.78, 16, DryJoint(0.6).CompressiveStrengthPa);
	REQUIRE(Wall.Blocks.size() == 57u);
	REQUIRE(Wall.Contacts == 137);
	const double VerticalN = 80000.0;
	const CertificateResult Control = DecideWith(Wall.Model, CertificateOptions{}, BeamLoads(Wall, VerticalN, 25000.0));
	INFO("at 25000 N: " << Control.State << " [" << Control.LoadFactorLower << ", " << Control.LoadFactorUpper << "]");
	CHECK(Control.State != CertificateState::Collapses);
	for (const double HorizontalN : {32782.75, 32897.66, 32832.5, 32957.75, 32958.0})
	{
		const CertificateResult Past = DecideWith(Wall.Model, CertificateOptions{}, BeamLoads(Wall, VerticalN, HorizontalN));
		INFO("H " << HorizontalN << ": " << Past.State << " [" << Past.LoadFactorLower << ", " << Past.LoadFactorUpper << "], moving " << Past.UnconstrainedBodies.size() << ", hinging " << Past.HingingInterfaces.size());
		CHECK(Past.LoadFactorUpper < 1.0);
		CHECK(Past.State == CertificateState::Collapses);
		CHECK(HasBody(Past.UnconstrainedBodies, Wall.Beam));
		CHECK(!Past.HingingInterfaces.empty());
	}
	MESSAGE("rocking wall selection: " << SecondsSince(Start) << " s");
}
TEST_CASE("Slow.Certificate.Selection.TheForaboschiWallPastItsLimitNamesItsMechanism")
{
	const auto Start = std::chrono::steady_clock::now();
	const BlockWall Wall = MakeForaboschiWall(1.0e9);
	REQUIRE(Wall.Blocks.size() == 376u);
	REQUIRE(Wall.Contacts == 1000);
	const CertificateResult Past = DecideWith(Wall.Model, CertificateOptions{}, BeamLoads(Wall, 140000.0, 85000.0));
	const double Seconds = SecondsSince(Start);
	INFO("V 140000 H 85000: " << Past.State << " [" << Past.LoadFactorLower << ", " << Past.LoadFactorUpper << "] in " << Past.Iterations << ", moving " << Past.UnconstrainedBodies.size() << ", hinging " << Past.HingingInterfaces.size() << ", " << Seconds << " s");
	MESSAGE("Foraboschi selection: " << Past.State << " [" << Past.LoadFactorLower << ", " << Past.LoadFactorUpper << "], moving " << Past.UnconstrainedBodies.size() << ", hinging " << Past.HingingInterfaces.size() << ", " << Seconds << " s");
	CHECK(Past.State == CertificateState::Collapses);
	CHECK(HasBody(Past.UnconstrainedBodies, Wall.Beam));
	CHECK(!Past.HingingInterfaces.empty());
}
TEST_CASE("Slow.Certificate.Anchor.ForaboschiWallWithCrushingBracketsThePaperModel")
{
	struct Case
	{
		double VerticalN;
		double ModelCapacityN;
		double CollapsedN;
	};
	const Case Cases[] = {{40000.0, 23900.0, 27000.0}, {80000.0, 39800.0, 44000.0}, {140000.0, 55800.0, 62000.0}};
	const auto Start = std::chrono::steady_clock::now();
	const BlockWall Wall = MakeForaboschiWall(1.21e6);
	REQUIRE(Wall.Blocks.size() == 376u);
	REQUIRE(Wall.Contacts == 1000);
	for (const Case& K : Cases)
	{
		const CertificateResult Holds = DecideWith(Wall.Model, CertificateOptions{}, BeamLoads(Wall, K.VerticalN, K.ModelCapacityN));
		const CertificateResult Falls = DecideWith(Wall.Model, CertificateOptions{}, BeamLoads(Wall, K.VerticalN, K.CollapsedN));
		INFO("V " << K.VerticalN << ": at " << K.ModelCapacityN << " N " << Holds.State << " [" << Holds.LoadFactorLower << ", " << Holds.LoadFactorUpper << "], at " << K.CollapsedN << " N " << Falls.State << " [" << Falls.LoadFactorLower << ", " << Falls.LoadFactorUpper << "]");
		MESSAGE("V " << K.VerticalN << ": " << Holds.State << " [" << Holds.LoadFactorLower << ", " << Holds.LoadFactorUpper << "] / " << Falls.State << " [" << Falls.LoadFactorLower << ", " << Falls.LoadFactorUpper << "]");
		CHECK(Holds.State == CertificateState::Stands);
		CHECK(Falls.State == CertificateState::Collapses);
	}
	MESSAGE("Foraboschi with crushing: " << SecondsSince(Start) << " s");
}
