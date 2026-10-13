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
