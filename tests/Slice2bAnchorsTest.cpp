#include "AnchorFixtures.h"
#include "StaticsTestSupport.h"

#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double ShapeTolerance = 1.0e-6;
constexpr double OnePercent = 0.01;
const Vec3 PlusX{1.0, 0.0, 0.0};
const Vec3 EdgeAxis{0.0, 1.0, 0.0};

const char* VerdictName(ComponentVerdict Verdict)
{
	switch (Verdict)
	{
	case ComponentVerdict::Stands: return "Stands";
	case ComponentVerdict::Overstressed: return "Overstressed";
	case ComponentVerdict::Collapses: return "Collapses";
	case ComponentVerdict::Undecided: return "Undecided";
	case ComponentVerdict::InvalidInput: return "InvalidInput";
	}
	return "ComponentVerdict(?)";
}

AuthorityOptions Lateral(double Lambda)
{
	AuthorityOptions Options;
	Options.Standing = LateralLoad(Lambda);
	return Options;
}

AuthorityResult JudgeWith(const Structure& Model, const AuthorityOptions& Options, std::span<const BodyLoad> Loads = {})
{
	return MakeElasticAuthority(MakeRbsmStandingAuthority())->Judge(Model, Loads, Options, nullptr);
}

struct BothReadings
{
	StandingResult Elastic;
	AuthorityResult Combined;
};

BothReadings ReadBoth(const Structure& Model, const AuthorityOptions& Options, std::span<const BodyLoad> Loads = {})
{
	BothReadings R{SolveWith(Model, Options.Standing, Loads), JudgeWith(Model, Options, Loads)};
	return R;
}

void CheckStandsOnBoth(const BothReadings& R)
{
	CHECK(R.Elastic.State == StandingState::Stands);
	INFO("combined " << doctest::String(VerdictName(R.Combined.Worst)));
	CHECK(R.Combined.Worst == ComponentVerdict::Stands);
}

void CheckFailsOnBoth(const BothReadings& R)
{
	CHECK(R.Elastic.State == StandingState::Mechanism);
	INFO("combined " << doctest::String(VerdictName(R.Combined.Worst)));
	CHECK(R.Combined.Worst == ComponentVerdict::Collapses);
}

std::vector<BodyId> CombinedRelease(const AuthorityResult& Result)
{
	std::vector<BodyId> Release;
	for (const ComponentReport& Component : Result.Components)
	{
		Release.insert(Release.end(), Component.Release.begin(), Component.Release.end());
	}
	return Release;
}

void CheckMovingSetIs(const std::vector<BodyId>& Moving, const std::vector<BodyId>& Want)
{
	INFO("moving " << Describe(Moving));
	CHECK(Moving.size() == Want.size());
	for (const BodyId& Each : Want)
	{
		CHECK(HasBody(Moving, Each));
	}
}

void CheckSlidesAlongX(const StandingResult& Result, BodyId Body)
{
	const MechanismMotion* Motion = FindMotion(Result, Body);
	CHECK(Motion != nullptr);
	if (Motion != nullptr)
	{
		CheckTranslationAlong(*Motion, PlusX, ShapeTolerance);
	}
}

void CheckTipsAboutTheToe(const StandingResult& Result, BodyId Body, const Box& Shape)
{
	const MechanismMotion* Motion = FindMotion(Result, Body);
	CHECK(Motion != nullptr);
	if (Motion != nullptr)
	{
		CheckRotationAboutEdge(*Motion, Shape.CentreM, DownhillEdgePointM(Shape), EdgeAxis, PlusX, ShapeTolerance);
	}
}

void CheckSlidesPastFrictionOnBoth(const Structure& Model, BodyId Body, double Friction)
{
	const BothReadings InsideBand = ReadBoth(Model, Lateral(Friction * (1.0 + OnePercent)));
	CHECK(InsideBand.Elastic.State == StandingState::Mechanism);
	CheckSlidesAlongX(InsideBand.Elastic, Body);
	INFO("combined inside the polygon band " << doctest::String(VerdictName(InsideBand.Combined.Worst)));
	CHECK(InsideBand.Combined.Worst == ComponentVerdict::Undecided);
	const BothReadings PastBand = ReadBoth(Model, Lateral(Friction / std::cos(Pi / 16.0) * (1.0 + OnePercent)));
	CheckFailsOnBoth(PastBand);
	CheckSlidesAlongX(PastBand.Elastic, Body);
	CHECK(CombinedRelease(PastBand.Combined).size() == 1u);
	CHECK(HasBody(CombinedRelease(PastBand.Combined), Body));
}

struct StepStack
{
	Stack Built;
	std::vector<BodyId> TopBlocks;
};

StepStack ConstantStepStack(int Count, double StepM)
{
	std::vector<PlacedBlock> Placed;
	for (int K = 0; K < Count; ++K)
	{
		Placed.push_back(PlacedBlock{K * StepM, K});
	}
	StepStack S{BuildOnTable(Placed, 5.0), {}};
	for (std::size_t K = 1; K < S.Built.Blocks.size(); ++K)
	{
		S.TopBlocks.push_back(S.Built.Blocks[K]);
	}
	return S;
}

constexpr double LibertonLengthM = 3.65;
constexpr double LibertonHeightM = 2.3;
constexpr double LibertonMassKg = 1865.0;
constexpr int LibertonCourses = 10;

StrengthProfile LibertonBondedJoint()
{
	StrengthProfile Profile = DryJoint(0.6);
	Profile.TensileStrengthPa = 0.1e6;
	Profile.ShearCohesionPa = 0.2e6;
	return Profile;
}

struct LibertonWall
{
	Structure Model;
	std::vector<WallCourse> Courses;
	std::vector<InterfaceId> JointsBottomUp;
};

LibertonWall MakeLibertonWall(double ThicknessM)
{
	LibertonWall W;
	const ProfileId Cracked = AddProfileOk(W.Model, DryJoint(0.6));
	const ProfileId Bonded = AddProfileOk(W.Model, LibertonBondedJoint());
	const BodyId Ground = AddGroundPlane(W.Model);
	const double CourseHeightM = LibertonHeightM / LibertonCourses;
	for (int K = 0; K < LibertonCourses; ++K)
	{
		const Box Shape{Vec3{0.0, 0.0, (K + 0.5) * CourseHeightM}, Vec3{ThicknessM / 2.0, LibertonLengthM / 2.0, CourseHeightM / 2.0}, LibertonMassKg / LibertonCourses};
		W.Courses.push_back(WallCourse{AddBox(W.Model, Shape), Shape});
	}
	W.JointsBottomUp.push_back(AddInterfaceOk(W.Model, OnGroundPlane(Ground, W.Courses[0].Body, W.Courses[0].Shape, Cracked)));
	for (std::size_t K = 1; K < W.Courses.size(); ++K)
	{
		W.JointsBottomUp.push_back(AddInterfaceOk(W.Model, *StackedFaces(W.Courses[K - 1].Body, W.Courses[K - 1].Shape, W.Courses[K].Body, W.Courses[K].Shape, Bonded)));
	}
	return W;
}

BodyLoad PushAt(const LibertonWall& W, double HeightM, double ForceN)
{
	const double CourseHeightM = LibertonHeightM / LibertonCourses;
	int Course = static_cast<int>(HeightM / CourseHeightM);
	if (Course >= LibertonCourses)
	{
		Course = LibertonCourses - 1;
	}
	return BodyLoad{W.Courses[static_cast<std::size_t>(Course)].Body, Vec3{ForceN, 0.0, 0.0}, Vec3{0.0, 0.0, HeightM}};
}

std::vector<BodyId> CourseBodies(const LibertonWall& W)
{
	std::vector<BodyId> All;
	for (const WallCourse& Each : W.Courses)
	{
		All.push_back(Each.Body);
	}
	return All;
}

void CheckOnlyTheBaseOpens(const StandingResult& Result, const LibertonWall& W)
{
	for (std::size_t J = 0; J < W.JointsBottomUp.size(); ++J)
	{
		const InterfaceReading* Reading = FindReading(Result, W.JointsBottomUp[J]);
		CHECK(Reading != nullptr);
		if (Reading == nullptr)
		{
			continue;
		}
		bool bAnyOpen = false;
		for (const CornerReading& Corner : Reading->Corners)
		{
			bAnyOpen = bAnyOpen || Corner.bOpen;
		}
		INFO("joint " << J << " any corner open " << bAnyOpen);
		CHECK(bAnyOpen == (J == 0));
	}
}
}

TEST_CASE("Parked.Statics.Anchor.FreeWallWithFrictionBelowThicknessOverHeightSlidesAtFriction")
{
	const double ThicknessM = BrickWidthM;
	const double HeightM = 10.0 * BrickHeightM;
	const double Friction = 0.1;
	CHECK(Friction < ThicknessM / HeightM);
	const BlockOnGround F = MakeBlockOnGround(ThicknessM, 1.0, HeightM, Friction, BrickDensityKgPerM3());

	CheckStandsOnBoth(ReadBoth(F.Model, Lateral(Friction * (1.0 - OnePercent))));
	CheckSlidesPastFrictionOnBoth(F.Model, F.Block, Friction);
	const BothReadings Failing = ReadBoth(F.Model, Lateral(Friction * (1.0 + OnePercent)));
	const InterfaceReading* Bed = FindReading(Failing.Elastic, F.Bed);
	CHECK(Bed != nullptr);
	if (Bed != nullptr)
	{
		for (const CornerReading& Corner : Bed->Corners)
		{
			CHECK_FALSE(Corner.bOpen);
			CHECK(Corner.bSlipping);
		}
	}
}

TEST_CASE("Statics.Anchor.ConstantStepStackStandsUpToLengthOverCountAndItsTopBlocksRotateBeyond")
{
	for (int Count = 2; Count <= 5; ++Count)
	{
		INFO("n = " << Count);
		const double StepM = BlockLengthM / Count;
		const StepStack Inside = ConstantStepStack(Count, StepM * (1.0 - OnePercent));
		CheckStandsOnBoth(ReadBoth(Inside.Built.Model, Lateral(0.0)));

		const StepStack Beyond = ConstantStepStack(Count, StepM * (1.0 + OnePercent));
		const BothReadings Failing = ReadBoth(Beyond.Built.Model, Lateral(0.0));
		CheckFailsOnBoth(Failing);
		CheckMovingSetIs(Failing.Elastic.UnconstrainedBodies, Beyond.TopBlocks);
		CheckMovingSetIs(CombinedRelease(Failing.Combined), Beyond.TopBlocks);
		const MechanismMotion* Motion = FindMotion(Failing.Elastic, Beyond.TopBlocks.front());
		CHECK(Motion != nullptr);
		if (Motion != nullptr)
		{
			INFO("rotation " << Motion->RotationRad);
			CHECK(std::abs(Motion->RotationRad.Y) > 0.0);
		}
	}
}

TEST_CASE("Parked.Statics.Anchor.SeatedBeamSlidesAtTheMirandolaAccelerationOnNeopreneAndHoldsAtTheCodeFriction")
{
	const double MirandolaLambda = 0.264;
	for (const double Friction : {0.13, 0.35})
	{
		INFO("mu = " << Friction);
		const BlockOnGround F = MakeBlockOnGround(1.0, 0.4, 0.6, Friction, 2400.0);
		CheckStandsOnBoth(ReadBoth(F.Model, Lateral(Friction * (1.0 - OnePercent))));
		CheckSlidesPastFrictionOnBoth(F.Model, F.Block, Friction);

		const BothReadings Mirandola = ReadBoth(F.Model, Lateral(MirandolaLambda));
		if (Friction < MirandolaLambda)
		{
			CheckFailsOnBoth(Mirandola);
			CheckSlidesAlongX(Mirandola.Elastic, F.Block);
		}
		else
		{
			CheckStandsOnBoth(Mirandola);
		}
	}
}

TEST_CASE("Statics.Anchor.ChristchurchParapetOverturnsAtThicknessOverHeight")
{
	const double ThicknessM = 0.230;
	const double Friction = 0.7;
	for (const double HeightM : {0.5, 1.0})
	{
		const double Lambda = ThicknessM / HeightM;
		INFO("h = " << HeightM << ", t/h = " << Lambda);
		CHECK(Friction > Lambda);
		const BlockOnGround F = MakeBlockOnGround(ThicknessM, 1.0, HeightM, Friction, BrickDensityKgPerM3());
		CheckStandsOnBoth(ReadBoth(F.Model, Lateral(Lambda * (1.0 - OnePercent))));
		const BothReadings Failing = ReadBoth(F.Model, Lateral(Lambda * (1.0 + OnePercent)));
		CheckFailsOnBoth(Failing);
		CheckTipsAboutTheToe(Failing.Elastic, F.Block, F.Shape);
	}
}

TEST_CASE("Statics.Anchor.LibertonWallStandsUnderSelfWeightWithOnlyTheCrackedBaseAbleToOpen")
{
	for (const double ThicknessM : {0.14, 0.105})
	{
		INFO("t = " << ThicknessM);
		const LibertonWall W = MakeLibertonWall(ThicknessM);
		CheckStandsOnBoth(ReadBoth(W.Model, Lateral(0.0)));
		const std::vector<BodyLoad> Small{PushAt(W, LibertonHeightM, 30.0)};
		const BothReadings Pushed = ReadBoth(W.Model, Lateral(0.0), Small);
		CheckStandsOnBoth(Pushed);
		for (const InterfaceReading& Reading : Pushed.Elastic.Readings)
		{
			CHECK(Reading.Utilisation < 1.0);
		}
	}
}

TEST_CASE("Statics.Anchor.LibertonWallTipsAtWeightTimesHalfThicknessOverPushHeight")
{
	const double WeightN = LibertonMassKg * GravityMPerS2;
	for (const double ThicknessM : {0.14, 0.105})
	{
		const LibertonWall W = MakeLibertonWall(ThicknessM);
		for (const double HeightM : {LibertonHeightM, 1.2})
		{
			const double ThresholdN = WeightN * ThicknessM / (2.0 * HeightM);
			INFO("t = " << ThicknessM << ", push height " << HeightM << ", threshold " << ThresholdN << " N");
			const std::vector<BodyLoad> Under{PushAt(W, HeightM, ThresholdN * (1.0 - OnePercent))};
			CheckStandsOnBoth(ReadBoth(W.Model, Lateral(0.0), Under));
			const std::vector<BodyLoad> Over{PushAt(W, HeightM, ThresholdN * (1.0 + OnePercent))};
			const BothReadings Failing = ReadBoth(W.Model, Lateral(0.0), Over);
			CheckFailsOnBoth(Failing);
			CheckOnlyTheBaseOpens(Failing.Elastic, W);
			CheckMovingSetIs(Failing.Elastic.UnconstrainedBodies, CourseBodies(W));
			CheckMovingSetIs(CombinedRelease(Failing.Combined), CourseBodies(W));
		}
	}
}

TEST_CASE("Statics.Anchor.LnecGraniteBlockTipsStaticallyAtHalfWidthOverHalfHeight")
{
	struct GraniteBlock
	{
		double WidthM;
		double DepthM;
		double HeightM;
		double MassKg;
	};
	const GraniteBlock Blocks[] = {{0.25, 0.754, 1.0, 503.0}, {0.17, 0.502, 1.0, 228.0}};
	const double Friction = 0.7;
	for (const GraniteBlock& Block : Blocks)
	{
		const double TanAlpha = Block.WidthM / Block.HeightM;
		INFO("b/h = " << TanAlpha);
		CHECK(Friction > TanAlpha);
		const double DensityKgPerM3 = Block.MassKg / (Block.WidthM * Block.DepthM * Block.HeightM);
		const BlockOnGround F = MakeBlockOnGround(Block.WidthM, Block.DepthM, Block.HeightM, Friction, DensityKgPerM3);
		CheckStandsOnBoth(ReadBoth(F.Model, Lateral(TanAlpha * (1.0 - OnePercent))));
		const BothReadings Failing = ReadBoth(F.Model, Lateral(TanAlpha * (1.0 + OnePercent)));
		CheckFailsOnBoth(Failing);
		CheckTipsAboutTheToe(Failing.Elastic, F.Block, F.Shape);
	}
}
