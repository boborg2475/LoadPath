#include "BrickFixtures.h"
#include "StaticsTestSupport.h"

#include "loadpath/Settings.h"
#include "loadpath/StandingAuthority.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
const DebugSettings RecordingOn{true};
constexpr double LambdaX = 0.625;
constexpr double LambdaY = 0.875;

StandingOptions PastFriction(double Threshold)
{
	StandingOptions Options;
	Options.Gravity = Vec3{LambdaX * GravityMPerS2, LambdaY * GravityMPerS2, -GravityMPerS2};
	Options.RestoreClosingRateThreshold = Threshold;
	return Options;
}

int OpenCornersOn(const StandingResult& Result, InterfaceId Id)
{
	const InterfaceReading* Reading = FindReading(Result, Id);
	REQUIRE(Reading != nullptr);
	int Count = 0;
	for (const CornerReading& C : Reading->Corners)
	{
		Count += C.bOpen ? 1 : 0;
	}
	return Count;
}

const JointPassRecord* RecordOf(const ElasticPassRecord& Pass, InterfaceId Id)
{
	const auto It = std::find_if(Pass.Joints.begin(), Pass.Joints.end(), [&](const JointPassRecord& J) { return J.Id == Id; });
	return It == Pass.Joints.end() ? nullptr : &*It;
}

bool Contains(const std::vector<BodyId>& Bodies, BodyId Body)
{
	return std::find(Bodies.begin(), Bodies.end(), Body) != Bodies.end();
}
}

TEST_CASE("Statics.Restore.AClosingCornerOpenInTheLastFullRankSolveIsReclosedAndTheStackSlides")
{
	const TwoBrickStack S = MakeTwoBrickStack(DryJoint(0.6), 0.03);
	StandingOptions Options = PastFriction(StandingOptions{}.RestoreClosingRateThreshold);
	Options.Debug = &RecordingOn;
	const StandingResult R = SolveWith(S.Model, Options);
	REQUIRE(R.Components.size() == 1u);
	INFO("state " << static_cast<int>(R.State) << ", end " << static_cast<int>(R.Components[0].End));
	CHECK(StandingOptions{}.RestoreClosingRateThreshold == 0.1);
	CHECK(R.State == StandingState::Mechanism);
	CHECK(R.Components[0].End == SolveEnd::Mechanism);
	CHECK(R.UnconstrainedBodies.size() == 2u);
	CHECK(OpenCornersOn(R, S.Bed) == 0);
	CHECK(OpenCornersOn(R, S.Joint) == 0);
	REQUIRE(R.Mechanism.size() == 2u);
	for (const MechanismMotion& M : R.Mechanism)
	{
		INFO("body " << M.Body.Index << " translation z " << M.TranslationM.Z);
		CHECK(std::abs(M.TranslationM.Z) <= 1.0e-9 * std::hypot(M.TranslationM.X, M.TranslationM.Y));
	}

	const StandingResult NeverReclosed = SolveWith(S.Model, PastFriction(2.0));
	const InterfaceReading* Unrestored = FindReading(NeverReclosed, S.Bed);
	REQUIRE(Unrestored != nullptr);
	const std::vector<ElasticPassRecord>& Passes = R.Components[0].Passes;
	bool bReclosedAfterOpen = false;
	for (std::size_t K = 0; K + 1 < Passes.size(); ++K)
	{
		const JointPassRecord* Before = RecordOf(Passes[K], S.Bed);
		const JointPassRecord* After = RecordOf(Passes[K + 1], S.Bed);
		REQUIRE(Before != nullptr);
		REQUIRE(After != nullptr);
		for (std::size_t C = 0; C < 4; ++C)
		{
			bReclosedAfterOpen = bReclosedAfterOpen || (Before->Corners[C] == CornerState::Open && After->Corners[C] != CornerState::Open && Unrestored->Corners[C].bOpen);
		}
	}
	CHECK_MESSAGE(bReclosedAfterOpen, "a bed corner open in one recorded iteration is closed in the next, and stays open when no restore may re-close it");
}

TEST_CASE("Statics.Restore.AThresholdAboveTheCornersClosingRateLeavesItOpen")
{
	const TwoBrickStack S = MakeTwoBrickStack(DryJoint(0.6), 0.03);
	const StandingResult AtDefault = SolveWith(S.Model, PastFriction(StandingOptions{}.RestoreClosingRateThreshold));
	const StandingResult AboveRate = SolveWith(S.Model, PastFriction(0.5));
	REQUIRE(AtDefault.Components.size() == 1u);
	REQUIRE(AboveRate.Components.size() == 1u);
	CHECK(OpenCornersOn(AtDefault, S.Bed) == 0);
	CHECK(OpenCornersOn(AboveRate, S.Bed) > 0);
}

TEST_CASE("Statics.Restore.ACornerReclosedBeforeAnyFullRankSolveSlipsWithNoFrictionCapacity")
{
	TwoBrickStack S = MakeTwoBrickStack(DryJoint(0.6), 0.03);
	StrengthProfile SoftShear = DryJoint(0.6);
	SoftShear.TangentialStiffnessPaPerM = 1.0e-14 * SoftShear.NormalStiffnessPaPerM;
	const ProfileId Soft = AddProfileOk(S.Model, SoftShear);
	const Box Upper = BrickShape(0.03, 1);
	const Box Cap{Vec3{0.03, 0.0, 2.0 * BrickHeightM + 0.01}, Vec3{0.02, 0.02, 0.01}, 1.0e-3};
	const BodyId CapId = AddBox(S.Model, Cap);
	AddInterfaceOk(S.Model, *StackedFaces(S.Upper, Upper, CapId, Cap, Soft));
	const Vec3 LateralN{LambdaX * BrickMassKg * GravityMPerS2, LambdaY * BrickMassKg * GravityMPerS2, 0.0};
	const BodyLoad Loads[] = {{S.Lower, LateralN, BrickShape(0.0, 0).CentreM}, {S.Upper, LateralN, Upper.CentreM}};
	const StandingResult R = SolveWith(S.Model, StandingOptions{}, Loads);
	const StandingResult NeverReclosed = [&] {
		StandingOptions Options;
		Options.RestoreClosingRateThreshold = 2.0;
		return SolveWith(S.Model, Options, Loads);
	}();
	REQUIRE(R.Components.size() == 1u);
	INFO("state " << static_cast<int>(R.State) << ", end " << static_cast<int>(R.Components[0].End));
	CHECK(OpenCornersOn(NeverReclosed, S.Bed) > 0);
	CHECK(R.State == StandingState::Mechanism);
	CHECK(Contains(R.UnconstrainedBodies, S.Lower));
	CHECK(Contains(R.UnconstrainedBodies, S.Upper));
	const InterfaceReading* Bed = FindReading(R, S.Bed);
	REQUIRE(Bed != nullptr);
	for (const CornerReading& C : Bed->Corners)
	{
		CHECK_FALSE(C.bOpen);
		CHECK(C.bSlipping);
	}
}
