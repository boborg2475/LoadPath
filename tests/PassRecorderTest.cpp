#include "BitwiseTestSupport.h"
#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/Settings.h"
#include "loadpath/StandingAuthority.h"

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
const DebugSettings RecordingOn{true};
const DebugSettings RecordingOff{false};

StandingOptions Recording(StandingOptions Options, const DebugSettings* Debug)
{
	Options.Debug = Debug;
	return Options;
}

StandingOptions BiaxialLateralLoad(double LambdaX, double LambdaY)
{
	StandingOptions Options;
	Options.Gravity = Vec3{LambdaX * GravityMPerS2, LambdaY * GravityMPerS2, -GravityMPerS2};
	return Options;
}

const char* EndName(SolveEnd End)
{
	switch (End)
	{
	case SolveEnd::None: return "None";
	case SolveEnd::Converged: return "Converged";
	case SolveEnd::Mechanism: return "Mechanism";
	case SolveEnd::RestoreFailedClosed: return "RestoreFailedClosed";
	case SolveEnd::PowerCheckFailed: return "PowerCheckFailed";
	case SolveEnd::IterationCap: return "IterationCap";
	case SolveEnd::InvalidInput: return "InvalidInput";
	}
	return "SolveEnd(?)";
}

CornerState StateOf(const CornerReading& Corner)
{
	return Corner.bOpen ? CornerState::Open : Corner.bSlipping ? CornerState::Slipping : CornerState::Closed;
}

std::string DescribePasses(const std::vector<ElasticPassRecord>& Passes)
{
	std::string Out;
	for (const ElasticPassRecord& Pass : Passes)
	{
		Out += " it" + std::to_string(Pass.Iteration) + "[";
		for (const JointPassRecord& J : Pass.Joints)
		{
			Out += " " + std::to_string(J.Id.Index) + ":";
			for (std::size_t C = 0; C < 4; ++C)
			{
				Out += J.Corners[C] == CornerState::Open ? "o" : J.Corners[C] == CornerState::Slipping ? "s" : "c";
				Out += J.bFlipped[C] ? "!" : "";
			}
		}
		Out += " ]";
	}
	return Out;
}

struct BondedPier
{
	Structure Model;
	std::vector<BodyId> Bricks;
	std::vector<InterfaceId> JointsBottomUp;
	std::vector<BodyLoad> Loads;
};

BondedPier MakeCrackingPier()
{
	constexpr double PushN = 50.0;
	constexpr double Friction = 0.7;
	constexpr double CohesionPa = 1.0e5;
	const double CornerAreaM2 = BrickLengthM * BrickWidthM / 4.0;
	const double BaseTensionN = PushN * 3.0 * BrickHeightM / (2.0 * BrickLengthM) - 3.0 * BrickMassKg * GravityMPerS2 / 4.0;
	const double TensilePa = BaseTensionN / (1.3 * CornerAreaM2);
	BondedPier P;
	const BodyId Ground = AddGroundPlane(P.Model);
	const ProfileId Profile = AddProfileOk(P.Model, BondedJoint(TensilePa, CohesionPa, Friction));
	for (int Course = 0; Course < 3; ++Course)
	{
		P.Bricks.push_back(AddBox(P.Model, BrickShape(0.0, Course)));
	}
	P.JointsBottomUp.push_back(AddInterfaceOk(P.Model, OnGroundPlane(Ground, P.Bricks[0], BrickShape(0.0, 0), Profile)));
	for (int Course = 1; Course < 3; ++Course)
	{
		P.JointsBottomUp.push_back(AddInterfaceOk(P.Model, *StackedFaces(P.Bricks[static_cast<std::size_t>(Course - 1)], BrickShape(0.0, Course - 1), P.Bricks[static_cast<std::size_t>(Course)], BrickShape(0.0, Course), Profile)));
	}
	P.Loads.push_back(BodyLoad{P.Bricks[2], Vec3{PushN, 0.0, 0.0}, Vec3{0.0, 0.0, 3.0 * BrickHeightM}});
	return P;
}

CascadeOptions CascadeRecording(const DebugSettings* Debug)
{
	CascadeOptions Options;
	Options.Authority.Standing.Debug = Debug;
	return Options;
}
}

TEST_CASE("Statics.Recorder.OffByDefaultRecordsNothing")
{
	const TwoBrickStack S = MakeTwoBrickStack(DryJoint(5.0), 0.03);
	const StandingOptions Load = BiaxialLateralLoad(1.375, 0.625);
	SUBCASE("null Debug")
	{
		const StandingResult R = SolveWith(S.Model, Load);
		REQUIRE(R.Components.size() == 1u);
		CHECK(R.Components[0].Passes.empty());
		CHECK(R.Components[0].End == SolveEnd::Converged);
	}
	SUBCASE("Debug present, recording false")
	{
		const StandingResult R = SolveWith(S.Model, Recording(Load, &RecordingOff));
		REQUIRE(R.Components.size() == 1u);
		CHECK(R.Components[0].Passes.empty());
		CHECK(R.Components[0].End == SolveEnd::Converged);
	}
}

TEST_CASE("Statics.Recorder.EveryIterationIsRecordedWithItsFlips")
{
	const TwoBrickStack S = MakeTwoBrickStack(DryJoint(5.0), 0.03);
	const StandingResult R = SolveWith(S.Model, Recording(BiaxialLateralLoad(1.375, 0.625), &RecordingOn));
	REQUIRE(R.Components.size() == 1u);
	const StandingComponent& Component = R.Components[0];
	INFO("state " << R.State << ", iterations " << Component.ActiveSetIterations << ", passes" << DescribePasses(Component.Passes));
	CHECK(R.State == StandingState::Stands);
	CHECK(Component.ActiveSetIterations >= 3);
	REQUIRE(Component.Passes.size() == static_cast<std::size_t>(Component.ActiveSetIterations));
	CHECK(std::any_of(Component.Passes[0].Joints.begin(), Component.Passes[0].Joints.end(), [](const JointPassRecord& Joint) { return std::any_of(Joint.Corners.begin(), Joint.Corners.end(), [](CornerState State) { return State != CornerState::Closed; }); }));
	if (Component.Passes.size() != static_cast<std::size_t>(Component.ActiveSetIterations))
	{
		return;
	}
	for (std::size_t I = 0; I < Component.Passes.size(); ++I)
	{
		const ElasticPassRecord& Pass = Component.Passes[I];
		INFO("iteration " << Pass.Iteration);
		CHECK(Pass.Iteration == static_cast<int>(I) + 1);
		REQUIRE(Pass.Joints.size() == Component.ReadingSlots.size());
		for (std::size_t J = 0; J < Pass.Joints.size(); ++J)
		{
			const JointPassRecord& Joint = Pass.Joints[J];
			CHECK(SameHandle(Joint.Id, R.Readings[Component.ReadingSlots[J]].Id));
			for (std::size_t C = 0; C < 4; ++C)
			{
				const bool bWantFlipped = I > 0 && Component.Passes[I - 1].Joints[J].Corners[C] != Joint.Corners[C];
				CHECK(Joint.bFlipped[C] == bWantFlipped);
			}
		}
	}
	const ElasticPassRecord& Last = Component.Passes.back();
	for (std::size_t J = 0; J < Last.Joints.size(); ++J)
	{
		const InterfaceReading& Reading = R.Readings[Component.ReadingSlots[J]];
		for (std::size_t C = 0; C < 4; ++C)
		{
			CHECK(Last.Joints[J].Corners[C] == StateOf(Reading.Corners[C]));
			CHECK(Last.Joints[J].CornerNormalN[C] == Reading.Corners[C].NormalForceN);
		}
		CHECK(Last.Joints[J].Utilisation == Reading.Utilisation);
		CHECK(Last.Joints[J].Axis == Reading.Axis);
	}
	bool bAnyFlip = false;
	for (std::size_t I = 1; I < Component.Passes.size(); ++I)
	{
		for (const JointPassRecord& Joint : Component.Passes[I].Joints)
		{
			for (const bool bFlipped : Joint.bFlipped)
			{
				bAnyFlip = bAnyFlip || bFlipped;
			}
		}
	}
	CHECK(bAnyFlip);
}

TEST_CASE("Statics.Recorder.TheEndReasonNamesWhyTheSolveStopped")
{
	SUBCASE("a standing brick converged")
	{
		const BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
		const StandingResult R = SolveWith(F.Model, StandingOptions{});
		REQUIRE(R.Components.size() == 1u);
		INFO("end " << EndName(R.Components[0].End));
		CHECK(R.State == StandingState::Stands);
		CHECK(R.Components[0].End == SolveEnd::Converged);
		CHECK(R.Components[0].Blocking.empty());
	}
	SUBCASE("an ungrounded body is a mechanism")
	{
		Structure Model;
		AddGroundPlane(Model);
		AddBox(Model, Box{Vec3{1.0, 0.0, 1.0}, BrickShape().HalfExtentsM, BrickMassKg});
		const StandingResult R = SolveWith(Model, StandingOptions{});
		REQUIRE(R.Components.size() == 1u);
		INFO("end " << EndName(R.Components[0].End));
		CHECK(R.State == StandingState::Mechanism);
		CHECK(R.Components[0].End == SolveEnd::Mechanism);
	}
	SUBCASE("a brick tipped across its width is a mechanism through the full solve")
	{
		const BrickOnGround F = MakeBrickOnGround(DryJoint(2.0));
		const StandingResult R = SolveWith(F.Model, BiaxialLateralLoad(0.0, BrickWidthM / BrickHeightM + 1.0e-3));
		REQUIRE(R.Components.size() == 1u);
		INFO("end " << EndName(R.Components[0].End) << " after " << R.ActiveSetIterations);
		CHECK(R.State == StandingState::Mechanism);
		CHECK(R.ActiveSetIterations >= 1);
		CHECK(R.Components[0].End == SolveEnd::Mechanism);
	}
	SUBCASE("the iteration cap")
	{
		const BrickOnGround F = MakeBrickOnGround(DryJoint(0.6));
		StandingOptions Options = TiltedPlane(29.0 * RadPerDegree);
		Options.MaxActiveSetIterations = 1;
		const StandingResult R = SolveWith(F.Model, Options);
		REQUIRE(R.Components.size() == 1u);
		INFO("end " << EndName(R.Components[0].End));
		CHECK(R.State == StandingState::Undecided);
		CHECK(R.Components[0].End == SolveEnd::IterationCap);
		CHECK(R.Components[0].Blocking.empty());
	}
}

TEST_CASE("Statics.Recorder.RecordingChangesNoAnswer")
{
	SUBCASE("a two-brick stack under a biaxial load")
	{
		const TwoBrickStack S = MakeTwoBrickStack(DryJoint(5.0), 0.03);
		const StandingOptions Load = BiaxialLateralLoad(1.375, 0.625);
		const StandingResult Plain = SolveWith(S.Model, Load);
		const StandingResult Recorded = SolveWith(S.Model, Recording(Load, &RecordingOn));
		CHECK(bitwise::Same(Plain, Recorded));
		REQUIRE(Recorded.Components.size() == 1u);
		CHECK(Recorded.Components[0].End == Plain.Components[0].End);
		CHECK_FALSE(Recorded.Components[0].Passes.empty());
	}
	SUBCASE("a brick tipped into a mechanism")
	{
		const BrickOnGround F = MakeBrickOnGround(DryJoint(2.0));
		const StandingOptions Load = BiaxialLateralLoad(0.0, BrickWidthM / BrickHeightM + 1.0e-3);
		const StandingResult Plain = SolveWith(F.Model, Load);
		const StandingResult Recorded = SolveWith(F.Model, Recording(Load, &RecordingOn));
		CHECK(Plain.State == StandingState::Mechanism);
		CHECK(bitwise::Same(Plain, Recorded));
		REQUIRE(Recorded.Components.size() == 1u);
		CHECK(Recorded.Components[0].Passes.size() == static_cast<std::size_t>(Recorded.Components[0].ActiveSetIterations));
	}
	SUBCASE("a bonded pier through the cascade")
	{
		BondedPier Plain = MakeCrackingPier();
		BondedPier Recorded = MakeCrackingPier();
		const CascadeVerdict A = MakeCascade(MakeAuthority())->Run(Plain.Model, Plain.Loads, CascadeOptions{}, nullptr);
		const CascadeVerdict B = MakeCascade(MakeAuthority())->Run(Recorded.Model, Recorded.Loads, CascadeRecording(&RecordingOn), nullptr);
		INFO("plain " << DescribeVerdict(A) << "\nrecorded " << DescribeVerdict(B));
		CHECK(bitwise::Same(A, B));
		CHECK(A.PassRecords.empty());
		CHECK_FALSE(B.PassRecords.empty());
	}
}

TEST_CASE("Cascade.Recorder.EveryPassIsRecordedWithItsReleasesAndCracks")
{
	BondedPier P = MakeCrackingPier();
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(P.Model, P.Loads, CascadeRecording(&RecordingOn), nullptr);
	INFO(DescribeVerdict(V) << ", recorded passes " << V.PassRecords.size());
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 2);
	REQUIRE(V.PassRecords.size() == static_cast<std::size_t>(V.Passes) + 1u);
	if (V.PassRecords.size() != static_cast<std::size_t>(V.Passes) + 1u)
	{
		return;
	}
	std::vector<ReleasedCluster> Clusters;
	std::vector<FailedInterface> Failures;
	for (std::size_t I = 0; I < V.PassRecords.size(); ++I)
	{
		const CascadePassRecord& Record = V.PassRecords[I];
		INFO("record " << I << " pass " << Record.Pass);
		CHECK(Record.Pass == V.FirstPass + static_cast<std::uint32_t>(I));
		if (I < static_cast<std::size_t>(V.Passes))
		{
			REQUIRE(Record.Components.size() == 1u);
			if (Record.Components.size() == 1u)
			{
				CHECK(Record.Components[0].End != SolveEnd::None);
				CHECK_FALSE(Record.Components[0].Passes.empty());
			}
			CHECK_FALSE(Record.Readings.empty());
		}
		Clusters.insert(Clusters.end(), Record.Clusters.begin(), Record.Clusters.end());
		Failures.insert(Failures.end(), Record.Failures.begin(), Record.Failures.end());
	}
	CHECK(V.PassRecords[0].Components[0].Verdict == ComponentVerdict::Overstressed);
	CHECK(V.PassRecords[0].Failures.size() == 1u);
	CHECK(V.PassRecords[0].Clusters.empty());
	CHECK(V.PassRecords[1].Components[0].Verdict == ComponentVerdict::Collapses);
	CHECK(V.PassRecords[1].Components[0].End == SolveEnd::Mechanism);
	CHECK(V.PassRecords[1].Clusters.size() == 1u);
	CHECK(V.PassRecords[2].Clusters.empty());
	CHECK(V.PassRecords[2].Failures.empty());
	CHECK(bitwise::Same(Clusters, V.Clusters));
	CHECK(bitwise::Same(Failures, V.Failures));
	CHECK(bitwise::Same(V.PassRecords.back().Readings, V.Readings));
	CHECK(bitwise::Same(V.PassRecords.back().Components, V.Components));
}
