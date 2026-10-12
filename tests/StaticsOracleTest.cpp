#include "FactorTestSupport.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"

#include "doctest.h"

#include <vector>

using namespace loadpath;
using namespace loadpath::test;
using namespace loadpath::test::oracle;

namespace
{
StandingOptions BiaxialOptions(double LambdaX, double LambdaY)
{
	StandingOptions Options;
	Options.Gravity = BiaxialGravity(LambdaX, LambdaY);
	return Options;
}

struct GridCounts
{
	int CanStand = 0;
	int CannotStand = 0;
	int Ambiguous = 0;
	int ClosedFormCanStand = 0;
	int Disagreements = 0;
	int Unconverged = 0;
};

GridCounts SingleBrickGrid(double Friction)
{
	GridCounts Counts;
	const Structure Model = MakeStack(Friction, {0.0});
	for (int IX = 0; IX <= GridStepsX; ++IX)
	{
		for (int IY = 0; IY <= GridStepsY; ++IY)
		{
			const double LambdaX = GridStep * IX;
			const double LambdaY = GridStep * IY;
			const OracleVerdict Oracle = StaticAdmissibility(Model, BiaxialGravity(LambdaX, LambdaY));
			const bool bClosedForm = RigidBrickCanStand(LambdaX, LambdaY, Friction);
			Counts.CanStand += Oracle.Verdict == Admissibility::CanStand ? 1 : 0;
			Counts.CannotStand += Oracle.Verdict == Admissibility::CannotStand ? 1 : 0;
			Counts.Ambiguous += Oracle.Verdict == Admissibility::Ambiguous ? 1 : 0;
			Counts.ClosedFormCanStand += bClosedForm ? 1 : 0;
			const bool bAgrees = bClosedForm ? Oracle.Verdict == Admissibility::CanStand : Oracle.Verdict == Admissibility::CannotStand;
			Counts.Disagreements += bAgrees ? 0 : 1;
			Counts.Unconverged += Oracle.bConverged ? 0 : 1;
		}
	}
	return Counts;
}
}

TEST_CASE("Statics.Solver.VerdictsNeverContradictTheStaticAdmissibilityOracle")
{
	SUBCASE("the oracle reproduces the rigid single-brick closed form")
	{
		struct Case
		{
			double Friction;
			int ExpectedCanStand;
		};
		const Case Cases[] = {{0.6, 22}, {5.0, 351}};
		const int GridPoints = (GridStepsX + 1) * (GridStepsY + 1);
		for (const Case& Each : Cases)
		{
			const GridCounts Counts = SingleBrickGrid(Each.Friction);
			INFO("friction " << Each.Friction << ": oracle can stand " << Counts.CanStand << ", cannot " << Counts.CannotStand << ", ambiguous " << Counts.Ambiguous << ", closed form can stand " << Counts.ClosedFormCanStand << ", disagreements " << Counts.Disagreements << ", unconverged " << Counts.Unconverged);
			CHECK(Counts.ClosedFormCanStand == Each.ExpectedCanStand);
			CHECK(Counts.CanStand == Each.ExpectedCanStand);
			CHECK(Counts.CannotStand == GridPoints - Each.ExpectedCanStand);
			CHECK(Counts.Ambiguous == 0);
			CHECK(Counts.Disagreements == 0);
			CHECK(Counts.Unconverged == 0);
		}
	}
	SUBCASE("seeded stacks and bridges")
	{
		int Stands = 0;
		int Mechanism = 0;
		int Undecided = 0;
		int OtherState = 0;
		int CanStandDraws = 0;
		int CannotStandDraws = 0;
		int AmbiguousDraws = 0;
		for (int Draw = 0; Draw < DrawCount; ++Draw)
		{
			const SeededDraw Each = MakeSeededDraw(Draw);
			const StandingResult Result = SolveWith(Each.Model, BiaxialOptions(Each.LambdaX, Each.LambdaY));
			const OracleVerdict Oracle = StaticAdmissibility(Each.Model, BiaxialGravity(Each.LambdaX, Each.LambdaY));
			INFO("master seed " << MasterSeed << ", draw " << Draw << " (generator seed " << Each.Seed << "): " << Each.Fixture << ", friction " << Each.Friction << ", lambda (" << Each.LambdaX << ", " << Each.LambdaY << ")");
			INFO("RBSM " << Result.State << " after " << Result.ActiveSetIterations << " passes, mechanism bodies " << Result.Mechanism.size() << "; oracle " << Oracle.Verdict << ", inner residual / demand " << Oracle.InnerResidualOverDemand << ", outer " << Oracle.OuterResidualOverDemand);
			CHECK(Oracle.bConverged);
			if (Oracle.Verdict == Admissibility::CannotStand)
			{
				CHECK(Result.State != StandingState::Stands);
			}
			if (Oracle.Verdict == Admissibility::CanStand)
			{
				CHECK(Result.State != StandingState::Mechanism);
				CHECK(Result.Mechanism.empty());
			}
			Stands += Result.State == StandingState::Stands ? 1 : 0;
			Mechanism += Result.State == StandingState::Mechanism ? 1 : 0;
			Undecided += Result.State == StandingState::Undecided ? 1 : 0;
			OtherState += Result.State == StandingState::Overstressed || Result.State == StandingState::InvalidInput ? 1 : 0;
			CanStandDraws += Oracle.Verdict == Admissibility::CanStand ? 1 : 0;
			CannotStandDraws += Oracle.Verdict == Admissibility::CannotStand ? 1 : 0;
			AmbiguousDraws += Oracle.Verdict == Admissibility::Ambiguous ? 1 : 0;
		}
		INFO("over " << DrawCount << " draws: RBSM stands " << Stands << ", mechanism " << Mechanism << ", undecided " << Undecided << ", other " << OtherState << "; oracle can stand " << CanStandDraws << ", cannot " << CannotStandDraws << ", ambiguous " << AmbiguousDraws);
		CHECK(OtherState == 0);
		CHECK(Stands > 0);
		CHECK(Mechanism > 0);
		CHECK(CanStandDraws > 0);
		CHECK(CannotStandDraws > 0);
	}
}

TEST_CASE("Statics.Oracle.RefusesABondedWallAndJudgesADryOne")
{
	const OracleVerdict Bonded = StaticAdmissibility(Ec6Wall(8, 10), BiaxialGravity(0.0, 0.5));
	INFO("8 x 10 ec6 wall at lambda (0, 0.5): " << Bonded.Verdict);
	CHECK(Bonded.Verdict == Admissibility::Unsupported);
	const OracleVerdict Dry = StaticAdmissibility(RunningBondWall(DryJoint(0.7), DryJoint(0.7), 3, 4), BiaxialGravity(0.1, 0.0));
	INFO("3 x 4 dry wall at lambda (0.1, 0): " << Dry.Verdict << ", inner residual / demand " << Dry.InnerResidualOverDemand);
	CHECK(Dry.Verdict == Admissibility::CanStand);
}
