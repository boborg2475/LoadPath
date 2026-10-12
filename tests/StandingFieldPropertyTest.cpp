#include "FactorTestSupport.h"
#include "OwnFieldTestSupport.h"
#include "StaticsTestSupport.h"

#include "doctest.h"

#include <algorithm>
#include <string>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct TableTally
{
	int Draws = 0;
	int Stands = 0;
	int Failed = 0;
	OwnFieldCheck Worst;
};

void CheckCase(const FactorCase& C, TableTally& Tally)
{
	++Tally.Draws;
	const StandingResult Result = SolveWith(C.Model, C.Options, C.Loads);
	if (Result.State != StandingState::Stands)
	{
		return;
	}
	++Tally.Stands;
	const OwnFieldCheck Check = CheckOwnField(C.Model, Result, C.Options.Gravity, C.Loads);
	Tally.Worst.WorstResidualOverW = std::max(Tally.Worst.WorstResidualOverW, Check.WorstResidualOverW);
	Tally.Worst.WorstTensionOverW = std::max(Tally.Worst.WorstTensionOverW, Check.WorstTensionOverW);
	Tally.Worst.WorstConeExcessOverW = std::max(Tally.Worst.WorstConeExcessOverW, Check.WorstConeExcessOverW);
	if (Check.Violations != 0)
	{
		++Tally.Failed;
		FAIL_CHECK(C.Name << ": " << Check.Violations << " violations, residual " << Check.WorstResidualOverW << " W, tension " << Check.WorstTensionOverW << " W, cone excess " << Check.WorstConeExcessOverW << " W");
	}
}

void Report(const std::string& Table, const TableTally& Tally)
{
	MESSAGE(Table << ": " << Tally.Draws << " draws, " << Tally.Stands << " Stand, " << Tally.Failed << " fail their own field; worst residual " << Tally.Worst.WorstResidualOverW << " W, tension " << Tally.Worst.WorstTensionOverW << " W, cone excess " << Tally.Worst.WorstConeExcessOverW << " W");
	CHECK(Tally.Failed == 0);
	CHECK(Tally.Stands > 0);
}
}

TEST_CASE("Statics.Field.EveryStandsIsInEquilibriumFromItsReadings")
{
	SUBCASE("seeded stacks and bridges")
	{
		TableTally Tally;
		for (int Draw = 0; Draw < FactorDrawCount; ++Draw)
		{
			CheckCase(SeededCase(Draw), Tally);
		}
		Report("seeded stacks and bridges", Tally);
	}
	SUBCASE("seeded walls")
	{
		TableTally Tally;
		for (int Draw = 0; Draw < FactorWallDrawCount; ++Draw)
		{
			CheckCase(WallCase(Draw), Tally);
		}
		Report("seeded walls", Tally);
	}
}
