#include "AnchorFixtures.h"
#include "StaticsTestSupport.h"

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

TEST_CASE("Statics.Anchor.HarmonicStackStandsInsideItsOverhangAndNotBeyond")
{
	const double PublishedOverhangL[] = {0.0, 0.5, 0.75, 11.0 / 12.0, 25.0 / 24.0};
	for (int Count = 2; Count <= 4; ++Count)
	{
		INFO("n = " << Count);
		CHECK(std::abs(HarmonicRightEdgesFromTopM(Count, 1.0).front() - PublishedOverhangL[Count] * BlockLengthM) <= 1.0e-12);

		const Stack Inside = BuildOnTable(HarmonicStack(Count, 1.0 - 1.0e-6), 0.0);
		const StandingResult InsideResult = SolveWith(Inside.Model, StandingOptions{});
		CHECK(InsideResult.State == StandingState::Stands);

		const Stack Beyond = BuildOnTable(HarmonicStack(Count, 1.0 + 1.0e-6), 0.0);
		const StandingResult BeyondResult = SolveWith(Beyond.Model, StandingOptions{});
		INFO("beyond reads " << BeyondResult.State);
		CHECK(BeyondResult.State == StandingState::Mechanism);
	}
}

TEST_CASE("Statics.Anchor.InvertedThreeTriangleFallsAndFourDiamondStands")
{
	const Stack Triangle = BuildOnTable(BrickRows({1, 2, 3}), 5.0);
	CHECK(Triangle.Model.LiveBodyCount() == 7u);
	CHECK(Triangle.Model.LiveInterfaceCount() == 7u);
	const StandingResult TriangleResult = SolveWith(Triangle.Model, StandingOptions{});
	INFO("inverted 3-triangle reads " << TriangleResult.State);
	CHECK(TriangleResult.State == StandingState::Mechanism);

	const Stack Diamond = BuildOnTable(BrickRows({1, 2, 3, 4, 3, 2, 1}), 5.0);
	CHECK(Diamond.Model.LiveBodyCount() == 17u);
	CHECK(Diamond.Model.LiveInterfaceCount() == 25u);
	const StandingResult DiamondResult = SolveWith(Diamond.Model, StandingOptions{});
	INFO("4-diamond reads " << DiamondResult.State);
	CHECK(DiamondResult.State == StandingState::Stands);
}
