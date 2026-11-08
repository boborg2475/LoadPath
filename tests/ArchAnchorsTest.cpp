#include "ArchFixtures.h"
#include "SlowInDebug.h"
#include "StaticsTestSupport.h"

#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

using namespace loadpath;
using namespace loadpath::test;

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Parked.Statics.Anchor.OrdunaHollowPileStandsThroughTheAuthorityAtLambdaPointFour")
{
	const HollowPile Pile = MakeHollowPile(0.7);
	CHECK(Pile.Blocks.size() == 30u);
	AuthorityOptions Options;
	Options.Standing = LateralLoad(0.4);
	const AuthorityResult Combined = MakeElasticAuthority(MakeRbsmStandingAuthority())->Judge(Pile.Model, {}, Options, nullptr);
	const StandingResult Elastic = SolveWith(Pile.Model, LateralLoad(0.4));
	INFO("lambda 0.4: elastic " << Elastic.State << ", authority worst " << static_cast<int>(Combined.Worst) << ", components " << Combined.Components.size());
	CHECK(Combined.Components.size() == 1u);
	CHECK(Combined.Worst == ComponentVerdict::Stands);
	CHECK(Elastic.State != StandingState::Mechanism);
}
