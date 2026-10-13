#include "AnchorFixtures.h"
#include "ArchFixtures.h"
#include "CertificateTestSupport.h"
#include "SlowInDebug.h"
#include "StaticsTestSupport.h"


#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double WedgeAngleDeg = 25.0;
constexpr double WedgeBlockMassKg = 10.0;
constexpr double WedgeMassKg = 1.0e-3;

struct WedgeUnderBlock
{
	Structure Model;
	BodyId Wedge;
	BodyId Block;
	Vec3 PushPointM;
	double WeightN = 0.0;
};

WedgeUnderBlock MakeWedgeUnderBlock(double Friction)
{
	const double Alpha = WedgeAngleDeg * RadPerDegree;
	const Vec3 SlopeCentreM{0.0, 0.0, 0.2};
	WedgeUnderBlock F;
	F.WeightN = WedgeBlockMassKg * GravityMPerS2;
	const ProfileId Rough = AddProfileOk(F.Model, DryJoint(Friction));
	const ProfileId Smooth = AddProfileOk(F.Model, DryJoint(0.0));
	const BodyId Floor = AddGroundPlane(F.Model);
	const BodyId Wall = AddBox(F.Model, Box{Vec3{-1.0, 0.0, 0.2}, Vec3{0.1, 0.5, 0.5}, 0.0}, true);
	F.Wedge = AddBox(F.Model, Box{Vec3{0.0, 0.0, 0.1}, Vec3{0.5, 0.25, 0.1}, WedgeMassKg});
	F.Block = AddBox(F.Model, Box{Plus(SlopeCentreM, Vec3{0.0, 0.0, 0.3}), Vec3{0.3, 0.25, 0.3}, WedgeBlockMassKg});
	F.PushPointM = Vec3{SlopeCentreM.X + SlopeCentreM.Z * std::tan(Alpha), 0.0, 0.0};

	InterfaceDesc Base;
	Base.BodyA = Floor;
	Base.BodyB = F.Wedge;
	Base.NormalAToB = Vec3{0.0, 0.0, 1.0};
	Base.CentreM = F.PushPointM;
	Base.AxisU = Vec3{1.0, 0.0, 0.0};
	Base.AxisV = Vec3{0.0, 1.0, 0.0};
	Base.HalfExtentUM = 0.5;
	Base.HalfExtentVM = 0.25;
	Base.Profile = Rough;
	AddInterfaceOk(F.Model, Base);

	InterfaceDesc Slope;
	Slope.BodyA = F.Wedge;
	Slope.BodyB = F.Block;
	Slope.NormalAToB = Vec3{-std::sin(Alpha), 0.0, std::cos(Alpha)};
	Slope.CentreM = SlopeCentreM;
	Slope.AxisU = Vec3{std::cos(Alpha), 0.0, std::sin(Alpha)};
	Slope.AxisV = Vec3{0.0, 1.0, 0.0};
	Slope.HalfExtentUM = 0.3;
	Slope.HalfExtentVM = 0.25;
	Slope.Profile = Rough;
	AddInterfaceOk(F.Model, Slope);

	InterfaceDesc Guide;
	Guide.BodyA = Wall;
	Guide.BodyB = F.Block;
	Guide.NormalAToB = Vec3{1.0, 0.0, 0.0};
	Guide.CentreM = Vec3{-0.3, 0.0, SlopeCentreM.Z};
	Guide.AxisU = Vec3{0.0, 0.0, 1.0};
	Guide.AxisV = Vec3{0.0, -1.0, 0.0};
	Guide.HalfExtentUM = 0.3;
	Guide.HalfExtentVM = 0.25;
	Guide.Profile = Smooth;
	AddInterfaceOk(F.Model, Guide);
	return F;
}

std::vector<BodyLoad> Push(const WedgeUnderBlock& F, double FractionOfWeight)
{
	return {BodyLoad{F.Wedge, Vec3{-FractionOfWeight * F.WeightN, 0.0, 0.0}, F.PushPointM}};
}

constexpr double PublishedRaiseFraction = 0.935;
constexpr double PublishedHoldFraction = 0.0436;
constexpr double ArchFriction = 0.7;
constexpr double ContinuousLeastEta = 0.107478;

Stack DiamondOnTable(int Widest, double Friction)
{
	std::vector<int> Rows;
	for (int Count = 1; Count <= Widest; ++Count)
	{
		Rows.push_back(Count);
	}
	for (int Count = Widest - 1; Count >= 1; --Count)
	{
		Rows.push_back(Count);
	}
	const std::vector<PlacedBlock> Placed = BrickRows(Rows);
	Stack S;
	const ProfileId P = AddProfileOk(S.Model, DryJoint(Friction));
	const Box Table{Vec3{0.0, 0.0, -0.5}, Vec3{5.0, 5.0, 0.5}, 0.0};
	const BodyId TableId = AddBox(S.Model, Table, true);
	for (const PlacedBlock& Block : Placed)
	{
		S.Blocks.push_back(AddBox(S.Model, BlockShape(Block)));
	}
	for (std::size_t Upper = 0; Upper < Placed.size(); ++Upper)
	{
		if (Placed[Upper].Row == 0)
		{
			AddInterfaceOk(S.Model, *StackedFaces(TableId, Table, S.Blocks[Upper], BlockShape(Placed[Upper]), P));
			continue;
		}
		for (std::size_t Lower = 0; Lower < Placed.size(); ++Lower)
		{
			if (Placed[Lower].Row + 1 != Placed[Upper].Row)
			{
				continue;
			}
			const std::optional<InterfaceDesc> Contact = StackedFaces(S.Blocks[Lower], BlockShape(Placed[Lower]), S.Blocks[Upper], BlockShape(Placed[Upper]), P);
			if (Contact)
			{
				AddInterfaceOk(S.Model, *Contact);
			}
		}
	}
	return S;
}

StandingState WedgeReading(double Friction, double PushFractionOfWeight)
{
	const WedgeUnderBlock F = MakeWedgeUnderBlock(Friction);
	return SolveWith(F.Model, StandingOptions{}, Push(F, PushFractionOfWeight)).State;
}

StandingState ArchReading(double Eta, double Friction)
{
	return SolveWith(MakeSemicircularArch(Eta, DryJoint(Friction)).Model, StandingOptions{}).State;
}

bool IsNotStanding(StandingState State)
{
	return State == StandingState::Mechanism || State == StandingState::Undecided;
}
}

TEST_CASE("Statics.Anchor.FrictionlessFiveDiamondFallsAndFourDiamondStands")
{
	const Stack Four = DiamondOnTable(4, 0.0);
	const Stack Five = DiamondOnTable(5, 0.0);
	CHECK(Five.Model.LiveBodyCount() == 26u);
	const StandingResult FourResult = SolveWith(Four.Model, StandingOptions{});
	const StandingResult FiveResult = SolveWith(Five.Model, StandingOptions{});
	INFO("4-diamond " << FourResult.State << " in " << FourResult.ActiveSetIterations << "; 5-diamond " << FiveResult.State << " in " << FiveResult.ActiveSetIterations);
	CHECK(FourResult.State == StandingState::Stands);
	CHECK(FiveResult.State == StandingState::Mechanism);
	CHECK(DecideWith(Four.Model, CertificateOptions{}).State == CertificateState::Stands);
	CHECK(DecideWith(Five.Model, CertificateOptions{}).State == CertificateState::Collapses);
}

TEST_CASE("Statics.Anchor.WedgeUnderBlockSlipsOutUnheldAtFrictionPointTwoAndSelfLocksAtPointTwoFive")
{
	CHECK(WedgeReading(0.20, 0.0) == StandingState::Mechanism);
	CHECK(WedgeReading(0.25, 0.0) == StandingState::Stands);
}

TEST_CASE("Statics.Anchor.WedgeUnderBlockRaisesTheBlockAtThePublishedRaiseForce")
{
	const StandingState Below = WedgeReading(0.20, 0.99 * PublishedRaiseFraction);
	const StandingState Above = WedgeReading(0.20, 1.01 * PublishedRaiseFraction);
	INFO("0.99 raise: " << Below << "; 1.01 raise: " << Above);
	CHECK(Below == StandingState::Stands);
	CHECK(Above == StandingState::Mechanism);
	CHECK(WedgeReading(0.25, 1.01 * PublishedRaiseFraction) == StandingState::Stands);
}

TEST_CASE("Statics.Anchor.WedgeUnderBlockNeverReadsTheWrongSideOfThePublishedHoldForce")
{
	const StandingState JustBelow = WedgeReading(0.20, 0.99 * PublishedHoldFraction);
	const StandingState JustAbove = WedgeReading(0.20, 1.01 * PublishedHoldFraction);
	const StandingState Below = WedgeReading(0.20, 0.9 * PublishedHoldFraction);
	const StandingState Above = WedgeReading(0.20, 1.2 * PublishedHoldFraction);
	INFO("hold x0.9 " << Below << ", x0.99 " << JustBelow << ", x1.01 " << JustAbove << ", x1.2 " << Above);
	CHECK(IsNotStanding(JustBelow));
	CHECK(JustAbove != StandingState::Mechanism);
	CHECK(Below == StandingState::Mechanism);
	CHECK(Above == StandingState::Stands);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Anchor.CoupletHeymanArchStandsAboveLeastThicknessAndNotBelow")
{
	for (const double Eta : {ContinuousLeastEta + 0.001, 0.1095})
	{
		const StandingState State = ArchReading(Eta, ArchFriction);
		INFO("eta " << Eta << ": " << State);
		CHECK(State == StandingState::Stands);
	}
	for (const double Eta : {ContinuousLeastEta - 0.001, 0.1065})
	{
		const StandingState State = ArchReading(Eta, ArchFriction);
		INFO("eta " << Eta << ": " << State);
		CHECK(IsNotStanding(State));
	}
	const ThresholdBracket Bracket = Threshold([](double Eta) { return ArchReading(Eta, ArchFriction) == StandingState::Stands; }, 0.1095, 0.1065, 1.0e-6);
	INFO("elastic eta bracket: last standing " << Bracket.LastStanding << ", first failing " << Bracket.FirstFailing);
	CHECK(Bracket.bBracketed);
	CHECK(Bracket.LastStanding <= ContinuousLeastEta + 1.0e-5);
	CHECK(Bracket.FirstFailing >= ContinuousLeastEta - 1.0e-5);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Anchor.CoupletHeymanArchStandsAtTwentyTwoDegreesOfFrictionAndNotAtTwentyOne")
{
	const StandingState Slides = ArchReading(0.1095, std::tan(21.0 * RadPerDegree));
	const StandingState Holds = ArchReading(0.1095, std::tan(22.0 * RadPerDegree));
	INFO("phi 21: " << Slides << "; phi 22: " << Holds);
	CHECK(IsNotStanding(Slides));
	CHECK(Holds == StandingState::Stands);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Anchor.OrdunaHollowPileReadsNoFalseMechanismBelowTheBracketAndNoStandingAboveIt")
{
	const HollowPile Pile = MakeHollowPile(0.7);
	const StandingState AtLower = SolveWith(Pile.Model, LateralLoad(0.427)).State;
	const StandingState AtUpper = SolveWith(Pile.Model, LateralLoad(0.553)).State;
	INFO("lambda 0.427: " << AtLower << "; lambda 0.553: " << AtUpper);
	CHECK(AtLower != StandingState::Mechanism);
	CHECK(AtLower != StandingState::InvalidInput);
	CHECK(IsNotStanding(AtUpper));
}
