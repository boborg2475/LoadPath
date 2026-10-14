#include "AnchorFixtures.h"
#include "BrickFixtures.h"
#include "CertificateTestSupport.h"
#include "StaticsTestSupport.h"

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double TipTiltRad = 80.0 * RadPerDegree;
constexpr double LoadFactorRelative = 1.0e-7;
constexpr double ColumnLambdaX = 1.5;

double BrickWeightN()
{
	return BrickMassKg * GravityMPerS2;
}

double BedAreaM2()
{
	return BrickLengthM * BrickWidthM;
}

StrengthProfile Bonded(double TensileStrengthPa, double ShearCohesionPa, double Friction)
{
	StrengthProfile Profile = DryJoint(Friction);
	Profile.TensileStrengthPa = TensileStrengthPa;
	Profile.ShearCohesionPa = ShearCohesionPa;
	return Profile;
}

StrengthProfile TiltBondFailingAt(double LoadFactor)
{
	const double TensilePa = LoadFactor * BrickWeightN() * ((BrickHeightM / BrickLengthM) * std::sin(TipTiltRad) - std::cos(TipTiltRad)) / BedAreaM2();
	return Bonded(TensilePa, 1.0e5, 0.7);
}

StrengthProfile CrushedAt(double LoadFactor)
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.CompressiveStrengthPa = LoadFactor * BrickWeightN() / BedAreaM2();
	return Profile;
}

double ColumnTensileStrengthPa(double LoadFactor)
{
	return LoadFactor * 3.0 * BrickWeightN() * (3.0 * ColumnLambdaX * BrickHeightM - BrickLengthM) / (BedAreaM2() * BrickLengthM);
}

struct OrderedColumn
{
	Structure Model;
	std::array<InterfaceId, 3> JointUnderBrick;
};

InterfaceDesc ColumnJoint(const std::array<BodyId, 3>& Bricks, BodyId Ground, int Course, ProfileId Profile)
{
	if (Course == 0)
	{
		return OnGroundPlane(Ground, Bricks[0], BrickShape(0.0, 0), Profile);
	}
	const std::size_t Upper = static_cast<std::size_t>(Course);
	return *StackedFaces(Bricks[Upper - 1], BrickShape(0.0, Course - 1), Bricks[Upper], BrickShape(0.0, Course), Profile);
}

OrderedColumn MakeOrderedColumn(const StrengthProfile& Profile, const std::array<int, 4>& BodyOrder, const std::array<int, 3>& JointOrder)
{
	OrderedColumn C;
	const ProfileId P = AddProfileOk(C.Model, Profile);
	BodyId Ground;
	std::array<BodyId, 3> Bricks{};
	for (const int Role : BodyOrder)
	{
		if (Role == 0)
		{
			Ground = AddGroundPlane(C.Model);
			continue;
		}
		Bricks[static_cast<std::size_t>(Role - 1)] = AddBox(C.Model, BrickShape(0.0, Role - 1));
	}
	for (const int Course : JointOrder)
	{
		C.JointUnderBrick[static_cast<std::size_t>(Course)] = AddInterfaceOk(C.Model, ColumnJoint(Bricks, Ground, Course, P));
	}
	return C;
}

struct InvalidCase
{
	const char* Name;
	void (*Arrange)(TwoBrickStack& S, std::vector<BodyLoad>& Loads, CertificateOptions& Options);
};

const InvalidCase InvalidCases[] = {
	{"load on a removed body", [](TwoBrickStack& S, std::vector<BodyLoad>& Loads, CertificateOptions&) {
		 S.Model.RemoveBody(S.Upper);
		 Loads.push_back(BodyLoad{S.Upper, Vec3{0.0, 0.0, -1.0}, BrickShape(0.0, 1).CentreM});
	 }},
	{"load on a never-issued body", [](TwoBrickStack&, std::vector<BodyLoad>& Loads, CertificateOptions&) { Loads.push_back(BodyLoad{BodyId{1000u, 1u}, Vec3{0.0, 0.0, -1.0}, Vec3{}}); }},
	{"NaN load force", [](TwoBrickStack& S, std::vector<BodyLoad>& Loads, CertificateOptions&) { Loads.push_back(BodyLoad{S.Upper, Vec3{0.0, 0.0, NaN}, BrickShape(0.0, 1).CentreM}); }},
	{"infinite load point", [](TwoBrickStack& S, std::vector<BodyLoad>& Loads, CertificateOptions&) { Loads.push_back(BodyLoad{S.Upper, Vec3{0.0, 0.0, -1.0}, Vec3{Infinity, 0.0, 0.0}}); }},
	{"NaN gravity", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.Gravity.Z = NaN; }},
	{"infinite gravity", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.Gravity.X = Infinity; }},
	{"two cone generators", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.ConeGenerators = 2; }},
	{"three cone generators", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.ConeGenerators = 3; }},
	{"five cone generators", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.ConeGenerators = 5; }},
	{"zero load factor cap", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.LoadFactorCap = 0.0; }},
	{"negative load factor cap", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.LoadFactorCap = -1.0; }},
	{"NaN load factor cap", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.LoadFactorCap = NaN; }},
	{"infinite load factor cap", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.LoadFactorCap = Infinity; }},
	{"zero feasibility tolerance", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.FeasibilityTolerance = 0.0; }},
	{"negative feasibility tolerance", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.FeasibilityTolerance = -1.0e-9; }},
	{"NaN feasibility tolerance", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.FeasibilityTolerance = NaN; }},
	{"infinite feasibility tolerance", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.FeasibilityTolerance = Infinity; }},
	{"zero iteration cap", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.MaxIterations = 0; }},
	{"negative iteration cap", [](TwoBrickStack&, std::vector<BodyLoad>&, CertificateOptions& Options) { Options.MaxIterations = -1; }},
};

struct VerdictFixture
{
	const char* Name;
	Structure (*Build)();
	CertificateOptions Options;
	CertificateState Expected;
};

double InsideTheFrictionBand(double Friction)
{
	return Friction * (1.0 + 0.5 * (CircumscribedOverInscribed(16) - 1.0));
}

std::vector<VerdictFixture> VerdictFixtures()
{
	return {
		{"dry brick under gravity only", [] { return MakeBrickOnGround(DryJoint(0.6)).Model; }, CertificateOptions{}, CertificateState::Stands},
		{"dry brick at lateral 0.3, friction 0.6", [] { return MakeBrickOnGround(DryJoint(0.6)).Model; }, CertificateLateral(0.3), CertificateState::Stands},
		{"dry brick inside the 16-gon band, friction 0.6", [] { return MakeBrickOnGround(DryJoint(0.6)).Model; }, CertificateLateral(InsideTheFrictionBand(0.6)), CertificateState::Undecided},
		{"dry brick at lateral 1, friction 0.6", [] { return MakeBrickOnGround(DryJoint(0.6)).Model; }, CertificateLateral(1.0), CertificateState::Collapses},
		{"dry brick at lateral 3.5, friction 5", [] { return MakeBrickOnGround(DryJoint(5.0)).Model; }, CertificateLateral(3.5), CertificateState::Collapses},
		{"bonded brick on an 80 degree tilt, load factor 2", [] { return MakeBrickOnGround(TiltBondFailingAt(2.0)).Model; }, CertificateTilt(TipTiltRad), CertificateState::Stands},
		{"bonded brick on an 80 degree tilt, load factor 0.5", [] { return MakeBrickOnGround(TiltBondFailingAt(0.5)).Model; }, CertificateTilt(TipTiltRad), CertificateState::Collapses},
		{"brick crushed at load factor 0.5", [] { return MakeBrickOnGround(CrushedAt(0.5)).Model; }, CertificateOptions{}, CertificateState::Collapses},
		{"two-brick stack past friction 0.6", [] { return MakeTwoBrickStack(DryJoint(0.6), 0.03).Model; }, CertificateLateral(0.625, 0.875), CertificateState::Collapses},
		{"three-brick bridge at friction 5, lambda (2, 0)", [] { return MakeThreeBrickBridge(DryJoint(5.0)).Model; }, CertificateLateral(2.0, 0.0), CertificateState::Stands},
	};
}

void CheckIdenticalVec3(const Vec3& A, const Vec3& B)
{
	CHECK(A.X == B.X);
	CHECK(A.Y == B.Y);
	CHECK(A.Z == B.Z);
}

void CheckIdenticalResults(const CertificateResult& A, const CertificateResult& B)
{
	CHECK(A.State == B.State);
	CHECK(A.LoadFactorLower == B.LoadFactorLower);
	CHECK(A.LoadFactorUpper == B.LoadFactorUpper);
	CHECK(A.Iterations == B.Iterations);
	CHECK(SameHandle(A.InvalidInterface, B.InvalidInterface));
	CHECK(Describe(A.UnconstrainedBodies) == Describe(B.UnconstrainedBodies));
	CHECK(Describe(A.HingingInterfaces) == Describe(B.HingingInterfaces));
	CHECK(A.Mechanism.size() == B.Mechanism.size());
	for (std::size_t M = 0; M < A.Mechanism.size() && M < B.Mechanism.size(); ++M)
	{
		INFO("mechanism entry " << M);
		CHECK(SameHandle(A.Mechanism[M].Body, B.Mechanism[M].Body));
		CheckIdenticalVec3(A.Mechanism[M].TranslationM, B.Mechanism[M].TranslationM);
		CheckIdenticalVec3(A.Mechanism[M].RotationRad, B.Mechanism[M].RotationRad);
	}
}
}

TEST_CASE("Certificate.Mechanics.InvalidInputIsReportedWithNoInterfaceNamed")
{
	for (const InvalidCase& Case : InvalidCases)
	{
		INFO("case: " << doctest::String(Case.Name));
		TwoBrickStack S = MakeTwoBrickStack(DryJoint(0.7), 0.0);
		std::vector<BodyLoad> Loads;
		CertificateOptions Options;
		Case.Arrange(S, Loads, Options);
		const CertificateResult Result = MakeLimitAnalysisCertificate()->Decide(S.Model, Loads, Options, {});
		INFO("state " << Result.State << ", named " << Result.InvalidInterface);
		CHECK(Result.State == CertificateState::InvalidInput);
		CHECK(SameHandle(Result.InvalidInterface, InterfaceId::Invalid()));
	}

	for (const int Generators : {4, 6, 32})
	{
		INFO("valid cone generators " << Generators);
		const TwoBrickStack S = MakeTwoBrickStack(DryJoint(0.7), 0.0);
		CertificateOptions Options;
		Options.ConeGenerators = Generators;
		const CertificateResult Result = DecideWith(S.Model, Options);
		INFO("state " << Result.State);
		CHECK(Result.State == CertificateState::Stands);
	}
}

TEST_CASE("Certificate.Mechanics.OnlyGroundedStructuresStandWithAnEmptyMechanism")
{
	const auto CheckStandsEmpty = [](const Structure& Model) {
		const CertificateResult Result = DecideWith(Model, CertificateOptions{});
		INFO("state " << Result.State);
		CHECK(Result.State == CertificateState::Stands);
		CHECK(Result.Mechanism.empty());
		CHECK(Result.UnconstrainedBodies.empty());
		CHECK(Result.HingingInterfaces.empty());
	};
	SUBCASE("empty structure")
	{
		CheckStandsEmpty(Structure{});
	}
	SUBCASE("two grounded bodies joined by an interface")
	{
		Structure Model;
		const ProfileId P = AddProfileOk(Model, DryJoint(0.7));
		const BodyId Lower = AddBox(Model, BrickShape(0.0, 0), true);
		const BodyId Upper = AddBox(Model, BrickShape(0.0, 1), true);
		AddInterfaceOk(Model, *StackedFaces(Lower, BrickShape(0.0, 0), Upper, BrickShape(0.0, 1), P));
		CheckStandsEmpty(Model);
	}
}

TEST_CASE("Certificate.Mechanics.RepeatedDecisionsAreBitIdentical")
{
	struct Case
	{
		const char* Name;
		Structure Model;
		CertificateOptions Options;
		CertificateState Expected;
	};
	const Case Cases[] = {
		{"bonded brick tipping on an 80 degree tilt", MakeBrickOnGround(TiltBondFailingAt(0.5)).Model, CertificateTilt(TipTiltRad), CertificateState::Collapses},
		{"dry three-brick column at lateral 0.5", MakeOrderedColumn(DryJoint(0.7), {0, 1, 2, 3}, {0, 1, 2}).Model, CertificateLateral(0.5), CertificateState::Stands},
	};
	for (const Case& Each : Cases)
	{
		INFO("case: " << doctest::String(Each.Name));
		const std::unique_ptr<ICollapseCertificate> Certificate = MakeLimitAnalysisCertificate();
		const CertificateResult First = Certificate->Decide(Each.Model, {}, Each.Options, {});
		const CertificateResult Second = Certificate->Decide(Each.Model, {}, Each.Options, {});
		const CertificateResult Fresh = MakeLimitAnalysisCertificate()->Decide(Each.Model, {}, Each.Options, {});
		INFO("state " << First.State << ", load factors [" << First.LoadFactorLower << ", " << First.LoadFactorUpper << "]");
		CHECK(First.State == Each.Expected);
		CheckIdenticalResults(First, Second);
		CheckIdenticalResults(First, Fresh);
	}
}

TEST_CASE("Certificate.Mechanics.InsertionOrderChangesNoVerdictOrLoadFactor")
{
	CHECK(3.0 * ColumnLambdaX * BrickHeightM > BrickLengthM);
	CHECK(2.0 * ColumnLambdaX * BrickHeightM < BrickLengthM);
	for (const double LoadFactor : {2.0, 0.5})
	{
		INFO("load factor " << LoadFactor);
		const StrengthProfile Profile = Bonded(ColumnTensileStrengthPa(LoadFactor), 1.0e5, 0.7);
		const OrderedColumn Natural = MakeOrderedColumn(Profile, {0, 1, 2, 3}, {0, 1, 2});
		const OrderedColumn Shuffled = MakeOrderedColumn(Profile, {3, 1, 0, 2}, {2, 0, 1});
		const CertificateOptions Options = CertificateLateral(ColumnLambdaX);
		const CertificateResult A = DecideWith(Natural.Model, Options);
		const CertificateResult B = DecideWith(Shuffled.Model, Options);
		INFO("natural " << A.State << " [" << A.LoadFactorLower << ", " << A.LoadFactorUpper << "], shuffled " << B.State << " [" << B.LoadFactorLower << ", " << B.LoadFactorUpper << "]");
		CHECK(A.State == (LoadFactor >= 1.0 ? CertificateState::Stands : CertificateState::Collapses));
		CHECK(A.State == B.State);
		CheckRelative(A.LoadFactorLower, LoadFactor, LoadFactorRelative, "natural lower load factor against the base-joint tension limit");
		CheckRelative(A.LoadFactorUpper, LoadFactor, LoadFactorRelative, "natural upper load factor against the base-joint tension limit");
		CheckRelative(B.LoadFactorLower, A.LoadFactorLower, 1.0e-9, "shuffled lower load factor");
		CheckRelative(B.LoadFactorUpper, A.LoadFactorUpper, 1.0e-9, "shuffled upper load factor");
		for (std::size_t Course = 0; Course < 3; ++Course)
		{
			INFO("joint under brick " << Course);
			CHECK(HasInterface(A.HingingInterfaces, Natural.JointUnderBrick[Course]) == HasInterface(B.HingingInterfaces, Shuffled.JointUnderBrick[Course]));
		}
	}
}

TEST_CASE("Certificate.Mechanics.LoadFactorBoundsAreOrderedAndAgreeWithTheState")
{
	const double BandLambda = InsideTheFrictionBand(0.6);
	CHECK(BandLambda > 0.6);
	CHECK(BandLambda < 0.6 * CircumscribedOverInscribed(16));
	for (const VerdictFixture& Each : VerdictFixtures())
	{
		INFO("case: " << doctest::String(Each.Name));
		const CertificateResult Result = DecideWith(Each.Build(), Each.Options);
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "] after " << Result.Iterations << " iterations");
		CHECK(Result.State == Each.Expected);
		CHECK(IsFinite(Result.LoadFactorLower));
		CHECK(IsFinite(Result.LoadFactorUpper));
		CHECK(Result.LoadFactorLower >= 0.0);
		CHECK(Result.LoadFactorLower <= Result.LoadFactorUpper);
		CHECK(Result.LoadFactorUpper <= Each.Options.LoadFactorCap * (1.0 + Each.Options.FeasibilityTolerance));
		CHECK((Result.State == CertificateState::Stands) == (Result.LoadFactorLower >= 1.0));
		CHECK((Result.State == CertificateState::Collapses) == (Result.LoadFactorUpper < 1.0));
	}
}

TEST_CASE("Certificate.Mechanics.MechanismListsAreEmptyOnStandsAndNamedOnCollapses")
{
	for (const VerdictFixture& Each : VerdictFixtures())
	{
		INFO("case: " << doctest::String(Each.Name));
		const Structure Model = Each.Build();
		const CertificateResult Result = DecideWith(Model, Each.Options);
		INFO("state " << Result.State << ", moving " << Describe(Result.UnconstrainedBodies) << ", hinging " << Describe(Result.HingingInterfaces));
		CHECK(Result.State == Each.Expected);
		if (Result.State == CertificateState::Stands)
		{
			CHECK(Result.Mechanism.empty());
			CHECK(Result.UnconstrainedBodies.empty());
			CHECK(Result.HingingInterfaces.empty());
		}
		if (Result.State == CertificateState::Collapses)
		{
			CHECK_FALSE(Result.Mechanism.empty());
			CHECK_FALSE(Result.UnconstrainedBodies.empty());
			CHECK_FALSE(Result.HingingInterfaces.empty());
			for (const BodyId& Body : Result.UnconstrainedBodies)
			{
				INFO("moving body " << Body);
				CHECK(FindMotion(Result, Body) != nullptr);
			}
			for (const InterfaceId& Id : Result.HingingInterfaces)
			{
				INFO("hinging interface " << Id);
				CHECK(Model.IsLive(Id));
			}
		}
	}
}

namespace
{
BodyId AddBrickOnGroundAt(Structure& Model, BodyId Ground, double CentreXM, ProfileId Profile, double MassKg)
{
	Box Shape = BrickShape(CentreXM, 0);
	Shape.MassKg = MassKg;
	const BodyId Brick = AddBox(Model, Shape);
	AddInterfaceOk(Model, OnGroundPlane(Ground, Brick, Shape, Profile));
	return Brick;
}
}

TEST_CASE("Certificate.Mechanics.ListsCollectEveryCollapsingComponentAndEdgeStatesAreRuled")
{
	const double SpacingM = 1.0;
	SUBCASE("a collapsing component beside one in the polygon band reads Undecided and is still listed")
	{
		const double Lambda = 0.905;
		CHECK(Lambda > 0.6 * CircumscribedOverInscribed(16));
		CHECK(Lambda > 0.9);
		CHECK(Lambda < 0.9 * CircumscribedOverInscribed(16));
		CHECK(Lambda * BrickHeightM < BrickLengthM);
		Structure Model;
		const ProfileId Slippery = AddProfileOk(Model, DryJoint(0.6));
		const ProfileId Grippy = AddProfileOk(Model, DryJoint(0.9));
		const BodyId Ground = AddGroundPlane(Model);
		const BodyId Sliding = AddBrickOnGroundAt(Model, Ground, 0.0, Slippery, BrickMassKg);
		AddBrickOnGroundAt(Model, Ground, SpacingM, Grippy, BrickMassKg);
		const CertificateResult Result = DecideWith(Model, CertificateLateral(Lambda));
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "], moving " << Describe(Result.UnconstrainedBodies));
		CHECK(Result.State == CertificateState::Undecided);
		CHECK(Result.LoadFactorUpper < 1.0);
		CHECK(Result.UnconstrainedBodies.size() == 1u);
		CHECK(HasBody(Result.UnconstrainedBodies, Sliding));
	}
	SUBCASE("a massless free body with no load stands")
	{
		Structure Model;
		const ProfileId P = AddProfileOk(Model, DryJoint(0.6));
		const BodyId Ground = AddGroundPlane(Model);
		AddBrickOnGroundAt(Model, Ground, 0.0, P, BrickMassKg);
		AddBrickOnGroundAt(Model, Ground, SpacingM, P, 0.0);
		const CertificateResult Result = DecideWith(Model, CertificateOptions{});
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
		CHECK(Result.State == CertificateState::Stands);
		CHECK(Result.UnconstrainedBodies.empty());
	}
	SUBCASE("a free body with no interface collapses at zero and is listed")
	{
		Structure Model;
		const ProfileId P = AddProfileOk(Model, DryJoint(0.6));
		const BodyId Ground = AddGroundPlane(Model);
		AddBrickOnGroundAt(Model, Ground, 0.0, P, BrickMassKg);
		const BodyId Floating = AddBox(Model, BrickShape(SpacingM, 0));
		const CertificateResult Result = DecideWith(Model, CertificateOptions{});
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "], moving " << Describe(Result.UnconstrainedBodies));
		CHECK(Result.State == CertificateState::Collapses);
		CHECK(std::abs(Result.LoadFactorLower) <= 1.0e-6);
		CHECK(std::abs(Result.LoadFactorUpper) <= 1.0e-6);
		CHECK(Result.UnconstrainedBodies.size() == 1u);
		CHECK(HasBody(Result.UnconstrainedBodies, Floating));
	}
	SUBCASE("a collapsing component beside a standing one reads Collapses")
	{
		const double Lambda = 0.7;
		CHECK(Lambda > 0.6 * CircumscribedOverInscribed(16));
		CHECK(Lambda < 0.9 / CircumscribedOverInscribed(16));
		Structure Model;
		const ProfileId Slippery = AddProfileOk(Model, DryJoint(0.6));
		const ProfileId Grippy = AddProfileOk(Model, DryJoint(0.9));
		const BodyId Ground = AddGroundPlane(Model);
		const BodyId Sliding = AddBrickOnGroundAt(Model, Ground, 0.0, Slippery, BrickMassKg);
		AddBrickOnGroundAt(Model, Ground, SpacingM, Grippy, BrickMassKg);
		const CertificateResult Result = DecideWith(Model, CertificateLateral(Lambda));
		INFO("state " << Result.State << ", moving " << Describe(Result.UnconstrainedBodies));
		CHECK(Result.State == CertificateState::Collapses);
		CHECK(Result.UnconstrainedBodies.size() == 1u);
		CHECK(HasBody(Result.UnconstrainedBodies, Sliding));
	}
}

TEST_CASE("Certificate.Mechanics.StandsSurvivesAnOuterSolveThatDoesNotConverge")
{
	const double TiltRad = 20.0 * RadPerDegree;
	const BrickOnGround F = MakeBrickOnGround(Bonded(1000.0, 200.0, 0.3));
	const CertificateResult Full = DecideWith(F.Model, CertificateTilt(TiltRad));
	const int IterationLimit = 10;
	INFO("unlimited: state " << Full.State << ", load factors [" << Full.LoadFactorLower << ", " << Full.LoadFactorUpper << "], iterations " << Full.Iterations);
	REQUIRE(Full.State == CertificateState::Stands);
	REQUIRE(Full.LoadFactorUpper < CertificateOptions{}.LoadFactorCap);
	REQUIRE(Full.Iterations > IterationLimit);
	CertificateOptions Limited = CertificateTilt(TiltRad);
	Limited.MaxIterations = IterationLimit;
	const CertificateResult Result = DecideWith(F.Model, Limited);
	INFO("limited to " << IterationLimit << ": state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
	CHECK(Result.State == CertificateState::Stands);
	CHECK(Result.LoadFactorLower >= 1.0);
	CHECK(std::abs(Result.LoadFactorLower - Full.LoadFactorLower) <= LoadFactorRelative * Full.LoadFactorLower);
	CHECK(Result.LoadFactorUpper < Limited.LoadFactorCap);
	CHECK(Result.LoadFactorUpper >= Full.LoadFactorUpper);
	CHECK(Result.LoadFactorUpper <= Full.LoadFactorUpper * (1.0 + 1.0e-6));
	CHECK(std::abs(Full.LoadFactorUpper - 7.71542823) <= 1.0e-8 * 7.71542823);
	CHECK(Result.Mechanism.empty());
}

TEST_CASE("Certificate.Mechanics.SimultaneousKnifeEdgeNamesTheTopBlockAndNeverTheAnalyticCentre")
{
	const Stack S = BuildOnTable(HarmonicStack(2, 1.0 + 1.0e-6), 0.0);
	const BodyId Top = S.Blocks.front();
	const CertificateResult Result = DecideWith(S.Model, CertificateOptions{});
	INFO("state " << Result.State << ", moving " << Describe(Result.UnconstrainedBodies) << ", hinging " << Describe(Result.HingingInterfaces));
	CHECK((Result.State == CertificateState::Collapses || Result.State == CertificateState::Undecided));
	if (Result.State == CertificateState::Collapses)
	{
		CHECK(HasBody(Result.UnconstrainedBodies, Top));
		CHECK(Result.UnconstrainedBodies.size() <= S.Blocks.size());
		CHECK(Result.HingingInterfaces.size() <= S.Blocks.size());
		CHECK(Result.HingingInterfaces.size() >= 1u);
		const MechanismMotion* TopMotion = FindMotion(Result, Top);
		const MechanismMotion* LowerMotion = FindMotion(Result, S.Blocks.back());
		REQUIRE(TopMotion != nullptr);
		const double LowerRotation = LowerMotion != nullptr ? Length(LowerMotion->RotationRad) : 0.0;
		INFO("top rotation " << TopMotion->RotationRad << ", lower rotation size " << LowerRotation);
		CHECK(LowerRotation <= 1.0e-2 * Length(TopMotion->RotationRad));
	}
	else
	{
		CHECK(Result.UnconstrainedBodies.empty());
		CHECK(Result.HingingInterfaces.empty());
		CHECK(Result.Mechanism.empty());
	}
	for (int Count = 2; Count <= 4; ++Count)
	{
		const Stack Wider = BuildOnTable(HarmonicStack(Count, 1.0 + 1.0e-4), 0.0);
		const CertificateResult Each = DecideWith(Wider.Model, CertificateOptions{});
		INFO("n = " << Count << ", overhang scale 1 + 1e-4: state " << Each.State << ", moving " << Describe(Each.UnconstrainedBodies) << ", hinging " << Describe(Each.HingingInterfaces));
		CHECK(Each.State == CertificateState::Collapses);
		CHECK(Each.UnconstrainedBodies.size() == 1u);
		CHECK(HasBody(Each.UnconstrainedBodies, Wider.Blocks.front()));
		CHECK(Each.HingingInterfaces.size() == 1u);
		const std::vector<InterfaceId> UnderTop = Wider.Model.InterfacesOf(Wider.Blocks.front());
		CHECK(UnderTop.size() == 1u);
		CHECK((!UnderTop.empty() && HasInterface(Each.HingingInterfaces, UnderTop.front())));
	}
}
