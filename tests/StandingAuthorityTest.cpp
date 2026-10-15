#include "StaticsTestSupport.h"

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
Box BrickBox(int Course)
{
	return Box{StackedBrickCentroidM(Course), Vec3{BrickLengthM / 2.0, BrickWidthM / 2.0, BrickHeightM / 2.0}, BrickMassKg};
}

double BrickWeightN()
{
	return BrickMassKg * GravityMPerS2;
}

StrengthProfile WithStiffness(double NormalPaPerM, double TangentialPaPerM)
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.NormalStiffnessPaPerM = NormalPaPerM;
	Profile.TangentialStiffnessPaPerM = TangentialPaPerM;
	return Profile;
}

InterfaceDesc JointUnder(const std::vector<BodyId>& Bricks, BodyId Ground, int Course, ProfileId Profile)
{
	if (Course == 0)
	{
		return OnGroundPlane(Ground, Bricks[0], BrickBox(0), Profile);
	}
	return *StackedFaces(Bricks[static_cast<std::size_t>(Course - 1)], BrickBox(Course - 1), Bricks[static_cast<std::size_t>(Course)], BrickBox(Course), Profile);
}

struct BrickStack
{
	Structure Model;
	BodyId Ground;
	std::vector<BodyId> Bricks;
	std::vector<InterfaceId> Joints;
};

BrickStack MakeBrickStack(int Count, const StrengthProfile& Profile)
{
	BrickStack S;
	const ProfileId P = AddProfileOk(S.Model, Profile);
	S.Ground = AddGroundPlane(S.Model);
	for (int Course = 0; Course < Count; ++Course)
	{
		S.Bricks.push_back(AddBox(S.Model, BrickBox(Course)));
	}
	for (int Course = 0; Course < Count; ++Course)
	{
		S.Joints.push_back(AddInterfaceOk(S.Model, JointUnder(S.Bricks, S.Ground, Course, P)));
	}
	return S;
}

struct InvalidCase
{
	const char* Name;
	StrengthProfile Profile;
	void (*Arrange)(BrickStack& S, std::vector<BodyLoad>& Loads, StandingOptions& Options, InterfaceId& Expected);
};

const InvalidCase InvalidCases[] = {
	{"zero normal stiffness", WithStiffness(0.0, 1.0e10), [](BrickStack& S, std::vector<BodyLoad>&, StandingOptions&, InterfaceId& Expected) { Expected = S.Joints[0]; }},
	{"zero tangential stiffness", WithStiffness(1.0e10, 0.0), [](BrickStack& S, std::vector<BodyLoad>&, StandingOptions&, InterfaceId& Expected) { Expected = S.Joints[0]; }},
	{"negative normal stiffness", WithStiffness(-1.0e10, 1.0e10), [](BrickStack& S, std::vector<BodyLoad>&, StandingOptions&, InterfaceId& Expected) { Expected = S.Joints[0]; }},
	{"negative tangential stiffness", WithStiffness(1.0e10, -1.0e10), [](BrickStack& S, std::vector<BodyLoad>&, StandingOptions&, InterfaceId& Expected) { Expected = S.Joints[0]; }},
	{"load on a wrong-generation handle", DryJoint(0.7), [](BrickStack& S, std::vector<BodyLoad>& Loads, StandingOptions&, InterfaceId&) {
		 Loads.push_back(BodyLoad{BodyId{S.Bricks[1].Index, S.Bricks[1].Generation + 1u}, Vec3{0.0, 0.0, -1.0}, StackedBrickCentroidM(1)});
	 }},
	{"load on a never-issued body", DryJoint(0.7), [](BrickStack&, std::vector<BodyLoad>& Loads, StandingOptions&, InterfaceId&) {
		 Loads.push_back(BodyLoad{BodyId{1000u, 1u}, Vec3{0.0, 0.0, -1.0}, Vec3{}});
	 }},
	{"NaN load force", DryJoint(0.7), [](BrickStack& S, std::vector<BodyLoad>& Loads, StandingOptions&, InterfaceId&) {
		 Loads.push_back(BodyLoad{S.Bricks[1], Vec3{0.0, 0.0, NaN}, StackedBrickCentroidM(1)});
	 }},
	{"infinite load point", DryJoint(0.7), [](BrickStack& S, std::vector<BodyLoad>& Loads, StandingOptions&, InterfaceId&) {
		 Loads.push_back(BodyLoad{S.Bricks[1], Vec3{0.0, 0.0, -1.0}, Vec3{Infinity, 0.0, 0.0}});
	 }},
	{"NaN gravity", DryJoint(0.7), [](BrickStack&, std::vector<BodyLoad>&, StandingOptions& Options, InterfaceId&) { Options.Gravity.Z = NaN; }},
	{"infinite gravity", DryJoint(0.7), [](BrickStack&, std::vector<BodyLoad>&, StandingOptions& Options, InterfaceId&) { Options.Gravity.X = Infinity; }},
};

void CheckIdenticalVec3(const Vec3& A, const Vec3& B)
{
	CHECK(A.X == B.X);
	CHECK(A.Y == B.Y);
	CHECK(A.Z == B.Z);
}

void CheckCloseVec3(const Vec3& A, const Vec3& B)
{
	CHECK(A.X == doctest::Approx(B.X).epsilon(1e-12));
	CHECK(A.Y == doctest::Approx(B.Y).epsilon(1e-12));
	CHECK(A.Z == doctest::Approx(B.Z).epsilon(1e-12));
}

void CheckIdenticalResults(const StandingResult& A, const StandingResult& B)
{
	CHECK(A.State == B.State);
	CHECK(A.ActiveSetIterations == B.ActiveSetIterations);
	CHECK(SameHandle(A.InvalidInterface, B.InvalidInterface));
	CHECK(A.Readings.size() == B.Readings.size());
	CHECK(A.Mechanism.size() == B.Mechanism.size());
	CHECK(DescribeAscending(A.UnconstrainedBodies) == DescribeAscending(B.UnconstrainedBodies));
	for (std::size_t R = 0; R < A.Readings.size() && R < B.Readings.size(); ++R)
	{
		const InterfaceReading& RA = A.Readings[R];
		const InterfaceReading& RB = B.Readings[R];
		INFO("reading " << R);
		CHECK(SameHandle(RA.Id, RB.Id));
		CHECK(RA.Utilisation == RB.Utilisation);
		CHECK(RA.Axis == RB.Axis);
		CHECK(RA.NormalForceN == RB.NormalForceN);
		CheckIdenticalVec3(RA.ShearForceN, RB.ShearForceN);
		CheckIdenticalVec3(RA.MomentNm, RB.MomentNm);
		for (std::size_t C = 0; C < 4; ++C)
		{
			INFO("corner " << C);
			CheckIdenticalVec3(RA.Corners[C].PointM, RB.Corners[C].PointM);
			CHECK(RA.Corners[C].NormalForceN == RB.Corners[C].NormalForceN);
			CheckIdenticalVec3(RA.Corners[C].ShearForceN, RB.Corners[C].ShearForceN);
			CHECK(RA.Corners[C].bOpen == RB.Corners[C].bOpen);
			CHECK(RA.Corners[C].bSlipping == RB.Corners[C].bSlipping);
		}
	}
	for (std::size_t M = 0; M < A.Mechanism.size() && M < B.Mechanism.size(); ++M)
	{
		CHECK(SameHandle(A.Mechanism[M].Body, B.Mechanism[M].Body));
		CheckIdenticalVec3(A.Mechanism[M].TranslationM, B.Mechanism[M].TranslationM);
		CheckIdenticalVec3(A.Mechanism[M].RotationRad, B.Mechanism[M].RotationRad);
	}
}

struct OrderedStack
{
	Structure Model;
	std::array<InterfaceId, 3> JointUnderBrick;
};

OrderedStack MakeOrderedStack(const std::array<int, 4>& BodyOrder, const std::array<int, 3>& JointOrder)
{
	OrderedStack S;
	const ProfileId P = AddProfileOk(S.Model, DryJoint(0.7));
	std::array<BodyId, 4> ByRole{};
	for (const int Role : BodyOrder)
	{
		ByRole[static_cast<std::size_t>(Role)] = Role == 0 ? AddGroundPlane(S.Model) : AddBox(S.Model, BrickBox(Role - 1));
	}
	const std::vector<BodyId> Bricks{ByRole[1], ByRole[2], ByRole[3]};
	for (const int Course : JointOrder)
	{
		S.JointUnderBrick[static_cast<std::size_t>(Course)] = AddInterfaceOk(S.Model, JointUnder(Bricks, ByRole[0], Course, P));
	}
	return S;
}
}

TEST_CASE("Statics.Authority.OnlyGroundedBodiesStandWithNoReadings")
{
	SUBCASE("empty structure")
	{
		const Structure Model{};
		const StandingResult Result = SolveWith(Model, StandingOptions{});
		CHECK(Result.State == StandingState::Stands);
		CHECK(Result.Readings.empty());
		CHECK(Result.Mechanism.empty());
	}
	SUBCASE("two grounded bodies")
	{
		Structure Model;
		AddBox(Model, BrickBox(0), true);
		AddBox(Model, BrickBox(1), true);
		const StandingResult Result = SolveWith(Model, StandingOptions{});
		CHECK(Result.State == StandingState::Stands);
		CHECK(Result.Readings.empty());
		CHECK(Result.Mechanism.empty());
	}
}

TEST_CASE("Statics.Authority.InvalidInputIsReportedAndNamesTheOffendingInterface")
{
	for (const InvalidCase& Case : InvalidCases)
	{
		INFO("case: " << doctest::String(Case.Name));
		BrickStack S = MakeBrickStack(2, Case.Profile);
		std::vector<BodyLoad> Loads;
		StandingOptions Options;
		InterfaceId Expected = InterfaceId::Invalid();
		Case.Arrange(S, Loads, Options, Expected);
		const StandingResult Result = MakeRbsmStandingAuthority()->Solve(S.Model, Loads, Options, {});
		CHECK(Result.State == StandingState::InvalidInput);
		INFO("named " << Result.InvalidInterface << ", expected " << Expected);
		CHECK(SameHandle(Result.InvalidInterface, Expected));
	}
}

TEST_CASE("Statics.Authority.TheFirstInvalidInterfaceInSlotOrderIsNamed")
{
	Structure Model;
	const ProfileId Good = AddProfileOk(Model, DryJoint(0.7));
	const ProfileId NoNormal = AddProfileOk(Model, WithStiffness(0.0, 1.0e10));
	const ProfileId NoTangential = AddProfileOk(Model, WithStiffness(1.0e10, 0.0));
	const BodyId Ground = AddGroundPlane(Model);
	const std::vector<BodyId> Bricks{AddBox(Model, BrickBox(0)), AddBox(Model, BrickBox(1)), AddBox(Model, BrickBox(2))};
	AddInterfaceOk(Model, JointUnder(Bricks, Ground, 0, Good));
	const InterfaceId FirstBad = AddInterfaceOk(Model, JointUnder(Bricks, Ground, 1, NoTangential));
	AddInterfaceOk(Model, JointUnder(Bricks, Ground, 2, NoNormal));
	const StandingResult Result = SolveWith(Model, StandingOptions{});
	CHECK(Result.State == StandingState::InvalidInput);
	INFO("named " << Result.InvalidInterface << ", expected " << FirstBad);
	CHECK(SameHandle(Result.InvalidInterface, FirstBad));
}

TEST_CASE("Statics.Authority.ZeroStiffnessOnARemovedInterfaceIsIgnored")
{
	Structure Model;
	const ProfileId Good = AddProfileOk(Model, DryJoint(0.7));
	const ProfileId NoNormal = AddProfileOk(Model, WithStiffness(0.0, 1.0e10));
	const BodyId Ground = AddGroundPlane(Model);
	const std::vector<BodyId> Bricks{AddBox(Model, BrickBox(0)), AddBox(Model, BrickBox(1))};
	AddInterfaceOk(Model, JointUnder(Bricks, Ground, 0, Good));
	AddInterfaceOk(Model, JointUnder(Bricks, Ground, 1, NoNormal));
	Model.RemoveBody(Bricks[1]);
	const StandingResult Result = SolveWith(Model, StandingOptions{});
	CHECK(Result.State == StandingState::Stands);
}

TEST_CASE("Statics.Authority.ReadingsCoverEveryLiveInterfaceOnceInAscendingSlotOrder")
{
	Structure Model;
	const ProfileId P = AddProfileOk(Model, DryJoint(0.7));
	const BodyId Ground = AddGroundPlane(Model);
	std::vector<BodyId> Bricks;
	for (int Course = 0; Course < 4; ++Course)
	{
		Bricks.push_back(AddBox(Model, BrickBox(Course)));
	}
	const InterfaceId UnderTop = AddInterfaceOk(Model, JointUnder(Bricks, Ground, 3, P));
	AddInterfaceOk(Model, JointUnder(Bricks, Ground, 0, P));
	AddInterfaceOk(Model, JointUnder(Bricks, Ground, 2, P));
	AddInterfaceOk(Model, JointUnder(Bricks, Ground, 1, P));
	Model.RemoveBody(Bricks[3]);

	const StandingResult Result = SolveWith(Model, StandingOptions{});
	const std::vector<InterfaceId> Live = Model.LiveInterfaces();
	std::vector<InterfaceId> Read;
	for (const InterfaceReading& Reading : Result.Readings)
	{
		Read.push_back(Reading.Id);
	}
	CHECK(Live.size() == 3u);
	CHECK(Describe(Read) == Describe(Live));
	CHECK(FindReading(Result, UnderTop) == nullptr);
}

TEST_CASE("Statics.Authority.RepeatedSolvesAreBitIdentical")
{
	const BrickStack S = MakeBrickStack(3, DryJoint(0.7));
	const StandingOptions Options = LateralLoad(0.5);
	const std::unique_ptr<IStandingAuthority> Authority = MakeRbsmStandingAuthority();
	const StandingResult First = Authority->Solve(S.Model, {}, Options, {});
	const StandingResult Second = Authority->Solve(S.Model, {}, Options, {});
	const StandingResult Fresh = MakeRbsmStandingAuthority()->Solve(S.Model, {}, Options, {});
	CHECK(First.State == StandingState::Stands);
	CHECK(First.Readings.size() == 3u);
	CheckIdenticalResults(First, Second);
	CheckIdenticalResults(First, Fresh);
}

TEST_CASE("Statics.Authority.InsertionOrderChangesNoReading")
{
	const OrderedStack Natural = MakeOrderedStack({0, 1, 2, 3}, {0, 1, 2});
	const OrderedStack Shuffled = MakeOrderedStack({3, 1, 0, 2}, {2, 0, 1});
	const StandingOptions Options = LateralLoad(0.5);
	const StandingResult A = SolveWith(Natural.Model, Options);
	const StandingResult B = SolveWith(Shuffled.Model, Options);
	CHECK(A.State == StandingState::Stands);
	CHECK(B.State == StandingState::Stands);
	for (std::size_t Course = 0; Course < 3; ++Course)
	{
		INFO("joint under brick " << Course);
		const InterfaceReading* RA = FindReading(A, Natural.JointUnderBrick[Course]);
		const InterfaceReading* RB = FindReading(B, Shuffled.JointUnderBrick[Course]);
		CHECK(RA != nullptr);
		CHECK(RB != nullptr);
		if (RA == nullptr || RB == nullptr)
		{
			continue;
		}
		CHECK(RA->Utilisation == doctest::Approx(RB->Utilisation).epsilon(1e-12));
		CHECK(RA->NormalForceN == doctest::Approx(RB->NormalForceN).epsilon(1e-12));
		for (std::size_t C = 0; C < 4; ++C)
		{
			INFO("corner " << C);
			CHECK(RA->Corners[C].NormalForceN == doctest::Approx(RB->Corners[C].NormalForceN).epsilon(1e-12));
			CheckCloseVec3(RA->Corners[C].ShearForceN, RB->Corners[C].ShearForceN);
		}
	}
}

TEST_CASE("Statics.Authority.BrickOnTheGroundCarriesItsWeightEquallyAtFourCorners")
{
	const StrengthProfile Profile = DryJoint(0.7);
	const BrickStack S = MakeBrickStack(1, Profile);
	const StandingResult Result = SolveWith(S.Model, StandingOptions{});
	CHECK(Result.State == StandingState::Stands);
	const InterfaceReading* Bed = FindReading(Result, S.Joints[0]);
	CHECK(Bed != nullptr);
	if (Bed == nullptr)
	{
		return;
	}
	const double WeightN = BrickWeightN();
	const double AreaM2 = BrickLengthM * BrickWidthM;
	CheckRelative(Bed->NormalForceN, WeightN, 1.0e-9, "normal force");
	CHECK(std::abs(Bed->ShearForceN.X) <= 1.0e-9 * WeightN);
	CHECK(std::abs(Bed->ShearForceN.Y) <= 1.0e-9 * WeightN);
	CHECK(std::abs(Bed->ShearForceN.Z) <= 1.0e-9 * WeightN);
	CHECK(std::abs(Bed->MomentNm.X) <= 1.0e-9 * WeightN * BrickLengthM);
	CHECK(std::abs(Bed->MomentNm.Y) <= 1.0e-9 * WeightN * BrickLengthM);
	CHECK(std::abs(Bed->MomentNm.Z) <= 1.0e-9 * WeightN * BrickLengthM);
	const double HalfU = BrickLengthM / 2.0;
	const double HalfV = BrickWidthM / 2.0;
	const std::array<Vec3, 4> CornerPoints{Vec3{HalfU, HalfV, 0.0}, Vec3{-HalfU, HalfV, 0.0}, Vec3{-HalfU, -HalfV, 0.0}, Vec3{HalfU, -HalfV, 0.0}};
	for (std::size_t C = 0; C < 4; ++C)
	{
		INFO("corner " << C);
		const CornerReading& Corner = Bed->Corners[C];
		CHECK(Length(Minus(Corner.PointM, CornerPoints[C])) <= 1.0e-12);
		CheckRelative(Corner.NormalForceN, WeightN / 4.0, 1.0e-9, "corner normal force");
		CHECK_FALSE(Corner.bOpen);
		CHECK_FALSE(Corner.bSlipping);
	}
	CheckRelative(Bed->Utilisation, (WeightN / 4.0) / (Profile.CompressiveStrengthPa * AreaM2 / 4.0), 1.0e-9, "utilisation");
	CHECK(Bed->Axis == GoverningAxis::Compression);
}

TEST_CASE("Statics.Authority.CentredPointLoadAddsToTheNormalForce")
{
	const BrickStack S = MakeBrickStack(1, DryJoint(0.7));
	const double WeightN = BrickWeightN();
	const double LoadN = 50.0;
	const std::array<BodyLoad, 1> Loads{BodyLoad{S.Bricks[0], Vec3{0.0, 0.0, -LoadN}, Vec3{0.0, 0.0, BrickHeightM}}};
	const StandingResult Result = SolveWith(S.Model, StandingOptions{}, Loads);
	CHECK(Result.State == StandingState::Stands);
	const InterfaceReading* Bed = FindReading(Result, S.Joints[0]);
	CHECK(Bed != nullptr);
	if (Bed == nullptr)
	{
		return;
	}
	CheckRelative(Bed->NormalForceN, WeightN + LoadN, 1.0e-9, "normal force");
}

TEST_CASE("Statics.Authority.OffsetPointLoadMakesTheMomentAndLoadsThePlusUCorners")
{
	const BrickStack S = MakeBrickStack(1, DryJoint(0.7));
	const double WeightN = BrickWeightN();
	const double LoadN = 50.0;
	const double HalfU = BrickLengthM / 2.0;
	const double OffsetM = BrickLengthM / 4.0;
	const std::array<BodyLoad, 1> Loads{BodyLoad{S.Bricks[0], Vec3{0.0, 0.0, -LoadN}, Vec3{OffsetM, 0.0, BrickHeightM}}};
	const StandingResult Result = SolveWith(S.Model, StandingOptions{}, Loads);
	CHECK(Result.State == StandingState::Stands);
	const InterfaceReading* Bed = FindReading(Result, S.Joints[0]);
	CHECK(Bed != nullptr);
	if (Bed == nullptr)
	{
		return;
	}
	CheckRelative(Bed->NormalForceN, WeightN + LoadN, 1.0e-9, "normal force");
	CheckRelative(Bed->MomentNm.Y, -LoadN * OffsetM, 1.0e-9, "moment about V");
	CHECK(std::abs(Bed->MomentNm.X) <= 1.0e-9 * LoadN * OffsetM);
	CHECK(std::abs(Bed->MomentNm.Z) <= 1.0e-9 * LoadN * OffsetM);
	const double PlusUCornerN = (WeightN + LoadN) / 4.0 + LoadN * OffsetM / (4.0 * HalfU);
	const double MinusUCornerN = (WeightN + LoadN) / 4.0 - LoadN * OffsetM / (4.0 * HalfU);
	CheckRelative(Bed->Corners[0].NormalForceN, PlusUCornerN, 1.0e-9, "corner (+u,+v)");
	CheckRelative(Bed->Corners[3].NormalForceN, PlusUCornerN, 1.0e-9, "corner (+u,-v)");
	CheckRelative(Bed->Corners[1].NormalForceN, MinusUCornerN, 1.0e-9, "corner (-u,+v)");
	CheckRelative(Bed->Corners[2].NormalForceN, MinusUCornerN, 1.0e-9, "corner (-u,-v)");
	CHECK(Bed->Corners[0].NormalForceN > Bed->Corners[1].NormalForceN);
	CHECK(Bed->Corners[3].NormalForceN > Bed->Corners[2].NormalForceN);
}
