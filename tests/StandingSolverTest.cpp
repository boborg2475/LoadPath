#include "StaticsTestSupport.h"

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr std::array<std::size_t, 2> DownhillCorners{0, 3};
constexpr std::array<std::size_t, 2> UphillCorners{1, 2};
constexpr std::array<double, 4> CornerSignU{1.0, -1.0, -1.0, 1.0};
constexpr std::array<double, 4> CornerSignV{1.0, 1.0, -1.0, -1.0};
constexpr double ShapeTolerance = 1.0e-6;

Box BrickShape(double CentreXM = 0.0, int Course = 0, double Scale = 1.0)
{
	return Box{Vec3{Scale * CentreXM, 0.0, Scale * ((Course + 0.5) * BrickHeightM)}, Vec3{Scale * (BrickLengthM / 2.0), Scale * (BrickWidthM / 2.0), Scale * (BrickHeightM / 2.0)}, Scale * Scale * Scale * BrickMassKg};
}

double BrickWeightN()
{
	return BrickMassKg * GravityMPerS2;
}

double BedAreaM2()
{
	return BrickLengthM * BrickWidthM;
}

double HeightOverLength()
{
	return BrickHeightM / BrickLengthM;
}

double CornerCapacityN(double StrengthPa)
{
	return StrengthPa * BedAreaM2() / 4.0;
}

double DownhillCornerNormalN(double TiltRad)
{
	return BrickWeightN() / 4.0 * (std::cos(TiltRad) + HeightOverLength() * std::sin(TiltRad));
}

double UphillCornerNormalN(double TiltRad)
{
	return BrickWeightN() / 4.0 * (std::cos(TiltRad) - HeightOverLength() * std::sin(TiltRad));
}

double CornerShareOfSlopeShearN(double TiltRad)
{
	return BrickWeightN() * std::sin(TiltRad) / 4.0;
}

struct BrickOnGround
{
	Structure Model;
	BodyId Brick;
	InterfaceId Bed;
	ProfileId Profile;
};

BrickOnGround MakeBrickOnGround(const StrengthProfile& Profile, double Scale = 1.0)
{
	BrickOnGround F;
	F.Profile = AddProfileOk(F.Model, Profile);
	const BodyId Ground = AddGroundPlane(F.Model);
	F.Brick = AddBox(F.Model, BrickShape(0.0, 0, Scale));
	F.Bed = AddInterfaceOk(F.Model, OnGroundPlane(Ground, F.Brick, BrickShape(0.0, 0, Scale), F.Profile));
	return F;
}

BrickOnGround MakeBrickAgainstWall(const StrengthProfile& Profile)
{
	BrickOnGround F = MakeBrickOnGround(Profile);
	const Box WallShape{Vec3{BrickLengthM / 2.0 + 0.05, 0.0, 0.5}, Vec3{0.05, 0.5, 0.5}, 0.0};
	const BodyId Wall = AddBox(F.Model, WallShape, true);
	AddInterfaceOk(F.Model, HeadJointAlongX(F.Brick, Wall, 0, F.Profile));
	return F;
}

StrengthProfile Bonded(double TensileStrengthPa, double ShearCohesionPa, double Friction)
{
	StrengthProfile Profile = DryJoint(Friction);
	Profile.TensileStrengthPa = TensileStrengthPa;
	Profile.ShearCohesionPa = ShearCohesionPa;
	return Profile;
}

StandingOptions BiaxialLateralLoad(double LambdaX, double LambdaY)
{
	StandingOptions Options;
	Options.Gravity = Vec3{LambdaX * GravityMPerS2, LambdaY * GravityMPerS2, -GravityMPerS2};
	return Options;
}

StandingOptions WithIterationCap(StandingOptions Options, int Cap)
{
	Options.MaxActiveSetIterations = Cap;
	return Options;
}

const InterfaceReading* RequireReading(const StandingResult& Result, InterfaceId Id)
{
	const InterfaceReading* Reading = FindReading(Result, Id);
	CHECK(Reading != nullptr);
	return Reading;
}

void CheckSlopeNormals(const InterfaceReading& Bed, double TiltRad)
{
	for (const std::size_t C : DownhillCorners)
	{
		INFO("downhill corner " << C);
		CheckRelative(Bed.Corners[C].NormalForceN, DownhillCornerNormalN(TiltRad), 1.0e-9, "normal force");
	}
	for (const std::size_t C : UphillCorners)
	{
		INFO("uphill corner " << C);
		CheckRelative(Bed.Corners[C].NormalForceN, UphillCornerNormalN(TiltRad), 1.0e-9, "normal force");
	}
}

void CheckBondedSlopeCorners(const InterfaceReading& Bed, double TiltRad)
{
	CheckSlopeNormals(Bed, TiltRad);
	for (std::size_t C = 0; C < 4; ++C)
	{
		INFO("corner " << C);
		CHECK_FALSE(Bed.Corners[C].bOpen);
		CHECK_FALSE(Bed.Corners[C].bSlipping);
		CheckRelative(Length(Bed.Corners[C].ShearForceN), CornerShareOfSlopeShearN(TiltRad), 1.0e-9, "shear force");
	}
}

struct TwoBrickStack
{
	Structure Model;
	BodyId Lower;
	BodyId Upper;
	InterfaceId Bed;
	InterfaceId Joint;
};

TwoBrickStack MakeTwoBrickStack(const StrengthProfile& Profile, double ShiftXM, double Scale = 1.0)
{
	TwoBrickStack S;
	const ProfileId P = AddProfileOk(S.Model, Profile);
	const BodyId Ground = AddGroundPlane(S.Model);
	S.Lower = AddBox(S.Model, BrickShape(0.0, 0, Scale));
	S.Upper = AddBox(S.Model, BrickShape(ShiftXM, 1, Scale));
	S.Bed = AddInterfaceOk(S.Model, OnGroundPlane(Ground, S.Lower, BrickShape(0.0, 0, Scale), P));
	S.Joint = AddInterfaceOk(S.Model, *StackedFaces(S.Lower, BrickShape(0.0, 0, Scale), S.Upper, BrickShape(ShiftXM, 1, Scale), P));
	return S;
}

constexpr double BridgeHalfSpanM = 0.12;

struct ThreeBrickBridge
{
	Structure Model;
	BodyId Left;
	BodyId Right;
	BodyId Top;
	InterfaceId LeftBed;
	InterfaceId RightBed;
	InterfaceId LeftJoint;
	InterfaceId RightJoint;
};

ThreeBrickBridge MakeThreeBrickBridge(const StrengthProfile& Profile, double Scale = 1.0)
{
	ThreeBrickBridge B;
	const ProfileId P = AddProfileOk(B.Model, Profile);
	const BodyId Ground = AddGroundPlane(B.Model);
	const Box LeftShape = BrickShape(-BridgeHalfSpanM, 0, Scale);
	const Box RightShape = BrickShape(BridgeHalfSpanM, 0, Scale);
	const Box TopShape = BrickShape(0.0, 1, Scale);
	B.Left = AddBox(B.Model, LeftShape);
	B.Right = AddBox(B.Model, RightShape);
	B.Top = AddBox(B.Model, TopShape);
	B.LeftBed = AddInterfaceOk(B.Model, OnGroundPlane(Ground, B.Left, LeftShape, P));
	B.RightBed = AddInterfaceOk(B.Model, OnGroundPlane(Ground, B.Right, RightShape, P));
	B.LeftJoint = AddInterfaceOk(B.Model, *StackedFaces(B.Left, LeftShape, B.Top, TopShape, P));
	B.RightJoint = AddInterfaceOk(B.Model, *StackedFaces(B.Right, RightShape, B.Top, TopShape, P));
	return B;
}

StrengthProfile CrushedUnderOwnWeight()
{
	StrengthProfile Profile = Bonded(1.0e4, 1.0e4, 0.7);
	Profile.CompressiveStrengthPa = 0.5 * BrickWeightN() / BedAreaM2();
	return Profile;
}

BodyId AddFloatingBrick(Structure& Model)
{
	return AddBox(Model, Box{Vec3{1.0, 0.0, 1.0}, BrickShape().HalfExtentsM, BrickMassKg});
}

InterfaceId AddBrickOnGroundAt(Structure& Model, BodyId Ground, double CentreXM, const StrengthProfile& Profile)
{
	const ProfileId P = AddProfileOk(Model, Profile);
	const BodyId Brick = AddBox(Model, BrickShape(CentreXM));
	return AddInterfaceOk(Model, OnGroundPlane(Ground, Brick, BrickShape(CentreXM), P));
}

StrengthProfile OverflowingStiffness()
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.NormalStiffnessPaPerM = 1.0e308;
	Profile.TangentialStiffnessPaPerM = 1.0e308;
	return Profile;
}

Vec3 InPlaneLoadDirection(const StandingOptions& Options)
{
	return Unit(Vec3{Options.Gravity.X, Options.Gravity.Y, 0.0});
}

double LateralOverVertical(const StandingOptions& Options)
{
	return std::hypot(Options.Gravity.X, Options.Gravity.Y) / -Options.Gravity.Z;
}

struct CornerCounts
{
	int Open = 0;
	int Slipping = 0;
};

CornerCounts CountCorners(const StandingResult& Result)
{
	CornerCounts Counts;
	for (const InterfaceReading& Reading : Result.Readings)
	{
		for (const CornerReading& Corner : Reading.Corners)
		{
			Counts.Open += Corner.bOpen ? 1 : 0;
			Counts.Slipping += Corner.bSlipping ? 1 : 0;
		}
	}
	return Counts;
}

Vec3 MechanismMotionAt(const Structure& Model, const StandingResult& Result, BodyId Body, const Vec3& PointM)
{
	const MechanismMotion* Motion = FindMotion(Result, Body);
	if (Motion == nullptr)
	{
		return Vec3{};
	}
	return Plus(Motion->TranslationM, Cross(Motion->RotationRad, Minus(PointM, Model.Body(Body).CentroidM)));
}

double ClosingRateAt(const Structure& Model, const StandingResult& Result, InterfaceId Id, const Vec3& PointM)
{
	const InterfaceRecord& Record = Model.Interface(Id);
	const Vec3 RelativeM = Minus(MechanismMotionAt(Model, Result, Record.BodyB, PointM), MechanismMotionAt(Model, Result, Record.BodyA, PointM));
	return -Dot(RelativeM, Record.NormalAToB);
}
}

TEST_CASE("Statics.Solver.CompressionOverCapacityIsOverstressed")
{
	struct Case
	{
		const char* Name;
		double StrengthTimesAreaOverWeight;
		StandingState Expected;
	};
	const Case Cases[] = {
		{"strength half the weight over the area", 0.5, StandingState::Overstressed},
		{"strength just under the weight over the area", 1.0 - 1.0e-6, StandingState::Overstressed},
		{"strength just over the weight over the area", 1.0 + 1.0e-6, StandingState::Stands},
	};
	for (const Case& Each : Cases)
	{
		INFO("case: " << doctest::String(Each.Name));
		StrengthProfile Profile = DryJoint(0.7);
		Profile.CompressiveStrengthPa = Each.StrengthTimesAreaOverWeight * BrickWeightN() / BedAreaM2();
		const BrickOnGround F = MakeBrickOnGround(Profile);
		const StandingResult Result = SolveWith(F.Model, StandingOptions{});
		CHECK(Result.State == Each.Expected);
		const InterfaceReading* Bed = RequireReading(Result, F.Bed);
		if (Bed == nullptr)
		{
			continue;
		}
		CHECK(Bed->Axis == GoverningAxis::Compression);
		CheckRelative(Bed->Utilisation, (BrickWeightN() / 4.0) / CornerCapacityN(Profile.CompressiveStrengthPa), 1.0e-9, "utilisation");
	}
}

TEST_CASE("Statics.Solver.TensileBondHoldsTheUphillCornersPastTheTippingTilt")
{
	const double TiltRad = 80.0 * RadPerDegree;
	const double Friction = 0.7;
	const double CohesionPa = 1.0e5;
	CHECK(std::tan(TiltRad) > BrickLengthM / BrickHeightM);
	struct Case
	{
		const char* Name;
		double TensileStrengthPa;
		StandingState Expected;
	};
	const Case Cases[] = {{"strong tensile bond", 1000.0, StandingState::Stands}, {"weak tensile bond", 100.0, StandingState::Overstressed}};
	for (const Case& Each : Cases)
	{
		INFO("case: " << doctest::String(Each.Name));
		const StrengthProfile Profile = Bonded(Each.TensileStrengthPa, CohesionPa, Friction);
		const BrickOnGround F = MakeBrickOnGround(Profile);
		const StandingResult Result = SolveWith(F.Model, TiltedPlane(TiltRad));
		CHECK(Result.State == Each.Expected);
		const InterfaceReading* Bed = RequireReading(Result, F.Bed);
		if (Bed == nullptr)
		{
			continue;
		}
		const double TensionRatio = -UphillCornerNormalN(TiltRad) / CornerCapacityN(Each.TensileStrengthPa);
		const double ShearRatio = CornerShareOfSlopeShearN(TiltRad) / CornerCapacityN(CohesionPa);
		const double CompressionRatio = DownhillCornerNormalN(TiltRad) / CornerCapacityN(Profile.CompressiveStrengthPa);
		CHECK(TensionRatio > ShearRatio);
		CHECK(TensionRatio > CompressionRatio);
		CheckBondedSlopeCorners(*Bed, TiltRad);
		for (const std::size_t C : UphillCorners)
		{
			INFO("uphill corner " << C);
			CHECK(Bed->Corners[C].NormalForceN < 0.0);
		}
		CHECK(Bed->Axis == GoverningAxis::Tension);
		CheckRelative(Bed->Utilisation, TensionRatio, 1.0e-9, "utilisation");
	}
}

TEST_CASE("Statics.Solver.CohesionHoldsTheBedPastTheFrictionTilt")
{
	const double TiltRad = 40.0 * RadPerDegree;
	const double Friction = 0.3;
	CHECK(std::tan(TiltRad) > Friction);
	CHECK(std::tan(TiltRad) < BrickLengthM / BrickHeightM);
	struct Case
	{
		const char* Name;
		double ShearCohesionPa;
		StandingState Expected;
	};
	const Case Cases[] = {{"strong cohesion", 1000.0, StandingState::Stands}, {"weak cohesion", 100.0, StandingState::Overstressed}};
	for (const Case& Each : Cases)
	{
		INFO("case: " << doctest::String(Each.Name));
		const StrengthProfile Profile = Bonded(0.0, Each.ShearCohesionPa, Friction);
		const BrickOnGround F = MakeBrickOnGround(Profile);
		const StandingResult Result = SolveWith(F.Model, TiltedPlane(TiltRad));
		CHECK(Result.State == Each.Expected);
		const InterfaceReading* Bed = RequireReading(Result, F.Bed);
		if (Bed == nullptr)
		{
			continue;
		}
		const double ShearRatio = CornerShareOfSlopeShearN(TiltRad) / (CornerCapacityN(Each.ShearCohesionPa) + Friction * UphillCornerNormalN(TiltRad));
		const double CompressionRatio = DownhillCornerNormalN(TiltRad) / CornerCapacityN(Profile.CompressiveStrengthPa);
		CHECK(ShearRatio > CompressionRatio);
		CheckBondedSlopeCorners(*Bed, TiltRad);
		CHECK(Bed->Axis == GoverningAxis::Shear);
		CheckRelative(Bed->Utilisation, ShearRatio, 1.0e-9, "utilisation");
	}
}

TEST_CASE("Statics.Solver.ShearStrengthCapLetsADryBedSlipBeforeFrictionWould")
{
	const double Friction = 1.5;
	const double CapPa = 600.0;
	StrengthProfile Profile = DryJoint(Friction);
	Profile.MaxShearStrengthPa = CapPa;
	const double ExpectedRad = std::asin(CapPa * BedAreaM2() / BrickWeightN());
	CHECK(std::tan(ExpectedRad) < Friction);
	CHECK(std::tan(ExpectedRad) < BrickLengthM / BrickHeightM);
	CHECK(Friction * UphillCornerNormalN(ExpectedRad) > CornerCapacityN(CapPa));
	const BrickOnGround F = MakeBrickOnGround(Profile);

	const ThresholdBracket Bracket = Threshold([&](double TiltRad) { return SolveWith(F.Model, TiltedPlane(TiltRad)).State == StandingState::Stands; }, 0.0, Pi / 4.0, 1.0e-8);
	INFO("last standing " << Bracket.LastStanding << ", first failing " << Bracket.FirstFailing << ", expected " << ExpectedRad);
	CHECK(Bracket.bBracketed);
	CHECK(std::abs(Bracket.LastStanding - ExpectedRad) <= 1.0e-6);
	CHECK(std::abs(Bracket.FirstFailing - ExpectedRad) <= 1.0e-6);
	if (Bracket.bBracketed)
	{
		const StandingResult Sliding = SolveWith(F.Model, TiltedPlane(Bracket.FirstFailing));
		CHECK(Sliding.State == StandingState::Mechanism);
		const MechanismMotion* Motion = FindMotion(Sliding, F.Brick);
		CHECK(Motion != nullptr);
		if (Motion != nullptr)
		{
			CheckTranslationAlong(*Motion, Vec3{1.0, 0.0, 0.0}, ShapeTolerance);
		}
	}

	const double HalfRad = ExpectedRad / 2.0;
	CHECK(Friction * UphillCornerNormalN(HalfRad) > CornerCapacityN(CapPa));
	const StandingResult Standing = SolveWith(F.Model, TiltedPlane(HalfRad));
	CHECK(Standing.State == StandingState::Stands);
	const InterfaceReading* Bed = RequireReading(Standing, F.Bed);
	if (Bed == nullptr)
	{
		return;
	}
	CHECK(Bed->Axis == GoverningAxis::Shear);
	CheckRelative(Bed->Utilisation, CornerShareOfSlopeShearN(HalfRad) / CornerCapacityN(CapPa), 1.0e-9, "utilisation");
}

TEST_CASE("Statics.Solver.UphillCornersSlipAtFrictionCapacityWhileTheDownhillCornersStick")
{
	const double TiltRad = 29.0 * RadPerDegree;
	const double Friction = 0.6;
	CHECK(std::tan(TiltRad) > Friction / (1.0 + Friction * HeightOverLength()));
	CHECK(std::tan(TiltRad) < Friction);
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	const StandingResult Result = SolveWith(F.Model, TiltedPlane(TiltRad));
	CHECK(Result.State == StandingState::Stands);
	CHECK(Result.ActiveSetIterations >= 2);
	const InterfaceReading* Bed = RequireReading(Result, F.Bed);
	if (Bed == nullptr)
	{
		return;
	}
	CheckSlopeNormals(*Bed, TiltRad);
	const double UphillShearN = Friction * UphillCornerNormalN(TiltRad);
	const double DownhillShearN = (4.0 * CornerShareOfSlopeShearN(TiltRad) - 2.0 * UphillShearN) / 2.0;
	for (const std::size_t C : UphillCorners)
	{
		INFO("uphill corner " << C);
		const CornerReading& Corner = Bed->Corners[C];
		CHECK_FALSE(Corner.bOpen);
		CHECK(Corner.bSlipping);
		CheckRelative(Length(Corner.ShearForceN), UphillShearN, 1.0e-9, "shear force");
		CheckRelative(Length(Corner.ShearForceN), Friction * Corner.NormalForceN, 1.0e-9, "shear force against its own normal");
	}
	for (const std::size_t C : DownhillCorners)
	{
		INFO("downhill corner " << C);
		const CornerReading& Corner = Bed->Corners[C];
		CHECK_FALSE(Corner.bOpen);
		CHECK_FALSE(Corner.bSlipping);
		CheckRelative(Length(Corner.ShearForceN), DownhillShearN, 1.0e-9, "shear force");
		CHECK(Length(Corner.ShearForceN) < Friction * Corner.NormalForceN);
	}
}

TEST_CASE("Statics.Solver.HeldFrictionSettlesAlongTheTrialSlipDirection")
{
	const double Friction = 5.0;
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	const StandingResult Result = SolveWith(F.Model, BiaxialLateralLoad(0.625, 1.25));
	CHECK(Result.State == StandingState::Stands);
	CHECK(Result.ActiveSetIterations >= 3);
	const InterfaceReading* Bed = RequireReading(Result, F.Bed);
	if (Bed == nullptr)
	{
		return;
	}
	const std::array<CornerReading, 4>& Corners = Bed->Corners;
	for (const std::size_t C : {std::size_t{0}, std::size_t{1}, std::size_t{3}})
	{
		INFO("corner " << C);
		CHECK_FALSE(Corners[C].bOpen);
		CHECK_FALSE(Corners[C].bSlipping);
		CHECK(Length(Corners[C].ShearForceN) < Friction * Corners[C].NormalForceN);
	}
	const CornerReading& Slipping = Corners[2];
	CHECK_FALSE(Slipping.bOpen);
	CHECK(Slipping.bSlipping);
	const Vec3 HeldN = Slipping.ShearForceN;
	const Vec3 TrialN = Minus(Plus(Corners[1].ShearForceN, Corners[3].ShearForceN), Corners[0].ShearForceN);
	INFO("held " << HeldN << ", trial extrapolated from the stuck corners " << TrialN);
	const double CapacityN = Friction * Slipping.NormalForceN;
	CheckRelative(Length(HeldN), CapacityN, 1.0e-9, "held shear force");
	CHECK(Length(TrialN) >= CapacityN * (1.0 - 1.0e-9));
	CHECK(Length(Cross(HeldN, TrialN)) <= 1.0e-9 * Length(HeldN) * Length(TrialN));
	CHECK(Dot(HeldN, TrialN) > 0.0);
}

TEST_CASE("Statics.Solver.IterationCapBelowThePassesNeededReturnsUndecided")
{
	struct Case
	{
		const char* Name;
		StrengthProfile Profile;
		StandingOptions Options;
		StandingState Expected;
	};
	const double TipLambda = BrickWidthM / BrickHeightM;
	const Case Cases[] = {
		{"uphill corners slip on a slope", DryJoint(0.6), TiltedPlane(29.0 * RadPerDegree), StandingState::Stands},
		{"tips across its width", DryJoint(2.0), BiaxialLateralLoad(0.0, TipLambda + 1.0e-3), StandingState::Mechanism},
	};
	for (const Case& Each : Cases)
	{
		INFO("case: " << doctest::String(Each.Name));
		const BrickOnGround F = MakeBrickOnGround(Each.Profile);
		const StandingResult Uncapped = SolveWith(F.Model, Each.Options);
		CHECK(Uncapped.State == Each.Expected);
		const int Needed = Uncapped.ActiveSetIterations;
		INFO("passes needed " << Needed);
		CHECK(Needed >= 2);
		const StandingResult Short = SolveWith(F.Model, WithIterationCap(Each.Options, Needed - 1));
		CHECK(Short.State == StandingState::Undecided);
		CHECK(Short.ActiveSetIterations == Needed - 1);
		const StandingResult Exact = SolveWith(F.Model, WithIterationCap(Each.Options, Needed));
		CHECK(Exact.State == Each.Expected);
	}
}

TEST_CASE("Statics.Solver.BiaxialLoadOpensOnlyTheWindwardCornerAndThreeCarryTheBrick")
{
	const BrickOnGround F = MakeBrickOnGround(Bonded(0.0, 1.0e7, 0.7));
	const double WeightN = BrickWeightN();
	struct Case
	{
		double TippingFractionX;
		double TippingFractionY;
	};
	const Case Cases[] = {{0.3, 0.4}, {0.7, 0.5}};
	for (const Case& Each : Cases)
	{
		const double Tx = Each.TippingFractionX;
		const double Ty = Each.TippingFractionY;
		const bool bWindwardOpen = Tx + Ty > 1.0;
		INFO("tx " << Tx << ", ty " << Ty);
		const StandingResult Result = SolveWith(F.Model, BiaxialLateralLoad(Tx * BrickLengthM / BrickHeightM, Ty * BrickWidthM / BrickHeightM));
		CHECK(Result.State == StandingState::Stands);
		const InterfaceReading* Bed = RequireReading(Result, F.Bed);
		if (Bed == nullptr)
		{
			continue;
		}
		std::array<double, 4> ExpectedN{};
		for (std::size_t C = 0; C < 4; ++C)
		{
			ExpectedN[C] = WeightN / 4.0 * (1.0 + CornerSignU[C] * Tx + CornerSignV[C] * Ty);
		}
		if (bWindwardOpen)
		{
			ExpectedN = {WeightN * (Tx + Ty) / 2.0, WeightN * (1.0 - Tx) / 2.0, 0.0, WeightN * (1.0 - Ty) / 2.0};
		}
		for (std::size_t C = 0; C < 4; ++C)
		{
			INFO("corner " << C);
			const bool bOpen = bWindwardOpen && C == 2;
			CHECK(Bed->Corners[C].bOpen == bOpen);
			CHECK_FALSE(Bed->Corners[C].bSlipping);
			if (bOpen)
			{
				CHECK(Bed->Corners[C].NormalForceN == 0.0);
				CHECK(Length(Bed->Corners[C].ShearForceN) == 0.0);
				continue;
			}
			CheckRelative(Bed->Corners[C].NormalForceN, ExpectedN[C], 1.0e-9, "normal force");
		}
	}
}

TEST_CASE("Statics.Solver.TwoBrickStackOverturnedByABiaxialLoadIsAMechanism")
{
	const double ShiftXM = 0.03;
	const double LambdaX = 1.5;
	const double LambdaY = 0.875;
	CHECK(LambdaY * BrickHeightM > BrickWidthM / 2.0);
	const TwoBrickStack S = MakeTwoBrickStack(Bonded(0.0, 1.0e7, 0.7), ShiftXM);
	const StandingResult Result = SolveWith(S.Model, BiaxialLateralLoad(LambdaX, LambdaY));
	INFO("state " << Result.State << " after " << Result.ActiveSetIterations << " passes");
	CHECK(Result.State == StandingState::Mechanism);
}

TEST_CASE("Statics.Solver.LoadsThatOverflowReadInfiniteUtilisationAndNeverStand")
{
	const BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
	const Vec3 HugeForceN{0.0, 0.0, -1.0e308};
	CHECK(IsFinite(HugeForceN));
	const std::array<BodyLoad, 2> Loads{BodyLoad{F.Brick, HugeForceN, BrickShape().CentreM}, BodyLoad{F.Brick, HugeForceN, BrickShape().CentreM}};
	const StandingResult Result = SolveWith(F.Model, StandingOptions{}, Loads);
	INFO("state " << Result.State);
	CHECK(Result.State != StandingState::Stands);
	CHECK(Result.State != StandingState::InvalidInput);
	const InterfaceReading* Bed = RequireReading(Result, F.Bed);
	if (Bed == nullptr)
	{
		return;
	}
	CHECK(Bed->Utilisation == Infinity);
}

TEST_CASE("Statics.Solver.BrickOverturnedByABiaxialLoadIsAMechanismAboutItsToe")
{
	const BrickOnGround F = MakeBrickOnGround(Bonded(0.0, 1.0e7, 0.7));
	struct Case
	{
		double LambdaX;
		double LambdaY;
	};
	const Case Cases[] = {{3.375, 0.375}, {3.5, 1.0}};
	for (const Case& Each : Cases)
	{
		INFO("lambda x " << Each.LambdaX << ", lambda y " << Each.LambdaY);
		CHECK(Each.LambdaX * BrickHeightM > BrickLengthM);
		CHECK(Each.LambdaY * BrickHeightM < BrickWidthM);
		const StandingResult Result = SolveWith(F.Model, BiaxialLateralLoad(Each.LambdaX, Each.LambdaY));
		INFO("state " << Result.State << " after " << Result.ActiveSetIterations << " passes");
		CHECK(Result.State == StandingState::Mechanism);
		const MechanismMotion* Motion = FindMotion(Result, F.Brick);
		CHECK(Motion != nullptr);
		if (Motion == nullptr)
		{
			continue;
		}
		CheckRotationAboutEdge(*Motion, BrickShape().CentreM, Vec3{BrickLengthM / 2.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{Each.LambdaX, Each.LambdaY, 0.0}, ShapeTolerance);
	}
}

TEST_CASE("Statics.Solver.BrickPushedIntoAWallIsNeverAMechanism")
{
	const BrickOnGround F = MakeBrickAgainstWall(DryJoint(0.3));
	for (const double Lambda : {0.4, 0.6, 0.9})
	{
		const StandingResult Result = SolveWith(F.Model, LateralLoad(Lambda));
		INFO("lambda " << Lambda << ": state " << Result.State << " after " << Result.ActiveSetIterations << " passes");
		const MechanismMotion Moving = Result.Mechanism.empty() ? MechanismMotion{} : Result.Mechanism.front();
		INFO("motion translation " << Moving.TranslationM << ", rotation " << Moving.RotationRad);
		CHECK(Result.State == StandingState::Stands);
	}
}

TEST_CASE("Statics.Solver.SquatDryBlockPastBothThresholdsIsAMechanism")
{
	const double Friction = 0.6;
	CHECK(Friction < BrickLengthM / BrickHeightM);
	CHECK(Friction < BrickWidthM / BrickHeightM);
	struct Case
	{
		const char* Name;
		StandingOptions Options;
	};
	const Case Cases[] = {
		{"lateral 3.375", LateralLoad(3.375)},
		{"lateral 4", LateralLoad(4.0)},
		{"biaxial (0, 1.75)", BiaxialLateralLoad(0.0, 1.75)},
		{"tilt 75 deg", TiltedPlane(75.0 * RadPerDegree)},
		{"tilt 80 deg", TiltedPlane(80.0 * RadPerDegree)},
	};
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	for (const Case& Each : Cases)
	{
		INFO("case: " << doctest::String(Each.Name));
		const Vec3 G = Each.Options.Gravity;
		CHECK(LateralOverVertical(Each.Options) > Friction);
		CHECK((std::abs(G.X) * BrickHeightM > -G.Z * BrickLengthM || std::abs(G.Y) * BrickHeightM > -G.Z * BrickWidthM));
		const StandingResult Result = SolveWith(F.Model, Each.Options);
		INFO("state " << Result.State << " after " << Result.ActiveSetIterations << " passes");
		CHECK(Result.State == StandingState::Mechanism);
		const MechanismMotion* Motion = FindMotion(Result, F.Brick);
		CHECK(Motion != nullptr);
		if (Motion == nullptr)
		{
			continue;
		}
		CheckTranslationAlong(*Motion, InPlaneLoadDirection(Each.Options), ShapeTolerance);
	}
}

TEST_CASE("Statics.Solver.ReStickHoldsATwoBrickStackUnderBiaxialLoad")
{
	const TwoBrickStack S = MakeTwoBrickStack(DryJoint(5.0), 0.03);
	const StandingResult Result = SolveWith(S.Model, BiaxialLateralLoad(1.375, 0.625));
	INFO("state " << Result.State << " after " << Result.ActiveSetIterations << " passes");
	CHECK(Result.State == StandingState::Stands);
	CHECK(Result.Readings.size() == 2u);
	const CornerCounts Counts = CountCorners(Result);
	INFO("open " << Counts.Open << ", slipping " << Counts.Slipping);
	CHECK(Counts.Slipping == 1);
	CHECK(Counts.Open == 2);
}

TEST_CASE("Statics.Solver.StatusRevisitIsNotACycleWhileHeldForcesStillMove")
{
	struct Case
	{
		double LambdaX;
		double LambdaY;
	};
	const double SingleFriction = 5.0;
	const BrickOnGround F = MakeBrickOnGround(DryJoint(SingleFriction));
	for (const Case& Each : {Case{3.0, 0.875}, Case{3.25, 0.75}})
	{
		INFO("single brick, lambda x " << Each.LambdaX << ", lambda y " << Each.LambdaY);
		CHECK(Each.LambdaX * BrickHeightM < BrickLengthM);
		CHECK(Each.LambdaY * BrickHeightM < BrickWidthM);
		CHECK(std::hypot(Each.LambdaX, Each.LambdaY) < SingleFriction);
		const StandingResult Result = SolveWith(F.Model, BiaxialLateralLoad(Each.LambdaX, Each.LambdaY));
		INFO("state " << Result.State << " after " << Result.ActiveSetIterations << " passes");
		CHECK(Result.State == StandingState::Stands);
	}

	const double StackFriction = 0.6;
	const StandingOptions StackLoad = BiaxialLateralLoad(0.625, 0.875);
	CHECK(LateralOverVertical(StackLoad) > StackFriction);
	const TwoBrickStack S = MakeTwoBrickStack(DryJoint(StackFriction), 0.03);
	const StandingResult Result = SolveWith(S.Model, StackLoad);
	INFO("two-brick stack: state " << Result.State << " after " << Result.ActiveSetIterations << " passes");
	CHECK(Result.State == StandingState::Mechanism);
}

TEST_CASE("Statics.Solver.UnconstrainedBodiesListsOnlyTheMovingBodies")
{
	BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
	const Box FloatingShape{Vec3{1.0, 0.0, 1.0}, BrickShape().HalfExtentsM, BrickMassKg};
	const BodyId Floating = AddBox(F.Model, FloatingShape);
	const StandingResult Result = SolveWith(F.Model, StandingOptions{});
	INFO("state " << Result.State << ", unconstrained " << Describe(Result.UnconstrainedBodies));
	CHECK(Result.State == StandingState::Mechanism);
	CHECK(Result.UnconstrainedBodies.size() == 1u);
	CHECK(HasBody(Result.UnconstrainedBodies, Floating));
	CHECK_FALSE(HasBody(Result.UnconstrainedBodies, F.Brick));
	CHECK(FindMotion(Result, F.Brick) == nullptr);
	const InterfaceReading* Bed = RequireReading(Result, F.Bed);
	if (Bed == nullptr)
	{
		return;
	}
	CheckRelative(Bed->NormalForceN, BrickWeightN(), 1.0e-9, "standing brick bed normal force");
	CHECK(Bed->Utilisation > 0.0);
}

TEST_CASE("Statics.Solver.TransverseTangentMagnitudeIsCapacityOverTrialShear")
{
	const double Friction = 5.0;
	const double LambdaX = 2.5;
	const double LambdaY = 0.25;
	CHECK(LambdaX * BrickHeightM < BrickLengthM);
	CHECK(LambdaY * BrickHeightM < BrickWidthM);
	CHECK(std::hypot(LambdaX, LambdaY) < Friction);
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	const StandingResult Result = SolveWith(F.Model, BiaxialLateralLoad(LambdaX, LambdaY));
	const CornerCounts Counts = CountCorners(Result);
	INFO("state " << Result.State << " after " << Result.ActiveSetIterations << " passes, open " << Counts.Open << ", slipping " << Counts.Slipping);
	CHECK(Result.State == StandingState::Stands);
	CHECK(Counts.Slipping >= 1);
}

TEST_CASE("Statics.Solver.SlipCapacityIsZeroForACornerInTension")
{
	const double TiltRad = 75.0 * RadPerDegree;
	const double Friction = 5.0;
	const double TensileStrengthPa = 1.0e4;
	const double WeightN = BrickWeightN();
	const double DownhillShareN = WeightN * std::sin(TiltRad) / 2.0;
	CHECK(UphillCornerNormalN(TiltRad) < 0.0);
	CHECK(-UphillCornerNormalN(TiltRad) < CornerCapacityN(TensileStrengthPa));
	CHECK(DownhillShareN < Friction * DownhillCornerNormalN(TiltRad));
	const BrickOnGround F = MakeBrickOnGround(Bonded(TensileStrengthPa, 0.0, Friction));
	const StandingResult Result = SolveWith(F.Model, TiltedPlane(TiltRad));
	INFO("state " << Result.State << " after " << Result.ActiveSetIterations << " passes");
	CHECK(Result.State == StandingState::Stands);
	const InterfaceReading* Bed = RequireReading(Result, F.Bed);
	if (Bed == nullptr)
	{
		return;
	}
	CheckSlopeNormals(*Bed, TiltRad);
	for (const std::size_t C : UphillCorners)
	{
		INFO("uphill corner " << C << ", shear " << Bed->Corners[C].ShearForceN);
		CHECK(Bed->Corners[C].NormalForceN < 0.0);
		CHECK_FALSE(Bed->Corners[C].bOpen);
		CHECK(Bed->Corners[C].bSlipping);
		CHECK(Length(Bed->Corners[C].ShearForceN) <= 1.0e-12 * WeightN);
	}
	for (const std::size_t C : DownhillCorners)
	{
		INFO("downhill corner " << C);
		CHECK_FALSE(Bed->Corners[C].bOpen);
		CHECK_FALSE(Bed->Corners[C].bSlipping);
		CheckRelative(Length(Bed->Corners[C].ShearForceN), DownhillShareN, 1.0e-9, "shear force");
	}
}

TEST_CASE("Statics.Solver.AnInterfaceBetweenGroundedBodiesReadsZeroWithAxisNone")
{
	Structure Model;
	const ProfileId P = AddProfileOk(Model, DryJoint(0.7));
	const Box Footing{Vec3{-1.0, 0.0, 0.25}, Vec3{0.25, 0.25, 0.25}, 125.0};
	const Box Plinth{Vec3{-1.0, 0.0, 0.75}, Vec3{0.25, 0.25, 0.25}, 125.0};
	const BodyId FootingId = AddBox(Model, Footing, true);
	const BodyId PlinthId = AddBox(Model, Plinth, true);
	const InterfaceId Between = AddInterfaceOk(Model, *StackedFaces(FootingId, Footing, PlinthId, Plinth, P));
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Brick = AddBox(Model, BrickShape(1.0));
	AddInterfaceOk(Model, OnGroundPlane(Ground, Brick, BrickShape(1.0), P));
	const StandingResult Result = SolveWith(Model, StandingOptions{});
	INFO("state " << Result.State);
	CHECK(Result.State == StandingState::Stands);
	const InterfaceReading* Reading = RequireReading(Result, Between);
	if (Reading == nullptr)
	{
		return;
	}
	CHECK(Reading->Utilisation == 0.0);
	CHECK(Reading->Axis == GoverningAxis::None);
	CHECK(Reading->NormalForceN == 0.0);
	for (std::size_t C = 0; C < 4; ++C)
	{
		INFO("corner " << C);
		CHECK_FALSE(Reading->Corners[C].bOpen);
		CHECK_FALSE(Reading->Corners[C].bSlipping);
	}
}

TEST_CASE("Statics.Solver.NonFiniteOptionsAreInvalidInput")
{
	struct Case
	{
		const char* Name;
		double PivotToleranceRelative;
		int MaxActiveSetIterations;
	};
	const StandingOptions Defaults;
	const Case Cases[] = {
		{"NaN pivot tolerance", NaN, Defaults.MaxActiveSetIterations},
		{"zero pivot tolerance", 0.0, Defaults.MaxActiveSetIterations},
		{"negative pivot tolerance", -1.0, Defaults.MaxActiveSetIterations},
		{"infinite pivot tolerance", Infinity, Defaults.MaxActiveSetIterations},
		{"zero iteration cap", Defaults.PivotToleranceRelative, 0},
		{"pivot tolerance one", 1.0, Defaults.MaxActiveSetIterations},
	};
	const BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
	for (const Case& Each : Cases)
	{
		INFO("case: " << doctest::String(Each.Name));
		StandingOptions Options;
		Options.PivotToleranceRelative = Each.PivotToleranceRelative;
		Options.MaxActiveSetIterations = Each.MaxActiveSetIterations;
		const StandingResult Result = SolveWith(F.Model, Options);
		INFO("state " << Result.State);
		CHECK(Result.State == StandingState::InvalidInput);
	}

	{
		StandingOptions HalfTolerance;
		HalfTolerance.PivotToleranceRelative = 0.5;
		const StandingResult Valid = SolveWith(F.Model, HalfTolerance);
		INFO("pivot tolerance one half: state " << Valid.State);
		CHECK(Valid.State != StandingState::InvalidInput);
	}

	const double StiffnessPaPerM = 1.0e308;
	const Box Big = BoxOfDensity(Vec3{0.0, 0.0, 5.0}, Vec3{5.0, 5.0, 5.0}, 2000.0);
	CHECK(IsFinite(StiffnessPaPerM));
	CHECK_FALSE(IsFinite(StiffnessPaPerM * (2.0 * Big.HalfExtentsM.X) * (2.0 * Big.HalfExtentsM.Y) / 4.0));
	StrengthProfile Stiff = DryJoint(0.7);
	Stiff.NormalStiffnessPaPerM = StiffnessPaPerM;
	Stiff.TangentialStiffnessPaPerM = StiffnessPaPerM;
	Structure Model;
	const ProfileId P = AddProfileOk(Model, Stiff);
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Block = AddBox(Model, Big);
	AddInterfaceOk(Model, OnGroundPlane(Ground, Block, Big, P));
	const StandingResult Result = SolveWith(Model, StandingOptions{});
	INFO("overflowing assembled stiffness: state " << Result.State);
	CHECK(Result.State == StandingState::InvalidInput);
}

TEST_CASE("Statics.Solver.RestoredCornerHoldsFrictionOnABridge")
{
	const double Friction = 5.0;
	const double HalfHeightM = BrickHeightM / 2.0;
	const double OverlapLowM = BridgeHalfSpanM - BrickLengthM / 2.0;
	const double OverlapHighM = BrickLengthM / 2.0;
	const double RightBedHighXM = BridgeHalfSpanM + BrickLengthM / 2.0;
	struct Case
	{
		double LambdaX;
		double LambdaY;
	};
	const Case Cases[] = {{2.5, 0.75}, {2.25, 0.5}, {2.25, 0.625}, {2.125, 0.625}, {2.0, 0.0}, {2.125, 0.0}};
	const ThreeBrickBridge B = MakeThreeBrickBridge(DryJoint(Friction));
	for (const Case& Each : Cases)
	{
		INFO("lambda x " << Each.LambdaX << ", lambda y " << Each.LambdaY);
		const double JointResultantXM = Each.LambdaX * HalfHeightM;
		CHECK(JointResultantXM > OverlapLowM);
		CHECK(JointResultantXM < OverlapHighM);
		CHECK(Each.LambdaY * HalfHeightM < BrickWidthM / 2.0);
		CHECK(BridgeHalfSpanM / 2.0 + Each.LambdaX * BrickHeightM < RightBedHighXM);
		CHECK(Each.LambdaY * BrickHeightM < BrickWidthM / 2.0);
		CHECK(std::hypot(Each.LambdaX, Each.LambdaY) < Friction);
		const StandingResult Result = SolveWith(B.Model, BiaxialLateralLoad(Each.LambdaX, Each.LambdaY));
		const CornerCounts Counts = CountCorners(Result);
		INFO("state " << Result.State << " after " << Result.ActiveSetIterations << " passes, open " << Counts.Open << ", slipping " << Counts.Slipping);
		CHECK(Result.State == StandingState::Stands);
	}
}

TEST_CASE("Statics.Solver.RestoredCornerTangentDoesNotInflateThePivots")
{
	const double Friction = 0.6;
	const auto CheckCarriedAndAdmissible = [&](const char* Name, const Structure& Model, std::span<const InterfaceId> Beds, int BrickCount, double LambdaX) {
		INFO("case: " << doctest::String(Name) << ", lambda x " << LambdaX);
		const StandingOptions Load = BiaxialLateralLoad(LambdaX, 0.0);
		CHECK(LateralOverVertical(Load) > Friction);
		const StandingResult Result = SolveWith(Model, Load);
		INFO("state " << Result.State << " after " << Result.ActiveSetIterations << " passes, moving " << Describe(Result.UnconstrainedBodies));
		CHECK(Result.State == StandingState::Mechanism);
		double BedNormalN = 0.0;
		for (const InterfaceId Bed : Beds)
		{
			const InterfaceReading* Reading = RequireReading(Result, Bed);
			for (std::size_t C = 0; Reading != nullptr && C < 4; ++C)
			{
				BedNormalN += Reading->Corners[C].NormalForceN;
			}
		}
		CheckRelative(BedNormalN, BrickCount * BrickWeightN(), 1.0e-6, "bed corner normal forces against the total weight");
		for (const InterfaceReading& Reading : Result.Readings)
		{
			for (std::size_t C = 0; C < 4; ++C)
			{
				const CornerReading& Corner = Reading.Corners[C];
				if (Corner.bOpen)
				{
					continue;
				}
				const double ClosingM = ClosingRateAt(Model, Result, Reading.Id, Corner.PointM);
				INFO("interface slot " << Reading.Id.Index << ", corner " << C << ": closing rate " << ClosingM << ", normal " << Corner.NormalForceN << ", slipping " << Corner.bSlipping);
				CHECK(ClosingM <= 10.0 * ShapeTolerance);
			}
		}
	};

	const TwoBrickStack S = MakeTwoBrickStack(DryJoint(Friction), 0.08);
	const std::array<InterfaceId, 1> StackBeds{S.Bed};
	CheckCarriedAndAdmissible("stack shifted 0.08", S.Model, StackBeds, 2, 1.5);

	const ThreeBrickBridge B = MakeThreeBrickBridge(DryJoint(Friction));
	const std::array<InterfaceId, 2> BridgeBeds{B.LeftBed, B.RightBed};
	for (const double LambdaX : {2.0, 2.625})
	{
		CheckCarriedAndAdmissible("bridge", B.Model, BridgeBeds, 3, LambdaX);
	}
}

TEST_CASE("Statics.Solver.VerdictsAreInvariantUnderGeometricSimilarity")
{
	enum class Fixture
	{
		Brick,
		Stack,
		Bridge
	};
	struct Case
	{
		const char* Name;
		Fixture Kind;
		double Friction;
		double ShiftXM;
		double LambdaX;
		double LambdaY;
	};
	const Case Cases[] = {
		{"brick, mu 0.6, (0.625, 0.5)", Fixture::Brick, 0.6, 0.0, 0.625, 0.5},
		{"brick, mu 0.6, (1, 0.25)", Fixture::Brick, 0.6, 0.0, 1.0, 0.25},
		{"stack shifted 0.03, mu 5, (1.375, 0.75)", Fixture::Stack, 5.0, 0.03, 1.375, 0.75},
		{"bridge, mu 5, (2.5, 0.75)", Fixture::Bridge, 5.0, 0.0, 2.5, 0.75},
	};
	const auto SolveAt = [](const Case& Each, double Scale) {
		const StrengthProfile Profile = DryJoint(Each.Friction);
		const StandingOptions Options = BiaxialLateralLoad(Each.LambdaX, Each.LambdaY);
		switch (Each.Kind)
		{
		case Fixture::Brick: return SolveWith(MakeBrickOnGround(Profile, Scale).Model, Options);
		case Fixture::Stack: return SolveWith(MakeTwoBrickStack(Profile, Each.ShiftXM, Scale).Model, Options);
		case Fixture::Bridge: return SolveWith(MakeThreeBrickBridge(Profile, Scale).Model, Options);
		}
		return StandingResult{};
	};
	const auto CheckScaled = [](double Got, double ReferenceValue, double ForceScale, const char* What) {
		const double Want = ForceScale * ReferenceValue;
		INFO(doctest::String(What) << ": got " << Got << ", want " << Want);
		CHECK(std::abs(Got - Want) <= 1.0e-12 * std::abs(Want));
	};
	for (const Case& Each : Cases)
	{
		INFO("case: " << doctest::String(Each.Name));
		const StandingResult Reference = SolveAt(Each, 1.0);
		for (const double Scale : {1.0 / 16.0, 16.0})
		{
			const double ForceScale = Scale * Scale * Scale;
			const StandingResult Scaled = SolveAt(Each, Scale);
			INFO("scale " << Scale << ": state " << Scaled.State << " after " << Scaled.ActiveSetIterations << " passes, reference " << Reference.State << " after " << Reference.ActiveSetIterations);
			CHECK(Scaled.State == Reference.State);
			REQUIRE(Scaled.Readings.size() == Reference.Readings.size());
			for (std::size_t R = 0; R < Reference.Readings.size(); ++R)
			{
				for (std::size_t C = 0; C < 4; ++C)
				{
					INFO("reading " << R << ", corner " << C);
					const CornerReading& Got = Scaled.Readings[R].Corners[C];
					const CornerReading& Want = Reference.Readings[R].Corners[C];
					CheckScaled(Got.NormalForceN, Want.NormalForceN, ForceScale, "normal force");
					CheckScaled(Got.ShearForceN.X, Want.ShearForceN.X, ForceScale, "shear force x");
					CheckScaled(Got.ShearForceN.Y, Want.ShearForceN.Y, ForceScale, "shear force y");
					CheckScaled(Got.ShearForceN.Z, Want.ShearForceN.Z, ForceScale, "shear force z");
				}
			}
		}
	}
}

TEST_CASE("Statics.Solver.CompositeStateIsTheWorstComponent")
{
	{
		BrickOnGround F = MakeBrickOnGround(DryJoint(0.6));
		const BodyId Floating = AddFloatingBrick(F.Model);
		const StandingResult Result = SolveWith(F.Model, WithIterationCap(TiltedPlane(29.0 * RadPerDegree), 1));
		INFO("floating brick beside a slipping brick capped at one pass: state " << Result.State << " after " << Result.ActiveSetIterations << " passes, unconstrained " << Describe(Result.UnconstrainedBodies));
		CHECK(Result.State == StandingState::Undecided);
		CHECK(Result.Mechanism.size() == 1u);
		CHECK(Result.UnconstrainedBodies.size() == 1u);
		CHECK(HasBody(Result.UnconstrainedBodies, Floating));
		CHECK(FindMotion(Result, Floating) != nullptr);
	}
	{
		Structure Model;
		const BodyId Ground = AddGroundPlane(Model);
		const InterfaceId CrushedBed = AddBrickOnGroundAt(Model, Ground, 0.0, CrushedUnderOwnWeight());
		const BodyId Floating = AddFloatingBrick(Model);
		const StandingResult Result = SolveWith(Model, StandingOptions{});
		INFO("floating brick beside a crushed brick: state " << Result.State);
		CHECK(Result.State == StandingState::Mechanism);
		CHECK(HasBody(Result.UnconstrainedBodies, Floating));
		const InterfaceReading* Bed = RequireReading(Result, CrushedBed);
		if (Bed != nullptr)
		{
			INFO("crushed bed utilisation " << Bed->Utilisation << ", axis " << Bed->Axis);
			CHECK(Bed->Utilisation > 1.0);
			CHECK(Bed->Axis == GoverningAxis::Compression);
		}
	}
	{
		Structure Model;
		const BodyId Ground = AddGroundPlane(Model);
		AddBrickOnGroundAt(Model, Ground, 0.0, CrushedUnderOwnWeight());
		AddBrickOnGroundAt(Model, Ground, 1.0, DryJoint(0.7));
		const StandingResult Result = SolveWith(Model, StandingOptions{});
		INFO("crushed brick beside a standing brick: state " << Result.State);
		CHECK(Result.State == StandingState::Overstressed);
	}
	{
		Structure Model;
		const BodyId Ground = AddGroundPlane(Model);
		AddBrickOnGroundAt(Model, Ground, 0.0, DryJoint(0.7));
		const Box Big = BoxOfDensity(Vec3{20.0, 0.0, 5.0}, Vec3{5.0, 5.0, 5.0}, 2000.0);
		const BodyId Block = AddBox(Model, Big);
		AddInterfaceOk(Model, OnGroundPlane(Ground, Block, Big, AddProfileOk(Model, OverflowingStiffness())));
		const StandingResult Result = SolveWith(Model, StandingOptions{});
		INFO("overflowing stiffness beside a standing brick: state " << Result.State);
		CHECK(Result.State == StandingState::InvalidInput);
	}

	const StandingOptions Slope = TiltedPlane(29.0 * RadPerDegree);
	Structure SlipAlone;
	AddBrickOnGroundAt(SlipAlone, AddGroundPlane(SlipAlone), 0.0, DryJoint(0.6));
	Structure BondedAlone;
	AddBrickOnGroundAt(BondedAlone, AddGroundPlane(BondedAlone), 1.0, Bonded(0.0, 1.0e7, 0.7));
	Structure Both;
	const BodyId BothGround = AddGroundPlane(Both);
	AddBrickOnGroundAt(Both, BothGround, 0.0, DryJoint(0.6));
	AddBrickOnGroundAt(Both, BothGround, 1.0, Bonded(0.0, 1.0e7, 0.7));
	const int SlipPasses = SolveWith(SlipAlone, Slope).ActiveSetIterations;
	const int BondedPasses = SolveWith(BondedAlone, Slope).ActiveSetIterations;
	const int BothPasses = SolveWith(Both, Slope).ActiveSetIterations;
	INFO("passes: slipping brick " << SlipPasses << ", bonded brick " << BondedPasses << ", both " << BothPasses);
	CHECK(BondedPasses == 1);
	CHECK(SlipPasses >= 2);
	CHECK(BothPasses == SlipPasses);
}

TEST_CASE("Statics.Solver.MechanismReadingsAreInEquilibriumWithTheRangeLoad")
{
	const double Friction = 0.6;
	const double Lambda = 3.375;
	CHECK(Lambda > Friction);
	CHECK(Friction * BrickHeightM / 2.0 < BrickWidthM / 2.0);
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	const StandingOptions Load = LateralLoad(Lambda);
	const StandingResult Result = SolveWith(F.Model, Load);
	const CornerCounts Counts = CountCorners(Result);
	INFO("state " << Result.State << " after " << Result.ActiveSetIterations << " passes, open " << Counts.Open << ", slipping " << Counts.Slipping);
	CHECK(Result.State == StandingState::Mechanism);
	const MechanismMotion* Motion = FindMotion(Result, F.Brick);
	const InterfaceReading* Bed = RequireReading(Result, F.Bed);
	if (Motion == nullptr || Bed == nullptr)
	{
		CHECK(Motion != nullptr);
		return;
	}
	CheckTranslationAlong(*Motion, Vec3{1.0, 0.0, 0.0}, ShapeTolerance);
	const Vec3 CentroidM = BrickShape().CentreM;
	const double WeightN = BrickWeightN();
	Vec3 ResidualN = Times(BrickMassKg, Load.Gravity);
	Vec3 ResidualNm;
	for (std::size_t C = 0; C < 4; ++C)
	{
		const CornerReading& Corner = Bed->Corners[C];
		INFO("corner " << C << ": normal " << Corner.NormalForceN << ", shear " << Corner.ShearForceN << ", open " << Corner.bOpen << ", slipping " << Corner.bSlipping);
		CHECK(Corner.bSlipping);
		const Vec3 ForceN = Plus(Vec3{0.0, 0.0, Corner.NormalForceN}, Corner.ShearForceN);
		ResidualN = Plus(ResidualN, ForceN);
		ResidualNm = Plus(ResidualNm, Cross(Minus(Corner.PointM, CentroidM), ForceN));
	}
	INFO("residual force " << ResidualN << ", residual moment about the centroid " << ResidualNm);
	CHECK(std::abs(ResidualN.Y) <= 1.0e-9 * WeightN);
	CHECK(std::abs(ResidualN.Z) <= 1.0e-9 * WeightN);
	CHECK(Length(ResidualNm) <= 1.0e-9 * WeightN * BrickLengthM);
	CHECK(ResidualN.X > 0.0);
	CheckRelative(Bed->NormalForceN, WeightN, 1.0e-9, "bed normal force against the weight");
}
