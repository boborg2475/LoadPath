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

Box BrickShape(double CentreXM = 0.0, int Course = 0)
{
	return Box{Vec3{CentreXM, 0.0, (Course + 0.5) * BrickHeightM}, Vec3{BrickLengthM / 2.0, BrickWidthM / 2.0, BrickHeightM / 2.0}, BrickMassKg};
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

BrickOnGround MakeBrickOnGround(const StrengthProfile& Profile)
{
	BrickOnGround F;
	F.Profile = AddProfileOk(F.Model, Profile);
	const BodyId Ground = AddGroundPlane(F.Model);
	F.Brick = AddBox(F.Model, BrickShape());
	F.Bed = AddInterfaceOk(F.Model, OnGroundPlane(Ground, F.Brick, BrickShape(), F.Profile));
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
	Structure Model;
	const ProfileId P = AddProfileOk(Model, Bonded(0.0, 1.0e7, 0.7));
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Lower = AddBox(Model, BrickShape(0.0, 0));
	const BodyId Upper = AddBox(Model, BrickShape(ShiftXM, 1));
	AddInterfaceOk(Model, OnGroundPlane(Ground, Lower, BrickShape(0.0, 0), P));
	AddInterfaceOk(Model, *StackedFaces(Lower, BrickShape(0.0, 0), Upper, BrickShape(ShiftXM, 1), P));
	const StandingResult Result = SolveWith(Model, BiaxialLateralLoad(LambdaX, LambdaY));
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
		CheckRotationAboutEdge(*Motion, BrickShape().CentreM, Vec3{BrickLengthM / 2.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, ShapeTolerance);
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
		CHECK(Result.State != StandingState::Mechanism);
	}
}
