#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "CertificateTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double BrickDensityKgPerM3 = 2000.0;
constexpr double RatioRelative = 1e-12;

Catalogue LoadRepositoryCatalogue()
{
	const CatalogueResult Result = LoadCatalogue(ReadCatalogueFile());
	INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
	REQUIRE(Result.Ok());
	return Result.Value;
}

struct BlockOnGround
{
	Structure Model;
	BodyId Block;
	Box Shape;
	double AreaM2 = 0;
	double WeightN = 0;
};

BlockOnGround MakeBlockOnGround(const StrengthProfile& Profile, double LengthM, double WidthM, double HeightM)
{
	BlockOnGround F;
	F.Shape = BoxOfDensity(Vec3{0.0, 0.0, HeightM / 2.0}, Vec3{LengthM / 2.0, WidthM / 2.0, HeightM / 2.0}, BrickDensityKgPerM3);
	F.AreaM2 = LengthM * WidthM;
	F.WeightN = F.Shape.MassKg * GravityMPerS2;
	const ProfileId P = AddProfileOk(F.Model, Profile);
	const BodyId Ground = AddGroundPlane(F.Model);
	F.Block = AddBox(F.Model, F.Shape);
	AddInterfaceOk(F.Model, OnGroundPlane(Ground, F.Block, F.Shape, P));
	return F;
}

BlockOnGround MakeBrickOnBed(const StrengthProfile& Profile)
{
	return MakeBlockOnGround(Profile, 0.215, 0.1025, 0.065);
}

CertificateOptions WeightlessCertificate()
{
	return CertificateUnderGravity(Vec3{0.0, 0.0, 0.0});
}

double RelativeGap(double Actual, double Expected)
{
	return std::abs(Actual - Expected) / std::abs(Expected);
}
}

TEST_CASE("Catalogue.Anchor.BondedBrickLiftsOffItsBedAtTensileStrengthTimesArea")
{
	const Catalogue Value = LoadRepositoryCatalogue();
	const JointRow& Bed = JointOrFail(Value, "clay_gp_mortar_bed_ec6");
	const BlockOnGround F = MakeBrickOnBed(Bed.Profile);
	const double BondN = Bed.Profile.TensileStrengthPa * F.AreaM2;
	REQUIRE(BondN > 0.0);
	struct Case
	{
		double BondFraction;
		CertificateState Certificate;
		ComponentVerdict Authority;
	};
	const Case Cases[] = {{0.99, CertificateState::Stands, ComponentVerdict::Stands}, {1.01, CertificateState::Collapses, ComponentVerdict::Overstressed}};
	for (const Case& Each : Cases)
	{
		const std::vector<BodyLoad> Loads = {BodyLoad{F.Block, Vec3{0.0, 0.0, F.WeightN + Each.BondFraction * BondN}, F.Shape.CentreM}};
		const CertificateResult Certificate = DecideWith(F.Model, CertificateOptions{}, Loads);
		const AuthorityResult Authority = Judge(F.Model, AuthorityUnderGravity(Vec3{0.0, 0.0, -GravityMPerS2}), Loads);
		INFO("uplift W + " << Each.BondFraction << " ft A, ft A = " << BondN << " N, W = " << F.WeightN << " N: certificate " << Certificate.State << " ["
			<< Certificate.LoadFactorLower << ", " << Certificate.LoadFactorUpper << "]; authority " << DescribeAuthority(Authority));
		CHECK(Certificate.State == Each.Certificate);
		CHECK(Authority.Worst == Each.Authority);
	}
}

TEST_CASE("Catalogue.Anchor.PerpendHalvesTheBedCohesionAndCapsAtTheEc6Ratio")
{
	const Catalogue Value = LoadRepositoryCatalogue();
	const StrengthProfile& Bed = JointOrFail(Value, "clay_gp_mortar_bed_ec6").Profile;
	const StrengthProfile& Perpend = JointOrFail(Value, "clay_gp_mortar_perpend_ec6").Profile;
	INFO("bed c " << Bed.ShearCohesionPa << ", cap " << Bed.MaxShearStrengthPa << "; perpend c " << Perpend.ShearCohesionPa << ", cap " << Perpend.MaxShearStrengthPa);
	CHECK(RelativeGap(Perpend.ShearCohesionPa, 0.5 * Bed.ShearCohesionPa) <= RatioRelative);
	CHECK(RelativeGap(Perpend.MaxShearStrengthPa, (0.045 / 0.065) * Bed.MaxShearStrengthPa) <= RatioRelative);
	CHECK(JointOrFail(Value, "clay_gp_mortar_perpend_ec6").Kind == "perpend");
}

TEST_CASE("Catalogue.Anchor.MeasuredBondIsTwiceTheCodeBondInTensionAndCohesion")
{
	const Catalogue Value = LoadRepositoryCatalogue();
	const JointRow& Code = JointOrFail(Value, "clay_gp_mortar_bed_ec6");
	const JointRow& Mean = JointOrFail(Value, "clay_gp_mortar_bed_gooch_mean");
	REQUIRE(Code.Profile.TensileStrengthPa > 0.0);
	REQUIRE(Code.Profile.ShearCohesionPa > 0.0);
	const double TensionRatio = Mean.Profile.TensileStrengthPa / Code.Profile.TensileStrengthPa;
	const double CohesionRatio = Mean.Profile.ShearCohesionPa / Code.Profile.ShearCohesionPa;
	INFO("tension ratio " << TensionRatio << ", cohesion ratio " << CohesionRatio);
	CHECK(Code.Basis == StrengthBasis::Characteristic);
	CHECK(Mean.Basis == StrengthBasis::Mean);
	CHECK(RelativeGap(TensionRatio, 2.0) <= RatioRelative);
	CHECK(RelativeGap(CohesionRatio, 2.0) <= RatioRelative);
}

TEST_CASE("Catalogue.Anchor.TwoScrewsSlipAtTwiceTheirShearCapacityWhateverTheArea")
{
	const Catalogue Value = LoadRepositoryCatalogue();
	const FastenerRow& Screw = FastenerOrFail(Value, "ec5_screw_8x60_steel_to_glulam");
	const int Count = 2;
	const double CapacityN = Count * Screw.ShearPerPlaneN;
	REQUIRE(CapacityN > 0.0);
	CHECK(0.97 < 1.0 / CircumscribedOverInscribed(16));
	const double Footprints[][2] = {{0.215, 0.1025}, {0.1, 0.1}};
	for (const auto& Footprint : Footprints)
	{
		const double AreaM2 = Footprint[0] * Footprint[1];
		const BlockOnGround F = MakeBlockOnGround(FastenerProfile(Screw, Count, AreaM2), Footprint[0], Footprint[1], 0.065);
		const Vec3 BedCentreM{0.0, 0.0, 0.0};
		for (const auto& [Fraction, Expected] : {std::pair{0.97, CertificateState::Stands}, std::pair{1.01, CertificateState::Collapses}})
		{
			const std::vector<BodyLoad> Loads = {BodyLoad{F.Block, Vec3{Fraction * CapacityN, 0.0, 0.0}, BedCentreM}};
			const CertificateResult Result = DecideWith(F.Model, WeightlessCertificate(), Loads);
			INFO("area " << AreaM2 << " m2, shear " << Fraction << " x " << CapacityN << " N: " << Result.State << " [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
			CHECK(Result.State == Expected);
		}
	}
}

TEST_CASE("Catalogue.Anchor.TwoScrewsPullOutAtTwiceTheirWithdrawalCapacityWhateverTheArea")
{
	const Catalogue Value = LoadRepositoryCatalogue();
	const FastenerRow& Screw = FastenerOrFail(Value, "ec5_screw_8x60_steel_to_glulam");
	const int Count = 2;
	const double CapacityN = Count * Screw.WithdrawalN;
	REQUIRE(CapacityN > 0.0);
	const double Footprints[][2] = {{0.215, 0.1025}, {0.1, 0.1}};
	for (const auto& Footprint : Footprints)
	{
		const double AreaM2 = Footprint[0] * Footprint[1];
		const BlockOnGround F = MakeBlockOnGround(FastenerProfile(Screw, Count, AreaM2), Footprint[0], Footprint[1], 0.065);
		for (const auto& [Fraction, Expected] : {std::pair{0.99, CertificateState::Stands}, std::pair{1.01, CertificateState::Collapses}})
		{
			const std::vector<BodyLoad> Loads = {BodyLoad{F.Block, Vec3{0.0, 0.0, Fraction * CapacityN}, F.Shape.CentreM}};
			const CertificateResult Result = DecideWith(F.Model, WeightlessCertificate(), Loads);
			INFO("area " << AreaM2 << " m2, pull " << Fraction << " x " << CapacityN << " N: " << Result.State << " [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
			CHECK(Result.State == Expected);
		}
	}
}

TEST_CASE("Catalogue.Anchor.MeanBoltShearIsTheBlassCollingRatioTimesTheCharacteristic")
{
	const Catalogue Value = LoadRepositoryCatalogue();
	const FastenerRow& Characteristic = FastenerOrFail(Value, "ec5_bolt_m16_double_shear");
	const FastenerRow& Mean = FastenerOrFail(Value, "ec5_bolt_m16_double_shear_mean");
	INFO("characteristic " << Characteristic.ShearPerPlaneN << " N, mean " << Mean.ShearPerPlaneN << " N");
	CHECK(Characteristic.Basis == StrengthBasis::Characteristic);
	CHECK(Mean.Basis == StrengthBasis::Mean);
	CHECK(RelativeGap(Mean.ShearPerPlaneN, 1.074 * Characteristic.ShearPerPlaneN) <= RatioRelative);
	CHECK(RelativeGap(2.0 * Characteristic.ShearPerPlaneN, 12.9e3) <= RatioRelative);
}

TEST_CASE("Catalogue.Anchor.LimeMortarFrictionIsTheTangentOfItsMeasuredAngle")
{
	const Catalogue Value = LoadRepositoryCatalogue();
	const double Friction = JointOrFail(Value, "lime_mortar_handmade_brick_upc_mean").Profile.FrictionCoefficient;
	INFO("mu " << Friction);
	CHECK(std::abs(Friction - std::tan(33.71 * RadPerDegree)) <= 1e-6);
}
