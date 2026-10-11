#include "CatalogueTestSupport.h"

#include "loadpath/Catalogue.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double PaPerMPa = 1e6;
constexpr double PaPerMPerGPaPerM = 1e9;
constexpr double NPerKN = 1e3;

CatalogueResult LoadRepositoryCatalogue()
{
	const std::string Json = ReadCatalogueFile();
	CHECK_MESSAGE(!Json.empty(), "the catalogue file must be readable at " LOADPATH_CATALOGUE_PATH);
	CatalogueResult Result = LoadCatalogue(Json);
	INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
	REQUIRE(Result.Ok());
	return Result;
}

struct JointFileValues
{
	const char* Name;
	double TensileMPa;
	double CohesionMPa;
	double Friction;
	double CompressiveMPa;
};

const JointFileValues JointValues[] = {
	{"clay_gp_mortar_bed_ec6", 0.10, 0.30, 0.4, 8.93},
	{"clay_gp_mortar_perpend_ec6", 0.20, 0.15, 0.4, 8.93},
	{"clay_gp_mortar_bed_gooch_mean", 0.20, 0.60, 1.05, 8.93},
	{"lime_mortar_handmade_brick_upc_mean", 0.0, 0.16, 0.667169287134503, 8.93},
	{"c24_bearing_perp_to_grain", 0.0, 0.0, 0.3, 3.75},
};

struct FastenerFileValues
{
	const char* Name;
	double ShearKN;
	double WithdrawalKN;
	double BearingMPa;
};

const FastenerFileValues FastenerValues[] = {
	{"ec5_screw_8x60_steel_to_glulam", 4.83, 1.74, 3.75},
	{"ec5_ring_nail_4x40_steel_plate", 1.607, 0.42, 3.75},
	{"ec5_bolt_m16_double_shear", 6.45, 0.0, 3.75},
	{"ec5_screw_8x60_steel_to_glulam_mean", 5.18259, 1.74, 3.75},
	{"ec5_bolt_m16_double_shear_mean", 6.9273, 0.0, 3.75},
};

struct CatalogueErrorCase
{
	const char* Name;
	CatalogueSpec Spec;
	bool bFastener;
	const char* MustName;
	const char* RowName;
};

CatalogueErrorCase JointCase(const char* Name, JsonElement Element, const char* MustName, const char* RowName = "bed_two")
{
	CatalogueSpec Spec = ValidCatalogueSpec();
	Spec.Joints[1] = std::move(Element);
	return {Name, Spec, false, MustName, RowName};
}

CatalogueErrorCase FastenerCase(const char* Name, JsonElement Element, const char* MustName, const char* RowName = "screw_two")
{
	CatalogueSpec Spec = ValidCatalogueSpec();
	Spec.Fasteners[1] = std::move(Element);
	return {Name, Spec, true, MustName, RowName};
}

std::vector<CatalogueErrorCase> ErrorCases()
{
	return {
		JointCase("missing tension", WithoutField(BedJointElement("bed_two"), "tensile_mpa"), "tensile_mpa"),
		JointCase("missing basis", WithoutField(BedJointElement("bed_two"), "basis"), "basis"),
		JointCase("string cohesion", WithField(BedJointElement("bed_two"), "cohesion_mpa", "\"0.30\""), "cohesion_mpa"),
		JointCase("number kind", WithField(BedJointElement("bed_two"), "kind", "3"), "kind"),
		JointCase("negative tension", WithField(BedJointElement("bed_two"), "tensile_mpa", "-0.10"), "tensile_mpa"),
		JointCase("negative friction normal stress", WithField(BedJointElement("bed_two"), "friction_normal_stress_mpa", "-0.2"), "friction_normal_stress_mpa"),
		JointCase("unknown kind", WithField(BedJointElement("bed_two"), "kind", "\"lintel\""), "kind"),
		JointCase("unknown basis", WithField(BedJointElement("bed_two"), "basis", "\"nominal\""), "basis"),
		JointCase("stiffness the structure refuses after conversion", WithField(BedJointElement("bed_two"), "normal_stiffness_gpa_per_m", "1e300"), "NonFiniteInput"),
		JointCase("duplicate joint name", BedJointElement("bed"), "bed", "bed"),
		FastenerCase("missing withdrawal", WithoutField(ScrewElement("screw_two"), "withdrawal_kn"), "withdrawal_kn"),
		FastenerCase("string shear", WithField(ScrewElement("screw_two"), "shear_per_plane_kn", "\"4.83\""), "shear_per_plane_kn"),
		FastenerCase("negative bearing", WithField(ScrewElement("screw_two"), "bearing_mpa", "-3.75"), "bearing_mpa"),
		FastenerCase("unknown fastener basis", WithField(ScrewElement("screw_two"), "basis", "\"nominal\""), "basis"),
		FastenerCase("duplicate fastener name", ScrewElement("screw"), "screw", "screw"),
	};
}
}

TEST_CASE("Catalogue.Loader.TheRepositoryFileLoadsEveryRowInFileOrder")
{
	const CatalogueResult Result = LoadRepositoryCatalogue();
	const std::vector<std::string> ExpectedJoints = {"clay_gp_mortar_bed_ec6", "clay_gp_mortar_perpend_ec6", "clay_gp_mortar_bed_gooch_mean",
		"lime_mortar_handmade_brick_upc_mean", "c24_bearing_perp_to_grain"};
	const std::vector<std::string> ExpectedFasteners = {"ec5_screw_8x60_steel_to_glulam", "ec5_ring_nail_4x40_steel_plate", "ec5_bolt_m16_double_shear",
		"ec5_screw_8x60_steel_to_glulam_mean", "ec5_bolt_m16_double_shear_mean"};
	REQUIRE(Result.Value.Joints.size() == ExpectedJoints.size());
	REQUIRE(Result.Value.Fasteners.size() == ExpectedFasteners.size());
	for (std::size_t R = 0; R < std::min(ExpectedJoints.size(), Result.Value.Joints.size()); ++R)
	{
		CHECK(Result.Value.Joints[R].Name == ExpectedJoints[R]);
		CHECK_FALSE(Result.Value.Joints[R].Source.empty());
		CHECK_FALSE(Result.Value.Joints[R].Note.empty());
	}
	for (std::size_t R = 0; R < std::min(ExpectedFasteners.size(), Result.Value.Fasteners.size()); ++R)
	{
		CHECK(Result.Value.Fasteners[R].Name == ExpectedFasteners[R]);
		CHECK_FALSE(Result.Value.Fasteners[R].Source.empty());
		CHECK_FALSE(Result.Value.Fasteners[R].Note.empty());
	}
}

TEST_CASE("Catalogue.Loader.BedRowConvertsEveryFieldExactlyOnce")
{
	const CatalogueResult Result = LoadRepositoryCatalogue();
	const JointRow& Bed = JointOrFail(Result.Value, "clay_gp_mortar_bed_ec6");
	CHECK(Bed.Kind == "bed");
	CHECK(Bed.Basis == StrengthBasis::Characteristic);
	CHECK(Bed.Profile.CompressiveStrengthPa == 8.93 * PaPerMPa);
	CHECK(Bed.Profile.ShearCohesionPa == 0.30 * PaPerMPa);
	CHECK(Bed.Profile.TensileStrengthPa == 0.10 * PaPerMPa);
	CHECK(Bed.Profile.FrictionCoefficient == 0.4);
	CHECK(Bed.Profile.MaxShearStrengthPa == 1.30 * PaPerMPa);
	CHECK(Bed.Profile.NormalStiffnessPaPerM == 893 * PaPerMPerGPaPerM);
	CHECK(Bed.Profile.TangentialStiffnessPaPerM == 357 * PaPerMPerGPaPerM);
	CHECK(std::isnan(Bed.FrictionNormalStressPa));
}

TEST_CASE("Catalogue.Loader.EveryRowConvertsItsStrengthsExactlyOnce")
{
	const CatalogueResult Result = LoadRepositoryCatalogue();
	for (const JointFileValues& Each : JointValues)
	{
		INFO("joint " << std::string(Each.Name));
		const JointRow& Row = JointOrFail(Result.Value, Each.Name);
		CHECK(Row.Profile.TensileStrengthPa == Each.TensileMPa * PaPerMPa);
		CHECK(Row.Profile.ShearCohesionPa == Each.CohesionMPa * PaPerMPa);
		CHECK(Row.Profile.FrictionCoefficient == Each.Friction);
		CHECK(Row.Profile.CompressiveStrengthPa == Each.CompressiveMPa * PaPerMPa);
	}
	for (const FastenerFileValues& Each : FastenerValues)
	{
		INFO("fastener " << std::string(Each.Name));
		const FastenerRow& Row = FastenerOrFail(Result.Value, Each.Name);
		CHECK(Row.ShearPerPlaneN == Each.ShearKN * NPerKN);
		CHECK(Row.WithdrawalN == Each.WithdrawalKN * NPerKN);
		CHECK(Row.BearingStrengthPa == Each.BearingMPa * PaPerMPa);
	}
}

TEST_CASE("Catalogue.Loader.MeasuredFrictionCarriesItsNormalStressAndCodeFrictionReadsNaN")
{
	const CatalogueResult Result = LoadRepositoryCatalogue();
	const JointRow& Gooch = JointOrFail(Result.Value, "clay_gp_mortar_bed_gooch_mean");
	CHECK(Gooch.Basis == StrengthBasis::Mean);
	CHECK(Gooch.FrictionNormalStressPa == 0.2 * PaPerMPa);
	CHECK(std::isnan(JointOrFail(Result.Value, "clay_gp_mortar_perpend_ec6").FrictionNormalStressPa));
	CHECK(std::isnan(JointOrFail(Result.Value, "c24_bearing_perp_to_grain").FrictionNormalStressPa));
}

TEST_CASE("Catalogue.Loader.AnAbsentShearCapReadsInfinite")
{
	const CatalogueResult Result = LoadRepositoryCatalogue();
	for (const char* Name : {"clay_gp_mortar_bed_gooch_mean", "lime_mortar_handmade_brick_upc_mean", "c24_bearing_perp_to_grain"})
	{
		INFO("joint " << std::string(Name));
		CHECK(JointOrFail(Result.Value, Name).Profile.MaxShearStrengthPa == std::numeric_limits<double>::infinity());
	}
}

TEST_CASE("Catalogue.Loader.AnUnknownNameFindsNothing")
{
	const CatalogueResult Result = LoadRepositoryCatalogue();
	CHECK(Result.Value.FindJoint("no_such_joint") == nullptr);
	CHECK(Result.Value.FindFastener("no_such_fastener") == nullptr);
	CHECK(Result.Value.FindJoint("ec5_screw_8x60_steel_to_glulam") == nullptr);
	CHECK(Result.Value.FindFastener("clay_gp_mortar_bed_ec6") == nullptr);
}

TEST_CASE("Catalogue.Loader.TheValidSyntheticDocumentLoads")
{
	const BuiltCatalogue Doc = BuildCatalogue(ValidCatalogueSpec());
	const CatalogueResult Result = LoadCatalogue(Doc.Json);
	INFO(Doc.Json << "line " << Result.Error.Line << ": " << Result.Error.Message);
	REQUIRE(Result.Ok());
	CHECK(Result.Value.Joints.size() == 2);
	CHECK(Result.Value.Fasteners.size() == 2);
}

TEST_CASE("Catalogue.Loader.RowErrorsNameTheFieldAtTheRowLine")
{
	for (const CatalogueErrorCase& Case : ErrorCases())
	{
		INFO(doctest::String(Case.Name));
		const BuiltCatalogue Doc = BuildCatalogue(Case.Spec);
		const CatalogueResult Result = LoadCatalogue(Doc.Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == (Case.bFastener ? Doc.FastenerLines : Doc.JointLines)[1]);
		CHECK_MESSAGE(Contains(Result.Error.Message, Case.MustName), "the message must name " << Case.MustName);
		CHECK_MESSAGE(Contains(Result.Error.Message, Case.RowName), "the message must name the row " << Case.RowName);
	}
}

TEST_CASE("Catalogue.Loader.AVersionOtherThanOneIsRefusedAtItsLine")
{
	for (const char* Version : {"2", "0", "\"1\""})
	{
		CatalogueSpec Spec = ValidCatalogueSpec();
		Spec.Version = Version;
		const BuiltCatalogue Doc = BuildCatalogue(Spec);
		const CatalogueResult Result = LoadCatalogue(Doc.Json);
		INFO("version " << Version << "; line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == Doc.VersionLine);
		CHECK(Contains(Result.Error.Message, "version"));
	}
}

TEST_CASE("Catalogue.Fastener.ProfileIsCountTimesCapacityOverAreaWithNoFriction")
{
	FastenerRow Row;
	Row.Name = "screw";
	Row.ShearPerPlaneN = 4.83 * NPerKN;
	Row.WithdrawalN = 1.74 * NPerKN;
	Row.BearingStrengthPa = 3.75 * PaPerMPa;
	const int Count = 2;
	const double AreaM2 = 0.01;
	const StrengthProfile Profile = FastenerProfile(Row, Count, AreaM2);
	CHECK(Profile.FrictionCoefficient == 0.0);
	CHECK(Profile.ShearCohesionPa == Count * Row.ShearPerPlaneN / AreaM2);
	CHECK(Profile.MaxShearStrengthPa == Count * Row.ShearPerPlaneN / AreaM2);
	CHECK(Profile.TensileStrengthPa == Count * Row.WithdrawalN / AreaM2);
	CHECK(Profile.CompressiveStrengthPa == Row.BearingStrengthPa);
}
