#include "ModelTestSupport.h"

#include "doctest.h"

#include <cmath>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct ProfileField
{
	const char* Name;
	double StrengthProfile::*Member;
};

constexpr ProfileField AllFields[] = {
	{"CompressiveStrengthPa", &StrengthProfile::CompressiveStrengthPa},
	{"ShearCohesionPa", &StrengthProfile::ShearCohesionPa},
	{"TensileStrengthPa", &StrengthProfile::TensileStrengthPa},
	{"FrictionCoefficient", &StrengthProfile::FrictionCoefficient},
	{"MaxShearStrengthPa", &StrengthProfile::MaxShearStrengthPa},
	{"NormalStiffnessPaPerM", &StrengthProfile::NormalStiffnessPaPerM},
	{"TangentialStiffnessPaPerM", &StrengthProfile::TangentialStiffnessPaPerM},
};

constexpr ProfileField FieldsThatMustBeFinite[] = {
	{"CompressiveStrengthPa", &StrengthProfile::CompressiveStrengthPa},
	{"ShearCohesionPa", &StrengthProfile::ShearCohesionPa},
	{"TensileStrengthPa", &StrengthProfile::TensileStrengthPa},
	{"FrictionCoefficient", &StrengthProfile::FrictionCoefficient},
	{"NormalStiffnessPaPerM", &StrengthProfile::NormalStiffnessPaPerM},
	{"TangentialStiffnessPaPerM", &StrengthProfile::TangentialStiffnessPaPerM},
};

constexpr ProfileField StrengthFields[] = {
	{"CompressiveStrengthPa", &StrengthProfile::CompressiveStrengthPa},
	{"ShearCohesionPa", &StrengthProfile::ShearCohesionPa},
	{"TensileStrengthPa", &StrengthProfile::TensileStrengthPa},
	{"MaxShearStrengthPa", &StrengthProfile::MaxShearStrengthPa},
};

constexpr ProfileField StrengthAndFrictionFields[] = {
	{"CompressiveStrengthPa", &StrengthProfile::CompressiveStrengthPa},
	{"ShearCohesionPa", &StrengthProfile::ShearCohesionPa},
	{"TensileStrengthPa", &StrengthProfile::TensileStrengthPa},
	{"FrictionCoefficient", &StrengthProfile::FrictionCoefficient},
	{"MaxShearStrengthPa", &StrengthProfile::MaxShearStrengthPa},
};

struct NamedProfile
{
	std::string Name;
	StrengthProfile Desc;
};

std::vector<NamedProfile> RefusedProfiles()
{
	std::vector<NamedProfile> Profiles;
	for (const ProfileField& Field : AllFields)
	{
		StrengthProfile Desc = DistinctFieldProfile();
		Desc.*Field.Member = NaN;
		Profiles.push_back({std::string("NaN ") + Field.Name, Desc});
	}
	for (const ProfileField& Field : FieldsThatMustBeFinite)
	{
		StrengthProfile Desc = DistinctFieldProfile();
		Desc.*Field.Member = Infinity;
		Profiles.push_back({std::string("infinite ") + Field.Name, Desc});
	}
	for (const ProfileField& Field : StrengthAndFrictionFields)
	{
		StrengthProfile Desc = DistinctFieldProfile();
		Desc.*Field.Member = -(Desc.*Field.Member);
		Profiles.push_back({std::string("negative ") + Field.Name, Desc});
	}
	StrengthProfile NegativeInfiniteCap = DistinctFieldProfile();
	NegativeInfiniteCap.MaxShearStrengthPa = -Infinity;
	Profiles.push_back({"-infinity MaxShearStrengthPa", NegativeInfiniteCap});
	return Profiles;
}

void CheckRefused(const Structure& Model, const Result<ProfileId>& Added, ModelError Expected)
{
	CHECK(Added.Error == Expected);
	CHECK_FALSE(Added.Ok());
	CHECK_MESSAGE(Added.Value.Generation == 0u, "a refused add must return the invalid handle");
	CHECK_FALSE(Model.IsValid(Added.Value));
}
}

TEST_CASE("Model.Profile.ValidAddReturnsAValidIdAndReadsBackEveryField")
{
	Structure Model;
	const StrengthProfile Desc = DistinctFieldProfile();
	const Result<ProfileId> Added = Model.AddProfile(Desc);

	CHECK(Added.Ok());
	CHECK(Added.Error == ModelError::None);
	CHECK_MESSAGE(Added.Value.Generation != 0u, "an accepted add must return a valid handle");
	CHECK(Model.IsValid(Added.Value));

	const StrengthProfile& Stored = Model.Profile(Added.Value);
	for (const ProfileField& Field : AllFields)
	{
		INFO(doctest::String(Field.Name));
		CHECK(Stored.*Field.Member == Desc.*Field.Member);
	}
}

TEST_CASE("Model.Profile.UnsetShearCapIsAcceptedAndReadsAsPositiveInfinity")
{
	Structure Model;
	StrengthProfile Desc = DistinctFieldProfile();
	Desc.MaxShearStrengthPa = StrengthProfile{}.MaxShearStrengthPa;
	const Result<ProfileId> Added = Model.AddProfile(Desc);

	CHECK_MESSAGE(Added.Error == ModelError::None, "the unset cap is +infinity by design and must not read as non-finite input");
	CHECK(Model.IsValid(Added.Value));
	CHECK(Model.Profile(Added.Value).MaxShearStrengthPa == Infinity);
	CHECK(Model.Profile(Added.Value).CompressiveStrengthPa == Desc.CompressiveStrengthPa);
}

TEST_CASE("Model.Profile.NaNInAnyFieldIsRefusedAsNonFiniteInput")
{
	for (const ProfileField& Field : AllFields)
	{
		INFO(doctest::String(Field.Name));
		Structure Model;
		StrengthProfile Desc = DistinctFieldProfile();
		Desc.*Field.Member = NaN;
		CheckRefused(Model, Model.AddProfile(Desc), ModelError::NonFiniteInput);
	}
}

TEST_CASE("Model.Profile.InfinityInAnyFieldButTheShearCapIsRefusedAsNonFiniteInput")
{
	for (const ProfileField& Field : FieldsThatMustBeFinite)
	{
		INFO(doctest::String(Field.Name));
		Structure Model;
		StrengthProfile Desc = DistinctFieldProfile();
		Desc.*Field.Member = Infinity;
		CheckRefused(Model, Model.AddProfile(Desc), ModelError::NonFiniteInput);
	}
}

TEST_CASE("Model.Profile.NegativeStrengthOrFrictionIsRefused")
{
	for (const ProfileField& Field : StrengthAndFrictionFields)
	{
		INFO(doctest::String(Field.Name));
		Structure Model;
		StrengthProfile Desc = DistinctFieldProfile();
		Desc.*Field.Member = -(Desc.*Field.Member);
		CheckRefused(Model, Model.AddProfile(Desc), ModelError::NegativeStrength);
	}
}

TEST_CASE("Model.Profile.DryJointProfileIsValid")
{
	Structure Model;
	StrengthProfile Desc;
	Desc.CompressiveStrengthPa = DistinctFieldProfile().CompressiveStrengthPa;
	Desc.ShearCohesionPa = 0.0;
	Desc.TensileStrengthPa = 0.0;
	Desc.FrictionCoefficient = 0.7;
	const Result<ProfileId> Added = Model.AddProfile(Desc);

	CHECK(Added.Error == ModelError::None);
	CHECK(Model.IsValid(Added.Value));
	const StrengthProfile& Stored = Model.Profile(Added.Value);
	CHECK(Stored.ShearCohesionPa == 0.0);
	CHECK(Stored.TensileStrengthPa == 0.0);
	CHECK(Stored.FrictionCoefficient == Desc.FrictionCoefficient);
	CHECK(Stored.CompressiveStrengthPa == Desc.CompressiveStrengthPa);
	CHECK(Stored.MaxShearStrengthPa == Infinity);
	CHECK_MESSAGE(Stored.NormalStiffnessPaPerM == 0.0, "zero stiffness is accepted in slice 1");
}

TEST_CASE("Model.Profile.NonFiniteOutranksNegativeWhenBothArePresent")
{
	SUBCASE("negative field before the NaN field")
	{
		Structure Model;
		StrengthProfile Desc = DistinctFieldProfile();
		Desc.CompressiveStrengthPa = -Desc.CompressiveStrengthPa;
		Desc.FrictionCoefficient = NaN;
		CheckRefused(Model, Model.AddProfile(Desc), ModelError::NonFiniteInput);
	}
	SUBCASE("NaN field before the negative field")
	{
		Structure Model;
		StrengthProfile Desc = DistinctFieldProfile();
		Desc.CompressiveStrengthPa = NaN;
		Desc.FrictionCoefficient = -Desc.FrictionCoefficient;
		CheckRefused(Model, Model.AddProfile(Desc), ModelError::NonFiniteInput);
	}
}

TEST_CASE("Model.Profile.NegativeInfinityShearCapIsRefusedAsNonFiniteInput")
{
	Structure Model;
	StrengthProfile Desc = DistinctFieldProfile();
	Desc.MaxShearStrengthPa = -Infinity;
	CheckRefused(Model, Model.AddProfile(Desc), ModelError::NonFiniteInput);
}

TEST_CASE("Model.Profile.RefusedAddChangesNothing")
{
	for (const NamedProfile& Refused : RefusedProfiles())
	{
		INFO(doctest::String(Refused.Name.c_str()));
		Structure Model;
		REQUIRE_FALSE(Model.AddProfile(Refused.Desc).Ok());
		CHECK_MESSAGE(!Model.IsValid(ProfileId{0u, 1u}), "a refused add must not occupy slot 0");
		const Result<ProfileId> Next = Model.AddProfile(DistinctFieldProfile());
		CHECK(Next.Ok());
		CHECK_MESSAGE(SameHandle(Next.Value, ProfileId{0u, 1u}), "the next accepted profile must take slot 0, got " << Next.Value);
	}
}

TEST_CASE("Model.Profile.UnknownHandleReadsNaNStrengths")
{
	Structure Model;
	REQUIRE(Model.AddProfile(DistinctFieldProfile()).Ok());
	const ProfileId Unknown[] = {ProfileId{}, ProfileId{1000u, 1u}, ProfileId{0u, 2u}};
	for (const ProfileId& Id : Unknown)
	{
		INFO(Id);
		for (const ProfileField& Field : StrengthFields)
		{
			INFO(doctest::String(Field.Name));
			CHECK_MESSAGE(std::isnan(Model.Profile(Id).*Field.Member), "a dead read must not yield a usable strength, got " << Model.Profile(Id).*Field.Member);
		}
	}
}
