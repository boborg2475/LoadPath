#include "ModelTestSupport.h"
#include "StructureFileTestSupport.h"

#include "loadpath/StructureFile.h"

#include "doctest.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct Column
{
	Structure Model;
	ProfileId Profile;
	BodyId B0;
	BodyId B1;
	BodyId B2;
	BodyId B3;
	InterfaceId J01;
	InterfaceId J12;
	InterfaceId J23;
};

Column MakeColumn()
{
	Column F;
	F.Profile = F.Model.AddProfile(DistinctFieldProfile()).Value;
	F.B0 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(0), true)).Value;
	F.B1 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(1))).Value;
	F.B2 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(2))).Value;
	F.B3 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(3))).Value;
	F.J01 = F.Model.AddInterface(BedJoint(F.B0, F.B1, 1, F.Profile)).Value;
	F.J12 = F.Model.AddInterface(BedJoint(F.B1, F.B2, 2, F.Profile)).Value;
	F.J23 = F.Model.AddInterface(BedJoint(F.B2, F.B3, 3, F.Profile)).Value;
	return F;
}

void CheckRecord(const Structure& Model, InterfaceId Id, InterfaceState State, std::uint32_t CrackPass, std::uint32_t SeverPass)
{
	const InterfaceRecord& R = Model.Interface(Id);
	INFO("interface " << Id << ": state " << R.State << ", crack " << R.CrackPass << ", sever " << R.SeverPass << "; want " << State << ", " << CrackPass << ", " << SeverPass);
	CHECK(R.State == State);
	CHECK(R.CrackPass == CrackPass);
	CHECK(R.SeverPass == SeverPass);
}

bool Lists(const std::vector<InterfaceId>& Ids, InterfaceId Id)
{
	for (const InterfaceId& Each : Ids)
	{
		if (SameHandle(Each, Id))
		{
			return true;
		}
	}
	return false;
}

void CheckSameProfile(const StrengthProfile& Got, const StrengthProfile& Want)
{
	CHECK(Got.CompressiveStrengthPa == Want.CompressiveStrengthPa);
	CHECK(Got.ShearCohesionPa == Want.ShearCohesionPa);
	CHECK(Got.TensileStrengthPa == Want.TensileStrengthPa);
	CHECK(Got.FrictionCoefficient == Want.FrictionCoefficient);
	CHECK(Got.MaxShearStrengthPa == Want.MaxShearStrengthPa);
	CHECK(Got.NormalStiffnessPaPerM == Want.NormalStiffnessPaPerM);
	CHECK(Got.TangentialStiffnessPaPerM == Want.TangentialStiffnessPaPerM);
}

StrengthProfile CrackedForm(StrengthProfile Profile)
{
	Profile.TensileStrengthPa = 0.0;
	Profile.ShearCohesionPa = 0.0;
	return Profile;
}

InterfaceId InterfaceNamed(const LoadedStructure& Loaded, const std::string& Name)
{
	for (std::uint32_t Index = 0; Index < Loaded.InterfaceNames.size(); ++Index)
	{
		if (Loaded.InterfaceNames[Index] == Name)
		{
			return InterfaceId{Index, 1u};
		}
	}
	return InterfaceId::Invalid();
}
}

TEST_CASE("Model.Structure.CrackOnlyFromIntactStampsOnce")
{
	Column F = MakeColumn();
	CHECK(F.Model.Crack(F.J12, 4));
	CheckRecord(F.Model, F.J12, InterfaceState::Cracked, 4, 0);
	CHECK_FALSE(F.Model.Crack(F.J12, 5));
	CheckRecord(F.Model, F.J12, InterfaceState::Cracked, 4, 0);
	CheckRecord(F.Model, F.J01, InterfaceState::Intact, 0, 0);
}

TEST_CASE("Model.Structure.SeverFromIntactOrCrackedKeepsTheCrackStamp")
{
	Column F = MakeColumn();
	CHECK(F.Model.Sever(F.J01, 2));
	CheckRecord(F.Model, F.J01, InterfaceState::Severed, 0, 2);
	CHECK(F.Model.Crack(F.J12, 1));
	CHECK(F.Model.Sever(F.J12, 3));
	CheckRecord(F.Model, F.J12, InterfaceState::Severed, 1, 3);
	CHECK_FALSE(F.Model.Sever(F.J12, 4));
	CHECK_FALSE(F.Model.Crack(F.J12, 4));
	CheckRecord(F.Model, F.J12, InterfaceState::Severed, 1, 3);
}

TEST_CASE("Model.Structure.CrackAndSeverRefuseARemovedInterface")
{
	Column F = MakeColumn();
	REQUIRE(F.Model.RemoveBody(F.B3));
	CHECK_FALSE(F.Model.Crack(F.J23, 1));
	CHECK_FALSE(F.Model.Sever(F.J23, 1));
	CheckRecord(F.Model, F.J23, InterfaceState::Removed, 0, 0);
	CHECK(F.Model.HighestPass() == 0u);
	CHECK(F.Model.Crack(F.J01, 1));
	CheckRecord(F.Model, F.J01, InterfaceState::Cracked, 1, 0);
}

TEST_CASE("Model.Structure.PassZeroIsRefused")
{
	Column F = MakeColumn();
	CHECK_FALSE(F.Model.Crack(F.J01, 0));
	CHECK_FALSE(F.Model.Sever(F.J01, 0));
	CheckRecord(F.Model, F.J01, InterfaceState::Intact, 0, 0);
	CHECK(F.Model.Crack(F.J12, 1));
	CHECK_FALSE(F.Model.Sever(F.J12, 0));
	CheckRecord(F.Model, F.J12, InterfaceState::Cracked, 1, 0);
	CHECK(F.Model.HighestPass() == 1u);
}

TEST_CASE("Model.Structure.HighestPassIsTheLargestStampWritten")
{
	Column F = MakeColumn();
	CHECK(F.Model.HighestPass() == 0u);
	CHECK(F.Model.Crack(F.J12, 3));
	CHECK(F.Model.HighestPass() == 3u);
	CHECK(F.Model.Sever(F.J01, 2));
	CHECK(F.Model.HighestPass() == 3u);
	CHECK(F.Model.Sever(F.J12, 7));
	CHECK(F.Model.HighestPass() == 7u);
	CHECK_FALSE(F.Model.Crack(F.J01, 9));
	CHECK(F.Model.HighestPass() == 7u);
	REQUIRE(F.Model.RemoveBody(F.B2));
	CHECK(F.Model.HighestPass() == 7u);
}

TEST_CASE("Model.Structure.CrackedIsLiveAndSeveredIsNot")
{
	Column F = MakeColumn();
	CHECK(F.Model.Crack(F.J01, 1));
	CHECK(F.Model.Sever(F.J12, 1));
	REQUIRE(F.Model.RemoveBody(F.B3));
	CHECK(F.Model.IsLive(F.J01));
	CHECK_FALSE(F.Model.IsLive(F.J12));
	CHECK_FALSE(F.Model.IsLive(F.J23));
	const std::vector<InterfaceId> Live = F.Model.LiveInterfaces();
	INFO("live " << Describe(Live));
	CHECK(Live.size() == 1u);
	CHECK(Lists(Live, F.J01));
	CHECK(F.Model.LiveInterfaceCount() == 1u);
	CHECK(Lists(F.Model.InterfacesOf(F.B1), F.J01));
	CHECK_FALSE(Lists(F.Model.InterfacesOf(F.B1), F.J12));
	CHECK(F.Model.InterfacesOf(F.B2).empty());
}

TEST_CASE("Model.Structure.RemoveBodyKeepsASeveredInterfaceAndItsStamp")
{
	Column F = MakeColumn();
	CHECK(F.Model.Crack(F.J12, 1));
	CHECK(F.Model.Crack(F.J01, 1));
	CHECK(F.Model.Sever(F.J01, 2));
	REQUIRE(F.Model.RemoveBody(F.B1));
	REQUIRE(F.Model.RemoveBody(F.B3));
	CheckRecord(F.Model, F.J01, InterfaceState::Severed, 1, 2);
	CheckRecord(F.Model, F.J12, InterfaceState::Removed, 1, 0);
	CheckRecord(F.Model, F.J23, InterfaceState::Removed, 0, 0);
}

TEST_CASE("Model.Structure.EffectiveProfileDropsTensionAndCohesionOnceCracked")
{
	Column F = MakeColumn();
	const StrengthProfile Raw = DistinctFieldProfile();
	{
		INFO("intact");
		CheckSameProfile(F.Model.EffectiveProfile(F.J01), Raw);
	}
	CHECK(F.Model.Crack(F.J01, 1));
	{
		INFO("cracked");
		CheckSameProfile(F.Model.EffectiveProfile(F.J01), CrackedForm(Raw));
	}
	CHECK(F.Model.Sever(F.J12, 1));
	{
		INFO("severed");
		CheckSameProfile(F.Model.EffectiveProfile(F.J12), CrackedForm(Raw));
	}
	REQUIRE(F.Model.RemoveBody(F.B3));
	{
		INFO("removed");
		CheckSameProfile(F.Model.EffectiveProfile(F.J23), CrackedForm(Raw));
	}
}

TEST_CASE("Model.StructureFile.RoundTripsInterfaceStateAndStamps")
{
	Column F = MakeColumn();
	CHECK(F.Model.Crack(F.J12, 2));
	CHECK(F.Model.Crack(F.J23, 1));
	CHECK(F.Model.Sever(F.J23, 3));
	LoadedStructure Original;
	Original.Model = F.Model;
	Original.BodyNames = {"b0", "b1", "b2", "b3"};
	Original.InterfaceNames = {"j01", "j12", "j23"};
	Original.ProfileNames = {"mortar"};
	const std::string Text = WriteStructure(Original);
	INFO(Text);
	const std::optional<LoadedStructure> Reloaded = LoadOk(Text);
	if (!Reloaded)
	{
		return;
	}
	const InterfaceId J01 = InterfaceNamed(*Reloaded, "j01");
	const InterfaceId J12 = InterfaceNamed(*Reloaded, "j12");
	const InterfaceId J23 = InterfaceNamed(*Reloaded, "j23");
	CheckRecord(Reloaded->Model, J01, InterfaceState::Intact, 0, 0);
	CheckRecord(Reloaded->Model, J12, InterfaceState::Cracked, 2, 0);
	CheckRecord(Reloaded->Model, J23, InterfaceState::Severed, 1, 3);
	CHECK(Reloaded->Model.HighestPass() == 3u);
}

TEST_CASE("Model.StructureFile.AFileWithoutInterfaceStateReadsIntactUnstamped")
{
	const std::optional<LoadedStructure> Loaded = LoadOk(ExampleDocument);
	if (!Loaded)
	{
		return;
	}
	const std::vector<InterfaceId> Live = Loaded->Model.LiveInterfaces();
	CHECK(Live.size() == 1u);
	for (const InterfaceId& Id : Live)
	{
		CheckRecord(Loaded->Model, Id, InterfaceState::Intact, 0, 0);
	}
	CHECK(Loaded->Model.HighestPass() == 0u);
}
