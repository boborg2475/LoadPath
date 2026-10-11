#include "StructureFileTestSupport.h"

#include "doctest.h"

#include <string>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
void CheckSameVec3(const Vec3& Got, const Vec3& Want, const char* What)
{
	INFO(doctest::String(What));
	CHECK(Got.X == Want.X);
	CHECK(Got.Y == Want.Y);
	CHECK(Got.Z == Want.Z);
}

void CheckSameStructure(const LoadedStructure& Got, const LoadedStructure& Want)
{
	CHECK(Got.Model.LiveBodyCount() == Want.Model.LiveBodyCount());
	CHECK(Got.Model.LiveInterfaceCount() == Want.Model.LiveInterfaceCount());
	CHECK(Got.ProfileNames.size() == Want.ProfileNames.size());

	for (std::uint32_t Index = 0; Index < Want.ProfileNames.size(); ++Index)
	{
		const std::string Name = Want.ProfileNames[Index];
		INFO("profile " << Name);
		const ProfileId GotId = ProfileNamed(Got, Name);
		CHECK(Got.Model.IsValid(GotId));
		const StrengthProfile& G = Got.Model.Profile(GotId);
		const StrengthProfile& W = Want.Model.Profile(ProfileId{Index, 1u});
		CHECK(G.CompressiveStrengthPa == W.CompressiveStrengthPa);
		CHECK(G.ShearCohesionPa == W.ShearCohesionPa);
		CHECK(G.TensileStrengthPa == W.TensileStrengthPa);
		CHECK(G.FrictionCoefficient == W.FrictionCoefficient);
		CHECK(G.MaxShearStrengthPa == W.MaxShearStrengthPa);
		CHECK(G.NormalStiffnessPaPerM == W.NormalStiffnessPaPerM);
		CHECK(G.TangentialStiffnessPaPerM == W.TangentialStiffnessPaPerM);
	}

	for (const BodyId& WantId : Want.Model.LiveBodies())
	{
		const std::string Name = NameOf(Want.BodyNames, WantId.Index);
		INFO("body " << Name);
		const BodyId GotId = BodyNamed(Got, Name);
		CHECK(Got.Model.IsLive(GotId));
		const BodyDesc& G = Got.Model.Body(GotId);
		const BodyDesc& W = Want.Model.Body(WantId);
		CHECK(G.MassKg == W.MassKg);
		CheckSameVec3(G.CentroidM, W.CentroidM, "centroid");
		CHECK(G.bGrounded == W.bGrounded);
	}

	const std::vector<InterfaceId> GotInterfaces = Got.Model.LiveInterfaces();
	const std::vector<InterfaceId> WantInterfaces = Want.Model.LiveInterfaces();
	if (GotInterfaces.size() != WantInterfaces.size())
	{
		return;
	}
	for (std::size_t K = 0; K < WantInterfaces.size(); ++K)
	{
		INFO("live interface " << K);
		const InterfaceRecord& G = Got.Model.Interface(GotInterfaces[K]);
		const InterfaceRecord& W = Want.Model.Interface(WantInterfaces[K]);
		CHECK(NameOf(Got.BodyNames, G.BodyA.Index) == NameOf(Want.BodyNames, W.BodyA.Index));
		CHECK(NameOf(Got.BodyNames, G.BodyB.Index) == NameOf(Want.BodyNames, W.BodyB.Index));
		CheckSameVec3(G.NormalAToB, W.NormalAToB, "normal");
		CheckSameVec3(G.CentreM, W.CentreM, "centre");
		CheckSameVec3(G.AxisU, W.AxisU, "axis U");
		CheckSameVec3(G.AxisV, W.AxisV, "axis V");
		CHECK(G.HalfExtentUM == W.HalfExtentUM);
		CHECK(G.HalfExtentVM == W.HalfExtentVM);
		CHECK(G.AreaM2 == W.AreaM2);
		CHECK(NameOf(Got.ProfileNames, G.Profile.Index) == NameOf(Want.ProfileNames, W.Profile.Index));
		CHECK(G.State == W.State);
	}
}

struct HandBuiltStructure
{
	LoadedStructure Loaded;
	BodyId B0;
	BodyId B1;
	BodyId B2;
};

HandBuiltStructure MakeHandBuilt()
{
	HandBuiltStructure H;
	LoadedStructure& L = H.Loaded;

	StrengthProfile Uncapped = DistinctFieldProfile();
	Uncapped.CompressiveStrengthPa = 1.0 / 3.0 * 1.0e7;
	Uncapped.MaxShearStrengthPa = Infinity;
	const Result<ProfileId> Capped = L.Model.AddProfile(DistinctFieldProfile());
	const Result<ProfileId> NoCap = L.Model.AddProfile(Uncapped);
	REQUIRE(Capped.Ok());
	REQUIRE(NoCap.Ok());
	L.ProfileNames = {"capped", "uncapped"};

	BodyDesc Massless = BrickAt(Vec3{BrickLengthM, 0.0, StackedBrickCentroidM(1).Z});
	Massless.MassKg = 0.0;
	BodyDesc Odd = BrickAt(Vec3{0.1 + 0.2, -1.0 / 7.0, StackedBrickCentroidM(1).Z});
	Odd.MassKg = 2.0 / 3.0;
	const Result<BodyId> B0 = L.Model.AddBody(BrickAt(StackedBrickCentroidM(0), true));
	const Result<BodyId> B1 = L.Model.AddBody(Odd);
	const Result<BodyId> B2 = L.Model.AddBody(Massless);
	REQUIRE(B0.Ok());
	REQUIRE(B1.Ok());
	REQUIRE(B2.Ok());
	H.B0 = B0.Value;
	H.B1 = B1.Value;
	H.B2 = B2.Value;
	L.BodyNames = {"b0", "b1", "b2"};

	InterfaceDesc Side = BedJoint(H.B0, H.B2, 1, NoCap.Value);
	Side.NormalAToB = Vec3{0.0, -1.0, 0.0};
	Side.AxisU = Vec3{1.0, 0.0, 0.0};
	Side.AxisV = Vec3{0.0, 0.0, 1.0};
	Side.HalfExtentUM = 1.0 / 3.0 * BrickLengthM;
	REQUIRE(L.Model.AddInterface(BedJoint(H.B0, H.B1, 1, Capped.Value)).Ok());
	REQUIRE(L.Model.AddInterface(HeadJointAlongX(H.B1, H.B2, 1, NoCap.Value)).Ok());
	REQUIRE(L.Model.AddInterface(Side).Ok());
	L.InterfaceNames = {"j0", "j1", "j2"};
	return H;
}

struct FieldErrorCase
{
	const char* Name;
	char Kind;
	std::size_t Index;
	const char* Key;
	const char* Value;
	const char* ElementName;
};

const FieldErrorCase FieldErrors[] = {
	{"missing mass_kg", 'b', 1, "mass_kg", nullptr, "b1"},
	{"missing a", 'i', 0, "a", nullptr, "interface"},
	{"missing b", 'i', 0, "b", nullptr, "interface"},
	{"missing normal", 'i', 0, "normal", nullptr, "interface"},
	{"missing centre_m", 'i', 0, "centre_m", nullptr, "interface"},
	{"missing u", 'i', 0, "u", nullptr, "interface"},
	{"missing v", 'i', 0, "v", nullptr, "interface"},
	{"missing half_u_m", 'i', 0, "half_u_m", nullptr, "interface"},
	{"missing half_v_m", 'i', 0, "half_v_m", nullptr, "interface"},
	{"missing profile", 'i', 0, "profile", nullptr, "interface"},
	{"mass_kg is a string", 'b', 1, "mass_kg", "\"heavy\"", "b1"},
	{"grounded is a number", 'b', 1, "grounded", "1", "b1"},
	{"compressive_pa is a string", 'p', 0, "compressive_pa", "\"strong\"", "mortar"},
	{"half_u_m is an array", 'i', 0, "half_u_m", "[0.1]", "interface"},
	{"centroid_m has 2 elements", 'b', 1, "centroid_m", "[0, 0]", "b1"},
	{"centroid_m has 4 elements", 'b', 1, "centroid_m", "[0, 0, 0.0975, 1]", "b1"},
	{"normal has 2 elements", 'i', 0, "normal", "[0, 1]", "interface"},
	{"normal has 4 elements", 'i', 0, "normal", "[0, 0, 1, 0]", "interface"},
	{"u has an element that is a string", 'i', 0, "u", "[1, \"0\", 0]", "interface"},
};

struct ReferenceErrorCase
{
	const char* Name;
	char Kind;
	std::size_t Index;
	const char* Key;
	const char* Value;
	const char* MustName;
};

const ReferenceErrorCase ReferenceErrors[] = {
	{"duplicate body name", 'b', 1, "name", "\"b0\"", "b0"},
	{"unknown body in a", 'i', 0, "a", "\"b9\"", "b9"},
	{"unknown body in b", 'i', 0, "b", "\"b9\"", "b9"},
	{"unknown profile", 'i', 0, "profile", "\"steel\"", "steel"},
};

struct RefusalCase
{
	const char* Name;
	char Kind;
	std::size_t Index;
	const char* Key;
	const char* Value;
	const char* ElementName;
	const char* ErrorName;
};

const RefusalCase Refusals[] = {
	{"negative mass", 'b', 1, "mass_kg", "-1", "b1", "NegativeMass"},
	{"zero normal", 'i', 0, "normal", "[0, 0, 0]", nullptr, "DegenerateNormal"},
	{"negative compressive strength", 'p', 0, "compressive_pa", "-1", "mortar", "NegativeStrength"},
};

struct SyntaxCase
{
	const char* Name;
	const char* Json;
	int Line;
};

const SyntaxCase SyntaxErrors[] = {
	{"unterminated string", "{\n  \"profiles\": [\n    { \"name\": \"mortar", 3},
	{"trailing comma in array", "{\n  \"bodies\": [\n    { \"name\": \"b0\", \"mass_kg\": 1 }, ]\n}", 3},
	{"trailing comma in object", "{\n  \"bodies\": [\n    { \"name\": \"b0\", \"mass_kg\": 1, }\n  ]\n}", 3},
	{"bare word", "{\n  \"bodies\": [\n    { \"name\": \"b0\", \"mass_kg\": 1, \"grounded\": yes }\n  ]\n}", 3},
	{"missing colon", "{\n  \"bodies\": [\n    { \"name\" \"b0\", \"mass_kg\": 1 }\n  ]\n}", 3},
	{"unclosed brackets at end of input", "{\n  \"bodies\": [\n    { \"name\": \"b0\", \"mass_kg\": 1 }", 3},
	{"extra closing bracket after the root", "{\n  \"bodies\": []\n}]", 3},
};

const char* const ForbiddenNumbers[] = {"01", ".5", "1.", "+1", "-", "1e", "--1"};
}

TEST_CASE("File.Load.ExampleDocumentReadsBackEveryField")
{
	const std::optional<LoadedStructure> Loaded = LoadOk(ExampleDocument);
	if (!Loaded)
	{
		return;
	}
	const LoadedStructure& L = *Loaded;

	CHECK(L.Model.LiveBodyCount() == 2u);
	CHECK(L.Model.LiveInterfaceCount() == 1u);
	CHECK(L.BodyNames == std::vector<std::string>{"b0", "b1"});
	CHECK(L.ProfileNames == std::vector<std::string>{"mortar"});
	CHECK(L.InterfaceNames.size() == 1u);

	const ProfileId Mortar = ProfileNamed(L, "mortar");
	CHECK(L.Model.IsValid(Mortar));
	const StrengthProfile& P = L.Model.Profile(Mortar);
	CHECK(P.CompressiveStrengthPa == 1e7);
	CHECK(P.ShearCohesionPa == 9e5);
	CHECK(P.TensileStrengthPa == 7e5);
	CHECK(P.FrictionCoefficient == 0.75);
	CHECK(P.MaxShearStrengthPa == 2e6);
	CHECK(P.NormalStiffnessPaPerM == 2e10);
	CHECK(P.TangentialStiffnessPaPerM == 1e10);

	const BodyId B0 = BodyNamed(L, "b0");
	const BodyId B1 = BodyNamed(L, "b1");
	CHECK(L.Model.IsLive(B0));
	CHECK(L.Model.IsLive(B1));
	CHECK(L.Model.Body(B0).MassKg == 2.72);
	CheckSameVec3(L.Model.Body(B0).CentroidM, Vec3{0.0, 0.0, 0.0325}, "b0 centroid");
	CHECK_FALSE(L.Model.Body(B0).bGrounded);
	CHECK(L.Model.Body(B1).MassKg == 3.1);
	CheckSameVec3(L.Model.Body(B1).CentroidM, Vec3{0.0, 0.0, 0.0975}, "b1 centroid");
	CHECK(L.Model.Body(B1).bGrounded);

	const std::vector<InterfaceId> Interfaces = L.Model.LiveInterfaces();
	if (Interfaces.size() != 1u)
	{
		return;
	}
	const InterfaceRecord& J = L.Model.Interface(Interfaces[0]);
	CHECK(SameHandle(J.BodyA, B0));
	CHECK(SameHandle(J.BodyB, B1));
	CheckSameVec3(J.NormalAToB, Vec3{0.0, 0.0, 2.0 / 2.0}, "normal");
	CheckSameVec3(J.CentreM, Vec3{0.0, 0.0, 0.065}, "centre");
	CheckSameVec3(J.AxisU, Vec3{1.0, 0.0, 0.0}, "axis U");
	CheckSameVec3(J.AxisV, Vec3{0.0, 1.0, 0.0}, "axis V");
	CHECK(J.HalfExtentUM == 0.1075);
	CHECK(J.HalfExtentVM == 0.05125);
	CHECK(J.AreaM2 == 4.0 * 0.1075 * 0.05125);
	CHECK(SameHandle(J.Profile, Mortar));
	CHECK(J.State == InterfaceState::Intact);
}

TEST_CASE("File.Load.DeclarationOrderIsFree")
{
	DocSpec Spec = ExampleSpec();
	Spec.Order = "ibp";
	const std::optional<LoadedStructure> Loaded = LoadOk(Build(Spec).Json);
	if (!Loaded)
	{
		return;
	}
	const LoadedStructure& L = *Loaded;

	CHECK(L.Model.LiveBodyCount() == 2u);
	CHECK(L.Model.LiveInterfaceCount() == 1u);
	CHECK(L.ProfileNames == std::vector<std::string>{"mortar"});
	const std::vector<InterfaceId> Interfaces = L.Model.LiveInterfaces();
	if (Interfaces.size() != 1u)
	{
		return;
	}
	const InterfaceRecord& J = L.Model.Interface(Interfaces[0]);
	CHECK(NameOf(L.BodyNames, J.BodyA.Index) == "b0");
	CHECK(NameOf(L.BodyNames, J.BodyB.Index) == "b1");
	CHECK(NameOf(L.ProfileNames, J.Profile.Index) == "mortar");
}

TEST_CASE("File.Load.MissingOptionalFieldsTakeTheStructDefaults")
{
	DocSpec Spec;
	Spec.Profiles.push_back({{"name", "\"lime\""}, {"compressive_pa", "2e6"}});
	Spec.Bodies.push_back({{"name", "\"b0\""}, {"mass_kg", "1.5"}, {"centroid_m", "[0, 0, 0.0325]"}});
	const std::optional<LoadedStructure> Loaded = LoadOk(Build(Spec).Json);
	if (!Loaded)
	{
		return;
	}
	const LoadedStructure& L = *Loaded;

	const StrengthProfile Defaults{};
	const ProfileId Lime = ProfileNamed(L, "lime");
	CHECK(L.Model.IsValid(Lime));
	const StrengthProfile& P = L.Model.Profile(Lime);
	CHECK(P.CompressiveStrengthPa == 2e6);
	CHECK(P.ShearCohesionPa == Defaults.ShearCohesionPa);
	CHECK(P.TensileStrengthPa == Defaults.TensileStrengthPa);
	CHECK(P.FrictionCoefficient == Defaults.FrictionCoefficient);
	CHECK(P.MaxShearStrengthPa == Defaults.MaxShearStrengthPa);
	CHECK(P.NormalStiffnessPaPerM == Defaults.NormalStiffnessPaPerM);
	CHECK(P.TangentialStiffnessPaPerM == Defaults.TangentialStiffnessPaPerM);

	const BodyId B0 = BodyNamed(L, "b0");
	CHECK(L.Model.IsLive(B0));
	CHECK(L.Model.Body(B0).MassKg == 1.5);
	CHECK(L.Model.Body(B0).bGrounded == BodyDesc{}.bGrounded);
}

TEST_CASE("File.Load.FieldErrorsNameTheElementAtItsOpeningLine")
{
	for (const FieldErrorCase& Case : FieldErrors)
	{
		INFO(doctest::String(Case.Name));
		const DocSpec Spec = Case.Value == nullptr ? Without(ExampleSpec(), Case.Kind, Case.Index, Case.Key)
			: WithValue(ExampleSpec(), Case.Kind, Case.Index, Case.Key, Case.Value);
		const BuiltDoc Doc = Build(Spec);
		const LoadResult Result = LoadStructure(Doc.Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == LinesOf(Doc, Case.Kind)[Case.Index]);
		CHECK_MESSAGE(Contains(Result.Error.Message, Case.Key), "the message must name the field");
		CHECK_MESSAGE(Contains(Result.Error.Message, Case.ElementName), "the message must name the element");
	}
}

TEST_CASE("File.Load.DuplicateAndUnknownNamesAreErrorsAtTheElement")
{
	for (const ReferenceErrorCase& Case : ReferenceErrors)
	{
		INFO(doctest::String(Case.Name));
		const BuiltDoc Doc = Build(WithValue(ExampleSpec(), Case.Kind, Case.Index, Case.Key, Case.Value));
		const LoadResult Result = LoadStructure(Doc.Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == LinesOf(Doc, Case.Kind)[Case.Index]);
		CHECK(Contains(Result.Error.Message, Case.MustName));
	}

	DocSpec Spec = ExampleSpec();
	Spec.Profiles.push_back({{"name", "\"mortar\""}, {"compressive_pa", "5e6"}});
	const BuiltDoc Doc = Build(Spec);
	const LoadResult Result = LoadStructure(Doc.Json);
	INFO("duplicate profile name; line " << Result.Error.Line << ": " << Result.Error.Message);
	CHECK_FALSE(Result.Ok());
	CHECK(Result.Error.Line == Doc.ProfileLines[1]);
	CHECK(Contains(Result.Error.Message, "mortar"));
}

TEST_CASE("File.Load.AddRefusalNamesTheElementAndTheModelError")
{
	for (const RefusalCase& Case : Refusals)
	{
		INFO(doctest::String(Case.Name));
		const LoadResult Result = LoadStructure(Build(WithValue(ExampleSpec(), Case.Kind, Case.Index, Case.Key, Case.Value)).Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Contains(Result.Error.Message, Case.ErrorName));
		if (Case.ElementName != nullptr)
		{
			CHECK(Contains(Result.Error.Message, Case.ElementName));
		}
	}
}

TEST_CASE("File.Load.SyntaxErrorsReportTheirLine")
{
	for (const SyntaxCase& Case : SyntaxErrors)
	{
		INFO(doctest::String(Case.Name));
		const LoadResult Result = LoadStructure(Case.Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == Case.Line);
	}
}

TEST_CASE("File.Load.NestingDeeperThan64IsASyntaxError")
{
	for (const int Depth : {65, 100000})
	{
		INFO("array depth " << Depth << " inside the root object");
		std::string Json = "{\"profiles\": ";
		Json.append(static_cast<std::size_t>(Depth), '[');
		Json.append(static_cast<std::size_t>(Depth), ']');
		Json += "}";
		const LoadResult Result = LoadStructure(Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == 1);
	}
}

TEST_CASE("File.Load.JsonNumberFormsParse")
{
	const std::optional<LoadedStructure> Loaded = LoadOk(
		"{ \"profiles\": [ { \"name\": \"p\", \"compressive_pa\": 1e7, \"cohesion_pa\": 0, \"tensile_pa\": 2.5E-3 } ],\n"
		"  \"bodies\": [ { \"name\": \"b0\", \"mass_kg\": 1, \"centroid_m\": [-0.5, 0, 0] } ] }");
	if (!Loaded)
	{
		return;
	}
	const LoadedStructure& L = *Loaded;

	const ProfileId P = ProfileNamed(L, "p");
	const BodyId B0 = BodyNamed(L, "b0");
	CHECK(L.Model.IsValid(P));
	CHECK(L.Model.IsLive(B0));
	CHECK(L.Model.Profile(P).CompressiveStrengthPa == 1e7);
	CHECK(L.Model.Profile(P).ShearCohesionPa == 0.0);
	CHECK(L.Model.Profile(P).TensileStrengthPa == 2.5E-3);
	CHECK(L.Model.Body(B0).MassKg == 1.0);
	CHECK(L.Model.Body(B0).CentroidM.X == -0.5);
}

TEST_CASE("File.Load.NumberFormsJsonForbidsAreSyntaxErrors")
{
	for (const char* Form : ForbiddenNumbers)
	{
		INFO("mass_kg: " << std::string(Form));
		const std::string Json = std::string("{\n  \"bodies\": [ { \"name\": \"b0\", \"mass_kg\": ") + Form + " } ]\n}";
		const LoadResult Result = LoadStructure(Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == 2);
	}
}

TEST_CASE("File.Load.StringEscapesDecodeAndRoundTrip")
{
	const std::string Escaped = R"({ "profiles": [ { "name": "m", "compressive_pa": 1e7 } ],
  "bodies": [ { "name": "b\"A", "mass_kg": 1, "centroid_m": [0, 0, 0] },
              { "name": "c", "mass_kg": 1, "centroid_m": [0, 0, 0.065] } ],
  "interfaces": [ { "a": "b\"A", "b": "c", "normal": [0, 0, 1], "centre_m": [0, 0, 0.0325],
                    "u": [1, 0, 0], "v": [0, 1, 0], "half_u_m": 0.1, "half_v_m": 0.05, "profile": "m" } ] })";
	const std::string Decoded = "b\"A";

	const std::optional<LoadedStructure> Loaded = LoadOk(Escaped);
	if (!Loaded)
	{
		return;
	}
	CHECK(Loaded->Model.IsLive(BodyNamed(*Loaded, Decoded)));
	CHECK(Loaded->Model.LiveInterfaceCount() == 1u);

	const std::optional<LoadedStructure> Reloaded = LoadOk(WriteStructure(*Loaded));
	if (!Reloaded)
	{
		return;
	}
	CHECK(Reloaded->Model.IsLive(BodyNamed(*Reloaded, Decoded)));
	CHECK(Reloaded->Model.LiveInterfaceCount() == 1u);
}

TEST_CASE("File.Load.DuplicateKeysTakeTheLastValue")
{
	const std::optional<LoadedStructure> Loaded =
		LoadOk(R"({ "bodies": [ { "name": "b0", "mass_kg": 1, "centroid_m": [0, 0, 0], "mass_kg": 3 } ] })");
	if (!Loaded)
	{
		return;
	}

	const BodyId B0 = BodyNamed(*Loaded, "b0");
	CHECK(Loaded->Model.IsLive(B0));
	CHECK(Loaded->Model.Body(B0).MassKg == 3.0);
}

TEST_CASE("File.Load.EmptyObjectLoadsAndTopLevelArrayIsRefused")
{
	const LoadResult Array = LoadStructure("[]");
	CHECK_FALSE(Array.Ok());

	const std::optional<LoadedStructure> Empty = LoadOk("{}");
	if (!Empty)
	{
		return;
	}
	CHECK(Empty->Model.LiveBodyCount() == 0u);
	CHECK(Empty->Model.LiveInterfaceCount() == 0u);
	CHECK(Empty->BodyNames.empty());
	CHECK(Empty->InterfaceNames.empty());
	CHECK(Empty->ProfileNames.empty());
}

TEST_CASE("File.Write.LoadWriteLoadEqualsLoadFieldForField")
{
	const std::optional<LoadedStructure> First = LoadOk(ExampleDocument);
	if (!First)
	{
		return;
	}
	const std::optional<LoadedStructure> Second = LoadOk(WriteStructure(*First));
	if (!Second)
	{
		return;
	}
	CheckSameStructure(*Second, *First);
}

TEST_CASE("File.Write.HandBuiltStructureReloadsFieldForField")
{
	const HandBuiltStructure H = MakeHandBuilt();
	const std::optional<LoadedStructure> Reloaded = LoadOk(WriteStructure(H.Loaded));
	if (!Reloaded)
	{
		return;
	}
	CheckSameStructure(*Reloaded, H.Loaded);
}

TEST_CASE("File.Write.RemovedBodyAndItsInterfacesAreNotWritten")
{
	HandBuiltStructure H = MakeHandBuilt();
	REQUIRE(H.Loaded.Model.RemoveBody(H.B1));

	const std::optional<LoadedStructure> Reloaded = LoadOk(WriteStructure(H.Loaded));
	if (!Reloaded)
	{
		return;
	}
	CHECK(Reloaded->BodyNames == std::vector<std::string>{"b0", "b2"});
	CHECK_FALSE(Reloaded->Model.IsLive(BodyNamed(*Reloaded, "b1")));
	CheckSameStructure(*Reloaded, H.Loaded);
	const std::vector<InterfaceId> Interfaces = Reloaded->Model.LiveInterfaces();
	CHECK(Interfaces.size() == 1u);
	if (Interfaces.size() != 1u)
	{
		return;
	}
	const InterfaceRecord& J = Reloaded->Model.Interface(Interfaces[0]);
	CHECK(NameOf(Reloaded->BodyNames, J.BodyA.Index) == "b0");
	CHECK(NameOf(Reloaded->BodyNames, J.BodyB.Index) == "b2");
}

TEST_CASE("File.Write.OutputContainsEveryFieldName")
{
	const HandBuiltStructure H = MakeHandBuilt();
	const std::string Text = WriteStructure(H.Loaded);
	const char* const Keys[] = {"profiles", "bodies", "interfaces", "name", "compressive_pa", "cohesion_pa", "tensile_pa",
		"friction", "max_shear_pa", "normal_stiffness_pa_per_m", "tangential_stiffness_pa_per_m", "mass_kg", "centroid_m",
		"grounded", "a", "b", "normal", "centre_m", "u", "v", "half_u_m", "half_v_m", "profile"};
	for (const char* Key : Keys)
	{
		INFO(doctest::String(Key));
		CHECK(Contains(Text, std::string("\"") + Key + "\""));
	}
}
