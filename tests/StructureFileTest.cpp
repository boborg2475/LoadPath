#include "StructureFileTestSupport.h"

#include "doctest.h"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
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
	{"zero normal", 'i', 0, "normal", "[0, 0, 0]", "interface", "DegenerateNormal"},
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
	{"CRLF line endings count one line each", "{\r\n  \"bodies\": [\r\n    { \"name\" \"b0\" }\r\n  ]\r\n}", 3},
	{"escaped newline in a name is not a line", "{\n  \"profiles\": [\n    { \"name\": \"a\\nb\",\n      \"compressive_pa\":\n        yes }\n  ]\n}", 5},
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
		const BuiltDoc Doc = Build(WithValue(ExampleSpec(), Case.Kind, Case.Index, Case.Key, Case.Value));
		const LoadResult Result = LoadStructure(Doc.Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == LinesOf(Doc, Case.Kind)[Case.Index]);
		CHECK(Contains(Result.Error.Message, Case.ErrorName));
		CHECK(Contains(Result.Error.Message, Case.ElementName));
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

namespace
{
std::string Bytes(std::initializer_list<unsigned> Values)
{
	std::string Out;
	for (const unsigned Value : Values)
	{
		Out += static_cast<char>(Value);
	}
	return Out;
}

std::string ProfileNameDocument(const std::string& NameSource)
{
	return "{\n  \"profiles\": [\n    { \"name\": \"" + NameSource + "\", \"compressive_pa\": 1e7 }\n  ]\n}";
}

std::string Esc(const char* After)
{
	return std::string(1, '\\') + After;
}

struct EscapeCase
{
	const char* Name;
	std::string Source;
	std::string Decoded;
};

std::vector<EscapeCase> EscapeCases()
{
	return {
		{"reverse solidus", "a" + Esc("\\") + "b", std::string("a") + '\\' + "b"},
		{"trailing reverse solidus", "a" + Esc("\\"), std::string("a") + '\\'},
		{"solidus", "a" + Esc("/") + "b", "a/b"},
		{"backspace form feed newline return tab", Esc("b") + Esc("f") + Esc("n") + Esc("r") + Esc("t"), "\b\f\n\r\t"},
		{"quote and reverse solidus together", Esc("\"") + Esc("\\") + Esc("\""), std::string("\"") + '\\' + "\""},
		{"u0000", "x" + Esc("u0000") + "y", std::string("x") + '\0' + "y"},
		{"u0041 one byte", Esc("u0041"), "A"},
		{"u007f largest one-byte", Esc("u007f"), Bytes({0x7F})},
		{"u0080 smallest two-byte", Esc("u0080"), Bytes({0xC2, 0x80})},
		{"u00e9 two bytes", Esc("u00e9"), Bytes({0xC3, 0xA9})},
		{"u00E9 upper-case hex", Esc("u00E9"), Bytes({0xC3, 0xA9})},
		{"u07ff largest two-byte", Esc("u07ff"), Bytes({0xDF, 0xBF})},
		{"u0800 smallest three-byte", Esc("u0800"), Bytes({0xE0, 0xA0, 0x80})},
		{"u20ac euro sign", Esc("u20ac"), Bytes({0xE2, 0x82, 0xAC})},
		{"uffff largest BMP", Esc("uffff"), Bytes({0xEF, 0xBF, 0xBF})},
		{"surrogate pair U+1F600", Esc("ud83d") + Esc("ude00"), Bytes({0xF0, 0x9F, 0x98, 0x80})},
		{"surrogate pair U+10000", Esc("ud800") + Esc("udc00"), Bytes({0xF0, 0x90, 0x80, 0x80})},
		{"surrogate pair U+10FFFF", Esc("udbff") + Esc("udfff"), Bytes({0xF4, 0x8F, 0xBF, 0xBF})},
		{"escape between plain text", "left" + Esc("u00e9") + "right", "left" + Bytes({0xC3, 0xA9}) + "right"},
		{"raw UTF-8 passes through", Bytes({0xC3, 0xA9}), Bytes({0xC3, 0xA9})},
		{"raw DEL passes through", Bytes({0x7F}), Bytes({0x7F})},
	};
}

struct MalformedEscapeCase
{
	const char* Name;
	std::string Source;
};

std::vector<MalformedEscapeCase> MalformedEscapes()
{
	return {
		{"invalid escape x", "a" + Esc("x") + "b"},
		{"invalid escape U", Esc("U00e9")},
		{"u with three hex digits", Esc("u00e")},
		{"u with a non-hex digit", Esc("u00g9")},
		{"lone high surrogate before plain text", Esc("ud83d") + "A"},
		{"lone high surrogate before a non-surrogate escape", Esc("ud83d") + Esc("u0041")},
		{"lone high surrogate at the end of the string", Esc("ud83d")},
		{"high surrogate followed by a high surrogate", Esc("ud83d") + Esc("ud83d")},
		{"lone low surrogate", Esc("ude00")},
	};
}

struct ControlCase
{
	const char* Name;
	std::string Json;
};

std::vector<ControlCase> RawControlCases()
{
	const std::string Before = "{\n  \"profiles\": [\n    { \"name\": \"a";
	const std::string After = "b\", \"compressive_pa\": 1e7 }\n  ]\n}";
	return {
		{"raw tab", Before + '\t' + After},
		{"raw newline", Before + '\n' + After},
		{"raw carriage return", Before + '\r' + After},
		{"raw 0x01", Before + '\x01' + After},
		{"raw 0x1f", Before + '\x1f' + After},
		{"raw NUL", Before + '\0' + After},
		{"raw tab in an unknown key", std::string("{\n  \"profiles\": [\n    { \"name\": \"a\", \"no") + '\t' + "te\": 1, \"compressive_pa\": 1e7 }\n  ]\n}"},
	};
}

LoadedStructure TwoBodiesOneInterface(const std::string& ProfileName, const std::string& NameA, const std::string& NameB)
{
	LoadedStructure L;
	const Result<ProfileId> Profile = L.Model.AddProfile(DistinctFieldProfile());
	const Result<BodyId> A = L.Model.AddBody(BrickAt(StackedBrickCentroidM(0), true));
	const Result<BodyId> B = L.Model.AddBody(BrickAt(StackedBrickCentroidM(1)));
	REQUIRE(Profile.Ok());
	REQUIRE(A.Ok());
	REQUIRE(B.Ok());
	REQUIRE(L.Model.AddInterface(BedJoint(A.Value, B.Value, 1, Profile.Value)).Ok());
	L.ProfileNames = {ProfileName};
	L.BodyNames = {NameA, NameB};
	L.InterfaceNames = {""};
	return L;
}

struct FailedLoadCase
{
	const char* Name;
	std::string Json;
};

std::vector<FailedLoadCase> FailedLoadCases()
{
	DocSpec ThirdBodyDuplicate = ExampleSpec();
	ThirdBodyDuplicate.Bodies.push_back({{"name", "\"b0\""}, {"mass_kg", "1"}});
	DocSpec ThirdBodyNegative = ExampleSpec();
	ThirdBodyNegative.Bodies.push_back({{"name", "\"b2\""}, {"mass_kg", "-1"}});
	return {
		{"interface missing a", Build(Without(ExampleSpec(), 'i', 0, "a")).Json},
		{"interface names an unknown profile", Build(WithValue(ExampleSpec(), 'i', 0, "profile", "\"steel\"")).Json},
		{"interface with a zero normal", Build(WithValue(ExampleSpec(), 'i', 0, "normal", "[0, 0, 0]")).Json},
		{"third body duplicates a name", Build(ThirdBodyDuplicate).Json},
		{"third body has negative mass", Build(ThirdBodyNegative).Json},
		{"syntax error in the interface", Build(WithValue(ExampleSpec(), 'i', 0, "half_u_m", "yes")).Json},
	};
}

Vec3 Normalised(const Vec3& V)
{
	const double L = Length(V);
	return Vec3{V.X / L, V.Y / L, V.Z / L};
}

Vec3 LeastAlignedAxis(const Vec3& N)
{
	const double X = std::abs(N.X);
	const double Y = std::abs(N.Y);
	const double Z = std::abs(N.Z);
	if (X <= Y && X <= Z)
	{
		return Vec3{1.0, 0.0, 0.0};
	}
	return Y <= Z ? Vec3{0.0, 1.0, 0.0} : Vec3{0.0, 0.0, 1.0};
}

bool Within4Ulps(double Got, double Want)
{
	return std::abs(Got - Want) <= 4.0 * std::numeric_limits<double>::epsilon() * std::abs(Want);
}

std::string Shortest(const Vec3& V)
{
	const double Components[] = {V.X, V.Y, V.Z};
	std::string Text = "(";
	for (int Index = 0; Index < 3; ++Index)
	{
		std::array<char, 32> Buffer{};
		const std::to_chars_result Written = std::to_chars(Buffer.data(), Buffer.data() + Buffer.size(), Components[Index]);
		Text.append(Buffer.data(), Written.ptr);
		Text += Index < 2 ? ", " : ")";
	}
	return Text;
}

std::string NestedUnderUnknownKey(int Levels, char Open, char Close)
{
	std::string Json = "{\"extra\": ";
	for (int Level = 2; Level <= Levels; ++Level)
	{
		Json += Open;
		if (Open == '{' && Level < Levels)
		{
			Json += "\"k\": ";
		}
	}
	Json.append(static_cast<std::size_t>(Levels - 1), Close);
	Json += "}";
	return Json;
}
}

TEST_CASE("File.Load.StandardEscapesDecode")
{
	for (const EscapeCase& Case : EscapeCases())
	{
		INFO(doctest::String(Case.Name));
		const std::optional<LoadedStructure> Loaded = LoadOk(ProfileNameDocument(Case.Source));
		if (!Loaded)
		{
			continue;
		}
		CHECK(Loaded->ProfileNames.size() == 1u);
		if (Loaded->ProfileNames.size() != 1u)
		{
			continue;
		}
		CHECK(Loaded->ProfileNames[0] == Case.Decoded);
	}

	const std::optional<LoadedStructure> Keyed = LoadOk(std::string(R"({ "bodies": [ { "name": "b0", "mass)") + Esc("u005f") + R"(kg": 4 } ] })");
	if (Keyed)
	{
		const BodyId B0 = BodyNamed(*Keyed, "b0");
		CHECK(Keyed->Model.IsLive(B0));
		CHECK(Keyed->Model.Body(B0).MassKg == 4.0);
	}
}

TEST_CASE("File.Load.MalformedEscapesAreSyntaxErrorsAtTheirLine")
{
	for (const MalformedEscapeCase& Case : MalformedEscapes())
	{
		INFO(doctest::String(Case.Name));
		const LoadResult Result = LoadStructure(ProfileNameDocument(Case.Source));
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == 3);
	}
}

TEST_CASE("File.Load.RawControlCharacterInAStringIsASyntaxErrorAtItsLine")
{
	for (const ControlCase& Case : RawControlCases())
	{
		INFO(doctest::String(Case.Name));
		const LoadResult Result = LoadStructure(Case.Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == 3);
	}
}

TEST_CASE("File.Write.NamesWithQuotesBackslashesAndControlCharactersRoundTrip")
{
	const std::string ProfileName = std::string("m\"") + '\\' + "/\b\f\n\r\t" + '\x01' + '\x1f' + '\x7f' + '\0' + Bytes({0xC3, 0xA9}) + "end";
	const std::string NameA = std::string("a") + '\\';
	const std::string NameB = std::string("\"") + '\n' + '\t' + "b";
	const LoadedStructure Original = TwoBodiesOneInterface(ProfileName, NameA, NameB);

	const std::optional<LoadedStructure> Reloaded = LoadOk(WriteStructure(Original));
	if (!Reloaded)
	{
		return;
	}
	CHECK(Reloaded->ProfileNames == std::vector<std::string>{ProfileName});
	CHECK(Reloaded->BodyNames == std::vector<std::string>{NameA, NameB});
	const std::vector<InterfaceId> Interfaces = Reloaded->Model.LiveInterfaces();
	CHECK(Interfaces.size() == 1u);
	if (Interfaces.size() != 1u)
	{
		return;
	}
	const InterfaceRecord& J = Reloaded->Model.Interface(Interfaces[0]);
	CHECK(NameOf(Reloaded->BodyNames, J.BodyA.Index) == NameA);
	CHECK(NameOf(Reloaded->BodyNames, J.BodyB.Index) == NameB);
	CHECK(NameOf(Reloaded->ProfileNames, J.Profile.Index) == ProfileName);
}

TEST_CASE("File.Load.UnderflowReadsAsTheNearestRepresentableValue")
{
	const std::optional<LoadedStructure> Loaded = LoadOk(
		"{\n  \"bodies\": [\n    { \"name\": \"b0\", \"mass_kg\": 1e-400,\n"
		"      \"centroid_m\": [4.9406564584124654e-324, -1e-400, 1e-320] }\n  ]\n}");
	if (!Loaded)
	{
		return;
	}
	const BodyId B0 = BodyNamed(*Loaded, "b0");
	REQUIRE(Loaded->Model.IsLive(B0));
	const BodyDesc& Body = Loaded->Model.Body(B0);
	CHECK(Body.MassKg >= 0.0);
	CHECK(Body.MassKg < 1e-300);
	CHECK(Body.CentroidM.X == std::numeric_limits<double>::denorm_min());
	CHECK(Body.CentroidM.Y == 0.0);
	CHECK(Body.CentroidM.Z == 1e-320);
}

TEST_CASE("File.Load.FailedLoadReturnsAnEmptyValue")
{
	for (const FailedLoadCase& Case : FailedLoadCases())
	{
		INFO(doctest::String(Case.Name));
		const LoadResult Result = LoadStructure(Case.Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Value.Model.LiveBodyCount() == 0u);
		CHECK(Result.Value.Model.LiveInterfaceCount() == 0u);
		CHECK_FALSE(Result.Value.Model.IsValid(ProfileId{0u, 1u}));
		CHECK(Result.Value.BodyNames.empty());
		CHECK(Result.Value.InterfaceNames.empty());
		CHECK(Result.Value.ProfileNames.empty());
	}
}

TEST_CASE("File.Write.NonAxisNormalRoundTripsWithin4Ulps")
{
	const Vec3 Normals[] = {{0.3, 0.4, 0.866}, {1.0, 1.0, 1.0}, {-0.2, 0.7, -0.1}, {1e-3, -0.5, 2.0}};
	for (const Vec3& Raw : Normals)
	{
		INFO("normal " << Shortest(Raw));
		const Vec3 N = Normalised(Raw);
		const Vec3 U = Normalised(Cross(LeastAlignedAxis(N), N));
		const Vec3 V = Cross(N, U);

		LoadedStructure Original = TwoBodiesOneInterface("p", "b0", "b1");
		REQUIRE(Original.Model.RemoveBody(BodyNamed(Original, "b1")));
		const Result<BodyId> B2 = Original.Model.AddBody(BrickAt(Vec3{0.1 + 0.2, -1.0 / 7.0, 0.0975}));
		REQUIRE(B2.Ok());
		Original.BodyNames.push_back("b2");
		InterfaceDesc Desc = BedJoint(BodyNamed(Original, "b0"), B2.Value, 1, ProfileId{0u, 1u});
		Desc.NormalAToB = Raw;
		Desc.CentreM = Vec3{0.1 + 0.2, -1.0 / 7.0, 0.065};
		Desc.AxisU = U;
		Desc.AxisV = V;
		Desc.HalfExtentUM = 1.0 / 3.0 * BrickLengthM;
		Desc.HalfExtentVM = 2.0 / 3.0 * BrickWidthM;
		const Result<InterfaceId> Added = Original.Model.AddInterface(Desc);
		INFO("AddInterface: " << Added.Error);
		REQUIRE(Added.Ok());
		Original.InterfaceNames.push_back("");

		const std::optional<LoadedStructure> Reloaded = LoadOk(WriteStructure(Original));
		if (!Reloaded)
		{
			continue;
		}
		const std::vector<InterfaceId> Interfaces = Reloaded->Model.LiveInterfaces();
		CHECK(Interfaces.size() == 1u);
		if (Interfaces.size() != 1u)
		{
			continue;
		}
		const InterfaceRecord& G = Reloaded->Model.Interface(Interfaces[0]);
		const InterfaceRecord& W = Original.Model.Interface(Added.Value);
		INFO("stored normal " << Shortest(W.NormalAToB) << ", reloaded " << Shortest(G.NormalAToB));
		CHECK(Within4Ulps(G.NormalAToB.X, W.NormalAToB.X));
		CHECK(Within4Ulps(G.NormalAToB.Y, W.NormalAToB.Y));
		CHECK(Within4Ulps(G.NormalAToB.Z, W.NormalAToB.Z));
		CheckSameVec3(G.CentreM, W.CentreM, "centre");
		CheckSameVec3(G.AxisU, W.AxisU, "axis U");
		CheckSameVec3(G.AxisV, W.AxisV, "axis V");
		CHECK(G.HalfExtentUM == W.HalfExtentUM);
		CHECK(G.HalfExtentVM == W.HalfExtentVM);
		CHECK(G.AreaM2 == W.AreaM2);
		CHECK(NameOf(Reloaded->BodyNames, G.BodyA.Index) == "b0");
		CHECK(NameOf(Reloaded->BodyNames, G.BodyB.Index) == "b2");
		CHECK(NameOf(Reloaded->ProfileNames, G.Profile.Index) == "p");
	}
}

TEST_CASE("File.Load.NonFiniteNumberIsASyntaxErrorAtItsLine")
{
	struct Placement
	{
		const char* Name;
		std::string Before;
		std::string After;
		int Line;
	};
	const Placement Placements[] = {
		{"body mass_kg", "{\n  \"bodies\": [\n    { \"name\": \"b0\",\n      \"mass_kg\": ", " }\n  ]\n}", 4},
		{"profile max_shear_pa", "{\n  \"profiles\": [\n    { \"name\": \"p\",\n      \"max_shear_pa\": ", " }\n  ]\n}", 4},
		{"unknown key", "{\n  \"extra\":\n    ", "\n}", 3},
	};
	for (const Placement& Where : Placements)
	{
		for (const char* Number : {"1e999", "-1e999", "1.8e308"})
		{
			INFO(doctest::String(Where.Name) << ": " << std::string(Number));
			const LoadResult Result = LoadStructure(Where.Before + Number + Where.After);
			INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
			CHECK_FALSE(Result.Ok());
			CHECK(Result.Error.Line == Where.Line);
		}
	}

	const std::optional<LoadedStructure> Largest = LoadOk(Placements[0].Before + "1.7976931348623157e308" + Placements[0].After);
	if (Largest)
	{
		const BodyId B0 = BodyNamed(*Largest, "b0");
		CHECK(Largest->Model.IsLive(B0));
		CHECK(Largest->Model.Body(B0).MassKg == std::numeric_limits<double>::max());
	}
}

TEST_CASE("File.Load.NestingOf64LevelsIsAcceptedAnd65Refused")
{
	for (const char Open : {'[', '{'})
	{
		const char Close = Open == '[' ? ']' : '}';
		INFO("nesting with " << std::string(1, Open));
		const LoadResult At64 = LoadStructure(NestedUnderUnknownKey(64, Open, Close));
		INFO("64 levels: line " << At64.Error.Line << ": " << At64.Error.Message);
		CHECK(At64.Ok());

		const LoadResult At65 = LoadStructure(NestedUnderUnknownKey(65, Open, Close));
		INFO("65 levels: line " << At65.Error.Line << ": " << At65.Error.Message);
		CHECK_FALSE(At65.Ok());
		CHECK(At65.Error.Line == 1);
	}
}

TEST_CASE("File.Load.ADocumentThatIsNotAnObjectReportsLine0")
{
	for (const char* Json : {"", "  \n\t\n", "[]", "\n\n[]", "42", "\"text\"", "null"})
	{
		INFO("document <" << std::string(Json) << ">");
		const LoadResult Result = LoadStructure(Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == 0);
		CHECK_FALSE(Result.Error.Message.empty());
	}
}

TEST_CASE("File.Load.WrongTypedTopLevelArraysAndNamesAreErrorsNamingTheField")
{
	struct TopLevelCase
	{
		const char* Json;
		const char* Field;
	};
	const TopLevelCase TopLevel[] = {
		{"{ \"bodies\": {} }", "bodies"},
		{"{ \"profiles\": 3 }", "profiles"},
		{"{ \"interfaces\": \"none\" }", "interfaces"},
		{"{ \"bodies\": true }", "bodies"},
	};
	for (const TopLevelCase& Case : TopLevel)
	{
		INFO("document: " << std::string(Case.Json));
		const LoadResult Result = LoadStructure(Case.Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line >= 1);
		CHECK(Contains(Result.Error.Message, Case.Field));
	}

	struct NameCase
	{
		char Kind;
		std::size_t Index;
		const char* Value;
	};
	const NameCase Names[] = {{'b', 1, "7"}, {'b', 0, "[\"b0\"]"}, {'p', 0, "true"}, {'p', 0, "{}"}};
	for (const NameCase& Case : Names)
	{
		INFO("kind " << std::string(1, Case.Kind) << " name: " << std::string(Case.Value));
		const BuiltDoc Doc = Build(WithValue(ExampleSpec(), Case.Kind, Case.Index, "name", Case.Value));
		const LoadResult Result = LoadStructure(Doc.Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == LinesOf(Doc, Case.Kind)[Case.Index]);
		CHECK(Contains(Result.Error.Message, "name"));
	}
}

TEST_CASE("File.Write.ShortProfileNamesGetGeneratedNames")
{
	LoadedStructure Unnamed;
	StrengthProfile Second = DistinctFieldProfile();
	Second.CompressiveStrengthPa = 1.0 / 3.0 * 1.0e7;
	REQUIRE(Unnamed.Model.AddProfile(DistinctFieldProfile()).Ok());
	REQUIRE(Unnamed.Model.AddProfile(Second).Ok());

	const std::optional<LoadedStructure> FromEmpty = LoadOk(WriteStructure(Unnamed));
	if (FromEmpty)
	{
		CHECK(FromEmpty->ProfileNames == std::vector<std::string>{"p0", "p1"});
		const ProfileId P1 = ProfileNamed(*FromEmpty, "p1");
		REQUIRE(FromEmpty->Model.IsValid(P1));
		CHECK(FromEmpty->Model.Profile(P1).CompressiveStrengthPa == Second.CompressiveStrengthPa);
	}

	Unnamed.ProfileNames = {"lime"};
	const std::optional<LoadedStructure> FromShort = LoadOk(WriteStructure(Unnamed));
	if (FromShort)
	{
		CHECK(FromShort->ProfileNames == std::vector<std::string>{"lime", "p1"});
	}
}

TEST_CASE("File.Write.ShortBodyNamesGetGeneratedNames")
{
	LoadedStructure Unnamed = TwoBodiesOneInterface("p", "b0", "b1");
	Unnamed.BodyNames.clear();
	Unnamed.ProfileNames.clear();

	const std::optional<LoadedStructure> FromEmpty = LoadOk(WriteStructure(Unnamed));
	if (FromEmpty)
	{
		CHECK(FromEmpty->BodyNames == std::vector<std::string>{"b0", "b1"});
		CHECK(FromEmpty->ProfileNames == std::vector<std::string>{"p0"});
		CHECK(FromEmpty->Model.LiveBodyCount() == 2u);
		CHECK(FromEmpty->Model.LiveInterfaceCount() == 1u);
	}

	Unnamed.BodyNames = {"left"};
	const std::optional<LoadedStructure> FromShort = LoadOk(WriteStructure(Unnamed));
	if (FromShort)
	{
		CHECK(FromShort->BodyNames == std::vector<std::string>{"left", "b1"});
		const std::vector<InterfaceId> Interfaces = FromShort->Model.LiveInterfaces();
		CHECK(Interfaces.size() == 1u);
		if (Interfaces.size() != 1u)
		{
			return;
		}
		const InterfaceRecord& J = FromShort->Model.Interface(Interfaces[0]);
		CHECK(NameOf(FromShort->BodyNames, J.BodyA.Index) == "left");
		CHECK(NameOf(FromShort->BodyNames, J.BodyB.Index) == "b1");
	}
}

TEST_CASE("Model.StructureFile.RejectsBadInterfaceStateAndStamps")
{
	struct Case
	{
		const char* Name;
		const char* State;
		const char* CrackPass;
		const char* SeverPass;
		const char* Named;
	};
	const Case Cases[] = {
		{"unknown state", "\"broken\"", nullptr, nullptr, "broken"},
		{"crack_pass negative", "\"cracked\"", "-1", nullptr, "crack_pass"},
		{"crack_pass fractional", "\"cracked\"", "1.5", nullptr, "crack_pass"},
		{"crack_pass above 2^32 - 1", "\"cracked\"", "4294967296", nullptr, "crack_pass"},
		{"intact with a crack stamp", "\"intact\"", "3", nullptr, "intact"},
		{"intact with a sever stamp", "\"intact\"", nullptr, "3", "intact"},
		{"cracked with sever_pass", "\"cracked\"", "2", "3", "cracked"},
		{"cracked with crack_pass 0", "\"cracked\"", "0", nullptr, "cracked"},
		{"severed with sever_pass 0", "\"severed\"", nullptr, "0", "severed"},
		{"severed with crack_pass above sever_pass", "\"severed\"", "4", "3", "severed"},
	};
	for (const Case& Each : Cases)
	{
		INFO(doctest::String(Each.Name));
		DocSpec Spec = WithValue(ExampleSpec(), 'i', 0, "state", Each.State);
		if (Each.CrackPass != nullptr)
		{
			Spec = WithValue(Spec, 'i', 0, "crack_pass", Each.CrackPass);
		}
		if (Each.SeverPass != nullptr)
		{
			Spec = WithValue(Spec, 'i', 0, "sever_pass", Each.SeverPass);
		}
		const BuiltDoc Doc = Build(Spec);
		const LoadResult Result = LoadStructure(Doc.Json);
		INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
		CHECK_FALSE(Result.Ok());
		CHECK(Result.Error.Line == Doc.InterfaceLines[0]);
		CHECK_MESSAGE(Contains(Result.Error.Message, Each.Named), "the message must name the field or the state");
		CHECK_MESSAGE(Contains(Result.Error.Message, "interface"), "the message must name the element");
	}
}
