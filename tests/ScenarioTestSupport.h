#pragma once

#include "CatalogueTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/Catalogue.h"
#include "loadpath/Scenario.h"
#include "loadpath/StrengthProfile.h"
#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

#include <algorithm>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace loadpath::test
{
inline constexpr std::string_view ColonnadeSlug = "colonnade-middle-post";
inline constexpr std::string_view WrongExpectationSlug = "colonnade-wrong-expectation";

// The plan's section 1 example, byte for byte the checked-in data/scenarios/colonnade-middle-post.json.
inline const std::string ColonnadeJson = R"({
  "title": "Colonnade losing its middle post",
  "summary": "Five dry timber posts carry four beams. The middle post goes; the two beams it carried fall, each on its own.",
  "build": [
    { "name": "ground", "size": [12, 2, 0.2], "centre": [4.4, 0, -0.1], "grounded": true },
    { "name": "post", "size": [0.2, 0.2, 2.0], "centre": [0, 0, 1.0], "material": "c24",
      "repeat": { "count": 5, "step": [2.2, 0, 0] } },
    { "name": "beam", "size": [2.199, 0.2, 0.3], "centre": [1.1, 0, 2.15], "material": "c24",
      "repeat": { "count": 4, "step": [2.2, 0, 0] } }
  ],
  "joints": [
    { "materials": ["c24", "c24"], "row": "c24_bearing_perp_to_grain" },
    { "materials": ["c24", "ground"], "row": "c24_bearing_perp_to_grain" }
  ],
  "steps": ["cascade", { "remove": ["post.2"] }, "cascade"],
  "expect": { "released": [["beam.1"], ["beam.2"]] }
}
)";

inline std::string Replaced(std::string Text, std::string_view From, std::string_view To)
{
	const std::size_t At = Text.find(From);
	INFO("fixture text must contain " << std::string(From));
	CHECK(At != std::string::npos);
	if (At != std::string::npos)
	{
		Text.replace(At, From.size(), To);
	}
	return Text;
}

inline std::string WrongExpectationJson()
{
	return Replaced(ColonnadeJson, R"("released": [["beam.1"], ["beam.2"]])", R"("released": [["beam.1"], ["post.1"]])");
}

inline const Catalogue& ScenarioRows()
{
	static const Catalogue Value = [] {
		const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
		INFO("catalogue line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
		CHECK(Loaded.Ok());
		return Loaded.Value;
	}();
	return Value;
}

inline ScenarioLoadResult LoadWithRows(std::string_view Json) { return LoadScenario(Json, ScenarioRows()); }

inline Scenario LoadScenarioOk(std::string_view Json)
{
	ScenarioLoadResult Loaded = LoadWithRows(Json);
	INFO("scenario load error line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	return std::move(Loaded.Value);
}

inline std::string NameOf(const Scenario& S, BodyId Id) { return Id.Index < S.BodyNames.size() ? S.BodyNames[Id.Index] : "#" + std::to_string(Id.Index); }

inline BodyId BodyOrFail(const Scenario& S, std::string_view Name)
{
	const BodyId Id = S.Find(Name);
	INFO("scenario body " << std::string(Name));
	CHECK(S.Model.IsKnown(Id));
	return Id;
}

inline std::string PairName(const Scenario& S, const InterfaceRecord& R)
{
	const std::string A = NameOf(S, R.BodyA);
	const std::string B = NameOf(S, R.BodyB);
	return A < B ? A + "/" + B : B + "/" + A;
}

inline std::vector<std::string> SortedNames(std::vector<std::string> Names)
{
	std::sort(Names.begin(), Names.end());
	return Names;
}

inline std::vector<std::vector<std::string>> SortedPartition(std::vector<std::vector<std::string>> Clusters)
{
	for (std::vector<std::string>& C : Clusters)
	{
		std::sort(C.begin(), C.end());
	}
	std::sort(Clusters.begin(), Clusters.end());
	return Clusters;
}

inline bool SameProfile(const StrengthProfile& A, const StrengthProfile& B)
{
	return A.CompressiveStrengthPa == B.CompressiveStrengthPa && A.ShearCohesionPa == B.ShearCohesionPa && A.TensileStrengthPa == B.TensileStrengthPa &&
		A.FrictionCoefficient == B.FrictionCoefficient && A.MaxShearStrengthPa == B.MaxShearStrengthPa &&
		A.NormalStiffnessPaPerM == B.NormalStiffnessPaPerM && A.TangentialStiffnessPaPerM == B.TangentialStiffnessPaPerM;
}

inline std::unique_ptr<IStructuralAuthority> StandardAuthority()
{
	return MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate());
}
}
