#pragma once

#include "ModelTestSupport.h"

#include "loadpath/StructureFile.h"

#include "doctest.h"

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace loadpath::test
{
inline constexpr std::string_view ExampleDocument = R"({ "profiles":   [ { "name": "mortar", "compressive_pa": 1e7, "cohesion_pa": 9e5, "tensile_pa": 7e5,
                    "friction": 0.75, "max_shear_pa": 2e6, "normal_stiffness_pa_per_m": 2e10, "tangential_stiffness_pa_per_m": 1e10 } ],
  "bodies":     [ { "name": "b0", "mass_kg": 2.72, "centroid_m": [0, 0, 0.0325], "grounded": false },
                  { "name": "b1", "mass_kg": 3.1, "centroid_m": [0, 0, 0.0975], "grounded": true } ],
  "interfaces": [ { "a": "b0", "b": "b1", "normal": [0, 0, 2], "centre_m": [0, 0, 0.065],
                    "u": [1, 0, 0], "v": [0, 1, 0], "half_u_m": 0.1075, "half_v_m": 0.05125, "profile": "mortar" } ] }
)";

struct JsonField
{
	std::string Key;
	std::string Value;
};

using JsonElement = std::vector<JsonField>;

struct DocSpec
{
	std::vector<JsonElement> Profiles;
	std::vector<JsonElement> Bodies;
	std::vector<JsonElement> Interfaces;
	std::string Order = "pbi";
};

struct BuiltDoc
{
	std::string Json;
	std::vector<int> ProfileLines;
	std::vector<int> BodyLines;
	std::vector<int> InterfaceLines;
};

inline DocSpec ExampleSpec()
{
	DocSpec Spec;
	Spec.Profiles.push_back({{"name", "\"mortar\""}, {"compressive_pa", "1e7"}, {"cohesion_pa", "9e5"}, {"tensile_pa", "7e5"},
		{"friction", "0.75"}, {"max_shear_pa", "2e6"}, {"normal_stiffness_pa_per_m", "2e10"}, {"tangential_stiffness_pa_per_m", "1e10"}});
	Spec.Bodies.push_back({{"name", "\"b0\""}, {"mass_kg", "2.72"}, {"centroid_m", "[0, 0, 0.0325]"}, {"grounded", "false"}});
	Spec.Bodies.push_back({{"name", "\"b1\""}, {"mass_kg", "3.1"}, {"centroid_m", "[0, 0, 0.0975]"}, {"grounded", "true"}});
	Spec.Interfaces.push_back({{"a", "\"b0\""}, {"b", "\"b1\""}, {"normal", "[0, 0, 2]"}, {"centre_m", "[0, 0, 0.065]"},
		{"u", "[1, 0, 0]"}, {"v", "[0, 1, 0]"}, {"half_u_m", "0.1075"}, {"half_v_m", "0.05125"}, {"profile", "\"mortar\""}});
	return Spec;
}

inline std::vector<JsonElement>& ElementsOf(DocSpec& Spec, char Kind)
{
	return Kind == 'p' ? Spec.Profiles : (Kind == 'b' ? Spec.Bodies : Spec.Interfaces);
}

inline const std::vector<int>& LinesOf(const BuiltDoc& Doc, char Kind)
{
	return Kind == 'p' ? Doc.ProfileLines : (Kind == 'b' ? Doc.BodyLines : Doc.InterfaceLines);
}

inline DocSpec Without(DocSpec Spec, char Kind, std::size_t Index, const std::string& Key)
{
	JsonElement& Element = ElementsOf(Spec, Kind)[Index];
	std::erase_if(Element, [&](const JsonField& Field) { return Field.Key == Key; });
	return Spec;
}

inline DocSpec WithValue(DocSpec Spec, char Kind, std::size_t Index, const std::string& Key, const std::string& Value)
{
	JsonElement& Element = ElementsOf(Spec, Kind)[Index];
	for (JsonField& Field : Element)
	{
		if (Field.Key == Key)
		{
			Field.Value = Value;
			return Spec;
		}
	}
	Element.push_back({Key, Value});
	return Spec;
}

inline void EmitKind(std::vector<std::string>& Lines, const char* Key, const std::vector<JsonElement>& Elements, std::vector<int>& Opening, bool bLast)
{
	Lines.push_back(std::string("  \"") + Key + "\": [");
	for (std::size_t E = 0; E < Elements.size(); ++E)
	{
		Opening.push_back(static_cast<int>(Lines.size()) + 1);
		Lines.push_back("    {");
		const JsonElement& Element = Elements[E];
		for (std::size_t F = 0; F < Element.size(); ++F)
		{
			Lines.push_back("      \"" + Element[F].Key + "\": " + Element[F].Value + (F + 1 < Element.size() ? "," : ""));
		}
		Lines.push_back(E + 1 < Elements.size() ? "    }," : "    }");
	}
	Lines.push_back(bLast ? "  ]" : "  ],");
}

inline BuiltDoc Build(const DocSpec& Spec)
{
	BuiltDoc Doc;
	std::vector<std::string> Lines{"{"};
	for (std::size_t K = 0; K < Spec.Order.size(); ++K)
	{
		const bool bLast = K + 1 == Spec.Order.size();
		switch (Spec.Order[K])
		{
		case 'p': EmitKind(Lines, "profiles", Spec.Profiles, Doc.ProfileLines, bLast); break;
		case 'b': EmitKind(Lines, "bodies", Spec.Bodies, Doc.BodyLines, bLast); break;
		default: EmitKind(Lines, "interfaces", Spec.Interfaces, Doc.InterfaceLines, bLast); break;
		}
	}
	Lines.push_back("}");
	for (const std::string& Line : Lines)
	{
		Doc.Json += Line + "\n";
	}
	return Doc;
}

inline bool Contains(std::string_view Text, std::string_view Needle)
{
	return Text.find(Needle) != std::string_view::npos;
}

inline std::string Lowercase(std::string_view Text)
{
	std::string Lower;
	for (const char C : Text)
	{
		Lower += static_cast<char>(std::tolower(static_cast<unsigned char>(C)));
	}
	return Lower;
}

inline std::string NameOf(const std::vector<std::string>& Names, std::uint32_t Index)
{
	return Index < Names.size() ? Names[Index] : std::string("<no name for slot ") + std::to_string(Index) + ">";
}

inline BodyId BodyNamed(const LoadedStructure& Loaded, std::string_view Name)
{
	for (const BodyId& Id : Loaded.Model.LiveBodies())
	{
		if (Id.Index < Loaded.BodyNames.size() && Loaded.BodyNames[Id.Index] == Name)
		{
			return Id;
		}
	}
	return BodyId::Invalid();
}

inline ProfileId ProfileNamed(const LoadedStructure& Loaded, std::string_view Name)
{
	for (std::uint32_t Index = 0; Index < Loaded.ProfileNames.size(); ++Index)
	{
		const ProfileId Id{Index, 1u};
		if (Loaded.ProfileNames[Index] == Name && Loaded.Model.IsValid(Id))
		{
			return Id;
		}
	}
	return ProfileId::Invalid();
}

inline std::optional<LoadedStructure> LoadOk(std::string_view Json)
{
	LoadResult Result = LoadStructure(Json);
	INFO("line " << Result.Error.Line << ": " << Result.Error.Message);
	CHECK(Result.Ok());
	if (!Result.Ok())
	{
		return std::nullopt;
	}
	return std::move(Result.Value);
}
}
