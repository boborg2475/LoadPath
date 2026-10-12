#pragma once

#include "StructureFileTestSupport.h"

#include "loadpath/Catalogue.h"

#include "doctest.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace loadpath::test
{
inline std::string ReadCatalogueFile()
{
	std::ifstream File(LOADPATH_CATALOGUE_PATH, std::ios::binary);
	std::ostringstream Text;
	Text << File.rdbuf();
	return Text.str();
}

inline const JointRow& JointOrFail(const Catalogue& Value, std::string_view Name)
{
	static const JointRow Missing{};
	const JointRow* Row = Value.FindJoint(Name);
	INFO("joint row " << std::string(Name));
	REQUIRE(Row != nullptr);
	return Row != nullptr ? *Row : Missing;
}

inline const FastenerRow& FastenerOrFail(const Catalogue& Value, std::string_view Name)
{
	static const FastenerRow Missing{};
	const FastenerRow* Row = Value.FindFastener(Name);
	INFO("fastener row " << std::string(Name));
	REQUIRE(Row != nullptr);
	return Row != nullptr ? *Row : Missing;
}

// Each row sits on its own line, so a row's opening line is the line it is written on.
struct CatalogueSpec
{
	std::string Version = "1";
	std::vector<JsonElement> Joints;
	std::vector<JsonElement> Fasteners;
};

struct BuiltCatalogue
{
	std::string Json;
	int VersionLine = 2;
	std::vector<int> JointLines;
	std::vector<int> FastenerLines;
};

inline JsonElement BedJointElement(const std::string& Name = "bed")
{
	return {{"name", "\"" + Name + "\""}, {"kind", "\"bed\""}, {"basis", "\"characteristic\""}, {"compressive_mpa", "8.93"},
		{"cohesion_mpa", "0.30"}, {"tensile_mpa", "0.10"}, {"friction", "0.4"}, {"shear_cap_mpa", "1.30"},
		{"normal_stiffness_gpa_per_m", "893"}, {"tangential_stiffness_gpa_per_m", "357"}, {"source", "\"s\""}, {"note", "\"n\""}};
}

inline JsonElement ScrewElement(const std::string& Name = "screw")
{
	return {{"name", "\"" + Name + "\""}, {"basis", "\"characteristic\""}, {"shear_per_plane_kn", "4.83"}, {"withdrawal_kn", "1.74"},
		{"bearing_mpa", "3.75"}, {"slip_modulus_mn_per_m", "6.20"}, {"bearing_normal_stiffness_gpa_per_m", "8.2"}, {"source", "\"s\""}, {"note", "\"n\""}};
}

inline CatalogueSpec ValidCatalogueSpec()
{
	CatalogueSpec Spec;
	Spec.Joints = {BedJointElement("bed"), BedJointElement("bed_two")};
	Spec.Fasteners = {ScrewElement("screw"), ScrewElement("screw_two")};
	return Spec;
}

inline JsonElement WithField(JsonElement Element, const std::string& Key, const std::string& Value)
{
	for (JsonField& Field : Element)
	{
		if (Field.Key == Key)
		{
			Field.Value = Value;
			return Element;
		}
	}
	Element.push_back({Key, Value});
	return Element;
}

inline JsonElement WithoutField(JsonElement Element, const std::string& Key)
{
	std::erase_if(Element, [&](const JsonField& Field) { return Field.Key == Key; });
	return Element;
}

inline std::string OneLineObject(const JsonElement& Element)
{
	std::string Text = "{ ";
	for (std::size_t F = 0; F < Element.size(); ++F)
	{
		Text += "\"" + Element[F].Key + "\": " + Element[F].Value + (F + 1 < Element.size() ? ", " : " ");
	}
	return Text + "}";
}

inline BuiltCatalogue BuildCatalogue(const CatalogueSpec& Spec)
{
	BuiltCatalogue Doc;
	std::vector<std::string> Lines{"{", "  \"version\": " + Spec.Version + ",", "  \"joints\": ["};
	for (std::size_t R = 0; R < Spec.Joints.size(); ++R)
	{
		Doc.JointLines.push_back(static_cast<int>(Lines.size()) + 1);
		Lines.push_back("    " + OneLineObject(Spec.Joints[R]) + (R + 1 < Spec.Joints.size() ? "," : ""));
	}
	Lines.push_back("  ],");
	Lines.push_back("  \"fasteners\": [");
	for (std::size_t R = 0; R < Spec.Fasteners.size(); ++R)
	{
		Doc.FastenerLines.push_back(static_cast<int>(Lines.size()) + 1);
		Lines.push_back("    " + OneLineObject(Spec.Fasteners[R]) + (R + 1 < Spec.Fasteners.size() ? "," : ""));
	}
	Lines.push_back("  ]");
	Lines.push_back("}");
	for (const std::string& Line : Lines)
	{
		Doc.Json += Line + "\n";
	}
	return Doc;
}
}
