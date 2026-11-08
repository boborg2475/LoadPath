#include "loadpath/Catalogue.h"

#include "JsonReader.h"

#include "loadpath/Structure.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <string>
#include <unordered_set>

namespace loadpath
{
namespace
{
using namespace detail;

constexpr double PaPerMPa = 1e6;
constexpr double PaPerMPerGPaPerM = 1e9;
constexpr double NPerKN = 1e3;
constexpr double NPerMPerMNPerM = 1e6;

class RowReader
{
public:
	RowReader(const JsonValue& Row, const char* Table, std::size_t Index, FileError& Error)
		: Fields(Row, std::string(Table) + " " + std::to_string(Index), Error)
		, Element(Row)
	{
	}

	bool Name(std::string& Out, std::unordered_set<std::string>& Seen)
	{
		if (!Fields.String("name", Out))
		{
			return false;
		}
		Fields.Label = Fields.Label.substr(0, Fields.Label.find(' ')) + " " + Out;
		return Seen.insert(Out).second || Fields.Fail("duplicate name " + Out);
	}

	bool Scaled(std::string_view Key, double Scale, double& Out, bool bRequired = true)
	{
		double Value = Out / Scale;
		if (!Fields.Number(Key, Value, bRequired))
		{
			return false;
		}
		if (Value < 0.0)
		{
			return Fields.Fail(std::string(Key) + " is negative");
		}
		if (Find(Element, Key) != nullptr && !std::isfinite(Value * Scale))
		{
			return Fields.Fail(std::string(Key) + " is " + ModelErrorText(ModelError::NonFiniteInput) + " after conversion");
		}
		Out = Value * Scale;
		return true;
	}

	bool Positive(std::string_view Key, double Scale, double& Out)
	{
		return Scaled(Key, Scale, Out) && (Out > 0.0 || Fields.Fail(std::string(Key) + " must be positive"));
	}

	bool OnlyKeys(std::initializer_list<std::string_view> Allowed)
	{
		for (const std::string& Key : Element.Keys)
		{
			if (std::find(Allowed.begin(), Allowed.end(), Key) == Allowed.end())
			{
				return Fields.Fail("unknown key " + Key);
			}
		}
		return true;
	}

	bool Basis(StrengthBasis& Out)
	{
		std::string Text;
		if (!Fields.String("basis", Text))
		{
			return false;
		}
		if (Text == "characteristic")
		{
			Out = StrengthBasis::Characteristic;
			return true;
		}
		if (Text == "mean")
		{
			Out = StrengthBasis::Mean;
			return true;
		}
		return Fields.Fail("unknown basis " + Text);
	}

	bool Kind(std::string& Out)
	{
		if (!Fields.String("kind", Out))
		{
			return false;
		}
		return Out == "bed" || Out == "perpend" || Out == "bearing" || Fields.Fail("unknown kind " + Out);
	}

	ElementFields Fields;
	const JsonValue& Element;
};

bool ReadJoint(const JsonValue& Value, std::size_t Index, std::unordered_set<std::string>& Seen, FileError& Error, JointRow& Row)
{
	if (Value.Type != JsonType::Object)
	{
		Error = {Value.Line, "joint " + std::to_string(Index) + ": not an object"};
		return false;
	}
	RowReader Reader(Value, "joint", Index, Error);
	StrengthProfile& P = Row.Profile;
	Row.FrictionNormalStressPa = NotANumber;
	if (!(Reader.Name(Row.Name, Seen) &&
			Reader.OnlyKeys({"name", "kind", "basis", "compressive_mpa", "cohesion_mpa", "tensile_mpa", "friction", "shear_cap_mpa",
				"normal_stiffness_gpa_per_m", "tangential_stiffness_gpa_per_m", "friction_normal_stress_mpa", "source", "note"}) &&
			Reader.Kind(Row.Kind) && Reader.Basis(Row.Basis) &&
			Reader.Scaled("compressive_mpa", PaPerMPa, P.CompressiveStrengthPa) && Reader.Scaled("cohesion_mpa", PaPerMPa, P.ShearCohesionPa) &&
			Reader.Scaled("tensile_mpa", PaPerMPa, P.TensileStrengthPa) && Reader.Scaled("friction", 1.0, P.FrictionCoefficient) &&
			Reader.Scaled("shear_cap_mpa", PaPerMPa, P.MaxShearStrengthPa, false) &&
			Reader.Positive("normal_stiffness_gpa_per_m", PaPerMPerGPaPerM, P.NormalStiffnessPaPerM) &&
			Reader.Positive("tangential_stiffness_gpa_per_m", PaPerMPerGPaPerM, P.TangentialStiffnessPaPerM) &&
			Reader.Scaled("friction_normal_stress_mpa", PaPerMPa, Row.FrictionNormalStressPa, false) &&
			Reader.Fields.String("source", Row.Source) && Reader.Fields.String("note", Row.Note)))
	{
		return false;
	}
	Structure Scratch;
	const Result<ProfileId> Added = Scratch.AddProfile(P);
	return Added.Ok() || Reader.Fields.Fail(ModelErrorText(Added.Error));
}

bool ReadFastener(const JsonValue& Value, std::size_t Index, std::unordered_set<std::string>& Seen, FileError& Error, FastenerRow& Row)
{
	if (Value.Type != JsonType::Object)
	{
		Error = {Value.Line, "fastener " + std::to_string(Index) + ": not an object"};
		return false;
	}
	RowReader Reader(Value, "fastener", Index, Error);
	return Reader.Name(Row.Name, Seen) &&
		Reader.OnlyKeys({"name", "basis", "shear_per_plane_kn", "withdrawal_kn", "bearing_mpa", "slip_modulus_mn_per_m",
			"bearing_normal_stiffness_gpa_per_m", "source", "note"}) &&
		Reader.Basis(Row.Basis) && Reader.Scaled("shear_per_plane_kn", NPerKN, Row.ShearPerPlaneN) &&
		Reader.Scaled("withdrawal_kn", NPerKN, Row.WithdrawalN) && Reader.Positive("bearing_mpa", PaPerMPa, Row.BearingStrengthPa) &&
		Reader.Positive("slip_modulus_mn_per_m", NPerMPerMNPerM, Row.SlipModulusNPerM) &&
		Reader.Positive("bearing_normal_stiffness_gpa_per_m", PaPerMPerGPaPerM, Row.BearingNormalStiffnessPaPerM) &&
		Reader.Fields.String("source", Row.Source) && Reader.Fields.String("note", Row.Note);
}

bool ReadMaterial(const JsonValue& Value, std::size_t Index, std::unordered_set<std::string>& Seen, FileError& Error, MaterialRow& Row)
{
	if (Value.Type != JsonType::Object)
	{
		Error = {Value.Line, "material " + std::to_string(Index) + ": not an object"};
		return false;
	}
	RowReader Reader(Value, "material", Index, Error);
	return Reader.Name(Row.Name, Seen) && Reader.OnlyKeys({"name", "density_kg_per_m3", "compressive_strength_mpa", "source", "note"}) &&
		Reader.Positive("density_kg_per_m3", 1.0, Row.DensityKgPerM3) && Reader.Scaled("compressive_strength_mpa", PaPerMPa, Row.CompressiveStrengthPa, false) && Reader.Fields.String("source", Row.Source) &&
		Reader.Fields.String("note", Row.Note);
}

bool ReadVersion(const JsonValue& Root, FileError& Error)
{
	const JsonValue* Version = Find(Root, "version");
	if (Version == nullptr)
	{
		Error = {Root.Line, "document: missing version"};
		return false;
	}
	if (Version->Type != JsonType::Number || Version->Number != 1.0)
	{
		Error = {Version->Line, "document: version must be 1"};
		return false;
	}
	return true;
}

template <typename RowType, typename ReadFn>
bool ReadTable(ElementFields& Document, std::string_view Key, std::vector<RowType>& Out, FileError& Error, ReadFn Read, bool bRequired = true)
{
	const JsonValue* Array = nullptr;
	if (!Document.Elements(Key, Array))
	{
		return false;
	}
	if (Array == nullptr)
	{
		return !bRequired || Document.Fail("missing " + std::string(Key));
	}
	std::unordered_set<std::string> Seen;
	for (std::size_t Index = 0; Index < Array->Items.size(); ++Index)
	{
		RowType Row;
		if (!Read(Array->Items[Index], Index, Seen, Error, Row))
		{
			return false;
		}
		Out.push_back(std::move(Row));
	}
	return true;
}

template <typename RowType>
const RowType* FindRow(const std::vector<RowType>& Rows, std::string_view Name)
{
	for (const RowType& Row : Rows)
	{
		if (Row.Name == Name)
		{
			return &Row;
		}
	}
	return nullptr;
}
}

const JointRow* Catalogue::FindJoint(std::string_view Name) const
{
	return FindRow(Joints, Name);
}

const FastenerRow* Catalogue::FindFastener(std::string_view Name) const
{
	return FindRow(Fasteners, Name);
}

const MaterialRow* Catalogue::FindMaterial(std::string_view Name) const
{
	return FindRow(Materials, Name);
}

CatalogueResult LoadCatalogue(std::string_view Json)
{
	CatalogueResult Result;
	JsonParser Parser(Json);
	JsonValue Root;
	if (!Parser.ParseDocument(Root))
	{
		Result.Error = Parser.Error;
		return Result;
	}
	ElementFields Document(Root, "document", Result.Error);
	if (!ReadVersion(Root, Result.Error) || !ReadTable(Document, "joints", Result.Value.Joints, Result.Error, ReadJoint) ||
		!ReadTable(Document, "fasteners", Result.Value.Fasteners, Result.Error, ReadFastener) ||
		!ReadTable(Document, "materials", Result.Value.Materials, Result.Error, ReadMaterial, false))
	{
		Result.Value = Catalogue{};
	}
	return Result;
}

StrengthProfile FastenerProfile(const FastenerRow& Row, int Count, double AreaM2)
{
	StrengthProfile Profile;
	Profile.FrictionCoefficient = 0.0;
	Profile.ShearCohesionPa = Count * Row.ShearPerPlaneN / AreaM2;
	Profile.MaxShearStrengthPa = Profile.ShearCohesionPa;
	Profile.TensileStrengthPa = Count * Row.WithdrawalN / AreaM2;
	Profile.CompressiveStrengthPa = Row.BearingStrengthPa;
	Profile.TangentialStiffnessPaPerM = Count * Row.SlipModulusNPerM / AreaM2;
	Profile.NormalStiffnessPaPerM = Row.BearingNormalStiffnessPaPerM;
	return Profile;
}
}
