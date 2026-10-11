#include "loadpath/StructureFile.h"

#include "JsonReader.h"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace loadpath
{
namespace
{
using namespace detail;

constexpr char HexDigits[] = "0123456789abcdef";

std::string NameAt(const std::vector<std::string>& Names, std::uint32_t Index, char Prefix)
{
	return Index < Names.size() ? Names[Index] : Prefix + std::to_string(Index);
}

class Loader
{
public:
	Loader(LoadedStructure& InOut, FileError& InError)
		: Out(InOut)
		, Error(InError)
	{
	}

	bool Load(const JsonValue& Root)
	{
		ElementFields Document(Root, "document", Error);
		const JsonValue* Profiles = nullptr;
		const JsonValue* Bodies = nullptr;
		const JsonValue* Interfaces = nullptr;
		return Document.Elements("profiles", Profiles) && Document.Elements("bodies", Bodies) &&
			Document.Elements("interfaces", Interfaces) && Each(Profiles, &Loader::Profile) && Each(Bodies, &Loader::Body) &&
			Each(Interfaces, &Loader::Interface) && HighestPass(Document);
	}

private:
	bool HighestPass(ElementFields& Document)
	{
		double Pass = 0.0;
		if (!Document.Number("highest_pass", Pass, false))
		{
			return false;
		}
		if (!IsPassNumber(Pass))
		{
			return Document.Fail("highest_pass must be a whole number from 0 to 4294967295");
		}
		Out.Model.MarkPass(static_cast<std::uint32_t>(Pass));
		return true;
	}

	bool Each(const JsonValue* Array, bool (Loader::*Add)(const JsonValue&))
	{
		if (Array == nullptr)
		{
			return true;
		}
		for (const JsonValue& Element : Array->Items)
		{
			if (!(this->*Add)(Element))
			{
				return false;
			}
		}
		return true;
	}

	bool Profile(const JsonValue& Element)
	{
		ElementFields Fields(Element, "profile", Error);
		std::string Name;
		if (!Fields.String("name", Name))
		{
			return false;
		}
		Fields.Label = "profile " + Name;
		if (ProfileIds.contains(Name))
		{
			return Fields.Fail("duplicate name");
		}
		StrengthProfile Desc;
		const bool bRead = Fields.Number("compressive_pa", Desc.CompressiveStrengthPa, false) &&
			Fields.Number("cohesion_pa", Desc.ShearCohesionPa, false) && Fields.Number("tensile_pa", Desc.TensileStrengthPa, false) &&
			Fields.Number("friction", Desc.FrictionCoefficient, false) && Fields.Number("max_shear_pa", Desc.MaxShearStrengthPa, false) &&
			Fields.Number("normal_stiffness_pa_per_m", Desc.NormalStiffnessPaPerM, false) &&
			Fields.Number("tangential_stiffness_pa_per_m", Desc.TangentialStiffnessPaPerM, false);
		if (!bRead)
		{
			return false;
		}
		const Result<ProfileId> Added = Out.Model.AddProfile(Desc);
		if (!Added.Ok())
		{
			return Fields.Fail(ModelErrorText(Added.Error));
		}
		ProfileIds.emplace(Name, Added.Value);
		Out.ProfileNames.push_back(std::move(Name));
		return true;
	}

	bool Body(const JsonValue& Element)
	{
		ElementFields Fields(Element, "body", Error);
		std::string Name;
		if (!Fields.String("name", Name))
		{
			return false;
		}
		Fields.Label = "body " + Name;
		if (BodyIds.contains(Name))
		{
			return Fields.Fail("duplicate name");
		}
		BodyDesc Desc;
		const bool bRead = Fields.Number("mass_kg", Desc.MassKg, true) && Fields.Vector("centroid_m", Desc.CentroidM, false) &&
			Fields.Boolean("grounded", Desc.bGrounded);
		if (!bRead)
		{
			return false;
		}
		const Result<BodyId> Added = Out.Model.AddBody(Desc);
		if (!Added.Ok())
		{
			return Fields.Fail(ModelErrorText(Added.Error));
		}
		BodyIds.emplace(Name, Added.Value);
		Out.BodyNames.push_back(std::move(Name));
		return true;
	}

	bool Interface(const JsonValue& Element)
	{
		ElementFields Fields(Element, "interface", Error);
		std::string A;
		std::string B;
		std::string ProfileName;
		std::string Name;
		std::string StateName = "intact";
		double CrackPass = 0.0;
		double SeverPass = 0.0;
		InterfaceDesc Desc;
		const bool bRead = Fields.String("a", A) && Fields.String("b", B) && Fields.Vector("normal", Desc.NormalAToB, true) &&
			Fields.Vector("centre_m", Desc.CentreM, true) && Fields.Vector("u", Desc.AxisU, true) &&
			Fields.Vector("v", Desc.AxisV, true) && Fields.Number("half_u_m", Desc.HalfExtentUM, true) &&
			Fields.Number("half_v_m", Desc.HalfExtentVM, true) && Fields.String("profile", ProfileName) &&
			Fields.String("name", Name, false) && Fields.String("state", StateName, false) &&
			Fields.Number("crack_pass", CrackPass, false) && Fields.Number("sever_pass", SeverPass, false);
		if (!bRead)
		{
			return false;
		}
		const auto FoundA = BodyIds.find(A);
		if (FoundA == BodyIds.end())
		{
			return Fields.Fail("unknown body " + A + " in a");
		}
		const auto FoundB = BodyIds.find(B);
		if (FoundB == BodyIds.end())
		{
			return Fields.Fail("unknown body " + B + " in b");
		}
		const auto FoundProfile = ProfileIds.find(ProfileName);
		if (FoundProfile == ProfileIds.end())
		{
			return Fields.Fail("unknown profile " + ProfileName);
		}
		Desc.BodyA = FoundA->second;
		Desc.BodyB = FoundB->second;
		Desc.Profile = FoundProfile->second;
		const Result<InterfaceId> Added = Out.Model.AddInterface(Desc);
		if (!Added.Ok())
		{
			return Fields.Fail(ModelErrorText(Added.Error));
		}
		Out.InterfaceNames.push_back(std::move(Name));
		return ApplyState(Fields, Added.Value, StateName, CrackPass, SeverPass);
	}

	bool ApplyState(ElementFields& Fields, InterfaceId Id, const std::string& StateName, double CrackPass, double SeverPass)
	{
		if (!IsPassNumber(CrackPass) || !IsPassNumber(SeverPass))
		{
			return Fields.Fail("crack_pass and sever_pass must be whole numbers from 0 to 4294967295");
		}
		const std::uint32_t Crack = static_cast<std::uint32_t>(CrackPass);
		const std::uint32_t Sever = static_cast<std::uint32_t>(SeverPass);
		bool bApplied = false;
		if (StateName == "intact")
		{
			bApplied = Crack == 0 && Sever == 0;
		}
		else if (StateName == "cracked")
		{
			bApplied = Sever == 0 && Out.Model.Crack(Id, Crack);
		}
		else if (StateName == "severed")
		{
			bApplied = (Crack == 0 || (Crack <= Sever && Out.Model.Crack(Id, Crack))) && Out.Model.Sever(Id, Sever);
		}
		else
		{
			return Fields.Fail("unknown state " + StateName);
		}
		return bApplied || Fields.Fail("state " + StateName + " does not match its pass stamps");
	}

	static bool IsPassNumber(double X)
	{
		return X >= 0.0 && X <= 4294967295.0 && X == std::floor(X);
	}

	LoadedStructure& Out;
	FileError& Error;
	std::unordered_map<std::string, ProfileId> ProfileIds;
	std::unordered_map<std::string, BodyId> BodyIds;
};

class ObjectWriter
{
public:
	explicit ObjectWriter(std::string& InOut)
		: Out(InOut)
	{
	}

	~ObjectWriter() { Out += " }"; }

	ObjectWriter(const ObjectWriter&) = delete;
	ObjectWriter& operator=(const ObjectWriter&) = delete;

	void String(std::string_view Key, std::string_view Value)
	{
		BeginField(Key);
		Out += '"';
		for (const char C : Value)
		{
			const unsigned char Byte = static_cast<unsigned char>(C);
			if (Byte < 0x20u)
			{
				Out += "\\u00";
				Out += HexDigits[Byte >> 4];
				Out += HexDigits[Byte & 0xFu];
				continue;
			}
			if (C == '"' || C == '\\')
			{
				Out += '\\';
			}
			Out += C;
		}
		Out += '"';
	}

	void Number(std::string_view Key, double Value)
	{
		BeginField(Key);
		AppendNumber(Value);
	}

	void Boolean(std::string_view Key, bool Value)
	{
		BeginField(Key);
		Out += Value ? "true" : "false";
	}

	void Vector(std::string_view Key, const Vec3& Value)
	{
		BeginField(Key);
		Out += '[';
		AppendNumber(Value.X);
		Out += ", ";
		AppendNumber(Value.Y);
		Out += ", ";
		AppendNumber(Value.Z);
		Out += ']';
	}

private:
	void BeginField(std::string_view Key)
	{
		Out += bFirst ? "{ \"" : ", \"";
		bFirst = false;
		Out += Key;
		Out += "\": ";
	}

	void AppendNumber(double Value)
	{
		std::array<char, 32> Buffer{};
		const std::to_chars_result Written = std::to_chars(Buffer.data(), Buffer.data() + Buffer.size(), Value);
		Out.append(Buffer.data(), Written.ptr);
	}

	std::string& Out;
	bool bFirst = true;
};

void BeginElement(std::string& Out, bool bFirst)
{
	Out += bFirst ? "\n    " : ",\n    ";
}
}

LoadResult LoadStructure(std::string_view Json)
{
	LoadResult Result;
	JsonParser Parser(Json);
	JsonValue Root;
	if (!Parser.ParseDocument(Root))
	{
		Result.Error = Parser.Error;
		return Result;
	}
	if (!Loader(Result.Value, Result.Error).Load(Root))
	{
		Result.Value = LoadedStructure{};
	}
	return Result;
}

std::string WriteStructure(const LoadedStructure& Loaded)
{
	const Structure& Model = Loaded.Model;
	std::string Out = "{\n  \"profiles\": [";
	for (std::uint32_t Index = 0; Model.IsValid(ProfileId{Index, 1u}); ++Index)
	{
		const StrengthProfile& P = Model.Profile(ProfileId{Index, 1u});
		BeginElement(Out, Index == 0);
		ObjectWriter Writer(Out);
		Writer.String("name", NameAt(Loaded.ProfileNames, Index, 'p'));
		Writer.Number("compressive_pa", P.CompressiveStrengthPa);
		Writer.Number("cohesion_pa", P.ShearCohesionPa);
		Writer.Number("tensile_pa", P.TensileStrengthPa);
		Writer.Number("friction", P.FrictionCoefficient);
		if (P.MaxShearStrengthPa != Infinity)
		{
			Writer.Number("max_shear_pa", P.MaxShearStrengthPa);
		}
		Writer.Number("normal_stiffness_pa_per_m", P.NormalStiffnessPaPerM);
		Writer.Number("tangential_stiffness_pa_per_m", P.TangentialStiffnessPaPerM);
	}
	Out += "\n  ],\n  \"bodies\": [";
	bool bFirst = true;
	for (const BodyId& Id : Model.LiveBodies())
	{
		const BodyDesc& Body = Model.Body(Id);
		BeginElement(Out, bFirst);
		bFirst = false;
		ObjectWriter Writer(Out);
		Writer.String("name", NameAt(Loaded.BodyNames, Id.Index, 'b'));
		Writer.Number("mass_kg", Body.MassKg);
		Writer.Vector("centroid_m", Body.CentroidM);
		Writer.Boolean("grounded", Body.bGrounded);
	}
	Out += "\n  ],\n  \"interfaces\": [";
	bFirst = true;
	for (std::uint32_t Index = 0; Model.Interface(InterfaceId{Index, 1u}).BodyA.Generation != 0; ++Index)
	{
		const InterfaceRecord& J = Model.Interface(InterfaceId{Index, 1u});
		if (!Model.IsLive(J.BodyA) || !Model.IsLive(J.BodyB))
		{
			continue;
		}
		BeginElement(Out, bFirst);
		bFirst = false;
		ObjectWriter Writer(Out);
		Writer.String("a", NameAt(Loaded.BodyNames, J.BodyA.Index, 'b'));
		Writer.String("b", NameAt(Loaded.BodyNames, J.BodyB.Index, 'b'));
		Writer.Vector("normal", J.NormalAToB);
		Writer.Vector("centre_m", J.CentreM);
		Writer.Vector("u", J.AxisU);
		Writer.Vector("v", J.AxisV);
		Writer.Number("half_u_m", J.HalfExtentUM);
		Writer.Number("half_v_m", J.HalfExtentVM);
		Writer.String("profile", NameAt(Loaded.ProfileNames, J.Profile.Index, 'p'));
		if (Index < Loaded.InterfaceNames.size() && !Loaded.InterfaceNames[Index].empty())
		{
			Writer.String("name", Loaded.InterfaceNames[Index]);
		}
		if (J.State != InterfaceState::Intact)
		{
			Writer.String("state", J.State == InterfaceState::Cracked ? "cracked" : "severed");
			Writer.Number("crack_pass", J.CrackPass);
			Writer.Number("sever_pass", J.SeverPass);
		}
	}
	Out += "\n  ]";
	if (Model.HighestPass() > 0)
	{
		Out += ",\n  \"highest_pass\": " + std::to_string(Model.HighestPass());
	}
	Out += "\n}\n";
	return Out;
}
}
