#include "loadpath/Scenario.h"

#include "JsonReader.h"
#include "StateNames.h"

#include "loadpath/geometry/RunningBondWall.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <map>
#include <set>
#include <utility>

namespace loadpath
{
namespace
{
using namespace detail;

constexpr double ToleranceM = 1e-9;
constexpr double PaPerMPa = 1e6;
constexpr std::uint32_t FirstGeneration = 1;
constexpr std::string_view GroundMaterial = "ground";

std::string Quoted(std::string_view Text) { return "'" + std::string(Text) + "'"; }

bool ParseRange(std::string_view Pattern, std::size_t Open, std::size_t& Close, long long& Low, long long& High)
{
	Close = Pattern.find(']', Open);
	if (Close == std::string_view::npos)
	{
		return false;
	}
	const std::string_view Body = Pattern.substr(Open + 1, Close - Open - 1);
	const std::size_t Dash = Body.find('-');
	if (Dash == std::string_view::npos || Dash == 0 || Dash + 1 >= Body.size() || Dash > 9 || Body.size() - Dash - 1 > 9)
	{
		return false;
	}
	Low = 0;
	High = 0;
	for (std::size_t I = 0; I < Body.size(); ++I)
	{
		if (I == Dash)
		{
			continue;
		}
		if (Body[I] < '0' || Body[I] > '9')
		{
			return false;
		}
		long long& Into = I < Dash ? Low : High;
		Into = Into * 10 + (Body[I] - '0');
	}
	return Low <= High;
}

bool PatternIsValid(std::string_view Pattern)
{
	for (std::size_t I = 0; I < Pattern.size(); ++I)
	{
		if (Pattern[I] == '[')
		{
			std::size_t Close = 0;
			long long Low = 0;
			long long High = 0;
			if (!ParseRange(Pattern, I, Close, Low, High))
			{
				return false;
			}
			I = Close;
		}
	}
	return true;
}

bool MatchFrom(std::string_view Pattern, std::size_t P, std::string_view Name, std::size_t N)
{
	while (P < Pattern.size())
	{
		const char C = Pattern[P];
		if (C == '*')
		{
			for (std::size_t K = N; K <= Name.size(); ++K)
			{
				if (MatchFrom(Pattern, P + 1, Name, K))
				{
					return true;
				}
			}
			return false;
		}
		if (C == '[')
		{
			std::size_t Close = 0;
			long long Low = 0;
			long long High = 0;
			if (!ParseRange(Pattern, P, Close, Low, High))
			{
				return false;
			}
			std::size_t End = N;
			long long Value = 0;
			while (End < Name.size() && Name[End] >= '0' && Name[End] <= '9' && End - N < 10)
			{
				Value = Value * 10 + (Name[End] - '0');
				++End;
			}
			if (End == N || (End < Name.size() && Name[End] >= '0' && Name[End] <= '9') || Value < Low || Value > High)
			{
				return false;
			}
			N = End;
			P = Close + 1;
			continue;
		}
		if (N >= Name.size() || Name[N] != C)
		{
			return false;
		}
		++P;
		++N;
	}
	return N == Name.size();
}

bool Matches(std::string_view Pattern, std::string_view Name) { return MatchFrom(Pattern, 0, Name, 0); }

bool JointMatches(std::string_view Pattern, std::string_view Joint)
{
	const std::size_t PatternSlash = Pattern.find('/');
	const std::size_t JointSlash = Joint.find('/');
	if (PatternSlash == std::string_view::npos || JointSlash == std::string_view::npos)
	{
		return false;
	}
	const std::string_view P0 = Pattern.substr(0, PatternSlash);
	const std::string_view P1 = Pattern.substr(PatternSlash + 1);
	const std::string_view J0 = Joint.substr(0, JointSlash);
	const std::string_view J1 = Joint.substr(JointSlash + 1);
	return (Matches(P0, J0) && Matches(P1, J1)) || (Matches(P0, J1) && Matches(P1, J0));
}

const JsonValue* Get(const JsonValue& Object, std::string_view Key) { return Find(Object, Key); }

struct BoxSpec
{
	std::string Name, Material;
	geometry::Box Shape;
	bool bGrounded = false;
	double DensityKgPerM3 = 0.0;
	int Line = 0;
};

struct WallItem
{
	std::string Name, On, Material, Bed, Perpend;
	int Courses = 0, Bricks = 0;
	Vec3 OriginM;
	Vec3 BrickM{0.215, 0.1025, 0.065};
	double DensityKgPerM3 = 0.0;
	std::vector<std::pair<std::string, int>> Omit;
	int Line = 0;
};

struct BuildItem
{
	bool bWall = false;
	std::vector<BoxSpec> Boxes;
	WallItem Wall;
};

struct PlacedBox
{
	geometry::Box Shape;
	int Line = 0;
	int Wall = -1;
	int OnSlot = -1;
};

enum class RowKind : std::uint8_t { Joint, Fastener };

struct Rule
{
	bool bBetween = false;
	std::array<std::string, 2> Pair;
	int Plane = 0;
	std::string Row;
	RowKind Kind = RowKind::Joint;
	int Count = 0;
	int Line = 0;
};

class Loader
{
public:
	Loader(const Catalogue& InRows, Scenario& InOut, FileError& InError)
		: Rows(InRows)
		, Out(InOut)
		, Error(InError)
	{
	}

	bool Load(const JsonValue& Root)
	{
		std::vector<BuildItem> Items;
		std::vector<Rule> Rules;
		return OnlyKeys(Root, {"title", "summary", "deferred", "tier", "materials", "profiles", "build", "joints", "steps", "expect"}) &&
			ReadHeader(Root) && ReadMaterials(Root) && ReadProfiles(Root) && ReadBuild(Root, Items) && ReadRules(Root, Rules) &&
			CheckNames(Items) && ResolveWalls(Items) && Place(Items) && CheckOverlaps() && Join(Rules) && ReadSteps(Root) && ReadExpect(Root);
	}

private:
	bool Fail(int Line, std::string Message)
	{
		Error = {Line, std::move(Message)};
		return false;
	}

	bool OnlyKeys(const JsonValue& Object, std::initializer_list<std::string_view> Allowed)
	{
		for (const std::string& Key : Object.Keys)
		{
			if (std::find(Allowed.begin(), Allowed.end(), Key) == Allowed.end())
			{
				return Fail(Object.Line, "unknown key " + Quoted(Key));
			}
		}
		return true;
	}

	bool Typed(const JsonValue& Object, std::string_view Key, JsonType Type, bool bRequired, const JsonValue*& Value)
	{
		Value = Get(Object, Key);
		if (Value == nullptr)
		{
			return !bRequired || Fail(Object.Line, "missing " + Quoted(Key));
		}
		return Value->Type == Type || Fail(Value->Line, Quoted(Key) + " has the wrong type");
	}

	bool Text(const JsonValue& Object, std::string_view Key, bool bRequired, std::string& Into)
	{
		const JsonValue* Value = nullptr;
		if (!Typed(Object, Key, JsonType::String, bRequired, Value))
		{
			return false;
		}
		if (Value != nullptr)
		{
			Into = Value->Text;
		}
		return true;
	}

	bool Vector(const JsonValue& Object, std::string_view Key, bool bRequired, Vec3& Into)
	{
		const JsonValue* Value = nullptr;
		if (!Typed(Object, Key, JsonType::Array, bRequired, Value))
		{
			return false;
		}
		if (Value == nullptr)
		{
			return true;
		}
		const std::vector<JsonValue>& Items = Value->Items;
		if (Items.size() != 3 || Items[0].Type != JsonType::Number || Items[1].Type != JsonType::Number || Items[2].Type != JsonType::Number)
		{
			return Fail(Value->Line, Quoted(Key) + " must hold 3 numbers");
		}
		Into = Vec3{Items[0].Number, Items[1].Number, Items[2].Number};
		return true;
	}

	bool Positive(const Vec3& V, int Line, std::string_view Key)
	{
		for (const double C : {V.X, V.Y, V.Z})
		{
			if (!(C > 0.0) || !std::isfinite(C))
			{
				return Fail(Line, std::string(Key) + " must be finite and positive");
			}
		}
		return true;
	}

	bool WholeNumber(const JsonValue& Value, std::string_view Key, double Minimum, int& Into)
	{
		if (Value.Type != JsonType::Number || !(Value.Number >= Minimum) || Value.Number > 1e9 || Value.Number != std::floor(Value.Number))
		{
			return Fail(Value.Line, std::string(Key) + " must be a whole number of at least " + std::to_string(static_cast<int>(Minimum)));
		}
		Into = static_cast<int>(Value.Number);
		return true;
	}

	bool ValidName(const std::string& Name, int Line)
	{
		if (Name.find('.') != std::string::npos)
		{
			return Fail(Line, "name " + Quoted(Name) + " contains a dot; dots are reserved for generated names");
		}
		const bool bValid = !Name.empty() && std::all_of(Name.begin(), Name.end(), [](char C) {
			return (C >= 'a' && C <= 'z') || (C >= 'A' && C <= 'Z') || (C >= '0' && C <= '9') || C == '_' || C == '-';
		});
		return bValid || Fail(Line, "name " + Quoted(Name) + " may hold only letters, digits, '_' and '-'");
	}

	bool ReadHeader(const JsonValue& Root)
	{
		if (!Text(Root, "title", true, Out.Title) || !Text(Root, "summary", false, Out.Summary))
		{
			return false;
		}
		const JsonValue* Deferred = nullptr;
		if (!Typed(Root, "deferred", JsonType::String, false, Deferred))
		{
			return false;
		}
		if (Deferred != nullptr)
		{
			if (Deferred->Text.empty())
			{
				return Fail(Deferred->Line, "deferred must not be empty");
			}
			Out.Deferred = Deferred->Text;
		}
		const JsonValue* Tier = nullptr;
		if (!Typed(Root, "tier", JsonType::String, false, Tier))
		{
			return false;
		}
		if (Tier != nullptr)
		{
			if (Tier->Text == "default")
			{
				Out.Tier = ScenarioTier::Default;
			}
			else if (Tier->Text == "slow_in_debug")
			{
				Out.Tier = ScenarioTier::SlowInDebug;
			}
			else if (Tier->Text == "slow")
			{
				Out.Tier = ScenarioTier::Slow;
			}
			else
			{
				return Fail(Tier->Line, "unknown tier " + Quoted(Tier->Text));
			}
		}
		return true;
	}

	bool ReadMaterials(const JsonValue& Root)
	{
		const JsonValue* Array = nullptr;
		if (!Typed(Root, "materials", JsonType::Array, false, Array) || Array == nullptr)
		{
			return Array == nullptr && Error.Message.empty();
		}
		for (const JsonValue& Item : Array->Items)
		{
			if (Item.Type != JsonType::Object)
			{
				return Fail(Item.Line, "a material must be an object");
			}
			std::string Name;
			const JsonValue* Density = nullptr;
			if (!OnlyKeys(Item, {"name", "density_kg_per_m3"}) || !Text(Item, "name", true, Name) ||
				!Typed(Item, "density_kg_per_m3", JsonType::Number, true, Density))
			{
				return false;
			}
			if (Rows.FindMaterial(Name) != nullptr || Name == GroundMaterial)
			{
				return Fail(Item.Line, "material " + Quoted(Name) + " is already in the catalogue");
			}
			if (LocalDensities.count(Name) != 0)
			{
				return Fail(Item.Line, "duplicate material " + Quoted(Name));
			}
			if (!(Density->Number > 0.0) || !std::isfinite(Density->Number))
			{
				return Fail(Item.Line, "density_kg_per_m3 must be finite and positive");
			}
			LocalDensities[Name] = Density->Number;
		}
		return true;
	}

	bool ReadProfiles(const JsonValue& Root)
	{
		const JsonValue* Array = nullptr;
		if (!Typed(Root, "profiles", JsonType::Array, false, Array) || Array == nullptr)
		{
			return Array == nullptr && Error.Message.empty();
		}
		for (const JsonValue& Item : Array->Items)
		{
			if (Item.Type != JsonType::Object)
			{
				return Fail(Item.Line, "a profile must be an object");
			}
			std::string Name;
			std::string From;
			if (!OnlyKeys(Item, {"name", "from", "cohesion_mpa", "tensile_mpa", "friction", "shear_cap_mpa"}) ||
				!Text(Item, "name", true, Name) || !Text(Item, "from", true, From))
			{
				return false;
			}
			if (Rows.FindJoint(Name) != nullptr || Rows.FindFastener(Name) != nullptr)
			{
				return Fail(Item.Line, "profile " + Quoted(Name) + " is already in the catalogue");
			}
			if (LocalProfiles.count(Name) != 0)
			{
				return Fail(Item.Line, "duplicate profile " + Quoted(Name));
			}
			const JointRow* Base = Rows.FindJoint(From);
			if (Base == nullptr)
			{
				return Fail(Item.Line, "unknown row " + Quoted(From));
			}
			StrengthProfile P = Base->Profile;
			if (!Override(Item, "cohesion_mpa", PaPerMPa, P.ShearCohesionPa) ||
				!Override(Item, "tensile_mpa", PaPerMPa, P.TensileStrengthPa) || !Override(Item, "friction", 1.0, P.FrictionCoefficient) ||
				!Override(Item, "shear_cap_mpa", PaPerMPa, P.MaxShearStrengthPa))
			{
				return false;
			}
			LocalProfiles[Name] = P;
		}
		return true;
	}

	bool Override(const JsonValue& Item, std::string_view Key, double Scale, double& Into)
	{
		const JsonValue* Value = Get(Item, Key);
		if (Value == nullptr)
		{
			return true;
		}
		if (Value->Type == JsonType::Null && Key == "shear_cap_mpa")
		{
			Into = Infinity;
			return true;
		}
		if (Value->Type != JsonType::Number || !(Value->Number >= 0.0) || !std::isfinite(Value->Number * Scale))
		{
			return Fail(Value->Line, std::string(Key) + " must be a finite number, not negative");
		}
		Into = Value->Number * Scale;
		return true;
	}

	bool Density(const std::string& Material, bool bGrounded, int Line, double& Into)
	{
		if (Material == GroundMaterial)
		{
			Into = 0.0;
			return bGrounded || Fail(Line, "material 'ground' is for grounded boxes only");
		}
		if (const MaterialRow* Row = Rows.FindMaterial(Material))
		{
			Into = Row->DensityKgPerM3;
			return true;
		}
		const auto Local = LocalDensities.find(Material);
		if (Local != LocalDensities.end())
		{
			Into = Local->second;
			return true;
		}
		return Fail(Line, "unknown material " + Quoted(Material));
	}

	bool IsJointRow(const std::string& Row) const { return LocalProfiles.count(Row) != 0 || Rows.FindJoint(Row) != nullptr; }

	bool ReadBuild(const JsonValue& Root, std::vector<BuildItem>& Items)
	{
		const JsonValue* Array = nullptr;
		if (!Typed(Root, "build", JsonType::Array, true, Array))
		{
			return false;
		}
		for (const JsonValue& Item : Array->Items)
		{
			if (Item.Type != JsonType::Object)
			{
				return Fail(Item.Line, "a build item must be an object");
			}
			BuildItem Built;
			Built.bWall = Get(Item, "wall") != nullptr;
			if (!(Built.bWall ? ReadWall(Item, Built.Wall) : ReadBox(Item, Built.Boxes)))
			{
				return false;
			}
			Items.push_back(std::move(Built));
		}
		return true;
	}

	bool ReadBox(const JsonValue& Item, std::vector<BoxSpec>& Into)
	{
		BoxSpec Box;
		Box.Line = Item.Line;
		Vec3 Size;
		Vec3 Centre;
		Vec3 Min;
		if (!OnlyKeys(Item, {"name", "size", "centre", "min", "material", "grounded", "repeat"}) || !Text(Item, "name", true, Box.Name) ||
			!ValidName(Box.Name, Item.Line) || !Vector(Item, "size", true, Size) || !Positive(Size, Item.Line, "size") ||
			!Vector(Item, "centre", false, Centre) || !Vector(Item, "min", false, Min))
		{
			return false;
		}
		const bool bCentre = Get(Item, "centre") != nullptr;
		if (bCentre == (Get(Item, "min") != nullptr))
		{
			return Fail(Item.Line, "exactly one of 'centre' and 'min'");
		}
		const JsonValue* Grounded = nullptr;
		if (!Typed(Item, "grounded", JsonType::Bool, false, Grounded))
		{
			return false;
		}
		Box.bGrounded = Grounded != nullptr && Grounded->Bool;
		Box.Material = Box.bGrounded ? std::string(GroundMaterial) : std::string();
		if (!Text(Item, "material", !Box.bGrounded, Box.Material) || !Density(Box.Material, Box.bGrounded, Item.Line, Box.DensityKgPerM3))
		{
			return false;
		}
		const Vec3 Half{0.5 * Size.X, 0.5 * Size.Y, 0.5 * Size.Z};
		Box.Shape = geometry::Box{bCentre ? Centre : Vec3{Min.X + Half.X, Min.Y + Half.Y, Min.Z + Half.Z}, Half};
		const JsonValue* Repeat = nullptr;
		if (!Typed(Item, "repeat", JsonType::Object, false, Repeat))
		{
			return false;
		}
		if (Repeat == nullptr)
		{
			Into.push_back(Box);
			return true;
		}
		int Count = 0;
		Vec3 Step;
		const JsonValue* CountValue = Get(*Repeat, "count");
		if (!OnlyKeys(*Repeat, {"count", "step"}))
		{
			return false;
		}
		if (CountValue == nullptr)
		{
			return Fail(Repeat->Line, "missing 'count'");
		}
		if (!WholeNumber(*CountValue, "count", 1.0, Count) || !Vector(*Repeat, "step", true, Step))
		{
			return false;
		}
		for (int K = 0; K < Count; ++K)
		{
			BoxSpec Copy = Box;
			Copy.Name = Box.Name + "." + std::to_string(K);
			const Vec3 C = Box.Shape.CentreM;
			Copy.Shape.CentreM = Vec3{C.X + K * Step.X, C.Y + K * Step.Y, C.Z + K * Step.Z};
			Into.push_back(std::move(Copy));
		}
		return true;
	}

	bool ReadWall(const JsonValue& Item, WallItem& Wall)
	{
		Wall.Line = Item.Line;
		if (!OnlyKeys(Item, {"wall", "courses", "bricks", "origin", "on", "material", "bed", "perpend", "omit"}) ||
			!Text(Item, "wall", true, Wall.Name) || !ValidName(Wall.Name, Item.Line))
		{
			return false;
		}
		for (const auto& [Key, Into] : {std::pair<std::string_view, int*>{"courses", &Wall.Courses}, std::pair<std::string_view, int*>{"bricks", &Wall.Bricks}})
		{
			const JsonValue* Value = Get(Item, Key);
			if (Value == nullptr)
			{
				return Fail(Item.Line, "missing " + Quoted(Key));
			}
			if (!WholeNumber(*Value, Key, 1.0, *Into))
			{
				return false;
			}
		}
		if (!Vector(Item, "origin", false, Wall.OriginM) || !Text(Item, "on", true, Wall.On) ||
			!Text(Item, "material", true, Wall.Material) || !Text(Item, "bed", true, Wall.Bed) || !Text(Item, "perpend", true, Wall.Perpend) ||
			!Density(Wall.Material, false, Item.Line, Wall.DensityKgPerM3))
		{
			return false;
		}
		for (const std::string* Row : {&Wall.Bed, &Wall.Perpend})
		{
			if (!IsJointRow(*Row))
			{
				return Fail(Item.Line, "unknown row " + Quoted(*Row) + " for a wall joint");
			}
		}
		const JsonValue* Omit = nullptr;
		if (!Typed(Item, "omit", JsonType::Array, false, Omit))
		{
			return false;
		}
		if (Omit != nullptr)
		{
			for (const JsonValue& Pattern : Omit->Items)
			{
				if (Pattern.Type != JsonType::String)
				{
					return Fail(Pattern.Line, "omit must hold names");
				}
				Wall.Omit.emplace_back(Pattern.Text, Pattern.Line);
			}
		}
		return true;
	}

	bool ReadRules(const JsonValue& Root, std::vector<Rule>& Rules)
	{
		const JsonValue* Array = nullptr;
		if (!Typed(Root, "joints", JsonType::Array, false, Array) || Array == nullptr)
		{
			return Array == nullptr && Error.Message.empty();
		}
		for (const JsonValue& Item : Array->Items)
		{
			if (Item.Type != JsonType::Object)
			{
				return Fail(Item.Line, "a joint rule must be an object");
			}
			Rule R;
			R.Line = Item.Line;
			if (!OnlyKeys(Item, {"between", "materials", "plane", "row", "count"}))
			{
				return false;
			}
			const JsonValue* Between = Get(Item, "between");
			const JsonValue* Materials = Get(Item, "materials");
			if ((Between == nullptr) == (Materials == nullptr))
			{
				return Fail(Item.Line, "exactly one of 'between' and 'materials'");
			}
			R.bBetween = Between != nullptr;
			const JsonValue& Pair = R.bBetween ? *Between : *Materials;
			if (Pair.Type != JsonType::Array || Pair.Items.size() != 2 || Pair.Items[0].Type != JsonType::String || Pair.Items[1].Type != JsonType::String)
			{
				return Fail(Pair.Line, std::string(R.bBetween ? "'between'" : "'materials'") + " must hold 2 names");
			}
			R.Pair = {Pair.Items[0].Text, Pair.Items[1].Text};
			if (R.bBetween && (!PatternIsValid(R.Pair[0]) || !PatternIsValid(R.Pair[1])))
			{
				return Fail(Pair.Line, "bad range in a 'between' pattern");
			}
			std::string Plane;
			if (!Text(Item, "plane", false, Plane) || !Text(Item, "row", true, R.Row))
			{
				return false;
			}
			if (!Plane.empty() || Get(Item, "plane") != nullptr)
			{
				if (Plane != "bed" && Plane != "side")
				{
					return Fail(Item.Line, "plane must be 'bed' or 'side'");
				}
				R.Plane = Plane == "bed" ? 1 : 2;
			}
			const JsonValue* Count = Get(Item, "count");
			if (IsJointRow(R.Row))
			{
				if (Count != nullptr)
				{
					return Fail(Item.Line, "row " + Quoted(R.Row) + " is a joint row and takes no count");
				}
			}
			else if (Rows.FindFastener(R.Row) != nullptr)
			{
				R.Kind = RowKind::Fastener;
				if (Count == nullptr)
				{
					return Fail(Item.Line, "row " + Quoted(R.Row) + " is a fastener and needs a count");
				}
				if (!WholeNumber(*Count, "count", 1.0, R.Count))
				{
					return false;
				}
			}
			else
			{
				return Fail(Item.Line, "unknown row " + Quoted(R.Row));
			}
			Rules.push_back(std::move(R));
		}
		return true;
	}

	bool CheckNames(const std::vector<BuildItem>& Items)
	{
		std::set<std::string> Seen;
		for (const BuildItem& Item : Items)
		{
			if (Item.bWall)
			{
				if (!Seen.insert(Item.Wall.Name).second)
				{
					return Fail(Item.Wall.Line, "duplicate name " + Quoted(Item.Wall.Name));
				}
				continue;
			}
			for (const BoxSpec& Box : Item.Boxes)
			{
				if (!Seen.insert(Box.Name).second)
				{
					return Fail(Box.Line, "duplicate name " + Quoted(Box.Name));
				}
			}
		}
		return true;
	}

	const BoxSpec* FindBox(const std::vector<BuildItem>& Items, const std::string& Name) const
	{
		for (const BuildItem& Item : Items)
		{
			for (const BoxSpec& Box : Item.Boxes)
			{
				if (Box.Name == Name)
				{
					return &Box;
				}
			}
		}
		return nullptr;
	}

	bool ResolveWalls(const std::vector<BuildItem>& Items)
	{
		for (const BuildItem& Item : Items)
		{
			if (!Item.bWall)
			{
				continue;
			}
			const WallItem& Wall = Item.Wall;
			const BoxSpec* On = FindBox(Items, Wall.On);
			if (On == nullptr)
			{
				return Fail(Wall.Line, "the wall's 'on' names no box: " + Quoted(Wall.On));
			}
			if (!On->bGrounded)
			{
				return Fail(Wall.Line, Quoted(Wall.On) + " is not grounded; a wall stands on a grounded box");
			}
			const double TopM = On->Shape.CentreM.Z + On->Shape.HalfExtentsM.Z;
			if (!(std::abs(TopM - Wall.OriginM.Z) <= ToleranceM))
			{
				return Fail(Wall.Line, "the top of " + Quoted(Wall.On) + " is not at the wall's base level");
			}
			const std::vector<std::string> Names = BrickNames(Wall);
			for (const auto& [Pattern, Line] : Wall.Omit)
			{
				if (!PatternIsValid(Pattern) || std::none_of(Names.begin(), Names.end(), [&](const std::string& N) { return Matches(Pattern, N); }))
				{
					return Fail(Line, "omit pattern " + Quoted(Pattern) + " matches no brick");
				}
			}
		}
		return true;
	}

	static int BricksInCourse(const WallItem& Wall, int Course) { return Course % 2 == 1 ? Wall.Bricks + 1 : Wall.Bricks; }

	static std::string BrickName(const WallItem& Wall, int Course, int Brick)
	{
		return Wall.Name + ".c" + std::to_string(Course) + ".b" + std::to_string(Brick);
	}

	static std::vector<std::string> BrickNames(const WallItem& Wall)
	{
		std::vector<std::string> Names;
		for (int C = 0; C < Wall.Courses; ++C)
		{
			for (int K = 0; K < BricksInCourse(Wall, C); ++K)
			{
				Names.push_back(BrickName(Wall, C, K));
			}
		}
		return Names;
	}

	ProfileId SharedProfile(const std::string& Key, const std::string& Name, const StrengthProfile& Profile)
	{
		const auto Found = SharedProfiles.find(Key);
		if (Found != SharedProfiles.end())
		{
			return Found->second;
		}
		const Result<ProfileId> Added = Out.Model.AddProfile(Profile);
		if (Added.Ok())
		{
			SharedProfiles[Key] = Added.Value;
			Out.ProfileNames.resize(Added.Value.Index + 1);
			Out.ProfileNames[Added.Value.Index] = Name;
		}
		return Added.Value;
	}

	ProfileId JointProfile(const std::string& Row)
	{
		const auto Local = LocalProfiles.find(Row);
		return SharedProfile(Row, Row, Local != LocalProfiles.end() ? Local->second : Rows.FindJoint(Row)->Profile);
	}

	BodyId AddBody(const std::string& Name, const std::string& Material, const geometry::Box& Shape, double MassKg, bool bGrounded, PlacedBox Placed)
	{
		const Result<BodyId> Added = Out.Model.AddBody(BodyDesc{MassKg, Shape.CentreM, bGrounded});
		if (!Added.Ok())
		{
			Fail(Placed.Line, "body " + Quoted(Name) + ": " + ModelErrorText(Added.Error));
			return BodyId::Invalid();
		}
		Out.BodyIds.push_back(Added.Value);
		Out.BodyNames.push_back(Name);
		Out.BodyMaterials.push_back(Material);
		Out.BodyBoxes.push_back(Shape);
		Out.BodyMassesKg.push_back(MassKg);
		Out.BodyGrounded.push_back(bGrounded);
		Placed.Shape = Shape;
		Boxes.push_back(Placed);
		return Added.Value;
	}

	bool Place(const std::vector<BuildItem>& Items)
	{
		std::map<std::string, int> SlotOf;
		int WallIndex = 0;
		for (const BuildItem& Item : Items)
		{
			if (!Item.bWall)
			{
				for (const BoxSpec& Box : Item.Boxes)
				{
					const Vec3 H = Box.Shape.HalfExtentsM;
					const double MassKg = Box.bGrounded ? 0.0 : Box.DensityKgPerM3 * 8.0 * H.X * H.Y * H.Z;
					PlacedBox Placed;
					Placed.Line = Box.Line;
					if (AddBody(Box.Name, Box.Material, Box.Shape, MassKg, Box.bGrounded, Placed) == BodyId::Invalid())
					{
						return false;
					}
					SlotOf[Box.Name] = static_cast<int>(Out.BodyNames.size()) - 1;
				}
				continue;
			}
			const auto On = SlotOf.find(Item.Wall.On);
			if (On == SlotOf.end())
			{
				return Fail(Item.Wall.Line, "the wall's 'on' box " + Quoted(Item.Wall.On) + " must come before the wall");
			}
			if (!PlaceWall(Item.Wall, On->second, WallIndex++))
			{
				return false;
			}
		}
		return true;
	}

	bool PlaceWall(const WallItem& Wall, int OnSlot, int WallIndex)
	{
		Structure Scratch;
		const Result<BodyId> ScratchGround = Scratch.AddBody(BodyDesc{0.0, Out.BodyBoxes[static_cast<std::size_t>(OnSlot)].CentreM, true});
		const StrengthProfile BedProfile = Out.Model.Profile(JointProfile(Wall.Bed));
		const StrengthProfile PerpendProfile = Out.Model.Profile(JointProfile(Wall.Perpend));
		geometry::WallSpec Spec;
		Spec.Courses = Wall.Courses;
		Spec.BricksPerCourse = Wall.Bricks;
		Spec.BrickM = Wall.BrickM;
		Spec.DensityKgPerM3 = Wall.DensityKgPerM3;
		Spec.Bed = Scratch.AddProfile(BedProfile).Value;
		Spec.Perpend = Scratch.AddProfile(PerpendProfile).Value;
		Spec.bRunningBond = true;
		Spec.OriginM = Wall.OriginM;
		Spec.bAlongY = false;
		Spec.Ground = ScratchGround.Value;
		const geometry::Wall Built = geometry::BuildRunningBondWall(Scratch, Spec);
		if (!Built.bComplete)
		{
			return Fail(Wall.Line, "the wall " + Quoted(Wall.Name) + " could not be built");
		}
		std::vector<BodyId> Map(Scratch.LiveBodyCount() + 1, BodyId::Invalid());
		Map[ScratchGround.Value.Index] = Out.BodyIds[static_cast<std::size_t>(OnSlot)];
		for (int C = 0; C < Wall.Courses; ++C)
		{
			const int Count = static_cast<int>(Built.Courses[static_cast<std::size_t>(C)].size());
			for (int K = 0; K < Count; ++K)
			{
				const std::string Name = BrickName(Wall, C, K);
				if (std::any_of(Wall.Omit.begin(), Wall.Omit.end(), [&](const auto& Omit) { return Matches(Omit.first, Name); }))
				{
					continue;
				}
				const BodyId ScratchId = Built.Courses[static_cast<std::size_t>(C)][static_cast<std::size_t>(K)];
				const BodyDesc& Desc = Scratch.Body(ScratchId);
				const bool bHalf = C % 2 == 1 && (K == 0 || K == Count - 1);
				const double HalfLength = (bHalf ? 0.5 : 1.0) * Wall.BrickM.X * 0.5;
				const double HalfThickness = 0.5 * Wall.BrickM.Y;
				const Vec3 Half = Vec3{HalfLength, HalfThickness, 0.5 * Wall.BrickM.Z};
				PlacedBox Placed;
				Placed.Line = Wall.Line;
				Placed.Wall = WallIndex;
				Placed.OnSlot = OnSlot;
				const BodyId Id = AddBody(Name, Wall.Material, geometry::Box{Desc.CentroidM, Half}, Desc.MassKg, false, Placed);
				if (Id == BodyId::Invalid())
				{
					return false;
				}
				Map[ScratchId.Index] = Id;
			}
		}
		const ProfileId Bed = JointProfile(Wall.Bed);
		const ProfileId Perpend = JointProfile(Wall.Perpend);
		for (std::uint32_t Slot = 0; Scratch.IsKnown(InterfaceId{Slot, FirstGeneration}); ++Slot)
		{
			const InterfaceRecord& R = Scratch.Interface(InterfaceId{Slot, FirstGeneration});
			const BodyId A = Map[R.BodyA.Index];
			const BodyId B = Map[R.BodyB.Index];
			if (A == BodyId::Invalid() || B == BodyId::Invalid())
			{
				continue;
			}
			const InterfaceDesc Desc{A, B, R.NormalAToB, R.CentreM, R.AxisU, R.AxisV, R.HalfExtentUM, R.HalfExtentVM, R.Profile == Spec.Bed ? Bed : Perpend};
			const Result<InterfaceId> Added = Out.Model.AddInterface(Desc);
			if (!Added.Ok())
			{
				return Fail(Wall.Line, "the wall " + Quoted(Wall.Name) + ": " + ModelErrorText(Added.Error));
			}
		}
		return true;
	}

	bool CheckOverlaps()
	{
		for (std::size_t J = 0; J < Boxes.size(); ++J)
		{
			for (std::size_t I = 0; I < J; ++I)
			{
				if (Boxes[I].Wall >= 0 && Boxes[I].Wall == Boxes[J].Wall)
				{
					continue;
				}
				const geometry::Box& A = Boxes[I].Shape;
				const geometry::Box& B = Boxes[J].Shape;
				bool bOverlap = true;
				const double CA[3] = {A.CentreM.X, A.CentreM.Y, A.CentreM.Z};
				const double HA[3] = {A.HalfExtentsM.X, A.HalfExtentsM.Y, A.HalfExtentsM.Z};
				const double CB[3] = {B.CentreM.X, B.CentreM.Y, B.CentreM.Z};
				const double HB[3] = {B.HalfExtentsM.X, B.HalfExtentsM.Y, B.HalfExtentsM.Z};
				for (int Axis = 0; Axis < 3; ++Axis)
				{
					const double Overlap = std::min(CA[Axis] + HA[Axis], CB[Axis] + HB[Axis]) - std::max(CA[Axis] - HA[Axis], CB[Axis] - HB[Axis]);
					bOverlap = bOverlap && Overlap > ToleranceM;
				}
				if (bOverlap)
				{
					return Fail(Boxes[J].Line, "boxes " + Quoted(Out.BodyNames[I]) + " and " + Quoted(Out.BodyNames[J]) + " overlap");
				}
			}
		}
		return true;
	}

	const Rule* RuleFor(const std::vector<Rule>& Rules, std::size_t I, std::size_t J, bool bBed) const
	{
		const auto PlaneOk = [&](const Rule& R) { return R.Plane == 0 || (R.Plane == 1) == bBed; };
		const std::string& NameI = Out.BodyNames[I];
		const std::string& NameJ = Out.BodyNames[J];
		for (const Rule& R : Rules)
		{
			if (R.bBetween && PlaneOk(R) &&
				((Matches(R.Pair[0], NameI) && Matches(R.Pair[1], NameJ)) || (Matches(R.Pair[0], NameJ) && Matches(R.Pair[1], NameI))))
			{
				return &R;
			}
		}
		const std::string& MaterialI = Out.BodyMaterials[I];
		const std::string& MaterialJ = Out.BodyMaterials[J];
		for (const Rule& R : Rules)
		{
			if (!R.bBetween && PlaneOk(R) && ((R.Pair[0] == MaterialI && R.Pair[1] == MaterialJ) || (R.Pair[0] == MaterialJ && R.Pair[1] == MaterialI)))
			{
				return &R;
			}
		}
		return nullptr;
	}

	bool Join(const std::vector<Rule>& Rules)
	{
		for (std::size_t I = 0; I < Boxes.size(); ++I)
		{
			for (std::size_t J = I + 1; J < Boxes.size(); ++J)
			{
				const PlacedBox& A = Boxes[I];
				const PlacedBox& B = Boxes[J];
				if ((A.Wall >= 0 && A.Wall == B.Wall) || (B.Wall >= 0 && B.OnSlot == static_cast<int>(I)) || (A.Wall >= 0 && A.OnSlot == static_cast<int>(J)))
				{
					continue;
				}
				std::optional<InterfaceDesc> Desc = geometry::FaceContact(Out.BodyIds[I], A.Shape, Out.BodyIds[J], B.Shape, ProfileId::Invalid(), ToleranceM);
				if (!Desc)
				{
					continue;
				}
				const Rule* R = RuleFor(Rules, I, J, std::abs(Desc->NormalAToB.Z) > 0.5);
				if (R == nullptr)
				{
					return Fail(B.Line, "no joint rule for " + Out.BodyNames[I] + " / " + Out.BodyNames[J] + " (" + Out.BodyMaterials[I] + " / " + Out.BodyMaterials[J] + ")");
				}
				if (R->Kind == RowKind::Joint)
				{
					Desc->Profile = JointProfile(R->Row);
				}
				else
				{
					const double AreaM2 = 4.0 * Desc->HalfExtentUM * Desc->HalfExtentVM;
					std::array<char, sizeof(double)> Bits{};
					std::memcpy(Bits.data(), &AreaM2, sizeof(double));
					const std::string Key = R->Row + "#" + std::to_string(R->Count) + "#" + std::string(Bits.data(), Bits.size());
					Desc->Profile = SharedProfile(Key, R->Row, FastenerProfile(*Rows.FindFastener(R->Row), R->Count, AreaM2));
				}
				const Result<InterfaceId> Added = Out.Model.AddInterface(*Desc);
				if (!Added.Ok())
				{
					return Fail(B.Line, "joint " + Out.BodyNames[I] + " / " + Out.BodyNames[J] + ": " + ModelErrorText(Added.Error));
				}
			}
		}
		return true;
	}

	bool Expand(const JsonValue& Pattern, std::vector<std::string>& Into)
	{
		if (Pattern.Type != JsonType::String)
		{
			return Fail(Pattern.Line, "expected a name or pattern");
		}
		const PatternExpansion Expanded = ExpandPattern(Pattern.Text, Out.BodyNames);
		if (!Expanded.Ok())
		{
			return Fail(Pattern.Line, Expanded.Error);
		}
		for (const std::string& Name : Expanded.Names)
		{
			if (std::find(Into.begin(), Into.end(), Name) == Into.end())
			{
				Into.push_back(Name);
			}
		}
		return true;
	}

	bool ReadSteps(const JsonValue& Root)
	{
		const JsonValue* Array = nullptr;
		if (!Typed(Root, "steps", JsonType::Array, false, Array))
		{
			return false;
		}
		if (Array == nullptr)
		{
			Out.Steps.push_back(ScenarioStep{});
			return true;
		}
		for (const JsonValue& Item : Array->Items)
		{
			ScenarioStep Step;
			if (Item.Type == JsonType::String)
			{
				if (Item.Text != "cascade" && Item.Text != "judge")
				{
					return Fail(Item.Line, "unknown step " + Quoted(Item.Text));
				}
				Step.Kind = Item.Text == "cascade" ? ScenarioStepKind::Cascade : ScenarioStepKind::Judge;
				bCascadeSeen = bCascadeSeen || Step.Kind == ScenarioStepKind::Cascade;
			}
			else if (Item.Type != JsonType::Object || Item.Keys.size() != 1)
			{
				return Fail(Item.Line, "a step is a name or an object with exactly one key");
			}
			else if (!ReadStepObject(Item, Step))
			{
				return false;
			}
			Out.Steps.push_back(std::move(Step));
		}
		return true;
	}

	bool ReadStepObject(const JsonValue& Item, ScenarioStep& Step)
	{
		const std::string& Verb = Item.Keys[0];
		const JsonValue& Value = Item.Items[0];
		if (Verb == "remove")
		{
			Step.Kind = ScenarioStepKind::Remove;
			if (Value.Type != JsonType::Array)
			{
				return Fail(Value.Line, "remove must hold names");
			}
			for (const JsonValue& Pattern : Value.Items)
			{
				if (!Expand(Pattern, Step.Names))
				{
					return false;
				}
			}
			if (!bCascadeSeen)
			{
				RemovedBeforeAnyCascade.insert(Step.Names.begin(), Step.Names.end());
			}
			return true;
		}
		if (Verb == "load")
		{
			Step.Kind = ScenarioStepKind::Load;
			std::string Body;
			if (Value.Type != JsonType::Object)
			{
				return Fail(Value.Line, "load must be an object");
			}
			if (!OnlyKeys(Value, {"body", "force_n"}) || !Text(Value, "body", true, Body) || !Vector(Value, "force_n", true, Step.ForceN))
			{
				return false;
			}
			if (!Out.Model.IsKnown(Out.Find(Body)))
			{
				return Fail(Value.Line, "no body matches " + Quoted(Body));
			}
			Step.Names = {Body};
			return true;
		}
		if (Verb == "lateral_g")
		{
			Step.Kind = ScenarioStepKind::Lateral;
			if (Value.Type != JsonType::Array || Value.Items.size() != 2 || Value.Items[0].Type != JsonType::Number || Value.Items[1].Type != JsonType::Number)
			{
				return Fail(Value.Line, "lateral_g must hold 2 numbers");
			}
			Step.LateralX = Value.Items[0].Number;
			Step.LateralY = Value.Items[1].Number;
			return true;
		}
		return Fail(Item.Line, "unknown step " + Quoted(Verb));
	}

	bool ExpandBodies(const JsonValue& Pattern, std::vector<std::string>& Into)
	{
		if (!Expand(Pattern, Into))
		{
			return false;
		}
		for (const std::string& Name : Into)
		{
			if (RemovedBeforeAnyCascade.count(Name) != 0)
			{
				return Fail(Pattern.Line, Quoted(Name) + " is removed before any cascade runs, so it cannot be expected");
			}
		}
		return true;
	}

	bool ReadExpect(const JsonValue& Root)
	{
		const JsonValue* Object = nullptr;
		if (!Typed(Root, "expect", JsonType::Object, false, Object) || Object == nullptr)
		{
			return Object == nullptr && Error.Message.empty();
		}
		ScenarioExpect Expect;
		if (!OnlyKeys(*Object, {"state", "released", "either", "passes", "cracked"}) || !Text(*Object, "state", false, Expect.State))
		{
			return false;
		}
		static constexpr std::array<std::string_view, 6> States = {"Stands", "Overstressed", "Collapses", "Undecided", "InvalidInput", "PassCapReached"};
		if (std::find(States.begin(), States.end(), Expect.State) == States.end())
		{
			return Fail(Object->Line, "unknown state " + Quoted(Expect.State));
		}
		const JsonValue* Released = nullptr;
		const JsonValue* Either = nullptr;
		const JsonValue* Passes = nullptr;
		const JsonValue* Cracked = nullptr;
		if (!Typed(*Object, "released", JsonType::Array, false, Released) || !Typed(*Object, "either", JsonType::Array, false, Either) ||
			!Typed(*Object, "passes", JsonType::Number, false, Passes) || !Typed(*Object, "cracked", JsonType::Array, false, Cracked))
		{
			return false;
		}
		if (Released != nullptr && !ReadReleased(*Released, Expect))
		{
			return false;
		}
		if (Either != nullptr)
		{
			for (const JsonValue& Pattern : Either->Items)
			{
				if (!ExpandBodies(Pattern, Expect.Either))
				{
					return false;
				}
			}
		}
		for (const std::vector<std::string>& Cluster : Expect.Released)
		{
			for (const std::string& Name : Cluster)
			{
				if (std::find(Expect.Either.begin(), Expect.Either.end(), Name) != Expect.Either.end())
				{
					return Fail(Object->Line, Quoted(Name) + " is listed under both released and either");
				}
			}
		}
		if (Passes != nullptr)
		{
			int Count = 0;
			if (!WholeNumber(*Passes, "passes", 0.0, Count))
			{
				return false;
			}
			Expect.Passes = Count;
		}
		if (Cracked != nullptr)
		{
			std::vector<std::string> Joints;
			for (const JsonValue& Joint : Cracked->Items)
			{
				if (Joint.Type != JsonType::String || std::count(Joint.Text.begin(), Joint.Text.end(), '/') != 1 || !PatternIsValid(Joint.Text))
				{
					return Fail(Joint.Line, "a cracked joint is written '<a>/<b>'");
				}
				Joints.push_back(Joint.Text);
			}
			Expect.Cracked = std::move(Joints);
		}
		Out.Expect = std::move(Expect);
		return true;
	}

	bool ReadReleased(const JsonValue& Released, ScenarioExpect& Expect)
	{
		const bool bNested = !Released.Items.empty() && Released.Items[0].Type == JsonType::Array;
		Expect.bPartition = bNested;
		if (!bNested)
		{
			Expect.Released.emplace_back();
		}
		for (const JsonValue& Item : Released.Items)
		{
			if ((Item.Type == JsonType::Array) != bNested)
			{
				return Fail(Item.Line, "released is a list of names or a list of lists, not both");
			}
			if (!bNested)
			{
				if (!ExpandBodies(Item, Expect.Released.back()))
				{
					return false;
				}
				continue;
			}
			std::vector<std::string> Cluster;
			for (const JsonValue& Pattern : Item.Items)
			{
				if (!ExpandBodies(Pattern, Cluster))
				{
					return false;
				}
			}
			Expect.Released.push_back(std::move(Cluster));
		}
		return true;
	}

	const Catalogue& Rows;
	Scenario& Out;
	FileError& Error;
	std::map<std::string, double> LocalDensities;
	std::map<std::string, StrengthProfile> LocalProfiles;
	std::map<std::string, ProfileId> SharedProfiles;
	std::vector<PlacedBox> Boxes;
	std::set<std::string> RemovedBeforeAnyCascade;
	bool bCascadeSeen = false;
};

std::string ClusterText(const std::vector<std::string>& Cluster)
{
	std::string Out = "{";
	for (std::size_t I = 0; I < Cluster.size(); ++I)
	{
		Out += (I == 0 ? "" : ", ") + Cluster[I];
	}
	return Out + "}";
}

std::string PartitionText(const std::vector<std::vector<std::string>>& Clusters)
{
	std::string Out;
	for (const std::vector<std::string>& Cluster : Clusters)
	{
		Out += (Out.empty() ? "" : " ") + ClusterText(Cluster);
	}
	return Out.empty() ? "{}" : Out;
}

std::string Joined(const std::vector<std::string>& Names)
{
	std::string Out;
	for (const std::string& Name : Names)
	{
		Out += (Out.empty() ? "" : ", ") + Name;
	}
	return Out;
}

std::vector<std::vector<std::string>> CanonicalPartition(const std::vector<std::vector<std::string>>& Clusters, const std::vector<std::string>& Either)
{
	std::vector<std::vector<std::string>> Out;
	for (const std::vector<std::string>& Cluster : Clusters)
	{
		std::vector<std::string> Kept;
		for (const std::string& Name : Cluster)
		{
			if (std::find(Either.begin(), Either.end(), Name) == Either.end())
			{
				Kept.push_back(Name);
			}
		}
		std::sort(Kept.begin(), Kept.end());
		if (!Kept.empty())
		{
			Out.push_back(std::move(Kept));
		}
	}
	std::sort(Out.begin(), Out.end());
	return Out;
}

std::set<std::string> Union(const std::vector<std::vector<std::string>>& Clusters)
{
	std::set<std::string> Out;
	for (const std::vector<std::string>& Cluster : Clusters)
	{
		Out.insert(Cluster.begin(), Cluster.end());
	}
	return Out;
}

std::vector<std::string> NamesOf(const Scenario& Value, const std::vector<BodyId>& Ids)
{
	std::vector<std::string> Out;
	for (const BodyId Id : Ids)
	{
		Out.push_back(Id.Index < Value.BodyNames.size() ? Value.BodyNames[Id.Index] : "#" + std::to_string(Id.Index));
	}
	return Out;
}

std::vector<BodyId> IdsOf(const Scenario& Value, const std::vector<std::string>& Names)
{
	std::vector<BodyId> Out;
	for (const std::string& Name : Names)
	{
		Out.push_back(Value.Find(Name));
	}
	return Out;
}

void RecordCascade(const Scenario& Value, const CascadeVerdict& Verdict, ScenarioStepRecord& Record, ScenarioOutcome& Outcome)
{
	for (const ReleasedCluster& Cluster : Verdict.Clusters)
	{
		Record.Released.push_back(NamesOf(Value, Cluster.Bodies));
		Outcome.Clusters.push_back(Record.Released.back());
	}
	for (const FailedInterface& Failure : Verdict.Failures)
	{
		if (Failure.bSevered)
		{
			continue;
		}
		const InterfaceRecord& R = Value.Model.Interface(Failure.Id);
		const std::string Joint = NamesOf(Value, {R.BodyA})[0] + "/" + NamesOf(Value, {R.BodyB})[0];
		if (std::find(Outcome.Cracked.begin(), Outcome.Cracked.end(), Joint) == Outcome.Cracked.end())
		{
			Outcome.Cracked.push_back(Joint);
		}
	}
	Record.State = CascadeStateText(Verdict.State);
	Record.Passes = Verdict.Passes;
	Outcome.State = Record.State;
	Outcome.Passes += Verdict.Passes;
}

bool RunStep(Scenario& Value, const ScenarioStep& Step, int Number, const ICascade& Cascade, const IStructuralAuthority& Authority, ScenarioOutcome& Outcome)
{
	ScenarioStepRecord Record;
	AuthorityOptions Options;
	Options.Standing.Gravity = Outcome.Gravity;
	Options.Certificate.Gravity = Outcome.Gravity;
	switch (Step.Kind)
	{
	case ScenarioStepKind::Cascade:
	{
		Record.Kind = "cascade";
		CascadeOptions CascadeSettings;
		CascadeSettings.Authority = Options;
		Outcome.Verdicts.push_back(Cascade.Run(Value.Model, Outcome.Loads, CascadeSettings, nullptr));
		RecordCascade(Value, Outcome.Verdicts.back(), Record, Outcome);
		break;
	}
	case ScenarioStepKind::Judge:
	{
		Record.Kind = "judge";
		Record.State = VerdictText(Authority.Judge(Value.Model, Outcome.Loads, Options, nullptr).Worst);
		Outcome.State = Record.State;
		break;
	}
	case ScenarioStepKind::Remove:
		Record.Kind = "remove";
		for (const std::string& Name : Step.Names)
		{
			if (!Value.Model.RemoveBody(Value.Find(Name)))
			{
				Outcome.RunError = "step " + std::to_string(Number) + ": cannot remove " + Quoted(Name) + ", it is already gone";
				Outcome.Steps.push_back(std::move(Record));
				return false;
			}
			Record.Removed.push_back(Name);
			Outcome.Removed.push_back(Name);
		}
		break;
	case ScenarioStepKind::Load:
	{
		Record.Kind = "load";
		const BodyId Id = Value.Find(Step.Names[0]);
		if (!Value.Model.IsLive(Id))
		{
			Outcome.RunError = "step " + std::to_string(Number) + ": cannot load " + Quoted(Step.Names[0]) + ", it is already gone";
			Outcome.Steps.push_back(std::move(Record));
			return false;
		}
		Outcome.Loads.push_back(BodyLoad{Id, Step.ForceN, Value.Model.Body(Id).CentroidM});
		break;
	}
	case ScenarioStepKind::Lateral:
		Record.Kind = "lateral_g";
		Outcome.Gravity = Vec3{Step.LateralX * GravityMPerS2, Step.LateralY * GravityMPerS2, -GravityMPerS2};
		break;
	}
	Outcome.Steps.push_back(std::move(Record));
	return true;
}
}

BodyId Scenario::Find(std::string_view Name) const
{
	for (std::size_t Slot = 0; Slot < BodyNames.size() && Slot < BodyIds.size(); ++Slot)
	{
		if (BodyNames[Slot] == Name)
		{
			return BodyIds[Slot];
		}
	}
	return BodyId::Invalid();
}

ScenarioLoadResult LoadScenario(std::string_view Json, const Catalogue& Rows)
{
	ScenarioLoadResult Result;
	JsonParser Parser(Json);
	JsonValue Root;
	if (!Parser.ParseDocument(Root))
	{
		Result.Error = Parser.Error;
		return Result;
	}
	if (!Loader(Rows, Result.Value, Result.Error).Load(Root))
	{
		Result.Value = Scenario{};
		if (Result.Error.Message.empty())
		{
			Result.Error = {Root.Line, "the scenario could not be loaded"};
		}
	}
	return Result;
}

ScenarioOutcome RunScenario(Scenario& Value, const ICascade& Cascade, const IStructuralAuthority& Authority)
{
	ScenarioOutcome Outcome;
	Outcome.Names = Value.BodyNames;
	{
		Scenario Drawn = Value;
		ScenarioOutcome Scratch;
		for (std::size_t Index = 0; Index < Drawn.Steps.size(); ++Index)
		{
			const ScenarioStep& Step = Drawn.Steps[Index];
			if (Step.Kind == ScenarioStepKind::Cascade || Step.Kind == ScenarioStepKind::Judge)
			{
				continue;
			}
			if (!RunStep(Drawn, Step, static_cast<int>(Index) + 1, Cascade, Authority, Scratch))
			{
				break;
			}
		}
		AuthorityOptions Options;
		Options.Standing.Gravity = Scratch.Gravity;
		Options.Certificate.Gravity = Scratch.Gravity;
		Outcome.Before = Authority.Judge(Drawn.Model, Scratch.Loads, Options, nullptr).Worst;
	}
	const auto Start = std::chrono::steady_clock::now();
	for (std::size_t Index = 0; Index < Value.Steps.size(); ++Index)
	{
		if (!RunStep(Value, Value.Steps[Index], static_cast<int>(Index) + 1, Cascade, Authority, Outcome))
		{
			break;
		}
	}
	Outcome.ElapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
	return Outcome;
}

ScenarioOutcome RunScenario(Scenario& Value)
{
	const std::unique_ptr<IStructuralAuthority> Authority = MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate());
	const std::unique_ptr<ICascade> Cascade = MakeCascade(MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate()));
	return RunScenario(Value, *Cascade, *Authority);
}

ExpectationReport CompareExpectation(const ScenarioExpect& Expect, const ScenarioOutcome& Outcome)
{
	ExpectationReport Report;
	if (!Outcome.RunError.empty())
	{
		Report.Lines.push_back("run error: " + Outcome.RunError);
	}
	if (Expect.State != Outcome.State)
	{
		Report.Lines.push_back("state: expected " + Expect.State + ", engine " + Outcome.State);
	}
	const auto IsEither = [&](const std::string& Name) { return std::find(Expect.Either.begin(), Expect.Either.end(), Name) != Expect.Either.end(); };
	const std::set<std::string> Engine = Union(Outcome.Clusters);
	const std::set<std::string> Expected = Union(Expect.Released);
	std::vector<std::string> Unexpected;
	std::vector<std::string> Standing;
	for (const std::string& Name : Engine)
	{
		if (Expected.count(Name) == 0 && !IsEither(Name))
		{
			Unexpected.push_back(Name);
		}
	}
	for (const std::string& Name : Expected)
	{
		if (Engine.count(Name) == 0 && !IsEither(Name))
		{
			Standing.push_back(Name);
		}
	}
	if (!Unexpected.empty())
	{
		Report.Lines.push_back("released, not expected: " + Joined(Unexpected));
	}
	if (!Standing.empty())
	{
		Report.Lines.push_back("expected to fall, still standing: " + Joined(Standing));
	}
	if (Expect.bPartition)
	{
		const std::vector<std::vector<std::string>> Want = CanonicalPartition(Expect.Released, Expect.Either);
		const std::vector<std::vector<std::string>> Got = CanonicalPartition(Outcome.Clusters, Expect.Either);
		if (Want != Got)
		{
			Report.Lines.push_back("expected " + PartitionText(Want) + "; engine " + PartitionText(Got));
		}
	}
	if (Expect.Passes && *Expect.Passes != Outcome.Passes)
	{
		Report.Lines.push_back("passes: expected " + std::to_string(*Expect.Passes) + ", engine " + std::to_string(Outcome.Passes));
	}
	if (Expect.Cracked)
	{
		std::vector<std::string> Extra;
		std::vector<std::string> Missing;
		for (const std::string& Joint : Outcome.Cracked)
		{
			if (std::none_of(Expect.Cracked->begin(), Expect.Cracked->end(), [&](const std::string& P) { return JointMatches(P, Joint); }))
			{
				Extra.push_back(Joint);
			}
		}
		for (const std::string& Pattern : *Expect.Cracked)
		{
			if (std::none_of(Outcome.Cracked.begin(), Outcome.Cracked.end(), [&](const std::string& J) { return JointMatches(Pattern, J); }))
			{
				Missing.push_back(Pattern);
			}
		}
		if (!Extra.empty())
		{
			Report.Lines.push_back("cracked, not expected: " + Joined(Extra));
		}
		if (!Missing.empty())
		{
			Report.Lines.push_back("expected to crack, not cracked: " + Joined(Missing));
		}
	}
	Report.bMet = Report.Lines.empty();
	return Report;
}

PatternExpansion ExpandPattern(std::string_view Pattern, std::span<const std::string> Names)
{
	PatternExpansion Expansion;
	if (!PatternIsValid(Pattern))
	{
		Expansion.Error = "bad range in " + Quoted(Pattern) + ": write [a-b] with a <= b";
		return Expansion;
	}
	for (const std::string& Name : Names)
	{
		if (Matches(Pattern, Name))
		{
			Expansion.Names.push_back(Name);
		}
	}
	if (Expansion.Names.empty())
	{
		Expansion.Error = "no body matches " + Quoted(Pattern);
	}
	return Expansion;
}

ViewerScene ViewerSceneOf(const Scenario& Value, const ScenarioOutcome& Outcome, std::string_view Slug)
{
	ViewerScene Scene;
	Scene.Slug = std::string(Slug);
	Scene.Title = Value.Title;
	Scene.Summary = Value.Summary;
	Scene.Model = Value.Model;
	for (std::size_t Slot = 0; Slot < Value.BodyNames.size(); ++Slot)
	{
		Scene.Boxes.push_back(ViewerBox{Value.BodyIds[Slot], Value.BodyNames[Slot], Value.BodyMaterials[Slot], Value.BodyBoxes[Slot].CentreM,
			Value.BodyBoxes[Slot].HalfExtentsM, {1.0, 0.0, 0.0, 0.0}, Value.BodyGrounded[Slot], Value.BodyMassesKg[Slot]});
	}
	Scene.RemovedBeforeRun = IdsOf(Value, Outcome.Removed);
	Scene.Gravity = Outcome.Gravity;
	Scene.Loads = Outcome.Loads;
	Scene.Before = Outcome.Before;
	Scene.After = Outcome.Verdicts;
	if (Value.Expect && !CompareExpectation(*Value.Expect, Outcome).bMet)
	{
		ViewerExpected Expected;
		Expected.State = Value.Expect->State;
		if (Value.Expect->bPartition)
		{
			for (const std::vector<std::string>& Cluster : Value.Expect->Released)
			{
				Expected.Clusters.push_back(IdsOf(Value, Cluster));
			}
		}
		else
		{
			const std::set<std::string> All = Union(Value.Expect->Released);
			if (!All.empty())
			{
				Expected.Clusters.push_back(IdsOf(Value, std::vector<std::string>(All.begin(), All.end())));
			}
		}
		Expected.Either = IdsOf(Value, Value.Expect->Either);
		Scene.Expected = std::move(Expected);
	}
	return Scene;
}

std::string WriteViewerJson(const Scenario& Value, const ScenarioOutcome& Outcome, std::string_view Slug)
{
	return WriteViewerJson(ViewerSceneOf(Value, Outcome, Slug));
}
}
