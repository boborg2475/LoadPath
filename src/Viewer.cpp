#include "loadpath/Viewer.h"

#include "StateNames.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string_view>
#include <system_error>

namespace loadpath
{
namespace
{
constexpr std::uint32_t FirstGeneration = 1;

class JsonWriter
{
public:
	explicit JsonWriter(std::string& InOut)
		: Out(InOut)
	{
	}

	void Number(double Value)
	{
		if (!std::isfinite(Value))
		{
			Out += "null";
			return;
		}
		std::array<char, 32> Buffer{};
		const std::to_chars_result Written = std::to_chars(Buffer.data(), Buffer.data() + Buffer.size(), Value);
		Out.append(Buffer.data(), Written.ptr);
	}

	void String(std::string_view Text)
	{
		Out += '"';
		for (const char C : Text)
		{
			if (C == '"' || C == '\\')
			{
				Out += '\\';
				Out += C;
			}
			else if (static_cast<unsigned char>(C) < 0x20u)
			{
				char Escape[8];
				std::snprintf(Escape, sizeof Escape, "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(C)));
				Out += Escape;
			}
			else
			{
				Out += C;
			}
		}
		Out += '"';
	}

	void Separate()
	{
		if (!bFirst)
		{
			Out += ',';
		}
		bFirst = false;
	}

	void Key(std::string_view Name)
	{
		Separate();
		String(Name);
		Out += ':';
		bFirst = true;
	}

	void Open(char C)
	{
		Separate();
		Out += C;
		bFirst = true;
	}

	void Close(char C)
	{
		Out += C;
		bFirst = false;
	}

	void Item(double Value)
	{
		Separate();
		Number(Value);
	}

	void FieldNumber(std::string_view Name, double Value)
	{
		Key(Name);
		Item(Value);
	}

	void FieldInteger(std::string_view Name, std::uint64_t Value)
	{
		Key(Name);
		Separate();
		Out += std::to_string(Value);
	}

	void FieldString(std::string_view Name, std::string_view Value)
	{
		Key(Name);
		Separate();
		String(Value);
	}

	void FieldBool(std::string_view Name, bool bValue)
	{
		Key(Name);
		Separate();
		Out += bValue ? "true" : "false";
	}

	void FieldNull(std::string_view Name)
	{
		Key(Name);
		Separate();
		Out += "null";
	}

	void FieldVec(std::string_view Name, const Vec3& V)
	{
		Key(Name);
		Open('[');
		Item(V.X);
		Item(V.Y);
		Item(V.Z);
		Close(']');
	}

	template <typename IdType>
	void FieldIds(std::string_view Name, const std::vector<IdType>& Ids)
	{
		Key(Name);
		Open('[');
		for (const IdType& Id : Ids)
		{
			Separate();
			Out += std::to_string(Id.Index);
		}
		Close(']');
	}

private:
	std::string& Out;
	bool bFirst = true;
};

std::vector<std::vector<std::uint32_t>> Partition(const std::vector<std::vector<BodyId>>& Clusters, const std::vector<BodyId>& Either)
{
	std::vector<std::vector<std::uint32_t>> Out;
	for (const std::vector<BodyId>& Cluster : Clusters)
	{
		std::vector<std::uint32_t> Slots;
		for (const BodyId Id : Cluster)
		{
			if (std::find(Either.begin(), Either.end(), Id) == Either.end())
			{
				Slots.push_back(Id.Index);
			}
		}
		std::sort(Slots.begin(), Slots.end());
		if (!Slots.empty())
		{
			Out.push_back(std::move(Slots));
		}
	}
	std::sort(Out.begin(), Out.end());
	return Out;
}

bool ExpectedMatchesEngine(const ViewerScene& Scene)
{
	const ViewerExpected& Expected = *Scene.Expected;
	if (Scene.After.empty() || Expected.State != detail::CascadeStateText(Scene.After.back().State))
	{
		return false;
	}
	std::vector<std::vector<BodyId>> Engine;
	for (const CascadeVerdict& Verdict : Scene.After)
	{
		for (const ReleasedCluster& Cluster : Verdict.Clusters)
		{
			Engine.push_back(Cluster.Bodies);
		}
	}
	return Partition(Engine, Expected.Either) == Partition(Expected.Clusters, Expected.Either);
}

void WriteBodies(JsonWriter& W, const ViewerScene& Scene)
{
	W.Key("bodies");
	W.Open('[');
	for (const ViewerBox& Box : Scene.Boxes)
	{
		W.Open('{');
		W.FieldInteger("id", Box.Id.Index);
		W.FieldString("name", Box.Name);
		W.FieldVec("c", Box.CentreM);
		W.FieldVec("h", Box.HalfExtentsM);
		W.Key("q");
		W.Open('[');
		for (const double Q : Box.Rotation)
		{
			W.Item(Q);
		}
		W.Close(']');
		W.FieldBool("grounded", Box.bGrounded);
		W.FieldNumber("mass_kg", Box.MassKg);
		W.FieldString("material", Box.Material);
		W.Close('}');
	}
	W.Close(']');
}

void WriteInterfaces(JsonWriter& W, const Structure& Model)
{
	W.Key("interfaces");
	W.Open('[');
	for (std::uint32_t Slot = 0; Model.IsKnown(InterfaceId{Slot, FirstGeneration}); ++Slot)
	{
		const InterfaceRecord& R = Model.Interface(InterfaceId{Slot, FirstGeneration});
		W.Open('{');
		W.FieldInteger("id", Slot);
		W.FieldInteger("a", R.BodyA.Index);
		W.FieldInteger("b", R.BodyB.Index);
		W.FieldVec("p", R.CentreM);
		W.FieldVec("n", R.NormalAToB);
		W.FieldVec("u", R.AxisU);
		W.FieldNumber("hu", R.HalfExtentUM);
		W.FieldNumber("hv", R.HalfExtentVM);
		W.FieldInteger("profile", R.Profile.Index);
		W.FieldString("state_after", detail::InterfaceStateText(R.State));
		W.Close('}');
	}
	W.Close(']');
}

Vec3 BoxCentre(const ViewerScene& Scene, BodyId Id)
{
	for (const ViewerBox& Box : Scene.Boxes)
	{
		if (Box.Id == Id)
		{
			return Box.CentreM;
		}
	}
	return Vec3{};
}

void WriteCluster(JsonWriter& W, const ViewerScene& Scene, const ReleasedCluster& Cluster)
{
	W.Open('{');
	W.FieldInteger("pass", Cluster.Pass);
	W.FieldIds("bodies", Cluster.Bodies);
	W.FieldIds("severed", Cluster.Severed);
	W.FieldInteger("motion_body", Cluster.Motion.Body.Index);
	W.Key("motion");
	W.Open('{');
	const Vec3 Reference = BoxCentre(Scene, Cluster.Motion.Body);
	const Vec3& T = Cluster.Motion.TranslationM;
	for (const BodyId Id : Cluster.Bodies)
	{
		const Vec3 Centre = BoxCentre(Scene, Id);
		const Vec3 Spin = Cross(Cluster.Motion.RotationRad, Vec3{Centre.X - Reference.X, Centre.Y - Reference.Y, Centre.Z - Reference.Z});
		W.Key(std::to_string(Id.Index));
		W.Open('{');
		W.FieldVec("t", Vec3{T.X + Spin.X, T.Y + Spin.Y, T.Z + Spin.Z});
		W.FieldVec("r", Cluster.Motion.RotationRad);
		W.Close('}');
	}
	W.Close('}');
	W.Close('}');
}

void WriteAfter(JsonWriter& W, const ViewerScene& Scene)
{
	W.Key("after");
	W.Open('{');
	if (Scene.After.empty())
	{
		W.FieldNull("state");
	}
	else
	{
		W.FieldString("state", detail::CascadeStateText(Scene.After.back().State));
	}
	std::uint64_t Passes = 0;
	for (const CascadeVerdict& Verdict : Scene.After)
	{
		Passes += static_cast<std::uint64_t>(Verdict.Passes);
	}
	W.FieldInteger("passes", Passes);
	W.Key("clusters");
	W.Open('[');
	std::vector<BodyId> Released;
	for (const CascadeVerdict& Verdict : Scene.After)
	{
		for (const ReleasedCluster& Cluster : Verdict.Clusters)
		{
			WriteCluster(W, Scene, Cluster);
			Released.insert(Released.end(), Cluster.Bodies.begin(), Cluster.Bodies.end());
		}
	}
	W.Close(']');
	W.Key("failures");
	W.Open('[');
	for (const CascadeVerdict& Verdict : Scene.After)
	{
		for (const FailedInterface& F : Verdict.Failures)
		{
			W.Open('{');
			W.FieldInteger("interface", F.Id.Index);
			W.FieldInteger("pass", F.Pass);
			W.FieldString("axis", detail::AxisText(F.Axis));
			W.FieldNumber("ratio", F.Ratio);
			W.FieldBool("severed", F.bSevered);
			W.Close('}');
		}
	}
	W.Close(']');
	W.FieldIds("removed", Released);
	if (Scene.ElapsedMs)
	{
		W.FieldNumber("elapsed_ms", *Scene.ElapsedMs);
	}
	W.Close('}');
}

void WriteExpected(JsonWriter& W, const ViewerExpected& Expected)
{
	W.Key("expected");
	W.Open('{');
	W.FieldString("state", Expected.State);
	W.Key("clusters");
	W.Open('[');
	for (const std::vector<BodyId>& Cluster : Expected.Clusters)
	{
		W.Open('{');
		W.FieldIds("bodies", Cluster);
		W.Close('}');
	}
	W.Close(']');
	W.FieldIds("either", Expected.Either);
	W.Close('}');
}
}

std::string WriteViewerJson(const ViewerScene& Scene)
{
	std::string Out;
	JsonWriter W(Out);
	W.Open('{');
	W.FieldString("slug", Scene.Slug);
	W.FieldString("title", Scene.Title);
	if (!Scene.Test.empty())
	{
		W.FieldString("test", Scene.Test);
	}
	W.FieldString("summary", Scene.Summary);
	W.FieldString("units", "m");
	W.FieldVec("gravity", Scene.Gravity);
	if (Scene.Gravity.X != 0.0 || Scene.Gravity.Y != 0.0)
	{
		W.Key("lateral");
		W.Open('[');
		W.Item(Scene.Gravity.X / GravityMPerS2);
		W.Item(Scene.Gravity.Y / GravityMPerS2);
		W.Close(']');
	}
	else
	{
		W.FieldNull("lateral");
	}
	W.Key("loads");
	W.Open('[');
	for (const BodyLoad& Load : Scene.Loads)
	{
		W.Open('{');
		W.FieldInteger("body", Load.Body.Index);
		W.FieldVec("force_n", Load.ForceN);
		W.FieldVec("point", Load.PointM);
		W.Close('}');
	}
	W.Close(']');
	WriteBodies(W, Scene);
	WriteInterfaces(W, Scene.Model);
	W.FieldIds("removed_before_run", Scene.RemovedBeforeRun);
	W.Key("before");
	W.Open('{');
	if (Scene.Before)
	{
		W.FieldString("verdict", detail::VerdictText(*Scene.Before));
	}
	else
	{
		W.FieldNull("verdict");
	}
	W.Close('}');
	WriteAfter(W, Scene);
	if (Scene.Expected && !ExpectedMatchesEngine(Scene))
	{
		WriteExpected(W, *Scene.Expected);
	}
	W.Close('}');
	Out += '\n';
	return Out;
}
}
