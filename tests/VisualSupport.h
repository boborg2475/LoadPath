#pragma once

#include "JsonReader.h"

#include "loadpath/Cli.h"
#include "loadpath/Viewer.h"

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace loadpath::test
{
// An in-memory file store that counts every call, so a test can assert that nothing touched it.
class MemoryFiles final : public IFileStore
{
public:
	std::map<std::string, std::string> Files;
	mutable std::vector<std::string> ReadPaths;
	mutable int Reads = 0;
	mutable int Lists = 0;
	int Writes = 0;

	std::optional<std::string> Read(std::string_view Path) const override
	{
		++Reads;
		ReadPaths.push_back(std::string(Path));
		const auto Found = Files.find(std::string(Path));
		if (Found == Files.end())
		{
			return std::nullopt;
		}
		return Found->second;
	}

	bool Write(std::string_view Path, std::string_view Contents) override
	{
		++Writes;
		Files[std::string(Path)] = std::string(Contents);
		return true;
	}

	std::vector<std::string> List(std::string_view Dir) const override
	{
		++Lists;
		const std::string Prefix = std::string(Dir) + "/";
		std::vector<std::string> Names;
		for (const auto& [Path, Contents] : Files)
		{
			if (Path.starts_with(Prefix) && Path.find('/', Prefix.size()) == std::string::npos)
			{
				Names.push_back(Path.substr(Prefix.size()));
			}
		}
		return Names;
	}

	int Calls() const { return Reads + Lists + Writes; }
};

class DiskFiles final : public IFileStore
{
public:
	std::optional<std::string> Read(std::string_view Path) const override
	{
		std::ifstream File{std::string(Path), std::ios::binary};
		if (!File)
		{
			return std::nullopt;
		}
		return std::string(std::istreambuf_iterator<char>(File), std::istreambuf_iterator<char>());
	}

	bool Write(std::string_view Path, std::string_view Contents) override
	{
		std::ofstream File{std::string(Path), std::ios::binary};
		File.write(Contents.data(), static_cast<std::streamsize>(Contents.size()));
		return static_cast<bool>(File);
	}

	std::vector<std::string> List(std::string_view Dir) const override
	{
		std::vector<std::string> Names;
		std::error_code Error;
		std::filesystem::directory_iterator It(std::filesystem::path(std::string(Dir)), Error);
		const std::filesystem::directory_iterator End;
		while (!Error && It != End)
		{
			if (It->is_regular_file(Error))
			{
				Names.push_back(It->path().filename().string());
			}
			It.increment(Error);
		}
		std::sort(Names.begin(), Names.end());
		return Names;
	}
};

// Unset (nullptr) and empty both mean no recording.
inline std::optional<std::string> VisualsDirFrom(const char* Value)
{
	if (Value == nullptr || *Value == '\0')
	{
		return std::nullopt;
	}
	return std::string(Value);
}

inline std::optional<std::string> VisualsDirFromEnvironment()
{
#if defined(_MSC_VER)
	char* Buffer = nullptr;
	std::size_t Length = 0;
	if (_dupenv_s(&Buffer, &Length, "LOADPATH_VISUALS") != 0 || Buffer == nullptr)
	{
		return std::nullopt;
	}
	const std::optional<std::string> Dir = VisualsDirFrom(Buffer);
	std::free(Buffer);
	return Dir;
#else
	return VisualsDirFrom(std::getenv("LOADPATH_VISUALS"));
#endif
}

inline bool IsSlug(std::string_view Slug)
{
	if (Slug.empty())
	{
		return false;
	}
	for (const char C : Slug)
	{
		if (!((C >= 'a' && C <= 'z') || (C >= '0' && C <= '9') || C == '-'))
		{
			return false;
		}
	}
	return true;
}

inline std::string VisualPath(std::string_view Dir, std::string_view Slug) { return std::string(Dir) + "/" + std::string(Slug) + ".json"; }

inline std::string RecordedTestName(std::string_view Json)
{
	detail::JsonValue Root;
	detail::JsonParser Parser(Json);
	if (!Parser.ParseDocument(Root))
	{
		return {};
	}
	const detail::JsonValue* Test = detail::Find(Root, "test");
	return Test != nullptr && Test->Type == detail::JsonType::String ? Test->Text : std::string();
}

inline std::array<Vec3, 3> BoxAxes(const ViewerBox& Box)
{
	const double W = Box.Rotation[0], X = Box.Rotation[1], Y = Box.Rotation[2], Z = Box.Rotation[3];
	return {Vec3{1 - 2 * (Y * Y + Z * Z), 2 * (X * Y + W * Z), 2 * (X * Z - W * Y)}, Vec3{2 * (X * Y - W * Z), 1 - 2 * (X * X + Z * Z), 2 * (Y * Z + W * X)},
		Vec3{2 * (X * Z + W * Y), 2 * (Y * Z - W * X), 1 - 2 * (X * X + Y * Y)}};
}

// Separating-axis test on the 15 box axes; true when every axis overlaps by more than ToleranceM.
inline bool BoxesOverlap(const ViewerBox& A, const ViewerBox& B, double ToleranceM)
{
	const std::array<Vec3, 3> Ua = BoxAxes(A), Ub = BoxAxes(B);
	const std::array<double, 3> Ha{A.HalfExtentsM.X, A.HalfExtentsM.Y, A.HalfExtentsM.Z}, Hb{B.HalfExtentsM.X, B.HalfExtentsM.Y, B.HalfExtentsM.Z};
	const auto Dot = [](const Vec3& P, const Vec3& Q) { return P.X * Q.X + P.Y * Q.Y + P.Z * Q.Z; };
	const auto Cross = [](const Vec3& P, const Vec3& Q) { return Vec3{P.Y * Q.Z - P.Z * Q.Y, P.Z * Q.X - P.X * Q.Z, P.X * Q.Y - P.Y * Q.X}; };
	const Vec3 D{B.CentreM.X - A.CentreM.X, B.CentreM.Y - A.CentreM.Y, B.CentreM.Z - A.CentreM.Z};
	std::vector<Vec3> Axes(Ua.begin(), Ua.end());
	Axes.insert(Axes.end(), Ub.begin(), Ub.end());
	for (const Vec3& P : Ua)
	{
		for (const Vec3& Q : Ub)
		{
			Axes.push_back(Cross(P, Q));
		}
	}
	for (const Vec3& Axis : Axes)
	{
		const double Length = std::sqrt(Dot(Axis, Axis));
		if (Length < 1e-9)
		{
			continue;
		}
		const Vec3 N{Axis.X / Length, Axis.Y / Length, Axis.Z / Length};
		double Reach = 0.0;
		for (int K = 0; K < 3; ++K)
		{
			Reach += Ha[K] * std::abs(Dot(Ua[K], N)) + Hb[K] * std::abs(Dot(Ub[K], N));
		}
		if (Reach - std::abs(Dot(D, N)) <= ToleranceM)
		{
			return false;
		}
	}
	return true;
}

inline std::string BuriedBox(const ViewerScene& Scene)
{
	for (const ViewerBox& Free : Scene.Boxes)
	{
		for (const ViewerBox& Ground : Scene.Boxes)
		{
			if (!Free.bGrounded && Ground.bGrounded && BoxesOverlap(Free, Ground, 1e-9))
			{
				return "free box " + Free.Name + " overlaps grounded box " + Ground.Name;
			}
		}
	}
	return {};
}

// Returns an error instead of failing, so its own tests can call it; empty means written, or nothing to do without a directory.
inline std::string RecordVisualTo(const std::optional<std::string>& Dir, IFileStore& Files, std::string_view TestName, ViewerScene Scene)
{
	if (!Dir)
	{
		return {};
	}
	if (!IsSlug(Scene.Slug))
	{
		return "visual slug '" + Scene.Slug + "' from " + std::string(TestName) + " is not lowercase letters, digits and dashes";
	}
	if (const std::string Buried = BuriedBox(Scene); !Buried.empty())
	{
		return "visual " + Scene.Slug + " from " + std::string(TestName) + ": " + Buried;
	}
	const std::string Path = VisualPath(*Dir, Scene.Slug);
	if (const std::optional<std::string> Existing = Files.Read(Path))
	{
		const std::string Previous = RecordedTestName(*Existing);
		if (Previous != TestName)
		{
			return "visual slug '" + Scene.Slug + "' is written by both " + Previous + " and " + std::string(TestName);
		}
	}
	Scene.Test = std::string(TestName);
	if (!Files.Write(Path, WriteViewerJson(Scene)))
	{
		return "cannot write " + Path + " for " + std::string(TestName);
	}
	return {};
}

inline std::string CurrentTestName()
{
	const doctest::detail::TestCase* Test = doctest::getContextOptions()->currentTest;
	return Test != nullptr ? std::string(Test->m_name) : std::string();
}

inline void RecordVisual(std::string_view TestName, const ViewerScene& Scene)
{
	const std::optional<std::string> Dir = VisualsDirFromEnvironment();
	if (!Dir)
	{
		return;
	}
	DiskFiles Files;
	const std::string Error = RecordVisualTo(Dir, Files, TestName, Scene);
	if (!Error.empty())
	{
		FAIL_CHECK(Error);
	}
}
}
