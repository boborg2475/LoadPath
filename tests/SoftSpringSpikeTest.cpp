#include "EndToEndSupport.h"
#include "WarehouseTestSupport.h"

#include "loadpath/Viewer.h"

#include "doctest.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;
using namespace loadpath::test::e2e;

namespace
{
std::string SpikeOutDir()
{
#if defined(_MSC_VER)
	char* Buffer = nullptr;
	std::size_t Length = 0;
	if (_dupenv_s(&Buffer, &Length, "LOADPATH_SPIKE_OUT") != 0 || Buffer == nullptr)
	{
		return {};
	}
	std::string Value(Buffer);
	std::free(Buffer);
	return Value;
#else
	const char* Value = std::getenv("LOADPATH_SPIKE_OUT");
	return Value == nullptr ? std::string{} : std::string(Value);
#endif
}

void WritePostRemoved(const std::string& Slug, const std::string& Title, const geometry::WarehouseSpec& Spec, bool bThin)
{
	const std::string Dir = SpikeOutDir();
	if (Dir.empty())
	{
		MESSAGE("LOADPATH_SPIKE_OUT not set; nothing written");
		return;
	}
	ShedScene H = BuildShed(Spec);
	const AuthorityResult First = ElasticAuthority()->Judge(H.S.Model, {}, AuthorityOptions{}, nullptr);
	RemoveNamed(H.S, "post1");
	static const DebugSettings Recording{true};
	CascadeOptions Options;
	Options.Authority.Standing.Debug = &Recording;
	const auto Start = std::chrono::steady_clock::now();
	CascadeVerdict V = MakeCascade(ElasticAuthority())->Run(H.S.Model, {}, Options, &First);
	const double Ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
	for (CascadePassRecord& Pass : V.PassRecords)
	{
		if (!bThin)
		{
			continue;
		}
		Pass.Readings.clear();
		for (ComponentReport& C : Pass.Components)
		{
			if (C.End == SolveEnd::Converged)
			{
				C.Passes.clear();
			}
		}
	}
	ViewerScene Scene;
	Scene.Slug = Slug;
	Scene.Title = Title;
	Scene.Summary = "post1 removed after a warm first judge; warehouse only: per-pass readings and the inner iterations of converged components are dropped to keep the file small.";
	Scene.Model = H.S.Model;
	for (const SceneBox& B : H.S.Boxes)
	{
		Scene.Boxes.push_back(ViewerBox{B.Id, B.Name, B.Material, B.CentreM, B.HalfExtentsM, B.Rotation, B.bGrounded, B.MassKg});
	}
	for (const std::string& Name : H.S.RemovedBeforeRun)
	{
		Scene.RemovedBeforeRun.push_back(SceneBody(H.S, Name));
	}
	Scene.After = {std::move(V)};
	Scene.ElapsedMs = Ms;
	std::ofstream Out(Dir + "/" + Slug + ".json", std::ios::binary);
	Out << WriteViewerJson(Scene);
	MESSAGE(Slug << " written, recorded cascade " << Ms << " ms");
}
}

TEST_CASE("Measure.Spike.SoftSpring.ShedPostRemovedViz")
{
	WritePostRemoved("shed-post-removed", "Shed, post1 removed", ShedSpec(), false);
}

#if defined(NDEBUG)
TEST_CASE("Measure.Spike.SoftSpring.WarehousePostRemovedViz")
{
	WritePostRemoved("warehouse-post-removed", "Warehouse, post1 removed", geometry::WarehouseSpec{}, true);
}
#endif
