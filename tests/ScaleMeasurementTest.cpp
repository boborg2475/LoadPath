#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "CertificateTestSupport.h"
#include "ScaleTestSupport.h"
#include "StaticsTestSupport.h"
#include "WarehouseTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/Catalogue.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/RunningBondWall.h"
#include "loadpath/geometry/Warehouse.h"

#include "doctest.h"

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
using Clock = std::chrono::steady_clock;

double MsSince(Clock::time_point Start) { return std::chrono::duration<double, std::milli>(Clock::now() - Start).count(); }

struct CascadeTiming
{
	double Ms = 0;
	int Passes = 0;
	std::string State = "n/a";
};

CascadeTiming TimeCascade(const Structure& Model, const std::vector<BodyId>& Remove)
{
	CascadeTiming Out;
	if (Remove.empty())
	{
		return Out;
	}
	Structure Copy = Model;
	for (const BodyId& Id : Remove)
	{
		CHECK(Copy.RemoveBody(Id));
	}
	const std::unique_ptr<ICascade> Cascade = MakeCascade(MakeAuthority());
	const Clock::time_point Start = Clock::now();
	const CascadeVerdict V = Cascade->Run(Copy, {}, CascadeOptions{}, nullptr);
	Out.Ms = MsSince(Start);
	Out.Passes = V.Passes;
	Out.State = CascadeStateName(V.State);
	return Out;
}

void Measure(const ScaleSubject& S)
{
	const Clock::time_point CaseStart = Clock::now();
	INFO(S.Name << ": live bodies " << S.Model.LiveBodyCount() << ", closed form " << S.ExpectedBodies);
	REQUIRE(S.Model.LiveBodyCount() == S.ExpectedBodies);
	if (S.Model.LiveBodyCount() != S.ExpectedBodies)
	{
		return;
	}
	Clock::time_point Start = Clock::now();
	const StandingResult Elastic = SolveWith(S.Model, StandingOptions{});
	const double ElasticMs = MsSince(Start);
	Start = Clock::now();
	const CertificateResult Certificate = DecideWith(S.Model, CertificateOptions{});
	const double CertificateMs = MsSince(Start);
	Start = Clock::now();
	const AuthorityResult First = Judge(S.Model, AuthorityOptions{});
	const double JudgeMs = MsSince(Start);
	const AuthorityResult Second = Judge(S.Model, AuthorityOptions{});
	const CascadeTiming Post = TimeCascade(S.Model, S.Posts.size() > 1 ? std::vector<BodyId>{S.Posts[1]} : std::vector<BodyId>{});
	const CascadeTiming Course = TimeCascade(S.Model, S.BaseCourse);
	MESSAGE(S.Name << ": pieces " << S.Model.LiveBodyCount() << ", joints " << S.Model.LiveInterfaceCount() << ", components " << First.Components.size() << ", elastic " << ElasticMs << " ms (" << Elastic.State << "), certificate "
		<< CertificateMs << " ms (" << Certificate.State << "), judge " << JudgeMs << " ms (" << std::string(VerdictName(First.Worst)) << "), cascade(post) " << Post.Ms << " ms + " << Post.Passes << " passes + " << Post.State << ", cascade(course) "
		<< Course.Ms << " ms + " << Course.Passes << " passes + " << Course.State);
	if (S.bMustStand)
	{
		CHECK(First.Worst == ComponentVerdict::Stands);
	}
	CHECK(First.Worst == Second.Worst);
	REQUIRE(First.Components.size() == Second.Components.size());
	for (std::size_t I = 0; I < First.Components.size() && I < Second.Components.size(); ++I)
	{
		INFO("component " << I);
		CHECK(First.Components[I].Verdict == Second.Components[I].Verdict);
		CHECK(SameIdList(First.Components[I].Bodies, Second.Components[I].Bodies));
	}
	MESSAGE(S.Name << ": total " << MsSince(CaseStart) << " ms");
}

void MeasureWarmPostRemoval(const ScaleSubject& S)
{
	INFO(S.Name << ": live bodies " << S.Model.LiveBodyCount() << ", closed form " << S.ExpectedBodies);
	REQUIRE(S.Model.LiveBodyCount() == S.ExpectedBodies);
	REQUIRE(S.Posts.size() > 1);
	const std::unique_ptr<IStructuralAuthority> Authority = MakeAuthority();
	Structure Copy = S.Model;
	Clock::time_point Start = Clock::now();
	const AuthorityResult First = Authority->Judge(Copy, {}, AuthorityOptions{}, nullptr);
	const double JudgeMs = MsSince(Start);
	REQUIRE(Copy.RemoveBody(S.Posts[1]));
	const std::unique_ptr<ICascade> Cascade = MakeCascade(MakeAuthority());
	Start = Clock::now();
	const CascadeVerdict V = Cascade->Run(Copy, {}, CascadeOptions{}, &First);
	const double CascadeMs = MsSince(Start);
	std::size_t Reused = 0;
	for (const ComponentReport& C : V.Components)
	{
		Reused += C.bReused ? 1u : 0u;
	}
	MESSAGE(S.Name << " warm: pieces " << S.Model.LiveBodyCount() << ", components " << V.Components.size() << " (first judge " << First.Components.size() << "), reused " << Reused << ", first judge " << JudgeMs << " ms (" << std::string(VerdictName(First.Worst)) << "), warm cascade(post) " << CascadeMs << " ms + " << V.Passes << " passes + " << std::string(CascadeStateName(V.State)));
	CHECK(V.State == CascadeState::Stands);
}
}

TEST_CASE("Measure.Scale.WallTenByEight")
{
	Measure(WallSubject("wall 10 x 8", 10, 8));
}

TEST_CASE("Measure.Scale.WallFortyByThirtyTwo")
{
	Measure(WallSubject("wall 40 x 32", 32, 40));
}

TEST_CASE("Measure.Scale.Shed")
{
	Measure(WarehouseSubject("shed", ShedSpec(), true));
}

TEST_CASE("Measure.Scale.ShedPostRemovedWarm")
{
	MeasureWarmPostRemoval(WarehouseSubject("shed", ShedSpec(), true));
}

TEST_CASE("Measure.Scale.HalfWarehouse")
{
	Measure(WarehouseSubject("half warehouse", HalfWarehouseSpec(), true));
}

#if defined(NDEBUG)
TEST_CASE("Measure.Scale.Warehouse")
{
	const geometry::WarehouseSpec Spec;
	CHECK(WarehouseBodies(Spec) == 3660u + 34u + 1u);
	Measure(WarehouseSubject("warehouse", Spec, false));
}

TEST_CASE("Measure.Scale.WarehousePostRemovedWarm")
{
	MeasureWarmPostRemoval(WarehouseSubject("warehouse", geometry::WarehouseSpec{}, false));
}
#endif
