#include "BitwiseTestSupport.h"
#include "CascadeTestSupport.h"
#include "ParallelTestSupport.h"
#include "SlowInDebug.h"

#include "loadpath/ParallelFor.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <latch>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr int ReaderThreads = 8;
constexpr int SharedPoolWorkers = 4;
constexpr int Repeats = 200;
constexpr int LateralRepeats = 15;
constexpr double LateralLambda = 0.5;

Structure MakeWall(const StrengthProfile& Joint)
{
	Structure Model;
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(Model, Joint);
	Spec.Perpend = Spec.Bed;
	Spec.Courses = 4;
	Spec.BricksPerCourse = 4;
	REQUIRE(geometry::BuildRunningBondWall(Model, Spec).bComplete);
	return Model;
}

struct ReaderOutcome
{
	ComponentPartition Partition;
	AuthorityResult Verdict;
	AuthorityResult Pooled;
};

bool AnyCollapses(const AuthorityResult& Verdict)
{
	return std::any_of(Verdict.Components.begin(), Verdict.Components.end(), [](const ComponentReport& C) { return C.Verdict == ComponentVerdict::Collapses; });
}

void CheckReadersAgreeWithSerial(const StrengthProfile& Joint, const AuthorityOptions& Options, int RepeatCount, bool bExpectCollapse)
{
	const Structure Template = MakeWall(Joint);
	const std::unique_ptr<IStructuralAuthority> Authority = MakeAuthority();
	const ThreadPoolFor Pool(SharedPoolWorkers);
	const std::unique_ptr<IStructuralAuthority> PooledAuthority = MakeElasticAuthority(MakeRbsmStandingAuthority(&Pool));

	Structure SerialModel = Template;
	const ComponentPartition SerialPartition = SerialModel.Components();
	const AuthorityResult SerialVerdict = Authority->Judge(SerialModel, {}, Options, nullptr);
	INFO("serial: " << DescribeAuthority(SerialVerdict));
	REQUIRE(SerialVerdict.Worst != ComponentVerdict::InvalidInput);
	REQUIRE(AnyCollapses(SerialVerdict) == bExpectCollapse);

	int PartitionMismatches = 0;
	int VerdictMismatches = 0;
	int PooledMismatches = 0;
	for (int Repeat = 0; Repeat < RepeatCount; ++Repeat)
	{
		const Structure Shared = Template;
		std::array<std::optional<ReaderOutcome>, ReaderThreads> Outcomes;
		std::latch Start(ReaderThreads);
		std::vector<std::thread> Readers;
		Readers.reserve(ReaderThreads);
		for (int T = 0; T < ReaderThreads; ++T)
		{
			Readers.emplace_back([&, T] {
				Start.arrive_and_wait();
				AuthorityResult Pooled = PooledAuthority->Judge(Shared, {}, Options, nullptr);
				ComponentPartition Partition = Shared.Components();
				AuthorityResult Verdict = Authority->Judge(Shared, {}, Options, nullptr);
				Outcomes[T] = ReaderOutcome{std::move(Partition), std::move(Verdict), std::move(Pooled)};
			});
		}
		for (std::thread& Reader : Readers)
		{
			Reader.join();
		}
		for (const std::optional<ReaderOutcome>& Outcome : Outcomes)
		{
			REQUIRE(Outcome.has_value());
			PartitionMismatches += bitwise::Same(Outcome->Partition, SerialPartition) ? 0 : 1;
			VerdictMismatches += bitwise::Same(Outcome->Verdict, SerialVerdict) ? 0 : 1;
			PooledMismatches += SameParallel(Outcome->Pooled, SerialVerdict) ? 0 : 1;
		}
	}
	INFO("readers " << RepeatCount * ReaderThreads << ", partition mismatches " << PartitionMismatches << ", verdict mismatches " << VerdictMismatches << ", shared " << SharedPoolWorkers << "-worker pool mismatches " << PooledMismatches << ", pool For calls " << Pool.Calls << ", indices run " << Pool.Ran.load());
	CHECK(PartitionMismatches == 0);
	CHECK(VerdictMismatches == 0);
	CHECK(PooledMismatches == 0);
	CHECK_MESSAGE(Pool.Calls >= static_cast<std::size_t>(RepeatCount * ReaderThreads), "every pooled judge went through the shared hook");
}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Concurrency.ConcurrentReadersOfOneStructureAgreeWithASerialRun")
{
	CheckReadersAgreeWithSerial(BondedJoint(2.0e5, 3.0e5, 0.7), AuthorityOptions{}, Repeats, false);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Concurrency.ConcurrentReadersOfAMechanismAgreeWithASerialRun")
{
	CheckReadersAgreeWithSerial(DryJoint(0.7), AuthorityUnderGravity(Vec3{0.0, LateralLambda * GravityMPerS2, -GravityMPerS2}), LateralRepeats, true);
}
