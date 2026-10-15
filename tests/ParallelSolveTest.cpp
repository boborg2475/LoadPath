#include "BitwiseTestSupport.h"
#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "EventRegionTestSupport.h"
#include "ParallelTestSupport.h"
#include "ScaleTestSupport.h"
#include "SlowInDebug.h"
#include "StaticsTestSupport.h"
#include "WarehouseTestSupport.h"

#include "loadpath/CollapseCertificate.h"
#include "loadpath/ParallelFor.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/Warehouse.h"

#include "doctest.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr std::uint64_t DrawMasterSeed = 0x9a7a11e1ull;
constexpr int DrawCount = 6;
constexpr std::uint64_t ShuffleMasterSeed = 20261015ull;
constexpr int ShufflesPerFixture = 3;
constexpr int PoolRepeats = 3;
constexpr int PoolSizes[] = {1, 2, 4, 8};

struct Fixture
{
	std::string Name;
	Structure Model;
	std::vector<BodyLoad> Loads;
	StandingOptions Options;
	std::vector<BodyId> Only;
};

struct Mixed
{
	Structure Model;
	std::vector<BodyLoad> Loads;
	std::vector<BodyId> FirstStack;
	BodyId Pushed;
	std::vector<BodyId> SecondStack;
	std::vector<BodyId> FloatingPair;
	BodyId Loose;
};

Mixed MakeMixed()
{
	ThreeComponents Three = MakeThreeComponents();
	Mixed M;
	M.Model = Three.Model;
	M.Loads = Three.Loads;
	M.FirstStack = Three.Parts[0];
	M.Pushed = Three.Parts[1][0];
	M.SecondStack = Three.Parts[2];
	const ProfileId Dry = AddProfileOk(M.Model, DryJoint(0.7));
	const Box LowerShape = BrickShape(4.0, 1);
	const Box UpperShape = BrickShape(4.0, 2);
	const BodyId Lower = AddBox(M.Model, LowerShape);
	const BodyId Upper = AddBox(M.Model, UpperShape);
	AddInterfaceOk(M.Model, *StackedFaces(Lower, LowerShape, Upper, UpperShape, Dry));
	M.FloatingPair = {Lower, Upper};
	M.Loose = AddBox(M.Model, BrickShape(5.0, 3));
	const Box DoomedShape = BrickShape(6.0, 0);
	const BodyId Doomed = AddBox(M.Model, DoomedShape);
	M.Loads.push_back(BodyLoad{Doomed, Vec3{1.0, 0.0, 0.0}, DoomedShape.CentreM});
	REQUIRE(M.Model.RemoveBody(Doomed));
	return M;
}

Fixture MixedFixture(const std::string& Name, const Mixed& M, std::vector<BodyId> Only)
{
	return Fixture{Name, M.Model, M.Loads, StandingOptions{}, std::move(Only)};
}

std::vector<Fixture> MixedFixtures()
{
	std::vector<Fixture> Out;
	const Mixed M = MakeMixed();
	Out.push_back(MixedFixture("mixed: two stacks, a pushed brick, a floating pair, a loose brick, a stale load", M, {}));
	Out.push_back(MixedFixture("mixed, Only the pushed brick and the floating pair", M, {M.Pushed, M.FloatingPair[0]}));
	return Out;
}

Fixture ShedFixture()
{
	ScaleSubject S = WarehouseSubject("shed", ShedSpec(), true);
	REQUIRE(S.Model.LiveBodyCount() == S.ExpectedBodies);
	return Fixture{"shed", S.Model, {}, StandingOptions{}, {}};
}

std::vector<Fixture> ShedAndDrawFixtures()
{
	std::vector<Fixture> Out;
	Out.push_back(ShedFixture());
	for (int K = 0; K < DrawCount; ++K)
	{
		const std::uint64_t Seed = DrawMasterSeed + static_cast<std::uint64_t>(K);
		MultiDraw D = MakeMultiDraw(Seed);
		std::ostringstream Name;
		Name << "draw seed " << Seed << ": " << D.Description;
		Out.push_back(Fixture{Name.str(), D.Model, D.Loads, D.Options.Standing, {}});
	}
	return Out;
}

std::size_t ProblemsSolved(const Structure& Model, std::span<const BodyId> Only)
{
	const ComponentPartition& Partition = Model.Components();
	if (Only.empty())
	{
		return Partition.FreeBodies.size() + 1;
	}
	std::set<int> Components;
	for (const BodyId Id : Only)
	{
		Components.insert(Partition.ComponentOfSlot[Id.Index]);
	}
	return Components.size() + 1;
}

std::string FirstDifference(const StandingResult& Got, const StandingResult& Want)
{
	std::ostringstream Out;
	if (Got.State != Want.State)
	{
		Out << "state " << Got.State << ", serial " << Want.State;
	}
	else if (Got.Readings.size() != Want.Readings.size())
	{
		Out << "readings " << Got.Readings.size() << ", serial " << Want.Readings.size();
	}
	else if (!bitwise::Same(Got.Readings, Want.Readings))
	{
		std::size_t K = 0;
		while (bitwise::Same(Got.Readings[K], Want.Readings[K]))
		{
			++K;
		}
		Out << "reading " << K << " (" << Got.Readings[K].Id << "): utilisation " << Got.Readings[K].Utilisation << ", serial " << Want.Readings[K].Utilisation;
	}
	else if (!bitwise::Same(Got.Mechanism, Want.Mechanism))
	{
		Out << "mechanism " << Got.Mechanism.size() << " motions, serial " << Want.Mechanism.size();
	}
	else if (!bitwise::Same(Got.UnconstrainedBodies, Want.UnconstrainedBodies))
	{
		Out << "unconstrained " << Got.UnconstrainedBodies.size() << ", serial " << Want.UnconstrainedBodies.size();
	}
	else if (!bitwise::Same(Got.Components, Want.Components))
	{
		Out << "components " << Got.Components.size() << ", serial " << Want.Components.size();
	}
	else if (Got.ActiveSetIterations != Want.ActiveSetIterations)
	{
		Out << "iterations " << Got.ActiveSetIterations << ", serial " << Want.ActiveSetIterations;
	}
	else if (!bitwise::Same(Got.InvalidInterface, Want.InvalidInterface))
	{
		Out << "invalid interface " << Got.InvalidInterface << ", serial " << Want.InvalidInterface;
	}
	else if (!bitwise::Same(Got.StaleLoads, Want.StaleLoads))
	{
		Out << "stale loads " << Got.StaleLoads.size() << ", serial " << Want.StaleLoads.size();
	}
	else
	{
		Out << "none";
	}
	return Out.str();
}

StandingResult SolveSerial(const Fixture& F)
{
	return MakeRbsmStandingAuthority()->Solve(F.Model, F.Loads, F.Options, F.Only);
}

StandingResult SolveThrough(const Fixture& F, const IParallelFor& Workers)
{
	return MakeRbsmStandingAuthority(&Workers)->Solve(F.Model, F.Loads, F.Options, F.Only);
}

void CheckOrder(const Fixture& F, const StandingResult& Serial, const std::string& OrderName, const OrderedFor& Hook)
{
	const std::size_t Expected = ProblemsSolved(F.Model, F.Only);
	const StandingResult Got = SolveThrough(F, Hook);
	INFO(OrderName << ": first difference " << FirstDifference(Got, Serial) << "; hook calls " << Hook.Calls << ", indices run " << Hook.Ran << ", problems " << Expected);
	CHECK_MESSAGE(Hook.Ran == Expected, "the hook ran problem 0 and every solved component");
	CHECK(SameParallel(Got, Serial));
}

void CheckEveryOrder(const Fixture& F, std::uint64_t ShuffleSeedBase)
{
	INFO(F.Name);
	const StandingResult Serial = SolveSerial(F);
	INFO("serial state " << Serial.State << ", components " << Serial.Components.size() << ", readings " << Serial.Readings.size());
	REQUIRE(Serial.State != StandingState::InvalidInput);
	CheckOrder(F, Serial, "reverse", ReverseFor());
	for (int K = 0; K < ShufflesPerFixture; ++K)
	{
		const std::uint64_t Seed = ShuffleSeedBase + static_cast<std::uint64_t>(K);
		std::ostringstream Name;
		Name << "shuffle seed " << Seed;
		CheckOrder(F, Serial, Name.str(), ShuffledFor(Seed));
	}
}

void CheckEveryPoolSize(const Fixture& F)
{
	INFO(F.Name);
	const StandingResult Serial = SolveSerial(F);
	INFO("serial state " << Serial.State << ", components " << Serial.Components.size() << ", readings " << Serial.Readings.size());
	REQUIRE(Serial.State != StandingState::InvalidInput);
	const std::size_t Expected = ProblemsSolved(F.Model, F.Only);
	for (const int Workers : PoolSizes)
	{
		const ThreadPoolFor Pool(Workers);
		int Mismatches = 0;
		std::string Difference = "none";
		for (int Repeat = 0; Repeat < PoolRepeats; ++Repeat)
		{
			const StandingResult Got = SolveThrough(F, Pool);
			if (!SameParallel(Got, Serial))
			{
				++Mismatches;
				Difference = FirstDifference(Got, Serial);
			}
		}
		INFO(Workers << " workers: " << Mismatches << " of " << PoolRepeats << " differ, last difference " << Difference << "; indices run " << Pool.Ran.load() << ", expected " << PoolRepeats * Expected);
		CHECK_MESSAGE(Pool.Ran.load() == PoolRepeats * Expected, "the pool ran problem 0 and every solved component on every solve");
		CHECK(Mismatches == 0);
	}
}

bool AnyConsulted(const AuthorityResult& R)
{
	return std::any_of(R.Components.begin(), R.Components.end(), [](const ComponentReport& C) { return C.bConsultedCertificate; });
}

bool AnyReused(const AuthorityResult& R)
{
	return std::any_of(R.Components.begin(), R.Components.end(), [](const ComponentReport& C) { return C.bReused; });
}

std::unique_ptr<IStructuralAuthority> SerialAuthority()
{
	return MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate());
}

std::unique_ptr<IStructuralAuthority> ParallelAuthority(const IParallelFor& Workers)
{
	return MakeCombinedAuthority(MakeRbsmStandingAuthority(&Workers), MakeLimitAnalysisCertificate());
}

StrengthProfile StiffJoint(double StiffnessPaPerM)
{
	StrengthProfile P = DryJoint(0.7);
	P.NormalStiffnessPaPerM = StiffnessPaPerM;
	P.TangentialStiffnessPaPerM = StiffnessPaPerM;
	return P;
}

struct InvalidFixture
{
	Fixture F;
	InterfaceId Expected = InterfaceId::Invalid();
};

InvalidFixture OverflowBetweenStacks()
{
	InvalidFixture Out;
	Structure& Model = Out.F.Model;
	const BodyId Ground = AddGroundPlane(Model);
	const ProfileId Dry = AddProfileOk(Model, DryJoint(0.7));
	AddDryStack(Model, Ground, Dry, 0.0);
	const ProfileId Stiff = AddProfileOk(Model, StiffJoint(1.0e305));
	const Box LowerShape = BoxOfDensity(Vec3{20.0, 0.0, 15.0}, Vec3{5.0, 5.0, 5.0}, 2000.0);
	const Box UpperShape = BoxOfDensity(Vec3{20.0, 0.0, 25.0}, Vec3{5.0, 5.0, 5.0}, 2000.0);
	const BodyId Lower = AddBox(Model, LowerShape);
	const BodyId Upper = AddBox(Model, UpperShape);
	AddInterfaceOk(Model, *StackedFaces(Lower, LowerShape, Upper, UpperShape, Stiff));
	AddDryStack(Model, Ground, Dry, 2.0);
	Out.F.Name = "a floating pair at stiffness 1e305 between two grounded stacks";
	return Out;
}

InvalidFixture ZeroStiffnessBeside()
{
	InvalidFixture Out;
	const Mixed M = MakeMixed();
	Out.F.Model = M.Model;
	Out.F.Loads = M.Loads;
	StrengthProfile NoNormal = DryJoint(0.7);
	NoNormal.NormalStiffnessPaPerM = 0.0;
	const ProfileId Bad = AddProfileOk(Out.F.Model, NoNormal);
	const BodyId Ground = AddGroundPlane(Out.F.Model);
	const Box Shape = BrickShape(7.0, 0);
	const BodyId Brick = AddBox(Out.F.Model, Shape);
	Out.Expected = AddInterfaceOk(Out.F.Model, OnGroundPlane(Ground, Brick, Shape, Bad));
	Out.F.Name = "the mixed fixture plus a brick on a zero normal stiffness bed";
	return Out;
}

InvalidFixture NonFiniteLoad()
{
	InvalidFixture Out;
	const Mixed M = MakeMixed();
	Out.F.Model = M.Model;
	Out.F.Loads = M.Loads;
	Out.F.Loads.push_back(BodyLoad{M.Pushed, Vec3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, Out.F.Model.Body(M.Pushed).CentroidM});
	Out.F.Name = "the mixed fixture plus a NaN load on the pushed brick";
	return Out;
}

std::string DescribeAuthorityDifference(const AuthorityResult& Got, const AuthorityResult& Want)
{
	std::ostringstream Out;
	Out << "worst " << VerdictName(Got.Worst) << " (serial " << VerdictName(Want.Worst) << "), components " << Got.Components.size() << " (serial " << Want.Components.size() << "), readings same " << bitwise::Same(Got.Readings, Want.Readings)
		<< ", components same " << bitwise::Same(Got.Components, Want.Components) << ", disagreements " << Got.Disagreements << " (serial " << Want.Disagreements << "), invalid " << Got.InvalidInterface << " (serial " << Want.InvalidInterface
		<< "), stale same " << bitwise::Same(Got.StaleLoads, Want.StaleLoads);
	return Out.str();
}

double MsSince(std::chrono::steady_clock::time_point Start) { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count(); }
}

TEST_CASE("Statics.Parallel.AnyExecutionOrderGivesTheSerialAnswer")
{
	const std::vector<Fixture> Fixtures = MixedFixtures();
	{
		const StandingResult Serial = SolveSerial(Fixtures[0]);
		INFO("mixed serial: state " << Serial.State << ", components " << Serial.Components.size() << ", stale loads " << Serial.StaleLoads.size());
		const auto Has = [&](StandingState State, bool bIterated) {
			return std::any_of(Serial.Components.begin(), Serial.Components.end(), [&](const StandingComponent& C) { return C.State == State && (C.ActiveSetIterations > 0) == bIterated; });
		};
		REQUIRE(Serial.Components.size() == 5u);
		REQUIRE(Has(StandingState::Stands, true));
		REQUIRE(Has(StandingState::Mechanism, true));
		REQUIRE(Has(StandingState::Mechanism, false));
		REQUIRE(Serial.StaleLoads.size() == 1u);
	}
	for (std::size_t K = 0; K < Fixtures.size(); ++K)
	{
		CheckEveryOrder(Fixtures[K], ShuffleMasterSeed + 100u * K);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Parallel.AnyExecutionOrderGivesTheSerialAnswerOnTheShedAndSeededDraws")
{
	const std::vector<Fixture> Fixtures = ShedAndDrawFixtures();
	REQUIRE(Fixtures[0].Model.Components().FreeBodies.size() == 5u);
	for (std::size_t K = 0; K < Fixtures.size(); ++K)
	{
		CheckEveryOrder(Fixtures[K], ShuffleMasterSeed + 1000u + 100u * K);
	}
}

TEST_CASE("Statics.Parallel.AnyWorkerCountGivesTheSerialAnswer")
{
	for (const Fixture& F : MixedFixtures())
	{
		CheckEveryPoolSize(F);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Parallel.AnyWorkerCountGivesTheSerialAnswerOnTheShedAndSeededDraws")
{
	for (const Fixture& F : ShedAndDrawFixtures())
	{
		CheckEveryPoolSize(F);
	}
}

TEST_CASE("Statics.Parallel.EachProblemIsHandedToTheHookOnceAsProblemZeroPlusEachSolvedComponent")
{
	const Mixed M = MakeMixed();
	const std::size_t Components = M.Model.Components().FreeBodies.size();
	REQUIRE(Components == 5u);
	struct OnlyCase
	{
		const char* Name;
		std::vector<BodyId> Only;
		std::size_t Expected;
	};
	const OnlyCase Cases[] = {
		OnlyCase{"every component", {}, Components + 1},
		OnlyCase{"Only the pushed brick and the lower brick of the floating pair", {M.Pushed, M.FloatingPair[0]}, 3u},
		OnlyCase{"Only both bricks of the first stack", {M.FirstStack[0], M.FirstStack[1]}, 2u},
		OnlyCase{"Only the loose brick", {M.Loose}, 2u},
	};
	for (const OnlyCase& Case : Cases)
	{
		INFO(std::string(Case.Name));
		const RecordingFor Hook;
		const std::unique_ptr<IStandingAuthority> Authority = MakeRbsmStandingAuthority(&Hook);
		const StandingResult Got = Authority->Solve(M.Model, M.Loads, StandingOptions{}, Case.Only);
		const StandingResult Serial = MakeRbsmStandingAuthority()->Solve(M.Model, M.Loads, StandingOptions{}, Case.Only);
		INFO("For calls " << Hook.Calls.size() << ", expected Count " << Case.Expected << " (problem 0 plus " << Case.Expected - 1 << " components); first difference " << FirstDifference(Got, Serial));
		CHECK(Serial.Components.size() == Case.Expected - 1);
		CHECK(SameParallel(Got, Serial));
		CHECK_MESSAGE(Hook.Calls.size() == 1u, "one For call per Solve");
		if (Hook.Calls.empty())
		{
			continue;
		}
		const ForCall& Call = Hook.Calls[0];
		INFO("Count " << Call.Count << ", indices run " << Call.Ran.size());
		CHECK_MESSAGE(Call.Count == Case.Expected, "Count is problem 0 plus each solved component");
		std::vector<std::size_t> Sorted = Call.Ran;
		std::sort(Sorted.begin(), Sorted.end());
		CHECK_MESSAGE(Sorted == AscendingOrder(Call.Count), "each index in [0, Count) runs exactly once");
		const StandingResult Again = Authority->Solve(M.Model, M.Loads, StandingOptions{}, Case.Only);
		CHECK(SameParallel(Again, Serial));
		CHECK_MESSAGE(Hook.Calls.size() == 2u, "a second Solve makes exactly one more For call");
	}
}

TEST_CASE("Statics.Parallel.TheCombinedAuthorityWithAParallelElasticAuthorityJudgesTheSame")
{
	const Mixed M = MakeMixed();
	Structure Model = M.Model;
	const ThreadPoolFor Pool(4);
	const OrderedFor Reverse = ReverseFor();
	const std::unique_ptr<IStructuralAuthority> Authorities[] = {SerialAuthority(), ParallelAuthority(Pool), ParallelAuthority(Reverse)};
	const std::string Names[] = {"serial", "pool of 4", "reverse order"};
	std::vector<AuthorityResult> Cold;
	for (const std::unique_ptr<IStructuralAuthority>& Authority : Authorities)
	{
		Cold.push_back(Authority->Judge(Model, M.Loads, AuthorityOptions{}, nullptr));
	}
	REQUIRE(Model.RemoveBody(M.SecondStack[1]));
	std::vector<AuthorityResult> Warm;
	for (std::size_t K = 0; K < Cold.size(); ++K)
	{
		Warm.push_back(Authorities[K]->Judge(Model, M.Loads, AuthorityOptions{}, &Cold[K]));
	}
	INFO("serial cold: " << DescribeAuthority(Cold[0]));
	INFO("serial warm: " << DescribeAuthority(Warm[0]));
	REQUIRE(AnyConsulted(Cold[0]));
	REQUIRE(Cold[0].StaleLoads.size() == 1u);
	REQUIRE(AnyReused(Warm[0]));
	for (std::size_t K = 1; K < Cold.size(); ++K)
	{
		INFO(Names[K] << " cold: " << DescribeAuthorityDifference(Cold[K], Cold[0]));
		INFO(Names[K] << " warm: " << DescribeAuthorityDifference(Warm[K], Warm[0]));
		CHECK(SameParallel(Cold[K], Cold[0]));
		CHECK(SameParallel(Warm[K], Warm[0]));
	}
	INFO("pool indices run " << Pool.Ran.load() << ", reverse indices run " << Reverse.Ran);
	CHECK_MESSAGE(Pool.Ran.load() > 0u, "the combined authority's elastic solves went through the pool");
	CHECK_MESSAGE(Reverse.Ran > 0u, "the combined authority's elastic solves went through the reverse hook");
}

TEST_CASE("Statics.Parallel.InvalidInputIsUnchanged")
{
	const InvalidFixture Fixtures[] = {OverflowBetweenStacks(), ZeroStiffnessBeside(), NonFiniteLoad()};
	const ThreadPoolFor Pool(4);
	const OrderedFor Reverse = ReverseFor();
	const IParallelFor* Hooks[] = {&Pool, &Reverse};
	const char* HookNames[] = {"pool of 4", "reverse order"};
	for (const InvalidFixture& I : Fixtures)
	{
		INFO(I.F.Name);
		const StandingResult Serial = SolveSerial(I.F);
		const AuthorityResult SerialJudged = SerialAuthority()->Judge(I.F.Model, I.F.Loads, AuthorityOptions{}, nullptr);
		INFO("serial: " << Serial.State << ", invalid " << Serial.InvalidInterface << ", judged " << VerdictName(SerialJudged.Worst));
		REQUIRE(Serial.State == StandingState::InvalidInput);
		REQUIRE(SerialJudged.Worst == ComponentVerdict::InvalidInput);
		CHECK(SameHandle(Serial.InvalidInterface, I.Expected));
		for (std::size_t H = 0; H < 2; ++H)
		{
			INFO(std::string(HookNames[H]));
			const StandingResult Got = SolveThrough(I.F, *Hooks[H]);
			INFO("standing: first difference " << FirstDifference(Got, Serial));
			CHECK(Got.State == StandingState::InvalidInput);
			CHECK(SameHandle(Got.InvalidInterface, Serial.InvalidInterface));
			CHECK(SameParallel(Got, Serial));
			const AuthorityResult Judged = ParallelAuthority(*Hooks[H])->Judge(I.F.Model, I.F.Loads, AuthorityOptions{}, nullptr);
			INFO("judged: " << DescribeAuthorityDifference(Judged, SerialJudged));
			CHECK(Judged.Worst == ComponentVerdict::InvalidInput);
			CHECK(SameHandle(Judged.InvalidInterface, SerialJudged.InvalidInterface));
			CHECK(SameParallel(Judged, SerialJudged));
		}
	}
}

#if defined(NDEBUG)
TEST_CASE("Measure.Parallel.WarehouseElasticSpeedup")
{
	const ScaleSubject S = WarehouseSubject("warehouse", geometry::WarehouseSpec{}, false);
	REQUIRE(S.Model.LiveBodyCount() == S.ExpectedBodies);
	const Fixture F{"warehouse", S.Model, {}, StandingOptions{}, {}};
	const std::size_t Expected = ProblemsSolved(F.Model, F.Only);
	StandingResult Serial;
	double SerialMs = std::numeric_limits<double>::infinity();
	for (int Run = 0; Run < 3; ++Run)
	{
		const std::chrono::steady_clock::time_point Start = std::chrono::steady_clock::now();
		Serial = SolveSerial(F);
		SerialMs = std::min(SerialMs, MsSince(Start));
	}
	MESSAGE("warehouse elastic: serial " << SerialMs << " ms, " << Serial.State << ", components " << Serial.Components.size() << ", problems " << Expected);
	for (const int Workers : {2, 4, 8})
	{
		const ThreadPoolFor Pool(Workers);
		double BestMs = std::numeric_limits<double>::infinity();
		int Mismatches = 0;
		std::string Difference = "none";
		for (int Run = 0; Run < 3; ++Run)
		{
			const std::chrono::steady_clock::time_point Start = std::chrono::steady_clock::now();
			const StandingResult Got = SolveThrough(F, Pool);
			BestMs = std::min(BestMs, MsSince(Start));
			if (!SameParallel(Got, Serial))
			{
				++Mismatches;
				Difference = FirstDifference(Got, Serial);
			}
		}
		MESSAGE("warehouse elastic: " << Workers << " workers " << BestMs << " ms, speedup " << SerialMs / BestMs << ", indices run " << Pool.Ran.load());
		INFO(Workers << " workers: " << Mismatches << " of 3 differ, last difference " << Difference);
		CHECK(Mismatches == 0);
		CHECK(Pool.Ran.load() == 3u * Expected);
	}
}
#endif
