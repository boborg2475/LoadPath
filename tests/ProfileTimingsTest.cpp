#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "ParallelTestSupport.h"
#include "ScaleTestSupport.h"
#include "StaticsTestSupport.h"
#include "WarehouseTestSupport.h"

#include "loadpath/ParallelFor.h"
#include "loadpath/PhaseTimings.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/RunningBondWall.h"
#include "loadpath/geometry/Warehouse.h"

#include "doctest.h"

#include <array>
#include <bit>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
#if defined(LOADPATH_PROFILE)
constexpr bool bProfiled = true;
#else
constexpr bool bProfiled = false;
#endif

constexpr double PhaseSumSlackRelative = 1e-12;
constexpr int GroundedStackBricks = 24;
constexpr double GroundedStackXM = 8.0;

enum class SolvePath : std::uint8_t
{
	Dense,
	Sparse,
	Shortcut
};

constexpr std::array<const char*, 7> FieldNames{"AssemblySeconds", "AnalysisSeconds", "FactorizationSeconds", "SolveSeconds", "CheckSeconds", "ReadoutSeconds", "TotalSeconds"};

std::array<double, 7> FieldsOf(const PhaseTimings& T) { return {T.AssemblySeconds, T.AnalysisSeconds, T.FactorizationSeconds, T.SolveSeconds, T.CheckSeconds, T.ReadoutSeconds, T.TotalSeconds}; }

double PhaseSum(const PhaseTimings& T) { return T.AssemblySeconds + T.AnalysisSeconds + T.FactorizationSeconds + T.SolveSeconds + T.CheckSeconds + T.ReadoutSeconds; }

std::string Describe(const PhaseTimings& T)
{
	std::ostringstream Out;
	const std::array<double, 7> Fields = FieldsOf(T);
	for (std::size_t I = 0; I < Fields.size(); ++I)
	{
		Out << (I == 0 ? "" : ", ") << FieldNames[I] << " " << Fields[I];
	}
	return Out.str();
}

void Accumulate(PhaseTimings& Sum, const PhaseTimings& T)
{
	Sum.AssemblySeconds += T.AssemblySeconds;
	Sum.AnalysisSeconds += T.AnalysisSeconds;
	Sum.FactorizationSeconds += T.FactorizationSeconds;
	Sum.SolveSeconds += T.SolveSeconds;
	Sum.CheckSeconds += T.CheckSeconds;
	Sum.ReadoutSeconds += T.ReadoutSeconds;
	Sum.TotalSeconds += T.TotalSeconds;
}

void CheckAllZero(const PhaseTimings& T, const std::string& What)
{
	const std::array<double, 7> Fields = FieldsOf(T);
	for (std::size_t I = 0; I < Fields.size(); ++I)
	{
		INFO(What << ": " << std::string(FieldNames[I]) << " " << Fields[I]);
		CHECK(Fields[I] == 0.0);
	}
}

void CheckSameBits(const PhaseTimings& Got, const PhaseTimings& Want, const std::string& What)
{
	const std::array<double, 7> G = FieldsOf(Got);
	const std::array<double, 7> W = FieldsOf(Want);
	for (std::size_t I = 0; I < G.size(); ++I)
	{
		INFO(What << ": " << std::string(FieldNames[I]) << " " << G[I] << ", sum over components " << W[I]);
		CHECK(std::bit_cast<std::uint64_t>(G[I]) == std::bit_cast<std::uint64_t>(W[I]));
	}
}

void CheckWithinTotal(const PhaseTimings& T, const std::string& What)
{
	INFO(What << ": " << Describe(T) << ", phase sum " << PhaseSum(T));
	for (const double Field : FieldsOf(T))
	{
		CHECK(std::isfinite(Field));
		CHECK(Field >= 0.0);
	}
	CHECK(PhaseSum(T) <= T.TotalSeconds * (1.0 + PhaseSumSlackRelative));
}

void CheckFilled(const PhaseTimings& T, SolvePath Path, const std::string& What)
{
	CheckWithinTotal(T, What);
	INFO(What << ": " << Describe(T));
	if (Path == SolvePath::Shortcut)
	{
		return;
	}
	CHECK(T.AssemblySeconds > 0.0);
	CHECK(T.FactorizationSeconds > 0.0);
	CHECK(T.SolveSeconds > 0.0);
	CHECK(T.ReadoutSeconds > 0.0);
	CHECK(T.TotalSeconds > 0.0);
	if (Path == SolvePath::Sparse)
	{
		CHECK(T.AnalysisSeconds > 0.0);
		CHECK(T.CheckSeconds > 0.0);
	}
	else
	{
		CHECK(T.AnalysisSeconds == 0.0);
		CHECK(T.CheckSeconds == 0.0);
	}
}

geometry::WallSpec SmallEc6Wall(Structure& Model, const MortarRows& Rows)
{
	geometry::WallSpec Spec;
	Spec.Courses = 5;
	Spec.BricksPerCourse = 4;
	Spec.Bed = AddProfileOk(Model, Rows.Bed);
	Spec.Perpend = AddProfileOk(Model, Rows.Perpend);
	return Spec;
}

struct TwoWallsAndAFloatingBrick
{
	Structure Model;
	BodyId Floating;
};

TwoWallsAndAFloatingBrick MakeTwoWallsAndAFloatingBrick()
{
	TwoWallsAndAFloatingBrick F;
	geometry::WallSpec Spec = SmallEc6Wall(F.Model, LoadMortarRows());
	const geometry::Wall First = geometry::BuildRunningBondWall(F.Model, Spec);
	REQUIRE(First.bComplete);
	Spec.Ground = First.Ground;
	Spec.OriginM = Vec3{2.0, 0.0, 0.0};
	const geometry::Wall Second = geometry::BuildRunningBondWall(F.Model, Spec);
	REQUIRE(Second.bComplete);
	F.Floating = AddBox(F.Model, BrickShape(5.0, 3));
	BodyId Below = AddBox(F.Model, BrickShape(GroundedStackXM, 0), true);
	for (int Course = 1; Course < GroundedStackBricks; ++Course)
	{
		const BodyId Above = AddBox(F.Model, BrickShape(GroundedStackXM, Course), true);
		AddInterfaceOk(F.Model, *StackedFaces(Below, BrickShape(GroundedStackXM, Course - 1), Above, BrickShape(GroundedStackXM, Course), Spec.Bed));
		Below = Above;
	}
	return F;
}

struct WallAndPushedBrick
{
	Structure Model;
	BodyId Brick;
	std::vector<BodyLoad> Loads;
};

WallAndPushedBrick MakeWallAndPushedBrick(bool bWithBrick)
{
	WallAndPushedBrick F;
	const geometry::Wall W = geometry::BuildRunningBondWall(F.Model, SmallEc6Wall(F.Model, LoadMortarRows()));
	REQUIRE(W.bComplete);
	if (!bWithBrick)
	{
		return F;
	}
	const Box Shape = BrickShape(2.0);
	const ProfileId Dry = AddProfileOk(F.Model, DryJoint(0.7));
	F.Brick = AddBox(F.Model, Shape);
	AddInterfaceOk(F.Model, OnGroundPlane(W.Ground, F.Brick, Shape, Dry));
	F.Loads.push_back(BodyLoad{F.Brick, Vec3{1.0 * BrickMassKg * GravityMPerS2, 0.0, 0.0}, Shape.CentreM});
	return F;
}

void CheckJudgeZero(const AuthorityResult& R, const std::string& What)
{
	CheckAllZero(R.ElasticTimings, What + " ElasticTimings");
	INFO(What << ": CertificateSeconds " << R.CertificateSeconds << ", JudgeSeconds " << R.JudgeSeconds);
	CHECK(R.CertificateSeconds == 0.0);
	CHECK(R.JudgeSeconds == 0.0);
}

void CheckJudgeCovers(const AuthorityResult& R, const std::string& What)
{
	INFO(What << ": " << Describe(R.ElasticTimings) << ", CertificateSeconds " << R.CertificateSeconds << ", JudgeSeconds " << R.JudgeSeconds);
	CHECK(std::isfinite(R.CertificateSeconds));
	CHECK(R.CertificateSeconds >= 0.0);
	CHECK(std::isfinite(R.JudgeSeconds));
	CHECK(R.JudgeSeconds >= R.ElasticTimings.TotalSeconds + R.CertificateSeconds);
}

class Fnv1a
{
public:
	void Bytes(std::uint64_t Value, int Count)
	{
		for (int I = 0; I < Count; ++I)
		{
			Hash ^= (Value >> (8 * I)) & 0xffu;
			Hash *= 1099511628211ull;
		}
	}
	void Word(std::uint64_t Value) { Bytes(Value, 8); }
	void Real(double Value) { Word(std::bit_cast<std::uint64_t>(Value)); }
	void Point(const Vec3& V)
	{
		Real(V.X);
		Real(V.Y);
		Real(V.Z);
	}
	template <class Handle>
	void Id(const Handle& H)
	{
		Word(H.Index);
		Word(H.Generation);
	}
	template <class Handle>
	void Ids(const std::vector<Handle>& Handles)
	{
		Word(Handles.size());
		for (const Handle& H : Handles)
		{
			Id(H);
		}
	}
	std::uint64_t Value() const { return Hash; }

private:
	std::uint64_t Hash = 14695981039346656037ull;
};

std::uint64_t ReadingsDigest(const AuthorityResult& R)
{
	Fnv1a H;
	H.Word(static_cast<std::uint64_t>(R.Worst));
	H.Word(static_cast<std::uint64_t>(R.Disagreements));
	H.Id(R.InvalidInterface);
	H.Word(R.Readings.size());
	for (const InterfaceReading& Reading : R.Readings)
	{
		H.Id(Reading.Id);
		H.Real(Reading.Utilisation);
		H.Word(static_cast<std::uint64_t>(Reading.Axis));
		H.Real(Reading.CompressionUtilisation);
		H.Real(Reading.TensionUtilisation);
		H.Real(Reading.ShearUtilisation);
		H.Real(Reading.NormalForceN);
		H.Point(Reading.ShearForceN);
		H.Point(Reading.MomentNm);
		for (const CornerReading& Corner : Reading.Corners)
		{
			H.Point(Corner.PointM);
			H.Real(Corner.NormalForceN);
			H.Point(Corner.ShearForceN);
			H.Word(Corner.bOpen ? 1u : 0u);
			H.Word(Corner.bSlipping ? 1u : 0u);
		}
	}
	H.Word(R.Components.size());
	for (const ComponentReport& C : R.Components)
	{
		H.Ids(C.Bodies);
		H.Word(static_cast<std::uint64_t>(C.Verdict));
		H.Word(static_cast<std::uint64_t>(C.Elastic));
		H.Word(static_cast<std::uint64_t>(C.Kinematic));
		H.Word(C.bConsultedCertificate ? 1u : 0u);
		H.Word(C.bDisagreed ? 1u : 0u);
		H.Real(C.LoadFactorLower);
		H.Real(C.LoadFactorUpper);
		H.Word(C.Mechanism.size());
		for (const MechanismMotion& M : C.Mechanism)
		{
			H.Id(M.Body);
			H.Point(M.TranslationM);
			H.Point(M.RotationRad);
		}
		H.Ids(C.Release);
		H.Ids(C.Hinging);
		H.Word(C.bReused ? 1u : 0u);
		H.Word(C.Loads.size());
		for (const BodyLoad& L : C.Loads)
		{
			H.Id(L.Body);
			H.Point(L.ForceN);
			H.Point(L.PointM);
		}
	}
	return H.Value();
}

std::string Hex(std::uint64_t Value)
{
	char Text[17] = {};
	std::snprintf(Text, sizeof(Text), "%016llx", static_cast<unsigned long long>(Value));
	return Text;
}

void CheckElasticTimings(const TwoWallsAndAFloatingBrick& F, const IParallelFor* Hook, const std::string& HookName)
{
	for (const SolvePath Path : {SolvePath::Dense, SolvePath::Sparse})
	{
		const std::string PathName = HookName + ", " + (Path == SolvePath::Dense ? "dense" : "sparse");
		StandingOptions Options;
		Options.DenseDofLimit = Path == SolvePath::Dense ? INT_MAX : 0;
		const StandingResult R = MakeRbsmStandingAuthority(Hook)->Solve(F.Model, {}, Options, {});
		INFO(PathName << " path, " << std::string(bProfiled ? "profiled" : "normal") << " build, state " << R.State);
		REQUIRE(R.Components.size() == 3u);
		CHECK(R.Readings.size() == F.Model.LiveInterfaces().size());
		PhaseTimings Sum;
		for (std::size_t K = 0; K < R.Components.size(); ++K)
		{
			const StandingComponent& C = R.Components[K];
			const bool bFloating = HasBody(C.Bodies, F.Floating);
			const std::string What = PathName + " component " + std::to_string(K) + (bFloating ? " (floating)" : " (wall)");
			INFO(What << ": state " << C.State << ", bodies " << C.Bodies.size());
			CHECK(C.State == (bFloating ? StandingState::Mechanism : StandingState::Stands));
			Accumulate(Sum, C.Timings);
			if (bProfiled)
			{
				CheckFilled(C.Timings, bFloating ? SolvePath::Shortcut : Path, What);
			}
			else
			{
				CheckAllZero(C.Timings, What);
			}
		}
		if (bProfiled)
		{
			CheckWithinTotal(R.Timings, PathName + " result");
			CheckSameBits(R.Timings, Sum, PathName + " result");
		}
		else
		{
			CheckAllZero(R.Timings, PathName + " result");
		}
	}
}
}

TEST_CASE("Profile.Timings.ElasticFillsThePhasesItRunsAndSumsItsComponents")
{
	const TwoWallsAndAFloatingBrick F = MakeTwoWallsAndAFloatingBrick();
	CheckElasticTimings(F, nullptr, "serial");
	const OrderedFor Reverse = ReverseFor();
	CheckElasticTimings(F, &Reverse, "reverse hook");
	const ThreadPoolFor Pool(4);
	CheckElasticTimings(F, &Pool, "4-worker pool");
}

TEST_CASE("Profile.Timings.JudgeSplitsElasticAndCertificateTimeAndReuseCostsNothing")
{
	const std::unique_ptr<IStructuralAuthority> Authority = MakeAuthority();
	const WallAndPushedBrick Both = MakeWallAndPushedBrick(true);
	const AuthorityResult First = Authority->Judge(Both.Model, Both.Loads, AuthorityOptions{}, nullptr);
	INFO("wall and pushed brick: " << DescribeAuthority(First));
	REQUIRE(First.Components.size() == 2u);
	const ComponentReport& Wall = First.Components[0];
	const ComponentReport& Pushed = First.Components[1];
	CHECK(SameIdList(Pushed.Bodies, std::vector<BodyId>{Both.Brick}));
	CHECK(Wall.Verdict == ComponentVerdict::Stands);
	CHECK_FALSE(Wall.bConsultedCertificate);
	CHECK(Pushed.bConsultedCertificate);
	CHECK(Pushed.Verdict == ComponentVerdict::Collapses);

	const WallAndPushedBrick Alone = MakeWallAndPushedBrick(false);
	const AuthorityResult WallOnly = Authority->Judge(Alone.Model, {}, AuthorityOptions{}, nullptr);
	INFO("wall alone: " << DescribeAuthority(WallOnly));
	REQUIRE(WallOnly.Components.size() == 1u);
	CHECK_FALSE(WallOnly.Components[0].bConsultedCertificate);

	const AuthorityResult Again = Authority->Judge(Both.Model, Both.Loads, AuthorityOptions{}, &First);
	INFO("judged again: " << DescribeAuthority(Again));
	REQUIRE(Again.Components.size() == 2u);
	CHECK(Again.Components[0].bReused);
	CHECK(Again.Components[1].bReused);

	if (bProfiled)
	{
		CheckFilled(First.ElasticTimings, SolvePath::Dense, "wall and pushed brick ElasticTimings");
		CheckJudgeCovers(First, "wall and pushed brick");
		CHECK(First.CertificateSeconds > 0.0);
		CheckFilled(WallOnly.ElasticTimings, SolvePath::Dense, "wall alone ElasticTimings");
		CheckJudgeCovers(WallOnly, "wall alone");
		CHECK(WallOnly.CertificateSeconds == 0.0);
		CheckAllZero(Again.ElasticTimings, "judged again ElasticTimings");
		CheckJudgeCovers(Again, "judged again");
		CHECK(Again.CertificateSeconds == 0.0);
	}
	else
	{
		CheckJudgeZero(First, "wall and pushed brick");
		CheckJudgeZero(WallOnly, "wall alone");
		CheckJudgeZero(Again, "judged again");
	}
}

TEST_CASE("Measure.Profile.ReadingsDigest")
{
	std::vector<ScaleSubject> Subjects;
	Subjects.push_back(WallSubject("wall 10 x 8", 10, 8));
	Subjects.push_back(WarehouseSubject("shed", ShedSpec(), true));
#if defined(NDEBUG)
	Subjects.push_back(WarehouseSubject("warehouse", geometry::WarehouseSpec{}, false));
#endif
	const auto Report = [](const std::string& Name, const AuthorityResult& R) { MESSAGE(Name << ": digest " << Hex(ReadingsDigest(R)) << ", " << std::string(bProfiled ? "profiled" : "normal") << " build, worst " << std::string(VerdictName(R.Worst)) << ", components " << R.Components.size() << ", readings " << R.Readings.size()); };
	for (const ScaleSubject& S : Subjects)
	{
		INFO(S.Name << ": live bodies " << S.Model.LiveBodyCount() << ", closed form " << S.ExpectedBodies);
		REQUIRE(S.Model.LiveBodyCount() == S.ExpectedBodies);
		Report(S.Name, Judge(S.Model, AuthorityOptions{}));
	}
	const WallAndPushedBrick Pushed = MakeWallAndPushedBrick(true);
	const AuthorityResult R = Judge(Pushed.Model, AuthorityOptions{}, Pushed.Loads);
	INFO("wall and pushed brick: " << DescribeAuthority(R));
	REQUIRE(R.Components.size() == 2u);
	CHECK(R.Components[1].bConsultedCertificate);
	CHECK(R.Components[1].Verdict == ComponentVerdict::Collapses);
	CHECK_FALSE(R.Components[1].Mechanism.empty());
	Report("wall and pushed brick", R);
}
