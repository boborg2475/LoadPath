#include "StaticsTestSupport.h"
#include "WarehouseTestSupport.h"

#include "loadpath/geometry/BoxContacts.h"
#include "loadpath/geometry/Warehouse.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double Exact = 1e-12;
constexpr double PositionTolM = 1e-9;

struct ExpectedBody
{
	int Wall = -1;
	int Course = -1;
	geometry::Box Shape;
	double MassKg = 0;
};

struct ExpectedJoint
{
	std::size_t Lower = 0, Upper = 0;
	double AreaM2 = 0;
};

struct Layout
{
	std::vector<ExpectedBody> Bricks, Posts, Beams, Planks;
	std::vector<ExpectedJoint> PostGround, BeamPost, PlankBeam;
};

geometry::WarehouseSpec ShedSpec(Structure& Model)
{
	geometry::WarehouseSpec Spec = loadpath::test::ShedSpec();
	Spec.Bed = AddProfileOk(Model, DryJoint(0.7));
	Spec.Perpend = AddProfileOk(Model, DryJoint(0.6));
	Spec.Bearing = AddProfileOk(Model, DryJoint(0.3));
	Spec.Plank = AddProfileOk(Model, DryJoint(0.5));
	return Spec;
}

int WallBricks(const geometry::WarehouseSpec& Spec, int W) { return W % 2 == 0 ? Spec.LengthBricks : Spec.WidthBricks; }

std::size_t ClosedFormWallBricks(int N, int M) { return static_cast<std::size_t>(N * M + N / 2); }
std::size_t ClosedFormWallBeds(int N, int M) { return static_cast<std::size_t>(M + 2 * M * (N - 1)); }
std::size_t ClosedFormWallPerpends(int N, int M) { return static_cast<std::size_t>(N * (M - 1) + N / 2); }

bool InOpening(const geometry::WarehouseSpec& Spec, int W, int Course, double S0Bricks, double S1Bricks)
{
	for (const geometry::Opening& O : Spec.Openings)
	{
		if (O.Wall == W && Course >= O.SillCourse && Course < O.LintelCourse && S0Bricks >= O.StartBrick - 1e-9 && S1Bricks <= O.StartBrick + O.WidthBricks + 1e-9)
		{
			return true;
		}
	}
	return false;
}

double Volume(const geometry::Box& B) { return 8.0 * B.HalfExtentsM.X * B.HalfExtentsM.Y * B.HalfExtentsM.Z; }

double Overlap1(double CA, double HA, double CB, double HB) { return std::min(CA + HA, CB + HB) - std::max(CA - HA, CB - HB); }

double PlanOverlapM2(const geometry::Box& A, const geometry::Box& B)
{
	return Overlap1(A.CentreM.X, A.HalfExtentsM.X, B.CentreM.X, B.HalfExtentsM.X) * Overlap1(A.CentreM.Y, A.HalfExtentsM.Y, B.CentreM.Y, B.HalfExtentsM.Y);
}

bool OverlapInVolume(const geometry::Box& A, const geometry::Box& B)
{
	const double Tol = 1e-9;
	return Overlap1(A.CentreM.X, A.HalfExtentsM.X, B.CentreM.X, B.HalfExtentsM.X) > Tol && Overlap1(A.CentreM.Y, A.HalfExtentsM.Y, B.CentreM.Y, B.HalfExtentsM.Y) > Tol
		&& Overlap1(A.CentreM.Z, A.HalfExtentsM.Z, B.CentreM.Z, B.HalfExtentsM.Z) > Tol;
}

Layout ExpectedLayout(const geometry::WarehouseSpec& Spec)
{
	Layout Out;
	const double L = Spec.BrickM.X, T = Spec.BrickM.Y, H = Spec.BrickM.Z;
	const double Lx = Spec.LengthBricks * L, Wy = Spec.WidthBricks * L, G = Spec.GroundZM, Gap = Spec.CornerGapM;
	const double BrickKgPerM = Spec.DensityKgPerM3 * T * H;
	for (int W = 0; W < 4; ++W)
	{
		const int M = WallBricks(Spec, W);
		for (int C = 0; C < Spec.Courses; ++C)
		{
			std::vector<std::pair<double, double>> Spans;
			if (C % 2 == 1)
			{
				Spans.push_back({0.0, 0.5});
				for (int K = 0; K < M - 1; ++K)
				{
					Spans.push_back({0.5 + K, 1.5 + K});
				}
				Spans.push_back({M - 0.5, static_cast<double>(M)});
			}
			else
			{
				for (int K = 0; K < M; ++K)
				{
					Spans.push_back({static_cast<double>(K), K + 1.0});
				}
			}
			for (const auto& [S0, S1] : Spans)
			{
				if (InOpening(Spec, W, C, S0, S1))
				{
					continue;
				}
				const double Mid = 0.5 * (S0 + S1) * L, Half = 0.5 * (S1 - S0) * L, Z = G + (C + 0.5) * H;
				geometry::Box B;
				switch (W)
				{
				case 0: B = {Vec3{Mid, 0.5 * T, Z}, Vec3{Half, 0.5 * T, 0.5 * H}}; break;
				case 2: B = {Vec3{Mid, Wy - 0.5 * T, Z}, Vec3{Half, 0.5 * T, 0.5 * H}}; break;
				case 1: B = {Vec3{-Gap - 0.5 * T, Mid, Z}, Vec3{0.5 * T, Half, 0.5 * H}}; break;
				default: B = {Vec3{Lx + Gap + 0.5 * T, Mid, Z}, Vec3{0.5 * T, Half, 0.5 * H}}; break;
				}
				Out.Bricks.push_back({W, C, B, BrickKgPerM * 2.0 * Half});
			}
		}
	}
	const double P = Spec.PostSideM, Hp = Spec.Courses * H, Span = (Lx - P) / Spec.Bays;
	const double YSide[2] = {T + P, Wy - T - P};
	for (int Side = 0; Side < 2; ++Side)
	{
		for (int K = 0; K <= Spec.Bays; ++K)
		{
			const geometry::Box B{Vec3{0.5 * P + K * Span, YSide[Side], G + 0.5 * Hp}, Vec3{0.5 * P, 0.5 * P, 0.5 * Hp}};
			Out.Posts.push_back({-1, -1, B, Spec.TimberDensityKgPerM3 * Volume(B)});
			Out.PostGround.push_back({Out.Posts.size() - 1, Out.Posts.size() - 1, P * P});
		}
	}
	for (int Side = 0; Side < 2; ++Side)
	{
		for (int J = 0; J < Spec.Bays; ++J)
		{
			const geometry::Box B{Vec3{0.5 * P + (J + 0.5) * Span, YSide[Side], G + Hp + 0.5 * Spec.BeamDepthM}, Vec3{0.5 * Span, 0.5 * Spec.BeamWidthM, 0.5 * Spec.BeamDepthM}};
			Out.Beams.push_back({-1, -1, B, Spec.TimberDensityKgPerM3 * Volume(B)});
			const std::size_t Left = static_cast<std::size_t>(Side * (Spec.Bays + 1) + J);
			Out.BeamPost.push_back({Left, Out.Beams.size() - 1, PlanOverlapM2(Out.Posts[Left].Shape, B)});
			Out.BeamPost.push_back({Left + 1, Out.Beams.size() - 1, PlanOverlapM2(Out.Posts[Left + 1].Shape, B)});
		}
	}
	const double PlankLengthM = YSide[1] - YSide[0] + Spec.BeamWidthM;
	for (int J = 0; J < Spec.Bays; ++J)
	{
		for (int I = 0; I < Spec.PlanksPerBay; ++I)
		{
			const geometry::Box B{Vec3{0.5 * P + J * Span + (I + 0.5) * Span / Spec.PlanksPerBay, 0.5 * (YSide[0] + YSide[1]), G + Hp + Spec.BeamDepthM + 0.5 * Spec.PlankThicknessM},
				Vec3{0.5 * Spec.PlankWidthM, 0.5 * PlankLengthM, 0.5 * Spec.PlankThicknessM}};
			Out.Planks.push_back({-1, -1, B, Spec.TimberDensityKgPerM3 * Volume(B)});
			Out.PlankBeam.push_back({static_cast<std::size_t>(J), Out.Planks.size() - 1, Spec.PlankWidthM * Spec.BeamWidthM});
			Out.PlankBeam.push_back({static_cast<std::size_t>(Spec.Bays + J), Out.Planks.size() - 1, Spec.PlankWidthM * Spec.BeamWidthM});
		}
	}
	return Out;
}

struct BuiltWarehouse
{
	Structure Model;
	geometry::WarehouseSpec Spec;
	geometry::Warehouse Value;
};

BuiltWarehouse BuildShed(bool bWithOpenings = true)
{
	BuiltWarehouse W;
	W.Spec = ShedSpec(W.Model);
	if (!bWithOpenings)
	{
		W.Spec.Openings.clear();
	}
	W.Value = geometry::BuildWarehouse(W.Model, W.Spec);
	return W;
}

std::vector<BodyId> LiveBricks(const Structure& Model, const geometry::Warehouse& W)
{
	std::vector<BodyId> Out;
	for (const geometry::Wall& Wall : W.Walls)
	{
		for (const std::vector<BodyId>& Course : Wall.Courses)
		{
			for (const BodyId& Id : Course)
			{
				if (Model.IsLive(Id))
				{
					Out.push_back(Id);
				}
			}
		}
	}
	return Out;
}

std::size_t Live(const Structure& Model, const std::vector<InterfaceId>& Ids)
{
	std::size_t N = 0;
	for (const InterfaceId& Id : Ids)
	{
		N += Model.IsLive(Id) ? 1u : 0u;
	}
	return N;
}

void Report(const BuiltWarehouse& W)
{
	MESSAGE("warehouse " << W.Spec.LengthBricks << " x " << W.Spec.WidthBricks << " x " << W.Spec.Courses << ", bays " << W.Spec.Bays << ": live bricks " << LiveBricks(W.Model, W.Value).size() << ", posts " << W.Value.Posts.size() << ", beams "
		<< W.Value.Beams.size() << ", planks " << W.Value.Planks.size() << ", bearings " << W.Value.Bearings.size() << ", plank joints " << W.Value.PlankJoints.size() << ", BodyCount " << W.Value.BodyCount() << ", InterfaceCount "
		<< W.Value.InterfaceCount() << ", live bodies " << W.Model.LiveBodyCount() << ", live interfaces " << W.Model.LiveInterfaceCount() << ", complete " << W.Value.bComplete);
}

void CheckBody(const Structure& Model, BodyId Id, const ExpectedBody& Want)
{
	REQUIRE(Model.IsLive(Id));
	const BodyDesc& Got = Model.Body(Id);
	INFO("got centroid " << Got.CentroidM << ", want " << Want.Shape.CentreM << "; got mass " << Got.MassKg << ", want " << Want.MassKg);
	CHECK_FALSE(Got.bGrounded);
	CHECK(Length(Minus(Got.CentroidM, Want.Shape.CentreM)) <= PositionTolM);
	CHECK(Got.MassKg == doctest::Approx(Want.MassKg).epsilon(Exact));
}

void CheckJoint(const Structure& Model, InterfaceId Id, BodyId Lower, BodyId Upper, double AreaM2, ProfileId Profile)
{
	REQUIRE(Model.IsLive(Id));
	const InterfaceRecord& R = Model.Interface(Id);
	INFO("joint " << Id.Index << ": bodies " << R.BodyA.Index << " -> " << R.BodyB.Index << " (want " << Lower.Index << " -> " << Upper.Index << "), area " << R.AreaM2 << " (want " << AreaM2 << "), normal " << R.NormalAToB);
	CHECK(SameHandle(R.BodyA, Lower));
	CHECK(SameHandle(R.BodyB, Upper));
	CHECK(SameHandle(R.Profile, Profile));
	CHECK(Length(Minus(R.NormalAToB, Vec3{0, 0, 1})) <= Exact);
	CHECK(R.AreaM2 == doctest::Approx(AreaM2).epsilon(1e-9));
}
}

TEST_CASE("Geometry.Warehouse.CountsMatchTheClosedForms")
{
	const BuiltWarehouse W = BuildShed(false);
	Report(W);
	const geometry::WarehouseSpec& S = W.Spec;
	const std::size_t Bays = static_cast<std::size_t>(S.Bays), PerBay = static_cast<std::size_t>(S.PlanksPerBay);
	std::size_t Bricks = 0, WallJoints = 0;
	for (int Wall = 0; Wall < 4; ++Wall)
	{
		const int M = WallBricks(S, Wall);
		INFO("wall " << Wall);
		std::size_t Got = 0;
		for (const std::vector<BodyId>& Course : W.Value.Walls[Wall].Courses)
		{
			Got += Course.size();
		}
		CHECK(Got == ClosedFormWallBricks(S.Courses, M));
		CHECK(W.Value.Walls[Wall].Beds.size() == ClosedFormWallBeds(S.Courses, M));
		CHECK(W.Value.Walls[Wall].Perpends.size() == ClosedFormWallPerpends(S.Courses, M));
		Bricks += ClosedFormWallBricks(S.Courses, M);
		WallJoints += ClosedFormWallBeds(S.Courses, M) + ClosedFormWallPerpends(S.Courses, M);
	}
	CHECK(Bricks == 420u);
	CHECK(WallJoints == 1140u);
	CHECK(W.Value.Posts.size() == 2 * (Bays + 1));
	CHECK(W.Value.Beams.size() == 2 * Bays);
	CHECK(W.Value.Planks.size() == Bays * PerBay);
	CHECK(W.Value.Bearings.size() == 2 * (Bays + 1) + 4 * Bays);
	CHECK(W.Value.PlankJoints.size() == 2 * Bays * PerBay);
	const std::size_t Bodies = Bricks + 2 * (Bays + 1) + 2 * Bays + Bays * PerBay + 1;
	const std::size_t Joints = WallJoints + 2 * (Bays + 1) + 4 * Bays + 2 * Bays * PerBay;
	CHECK(Bodies == 437u);
	CHECK(Joints == 1166u);
	CHECK(W.Value.BodyCount() == Bodies);
	CHECK(W.Value.InterfaceCount() == Joints);
	CHECK(W.Model.LiveBodyCount() == Bodies);
	CHECK(W.Model.LiveInterfaceCount() == Joints);
}

TEST_CASE("Geometry.Warehouse.OneGroundCarriesEveryWallAndEveryPost")
{
	const BuiltWarehouse W = BuildShed();
	Report(W);
	REQUIRE(W.Model.IsLive(W.Value.Ground));
	if (!W.Model.IsLive(W.Value.Ground))
	{
		return;
	}
	CHECK(W.Model.Body(W.Value.Ground).bGrounded);
	for (int Wall = 0; Wall < 4; ++Wall)
	{
		INFO("wall " << Wall);
		CHECK(SameHandle(W.Value.Walls[Wall].Ground, W.Value.Ground));
	}
	const std::vector<BodyId> Bricks = LiveBricks(W.Model, W.Value);
	CHECK_FALSE(Bricks.empty());
	CHECK(W.Model.LiveBodyCount() == Bricks.size() + W.Value.Posts.size() + W.Value.Beams.size() + W.Value.Planks.size() + 1);
}

TEST_CASE("Geometry.Warehouse.EveryBodySitsWhereTheLayoutPutsItAndWeighsItsVolume")
{
	const BuiltWarehouse W = BuildShed();
	Report(W);
	const Layout Want = ExpectedLayout(W.Spec);
	const std::vector<BodyId> Bricks = LiveBricks(W.Model, W.Value);
	CHECK(Bricks.size() == Want.Bricks.size());
	CHECK(W.Value.Posts.size() == Want.Posts.size());
	CHECK(W.Value.Beams.size() == Want.Beams.size());
	CHECK(W.Value.Planks.size() == Want.Planks.size());
	if (Bricks.size() != Want.Bricks.size() || W.Value.Posts.size() != Want.Posts.size() || W.Value.Beams.size() != Want.Beams.size() || W.Value.Planks.size() != Want.Planks.size())
	{
		return;
	}
	for (std::size_t I = 0; I < Bricks.size(); ++I)
	{
		INFO("brick " << I << " (wall " << Want.Bricks[I].Wall << ", course " << Want.Bricks[I].Course << ")");
		CheckBody(W.Model, Bricks[I], Want.Bricks[I]);
	}
	for (std::size_t I = 0; I < Want.Posts.size(); ++I)
	{
		INFO("post " << I);
		CheckBody(W.Model, W.Value.Posts[I], Want.Posts[I]);
	}
	for (std::size_t I = 0; I < Want.Beams.size(); ++I)
	{
		INFO("beam " << I);
		CheckBody(W.Model, W.Value.Beams[I], Want.Beams[I]);
	}
	for (std::size_t I = 0; I < Want.Planks.size(); ++I)
	{
		INFO("plank " << I);
		CheckBody(W.Model, W.Value.Planks[I], Want.Planks[I]);
	}
}

TEST_CASE("Geometry.Warehouse.BearingsAndPlankJointsJoinTheBodiesTheyRestOn")
{
	const BuiltWarehouse W = BuildShed();
	Report(W);
	const Layout Want = ExpectedLayout(W.Spec);
	CHECK(W.Value.Bearings.size() == Want.PostGround.size() + Want.BeamPost.size());
	CHECK(W.Value.PlankJoints.size() == Want.PlankBeam.size());
	CHECK(W.Value.Posts.size() == Want.Posts.size());
	CHECK(W.Value.Beams.size() == Want.Beams.size());
	CHECK(W.Value.Planks.size() == Want.Planks.size());
	if (W.Value.Bearings.size() != Want.PostGround.size() + Want.BeamPost.size() || W.Value.PlankJoints.size() != Want.PlankBeam.size() || W.Value.Posts.size() != Want.Posts.size() || W.Value.Beams.size() != Want.Beams.size()
		|| W.Value.Planks.size() != Want.Planks.size())
	{
		return;
	}
	std::size_t B = 0;
	for (const ExpectedJoint& J : Want.PostGround)
	{
		INFO("post-ground bearing " << B);
		CHECK(W.Model.Interface(W.Value.Bearings[B]).CentreM.Z == doctest::Approx(W.Spec.GroundZM).epsilon(Exact));
		CheckJoint(W.Model, W.Value.Bearings[B++], W.Value.Ground, W.Value.Posts[J.Upper], J.AreaM2, W.Spec.Bearing);
	}
	for (const ExpectedJoint& J : Want.BeamPost)
	{
		INFO("beam-post bearing " << B);
		CHECK(J.AreaM2 == doctest::Approx(0.5 * W.Spec.PostSideM * W.Spec.BeamWidthM).epsilon(Exact));
		CheckJoint(W.Model, W.Value.Bearings[B++], W.Value.Posts[J.Lower], W.Value.Beams[J.Upper], J.AreaM2, W.Spec.Bearing);
	}
	for (std::size_t I = 0; I < Want.PlankBeam.size(); ++I)
	{
		INFO("plank joint " << I);
		CheckJoint(W.Model, W.Value.PlankJoints[I], W.Value.Beams[Want.PlankBeam[I].Lower], W.Value.Planks[Want.PlankBeam[I].Upper], Want.PlankBeam[I].AreaM2, W.Spec.Plank);
	}
}

TEST_CASE("Geometry.Warehouse.NoTwoBodiesOverlapAndNoTwoBricksTouchWithoutAJoint")
{
	const BuiltWarehouse W = BuildShed();
	Report(W);
	const Layout Want = ExpectedLayout(W.Spec);
	const std::vector<BodyId> Bricks = LiveBricks(W.Model, W.Value);
	REQUIRE(Bricks.size() == Want.Bricks.size());
	if (Bricks.size() != Want.Bricks.size())
	{
		return;
	}
	std::vector<geometry::Box> All;
	for (const std::vector<ExpectedBody>* Group : {&Want.Bricks, &Want.Posts, &Want.Beams, &Want.Planks})
	{
		for (const ExpectedBody& Body : *Group)
		{
			All.push_back(Body.Shape);
		}
	}
	std::size_t Overlaps = 0;
	for (std::size_t I = 0; I < All.size(); ++I)
	{
		for (std::size_t J = I + 1; J < All.size(); ++J)
		{
			if (OverlapInVolume(All[I], All[J]))
			{
				++Overlaps;
				INFO("bodies " << I << " and " << J << " overlap in volume");
				CHECK(false);
			}
		}
	}
	std::set<std::pair<std::uint32_t, std::uint32_t>> Listed;
	for (const geometry::Wall& Wall : W.Value.Walls)
	{
		for (const std::vector<InterfaceId>* Ids : {&Wall.Beds, &Wall.Perpends})
		{
			for (const InterfaceId& Id : *Ids)
			{
				if (W.Model.IsLive(Id) && !SameHandle(W.Model.Interface(Id).BodyA, W.Value.Ground))
				{
					const InterfaceRecord& R = W.Model.Interface(Id);
					Listed.insert({std::min(R.BodyA.Index, R.BodyB.Index), std::max(R.BodyA.Index, R.BodyB.Index)});
				}
			}
		}
	}
	std::size_t Touching = 0, Unlisted = 0;
	for (std::size_t I = 0; I < Bricks.size(); ++I)
	{
		for (std::size_t J = I + 1; J < Bricks.size(); ++J)
		{
			if (!geometry::FaceContact(Bricks[I], Want.Bricks[I].Shape, Bricks[J], Want.Bricks[J].Shape, W.Spec.Bed))
			{
				continue;
			}
			++Touching;
			if (Listed.count({std::min(Bricks[I].Index, Bricks[J].Index), std::max(Bricks[I].Index, Bricks[J].Index)}) == 0)
			{
				++Unlisted;
				INFO("bricks " << I << " (wall " << Want.Bricks[I].Wall << ") and " << J << " (wall " << Want.Bricks[J].Wall << ") touch with no joint");
				CHECK(false);
			}
		}
	}
	MESSAGE("bodies " << All.size() << ", overlapping pairs " << Overlaps << ", touching brick pairs " << Touching << ", unlisted " << Unlisted << ", listed brick-brick joints " << Listed.size());
	CHECK(Overlaps == 0u);
	CHECK(Unlisted == 0u);
	CHECK(Touching == Listed.size());
}

TEST_CASE("Geometry.Warehouse.AnOpeningRemovesExactlyItsBricksAndTheJointsThatTouchedThem")
{
	const BuiltWarehouse Closed = BuildShed(false);
	const BuiltWarehouse Open = BuildShed(true);
	Report(Closed);
	Report(Open);
	const Layout ClosedLayout = ExpectedLayout(Closed.Spec);
	const Layout OpenLayout = ExpectedLayout(Open.Spec);
	const std::size_t Removed = ClosedLayout.Bricks.size() - OpenLayout.Bricks.size();
	CHECK(Removed == 17u);
	const std::vector<BodyId> ClosedBricks = LiveBricks(Closed.Model, Closed.Value);
	REQUIRE(ClosedBricks.size() == ClosedLayout.Bricks.size());
	if (ClosedBricks.size() != ClosedLayout.Bricks.size())
	{
		return;
	}
	std::set<std::uint32_t> Gone;
	for (std::size_t I = 0; I < ClosedBricks.size(); ++I)
	{
		const ExpectedBody& B = ClosedLayout.Bricks[I];
		const double L = Closed.Spec.BrickM.X;
		const double Along = B.Wall % 2 == 0 ? B.Shape.CentreM.X : B.Shape.CentreM.Y;
		const double Half = B.Wall % 2 == 0 ? B.Shape.HalfExtentsM.X : B.Shape.HalfExtentsM.Y;
		if (InOpening(Open.Spec, B.Wall, B.Course, (Along - Half) / L, (Along + Half) / L))
		{
			Gone.insert(ClosedBricks[I].Index);
		}
	}
	std::size_t TouchedJoints = 0;
	for (const geometry::Wall& Wall : Closed.Value.Walls)
	{
		for (const std::vector<InterfaceId>* Ids : {&Wall.Beds, &Wall.Perpends})
		{
			for (const InterfaceId& Id : *Ids)
			{
				const InterfaceRecord& R = Closed.Model.Interface(Id);
				TouchedJoints += (Gone.count(R.BodyA.Index) + Gone.count(R.BodyB.Index)) > 0 ? 1u : 0u;
			}
		}
	}
	MESSAGE("removed bricks " << Gone.size() << ", joints touching them " << TouchedJoints);
	CHECK(Gone.size() == Removed);
	CHECK(Closed.Model.LiveBodyCount() - Open.Model.LiveBodyCount() == Removed);
	CHECK(Closed.Model.LiveInterfaceCount() - Open.Model.LiveInterfaceCount() == TouchedJoints);
	CHECK(Open.Value.BodyCount() == Open.Model.LiveBodyCount());
	CHECK(Open.Value.InterfaceCount() == Open.Model.LiveInterfaceCount());
	CHECK(LiveBricks(Open.Model, Open.Value).size() == OpenLayout.Bricks.size());
	std::size_t WallJoints = 0;
	for (const geometry::Wall& Wall : Open.Value.Walls)
	{
		WallJoints += Live(Open.Model, Wall.Beds) + Live(Open.Model, Wall.Perpends);
	}
	CHECK(WallJoints + Live(Open.Model, Open.Value.Bearings) + Live(Open.Model, Open.Value.PlankJoints) == Open.Model.LiveInterfaceCount());
}

TEST_CASE("Geometry.Warehouse.AWarehouseWithInvalidProfilesIsNotComplete")
{
	Structure Model;
	const geometry::WarehouseSpec Spec;
	const geometry::Warehouse Default = geometry::BuildWarehouse(Model, Spec);
	INFO("default spec: complete " << Default.bComplete);
	CHECK_FALSE(Default.bComplete);
	const BuiltWarehouse Valid = BuildShed();
	Report(Valid);
	CHECK(Valid.Value.bComplete);
}

TEST_CASE("Geometry.Warehouse.TwoBuildsFromOneSpecAreIdentical")
{
	const BuiltWarehouse A = BuildShed();
	const BuiltWarehouse B = BuildShed();
	Report(A);
	std::vector<BodyId> All[2] = {LiveBricks(A.Model, A.Value), LiveBricks(B.Model, B.Value)};
	const geometry::Warehouse* Values[2] = {&A.Value, &B.Value};
	for (int K = 0; K < 2; ++K)
	{
		for (const std::vector<BodyId>* Group : {&Values[K]->Posts, &Values[K]->Beams, &Values[K]->Planks})
		{
			All[K].insert(All[K].end(), Group->begin(), Group->end());
		}
	}
	REQUIRE(All[0].size() == ExpectedLayout(A.Spec).Bricks.size() + 16u);
	REQUIRE(All[0].size() == All[1].size());
	if (All[0].size() != All[1].size())
	{
		return;
	}
	for (std::size_t I = 0; I < All[0].size(); ++I)
	{
		INFO("body " << I);
		CHECK(All[0][I].Index == All[1][I].Index);
		CHECK(A.Model.Body(All[0][I]).MassKg == B.Model.Body(All[1][I]).MassKg);
		CHECK(Length(Minus(A.Model.Body(All[0][I]).CentroidM, B.Model.Body(All[1][I]).CentroidM)) == 0.0);
	}
}
