#include "StaticsTestSupport.h"

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double BlockLengthM = 1.0;
constexpr double BlockHeightM = 0.2;
constexpr double BlockDepthM = 0.5;
constexpr double BlockMassKg = 1.0;
constexpr double StackFriction = 0.5;

struct PlacedBlock
{
	double CentreXM;
	int Row;
};

Box BlockShape(const PlacedBlock& Block)
{
	return Box{Vec3{Block.CentreXM, 0.0, (Block.Row + 0.5) * BlockHeightM}, Vec3{BlockLengthM / 2.0, BlockDepthM / 2.0, BlockHeightM / 2.0}, BlockMassKg};
}

struct Stack
{
	Structure Model;
	std::vector<BodyId> Blocks;
};

Stack BuildOnTable(const std::vector<PlacedBlock>& Placed, double TableEdgeXM)
{
	Stack S;
	const ProfileId P = AddProfileOk(S.Model, DryJoint(StackFriction));
	const Box Table{Vec3{TableEdgeXM - 5.0, 0.0, -0.5}, Vec3{5.0, 5.0, 0.5}, 0.0};
	const BodyId TableId = AddBox(S.Model, Table, true);
	for (const PlacedBlock& Block : Placed)
	{
		S.Blocks.push_back(AddBox(S.Model, BlockShape(Block)));
	}
	for (std::size_t Upper = 0; Upper < Placed.size(); ++Upper)
	{
		if (Placed[Upper].Row == 0)
		{
			AddInterfaceOk(S.Model, *StackedFaces(TableId, Table, S.Blocks[Upper], BlockShape(Placed[Upper]), P));
			continue;
		}
		for (std::size_t Lower = 0; Lower < Placed.size(); ++Lower)
		{
			if (Placed[Lower].Row + 1 != Placed[Upper].Row)
			{
				continue;
			}
			const std::optional<InterfaceDesc> Contact = StackedFaces(S.Blocks[Lower], BlockShape(Placed[Lower]), S.Blocks[Upper], BlockShape(Placed[Upper]), P);
			if (Contact)
			{
				AddInterfaceOk(S.Model, *Contact);
			}
		}
	}
	return S;
}

std::vector<double> HarmonicRightEdgesFromTopM(int Count, double OffsetScale)
{
	std::vector<double> RightEdges(static_cast<std::size_t>(Count));
	double Edge = 0.0;
	for (int K = Count; K >= 1; --K)
	{
		Edge += OffsetScale * BlockLengthM / (2.0 * K);
		RightEdges[static_cast<std::size_t>(K - 1)] = Edge;
	}
	return RightEdges;
}

std::vector<PlacedBlock> HarmonicStack(int Count, double OffsetScale)
{
	const std::vector<double> RightEdges = HarmonicRightEdgesFromTopM(Count, OffsetScale);
	std::vector<PlacedBlock> Placed;
	for (int K = 1; K <= Count; ++K)
	{
		Placed.push_back(PlacedBlock{RightEdges[static_cast<std::size_t>(K - 1)] - BlockLengthM / 2.0, Count - K});
	}
	return Placed;
}

std::vector<PlacedBlock> BrickRows(const std::vector<int>& CountsBottomUp)
{
	std::vector<PlacedBlock> Placed;
	for (std::size_t Row = 0; Row < CountsBottomUp.size(); ++Row)
	{
		const int Count = CountsBottomUp[Row];
		for (int J = 0; J < Count; ++J)
		{
			Placed.push_back(PlacedBlock{(J - (Count - 1) / 2.0) * BlockLengthM, static_cast<int>(Row)});
		}
	}
	return Placed;
}
}

TEST_CASE("Statics.Anchor.HarmonicStackStandsInsideItsOverhangAndNotBeyond")
{
	const double PublishedOverhangL[] = {0.0, 0.5, 0.75, 11.0 / 12.0, 25.0 / 24.0};
	for (int Count = 2; Count <= 4; ++Count)
	{
		INFO("n = " << Count);
		CHECK(std::abs(HarmonicRightEdgesFromTopM(Count, 1.0).front() - PublishedOverhangL[Count] * BlockLengthM) <= 1.0e-12);

		const Stack Inside = BuildOnTable(HarmonicStack(Count, 1.0 - 1.0e-6), 0.0);
		const StandingResult InsideResult = SolveWith(Inside.Model, StandingOptions{});
		CHECK(InsideResult.State == StandingState::Stands);

		const Stack Beyond = BuildOnTable(HarmonicStack(Count, 1.0 + 1.0e-6), 0.0);
		const StandingResult BeyondResult = SolveWith(Beyond.Model, StandingOptions{});
		INFO("beyond reads " << BeyondResult.State);
		CHECK(BeyondResult.State == StandingState::Mechanism);
	}
}

TEST_CASE("Statics.Anchor.InvertedThreeTriangleFallsAndFourDiamondStands")
{
	const Stack Triangle = BuildOnTable(BrickRows({1, 2, 3}), 5.0);
	CHECK(Triangle.Model.LiveBodyCount() == 7u);
	CHECK(Triangle.Model.LiveInterfaceCount() == 7u);
	const StandingResult TriangleResult = SolveWith(Triangle.Model, StandingOptions{});
	INFO("inverted 3-triangle reads " << TriangleResult.State);
	CHECK(TriangleResult.State == StandingState::Mechanism);

	const Stack Diamond = BuildOnTable(BrickRows({1, 2, 3, 4, 3, 2, 1}), 5.0);
	CHECK(Diamond.Model.LiveBodyCount() == 17u);
	CHECK(Diamond.Model.LiveInterfaceCount() == 25u);
	const StandingResult DiamondResult = SolveWith(Diamond.Model, StandingOptions{});
	INFO("4-diamond reads " << DiamondResult.State);
	CHECK(DiamondResult.State == StandingState::Stands);
}
