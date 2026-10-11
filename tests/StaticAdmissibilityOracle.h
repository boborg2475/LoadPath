#pragma once

#include "StaticsTestSupport.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <ostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// The test-tier static-admissibility oracle: a hand-written Lawson-Hanson NNLS sharing no code with src/.
namespace loadpath::test::oracle
{
inline constexpr int GeneratorsPerCorner = 16;
inline constexpr int CornersPerInterface = 4;
inline constexpr int RowsPerBody = 6;
inline constexpr double MomentRowLengthM = 0.1;
inline constexpr double InnerResidualRelative = 1.0e-9;
inline constexpr double OuterResidualRelative = 1.0e-7;
inline constexpr double DependentColumnRelative = 1.0e-12;
inline constexpr double GridStep = 0.125;
inline constexpr int GridStepsX = 32;
inline constexpr int GridStepsY = 16;
inline constexpr std::uint32_t MasterSeed = 777u;
#if defined(NDEBUG)
inline constexpr bool bOptimisedBuild = true;
#else
inline constexpr bool bOptimisedBuild = false;
#endif
inline constexpr int DrawCount = bOptimisedBuild ? 300 : 20;

enum class Admissibility
{
	CanStand,
	CannotStand,
	Ambiguous
};

inline const char* AdmissibilityName(Admissibility Verdict)
{
	switch (Verdict)
	{
	case Admissibility::CanStand: return "CanStand";
	case Admissibility::CannotStand: return "CannotStand";
	case Admissibility::Ambiguous: return "Ambiguous";
	}
	return "Admissibility(?)";
}

inline std::ostream& operator<<(std::ostream& Out, Admissibility Verdict)
{
	return Out << AdmissibilityName(Verdict);
}

inline std::size_t At(int Index)
{
	return static_cast<std::size_t>(Index);
}

struct ColumnMajor
{
	int Rows = 0;
	int Cols = 0;
	std::vector<double> Values;

	ColumnMajor(int InRows, int InCols) : Rows(InRows), Cols(InCols), Values(At(InRows) * At(InCols), 0.0) {}
	double* Column(int Col) { return Values.data() + At(Col) * At(Rows); }
	const double* Column(int Col) const { return Values.data() + At(Col) * At(Rows); }
};

inline double DotProduct(const double* A, const double* B, int Count)
{
	double Sum = 0.0;
	for (int I = 0; I < Count; ++I)
	{
		Sum += A[I] * B[I];
	}
	return Sum;
}

inline double NormOf(const std::vector<double>& V)
{
	return std::sqrt(DotProduct(V.data(), V.data(), static_cast<int>(V.size())));
}

inline std::vector<double> ResidualOf(const ColumnMajor& A, const std::vector<double>& B, const std::vector<int>& Columns, const std::vector<double>& Coefficients)
{
	std::vector<double> Residual = B;
	for (std::size_t K = 0; K < Columns.size(); ++K)
	{
		const double* Col = A.Column(Columns[K]);
		for (int Row = 0; Row < A.Rows; ++Row)
		{
			Residual[At(Row)] -= Coefficients[K] * Col[Row];
		}
	}
	return Residual;
}

class PassiveLeastSquares
{
public:
	PassiveLeastSquares(const ColumnMajor& InA, const std::vector<double>& InB)
		: A(InA), B(InB), AtB(At(InA.Cols)), Gram(At(InA.Cols) * At(InA.Cols), 0.0), bGramColumn(At(InA.Cols), 0)
	{
		for (int J = 0; J < A.Cols; ++J)
		{
			AtB[At(J)] = DotProduct(A.Column(J), B.data(), A.Rows);
		}
	}

	bool Solve(const std::vector<int>& Set, std::vector<double>& Z)
	{
		Size = static_cast<int>(Set.size());
		for (const int J : Set)
		{
			EnsureGramColumn(J);
		}
		Factor.assign(At(Size) * At(Size), 0.0);
		for (int Row = 0; Row < Size; ++Row)
		{
			for (int Col = 0; Col <= Row; ++Col)
			{
				FactorAt(Row, Col) = GramAt(Set[At(Row)], Set[At(Col)]);
			}
		}
		for (int Col = 0; Col < Size; ++Col)
		{
			double Diagonal = FactorAt(Col, Col);
			for (int P = 0; P < Col; ++P)
			{
				Diagonal -= FactorAt(Col, P) * FactorAt(Col, P);
			}
			if (!(Diagonal > DependentColumnRelative * GramAt(Set[At(Col)], Set[At(Col)])))
			{
				return false;
			}
			FactorAt(Col, Col) = std::sqrt(Diagonal);
			for (int Row = Col + 1; Row < Size; ++Row)
			{
				double Value = FactorAt(Row, Col);
				for (int P = 0; P < Col; ++P)
				{
					Value -= FactorAt(Row, P) * FactorAt(Col, P);
				}
				FactorAt(Row, Col) = Value / FactorAt(Col, Col);
			}
		}
		std::vector<double> Rhs(At(Size));
		for (int I = 0; I < Size; ++I)
		{
			Rhs[At(I)] = AtB[At(Set[At(I)])];
		}
		Z = Substitute(Rhs);
		const std::vector<double> Residual = ResidualOf(A, B, Set, Z);
		for (int I = 0; I < Size; ++I)
		{
			Rhs[At(I)] = DotProduct(A.Column(Set[At(I)]), Residual.data(), A.Rows);
		}
		const std::vector<double> Correction = Substitute(Rhs);
		for (int I = 0; I < Size; ++I)
		{
			Z[At(I)] += Correction[At(I)];
		}
		return true;
	}

private:
	const ColumnMajor& A;
	const std::vector<double>& B;
	std::vector<double> AtB;
	std::vector<double> Gram;
	std::vector<char> bGramColumn;
	std::vector<double> Factor;
	int Size = 0;

	double& GramAt(int Row, int Col) { return Gram[At(Row) * At(A.Cols) + At(Col)]; }
	double& FactorAt(int Row, int Col) { return Factor[At(Row) * At(Size) + At(Col)]; }

	void EnsureGramColumn(int J)
	{
		if (bGramColumn[At(J)] != 0)
		{
			return;
		}
		for (int I = 0; I < A.Cols; ++I)
		{
			const double Value = DotProduct(A.Column(I), A.Column(J), A.Rows);
			GramAt(I, J) = Value;
			GramAt(J, I) = Value;
		}
		bGramColumn[At(J)] = 1;
	}

	std::vector<double> Substitute(const std::vector<double>& Rhs)
	{
		std::vector<double> Y = Rhs;
		for (int Row = 0; Row < Size; ++Row)
		{
			for (int P = 0; P < Row; ++P)
			{
				Y[At(Row)] -= FactorAt(Row, P) * Y[At(P)];
			}
			Y[At(Row)] /= FactorAt(Row, Row);
		}
		for (int Row = Size - 1; Row >= 0; --Row)
		{
			for (int P = Row + 1; P < Size; ++P)
			{
				Y[At(Row)] -= FactorAt(P, Row) * Y[At(P)];
			}
			Y[At(Row)] /= FactorAt(Row, Row);
		}
		return Y;
	}
};

struct NnlsOutcome
{
	double ResidualN = NaN;
	bool bConverged = false;
};

inline std::vector<int> PassiveSet(const std::vector<char>& bPassive)
{
	std::vector<int> Set;
	for (std::size_t J = 0; J < bPassive.size(); ++J)
	{
		if (bPassive[J] != 0)
		{
			Set.push_back(static_cast<int>(J));
		}
	}
	return Set;
}

inline double ResidualNormAt(const ColumnMajor& A, const std::vector<double>& B, const std::vector<double>& X)
{
	std::vector<int> Columns;
	std::vector<double> Coefficients;
	for (int J = 0; J < A.Cols; ++J)
	{
		if (X[At(J)] != 0.0)
		{
			Columns.push_back(J);
			Coefficients.push_back(X[At(J)]);
		}
	}
	return NormOf(ResidualOf(A, B, Columns, Coefficients));
}

inline NnlsOutcome LawsonHanson(const ColumnMajor& A, const std::vector<double>& B)
{
	std::vector<double> X(At(A.Cols), 0.0);
	std::vector<char> bPassive(At(A.Cols), 0);
	std::vector<char> bBlocked(At(A.Cols), 0);
	double LargestEntry = 0.0;
	for (const double Value : A.Values)
	{
		LargestEntry = std::max(LargestEntry, std::abs(Value));
	}
	const double GradientTolerance = 1.0e-12 * LargestEntry * NormOf(B);
	PassiveLeastSquares Solver(A, B);
	NnlsOutcome Outcome;
	for (int Iteration = 0; Iteration < 3 * A.Cols; ++Iteration)
	{
		const std::vector<int> Current = PassiveSet(bPassive);
		std::vector<double> CurrentX;
		for (const int J : Current)
		{
			CurrentX.push_back(X[At(J)]);
		}
		const std::vector<double> Residual = ResidualOf(A, B, Current, CurrentX);
		int Entering = -1;
		double Best = GradientTolerance;
		for (int J = 0; J < A.Cols; ++J)
		{
			if (bPassive[At(J)] != 0 || bBlocked[At(J)] != 0)
			{
				continue;
			}
			const double Gradient = DotProduct(A.Column(J), Residual.data(), A.Rows);
			if (Gradient > Best)
			{
				Best = Gradient;
				Entering = J;
			}
		}
		if (Entering < 0)
		{
			Outcome.bConverged = true;
			break;
		}
		bPassive[At(Entering)] = 1;
		for (int Inner = 0; Inner <= A.Cols; ++Inner)
		{
			const std::vector<int> Set = PassiveSet(bPassive);
			std::vector<double> Z;
			if (!Solver.Solve(Set, Z))
			{
				if (Inner != 0)
				{
					Outcome.ResidualN = ResidualNormAt(A, B, X);
					return Outcome;
				}
				bPassive[At(Entering)] = 0;
				bBlocked[At(Entering)] = 1;
				break;
			}
			bool bAllPositive = true;
			bool bEnteringRejected = false;
			for (std::size_t K = 0; K < Set.size(); ++K)
			{
				if (!(Z[K] > 0.0))
				{
					bAllPositive = false;
					bEnteringRejected = bEnteringRejected || (Inner == 0 && Set[K] == Entering);
				}
			}
			if (bAllPositive)
			{
				std::fill(X.begin(), X.end(), 0.0);
				for (std::size_t K = 0; K < Set.size(); ++K)
				{
					X[At(Set[K])] = Z[K];
				}
				std::fill(bBlocked.begin(), bBlocked.end(), static_cast<char>(0));
				break;
			}
			if (bEnteringRejected)
			{
				bPassive[At(Entering)] = 0;
				bBlocked[At(Entering)] = 1;
				break;
			}
			double Alpha = Infinity;
			int Leaving = -1;
			for (std::size_t K = 0; K < Set.size(); ++K)
			{
				const double Xj = X[At(Set[K])];
				if (!(Z[K] > 0.0) && Xj / (Xj - Z[K]) < Alpha)
				{
					Alpha = Xj / (Xj - Z[K]);
					Leaving = Set[K];
				}
			}
			for (std::size_t K = 0; K < Set.size(); ++K)
			{
				X[At(Set[K])] += Alpha * (Z[K] - X[At(Set[K])]);
			}
			X[At(Leaving)] = 0.0;
			for (const int J : Set)
			{
				if (!(X[At(J)] > 0.0))
				{
					X[At(J)] = 0.0;
					bPassive[At(J)] = 0;
				}
			}
			std::fill(bBlocked.begin(), bBlocked.end(), static_cast<char>(0));
		}
	}
	Outcome.ResidualN = ResidualNormAt(A, B, X);
	return Outcome;
}

struct EquilibriumSystem
{
	ColumnMajor Generators;
	std::vector<double> Demand;
};

inline EquilibriumSystem AssembleEquilibrium(const Structure& Model, const Vec3& GravityVectorMPerS2, double ConeScale)
{
	const std::vector<BodyId> Bodies = Model.LiveBodies();
	const std::vector<InterfaceId> Interfaces = Model.LiveInterfaces();
	std::vector<int> RowOf;
	int Rows = 0;
	for (const BodyId& Body : Bodies)
	{
		if (RowOf.size() <= Body.Index)
		{
			RowOf.resize(static_cast<std::size_t>(Body.Index) + 1, -1);
		}
		if (!Model.Body(Body).bGrounded)
		{
			RowOf[Body.Index] = Rows;
			Rows += RowsPerBody;
		}
	}
	EquilibriumSystem System{ColumnMajor(Rows, static_cast<int>(Interfaces.size()) * CornersPerInterface * GeneratorsPerCorner), std::vector<double>(At(Rows), 0.0)};
	for (const BodyId& Body : Bodies)
	{
		const BodyDesc& Desc = Model.Body(Body);
		if (Desc.bGrounded)
		{
			continue;
		}
		const std::size_t Row = At(RowOf[Body.Index]);
		System.Demand[Row + 0] = -Desc.MassKg * GravityVectorMPerS2.X;
		System.Demand[Row + 1] = -Desc.MassKg * GravityVectorMPerS2.Y;
		System.Demand[Row + 2] = -Desc.MassKg * GravityVectorMPerS2.Z;
	}
	const auto AddWrench = [&](BodyId Body, int Column, const Vec3& PointM, const Vec3& ForceN) {
		const BodyDesc& Desc = Model.Body(Body);
		if (Desc.bGrounded)
		{
			return;
		}
		double* Entries = System.Generators.Column(Column) + RowOf[Body.Index];
		const Vec3 MomentNm = Cross(Minus(PointM, Desc.CentroidM), ForceN);
		Entries[0] += ForceN.X;
		Entries[1] += ForceN.Y;
		Entries[2] += ForceN.Z;
		Entries[3] += MomentNm.X / MomentRowLengthM;
		Entries[4] += MomentNm.Y / MomentRowLengthM;
		Entries[5] += MomentNm.Z / MomentRowLengthM;
	};
	const double SignsU[CornersPerInterface] = {1.0, -1.0, -1.0, 1.0};
	const double SignsV[CornersPerInterface] = {1.0, 1.0, -1.0, -1.0};
	int Column = 0;
	for (const InterfaceId& Id : Interfaces)
	{
		const InterfaceRecord& Record = Model.Interface(Id);
		const double ConeRadius = ConeScale * Model.Profile(Record.Profile).FrictionCoefficient;
		for (int Corner = 0; Corner < CornersPerInterface; ++Corner)
		{
			const Vec3 PointM = Plus(Record.CentreM, Plus(Times(SignsU[Corner] * Record.HalfExtentUM, Record.AxisU), Times(SignsV[Corner] * Record.HalfExtentVM, Record.AxisV)));
			for (int J = 0; J < GeneratorsPerCorner; ++J, ++Column)
			{
				const double AngleRad = 2.0 * Pi * J / GeneratorsPerCorner;
				const Vec3 Generator = Plus(Record.NormalAToB, Plus(Times(ConeRadius * std::cos(AngleRad), Record.AxisU), Times(ConeRadius * std::sin(AngleRad), Record.AxisV)));
				AddWrench(Record.BodyB, Column, PointM, Generator);
				AddWrench(Record.BodyA, Column, PointM, Times(-1.0, Generator));
			}
		}
	}
	return System;
}

struct OracleVerdict
{
	Admissibility Verdict = Admissibility::Ambiguous;
	double InnerResidualOverDemand = NaN;
	double OuterResidualOverDemand = NaN;
	bool bConverged = false;
};

inline OracleVerdict StaticAdmissibility(const Structure& Model, const Vec3& GravityVectorMPerS2)
{
	OracleVerdict Out;
	const EquilibriumSystem Inner = AssembleEquilibrium(Model, GravityVectorMPerS2, 1.0);
	const double DemandN = NormOf(Inner.Demand);
	const NnlsOutcome InnerFit = LawsonHanson(Inner.Generators, Inner.Demand);
	Out.InnerResidualOverDemand = InnerFit.ResidualN / DemandN;
	Out.bConverged = InnerFit.bConverged;
	if (InnerFit.ResidualN <= InnerResidualRelative * DemandN)
	{
		Out.Verdict = Admissibility::CanStand;
		return Out;
	}
	const EquilibriumSystem Outer = AssembleEquilibrium(Model, GravityVectorMPerS2, 1.0 / std::cos(Pi / GeneratorsPerCorner));
	const NnlsOutcome OuterFit = LawsonHanson(Outer.Generators, Outer.Demand);
	Out.OuterResidualOverDemand = OuterFit.ResidualN / DemandN;
	Out.bConverged = Out.bConverged && OuterFit.bConverged;
	Out.Verdict = OuterFit.ResidualN > OuterResidualRelative * DemandN ? Admissibility::CannotStand : Admissibility::Ambiguous;
	return Out;
}

inline Vec3 BiaxialGravity(double LambdaX, double LambdaY)
{
	return Vec3{LambdaX * GravityMPerS2, LambdaY * GravityMPerS2, -GravityMPerS2};
}
inline Box CourseBrick(double CentreXM, int Course)
{
	return Box{Vec3{CentreXM, 0.0, (Course + 0.5) * BrickHeightM}, Vec3{BrickLengthM / 2.0, BrickWidthM / 2.0, BrickHeightM / 2.0}, BrickMassKg};
}

inline void AddJointOnTop(Structure& Model, BodyId Lower, const Box& LowerBox, BodyId Upper, const Box& UpperBox, ProfileId Profile)
{
	const std::optional<InterfaceDesc> Joint = StackedFaces(Lower, LowerBox, Upper, UpperBox, Profile);
	CHECK(Joint.has_value());
	if (Joint.has_value())
	{
		AddInterfaceOk(Model, *Joint);
	}
}

inline Structure MakeStack(double Friction, const std::vector<double>& CentresXM)
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, DryJoint(Friction));
	const BodyId Ground = AddGroundPlane(Model);
	BodyId Below;
	Box BelowBox;
	for (std::size_t Course = 0; Course < CentresXM.size(); ++Course)
	{
		const Box Brick = CourseBrick(CentresXM[Course], static_cast<int>(Course));
		const BodyId Id = AddBox(Model, Brick);
		if (Course == 0)
		{
			AddInterfaceOk(Model, OnGroundPlane(Ground, Id, Brick, Profile));
		}
		else
		{
			AddJointOnTop(Model, Below, BelowBox, Id, Brick, Profile);
		}
		Below = Id;
		BelowBox = Brick;
	}
	return Model;
}

inline Structure MakeBridge(double Friction, double HalfSpanM, double TopOffsetM)
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, DryJoint(Friction));
	const BodyId Ground = AddGroundPlane(Model);
	const Box LeftBox = CourseBrick(-HalfSpanM, 0);
	const Box RightBox = CourseBrick(HalfSpanM, 0);
	const Box TopBox = CourseBrick(TopOffsetM, 1);
	const BodyId Left = AddBox(Model, LeftBox);
	const BodyId Right = AddBox(Model, RightBox);
	const BodyId Top = AddBox(Model, TopBox);
	AddInterfaceOk(Model, OnGroundPlane(Ground, Left, LeftBox, Profile));
	AddInterfaceOk(Model, OnGroundPlane(Ground, Right, RightBox, Profile));
	AddJointOnTop(Model, Left, LeftBox, Top, TopBox, Profile);
	AddJointOnTop(Model, Right, RightBox, Top, TopBox, Profile);
	return Model;
}

inline double Uniform(std::mt19937& Rng, double Low, double High)
{
	return Low + (High - Low) * (static_cast<double>(Rng()) / 4294967296.0);
}

inline bool RigidBrickCanStand(double LambdaX, double LambdaY, double Friction)
{
	return LambdaX * BrickHeightM <= BrickLengthM && LambdaY * BrickHeightM <= BrickWidthM && std::hypot(LambdaX, LambdaY) <= Friction;
}

struct SeededDraw
{
	std::uint32_t Seed = 0;
	Structure Model;
	std::string Fixture;
	double Friction = NaN;
	double LambdaX = NaN;
	double LambdaY = NaN;
};

// Draw k of the seeded stacks and bridges, generated from MasterSeed + k so every run sees the same cases.
inline SeededDraw MakeSeededDraw(int Draw)
{
	SeededDraw Out;
	Out.Seed = MasterSeed + static_cast<std::uint32_t>(Draw);
	std::mt19937 Rng(Out.Seed);
	const bool bStack = (Rng() % 2u) == 0u;
	Out.Friction = Uniform(Rng, 0.4, 1.0);
	std::ostringstream Fixture;
	if (bStack)
	{
		const int Courses = 2 + static_cast<int>(Rng() % 4u);
		std::vector<double> CentresXM{0.0};
		for (int Course = 1; Course < Courses; ++Course)
		{
			CentresXM.push_back(CentresXM.back() + Uniform(Rng, -0.04, 0.04));
		}
		Fixture << "stack of " << Courses << " centred at x";
		for (const double CentreXM : CentresXM)
		{
			Fixture << " " << CentreXM;
		}
		Out.Model = MakeStack(Out.Friction, CentresXM);
	}
	else
	{
		const double HalfSpanM = Uniform(Rng, 0.10, 0.13);
		const double TopOffsetM = Uniform(Rng, -0.02, 0.02);
		Fixture << "bridge half span " << HalfSpanM << ", top offset " << TopOffsetM;
		Out.Model = MakeBridge(Out.Friction, HalfSpanM, TopOffsetM);
	}
	Out.LambdaX = Uniform(Rng, 0.0, 1.5);
	Out.LambdaY = Uniform(Rng, 0.0, 1.0);
	Out.Fixture = Fixture.str();
	return Out;
}
}
