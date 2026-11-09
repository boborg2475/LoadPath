#pragma once

#include "loadpath/StandingAuthority.h"
#include "loadpath/StrengthProfile.h"
#include "loadpath/Units.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace loadpath::test::oracle
{
inline constexpr double BrickXM = 0.215;
inline constexpr double BrickYM = 0.1025;
inline constexpr double BrickZM = 0.065;
inline constexpr double PiValue = 3.14159265358979323846;

// A rigid body in the x-z plane; its three freedoms are measured at its centroid.
struct Body2
{
	std::string Name;
	double XM = 0.0, ZM = 0.0, MassKg = 0.0;
	bool bFixed = false;
};

// Two corner lines of a contact rectangle, each carrying half its area; the normal points from A into B.
struct Contact2
{
	std::string Joint;
	int A = 0, B = 0;
	double NX = 0.0, NZ = 0.0;
	std::array<double, 2> PX{}, PZ{};
	double CornerAreaM2 = 0.0;
	bool bPerpend = false;
	bool bCracked = false;
	std::array<bool, 2> bOpen{}, bSlip{};
	std::array<double, 2> HeldN{};
};

struct Model2
{
	std::vector<Body2> Bodies;
	std::vector<Contact2> Contacts;
	StrengthProfile Bed, Perpend;
};

struct Reading2
{
	std::string Joint;
	double Utilisation = 0.0, Tension = 0.0;
	GoverningAxis Axis = GoverningAxis::None;
	bool bCracked = false;
};

enum class End2
{
	Solved,
	Mechanism,
	NoSettle
};

struct Solve2
{
	End2 End = End2::Solved;
	std::vector<Reading2> Readings;
};

inline const StrengthProfile& RowOf(const Model2& M, const Contact2& C) { return C.bPerpend ? M.Perpend : M.Bed; }

inline int AddBody2(Model2& M, std::string Name, double XM, double ZM, double MassKg, bool bFixed = false)
{
	M.Bodies.push_back(Body2{std::move(Name), XM, ZM, MassKg, bFixed});
	return static_cast<int>(M.Bodies.size()) - 1;
}

inline void AddContact2(Model2& M, std::string Joint, int A, int B, double NX, double NZ, std::array<double, 2> PX, std::array<double, 2> PZ, double AreaM2, bool bPerpend)
{
	Contact2 C;
	C.Joint = std::move(Joint);
	C.A = A;
	C.B = B;
	C.NX = NX;
	C.NZ = NZ;
	C.PX = PX;
	C.PZ = PZ;
	C.CornerAreaM2 = 0.5 * AreaM2;
	C.bPerpend = bPerpend;
	M.Contacts.push_back(std::move(C));
}

struct Row2
{
	std::array<int, 6> Dof{-1, -1, -1, -1, -1, -1};
	std::array<double, 6> Normal{}, Tangent{};
};

inline Row2 CornerRow(const Model2& M, const std::vector<int>& DofOf, const Contact2& C, std::size_t K)
{
	Row2 R;
	const double TX = -C.NZ;
	const double TZ = C.NX;
	for (int Side = 0; Side < 2; ++Side)
	{
		const int Body = Side == 0 ? C.A : C.B;
		const double S = Side == 0 ? -1.0 : 1.0;
		const double DX = C.PX[K] - M.Bodies[static_cast<std::size_t>(Body)].XM;
		const double DZ = C.PZ[K] - M.Bodies[static_cast<std::size_t>(Body)].ZM;
		const int Base = DofOf[static_cast<std::size_t>(Body)];
		for (int D = 0; D < 3; ++D)
		{
			R.Dof[static_cast<std::size_t>(3 * Side + D)] = Base < 0 ? -1 : Base + D;
		}
		R.Normal[static_cast<std::size_t>(3 * Side)] = S * C.NX;
		R.Normal[static_cast<std::size_t>(3 * Side + 1)] = S * C.NZ;
		R.Normal[static_cast<std::size_t>(3 * Side + 2)] = S * (-C.NX * DZ + C.NZ * DX);
		R.Tangent[static_cast<std::size_t>(3 * Side)] = S * TX;
		R.Tangent[static_cast<std::size_t>(3 * Side + 1)] = S * TZ;
		R.Tangent[static_cast<std::size_t>(3 * Side + 2)] = S * (-TX * DZ + TZ * DX);
	}
	return R;
}

inline double Dot(const Row2& R, const std::array<double, 6>& G, const std::vector<double>& U)
{
	double Sum = 0.0;
	for (std::size_t I = 0; I < 6; ++I)
	{
		if (R.Dof[I] >= 0)
		{
			Sum += G[I] * U[static_cast<std::size_t>(R.Dof[I])];
		}
	}
	return Sum;
}

// Lower band storage, row I holding columns I - Band .. I.
struct Band2
{
	int N = 0, Width = 0;
	std::vector<double> Values;
	double& At(int I, int J) { return Values[static_cast<std::size_t>(I) * static_cast<std::size_t>(Width + 1) + static_cast<std::size_t>(I - J)]; }
};

// Banded Cholesky; false when a pivot falls below 1e-10 of its diagonal, which is how a mechanism shows.
inline bool SolveBanded(Band2& K, std::vector<double>& F)
{
	const int N = K.N;
	const int W = K.Width;
	for (int J = 0; J < N; ++J)
	{
		const double Diagonal = K.At(J, J);
		double Pivot = Diagonal;
		for (int P = std::max(0, J - W); P < J; ++P)
		{
			Pivot -= K.At(J, P) * K.At(J, P);
		}
		if (!(Pivot > 1e-10 * Diagonal))
		{
			return false;
		}
		K.At(J, J) = std::sqrt(Pivot);
		for (int I = J + 1; I < std::min(N, J + W + 1); ++I)
		{
			double Sum = K.At(I, J);
			for (int P = std::max(0, I - W); P < J; ++P)
			{
				Sum -= K.At(I, P) * K.At(J, P);
			}
			K.At(I, J) = Sum / K.At(J, J);
		}
	}
	for (int I = 0; I < N; ++I)
	{
		double Sum = F[static_cast<std::size_t>(I)];
		for (int P = std::max(0, I - W); P < I; ++P)
		{
			Sum -= K.At(I, P) * F[static_cast<std::size_t>(P)];
		}
		F[static_cast<std::size_t>(I)] = Sum / K.At(I, I);
	}
	for (int I = N - 1; I >= 0; --I)
	{
		double Sum = F[static_cast<std::size_t>(I)];
		for (int P = I + 1; P < std::min(N, I + W + 1); ++P)
		{
			Sum -= K.At(P, I) * F[static_cast<std::size_t>(P)];
		}
		F[static_cast<std::size_t>(I)] = Sum / K.At(I, I);
	}
	return true;
}

inline bool SolveLinear(const Model2& M, const std::vector<int>& DofOf, int N, std::vector<double>& U)
{
	Band2 K;
	K.N = N;
	for (const Contact2& C : M.Contacts)
	{
		const int A = DofOf[static_cast<std::size_t>(C.A)];
		const int B = DofOf[static_cast<std::size_t>(C.B)];
		if (A >= 0 && B >= 0)
		{
			K.Width = std::max(K.Width, std::abs(A - B) + 2);
		}
	}
	K.Width = std::max(K.Width, 2);
	K.Values.assign(static_cast<std::size_t>(N) * static_cast<std::size_t>(K.Width + 1), 0.0);
	U.assign(static_cast<std::size_t>(N), 0.0);
	for (const Contact2& C : M.Contacts)
	{
		const StrengthProfile& P = RowOf(M, C);
		for (std::size_t Corner = 0; Corner < 2; ++Corner)
		{
			if (C.bOpen[Corner])
			{
				continue;
			}
			const Row2 R = CornerRow(M, DofOf, C, Corner);
			const double Kn = P.NormalStiffnessPaPerM * C.CornerAreaM2;
			const double Kt = C.bSlip[Corner] ? 0.0 : P.TangentialStiffnessPaPerM * C.CornerAreaM2;
			for (std::size_t I = 0; I < 6; ++I)
			{
				for (std::size_t J = 0; J < 6; ++J)
				{
					if (R.Dof[I] >= 0 && R.Dof[J] >= 0 && R.Dof[J] <= R.Dof[I])
					{
						K.At(R.Dof[I], R.Dof[J]) += Kn * R.Normal[I] * R.Normal[J] + Kt * R.Tangent[I] * R.Tangent[J];
					}
				}
			}
			if (C.bSlip[Corner])
			{
				for (std::size_t I = 0; I < 6; ++I)
				{
					if (R.Dof[I] >= 0)
					{
						U[static_cast<std::size_t>(R.Dof[I])] -= C.HeldN[Corner] * R.Tangent[I];
					}
				}
			}
		}
	}
	for (std::size_t B = 0; B < M.Bodies.size(); ++B)
	{
		if (DofOf[B] >= 0)
		{
			U[static_cast<std::size_t>(DofOf[B] + 1)] -= M.Bodies[B].MassKg * GravityMPerS2;
		}
	}
	return SolveBanded(K, U);
}

inline Reading2 ReadContact(const Model2& M, const std::vector<int>& DofOf, const Contact2& C, const std::vector<double>& U)
{
	const StrengthProfile& P = RowOf(M, C);
	Reading2 R;
	R.Joint = C.Joint;
	R.bCracked = C.bCracked;
	const auto Govern = [&](double Ratio, GoverningAxis Axis) {
		if (Ratio > R.Utilisation || std::isnan(Ratio))
		{
			R.Utilisation = Ratio;
			R.Axis = Axis;
		}
	};
	for (std::size_t Corner = 0; Corner < 2; ++Corner)
	{
		if (C.bOpen[Corner])
		{
			continue;
		}
		const Row2 Row = CornerRow(M, DofOf, C, Corner);
		const double A = C.CornerAreaM2;
		const double NormalN = -P.NormalStiffnessPaPerM * A * Dot(Row, Row.Normal, U);
		const double ShearN = C.bSlip[Corner] ? std::abs(C.HeldN[Corner]) : std::abs(P.TangentialStiffnessPaPerM * A * Dot(Row, Row.Tangent, U));
		const double TensileN = C.bCracked ? 0.0 : P.TensileStrengthPa * A;
		const double CohesionN = C.bCracked ? 0.0 : P.ShearCohesionPa * A;
		const double Tension = NormalN < 0.0 ? (TensileN > 0.0 ? -NormalN / TensileN : std::numeric_limits<double>::infinity()) : 0.0;
		const double ShearCapacityN = std::min(CohesionN + P.FrictionCoefficient * std::max(NormalN, 0.0), P.MaxShearStrengthPa * A);
		const double Shear = ShearN > 0.0 ? (ShearCapacityN > 0.0 ? ShearN / ShearCapacityN : std::numeric_limits<double>::infinity()) : 0.0;
		R.Tension = std::max(R.Tension, Tension);
		Govern(std::max(NormalN, 0.0) / (P.CompressiveStrengthPa * A), GoverningAxis::Compression);
		Govern(Tension, GoverningAxis::Tension);
		Govern(Shear, GoverningAxis::Shear);
	}
	return R;
}

inline bool UpdateCrackedCorner(const Model2& M, const std::vector<int>& DofOf, Contact2& C, std::size_t Corner, const std::vector<double>& U)
{
	const StrengthProfile& P = RowOf(M, C);
	const Row2 Row = CornerRow(M, DofOf, C, Corner);
	const double Opening = Dot(Row, Row.Normal, U);
	const bool bOpen = C.bOpen[Corner] ? Opening > 0.0 : Opening > 1e-15;
	bool bChanged = bOpen != C.bOpen[Corner];
	C.bOpen[Corner] = bOpen;
	if (bOpen)
	{
		bChanged = bChanged || C.bSlip[Corner];
		C.bSlip[Corner] = false;
		return bChanged;
	}
	const double LimitN = P.FrictionCoefficient * std::max(-P.NormalStiffnessPaPerM * C.CornerAreaM2 * Opening, 0.0);
	const double TrialN = P.TangentialStiffnessPaPerM * C.CornerAreaM2 * Dot(Row, Row.Tangent, U);
	if (!C.bSlip[Corner] && std::abs(TrialN) > LimitN * (1.0 + 1e-9))
	{
		C.bSlip[Corner] = true;
		C.HeldN[Corner] = TrialN > 0.0 ? LimitN : -LimitN;
		return true;
	}
	if (C.bSlip[Corner])
	{
		const double Held = C.HeldN[Corner] > 0.0 ? LimitN : -LimitN;
		bChanged = bChanged || std::abs(Held - C.HeldN[Corner]) > 1e-9 * (std::abs(Held) + 1e-6);
		C.HeldN[Corner] = Held;
	}
	return bChanged;
}

// Bonded joints stay linear; a cracked joint's corners open in tension and slip at friction, iterated until no state or held force changes.
inline Solve2 Solve(Model2& M)
{
	std::vector<int> DofOf(M.Bodies.size(), -1);
	int N = 0;
	for (std::size_t B = 0; B < M.Bodies.size(); ++B)
	{
		if (!M.Bodies[B].bFixed)
		{
			DofOf[B] = N;
			N += 3;
		}
	}
	Solve2 Out;
	std::vector<double> U;
	for (int Iteration = 0; Iteration < 2000; ++Iteration)
	{
		if (!SolveLinear(M, DofOf, N, U))
		{
			Out.End = End2::Mechanism;
			return Out;
		}
		bool bChanged = false;
		for (Contact2& C : M.Contacts)
		{
			for (std::size_t Corner = 0; C.bCracked && Corner < 2; ++Corner)
			{
				bChanged = UpdateCrackedCorner(M, DofOf, C, Corner, U) || bChanged;
			}
		}
		if (!bChanged)
		{
			for (const Contact2& C : M.Contacts)
			{
				Out.Readings.push_back(ReadContact(M, DofOf, C, U));
			}
			return Out;
		}
	}
	Out.End = End2::NoSettle;
	return Out;
}

inline const Reading2* Worst(const Solve2& S)
{
	const Reading2* Top = nullptr;
	for (const Reading2& R : S.Readings)
	{
		if (Top == nullptr || R.Utilisation > Top->Utilisation)
		{
			Top = &R;
		}
	}
	return Top;
}

struct Sequence2
{
	bool bStands = false;
	End2 End = End2::Solved;
	std::vector<std::string> Cracked;
};

// The hinge sequence: crack the single worst joint over 1 and solve again; a mechanism, an unsettled active set or a cracked joint past friction falls.
inline Sequence2 RunHingeSequence(Model2 M)
{
	Sequence2 Out;
	for (std::size_t Step = 0; Step <= M.Contacts.size(); ++Step)
	{
		const Solve2 S = Solve(M);
		Out.End = S.End;
		if (S.End != End2::Solved)
		{
			return Out;
		}
		const Reading2* Top = Worst(S);
		if (Top == nullptr || Top->Utilisation <= 1.0 + 1e-6)
		{
			Out.bStands = true;
			return Out;
		}
		const std::size_t Index = static_cast<std::size_t>(Top - S.Readings.data());
		if (M.Contacts[Index].bCracked)
		{
			return Out;
		}
		M.Contacts[Index].bCracked = true;
		Out.Cracked.push_back(Top->Joint);
	}
	return Out;
}

struct Pier2
{
	std::string Name;
	double OriginXM = 0.0;
	int Courses = 0, Bricks = 0;
};

struct Span2
{
	double X0 = 0.0, X1 = 0.0;
	int Brick = 0;
};

inline std::vector<Span2> CourseSpans(const Pier2& P, int Course)
{
	std::vector<Span2> Out;
	double X = P.OriginXM;
	const int Count = Course % 2 == 1 ? P.Bricks + 1 : P.Bricks;
	for (int K = 0; K < Count; ++K)
	{
		const bool bBat = Course % 2 == 1 && (K == 0 || K == Count - 1);
		const double Length = bBat ? 0.5 * BrickXM : BrickXM;
		Out.push_back(Span2{X, X + Length, K});
		X += Length;
	}
	return Out;
}

inline std::string BrickName2(const Pier2& P, int Course, int Brick) { return P.Name + ".c" + std::to_string(Course) + ".b" + std::to_string(Brick); }

// Every brick is a rigid body, bats at half mass; bodies are added top course first when bTopFirst, so the band stays narrow.
inline std::vector<std::vector<int>> AddPier2(Model2& M, const Pier2& P, int Ground, double DensityKgPerM3, bool bTopFirst)
{
	std::vector<std::vector<int>> Bricks(static_cast<std::size_t>(P.Courses));
	for (int I = 0; I < P.Courses; ++I)
	{
		const int C = bTopFirst ? P.Courses - 1 - I : I;
		for (const Span2& S : CourseSpans(P, C))
		{
			Bricks[static_cast<std::size_t>(C)].push_back(
				AddBody2(M, BrickName2(P, C, S.Brick), 0.5 * (S.X0 + S.X1), (C + 0.5) * BrickZM, DensityKgPerM3 * (S.X1 - S.X0) * BrickYM * BrickZM));
		}
	}
	for (int C = 0; C < P.Courses; ++C)
	{
		const std::vector<Span2> Spans = CourseSpans(P, C);
		const std::vector<int>& Row = Bricks[static_cast<std::size_t>(C)];
		for (std::size_t K = 0; K + 1 < Spans.size(); ++K)
		{
			const double X = Spans[K].X1;
			AddContact2(M, BrickName2(P, C, Spans[K].Brick) + "/" + BrickName2(P, C, Spans[K + 1].Brick), Row[K], Row[K + 1], 1.0, 0.0, {X, X}, {C * BrickZM, (C + 1) * BrickZM},
				BrickZM * BrickYM, true);
		}
		if (C == 0)
		{
			for (std::size_t K = 0; K < Spans.size(); ++K)
			{
				AddContact2(M, BrickName2(P, 0, Spans[K].Brick) + "/ground", Ground, Row[K], 0.0, 1.0, {Spans[K].X0, Spans[K].X1}, {0.0, 0.0}, (Spans[K].X1 - Spans[K].X0) * BrickYM, false);
			}
			continue;
		}
		const std::vector<Span2> Below = CourseSpans(P, C - 1);
		for (std::size_t L = 0; L < Below.size(); ++L)
		{
			for (std::size_t K = 0; K < Spans.size(); ++K)
			{
				const double X0 = std::max(Below[L].X0, Spans[K].X0);
				const double X1 = std::min(Below[L].X1, Spans[K].X1);
				if (X1 - X0 > 1e-9)
				{
					const double Z = C * BrickZM;
					AddContact2(M, BrickName2(P, C - 1, Below[L].Brick) + "/" + BrickName2(P, C, Spans[K].Brick), Bricks[static_cast<std::size_t>(C - 1)][L], Row[K], 0.0, 1.0, {X0, X1},
						{Z, Z}, (X1 - X0) * BrickYM, false);
				}
			}
		}
	}
	return Bricks;
}

struct SoldierCourse
{
	Pier2 Left, Right;
	int Count = 0;
	double BottomM = 0.0, HeightM = 0.0, WidthM = 0.0;
	double DensityKgPerM3 = 0.0;
};

// The A2 model: soldiers and pier bricks rigid, soldier joints and the pier-face junction on the perpend row, contact by contact.
inline Model2 BuildSoldierCourse(const SoldierCourse& S, const StrengthProfile& Bed, const StrengthProfile& Perpend)
{
	Model2 M;
	M.Bed = Bed;
	M.Perpend = Perpend;
	const int Ground = AddBody2(M, "ground", 0.0, 0.0, 0.0, true);
	const std::vector<std::vector<int>> Left = AddPier2(M, S.Left, Ground, S.DensityKgPerM3, false);
	const double FaceM = S.Left.OriginXM + S.Left.Bricks * BrickXM;
	std::vector<int> Soldiers;
	for (int K = 0; K < S.Count; ++K)
	{
		Soldiers.push_back(AddBody2(M, "s." + std::to_string(K), FaceM + (K + 0.5) * S.WidthM, S.BottomM + 0.5 * S.HeightM, S.DensityKgPerM3 * S.WidthM * BrickYM * S.HeightM));
	}
	const std::vector<std::vector<int>> Right = AddPier2(M, S.Right, Ground, S.DensityKgPerM3, true);
	const double TopM = S.BottomM + S.HeightM;
	for (int K = 0; K + 1 < S.Count; ++K)
	{
		const double X = FaceM + (K + 1) * S.WidthM;
		AddContact2(M, "s." + std::to_string(K) + "/s." + std::to_string(K + 1), Soldiers[static_cast<std::size_t>(K)], Soldiers[static_cast<std::size_t>(K + 1)], 1.0, 0.0, {X, X},
			{S.BottomM, TopM}, S.HeightM * BrickYM, true);
	}
	for (int Side = 0; Side < 2; ++Side)
	{
		const Pier2& P = Side == 0 ? S.Left : S.Right;
		const std::vector<std::vector<int>>& Bricks = Side == 0 ? Left : Right;
		const int Soldier = Soldiers[static_cast<std::size_t>(Side == 0 ? 0 : S.Count - 1)];
		const double X = Side == 0 ? FaceM : P.OriginXM;
		for (int C = 0; C < P.Courses; ++C)
		{
			const double Z0 = std::max(C * BrickZM, S.BottomM);
			const double Z1 = std::min((C + 1) * BrickZM, TopM);
			if (!(Z1 - Z0 > 1e-9))
			{
				continue;
			}
			const std::vector<Span2> Spans = CourseSpans(P, C);
			const std::size_t K = Side == 0 ? Spans.size() - 1 : 0;
			const int Brick = Bricks[static_cast<std::size_t>(C)][K];
			const std::string Joint = "s." + std::to_string(Side == 0 ? 0 : S.Count - 1) + "/" + BrickName2(P, C, Spans[K].Brick);
			AddContact2(M, Joint, Side == 0 ? Brick : Soldier, Side == 0 ? Soldier : Brick, 1.0, 0.0, {X, X}, {Z0, Z1}, (Z1 - Z0) * BrickYM, true);
		}
	}
	return M;
}

struct BrickRing
{
	Pier2 Left, Right;
	double CentreXM = 0.0, SpringingZM = 0.0, RadiusM = 0.0, ThicknessM = 0.0, DepthM = 0.0;
	int Voussoirs = 0;
	double DensityKgPerM3 = 0.0;
};

// The A1 model: voussoirs rigid with exact sector weights, pier bricks rigid, the springing on the bed row contact by contact.
inline Model2 BuildBrickRing(const BrickRing& A, const StrengthProfile& Bed, const StrengthProfile& Perpend)
{
	Model2 M;
	M.Bed = Bed;
	M.Perpend = Perpend;
	const int Ground = AddBody2(M, "ground", 0.0, 0.0, 0.0, true);
	const std::vector<std::vector<int>> Left = AddPier2(M, A.Left, Ground, A.DensityKgPerM3, false);
	const double Delta = 0.5 * PiValue / A.Voussoirs;
	const double Ri = A.RadiusM - 0.5 * A.ThicknessM;
	const double Ro = A.RadiusM + 0.5 * A.ThicknessM;
	const double Rc = (2.0 / 3.0) * (std::sin(Delta) / Delta) * (Ro * Ro * Ro - Ri * Ri * Ri) / (Ro * Ro - Ri * Ri);
	const double MassKg = A.DensityKgPerM3 * A.DepthM * Delta * (Ro * Ro - Ri * Ri);
	std::vector<int> V;
	for (int K = 0; K < A.Voussoirs; ++K)
	{
		const double Beta = -0.5 * PiValue + (2 * K + 1) * Delta;
		V.push_back(AddBody2(M, "v." + std::to_string(K), A.CentreXM + Rc * std::sin(Beta), A.SpringingZM + Rc * std::cos(Beta), MassKg));
	}
	const std::vector<std::vector<int>> Right = AddPier2(M, A.Right, Ground, A.DensityKgPerM3, true);
	for (int K = 1; K < A.Voussoirs; ++K)
	{
		const double Phi = -0.5 * PiValue + 2 * K * Delta;
		const double S = std::sin(Phi);
		const double C = std::cos(Phi);
		AddContact2(M, "v." + std::to_string(K - 1) + "/v." + std::to_string(K), V[static_cast<std::size_t>(K - 1)], V[static_cast<std::size_t>(K)], C, -S,
			{A.CentreXM + Ri * S, A.CentreXM + Ro * S}, {A.SpringingZM + Ri * C, A.SpringingZM + Ro * C}, A.ThicknessM * A.DepthM, false);
	}
	for (int Side = 0; Side < 2; ++Side)
	{
		const Pier2& P = Side == 0 ? A.Left : A.Right;
		const int Top = P.Courses - 1;
		const int Voussoir = Side == 0 ? 0 : A.Voussoirs - 1;
		const double Foot = A.CentreXM + (Side == 0 ? -A.RadiusM : A.RadiusM);
		const std::vector<Span2> Spans = CourseSpans(P, Top);
		for (std::size_t K = 0; K < Spans.size(); ++K)
		{
			const double X0 = std::max(Spans[K].X0, Foot - 0.5 * A.ThicknessM);
			const double X1 = std::min(Spans[K].X1, Foot + 0.5 * A.ThicknessM);
			if (X1 - X0 > 1e-9)
			{
				AddContact2(M, "v." + std::to_string(Voussoir) + "/" + BrickName2(P, Top, Spans[K].Brick), (Side == 0 ? Left : Right)[static_cast<std::size_t>(Top)][K],
					V[static_cast<std::size_t>(Voussoir)], 0.0, 1.0, {X0, X1}, {A.SpringingZM, A.SpringingZM}, (X1 - X0) * A.DepthM, false);
			}
		}
	}
	return M;
}

inline void RemoveBody2(Model2& M, std::string_view Name)
{
	for (std::size_t I = 0; I < M.Bodies.size(); ++I)
	{
		if (M.Bodies[I].Name != Name)
		{
			continue;
		}
		M.Bodies[I].bFixed = true;
		M.Bodies[I].MassKg = 0.0;
		const int Index = static_cast<int>(I);
		std::erase_if(M.Contacts, [&](const Contact2& C) { return C.A == Index || C.B == Index; });
	}
}

inline int NearestOddVoussoirs(double RadiusM)
{
	const double Exact = PiValue * RadiusM / BrickZM;
	const int Down = 2 * static_cast<int>(std::floor((Exact - 1.0) / 2.0)) + 1;
	return Exact - Down <= Down + 2 - Exact ? Down : Down + 2;
}
}
