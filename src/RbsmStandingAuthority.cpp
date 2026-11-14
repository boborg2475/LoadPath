#include "loadpath/StandingAuthority.h"

#if defined(_MSC_VER)
#pragma warning(disable : 4996)
#endif

#include "loadpath/Settings.h"

#include "EigenInclude.h"
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4702)
#endif
#include <Eigen/Eigenvalues>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
#include "PhaseClock.h"
#include "StaticsCommon.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

namespace loadpath
{
namespace
{
using statics::Add;
using statics::AddForceAt;
using statics::CornerCount;
using statics::CornerJacobian;
using statics::DofPerBody;
using statics::Scale;
using statics::Sub;

constexpr double Infinity = std::numeric_limits<double>::infinity();

enum class CornerStatus : std::uint8_t
{
	Stuck,
	Slipping,
	Open
};

struct Joint : statics::JointGeometry
{
	double NormalStiffnessNPerM = 0;
	double TangentialStiffnessNPerM = 0;
	double CompressiveCapacityN = 0;
	double TensileCapacityN = 0;
	double CohesionCapacityN = 0;
	double ShearCapN = 0;
	double Friction = 0;
	bool bReleasesOpening = false;
	bool bReleasesSlip = false;
};

struct JointState
{
	std::array<CornerStatus, CornerCount> Status{};
	std::array<Vec3, CornerCount> HeldN{};
	std::array<double, CornerCount> TransverseNPerM{};
	std::array<Vec3, CornerCount> ReferenceM{};
	std::array<int, CornerCount> ReclosureCount{};
};

struct Problem
{
	int DofCount = 0;
	std::vector<BodyId> FreeBodies;
	std::vector<Joint> Joints;
	std::vector<std::size_t> ReadingSlots;
	Eigen::VectorXd AppliedLoad;
	Eigen::VectorXd Scaling;
	double ForceScaleN = 0;
	bool bGrounded = false;
	bool bHostLoaded = false;
};

using IndexVector = Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>;

struct Factorisation
{
	Eigen::MatrixXd Lower;
	IndexVector Order;
	Eigen::Index Rank = 0;
};

enum class Restoration : std::uint8_t
{
	None,
	Restored
};

struct UpdateOutcome
{
	bool bStatusChanged = false;
	bool bHeldSettled = true;
	bool bUndecided = false;
};

struct Ending
{
	SolveEnd End = SolveEnd::None;
	std::vector<ElasticPassRecord> Passes;
};

struct TrialForces
{
	double GapM = 0;
	double NormalN = 0;
	Vec3 ShearN;
};

struct CornerRatios
{
	double Compression = 0;
	double Tension = 0;
	double Shear = 0;
};

bool IsExactlyZero(const Vec3& V)
{
	return V.X == 0.0 && V.Y == 0.0 && V.Z == 0.0;
}

double PositivePart(double X)
{
	return X < 0.0 ? 0.0 : X;
}

bool IsUsableStiffness(double StiffnessPaPerM)
{
	return StiffnessPaPerM > 0.0 && IsFinite(StiffnessPaPerM);
}

bool FindInvalidInput(const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options, InterfaceId& Offending)
{
	if (!statics::AreStandingOptionsValid(Options))
	{
		return true;
	}
	for (const InterfaceId Id : Model.LiveInterfaces())
	{
		const StrengthProfile Profile = Model.EffectiveProfile(Id);
		if (!IsUsableStiffness(Profile.NormalStiffnessPaPerM) || !IsUsableStiffness(Profile.TangentialStiffnessPaPerM))
		{
			Offending = Id;
			return true;
		}
	}
	for (const BodyLoad& Load : Loads)
	{
		if (!Model.IsLive(Load.Body) || !IsFinite(Load.ForceN) || !IsFinite(Load.PointM))
		{
			return true;
		}
	}
	return !IsFinite(Options.Gravity);
}

Joint MakeJoint(const Structure& Model, InterfaceId Id, const std::vector<int>& OffsetOfSlot)
{
	const InterfaceRecord& Record = Model.Interface(Id);
	const StrengthProfile Profile = Model.EffectiveProfile(Id);
	const double CornerAreaM2 = Record.AreaM2 / 4.0;
	Joint J;
	static_cast<statics::JointGeometry&>(J) = statics::MakeJointGeometry(Model, Id, OffsetOfSlot);
	J.NormalStiffnessNPerM = Profile.NormalStiffnessPaPerM * CornerAreaM2;
	J.TangentialStiffnessNPerM = Profile.TangentialStiffnessPaPerM * CornerAreaM2;
	J.CompressiveCapacityN = Profile.CompressiveStrengthPa * CornerAreaM2;
	J.TensileCapacityN = Profile.TensileStrengthPa * CornerAreaM2;
	J.CohesionCapacityN = Profile.ShearCohesionPa * CornerAreaM2;
	J.ShearCapN = Profile.MaxShearStrengthPa * CornerAreaM2;
	J.Friction = Profile.FrictionCoefficient;
	J.bReleasesOpening = !(Profile.TensileStrengthPa > 0.0);
	J.bReleasesSlip = !(Profile.ShearCohesionPa > 0.0);
	return J;
}

Eigen::VectorXd RotationalScaling(const Problem& P)
{
	const double InverseLengthPerM = 1.0 / statics::CharacteristicLengthM(P.Joints);
	Eigen::VectorXd Scaling = Eigen::VectorXd::Ones(P.DofCount);
	for (int Offset = 0; Offset < P.DofCount; Offset += DofPerBody)
	{
		Scaling.segment(Offset + 3, 3).setConstant(InverseLengthPerM);
	}
	return Scaling;
}

std::vector<Problem> BuildProblems(const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options)
{
	const ComponentPartition& Partition = Model.Components();
	const std::vector<int>& OffsetOfSlot = Partition.OffsetOfSlot;
	std::vector<Problem> Problems(Partition.FreeBodies.size() + 1);
	for (std::size_t K = 0; K < Partition.FreeBodies.size(); ++K)
	{
		Problems[K + 1].FreeBodies = Partition.FreeBodies[K];
		Problems[K + 1].DofCount = DofPerBody * static_cast<int>(Partition.FreeBodies[K].size());
	}
	const std::vector<InterfaceId> Interfaces = Model.LiveInterfaces();
	for (Problem& P : Problems)
	{
		P.AppliedLoad = Eigen::VectorXd::Zero(P.DofCount);
		for (const BodyId Id : P.FreeBodies)
		{
			const BodyDesc& Body = Model.Body(Id);
			const Vec3 WeightN = Scale(Body.MassKg, Options.Gravity);
			AddForceAt(P.AppliedLoad, OffsetOfSlot[Id.Index], Body.CentroidM, Body.CentroidM, WeightN);
			P.ForceScaleN += Length(WeightN);
		}
	}
	for (const BodyLoad& Load : Loads)
	{
		const int Offset = OffsetOfSlot[Load.Body.Index];
		if (Offset >= 0)
		{
			Problem& P = Problems[static_cast<std::size_t>(Partition.ComponentOfSlot[Load.Body.Index] + 1)];
			AddForceAt(P.AppliedLoad, Offset, Model.Body(Load.Body).CentroidM, Load.PointM, Load.ForceN);
			P.ForceScaleN += Length(Load.ForceN);
			P.bHostLoaded = P.bHostLoaded || Load.ForceN.X != 0.0 || Load.ForceN.Y != 0.0 || Load.ForceN.Z != 0.0;
		}
	}
	for (std::size_t Slot = 0; Slot < Interfaces.size(); ++Slot)
	{
		const InterfaceRecord& Record = Model.Interface(Interfaces[Slot]);
		const int Component = statics::ComponentOf(Partition, Record);
		Problem& P = Problems[static_cast<std::size_t>(Component + 1)];
		P.Joints.push_back(MakeJoint(Model, Interfaces[Slot], OffsetOfSlot));
		P.ReadingSlots.push_back(Slot);
		P.bGrounded = P.bGrounded || P.Joints.back().OffsetA < 0 || P.Joints.back().OffsetB < 0;
	}
	for (Problem& P : Problems)
	{
		P.Scaling = RotationalScaling(P);
	}
	return Problems;
}

Eigen::Matrix3d CornerStiffness(const Joint& J, const JointState& State, std::size_t C)
{
	const Eigen::Vector3d N(J.Normal.X, J.Normal.Y, J.Normal.Z);
	const Eigen::Matrix3d NormalPart = N * N.transpose();
	if (State.Status[C] == CornerStatus::Open)
	{
		return Eigen::Matrix3d::Zero();
	}
	if (State.Status[C] == CornerStatus::Slipping)
	{
		const double HeldSizeN = Length(State.HeldN[C]);
		if (!(HeldSizeN > 0.0))
		{
			return J.NormalStiffnessNPerM * NormalPart;
		}
		const Eigen::Vector3d S = Eigen::Vector3d(State.HeldN[C].X, State.HeldN[C].Y, State.HeldN[C].Z) / HeldSizeN;
		return J.NormalStiffnessNPerM * NormalPart + State.TransverseNPerM[C] * (Eigen::Matrix3d::Identity() - NormalPart - S * S.transpose());
	}
	return J.NormalStiffnessNPerM * NormalPart + J.TangentialStiffnessNPerM * (Eigen::Matrix3d::Identity() - NormalPart);
}

template <class AddBlock>
void ForEachStiffnessBlock(const Problem& P, const std::vector<JointState>& States, AddBlock&& Add)
{
	for (std::size_t I = 0; I < P.Joints.size(); ++I)
	{
		const Joint& J = P.Joints[I];
		for (std::size_t C = 0; C < CornerCount; ++C)
		{
			const Eigen::Matrix3d Kc = CornerStiffness(J, States[I], C);
			const std::array<int, 2> Offsets{J.OffsetA, J.OffsetB};
			const std::array<double, 2> Signs{-1.0, 1.0};
			const std::array<Eigen::Matrix<double, 3, 6>, 2> G{CornerJacobian(Sub(J.CornerM[C], J.CentroidAM)), CornerJacobian(Sub(J.CornerM[C], J.CentroidBM))};
			for (std::size_t R = 0; R < 2; ++R)
			{
				for (std::size_t S = 0; S < 2; ++S)
				{
					if (Offsets[R] < 0 || Offsets[S] < 0)
					{
						continue;
					}
					const Eigen::Matrix<double, 6, 6> Block = (Signs[R] * Signs[S]) * (G[R].transpose() * Kc * G[S]);
					Add(Offsets[R], Offsets[S], Block);
				}
			}
		}
	}
}

Eigen::MatrixXd AssembleStiffness(const Problem& P, const std::vector<JointState>& States)
{
	Eigen::MatrixXd K = Eigen::MatrixXd::Zero(P.DofCount, P.DofCount);
	ForEachStiffnessBlock(P, States, [&](int Row, int Col, const Eigen::Matrix<double, 6, 6>& Block) { K.block<6, 6>(Row, Col) += Block; });
	return K;
}

using SparseMatrix = Eigen::SparseMatrix<double>;

bool IsScaledStiffnessFinite(const Problem& P, const std::vector<JointState>& States)
{
	using Block6 = Eigen::Matrix<double, 6, 6>;
	std::vector<Block6> Diagonal(static_cast<std::size_t>(P.DofCount / 6), Block6::Zero());
	std::unordered_map<std::int64_t, Block6> OffDiagonal;
	ForEachStiffnessBlock(P, States, [&](int Row, int Col, const Block6& Block) {
		if (Row == Col)
		{
			Diagonal[static_cast<std::size_t>(Row / 6)] += Block;
			return;
		}
		const std::int64_t Key = static_cast<std::int64_t>(Row) * P.DofCount + Col;
		OffDiagonal.try_emplace(Key, Block6::Zero()).first->second += Block;
	});
	const auto IsFinite = [&](int Row, int Col, const Block6& K) {
		for (int R = 0; R < 6; ++R)
		{
			for (int S = 0; S < 6; ++S)
			{
				if (!std::isfinite(P.Scaling[Row + R] * K(R, S) * P.Scaling[Col + S]))
				{
					return false;
				}
			}
		}
		return true;
	};
	for (std::size_t B = 0; B < Diagonal.size(); ++B)
	{
		if (!IsFinite(static_cast<int>(B) * 6, static_cast<int>(B) * 6, Diagonal[B]))
		{
			return false;
		}
	}
	for (const auto& [Key, K] : OffDiagonal)
	{
		if (!IsFinite(static_cast<int>(Key / P.DofCount), static_cast<int>(Key % P.DofCount), K))
		{
			return false;
		}
	}
	return true;
}

SparseMatrix AssembleScaledSparseStiffness(const Problem& P, const std::vector<JointState>& States)
{
	std::vector<Eigen::Triplet<double>> Triplets;
	ForEachStiffnessBlock(P, States, [&](int Row, int Col, const Eigen::Matrix<double, 6, 6>& Block) {
		for (int R = 0; R < 6; ++R)
		{
			for (int S = 0; S < 6; ++S)
			{
				Triplets.emplace_back(Row + R, Col + S, P.Scaling[Row + R] * Block(R, S) * P.Scaling[Col + S]);
			}
		}
	});
	SparseMatrix K(P.DofCount, P.DofCount);
	K.setFromTriplets(Triplets.begin(), Triplets.end());
	return K;
}

Eigen::VectorXd AssembleLoad(const Problem& P, const std::vector<JointState>& States)
{
	Eigen::VectorXd Load = P.AppliedLoad;
	for (std::size_t I = 0; I < P.Joints.size(); ++I)
	{
		const Joint& J = P.Joints[I];
		for (std::size_t C = 0; C < CornerCount; ++C)
		{
			if (States[I].Status[C] == CornerStatus::Stuck && !IsExactlyZero(States[I].ReferenceM[C]))
			{
				const Vec3 PreForceN = Scale(J.TangentialStiffnessNPerM, States[I].ReferenceM[C]);
				AddForceAt(Load, J.OffsetB, J.CentroidBM, J.CornerM[C], PreForceN);
				AddForceAt(Load, J.OffsetA, J.CentroidAM, J.CornerM[C], Scale(-1.0, PreForceN));
			}
			if (States[I].Status[C] != CornerStatus::Slipping)
			{
				continue;
			}
			AddForceAt(Load, J.OffsetB, J.CentroidBM, J.CornerM[C], States[I].HeldN[C]);
			AddForceAt(Load, J.OffsetA, J.CentroidAM, J.CornerM[C], Scale(-1.0, States[I].HeldN[C]));
		}
	}
	return Load;
}

Factorisation FactoriseBlocked(const Eigen::MatrixXd& K, double PivotToleranceRelative)
{
	constexpr Eigen::Index PanelWidth = 64;
	const Eigen::Index N = K.rows();
	Factorisation F;
	F.Lower = K;
	F.Order = IndexVector::LinSpaced(N, 0, N - 1);
	Eigen::VectorXd Diagonal = K.diagonal();
	Eigen::VectorXd Scaled(PanelWidth);
	double LargestPivot = 0.0;
	for (Eigen::Index Start = 0; Start < N; Start += PanelWidth)
	{
		const Eigen::Index Width = std::min(PanelWidth, N - Start);
		for (Eigen::Index Step = Start; Step < Start + Width; ++Step)
		{
			Eigen::Index Pick = 0;
			Diagonal.tail(N - Step).maxCoeff(&Pick);
			Pick += Step;
			const double Pivot = Diagonal(Pick);
			if (!(Pivot > PivotToleranceRelative * LargestPivot))
			{
				return F;
			}
			LargestPivot = std::max(LargestPivot, Pivot);
			F.Lower.row(Step).swap(F.Lower.row(Pick));
			F.Lower.col(Step).swap(F.Lower.col(Pick));
			std::swap(Diagonal(Step), Diagonal(Pick));
			std::swap(F.Order(Step), F.Order(Pick));
			const Eigen::Index Done = Step - Start;
			const Eigen::Index Rest = N - Step - 1;
			if (Done > 0)
			{
				Scaled.head(Done) = F.Lower.diagonal().segment(Start, Done).cwiseProduct(F.Lower.row(Step).segment(Start, Done).transpose());
				F.Lower.col(Step).tail(Rest).noalias() -= F.Lower.block(Step + 1, Start, Rest, Done) * Scaled.head(Done);
			}
			F.Lower(Step, Step) = Pivot;
			F.Lower.col(Step).tail(Rest) /= Pivot;
			Diagonal.tail(Rest) -= Pivot * F.Lower.col(Step).tail(Rest).cwiseAbs2();
			F.Rank = Step + 1;
		}
		const Eigen::Index Next = Start + Width;
		const Eigen::Index Rest = N - Next;
		if (Rest > 0)
		{
			const auto Panel = F.Lower.block(Next, Start, Rest, Width);
			const Eigen::MatrixXd PanelD = Panel * F.Lower.diagonal().segment(Start, Width).asDiagonal();
			F.Lower.bottomRightCorner(Rest, Rest).noalias() -= PanelD * Panel.transpose();
			Diagonal.tail(Rest) = F.Lower.diagonal().tail(Rest);
		}
	}
	return F;
}

Eigen::VectorXd Scatter(const Factorisation& F, const Eigen::VectorXd& Permuted)
{
	Eigen::VectorXd Original = Eigen::VectorXd::Zero(F.Order.size());
	for (Eigen::Index K = 0; K < Permuted.size(); ++K)
	{
		Original(F.Order(K)) = Permuted(K);
	}
	return Original;
}

Eigen::VectorXd SolveInRange(const Factorisation& F, const Eigen::VectorXd& Load)
{
	const Eigen::Index R = F.Rank;
	const auto L11 = F.Lower.topLeftCorner(R, R).triangularView<Eigen::UnitLower>();
	Eigen::VectorXd Head(R);
	for (Eigen::Index K = 0; K < R; ++K)
	{
		Head(K) = Load(F.Order(K));
	}
	Head = L11.solve(Head);
	Head = Head.cwiseQuotient(F.Lower.diagonal().head(R));
	Head = L11.transpose().solve(Head);
	return Scatter(F, Head);
}

std::vector<Eigen::VectorXd> NullBasis(const Factorisation& F)
{
	const Eigen::Index N = F.Order.size();
	const Eigen::Index R = F.Rank;
	const auto L11 = F.Lower.topLeftCorner(R, R).triangularView<Eigen::UnitLower>();
	std::vector<Eigen::VectorXd> Basis;
	for (Eigen::Index J = R; J < N; ++J)
	{
		Eigen::VectorXd Z = Eigen::VectorXd::Zero(N);
		Z(J) = 1.0;
		Z.head(R) = -L11.transpose().solve(F.Lower.row(J).head(R).transpose());
		Basis.push_back(Scatter(F, Z));
	}
	return Basis;
}

Eigen::VectorXd ProjectOntoSpan(std::vector<Eigen::VectorXd> Basis, const Eigen::VectorXd& Load)
{
	std::vector<Eigen::VectorXd> Orthonormal;
	for (Eigen::VectorXd& V : Basis)
	{
		for (int Pass = 0; Pass < 2; ++Pass)
		{
			for (const Eigen::VectorXd& Q : Orthonormal)
			{
				V -= Q.dot(V) * Q;
			}
		}
		const double Size = V.norm();
		if (Size > 0.0)
		{
			Orthonormal.push_back(V / Size);
		}
	}
	Eigen::VectorXd Projection = Eigen::VectorXd::Zero(Load.size());
	for (const Eigen::VectorXd& Q : Orthonormal)
	{
		Projection += Q.dot(Load) * Q;
	}
	return Projection;
}

Vec3 DisplacementAt(const Eigen::VectorXd& X, int Offset, const Vec3& ArmM)
{
	if (Offset < 0)
	{
		return Vec3{};
	}
	const Vec3 TranslationM{X(Offset + 0), X(Offset + 1), X(Offset + 2)};
	const Vec3 RotationRad{X(Offset + 3), X(Offset + 4), X(Offset + 5)};
	return Add(TranslationM, Cross(RotationRad, ArmM));
}

Vec3 RelativeDisplacement(const Joint& J, std::size_t C, const Eigen::VectorXd& X)
{
	return Sub(DisplacementAt(X, J.OffsetB, Sub(J.CornerM[C], J.CentroidBM)), DisplacementAt(X, J.OffsetA, Sub(J.CornerM[C], J.CentroidAM)));
}

TrialForces Trial(const Joint& J, std::size_t C, const Eigen::VectorXd& X)
{
	const Vec3 Delta = RelativeDisplacement(J, C, X);
	TrialForces T;
	T.GapM = Dot(Delta, J.Normal);
	T.NormalN = -J.NormalStiffnessNPerM * T.GapM;
	T.ShearN = Scale(-J.TangentialStiffnessNPerM, Sub(Delta, Scale(T.GapM, J.Normal)));
	return T;
}

double SlipCapacityN(const Joint& J, double NormalN)
{
	return std::min(J.Friction * PositivePart(NormalN), J.ShearCapN);
}

Vec3 HeldForce(const Vec3& TrialShearN, double CapacityN)
{
	const double Size = Length(TrialShearN);
	if (!(Size > 0.0))
	{
		return Vec3{};
	}
	return Scale(CapacityN / Size, TrialShearN);
}

double TransverseStiffnessNPerM(const Joint& J, const Vec3& TrialShearN, double CapacityN)
{
	const double Size = Length(TrialShearN);
	if (!(Size > 0.0))
	{
		return 0.0;
	}
	return J.TangentialStiffnessNPerM * CapacityN / Size;
}

double MotionScaleM(const Problem& P, const Eigen::VectorXd& Motion)
{
	double ScaleM = 0.0;
	for (const Joint& J : P.Joints)
	{
		for (std::size_t C = 0; C < CornerCount; ++C)
		{
			ScaleM = std::max(ScaleM, Length(RelativeDisplacement(J, C, Motion)));
		}
	}
	return ScaleM;
}

bool IsGapIndeterminate(const Problem& P, const StandingOptions& Options, const Joint& J, std::size_t C, const std::vector<Eigen::VectorXd>& NeutralMotions)
{
	return std::any_of(NeutralMotions.begin(), NeutralMotions.end(), [&](const Eigen::VectorXd& Motion) {
		return std::abs(Dot(RelativeDisplacement(J, C, Motion), J.Normal)) > Options.AdmissibilityToleranceRelative * MotionScaleM(P, Motion);
	});
}

void ReEnterSlipping(const Joint& J, std::size_t C, const Vec3& TangentialMotionM, double FullRankNormalN, JointState& State)
{
	State.Status[C] = CornerStatus::Slipping;
	State.HeldN[C] = HeldForce(Scale(-1.0, TangentialMotionM), SlipCapacityN(J, FullRankNormalN));
	State.TransverseNPerM[C] = 0.0;
}

void ResetReferenceOnReclosure(const Joint& J, std::size_t C, const TrialForces& T, JointState& State)
{
	State.ReclosureCount[C] += 1;
	if (State.ReclosureCount[C] < 2)
	{
		return;
	}
	State.ReferenceM[C] = Scale(-1.0 / J.TangentialStiffnessNPerM, T.ShearN);
}

double OpeningFloorN(const Joint& J, const StandingOptions& Options, double ForceScaleN)
{
	return (J.bReleasesSlip ? Options.DryNoiseFloorRelative : Options.OpenTensionToleranceRelative) * ForceScaleN;
}

UpdateOutcome UpdateActiveSet(const Problem& P, const StandingOptions& Options, const Eigen::VectorXd& X, const std::vector<Eigen::VectorXd>& NeutralMotions, std::vector<JointState>& States)
{
	UpdateOutcome Outcome;
	const double HeldToleranceN = Options.HeldForceToleranceRelative * P.ForceScaleN;
	if (!std::isfinite(P.ForceScaleN))
	{
		Outcome.bUndecided = true;
		return Outcome;
	}
	const double NoiseFloorN = Options.DryNoiseFloorRelative * P.ForceScaleN;
	const double CloseToleranceN = Options.CloseCompressionToleranceRelative * P.ForceScaleN;
	for (std::size_t I = 0; I < P.Joints.size(); ++I)
	{
		const Joint& J = P.Joints[I];
		JointState& State = States[I];
		for (std::size_t C = 0; C < CornerCount; ++C)
		{
			const TrialForces T = Trial(J, C, X);
			CornerStatus& Status = State.Status[C];
			Vec3& HeldN = State.HeldN[C];
			double& TransverseNPerM = State.TransverseNPerM[C];
			const bool bDry = J.bReleasesOpening && J.bReleasesSlip;
			if (Status == CornerStatus::Open)
			{
				const bool bLoaded = bDry ? T.NormalN > Options.DryRecloseFactor * NoiseFloorN : T.NormalN > CloseToleranceN;
				if (bLoaded && !IsGapIndeterminate(P, Options, J, C, NeutralMotions))
				{
					const double ClosingCapacityN = SlipCapacityN(J, T.NormalN);
					if (J.bReleasesSlip && Length(T.ShearN) > ClosingCapacityN)
					{
						ReEnterSlipping(J, C, Scale(-1.0, T.ShearN), T.NormalN, State);
						TransverseNPerM = TransverseStiffnessNPerM(J, T.ShearN, ClosingCapacityN);
					}
					else
					{
						Status = CornerStatus::Stuck;
						if (!J.bReleasesSlip)
						{
							ResetReferenceOnReclosure(J, C, T, State);
						}
					}
					Outcome.bStatusChanged = true;
				}
				continue;
			}
			if ((J.bReleasesOpening && T.NormalN < -OpeningFloorN(J, Options, P.ForceScaleN)) || (bDry && std::abs(T.NormalN) <= NoiseFloorN))
			{
				Status = CornerStatus::Open;
				HeldN = Vec3{};
				Outcome.bStatusChanged = true;
				continue;
			}
			if (!J.bReleasesSlip)
			{
				continue;
			}
			const double CapacityN = SlipCapacityN(J, T.NormalN);
			const double ShearN = Length(T.ShearN);
			if (Status == CornerStatus::Stuck)
			{
				if (ShearN > CapacityN)
				{
					Status = CornerStatus::Slipping;
					HeldN = HeldForce(T.ShearN, CapacityN);
					TransverseNPerM = TransverseStiffnessNPerM(J, T.ShearN, CapacityN);
					Outcome.bStatusChanged = true;
				}
				continue;
			}
			if (ShearN < CapacityN * (1.0 - Options.StickMarginRelative))
			{
				Status = CornerStatus::Stuck;
				HeldN = Vec3{};
				Outcome.bStatusChanged = true;
				continue;
			}
			const Vec3 NextHeldN = HeldForce(T.ShearN, CapacityN);
			if (!(Length(Sub(NextHeldN, HeldN)) <= HeldToleranceN))
			{
				Outcome.bHeldSettled = false;
			}
			HeldN = NextHeldN;
			TransverseNPerM = TransverseStiffnessNPerM(J, T.ShearN, CapacityN);
		}
	}
	return Outcome;
}

Restoration RestoreInadmissibleReleases(const Problem& P, const StandingOptions& Options, const Eigen::VectorXd& Motion, const Eigen::VectorXd* FullRankX, std::vector<JointState>& States)
{
	const double ScaleM = MotionScaleM(P, Motion);
	const double ToleranceM = Options.AdmissibilityToleranceRelative * ScaleM;
	const double ClosingM = Options.RestoreClosingRateThreshold * ScaleM;
	Restoration Outcome = Restoration::None;
	for (std::size_t I = 0; I < P.Joints.size(); ++I)
	{
		const Joint& J = P.Joints[I];
		JointState& State = States[I];
		for (std::size_t C = 0; C < CornerCount; ++C)
		{
			const Vec3 Delta = RelativeDisplacement(J, C, Motion);
			const double GapM = Dot(Delta, J.Normal);
			const Vec3 TangentialM = Sub(Delta, Scale(GapM, J.Normal));
			const double HeldSizeN = Length(State.HeldN[C]);
			const bool bCloses = State.Status[C] == CornerStatus::Open && GapM < -ToleranceM && -GapM > ClosingM;
			const bool bSlidesWithFriction = State.Status[C] == CornerStatus::Slipping && HeldSizeN > 0.0 &&
				Dot(TangentialM, State.HeldN[C]) > ToleranceM * HeldSizeN;
			if (bCloses)
			{
				if (J.bReleasesSlip)
				{
					ReEnterSlipping(J, C, TangentialM, FullRankX == nullptr ? 0.0 : Trial(J, C, *FullRankX).NormalN, State);
				}
				else
				{
					State.Status[C] = CornerStatus::Stuck;
					State.HeldN[C] = Vec3{};
				}
				Outcome = Restoration::Restored;
			}
			else if (bSlidesWithFriction)
			{
				State.Status[C] = CornerStatus::Stuck;
				State.HeldN[C] = Vec3{};
				Outcome = Restoration::Restored;
			}
		}
	}
	return Outcome;
}

double CohesiveDissipationW(const Problem& P, const StandingOptions& Options, const std::vector<JointState>& States, const Eigen::VectorXd& Motion)
{
	const double ToleranceM = Options.AdmissibilityToleranceRelative * MotionScaleM(P, Motion);
	double DissipationW = 0.0;
	for (std::size_t I = 0; I < P.Joints.size(); ++I)
	{
		const Joint& J = P.Joints[I];
		if (J.bReleasesSlip)
		{
			continue;
		}
		for (std::size_t C = 0; C < CornerCount; ++C)
		{
			if (States[I].Status[C] != CornerStatus::Open)
			{
				continue;
			}
			const Vec3 Delta = RelativeDisplacement(J, C, Motion);
			const double SlideM = Length(Sub(Delta, Scale(Dot(Delta, J.Normal), J.Normal)));
			if (SlideM > ToleranceM)
			{
				DissipationW += std::min(J.CohesionCapacityN, J.ShearCapN) * SlideM;
			}
		}
	}
	return DissipationW;
}

double Ratio(double DemandN, double CapacityN)
{
	if (DemandN == 0.0)
	{
		return 0.0;
	}
	const double Value = DemandN / CapacityN;
	return IsFinite(Value) ? Value : Infinity;
}

CornerRatios RatiosOf(const Joint& J, const CornerReading& Corner)
{
	const double CompressionN = PositivePart(Corner.NormalForceN);
	const double TensionN = PositivePart(-Corner.NormalForceN);
	CornerRatios Ratios;
	Ratios.Compression = Ratio(CompressionN, J.CompressiveCapacityN);
	Ratios.Tension = Ratio(TensionN, J.TensileCapacityN);
	Ratios.Shear = Ratio(Length(Corner.ShearForceN), std::min(J.CohesionCapacityN + J.Friction * CompressionN, J.ShearCapN));
	return Ratios;
}

bool IsOverstressed(const Joint& J, const CornerRatios& Ratios)
{
	const bool bNonFinite = !IsFinite(Ratios.Compression) || !IsFinite(Ratios.Tension) || !IsFinite(Ratios.Shear);
	const bool bCrushed = !(Ratios.Compression <= 1.0);
	const bool bTornBond = !J.bReleasesOpening && !(Ratios.Tension <= 1.0);
	const bool bShearedBond = !J.bReleasesSlip && !(Ratios.Shear <= 1.0);
	return bNonFinite || bCrushed || bTornBond || bShearedBond;
}

void Raise(double Candidate, double& AxisUtilisation)
{
	if (Candidate > AxisUtilisation)
	{
		AxisUtilisation = Candidate;
	}
}

void Govern(double Candidate, GoverningAxis Axis, InterfaceReading& Reading)
{
	if (Candidate > Reading.Utilisation)
	{
		Reading.Utilisation = Candidate;
		Reading.Axis = Axis;
	}
}

InterfaceReading ReadJoint(const Joint& J, const JointState& State, const Eigen::VectorXd& X, double NoiseFloorN, bool& bOverstressed)
{
	InterfaceReading Reading;
	Reading.Id = J.Id;
	for (std::size_t C = 0; C < CornerCount; ++C)
	{
		CornerReading& Corner = Reading.Corners[C];
		Corner.PointM = J.CornerM[C];
		Corner.bOpen = State.Status[C] == CornerStatus::Open;
		Corner.bSlipping = State.Status[C] == CornerStatus::Slipping;
		if (!Corner.bOpen)
		{
			const TrialForces T = Trial(J, C, X);
			const bool bNoiseTension = J.bReleasesOpening && std::isfinite(NoiseFloorN) && T.NormalN < 0.0 && T.NormalN >= -NoiseFloorN;
			Corner.NormalForceN = bNoiseTension ? 0.0 : T.NormalN;
			Corner.ShearForceN = Corner.bSlipping ? State.HeldN[C] : IsExactlyZero(State.ReferenceM[C]) ? T.ShearN : Add(T.ShearN, Scale(J.TangentialStiffnessNPerM, State.ReferenceM[C]));
		}
		const Vec3 ForceN = Add(Scale(Corner.NormalForceN, J.Normal), Corner.ShearForceN);
		Reading.NormalForceN += Corner.NormalForceN;
		Reading.ShearForceN = Add(Reading.ShearForceN, Corner.ShearForceN);
		Reading.MomentNm = Add(Reading.MomentNm, Cross(Sub(Corner.PointM, J.CentreM), ForceN));
		const CornerRatios Ratios = RatiosOf(J, Corner);
		Govern(Ratios.Compression, GoverningAxis::Compression, Reading);
		Govern(Ratios.Tension, GoverningAxis::Tension, Reading);
		Govern(Ratios.Shear, GoverningAxis::Shear, Reading);
		Raise(Ratios.Compression, Reading.CompressionUtilisation);
		Raise(Ratios.Tension, Reading.TensionUtilisation);
		Raise(Ratios.Shear, Reading.ShearUtilisation);
		bOverstressed = bOverstressed || IsOverstressed(J, Ratios);
	}
	return Reading;
}

std::vector<InterfaceReading> ReadAll(const Problem& P, const StandingOptions& Options, const std::vector<JointState>& States, const Eigen::VectorXd& X, bool& bOverstressed, PhaseTimings& Timings)
{
	const profile::ScopedPhase Readout(Timings.ReadoutSeconds);
	std::vector<InterfaceReading> Readings;
	for (std::size_t I = 0; I < P.Joints.size(); ++I)
	{
		Readings.push_back(ReadJoint(P.Joints[I], States[I], X, OpeningFloorN(P.Joints[I], Options, P.ForceScaleN), bOverstressed));
	}
	return Readings;
}

void FillMechanism(const Problem& P, const StandingOptions& Options, const Eigen::VectorXd& Motion, StandingResult& Result)
{
	const double Largest = Motion.cwiseAbs().maxCoeff();
	const Eigen::VectorXd Unit = Motion / Largest;
	for (std::size_t K = 0; K < P.FreeBodies.size(); ++K)
	{
		const int O = static_cast<int>(K) * DofPerBody;
		if (!(Unit.segment(O, DofPerBody).cwiseAbs().maxCoeff() > Options.UnconstrainedComponent))
		{
			continue;
		}
		Result.UnconstrainedBodies.push_back(P.FreeBodies[K]);
		Result.Mechanism.push_back(MechanismMotion{P.FreeBodies[K], Vec3{Unit(O + 0), Unit(O + 1), Unit(O + 2)}, Vec3{Unit(O + 3), Unit(O + 4), Unit(O + 5)}});
	}
}

bool IsAllFinite(const Eigen::MatrixXd& M)
{
	return std::all_of(M.data(), M.data() + M.size(), [](double Value) { return std::isfinite(Value); });
}

bool IsAllFinite(const SparseMatrix& M)
{
	return std::all_of(M.valuePtr(), M.valuePtr() + M.nonZeros(), [](double Value) { return std::isfinite(Value); });
}

constexpr int InverseIterationCount = 5;

using SparseFactor = Eigen::SimplicialLDLT<SparseMatrix, Eigen::Lower, Eigen::AMDOrdering<int>>;

double SmallestEigenvalueEstimate(const SparseMatrix& K, const SparseFactor& Factor)
{
	std::minstd_rand Generator(1);
	Eigen::VectorXd V(K.rows());
	for (Eigen::Index I = 0; I < V.size(); ++I)
	{
		V[I] = (Generator() & 1u) != 0 ? 1.0 : -1.0;
	}
	V.normalize();
	for (int Step = 0; Step < InverseIterationCount; ++Step)
	{
		V = Factor.solve(V);
		const double Norm = V.norm();
		if (!(Norm > 0.0) || !std::isfinite(Norm))
		{
			return 0.0;
		}
		V /= Norm;
	}
	return V.dot(K * V);
}

struct SparseAttempt
{
	bool bNonFinite = false;
	std::optional<Eigen::VectorXd> ScaledX;
};

SparseAttempt SolveSparseFullRank(const Problem& P, const std::vector<JointState>& States, const Eigen::VectorXd& Load, double PivotToleranceRelative, double SparseAcceptanceMargin,
	PhaseTimings& Timings)
{
	SparseAttempt Attempt;
	const SparseMatrix K = profile::Timed(Timings.AssemblySeconds, [&] { return AssembleScaledSparseStiffness(P, States); });
	if (!IsAllFinite(K))
	{
		Attempt.bNonFinite = true;
		return Attempt;
	}
	SparseFactor Factor;
	profile::AnalyseAndFactorise(Factor, K, Timings);
	if (Factor.info() != Eigen::Success)
	{
		return Attempt;
	}
	const Eigen::VectorXd Pivots = Factor.vectorD();
	const double LargestPivot = Pivots.cwiseAbs().maxCoeff();
	const bool bFullRank = std::all_of(Pivots.data(), Pivots.data() + Pivots.size(), [&](double Pivot) {
		return std::isfinite(Pivot) && Pivot > PivotToleranceRelative * LargestPivot;
	});
	if (bFullRank && profile::Timed(Timings.CheckSeconds, [&] { return SmallestEigenvalueEstimate(K, Factor); }) > SparseAcceptanceMargin * PivotToleranceRelative * LargestPivot)
	{
		Attempt.ScaledX = profile::Timed(Timings.SolveSeconds, [&]() -> Eigen::VectorXd { return Factor.solve(Load); });
	}
	return Attempt;
}

int Severity(StandingState State)
{
	switch (State)
	{
	case StandingState::Stands: return 0;
	case StandingState::Overstressed: return 1;
	case StandingState::Mechanism: return 2;
	case StandingState::Undecided: return 3;
	case StandingState::InvalidInput: return 4;
	}
	return 4;
}

enum class StepOutcome : std::uint8_t
{
	Continue,
	Finished
};

bool IsRecording(const StandingOptions& Options)
{
	return Options.Debug != nullptr && Options.Debug->bRecordPasses;
}

CornerState StateOf(CornerStatus Status)
{
	switch (Status)
	{
	case CornerStatus::Stuck: return CornerState::Closed;
	case CornerStatus::Slipping: return CornerState::Slipping;
	case CornerStatus::Open: return CornerState::Open;
	}
	return CornerState::Closed;
}

void RecordPass(const Problem& P, const StandingOptions& Options, const std::vector<JointState>& Solved, const std::vector<JointState>& After, const Eigen::VectorXd& X, int Iteration, std::vector<ElasticPassRecord>& Passes)
{
	const ElasticPassRecord* Previous = Passes.empty() ? nullptr : &Passes.back();
	ElasticPassRecord Pass;
	Pass.Iteration = Iteration;
	for (std::size_t I = 0; I < P.Joints.size(); ++I)
	{
		bool bIgnored = false;
		const InterfaceReading Reading = ReadJoint(P.Joints[I], Solved[I], X, OpeningFloorN(P.Joints[I], Options, P.ForceScaleN), bIgnored);
		JointPassRecord Record;
		Record.Id = Reading.Id;
		for (std::size_t C = 0; C < CornerCount; ++C)
		{
			Record.Corners[C] = StateOf(After[I].Status[C]);
			Record.bFlipped[C] = Previous != nullptr && Previous->Joints[I].Corners[C] != Record.Corners[C];
			Record.CornerNormalN[C] = Reading.Corners[C].NormalForceN;
			Record.CornerShearN[C] = Reading.Corners[C].ShearForceN;
		}
		Record.NormalForceN = Reading.NormalForceN;
		Record.ShearForceN = Reading.ShearForceN;
		Record.MomentNm = Reading.MomentNm;
		Record.Utilisation = Reading.Utilisation;
		Record.Axis = Reading.Axis;
		Pass.Joints.push_back(std::move(Record));
	}
	Passes.push_back(std::move(Pass));
}

StepOutcome AdvanceActiveSet(const Problem& P, const StandingOptions& Options, const Eigen::VectorXd& X, const std::vector<Eigen::VectorXd>& NeutralMotions, bool& bOverstressed, std::vector<JointState>& States, StandingResult& Result, PhaseTimings& Timings, int Iteration, Ending& E)
{
	std::vector<JointState> Next = States;
	const UpdateOutcome Outcome = UpdateActiveSet(P, Options, X, NeutralMotions, Next);
	if (IsRecording(Options))
	{
		RecordPass(P, Options, States, Next, X, Iteration, E.Passes);
	}
	if (Outcome.bUndecided)
	{
		Result.Readings = ReadAll(P, Options, States, X, bOverstressed, Timings);
		Result.State = StandingState::Undecided;
		return StepOutcome::Finished;
	}
	if (!Outcome.bStatusChanged && Outcome.bHeldSettled)
	{
		Result.Readings = ReadAll(P, Options, States, X, bOverstressed, Timings);
		Result.State = bOverstressed ? StandingState::Overstressed : StandingState::Stands;
		E.End = SolveEnd::Converged;
		return StepOutcome::Finished;
	}
	States = std::move(Next);
	return StepOutcome::Continue;
}

StandingResult FallRigid(const Problem& P, const StandingOptions& Options)
{
	StandingResult Result;
	Result.State = StandingState::Mechanism;
	const double Largest = std::max({std::abs(Options.Gravity.X), std::abs(Options.Gravity.Y), std::abs(Options.Gravity.Z)});
	const Vec3 Fall{Options.Gravity.X / Largest, Options.Gravity.Y / Largest, Options.Gravity.Z / Largest};
	for (const BodyId Id : P.FreeBodies)
	{
		Result.UnconstrainedBodies.push_back(Id);
		Result.Mechanism.push_back(MechanismMotion{Id, Fall, Vec3{}});
	}
	for (const Joint& J : P.Joints)
	{
		InterfaceReading Reading;
		Reading.Id = J.Id;
		for (std::size_t C = 0; C < CornerCount; ++C)
		{
			Reading.Corners[C].PointM = J.CornerM[C];
		}
		Result.Readings.push_back(Reading);
	}
	return Result;
}

struct SoftSpikeSwitches
{
	double Relative = 0.0;
	bool bAllSizes = false;
	double Band = 1e-6;
	double NeutralBand = 2e-13;
	double NeutralFloor = 0.0;
	double GrayLow = 0.0;
	double GrayHigh = 0.0;
	int NullPasses = 4;
	int MaxPasses = 60;
	int Ritz = 8;
	bool bPivotRange = false;
	int MaxRefines = 10;
	double PassTolerance = 1e-14;
	double StallRatio = 0.5;
	const char* Log = nullptr;
	const char* Diag = nullptr;
	std::vector<double> DiagRelatives;
	const char* EndLog = nullptr;
};

const SoftSpikeSwitches& SoftSpike()
{
	static const SoftSpikeSwitches Switches = [] {
		SoftSpikeSwitches S;
		if (const char* V = std::getenv("LOADPATH_SOFT_SINGULAR"))
		{
			S.Relative = std::atof(V);
		}
		S.bAllSizes = std::getenv("LOADPATH_SOFT_ALL_SIZES") != nullptr;
		if (const char* V = std::getenv("LOADPATH_SOFT_BAND"))
		{
			S.Band = std::atof(V);
		}
		if (const char* V = std::getenv("LOADPATH_SOFT_PASSES"))
		{
			S.MaxPasses = std::atoi(V);
		}
		if (const char* V = std::getenv("LOADPATH_SOFT_REFINES"))
		{
			S.MaxRefines = std::atoi(V);
		}
		if (const char* V = std::getenv("LOADPATH_SOFT_PASS_TOL"))
		{
			S.PassTolerance = std::atof(V);
		}
		if (const char* V = std::getenv("LOADPATH_SOFT_STALL"))
		{
			S.StallRatio = std::atof(V);
		}
		if (const char* V = std::getenv("LOADPATH_SOFT_RITZ"))
		{
			S.Ritz = std::atoi(V);
		}
		if (const char* V = std::getenv("LOADPATH_SOFT_RANGE"))
		{
			S.bPivotRange = std::string(V) == "pivots";
		}
		if (const char* V = std::getenv("LOADPATH_SOFT_NEUTRAL_BAND"))
		{
			S.NeutralBand = std::atof(V);
		}
		if (const char* V = std::getenv("LOADPATH_NEUTRAL_FLOOR"))
		{
			S.NeutralFloor = std::atof(V);
			if (S.NeutralFloor > 0.0 && std::getenv("LOADPATH_SOFT_NEUTRAL_BAND") == nullptr)
			{
				S.NeutralBand = S.NeutralFloor;
			}
		}
		if (const char* V = std::getenv("LOADPATH_SOFT_NULL_PASSES"))
		{
			S.NullPasses = std::atoi(V);
		}
		if (const char* V = std::getenv("LOADPATH_SOFT_GRAY_LOW"))
		{
			S.GrayLow = std::atof(V);
		}
		if (const char* V = std::getenv("LOADPATH_SOFT_GRAY_HIGH"))
		{
			S.GrayHigh = std::atof(V);
		}
		S.Log = std::getenv("LOADPATH_SOFT_LOG");
		S.Diag = std::getenv("LOADPATH_SOFT_DIAG");
		S.EndLog = std::getenv("LOADPATH_END_LOG");
		if (const char* V = std::getenv("LOADPATH_SOFT_DIAG_EPS"))
		{
			std::string Text(V);
			std::size_t Start = 0;
			while (Start < Text.size())
			{
				const std::size_t Comma = Text.find(',', Start);
				S.DiagRelatives.push_back(std::atof(Text.substr(Start, Comma - Start).c_str()));
				if (Comma == std::string::npos)
				{
					break;
				}
				Start = Comma + 1;
			}
		}
		return S;
	}();
	return Switches;
}

void AppendLine(const char* Path, const std::string& Line)
{
	if (Path == nullptr)
	{
		return;
	}
	static std::mutex Lock;
	const std::lock_guard<std::mutex> Guard(Lock);
	if (std::FILE* F = std::fopen(Path, "a"))
	{
		std::fputs(Line.c_str(), F);
		std::fclose(F);
	}
}

struct SoftSingular
{
	bool bFactored = false;
	Eigen::VectorXd Projection;
	Eigen::VectorXd ScaledX;
	int Passes = 0;
	int Refines = 0;
	int NullRitz = 0;
	int Pinned = -1;
	bool bGray = false;
	double ProjectionChange = 0.0;
	double Residual = 0.0;
	double Seconds = 0.0;
};

SoftSingular SolveSoftSingular(const SparseMatrix& Ks, const Eigen::VectorXd& Load, double Relative, int MaxPasses, int MaxRefines, double PassTolerance, double StallRatio, double PivotToleranceRelative)
{
	SoftSingular Out;
	const auto Start = std::chrono::steady_clock::now();
	const double Largest = Ks.diagonal().maxCoeff();
	const double Shift = Relative * Largest;
	if (!(Shift > 0.0) || !std::isfinite(Shift))
	{
		return Out;
	}
	SparseMatrix A = Ks;
	for (Eigen::Index I = 0; I < A.rows(); ++I)
	{
		A.coeffRef(I, I) += Shift;
	}
	SparseFactor Factor;
	Factor.compute(A);
	if (Factor.info() != Eigen::Success)
	{
		return Out;
	}
	Out.bFactored = true;
	const double LoadNorm = Load.norm();
	if (!(LoadNorm > 0.0) || !std::isfinite(LoadNorm))
	{
		Out.bFactored = false;
		return Out;
	}
	Eigen::VectorXd V = Load;
	for (int Pass = 1; SoftSpike().Ritz == 0 && Pass <= MaxPasses; ++Pass)
	{
		const Eigen::VectorXd Next = Shift * Factor.solve(V);
		const double Change = (Next - V).norm() / LoadNorm;
		const bool bStalled = Pass >= 3 && Change > StallRatio * Out.ProjectionChange;
		Out.ProjectionChange = Change;
		V = Next;
		Out.Passes = Pass;
		if (Change <= PassTolerance || bStalled)
		{
			break;
		}
	}
	const int RitzSize = SoftSpike().Ritz;
	if (RitzSize > 0)
	{
		Eigen::MatrixXd Q(Load.size(), RitzSize + 1);
		Eigen::Index Columns = 0;
		Eigen::VectorXd W = Load / LoadNorm;
		for (int Step = 0; Step <= RitzSize; ++Step)
		{
			for (int Sweep = 0; Sweep < 2; ++Sweep)
			{
				W -= Q.leftCols(Columns) * (Q.leftCols(Columns).transpose() * W);
			}
			const double Size = W.norm();
			if (!(Size > 1e-14) || !std::isfinite(Size))
			{
				break;
			}
			Q.col(Columns++) = W / Size;
			if (Step < RitzSize)
			{
				W = Factor.solve(Eigen::VectorXd(Q.col(Columns - 1)));
				Out.Passes = Step + 1;
			}
		}
		const Eigen::MatrixXd Basis = Q.leftCols(Columns);
		const Eigen::MatrixXd H = Basis.transpose() * (Ks * Basis);
		const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> Solver(0.5 * (H + H.transpose()));
		V = Eigen::VectorXd::Zero(Load.size());
		for (Eigen::Index I = 0; I < Columns; ++I)
		{
			const double Theta = Solver.eigenvalues()(I);
			if (Theta > SoftSpike().GrayLow * Largest && Theta < SoftSpike().GrayHigh * Largest)
			{
				Out.bGray = true;
			}
		}
		for (Eigen::Index I = 0; I < Columns; ++I)
		{
			if (Solver.eigenvalues()(I) < PivotToleranceRelative * Largest)
			{
				const Eigen::VectorXd U = Basis * Solver.eigenvectors().col(I);
				V += U.dot(Load) * U;
				++Out.NullRitz;
			}
		}
	}
	Out.Projection = V;
	const Eigen::VectorXd Balanced = Load - V;
	Eigen::VectorXd X = Factor.solve(Balanced);
	for (int Refine = 1; Refine <= MaxRefines; ++Refine)
	{
		const Eigen::VectorXd R = Balanced - Ks * X;
		Out.Residual = R.norm() / LoadNorm;
		if (Out.Residual <= 1e-13)
		{
			break;
		}
		X += Factor.solve(R);
		Out.Refines = Refine;
	}
	Out.Residual = (Balanced - Ks * X).norm() / LoadNorm;
	if (SoftSpike().bPivotRange)
	{
		SparseFactor Plain;
		Plain.compute(Ks);
		if (Plain.info() != Eigen::Success)
		{
			Out.bFactored = false;
			return Out;
		}
		const Eigen::VectorXd Pivots = Plain.vectorD();
		const double LargestPivot = Pivots.cwiseAbs().maxCoeff();
		std::vector<bool> bFree(static_cast<std::size_t>(Ks.rows()), false);
		Out.Pinned = 0;
		for (Eigen::Index K = 0; K < Pivots.size(); ++K)
		{
			if (!(Pivots(K) > PivotToleranceRelative * LargestPivot))
			{
				bFree[static_cast<std::size_t>(Plain.permutationPinv().indices()(K))] = true;
				++Out.Pinned;
			}
		}
		std::vector<Eigen::Triplet<double>> Triplets;
		for (Eigen::Index Col = 0; Col < Ks.outerSize(); ++Col)
		{
			for (SparseMatrix::InnerIterator It(Ks, Col); It; ++It)
			{
				if (!bFree[static_cast<std::size_t>(It.row())] && !bFree[static_cast<std::size_t>(It.col())])
				{
					Triplets.emplace_back(static_cast<int>(It.row()), static_cast<int>(It.col()), It.value());
				}
			}
		}
		for (Eigen::Index I = 0; I < Ks.rows(); ++I)
		{
			if (bFree[static_cast<std::size_t>(I)])
			{
				Triplets.emplace_back(static_cast<int>(I), static_cast<int>(I), Largest);
			}
		}
		SparseMatrix Reduced(Ks.rows(), Ks.cols());
		Reduced.setFromTriplets(Triplets.begin(), Triplets.end());
		SparseFactor Kept;
		Kept.compute(Reduced);
		if (Kept.info() != Eigen::Success)
		{
			Out.bFactored = false;
			return Out;
		}
		Eigen::VectorXd Head = Load;
		for (Eigen::Index I = 0; I < Head.size(); ++I)
		{
			if (bFree[static_cast<std::size_t>(I)])
			{
				Head(I) = 0.0;
			}
		}
		X = Kept.solve(Head);
		for (int Refine = 0; Refine < 2; ++Refine)
		{
			X += Kept.solve(Head - Reduced * X);
		}
	}
	Out.ScaledX = X;
	if (!Out.Projection.allFinite() || !Out.ScaledX.allFinite() || Out.bGray)
	{
		Out.bFactored = false;
	}
	Out.Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
	return Out;
}

std::vector<Eigen::VectorXd> SoftNullBasis(const SparseMatrix& Ks, double Relative, double PivotToleranceRelative, int Passes)
{
	const Eigen::Index N = Ks.rows();
	const double Largest = Ks.diagonal().maxCoeff();
	const double Shift = Relative * Largest;
	if (!(Shift > 0.0) || !std::isfinite(Shift))
	{
		return {};
	}
	SparseMatrix A = Ks;
	for (Eigen::Index I = 0; I < N; ++I)
	{
		A.coeffRef(I, I) += Shift;
	}
	SparseFactor Factor;
	Factor.compute(A);
	if (Factor.info() != Eigen::Success)
	{
		return {};
	}
	std::minstd_rand Generator(7);
	for (Eigen::Index Block = 4;; Block *= 2)
	{
		const Eigen::Index Start = std::min(Block, N);
		const Eigen::Index Capacity = std::min<Eigen::Index>(Start * (Passes + 1), N);
		Eigen::MatrixXd Q(N, Capacity);
		Eigen::Index Columns = 0;
		const auto Append = [&](Eigen::VectorXd W) {
			if (Columns >= Capacity)
			{
				return false;
			}
			const double Before = W.norm();
			for (int Sweep = 0; Sweep < 2; ++Sweep)
			{
				W -= Q.leftCols(Columns) * (Q.leftCols(Columns).transpose() * W);
			}
			const double Size = W.norm();
			if (!(Size > 1e-10 * Before) || !std::isfinite(Size))
			{
				return false;
			}
			Q.col(Columns++) = W / Size;
			return true;
		};
		std::vector<Eigen::Index> Current;
		for (Eigen::Index J = 0; J < Start; ++J)
		{
			Eigen::VectorXd W(N);
			for (Eigen::Index I = 0; I < N; ++I)
			{
				W(I) = (Generator() & 1u) != 0 ? 1.0 : -1.0;
			}
			if (Append(W))
			{
				Current.push_back(Columns - 1);
			}
		}
		for (int Pass = 0; Pass < Passes; ++Pass)
		{
			std::vector<Eigen::Index> Next;
			for (const Eigen::Index C : Current)
			{
				if (Append(Factor.solve(Eigen::VectorXd(Q.col(C)))))
				{
					Next.push_back(Columns - 1);
				}
			}
			Current = Next;
		}
		const Eigen::MatrixXd Basis = Q.leftCols(Columns);
		const Eigen::MatrixXd H = Basis.transpose() * (Ks * Basis);
		const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> Solver(0.5 * (H + H.transpose()));
		std::vector<Eigen::VectorXd> Null;
		for (Eigen::Index I = 0; I < Columns; ++I)
		{
			const double Theta = Solver.eigenvalues()(I);
			if (Theta > SoftSpike().GrayLow * Largest && Theta < SoftSpike().GrayHigh * Largest)
			{
				return {};
			}
		}
		for (Eigen::Index I = 0; I < Columns; ++I)
		{
			if (Solver.eigenvalues()(I) < PivotToleranceRelative * Largest)
			{
				Null.push_back(Basis * Solver.eigenvectors().col(I));
			}
		}
		if (static_cast<Eigen::Index>(Null.size()) < Start || Start >= N)
		{
			return Null;
		}
	}
}

void DiagnoseSoft(const Problem& P, const std::vector<JointState>& States, const Eigen::MatrixXd& K, const Factorisation& F, const std::vector<Eigen::VectorXd>& Basis, const Eigen::VectorXd& Load, const Eigen::VectorXd& Projection, int Iteration, double PivotToleranceRelative)
{
	const SoftSpikeSwitches& S = SoftSpike();
	const SparseMatrix Ks = AssembleScaledSparseStiffness(P, States);
	std::vector<Eigen::VectorXd> Orthonormal;
	for (Eigen::VectorXd V : Basis)
	{
		for (int Pass = 0; Pass < 2; ++Pass)
		{
			for (const Eigen::VectorXd& Q : Orthonormal)
			{
				V -= Q.dot(V) * Q;
			}
		}
		if (V.norm() > 0.0)
		{
			Orthonormal.push_back(V.normalized());
		}
	}
	const auto StripNull = [&](Eigen::VectorXd V) {
		for (const Eigen::VectorXd& Q : Orthonormal)
		{
			V -= Q.dot(V) * Q;
		}
		return V;
	};
	const double LoadNorm = Load.norm();
	const Eigen::VectorXd DenseX = SolveInRange(F, Load);
	const Eigen::VectorXd ExactX = StripNull(SolveInRange(F, Load - Projection));
	const double ProjectionNorm = Projection.norm();
	for (const double Relative : S.DiagRelatives)
	{
		const SoftSingular Soft = SolveSoftSingular(Ks, Load, Relative, S.MaxPasses, S.MaxRefines, S.PassTolerance, S.StallRatio, PivotToleranceRelative);
		if (!Soft.bFactored)
		{
			AppendLine(S.Diag, std::to_string(P.DofCount) + "," + std::to_string(Iteration) + ",nofactor\n");
			continue;
		}
		char Line[512];
		std::snprintf(Line, sizeof(Line), "%d,%d,%d,%d,%.1e,%d,%d,%.3e,%.3e,%.3e,%.3e,%.3e,%.3e,%.3e,%.4f,%.3e,%.3e,%d,%.3e\n", P.DofCount, Iteration, static_cast<int>(F.Rank), static_cast<int>(P.DofCount - F.Rank), Relative,
			Soft.Passes, Soft.Refines, ProjectionNorm / LoadNorm, (Soft.Projection - Projection).norm() / std::max(ProjectionNorm, 1e-300), (StripNull(Soft.ScaledX) - ExactX).norm() / ExactX.norm(),
			(K * (Soft.ScaledX - DenseX)).norm() / LoadNorm, Soft.Residual, Soft.ProjectionChange, StripNull(Soft.Projection).norm() / LoadNorm, Soft.Seconds, (Ks * Projection).norm() / (Ks.diagonal().mean() * std::max(ProjectionNorm, 1e-300)), (Ks * Soft.Projection).norm() / (Ks.diagonal().mean() * std::max(Soft.Projection.norm(), 1e-300)), Soft.Pinned, (Soft.ScaledX - DenseX).norm() / DenseX.norm());
		AppendLine(S.Diag, Line);
	}
}

StandingResult SolveProblem(const Problem& P, const StandingOptions& Options, PhaseTimings& Timings, Ending& E)
{
	StandingResult Result;
	std::vector<JointState> States(P.Joints.size());
	Eigen::VectorXd X = Eigen::VectorXd::Zero(P.DofCount);
	bool bOverstressed = false;
	if (P.DofCount == 0)
	{
		Result.Readings = ReadAll(P, Options, States, X, bOverstressed, Timings);
		Result.State = bOverstressed ? StandingState::Overstressed : StandingState::Stands;
		E.End = SolveEnd::Converged;
		return Result;
	}
	const bool bGravity = Options.Gravity.X != 0.0 || Options.Gravity.Y != 0.0 || Options.Gravity.Z != 0.0;
	if (!P.bGrounded && !P.bHostLoaded && bGravity)
	{
		if (!profile::Timed(Timings.AssemblySeconds, [&] { return IsScaledStiffnessFinite(P, States); }))
		{
			Result.State = StandingState::InvalidInput;
			E.End = SolveEnd::InvalidInput;
			return Result;
		}
		E.End = SolveEnd::Mechanism;
		return profile::Timed(Timings.ReadoutSeconds, [&] { return FallRigid(P, Options); });
	}
	const bool bRecord = IsRecording(Options);
	std::vector<JointState> FullRankStates;
	Eigen::VectorXd FullRankX;
	bool bHaveFullRank = false;
	const auto D = P.Scaling.asDiagonal();
	const SoftSpikeSwitches& Spike = SoftSpike();
	const double SoftRelative = Options.SingularSoftSpringRelative > 0.0 ? Options.SingularSoftSpringRelative : Spike.Relative;
	const bool bDiag = Spike.Diag != nullptr && !Spike.DiagRelatives.empty();
	const bool bSoft = SoftRelative > 0.0 && !bDiag;
	const auto Resolve = [&](const Eigen::VectorXd& Projection, const Eigen::VectorXd& ScaledX, int Iteration) -> bool {
		const Eigen::VectorXd Motion = D * Projection;
		X = D * ScaledX;
		const std::vector<JointState> Solved = bRecord ? States : std::vector<JointState>{};
		const Restoration Outcome = RestoreInadmissibleReleases(P, Options, Motion, bHaveFullRank ? &FullRankX : nullptr, States);
		if (bRecord)
		{
			RecordPass(P, Options, Solved, States, X, Iteration, E.Passes);
		}
		if (Outcome == Restoration::None)
		{
			const double PowerW = AssembleLoad(P, States).dot(Motion);
			if (!(PowerW > CohesiveDissipationW(P, Options, States, Motion)))
			{
				Result.Readings = bHaveFullRank ? ReadAll(P, Options, FullRankStates, FullRankX, bOverstressed, Timings) : ReadAll(P, Options, States, X, bOverstressed, Timings);
				Result.State = StandingState::Undecided;
				E.End = SolveEnd::PowerCheckFailed;
				return true;
			}
			Result.Readings = ReadAll(P, Options, States, X, bOverstressed, Timings);
			Result.State = StandingState::Mechanism;
			profile::Timed(Timings.ReadoutSeconds, [&] { FillMechanism(P, Options, Motion, Result); });
			E.End = SolveEnd::Mechanism;
			return true;
		}
		return false;
	};
	const auto TrySoft = [&](const Eigen::VectorXd& Load, int Iteration, std::vector<Eigen::VectorXd>& NeutralMotions, std::optional<bool>& Finished) {
		const SparseMatrix Ks = profile::Timed(Timings.AssemblySeconds, [&] { return AssembleScaledSparseStiffness(P, States); });
		const SoftSingular Soft = profile::Timed(Timings.FactorizationSeconds, [&] { return SolveSoftSingular(Ks, Load, SoftRelative, Spike.MaxPasses, Spike.MaxRefines, Spike.PassTolerance, Spike.StallRatio, Options.PivotToleranceRelative); });
		const bool bUse = Soft.bFactored && !(Soft.Projection.norm() <= Spike.Band * Load.norm());
		const bool bNeutral = Soft.bFactored && Soft.Projection.norm() <= Spike.NeutralBand * Load.norm();
		std::vector<Eigen::VectorXd> Null;
		double NullSeconds = 0.0;
		if (bNeutral)
		{
			const auto Begin = std::chrono::steady_clock::now();
			Null = profile::Timed(Timings.FactorizationSeconds, [&] { return SoftNullBasis(Ks, SoftRelative, Options.PivotToleranceRelative, Spike.NullPasses); });
			NullSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Begin).count();
		}
		if (Spike.Log != nullptr)
		{
			char Line[256];
			std::snprintf(Line, sizeof(Line), "%d,%d,%s,%d,%d,%.3e,%.3e,%.4f,%d,%.4f\n", P.DofCount, Iteration, bUse ? "soft" : (bNeutral && !Null.empty() ? "neutral" : "fallback"), Soft.Passes, Soft.Refines,
				Soft.bFactored ? Soft.Projection.norm() / Load.norm() : -1.0, Soft.Residual, Soft.Seconds, static_cast<int>(Null.size()), NullSeconds);
			AppendLine(Spike.Log, Line);
		}
		if (bUse)
		{
			Finished = Resolve(Soft.Projection, Soft.ScaledX, Iteration);
		}
		else if (bNeutral && !Null.empty())
		{
			for (const Eigen::VectorXd& V : Null)
			{
				NeutralMotions.push_back(D * V);
			}
			X = D * Soft.ScaledX;
			Finished = AdvanceActiveSet(P, Options, X, NeutralMotions, bOverstressed, States, Result, Timings, Iteration, E) == StepOutcome::Finished;
		}
	};
	for (int Iteration = 1; Iteration <= Options.MaxActiveSetIterations; ++Iteration)
	{
		Result.ActiveSetIterations = Iteration;
		const Eigen::VectorXd Load = profile::Timed(Timings.AssemblySeconds, [&]() -> Eigen::VectorXd { return D * AssembleLoad(P, States); });
		std::vector<Eigen::VectorXd> NeutralMotions;
		if (P.DofCount > Options.DenseDofLimit)
		{
			const SparseAttempt Attempt = SolveSparseFullRank(P, States, Load, Options.PivotToleranceRelative, Options.SparseAcceptanceMargin, Timings);
			if (Iteration == 1 && Attempt.bNonFinite)
			{
				Result.State = StandingState::InvalidInput;
				E.End = SolveEnd::InvalidInput;
				return Result;
			}
			if (Attempt.ScaledX)
			{
				X = D * *Attempt.ScaledX;
				FullRankStates = States;
				FullRankX = X;
				bHaveFullRank = true;
				if (AdvanceActiveSet(P, Options, X, NeutralMotions, bOverstressed, States, Result, Timings, Iteration, E) == StepOutcome::Finished)
				{
					return Result;
				}
				continue;
			}
			if (bSoft)
			{
				std::optional<bool> Finished;
				TrySoft(Load, Iteration, NeutralMotions, Finished);
				if (Finished)
				{
					if (*Finished)
					{
						return Result;
					}
					continue;
				}
			}
		}
		const Eigen::MatrixXd K = profile::Timed(Timings.AssemblySeconds, [&]() -> Eigen::MatrixXd { return D * AssembleStiffness(P, States) * D; });
		if (Iteration == 1 && !IsAllFinite(K))
		{
			Result.State = StandingState::InvalidInput;
			E.End = SolveEnd::InvalidInput;
			return Result;
		}
		const Factorisation F = profile::Timed(Timings.FactorizationSeconds, [&] { return FactoriseBlocked(K, Options.PivotToleranceRelative); });
		if (F.Rank < P.DofCount)
		{
			if (bSoft && Spike.bAllSizes && P.DofCount <= Options.DenseDofLimit)
			{
				std::optional<bool> Finished;
				TrySoft(Load, Iteration, NeutralMotions, Finished);
				if (Finished)
				{
					if (*Finished)
					{
						return Result;
					}
					continue;
				}
			}
			const std::vector<Eigen::VectorXd> Basis = profile::Timed(Timings.SolveSeconds, [&] { return NullBasis(F); });
			const Eigen::VectorXd Projection = profile::Timed(Timings.SolveSeconds, [&] { return ProjectOntoSpan(Basis, Load); });
			if (bDiag)
			{
				DiagnoseSoft(P, States, K, F, Basis, Load, Projection, Iteration, Options.PivotToleranceRelative);
			}
			if (!(Projection.norm() <= (Spike.NeutralFloor > 0.0 ? Spike.NeutralFloor : Options.NeutralLoadToleranceRelative) * Load.norm()))
			{
				if (Resolve(Projection, SolveInRange(F, Load), Iteration))
				{
					return Result;
				}
				continue;
			}
			for (const Eigen::VectorXd& V : Basis)
			{
				NeutralMotions.push_back(D * V);
			}
		}
		X = profile::Timed(Timings.SolveSeconds, [&]() -> Eigen::VectorXd { return D * SolveInRange(F, Load); });
		if (F.Rank == P.DofCount)
		{
			FullRankStates = States;
			FullRankX = X;
			bHaveFullRank = true;
		}
		if (AdvanceActiveSet(P, Options, X, NeutralMotions, bOverstressed, States, Result, Timings, Iteration, E) == StepOutcome::Finished)
		{
			return Result;
		}
	}
	Result.Readings = ReadAll(P, Options, States, X, bOverstressed, Timings);
	Result.State = StandingState::Undecided;
	E.End = SolveEnd::IterationCap;
	return Result;
}

bool ChooseComponents(const Structure& Model, std::span<const BodyId> Only, std::vector<bool>& Chosen)
{
	if (Only.empty())
	{
		return true;
	}
	const ComponentPartition& Partition = Model.Components();
	Chosen.assign(Partition.FreeBodies.size(), false);
	for (const BodyId Id : Only)
	{
		if (!Model.IsLive(Id) || Model.Body(Id).bGrounded)
		{
			return false;
		}
		Chosen[static_cast<std::size_t>(Partition.ComponentOfSlot[Id.Index])] = true;
	}
	return true;
}

bool IsIncluded(const std::vector<bool>& Chosen, std::size_t ProblemIndex)
{
	return Chosen.empty() || ProblemIndex == 0 || Chosen[ProblemIndex - 1];
}

StandingResult SolveComponents(const std::vector<Problem>& Problems, std::size_t InterfaceCount, const StandingOptions& Options, const std::vector<bool>& Chosen, const IParallelFor* Workers)
{
	std::vector<bool> bSlotIncluded(InterfaceCount, false);
	std::vector<std::size_t> Work;
	for (std::size_t I = 0; I < Problems.size(); ++I)
	{
		if (IsIncluded(Chosen, I))
		{
			Work.push_back(I);
			for (const std::size_t Slot : Problems[I].ReadingSlots)
			{
				bSlotIncluded[Slot] = true;
			}
		}
	}
	std::vector<std::size_t> LocalOfSlot(InterfaceCount, 0);
	std::size_t ReadingCount = 0;
	for (std::size_t Slot = 0; Slot < InterfaceCount; ++Slot)
	{
		if (bSlotIncluded[Slot])
		{
			LocalOfSlot[Slot] = ReadingCount++;
		}
	}
	std::vector<StandingResult> Solved(Work.size());
	std::vector<Ending> Endings(Work.size());
	const std::function<void(std::size_t)> SolveOne = [&](std::size_t K) {
		PhaseTimings Timings;
		Solved[K] = profile::Timed(Timings.TotalSeconds, [&] { return SolveProblem(Problems[Work[K]], Options, Timings, Endings[K]); });
		Solved[K].Timings = Timings;
		if (SoftSpike().EndLog != nullptr)
		{
			char Line[128];
			std::snprintf(Line, sizeof(Line), "%d,%d,%d,%d,%.4f\n", Problems[Work[K]].DofCount, static_cast<int>(Endings[K].End), static_cast<int>(Solved[K].State), Solved[K].ActiveSetIterations, Timings.TotalSeconds);
			AppendLine(SoftSpike().EndLog, Line);
		}
	};
	if (Workers != nullptr)
	{
		Workers->For(Work.size(), SolveOne);
	}
	else
	{
		for (std::size_t K = 0; K < Work.size(); ++K)
		{
			SolveOne(K);
		}
	}
	StandingResult Result;
	Result.State = StandingState::Stands;
	Result.Readings.resize(ReadingCount);
	for (std::size_t K = 0; K < Work.size(); ++K)
	{
		const Problem& P = Problems[Work[K]];
		StandingResult& Component = Solved[K];
		if (Component.State == StandingState::InvalidInput)
		{
			StandingResult Invalid;
			Invalid.State = StandingState::InvalidInput;
			return Invalid;
		}
		if (Severity(Component.State) > Severity(Result.State))
		{
			Result.State = Component.State;
		}
		std::vector<std::size_t> Slots(P.ReadingSlots.size());
		for (std::size_t R = 0; R < P.ReadingSlots.size(); ++R)
		{
			Slots[R] = LocalOfSlot[P.ReadingSlots[R]];
			Result.Readings[Slots[R]] = Component.Readings[R];
		}
		Result.Mechanism.insert(Result.Mechanism.end(), Component.Mechanism.begin(), Component.Mechanism.end());
		Result.UnconstrainedBodies.insert(Result.UnconstrainedBodies.end(), Component.UnconstrainedBodies.begin(), Component.UnconstrainedBodies.end());
		Result.ActiveSetIterations = std::max(Result.ActiveSetIterations, Component.ActiveSetIterations);
		if (!P.FreeBodies.empty())
		{
			profile::Accumulate(Result.Timings, Component.Timings);
			StandingComponent Part{P.FreeBodies, Component.State, Component.ActiveSetIterations, std::move(Slots), Component.Timings};
			Part.Passes = std::move(Endings[K].Passes);
			Part.End = Endings[K].End;
			Result.Components.push_back(std::move(Part));
		}
	}
	return Result;
}

class RbsmStandingAuthority final : public IStandingAuthority
{
public:
	explicit RbsmStandingAuthority(const IParallelFor* InWorkers) : Workers(InWorkers) {}

	StandingResult Solve(const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options, std::span<const BodyId> Only) const override
	{
		std::vector<std::size_t> Stale;
		const std::vector<BodyLoad> Live = statics::WithoutStaleLoads(Model, Loads, Stale);
		StandingResult Result = SolveLive(Model, Live, Options, Only);
		Result.StaleLoads = std::move(Stale);
		return Result;
	}

private:
	StandingResult SolveLive(const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options, std::span<const BodyId> Only) const
	{
		InterfaceId Offending = InterfaceId::Invalid();
		std::vector<bool> Chosen;
		const bool bInvalid = FindInvalidInput(Model, Loads, Options, Offending);
		if (bInvalid || !ChooseComponents(Model, Only, Chosen))
		{
			StandingResult Result;
			Result.State = StandingState::InvalidInput;
			Result.InvalidInterface = bInvalid ? Offending : InterfaceId::Invalid();
			return Result;
		}
		return SolveComponents(BuildProblems(Model, Loads, Options), Model.LiveInterfaces().size(), Options, Chosen, Workers);
	}

	const IParallelFor* Workers = nullptr;
};
}

std::unique_ptr<IStandingAuthority> MakeRbsmStandingAuthority(const IParallelFor* Workers)
{
	return std::make_unique<RbsmStandingAuthority>(Workers);
}
}
