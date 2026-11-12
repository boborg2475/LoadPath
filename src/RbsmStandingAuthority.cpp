#include "loadpath/StandingAuthority.h"

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

#include "loadpath/Settings.h"

#include "EigenInclude.h"
#include "PhaseClock.h"
#include "StaticsCommon.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
	Restored,
	Undecided
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
	std::vector<InterfaceId> Blocking;
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

Restoration RestoreInadmissibleReleases(const Problem& P, const StandingOptions& Options, const Eigen::VectorXd& Motion, const std::vector<JointState>* FullRankStates, const Eigen::VectorXd& FullRankX, std::vector<JointState>& States, std::vector<InterfaceId>& Blocking)
{
	const double ToleranceM = Options.AdmissibilityToleranceRelative * MotionScaleM(P, Motion);
	Restoration Outcome = Restoration::None;
	bool bBlocked = false;
	for (std::size_t I = 0; I < P.Joints.size(); ++I)
	{
		const Joint& J = P.Joints[I];
		JointState& State = States[I];
		bool bListed = false;
		for (std::size_t C = 0; C < CornerCount; ++C)
		{
			const Vec3 Delta = RelativeDisplacement(J, C, Motion);
			const double GapM = Dot(Delta, J.Normal);
			const Vec3 TangentialM = Sub(Delta, Scale(GapM, J.Normal));
			const double HeldSizeN = Length(State.HeldN[C]);
			const bool bCloses = State.Status[C] == CornerStatus::Open && GapM < -ToleranceM;
			const bool bSlidesWithFriction = State.Status[C] == CornerStatus::Slipping && HeldSizeN > 0.0 &&
				Dot(TangentialM, State.HeldN[C]) > ToleranceM * HeldSizeN;
			if (bCloses)
			{
				if (FullRankStates == nullptr || (*FullRankStates)[I].Status[C] == CornerStatus::Open)
				{
					if (!bListed)
					{
						Blocking.push_back(J.Id);
						bListed = true;
					}
					bBlocked = true;
					continue;
				}
				if (bBlocked)
				{
					continue;
				}
				if (J.bReleasesSlip)
				{
					ReEnterSlipping(J, C, TangentialM, Trial(J, C, FullRankX).NormalN, State);
				}
				else
				{
					State.Status[C] = CornerStatus::Stuck;
					State.HeldN[C] = Vec3{};
				}
				Outcome = Restoration::Restored;
			}
			else if (bSlidesWithFriction && !bBlocked)
			{
				State.Status[C] = CornerStatus::Stuck;
				State.HeldN[C] = Vec3{};
				Outcome = Restoration::Restored;
			}
		}
	}
	return bBlocked ? Restoration::Undecided : Outcome;
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

struct SoftSprings
{
	double Relative = 0.0;
	double MechanismRatio = 0.0;
};

SoftSprings EffectiveSoftSprings(const StandingOptions& Options)
{
	SoftSprings Soft{Options.SoftSpringRelative, Options.SoftSpringMechanismRatio};
	if (const char* Value = std::getenv("LOADPATH_SOFT_SPRING"); Value != nullptr && Soft.Relative == 0.0)
	{
		Soft.Relative = std::atof(Value);
	}
	if (const char* Value = std::getenv("LOADPATH_SOFT_RATIO"); Value != nullptr)
	{
		Soft.MechanismRatio = std::atof(Value);
	}
	return Soft;
}

Eigen::VectorXd ScaledSoftDiagonal(const Problem& P, double Relative)
{
	Eigen::VectorXd Diagonal = Eigen::VectorXd::Zero(P.DofCount);
	const std::vector<JointState> Closed(P.Joints.size());
	ForEachStiffnessBlock(P, Closed, [&](int Row, int Col, const Eigen::Matrix<double, 6, 6>& Block) {
		if (Row == Col)
		{
			Diagonal.segment(Row, 6) += Block.diagonal();
		}
	});
	return Relative * Diagonal.cwiseProduct(P.Scaling).cwiseProduct(P.Scaling);
}

std::optional<Eigen::VectorXd> SolveSoft(const Problem& P, const StandingOptions& Options, const std::vector<JointState>& States, const Eigen::VectorXd& SoftDiagonal, const Eigen::VectorXd& Load, PhaseTimings& Timings)
{
	if (P.DofCount > Options.DenseDofLimit)
	{
		SparseMatrix K = profile::Timed(Timings.AssemblySeconds, [&] { return AssembleScaledSparseStiffness(P, States); });
		for (int I = 0; I < P.DofCount; ++I)
		{
			K.coeffRef(I, I) += SoftDiagonal[I];
		}
		if (!IsAllFinite(K))
		{
			return std::nullopt;
		}
		SparseFactor Factor;
		profile::AnalyseAndFactorise(Factor, K, Timings);
		if (Factor.info() != Eigen::Success)
		{
			return std::nullopt;
		}
		return profile::Timed(Timings.SolveSeconds, [&]() -> Eigen::VectorXd { return Factor.solve(Load); });
	}
	const auto D = P.Scaling.asDiagonal();
	Eigen::MatrixXd K = profile::Timed(Timings.AssemblySeconds, [&]() -> Eigen::MatrixXd { return D * AssembleStiffness(P, States) * D; });
	K.diagonal() += SoftDiagonal;
	if (!IsAllFinite(K))
	{
		return std::nullopt;
	}
	const Eigen::LDLT<Eigen::MatrixXd> Factor = profile::Timed(Timings.FactorizationSeconds, [&] { return Eigen::LDLT<Eigen::MatrixXd>(K); });
	if (Factor.info() != Eigen::Success)
	{
		return std::nullopt;
	}
	return profile::Timed(Timings.SolveSeconds, [&]() -> Eigen::VectorXd { return Factor.solve(Load); });
}

std::vector<double> SoftForceRatios(const Problem& P, const Eigen::VectorXd& SoftDiagonal, const Eigen::VectorXd& ScaledX)
{
	const double MeanLoadN = P.ForceScaleN / static_cast<double>(P.FreeBodies.size());
	std::vector<double> Ratios(P.FreeBodies.size());
	for (std::size_t K = 0; K < P.FreeBodies.size(); ++K)
	{
		const int O = static_cast<int>(K) * DofPerBody;
		Ratios[K] = SoftDiagonal.segment(O, DofPerBody).cwiseProduct(ScaledX.segment(O, DofPerBody)).norm() / MeanLoadN;
	}
	return Ratios;
}

void PrintSoftHistogram(const std::vector<double>& Ratios, double Threshold)
{
	if (std::getenv("LOADPATH_SOFT_HIST") == nullptr)
	{
		return;
	}
	std::array<int, 40> Bins{};
	int Above = 0;
	for (const double R : Ratios)
	{
		const int Bin = std::clamp(static_cast<int>(std::floor(std::log10(std::max(R, 1e-30)))) + 30, 0, 39);
		++Bins[static_cast<std::size_t>(Bin)];
		Above += R > Threshold ? 1 : 0;
	}
	std::fprintf(stderr, "SOFTHIST bodies=%zu above=%d threshold=%g :", Ratios.size(), Above, Threshold);
	for (std::size_t B = 0; B < Bins.size(); ++B)
	{
		if (Bins[B] != 0)
		{
			std::fprintf(stderr, " [1e%d,1e%d):%d", static_cast<int>(B) - 30, static_cast<int>(B) - 29, Bins[B]);
		}
	}
	std::fprintf(stderr, "\n");
	if (const char* Path = std::getenv("LOADPATH_SOFT_DUMP"); Path != nullptr)
	{
		if (std::FILE* F = std::fopen(Path, "a"))
		{
			for (std::size_t K = 0; K < Ratios.size(); ++K)
			{
				std::fprintf(F, "%zu,%.6e\n", K, Ratios[K]);
			}
			std::fclose(F);
		}
	}
}

StandingResult SolveSoftProblem(const Problem& P, const StandingOptions& Options, const SoftSprings& Soft, PhaseTimings& Timings, Ending& E)
{
	StandingResult Result;
	std::vector<JointState> States(P.Joints.size());
	Eigen::VectorXd X = Eigen::VectorXd::Zero(P.DofCount);
	bool bOverstressed = false;
	const auto D = P.Scaling.asDiagonal();
	const Eigen::VectorXd SoftDiagonal = ScaledSoftDiagonal(P, Soft.Relative);
	const std::vector<Eigen::VectorXd> NeutralMotions;
	for (int Iteration = 1; Iteration <= Options.MaxActiveSetIterations; ++Iteration)
	{
		Result.ActiveSetIterations = Iteration;
		const Eigen::VectorXd Load = profile::Timed(Timings.AssemblySeconds, [&]() -> Eigen::VectorXd { return D * AssembleLoad(P, States); });
		const std::optional<Eigen::VectorXd> ScaledX = SolveSoft(P, Options, States, SoftDiagonal, Load, Timings);
		if (!ScaledX)
		{
			Result.State = StandingState::InvalidInput;
			E.End = SolveEnd::InvalidInput;
			return Result;
		}
		X = D * *ScaledX;
		if (AdvanceActiveSet(P, Options, X, NeutralMotions, bOverstressed, States, Result, Timings, Iteration, E) == StepOutcome::Continue)
		{
			continue;
		}
		if (Result.State != StandingState::Stands && Result.State != StandingState::Overstressed)
		{
			return Result;
		}
		const std::vector<double> Ratios = SoftForceRatios(P, SoftDiagonal, *ScaledX);
		PrintSoftHistogram(Ratios, Soft.MechanismRatio);
		const double Largest = X.cwiseAbs().maxCoeff();
		for (std::size_t K = 0; K < P.FreeBodies.size(); ++K)
		{
			if (!(Ratios[K] > Soft.MechanismRatio))
			{
				continue;
			}
			const int O = static_cast<int>(K) * DofPerBody;
			Result.UnconstrainedBodies.push_back(P.FreeBodies[K]);
			Result.Mechanism.push_back(MechanismMotion{P.FreeBodies[K], Vec3{X(O + 0) / Largest, X(O + 1) / Largest, X(O + 2) / Largest}, Vec3{X(O + 3) / Largest, X(O + 4) / Largest, X(O + 5) / Largest}});
		}
		if (!Result.UnconstrainedBodies.empty())
		{
			Result.State = StandingState::Mechanism;
			E.End = SolveEnd::Mechanism;
		}
		return Result;
	}
	Result.Readings = ReadAll(P, Options, States, X, bOverstressed, Timings);
	Result.State = StandingState::Undecided;
	E.End = SolveEnd::IterationCap;
	return Result;
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
	if (const SoftSprings Soft = EffectiveSoftSprings(Options); Soft.Relative > 0.0)
	{
		return SolveSoftProblem(P, Options, Soft, Timings, E);
	}
	const bool bRecord = IsRecording(Options);
	std::vector<JointState> FullRankStates;
	Eigen::VectorXd FullRankX;
	bool bHaveFullRank = false;
	const auto D = P.Scaling.asDiagonal();
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
			const std::vector<Eigen::VectorXd> Basis = profile::Timed(Timings.SolveSeconds, [&] { return NullBasis(F); });
			const Eigen::VectorXd Projection = profile::Timed(Timings.SolveSeconds, [&] { return ProjectOntoSpan(Basis, Load); });
			if (!(Projection.norm() <= Options.NeutralLoadToleranceRelative * Load.norm()))
			{
				const Eigen::VectorXd Motion = D * Projection;
				X = profile::Timed(Timings.SolveSeconds, [&]() -> Eigen::VectorXd { return D * SolveInRange(F, Load); });
				const std::vector<JointState> Solved = bRecord ? States : std::vector<JointState>{};
				const Restoration Outcome = RestoreInadmissibleReleases(P, Options, Motion, bHaveFullRank ? &FullRankStates : nullptr, FullRankX, States, E.Blocking);
				if (bRecord)
				{
					RecordPass(P, Options, Solved, States, X, Iteration, E.Passes);
				}
				if (Outcome == Restoration::Undecided)
				{
					Result.Readings = bHaveFullRank ? ReadAll(P, Options, FullRankStates, FullRankX, bOverstressed, Timings) : ReadAll(P, Options, States, X, bOverstressed, Timings);
					Result.State = StandingState::Undecided;
					E.End = SolveEnd::RestoreFailedClosed;
					return Result;
				}
				if (Outcome == Restoration::None)
				{
					const double PowerW = AssembleLoad(P, States).dot(Motion);
					if (!(PowerW > CohesiveDissipationW(P, Options, States, Motion)))
					{
						Result.Readings = bHaveFullRank ? ReadAll(P, Options, FullRankStates, FullRankX, bOverstressed, Timings) : ReadAll(P, Options, States, X, bOverstressed, Timings);
						Result.State = StandingState::Undecided;
						E.End = SolveEnd::PowerCheckFailed;
						return Result;
					}
					Result.Readings = ReadAll(P, Options, States, X, bOverstressed, Timings);
					Result.State = StandingState::Mechanism;
					profile::Timed(Timings.ReadoutSeconds, [&] { FillMechanism(P, Options, Motion, Result); });
					E.End = SolveEnd::Mechanism;
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
			Part.Blocking = std::move(Endings[K].Blocking);
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
