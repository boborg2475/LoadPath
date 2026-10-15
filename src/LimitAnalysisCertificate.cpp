#include "loadpath/CollapseCertificate.h"

#include "CapSiteFactor.h"
#include "CertificateProbe.h"
#include "EigenInclude.h"
#include "StaticsCommon.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace loadpath
{
namespace
{
using statics::CornerCount;
using statics::DofPerBody;
using statics::Scale;
using statics::Sub;

constexpr double Pi = 3.14159265358979323846;
constexpr double StepToBoundary = 0.995;
constexpr int RefinementPasses = 1;
constexpr int SelectionRefinementPasses = 3;
constexpr int ShiftedRefinementPasses = 20;
constexpr std::array<double, 3> RetryShifts{1.0e-16, 1.0e-14, 1.0e-12};
constexpr double ComplementarityRestart = 0.5;
constexpr double UnconstrainedComponent = 1.0e-3;
constexpr double ObjectiveNorm = 1.0;
constexpr double Unbounded = std::numeric_limits<double>::infinity();
constexpr int FacetPivotLimit = 20000;
constexpr int FacetBlandAfter = 2000;
constexpr double UnitRoundoff = std::numeric_limits<double>::epsilon() / 2.0;
constexpr double LightBodyLoad = 1.0e-6;
constexpr std::array<double, 2> RecoveryMargins{1.0e-5, 1.0e-3};
constexpr double AtCapRelative = 1.0e-6;
constexpr double FailedInnerMinimumRelativeWork = 1.0e-6;
constexpr double FailedInnerMinimumMotionL1 = 0.5;

struct Site
{
	int OffsetA = -1;
	int OffsetB = -1;
	Vec3 ArmA;
	Vec3 ArmB;
};

struct JointData
{
	InterfaceId Id;
	Vec3 Normal;
	Vec3 AxisU;
	Vec3 AxisV;
	double Friction = 0.0;
	double TensionN = 0.0;
	double CohesionN = 0.0;
	double InnerTensionN = 0.0;
	double InnerCohesionN = 0.0;
	double CrushingN = 0.0;
	double ShearCapN = Unbounded;
	std::size_t FirstSite = 0;
};

struct Component
{
	int Rows = 0;
	std::vector<BodyId> FreeBodies;
	std::vector<JointData> Joints;
	std::vector<Site> Sites;
	Eigen::VectorXd Demand;
	double ForceScaleN = 1.0;
	double LengthM = 1.0;
	bool bHasBodyLoad = false;
};

struct Group
{
	std::size_t SiteIndex = 0;
	std::size_t First = 0;
	std::size_t Count = 0;
	double Bound = 0.0;
	int CapIndex = -1;
};

struct CapSite
{
	std::size_t SiteIndex = 0;
	std::vector<std::size_t> Groups;
	std::vector<Vec3> Facets;
	double Bound = 0.0;
	Eigen::Index FirstRow = 0;
	Eigen::Index FirstSlack = 0;
	Eigen::MatrixXd FacetMatrix;
	Eigen::MatrixXd DirectionMatrix;
	std::vector<Eigen::Index> DirectionGroup;
	std::vector<std::size_t> DirectionVariable;
};

struct Program
{
	std::vector<Group> Groups;
	std::vector<CapSite> CapSites;
	std::vector<Vec3> Directions;
	Eigen::VectorXd Offset;
	double Cap = 0.0;
	Eigen::Index LoadFactor = 0;
	Eigen::Index FirstSlack = 0;
	Eigen::Index CapSlack = 0;
	Eigen::Index Variables = 0;
	Eigen::Index FirstGroupRow = 0;
	Eigen::Index CapRow = 0;
	Eigen::Index FirstCapRow = 0;
	Eigen::Index FirstCapSlack = 0;
	Eigen::Index RowCount = 0;
};

struct BorderLastOrdering
{
	using PermutationType = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int>;

	template <typename MatrixType>
	void operator()(const MatrixType& Mat, PermutationType& Perm)
	{
		const Eigen::Index Sites = Mat.rows() - 1;
		const Eigen::SparseMatrix<double, Eigen::ColMajor, int> Block = Mat.topLeftCorner(Sites, Sites);
		PermutationType Inner;
		Eigen::AMDOrdering<int>()(Block, Inner);
		Perm.resize(Sites + 1);
		for (Eigen::Index I = 0; I < Sites; ++I)
		{
			Perm.indices()(I) = Inner.indices()(I);
		}
		Perm.indices()(Sites) = static_cast<int>(Sites);
	}
};

struct SparseNormal
{
	std::vector<Eigen::Triplet<double>> Triplets;
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower, BorderLastOrdering> BorderedFactor;
	Eigen::SparseMatrix<double> Bordered;
	std::vector<Eigen::Triplet<double>> BorderedTriplets;
	bool bAnalysed = false;
};

struct CapScratch
{
	Eigen::VectorXd GroupSlackD;
	Eigen::VectorXd FacetSlackD;
	Eigen::VectorXd DirectionD;
	Eigen::MatrixXd FT;
};

struct Normal
{
	SparseNormal* Sparse = nullptr;
	Eigen::VectorXd D;
	std::vector<Vec3> Z;
	std::vector<double> H;
	double Ht = 0.0;
	std::vector<Eigen::MatrixXd> CapF;
	std::vector<CapSiteFactor> CapStructured;
	std::vector<Eigen::MatrixXd> CapSolvedF;
	std::vector<CapScratch> CapScratchD;
	Eigen::LLT<Eigen::MatrixXd> Factor;
	bool bShifted = false;
	mutable std::vector<Eigen::MatrixXd> CapRhs;
	mutable std::vector<Eigen::MatrixXd> CapSolved;
	mutable std::vector<Eigen::VectorXd> CapCorrection;
};

struct Embedding
{
	Eigen::VectorXd X;
	Eigen::VectorXd S;
	Eigen::VectorXd Y;
	double Tau = 1.0;
	double Kappa = 1.0;
};

struct Outcome
{
	bool bConverged = false;
	double LoadFactor = 0.0;
	Eigen::VectorXd Y;
	int Iterations = 0;
	double DualBound = Unbounded;
	Eigen::VectorXd LastY;
	Eigen::VectorXd X;
	Eigen::VectorXd AllY;
};

struct ComponentVerdict
{
	CertificateState State = CertificateState::Undecided;
	double Lower = 0.0;
	double Upper = 0.0;
	int Iterations = 0;
	std::vector<MechanismMotion> Mechanism;
	std::vector<BodyId> Moving;
	std::vector<InterfaceId> Hinging;
};

bool IsPositiveFinite(double X)
{
	return X > 0.0 && IsFinite(X);
}

bool IsInputValid(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options)
{
	const bool bOptionsValid = statics::AreCertificateOptionsValid(Options);
	if (!bOptionsValid)
	{
		return false;
	}
	return std::all_of(Loads.begin(), Loads.end(), [&](const BodyLoad& Load) { return Model.IsLive(Load.Body) && IsFinite(Load.ForceN) && IsFinite(Load.PointM); });
}

void AddWrench(double* Out, int Offset, const Vec3& Arm, const Vec3& Force)
{
	if (Offset < 0)
	{
		return;
	}
	const Vec3 Moment = Cross(Arm, Force);
	double* Row = Out + Offset;
	Row[0] += Force.X;
	Row[1] += Force.Y;
	Row[2] += Force.Z;
	Row[3] += Moment.X;
	Row[4] += Moment.Y;
	Row[5] += Moment.Z;
}

void ApplyCornerForce(double* Out, const Site& S, const Vec3& ForceOnB)
{
	AddWrench(Out, S.OffsetB, S.ArmB, ForceOnB);
	AddWrench(Out, S.OffsetA, S.ArmA, Scale(-1.0, ForceOnB));
}

Vec3 PointVelocity(const double* Y, int Offset, const Vec3& Arm)
{
	if (Offset < 0)
	{
		return Vec3{};
	}
	const double* Row = Y + Offset;
	return statics::Add(Vec3{Row[0], Row[1], Row[2]}, Cross(Vec3{Row[3], Row[4], Row[5]}, Arm));
}

Vec3 RelativeVelocity(const Site& S, const double* Y)
{
	return Sub(PointVelocity(Y, S.OffsetB, S.ArmB), PointVelocity(Y, S.OffsetA, S.ArmA));
}

std::vector<Component> BuildComponents(const Structure& Model, std::span<const BodyLoad> Loads, const Vec3& Gravity, const std::vector<bool>& Chosen)
{
	const ComponentPartition& Partition = Model.Components();
	const auto IsChosen = [&](int K) { return Chosen.empty() || Chosen[static_cast<std::size_t>(K)]; };
	std::vector<Component> Components(Partition.FreeBodies.size());
	std::vector<std::vector<statics::JointGeometry>> Geometry(Components.size());
	for (std::size_t K = 0; K < Components.size(); ++K)
	{
		Components[K].FreeBodies = Partition.FreeBodies[K];
		Components[K].Rows = DofPerBody * static_cast<int>(Partition.FreeBodies[K].size());
	}
	for (const InterfaceId Id : Model.LiveInterfaces())
	{
		const InterfaceRecord& Record = Model.Interface(Id);
		const int K = statics::ComponentOf(Partition, Record);
		if (K < 0 || !IsChosen(K))
		{
			continue;
		}
		const StrengthProfile Profile = Model.EffectiveProfile(Id);
		const double CornerAreaM2 = Record.AreaM2 / 4.0;
		JointData J;
		J.Id = Id;
		J.Normal = Record.NormalAToB;
		J.AxisU = Record.AxisU;
		J.AxisV = Record.AxisV;
		J.Friction = Profile.FrictionCoefficient;
		J.TensionN = Profile.TensileStrengthPa * CornerAreaM2;
		J.CohesionN = Profile.ShearCohesionPa * CornerAreaM2;
		J.CrushingN = Profile.CompressiveStrengthPa * CornerAreaM2;
		J.ShearCapN = Profile.MaxShearStrengthPa * CornerAreaM2;
		const double InnerTensionPa = J.Friction > 0.0 ? std::min(Profile.TensileStrengthPa, Profile.ShearCohesionPa / J.Friction) : Profile.TensileStrengthPa;
		J.InnerTensionN = InnerTensionPa * CornerAreaM2;
		J.InnerCohesionN = (Profile.ShearCohesionPa - J.Friction * InnerTensionPa) * CornerAreaM2;
		Components[static_cast<std::size_t>(K)].Joints.push_back(J);
		Geometry[static_cast<std::size_t>(K)].push_back(statics::MakeJointGeometry(Model, Id, Partition.OffsetOfSlot));
	}
	std::vector<Eigen::VectorXd> Applied(Components.size());
	for (std::size_t K = 0; K < Components.size(); ++K)
	{
		if (!IsChosen(static_cast<int>(K)))
		{
			continue;
		}
		Component& C = Components[K];
		C.LengthM = statics::CharacteristicLengthM(Geometry[K]);
		Applied[K] = Eigen::VectorXd::Zero(C.Rows);
		C.ForceScaleN = 0.0;
		for (const BodyId Id : C.FreeBodies)
		{
			const Vec3 WeightN = Scale(Model.Body(Id).MassKg, Gravity);
			AddWrench(Applied[K].data(), Partition.OffsetOfSlot[Id.Index], Vec3{}, WeightN);
			C.ForceScaleN += Length(WeightN);
		}
	}
	for (const BodyLoad& Load : Loads)
	{
		const int K = Partition.ComponentOfSlot[Load.Body.Index];
		if (K < 0 || !IsChosen(K))
		{
			continue;
		}
		Component& C = Components[static_cast<std::size_t>(K)];
		const Vec3 Arm = Scale(1.0 / C.LengthM, Sub(Load.PointM, Model.Body(Load.Body).CentroidM));
		AddWrench(Applied[static_cast<std::size_t>(K)].data(), Partition.OffsetOfSlot[Load.Body.Index], Arm, Load.ForceN);
		C.ForceScaleN += Length(Load.ForceN);
		C.bHasBodyLoad = C.bHasBodyLoad || Length(Load.ForceN) > 0.0;
	}
	for (std::size_t K = 0; K < Components.size(); ++K)
	{
		if (!IsChosen(static_cast<int>(K)))
		{
			continue;
		}
		Component& C = Components[K];
		if (!(C.ForceScaleN > 0.0))
		{
			C.ForceScaleN = 1.0;
		}
		C.Demand = Applied[K] / -C.ForceScaleN;
		const double InverseLengthPerM = 1.0 / C.LengthM;
		for (std::size_t I = 0; I < C.Joints.size(); ++I)
		{
			JointData& J = C.Joints[I];
			const statics::JointGeometry& G = Geometry[K][I];
			J.FirstSite = C.Sites.size();
			for (std::size_t Corner = 0; Corner < CornerCount; ++Corner)
			{
				Site S;
				S.OffsetA = G.OffsetA;
				S.OffsetB = G.OffsetB;
				S.ArmA = Scale(InverseLengthPerM, Sub(G.CornerM[Corner], G.CentroidAM));
				S.ArmB = Scale(InverseLengthPerM, Sub(G.CornerM[Corner], G.CentroidBM));
				C.Sites.push_back(S);
			}
		}
	}
	return Components;
}

void AddGroup(Program& P, std::size_t SiteIndex, double Bound, const std::vector<Vec3>& Directions)
{
	if (!(Bound > 0.0))
	{
		return;
	}
	P.Groups.push_back(Group{SiteIndex, P.Directions.size(), Directions.size(), Bound});
	P.Directions.insert(P.Directions.end(), Directions.begin(), Directions.end());
}

Program BuildProgram(const Component& C, int Generators, double RadiusScale, double Cap, bool bInscribed)
{
	Program P;
	P.Cap = Cap;
	P.Offset = Eigen::VectorXd::Zero(C.Rows);
	const std::size_t Count = static_cast<std::size_t>(Generators);
	std::vector<Vec3> Friction(Count);
	std::vector<Vec3> Cohesion(Count);
	for (const JointData& J : C.Joints)
	{
		for (std::size_t K = 0; K < Count; ++K)
		{
			const double AngleRad = 2.0 * Pi * static_cast<double>(K) / static_cast<double>(Generators);
			const Vec3 InPlane = statics::Add(Scale(std::cos(AngleRad), J.AxisU), Scale(std::sin(AngleRad), J.AxisV));
			Friction[K] = statics::Add(J.Normal, Scale(RadiusScale * J.Friction, InPlane));
			Cohesion[K] = Scale(RadiusScale, InPlane);
		}
		const std::vector<Vec3> NormalOnly{J.Normal};
		const double TensionN = bInscribed ? J.InnerTensionN : J.TensionN;
		const double CohesionN = bInscribed ? J.InnerCohesionN : J.CohesionN;
		for (std::size_t Corner = 0; Corner < CornerCount; ++Corner)
		{
			const std::size_t SiteIndex = J.FirstSite + Corner;
			ApplyCornerForce(P.Offset.data(), C.Sites[SiteIndex], Scale(TensionN / C.ForceScaleN, J.Normal));
			const std::size_t GroupsBefore = P.Groups.size();
			if (!(J.ShearCapN > 0.0))
			{
				AddGroup(P, SiteIndex, (J.CrushingN + TensionN) / C.ForceScaleN, NormalOnly);
				continue;
			}
			AddGroup(P, SiteIndex, (J.CrushingN + TensionN) / C.ForceScaleN, Friction);
			AddGroup(P, SiteIndex, CohesionN / C.ForceScaleN, Cohesion);
			if (IsPositiveFinite(J.ShearCapN) && P.Groups.size() > GroupsBefore)
			{
				CapSite Shear;
				Shear.SiteIndex = SiteIndex;
				Shear.Bound = (bInscribed ? std::cos(Pi / static_cast<double>(Generators)) : 1.0) * J.ShearCapN / C.ForceScaleN;
				for (std::size_t G = GroupsBefore; G < P.Groups.size(); ++G)
				{
					P.Groups[G].CapIndex = static_cast<int>(P.CapSites.size());
					Shear.Groups.push_back(G);
				}
				for (std::size_t K = 0; K < Count; ++K)
				{
					const double AngleRad = 2.0 * Pi * static_cast<double>(K) / static_cast<double>(Generators);
					Shear.Facets.push_back(statics::Add(Scale(std::cos(AngleRad), J.AxisU), Scale(std::sin(AngleRad), J.AxisV)));
				}
				Shear.FacetMatrix.resize(static_cast<Eigen::Index>(Shear.Facets.size()), 3);
				for (std::size_t K = 0; K < Shear.Facets.size(); ++K)
				{
					Shear.FacetMatrix.row(static_cast<Eigen::Index>(K)) = Eigen::RowVector3d(Shear.Facets[K].X, Shear.Facets[K].Y, Shear.Facets[K].Z);
				}
				for (std::size_t W = 0; W < Shear.Groups.size(); ++W)
				{
					const Group& Gr = P.Groups[Shear.Groups[W]];
					for (std::size_t Jd = Gr.First; Jd < Gr.First + Gr.Count; ++Jd)
					{
						Shear.DirectionGroup.push_back(static_cast<Eigen::Index>(W));
						Shear.DirectionVariable.push_back(Jd);
					}
				}
				Shear.DirectionMatrix.resize(static_cast<Eigen::Index>(Shear.DirectionVariable.size()), 3);
				for (std::size_t Row = 0; Row < Shear.DirectionVariable.size(); ++Row)
				{
					const Vec3& Dir = P.Directions[Shear.DirectionVariable[Row]];
					Shear.DirectionMatrix.row(static_cast<Eigen::Index>(Row)) = Eigen::RowVector3d(Dir.X, Dir.Y, Dir.Z);
				}
				P.CapSites.push_back(std::move(Shear));
			}
		}
	}
	const Eigen::Index GroupCount = static_cast<Eigen::Index>(P.Groups.size());
	P.LoadFactor = static_cast<Eigen::Index>(P.Directions.size());
	P.FirstSlack = P.LoadFactor + 1;
	P.CapSlack = P.FirstSlack + GroupCount;
	P.FirstCapSlack = P.CapSlack + 1;
	P.FirstGroupRow = C.Rows;
	P.CapRow = P.FirstGroupRow + GroupCount;
	P.FirstCapRow = P.CapRow + 1;
	Eigen::Index FacetCount = 0;
	for (CapSite& Shear : P.CapSites)
	{
		Shear.FirstRow = P.FirstCapRow + FacetCount;
		Shear.FirstSlack = P.FirstCapSlack + FacetCount;
		FacetCount += static_cast<Eigen::Index>(Shear.Facets.size());
	}
	P.Variables = P.FirstCapSlack + FacetCount;
	P.RowCount = P.FirstCapRow + FacetCount;
	return P;
}

Eigen::VectorXd RowBounds(const Component& C, const Program& P)
{
	Eigen::VectorXd B(P.RowCount);
	B.head(C.Rows) = P.Offset;
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		B(P.FirstGroupRow + static_cast<Eigen::Index>(G)) = P.Groups[G].Bound;
	}
	B(P.CapRow) = P.Cap;
	for (const CapSite& Cap : P.CapSites)
	{
		for (std::size_t K = 0; K < Cap.Facets.size(); ++K)
		{
			B(Cap.FirstRow + static_cast<Eigen::Index>(K)) = Cap.Bound;
		}
	}
	return B;
}

Eigen::VectorXd Apply(const Component& C, const Program& P, const Eigen::VectorXd& V)
{
	Eigen::VectorXd Out(P.RowCount);
	double* O = Out.data();
	const double* Values = V.data();
	const double* Demand = C.Demand.data();
	const double T = Values[P.LoadFactor];
	for (int R = 0; R < C.Rows; ++R)
	{
		O[R] = -T * Demand[R];
	}
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		const Group& Gr = P.Groups[G];
		Vec3 Force;
		double Sum = 0.0;
		for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
		{
			const Vec3& Dir = P.Directions[J];
			Force.X += Values[J] * Dir.X;
			Force.Y += Values[J] * Dir.Y;
			Force.Z += Values[J] * Dir.Z;
			Sum += Values[J];
		}
		ApplyCornerForce(O, C.Sites[Gr.SiteIndex], Force);
		O[P.FirstGroupRow + static_cast<Eigen::Index>(G)] = Sum + Values[P.FirstSlack + static_cast<Eigen::Index>(G)];
	}
	O[P.CapRow] = T + Values[P.CapSlack];
	for (const CapSite& Cap : P.CapSites)
	{
		Vec3 Shear;
		for (const std::size_t G : Cap.Groups)
		{
			const Group& Gr = P.Groups[G];
			for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
			{
				Shear = statics::Add(Shear, Scale(Values[J], P.Directions[J]));
			}
		}
		for (std::size_t K = 0; K < Cap.Facets.size(); ++K)
		{
			const Eigen::Index Offset = static_cast<Eigen::Index>(K);
			O[Cap.FirstRow + Offset] = Dot(Cap.Facets[K], Shear) + Values[Cap.FirstSlack + Offset];
		}
	}
	return Out;
}

double DemandDot(const Component& C, const double* Y)
{
	const double* Demand = C.Demand.data();
	double Sum = 0.0;
	for (int R = 0; R < C.Rows; ++R)
	{
		Sum += Demand[R] * Y[R];
	}
	return Sum;
}

bool HasPositiveRelativeWork(const Component& C, const double* Y)
{
	return detail::FailedInnerMechanismIsAdmissible(C.Demand.data(), Y, C.Rows);
}

Eigen::VectorXd ApplyTranspose(const Component& C, const Program& P, const Eigen::VectorXd& Y)
{
	Eigen::VectorXd Out(P.Variables);
	double* O = Out.data();
	const double* Duals = Y.data();
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		const Group& Gr = P.Groups[G];
		const Vec3 V = RelativeVelocity(C.Sites[Gr.SiteIndex], Duals);
		const double GroupDual = Duals[P.FirstGroupRow + static_cast<Eigen::Index>(G)];
		for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
		{
			O[J] = Dot(P.Directions[J], V) + GroupDual;
		}
		O[P.FirstSlack + static_cast<Eigen::Index>(G)] = GroupDual;
	}
	O[P.LoadFactor] = -DemandDot(C, Duals) + Duals[P.CapRow];
	O[P.CapSlack] = Duals[P.CapRow];
	for (const CapSite& Cap : P.CapSites)
	{
		Vec3 Facet;
		for (std::size_t K = 0; K < Cap.Facets.size(); ++K)
		{
			const Eigen::Index Offset = static_cast<Eigen::Index>(K);
			Facet = statics::Add(Facet, Scale(Duals[Cap.FirstRow + Offset], Cap.Facets[K]));
			O[Cap.FirstSlack + Offset] = Duals[Cap.FirstRow + Offset];
		}
		for (const std::size_t G : Cap.Groups)
		{
			const Group& Gr = P.Groups[G];
			for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
			{
				O[J] += Dot(P.Directions[J], Facet);
			}
		}
	}
	return Out;
}

bool HasPositivePivots(const Eigen::LLT<Eigen::MatrixXd>& Factor)
{
	if (Factor.info() != Eigen::Success)
	{
		return false;
	}
	const Eigen::VectorXd Pivots = Factor.matrixLLT().diagonal();
	return std::all_of(Pivots.data(), Pivots.data() + Pivots.size(), [](double Pivot) { return Pivot > 0.0 && IsFinite(Pivot); });
}

template <typename AddShift, typename TryFactor>
bool EscalateShifts(double LargestDiagonal, AddShift&& Add, TryFactor&& Try)
{
	if (!(LargestDiagonal > 0.0))
	{
		return false;
	}
	double Applied = 0.0;
	for (const double Shift : RetryShifts)
	{
		Add((Shift - Applied) * LargestDiagonal);
		Applied = Shift;
		if (Try())
		{
			return true;
		}
	}
	return false;
}

bool FactorWithRetryShifts(Eigen::MatrixXd& M, Eigen::LLT<Eigen::MatrixXd>& Factor)
{
	return EscalateShifts(
		M.diagonal().maxCoeff(), [&](double Delta) { M.diagonal().array() += Delta; },
		[&]() {
			Factor.compute(M);
			return HasPositivePivots(Factor);
		});
}

using Matrix3 = std::array<double, 9>;
using Jacobian = std::array<double, 18>;

void AddSiteBlock(double* M, int Rows, int OffsetRow, const Jacobian& RowSide, int OffsetColumn, const Jacobian& ColumnSide, double Sign, const Matrix3& K)
{
	std::array<double, 18> KG{};
	for (std::size_t R = 0; R < 3; ++R)
	{
		for (std::size_t J = 0; J < 6; ++J)
		{
			KG[R * 6 + J] = K[R * 3 + 0] * ColumnSide[0 * 6 + J] + K[R * 3 + 1] * ColumnSide[1 * 6 + J] + K[R * 3 + 2] * ColumnSide[2 * 6 + J];
		}
	}
	for (std::size_t J = 0; J < 6; ++J)
	{
		double* Column = M + static_cast<std::size_t>(OffsetColumn + static_cast<int>(J)) * static_cast<std::size_t>(Rows) + static_cast<std::size_t>(OffsetRow);
		for (std::size_t I = 0; I < 6; ++I)
		{
			Column[I] += Sign * (RowSide[0 * 6 + I] * KG[0 * 6 + J] + RowSide[1 * 6 + I] * KG[1 * 6 + J] + RowSide[2 * 6 + I] * KG[2 * 6 + J]);
		}
	}
}

Eigen::MatrixXd AssembleSites(const Component& C, const std::vector<Matrix3>& SiteMatrix)
{
	Eigen::MatrixXd M = Eigen::MatrixXd::Zero(C.Rows, C.Rows);
	for (std::size_t I = 0; I < C.Sites.size(); ++I)
	{
		const Site& S = C.Sites[I];
		const std::array<int, 2> Offsets{S.OffsetA, S.OffsetB};
		const std::array<double, 2> Signs{-1.0, 1.0};
		const std::array<Jacobian, 2> Jac{statics::CornerJacobianRows(S.ArmA), statics::CornerJacobianRows(S.ArmB)};
		for (std::size_t A = 0; A < 2; ++A)
		{
			for (std::size_t B = 0; B < 2; ++B)
			{
				if (Offsets[A] >= 0 && Offsets[B] >= 0)
				{
					AddSiteBlock(M.data(), C.Rows, Offsets[A], Jac[A], Offsets[B], Jac[B], Signs[A] * Signs[B], SiteMatrix[I]);
				}
			}
		}
	}
	return M;
}

void AddSparseSiteBlock(std::vector<Eigen::Triplet<double>>& Triplets, int OffsetRow, const Jacobian& RowSide, int OffsetColumn, const Jacobian& ColumnSide, double Sign, const Matrix3& K)
{
	std::array<double, 18> KG{};
	for (std::size_t R = 0; R < 3; ++R)
	{
		for (std::size_t J = 0; J < 6; ++J)
		{
			KG[R * 6 + J] = K[R * 3 + 0] * ColumnSide[0 * 6 + J] + K[R * 3 + 1] * ColumnSide[1 * 6 + J] + K[R * 3 + 2] * ColumnSide[2 * 6 + J];
		}
	}
	for (std::size_t J = 0; J < 6; ++J)
	{
		for (std::size_t I = 0; I < 6; ++I)
		{
			Triplets.emplace_back(OffsetRow + static_cast<int>(I), OffsetColumn + static_cast<int>(J), Sign * (RowSide[0 * 6 + I] * KG[0 * 6 + J] + RowSide[1 * 6 + I] * KG[1 * 6 + J] + RowSide[2 * 6 + I] * KG[2 * 6 + J]));
		}
	}
}

bool HasBorderedPivots(const SparseNormal& S)
{
	if (S.BorderedFactor.info() != Eigen::Success)
	{
		return false;
	}
	const Eigen::VectorXd Pivots = S.BorderedFactor.vectorD();
	const Eigen::Index Last = Pivots.size() - 1;
	for (Eigen::Index I = 0; I < Last; ++I)
	{
		if (!(Pivots(I) > 0.0) || !IsFinite(Pivots(I)))
		{
			return false;
		}
	}
	return Pivots(Last) < 0.0 && IsFinite(Pivots(Last));
}

bool FactorBordered(const Component& C, double Coupling, SparseNormal& S, bool& bShifted)
{
	const int Border = C.Rows;
	const double* Demand = C.Demand.data();
	S.BorderedTriplets.assign(S.Triplets.begin(), S.Triplets.end());
	S.BorderedTriplets.emplace_back(Border, Border, -1.0 / Coupling);
	for (int Row = 0; Row < C.Rows; ++Row)
	{
		if (Demand[Row] != 0.0)
		{
			S.BorderedTriplets.emplace_back(Row, Border, Demand[Row]);
			S.BorderedTriplets.emplace_back(Border, Row, Demand[Row]);
		}
	}
	S.Bordered.resize(C.Rows + 1, C.Rows + 1);
	S.Bordered.setFromTriplets(S.BorderedTriplets.begin(), S.BorderedTriplets.end());
	if (!S.bAnalysed)
	{
		S.BorderedFactor.analyzePattern(S.Bordered);
		S.bAnalysed = true;
	}
	S.BorderedFactor.factorize(S.Bordered);
	if (HasBorderedPivots(S))
	{
		return true;
	}
	double LargestDiagonal = -std::numeric_limits<double>::infinity();
	for (int Row = 0; Row < C.Rows; ++Row)
	{
		LargestDiagonal = std::max(LargestDiagonal, S.Bordered.coeff(Row, Row) + Coupling * Demand[Row] * Demand[Row]);
	}
	bShifted = true;
	return EscalateShifts(
		LargestDiagonal,
		[&](double Delta) {
			for (int Row = 0; Row < C.Rows; ++Row)
			{
				S.Bordered.coeffRef(Row, Row) += Delta;
			}
		},
		[&]() {
			S.BorderedFactor.factorize(S.Bordered);
			return HasBorderedPivots(S);
		});
}

bool FactorSparse(const Component& C, double Dt, double DeltaT, const std::vector<Matrix3>& SiteMatrix, Normal& N)
{
	SparseNormal& S = *N.Sparse;
	S.Triplets.clear();
	for (int Row = 0; Row < C.Rows; ++Row)
	{
		S.Triplets.emplace_back(Row, Row, 0.0);
	}
	for (std::size_t I = 0; I < C.Sites.size(); ++I)
	{
		const Site& St = C.Sites[I];
		const std::array<int, 2> Offsets{St.OffsetA, St.OffsetB};
		const std::array<double, 2> Signs{-1.0, 1.0};
		const std::array<Jacobian, 2> Jac{statics::CornerJacobianRows(St.ArmA), statics::CornerJacobianRows(St.ArmB)};
		for (std::size_t A = 0; A < 2; ++A)
		{
			for (std::size_t B = 0; B < 2; ++B)
			{
				if (Offsets[A] >= 0 && Offsets[B] >= 0)
				{
					AddSparseSiteBlock(S.Triplets, Offsets[A], Jac[A], Offsets[B], Jac[B], Signs[A] * Signs[B], SiteMatrix[I]);
				}
			}
		}
	}
	N.Ht = Dt + DeltaT;
	const double Coupling = Dt * DeltaT / N.Ht;
	return FactorBordered(C, Coupling, S, N.bShifted);
}

bool FactorCapStructured(const Program& P, const CapSite& Cap, const double* D, CapScratch& S, CapSiteFactor& Factor, Eigen::MatrixXd& F, Eigen::MatrixXd& SolvedF, Matrix3& K)
{
	const Eigen::Index GroupRows = static_cast<Eigen::Index>(Cap.Groups.size());
	const Eigen::Index FacetCount = static_cast<Eigen::Index>(Cap.Facets.size());
	const Eigen::Index DirectionCount = static_cast<Eigen::Index>(Cap.DirectionVariable.size());
	S.GroupSlackD.resize(GroupRows);
	S.FacetSlackD.resize(FacetCount);
	S.DirectionD.resize(DirectionCount);
	for (Eigen::Index W = 0; W < GroupRows; ++W)
	{
		S.GroupSlackD(W) = D[P.FirstSlack + static_cast<Eigen::Index>(Cap.Groups[static_cast<std::size_t>(W)])];
	}
	for (Eigen::Index Facet = 0; Facet < FacetCount; ++Facet)
	{
		S.FacetSlackD(Facet) = D[Cap.FirstSlack + Facet];
	}
	for (Eigen::Index Row = 0; Row < DirectionCount; ++Row)
	{
		S.DirectionD(Row) = D[Cap.DirectionVariable[static_cast<std::size_t>(Row)]];
	}
	Factor.Build(S.GroupSlackD, S.FacetSlackD, Cap.FacetMatrix, Cap.DirectionMatrix, Cap.DirectionGroup, S.DirectionD);
	if (!Factor.bPositive)
	{
		return false;
	}
	const Eigen::Matrix3d Full = Cap.DirectionMatrix.transpose() * S.DirectionD.asDiagonal() * Cap.DirectionMatrix;
	F.resize(3, GroupRows + FacetCount);
	F.leftCols(GroupRows) = Factor.GroupSum;
	F.rightCols(FacetCount).noalias() = Full.lazyProduct(Cap.FacetMatrix.transpose());
	S.FT = F.transpose();
	Factor.SolveInto(S.FT, SolvedF);
	const Eigen::Matrix3d K3 = Full - F * SolvedF;
	for (std::size_t R = 0; R < 3; ++R)
	{
		for (std::size_t Col = 0; Col < 3; ++Col)
		{
			K[R * 3 + Col] += K3(static_cast<Eigen::Index>(R), static_cast<Eigen::Index>(Col));
		}
	}
	return true;
}

bool FactorNormal(const Component& C, const Program& P, Normal& N)
{
	N.Z.resize(P.Groups.size());
	N.H.resize(P.Groups.size());
	const double* D = N.D.data();
	std::vector<Matrix3> SiteMatrix(C.Sites.size(), Matrix3{});
	N.CapF.resize(P.CapSites.size());
	N.CapStructured.resize(P.CapSites.size());
	N.CapSolvedF.resize(P.CapSites.size());
	N.CapScratchD.resize(P.CapSites.size());
	N.CapRhs.resize(P.CapSites.size());
	N.CapSolved.resize(P.CapSites.size());
	N.CapCorrection.resize(P.CapSites.size());
	for (std::size_t Index = 0; Index < P.CapSites.size(); ++Index)
	{
		const CapSite& Cap = P.CapSites[Index];
		if (!FactorCapStructured(P, Cap, D, N.CapScratchD[Index], N.CapStructured[Index], N.CapF[Index], N.CapSolvedF[Index], SiteMatrix[Cap.SiteIndex]))
		{
			return false;
		}
	}
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		const Group& Gr = P.Groups[G];
		if (Gr.CapIndex >= 0)
		{
			continue;
		}
		double Sum = D[P.FirstSlack + static_cast<Eigen::Index>(G)];
		std::array<double, 3> Z{};
		Matrix3& K = SiteMatrix[Gr.SiteIndex];
		for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
		{
			const std::array<double, 3> Dir{P.Directions[J].X, P.Directions[J].Y, P.Directions[J].Z};
			Sum += D[J];
			for (std::size_t R = 0; R < 3; ++R)
			{
				const double Weighted = D[J] * Dir[R];
				Z[R] += Weighted;
				for (std::size_t Col = 0; Col < 3; ++Col)
				{
					K[R * 3 + Col] += Weighted * Dir[Col];
				}
			}
		}
		for (std::size_t R = 0; R < 3; ++R)
		{
			for (std::size_t Col = 0; Col < 3; ++Col)
			{
				K[R * 3 + Col] -= Z[R] * Z[Col] / Sum;
			}
		}
		N.H[G] = Sum;
		N.Z[G] = Vec3{Z[0], Z[1], Z[2]};
	}
	if (N.Sparse != nullptr)
	{
		return FactorSparse(C, D[P.LoadFactor], D[P.CapSlack], SiteMatrix, N);
	}
	Eigen::MatrixXd M = AssembleSites(C, SiteMatrix);
	const double Dt = D[P.LoadFactor];
	const double DeltaT = D[P.CapSlack];
	N.Ht = Dt + DeltaT;
	const double Coupling = Dt * DeltaT / N.Ht;
	const double* Demand = C.Demand.data();
	double* Mp = M.data();
	for (int Col = 0; Col < C.Rows; ++Col)
	{
		for (int Row = 0; Row < C.Rows; ++Row)
		{
			Mp[static_cast<std::size_t>(Col) * static_cast<std::size_t>(C.Rows) + static_cast<std::size_t>(Row)] += Coupling * Demand[Row] * Demand[Col];
		}
	}
	N.Factor.compute(M);
	if (HasPositivePivots(N.Factor))
	{
		return true;
	}
	N.bShifted = true;
	return FactorWithRetryShifts(M, N.Factor);
}

Eigen::VectorXd SolveNormal(const Component& C, const Program& P, const Normal& N, const Eigen::VectorXd& R)
{
	const double* Rp = R.data();
	const double* Demand = C.Demand.data();
	const double Dt = N.D(P.LoadFactor);
	const double CapShare = Dt * Rp[P.CapRow] / N.Ht;
	Eigen::VectorXd Y(P.RowCount);
	double* Yp = Y.data();
	for (int Row = 0; Row < C.Rows; ++Row)
	{
		Yp[Row] = Rp[Row] + CapShare * Demand[Row];
	}
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		if (P.Groups[G].CapIndex >= 0)
		{
			continue;
		}
		const double Share = Rp[P.FirstGroupRow + static_cast<Eigen::Index>(G)] / N.H[G];
		ApplyCornerForce(Yp, C.Sites[P.Groups[G].SiteIndex], Scale(-Share, N.Z[G]));
	}
	for (std::size_t Index = 0; Index < P.CapSites.size(); ++Index)
	{
		const CapSite& Cap = P.CapSites[Index];
		const Eigen::Index GroupRows = static_cast<Eigen::Index>(Cap.Groups.size());
		Eigen::MatrixXd& Rw = N.CapRhs[Index];
		Rw.resize(GroupRows + static_cast<Eigen::Index>(Cap.Facets.size()), 1);
		for (Eigen::Index W = 0; W < GroupRows; ++W)
		{
			Rw(W) = Rp[P.FirstGroupRow + static_cast<Eigen::Index>(Cap.Groups[static_cast<std::size_t>(W)])];
		}
		for (std::size_t K = 0; K < Cap.Facets.size(); ++K)
		{
			Rw(GroupRows + static_cast<Eigen::Index>(K)) = Rp[Cap.FirstRow + static_cast<Eigen::Index>(K)];
		}
		N.CapStructured[Index].SolveInto(Rw, N.CapSolved[Index]);
		const Eigen::Vector3d Force = N.CapF[Index] * N.CapSolved[Index].col(0);
		ApplyCornerForce(Yp, C.Sites[Cap.SiteIndex], Vec3{-Force(0), -Force(1), -Force(2)});
	}
	auto Equilibrium = Y.head(C.Rows);
	if (N.Sparse != nullptr)
	{
		Eigen::VectorXd Extended = Eigen::VectorXd::Zero(C.Rows + 1);
		Extended.head(C.Rows) = Equilibrium;
		Equilibrium = N.Sparse->BorderedFactor.solve(Extended).head(C.Rows);
	}
	else
	{
		N.Factor.solveInPlace(Equilibrium);
	}
	for (std::size_t Index = 0; Index < P.CapSites.size(); ++Index)
	{
		const CapSite& Cap = P.CapSites[Index];
		const Eigen::Index GroupRows = static_cast<Eigen::Index>(Cap.Groups.size());
		const Vec3 V = RelativeVelocity(C.Sites[Cap.SiteIndex], Yp);
		const Eigen::Vector3d Velocity(V.X, V.Y, V.Z);
		Eigen::VectorXd& Yw = N.CapCorrection[Index];
		Yw.noalias() = N.CapSolvedF[Index] * Velocity;
		Yw = N.CapSolved[Index].col(0) - Yw;
		for (Eigen::Index W = 0; W < GroupRows; ++W)
		{
			Yp[P.FirstGroupRow + static_cast<Eigen::Index>(Cap.Groups[static_cast<std::size_t>(W)])] = Yw(W);
		}
		for (std::size_t K = 0; K < Cap.Facets.size(); ++K)
		{
			Yp[Cap.FirstRow + static_cast<Eigen::Index>(K)] = Yw(GroupRows + static_cast<Eigen::Index>(K));
		}
	}
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		if (P.Groups[G].CapIndex >= 0)
		{
			continue;
		}
		const Eigen::Index Row = P.FirstGroupRow + static_cast<Eigen::Index>(G);
		Yp[Row] = (Rp[Row] - Dot(N.Z[G], RelativeVelocity(C.Sites[P.Groups[G].SiteIndex], Yp))) / N.H[G];
	}
	Yp[P.CapRow] = (Rp[P.CapRow] + Dt * DemandDot(C, Yp)) / N.Ht;
	return Y;
}

double OwnLoad(const Component& C, int Offset)
{
	if (Offset < 0)
	{
		return 0.0;
	}
	double Largest = 0.0;
	for (int R = 0; R < DofPerBody; ++R)
	{
		Largest = std::max(Largest, std::abs(C.Demand.data()[Offset + R]));
	}
	return Largest;
}

Embedding StartingPoint(const Component& C, const Program& P)
{
	Embedding E;
	E.X = Eigen::VectorXd::Zero(P.Variables);
	double* X = E.X.data();
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		const Group& Gr = P.Groups[G];
		double Start = std::min(1.0, Gr.Bound / (2.0 * static_cast<double>(Gr.Count)));
		const Site& S = C.Sites[Gr.SiteIndex];
		for (const int Offset : {S.OffsetA, S.OffsetB})
		{
			const double Load = OwnLoad(C, Offset);
			if (Load > 0.0 && Load < LightBodyLoad)
			{
				Start = std::min(Start, Load);
			}
		}
		for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
		{
			X[J] = Start;
		}
		X[P.FirstSlack + static_cast<Eigen::Index>(G)] = Gr.Bound - Start * static_cast<double>(Gr.Count);
	}
	X[P.LoadFactor] = std::min(1.0, 0.5 * P.Cap);
	X[P.CapSlack] = P.Cap - X[P.LoadFactor];
	for (const CapSite& Cap : P.CapSites)
	{
		Vec3 Shear;
		for (const std::size_t G : Cap.Groups)
		{
			const Group& Gr = P.Groups[G];
			for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
			{
				Shear = statics::Add(Shear, Scale(X[J], P.Directions[J]));
			}
		}
		for (std::size_t K = 0; K < Cap.Facets.size(); ++K)
		{
			X[Cap.FirstSlack + static_cast<Eigen::Index>(K)] = Cap.Bound - Dot(Cap.Facets[K], Shear);
		}
	}
	E.S = Eigen::VectorXd(P.Variables);
	for (Eigen::Index I = 0; I < P.Variables; ++I)
	{
		E.S.data()[I] = 1.0 / X[I];
	}
	E.Y = Eigen::VectorXd::Zero(P.RowCount);
	return E;
}

struct Direction
{
	Eigen::VectorXd X;
	Eigen::VectorXd S;
	Eigen::VectorXd Y;
	double Tau = 0.0;
	double Kappa = 0.0;
};

struct Residuals
{
	Eigen::VectorXd Primal;
	Eigen::VectorXd Dual;
	Eigen::VectorXd Reaction;
	double Gap = 0.0;
};

struct NewtonRhs
{
	Eigen::VectorXd R1;
	Eigen::VectorXd R2;
	double R3 = 0.0;
	Eigen::VectorXd R4;
	double R5 = 0.0;
};

struct NewtonContext
{
	const Component* C = nullptr;
	const Program* P = nullptr;
	const Normal* N = nullptr;
	const Eigen::VectorXd* Bounds = nullptr;
	Eigen::VectorXd Q;
	Eigen::VectorXd U1;
	double Denominator = 0.0;
};

double DotProduct(const Eigen::VectorXd& A, const Eigen::VectorXd& B)
{
	const double* Ap = A.data();
	const double* Bp = B.data();
	double Sum = 0.0;
	for (Eigen::Index I = 0; I < A.size(); ++I)
	{
		Sum += Ap[I] * Bp[I];
	}
	return Sum;
}

double LargestMagnitude(const double* Values, Eigen::Index Count)
{
	double Largest = 0.0;
	for (Eigen::Index I = 0; I < Count; ++I)
	{
		Largest = std::max(Largest, std::abs(Values[I]));
	}
	return Largest;
}

Direction SolveLinearised(const NewtonContext& Ctx, const Embedding& E, const NewtonRhs& Rhs)
{
	const Component& C = *Ctx.C;
	const Program& P = *Ctx.P;
	const Normal& N = *Ctx.N;
	const Eigen::Index Count = P.Variables;
	const double* D = N.D.data();
	const double* X = E.X.data();
	const double* S = E.S.data();
	Eigen::VectorXd Through(Count);
	Eigen::VectorXd Weighted(Count);
	for (Eigen::Index I = 0; I < Count; ++I)
	{
		Through.data()[I] = Rhs.R4.data()[I] / X[I] - Rhs.R2.data()[I];
		Weighted.data()[I] = D[I] * Through.data()[I];
	}
	Eigen::VectorXd Reduced = Apply(C, P, Weighted);
	for (Eigen::Index Row = 0; Row < P.RowCount; ++Row)
	{
		Reduced.data()[Row] = Rhs.R1.data()[Row] - Reduced.data()[Row];
	}
	Direction Dir;
	Dir.Y = SolveNormal(C, P, N, Reduced);
	Dir.X = ApplyTranspose(C, P, Dir.Y);
	for (Eigen::Index I = 0; I < Count; ++I)
	{
		Dir.X.data()[I] = D[I] * (Dir.X.data()[I] + Through.data()[I]);
	}
	Dir.Tau = (Rhs.R3 - ObjectiveNorm * Dir.X.data()[P.LoadFactor] - DotProduct(*Ctx.Bounds, Dir.Y) + Rhs.R5 / E.Tau) / Ctx.Denominator;
	for (Eigen::Index Row = 0; Row < P.RowCount; ++Row)
	{
		Dir.Y.data()[Row] += Dir.Tau * Ctx.Q.data()[Row];
	}
	Dir.S = Eigen::VectorXd(Count);
	for (Eigen::Index I = 0; I < Count; ++I)
	{
		Dir.X.data()[I] += Dir.Tau * Ctx.U1.data()[I];
		Dir.S.data()[I] = (Rhs.R4.data()[I] - S[I] * Dir.X.data()[I]) / X[I];
	}
	Dir.Kappa = (Rhs.R5 - E.Kappa * Dir.Tau) / E.Tau;
	return Dir;
}

Direction SolveNewton(const NewtonContext& Ctx, const Embedding& E, const Residuals& R, double Reduction, const Eigen::VectorXd& R4, double R5, int Passes)
{
	NewtonRhs Rhs;
	Rhs.R1 = -Reduction * R.Primal;
	Rhs.R2 = -Reduction * R.Dual;
	Rhs.R3 = -Reduction * R.Gap;
	Rhs.R4 = R4;
	Rhs.R5 = R5;
	Direction Dir = SolveLinearised(Ctx, E, Rhs);
	for (int Pass = 0; Pass < Passes; ++Pass)
	{
		NewtonRhs Error;
		Error.R1 = Apply(*Ctx.C, *Ctx.P, Dir.X);
		for (Eigen::Index Row = 0; Row < Ctx.P->RowCount; ++Row)
		{
			Error.R1.data()[Row] = Rhs.R1.data()[Row] - (Error.R1.data()[Row] - Dir.Tau * Ctx.Bounds->data()[Row]);
		}
		Error.R2 = Eigen::VectorXd::Zero(Ctx.P->Variables);
		Error.R4 = Error.R2;
		const Direction Fix = SolveLinearised(Ctx, E, Error);
		for (Eigen::Index I = 0; I < Ctx.P->Variables; ++I)
		{
			Dir.X.data()[I] += Fix.X.data()[I];
			Dir.S.data()[I] += Fix.S.data()[I];
		}
		for (Eigen::Index Row = 0; Row < Ctx.P->RowCount; ++Row)
		{
			Dir.Y.data()[Row] += Fix.Y.data()[Row];
		}
		Dir.Tau += Fix.Tau;
		Dir.Kappa += Fix.Kappa;
	}
	return Dir;
}

double MaxStep(const Embedding& E, const Direction& Dir)
{
	double Limit = Unbounded;
	const double* X = E.X.data();
	const double* S = E.S.data();
	const double* DX = Dir.X.data();
	const double* DS = Dir.S.data();
	for (Eigen::Index I = 0; I < E.X.size(); ++I)
	{
		if (DX[I] < 0.0)
		{
			Limit = std::min(Limit, -X[I] / DX[I]);
		}
		if (DS[I] < 0.0)
		{
			Limit = std::min(Limit, -S[I] / DS[I]);
		}
	}
	if (Dir.Tau < 0.0)
	{
		Limit = std::min(Limit, -E.Tau / Dir.Tau);
	}
	if (Dir.Kappa < 0.0)
	{
		Limit = std::min(Limit, -E.Kappa / Dir.Kappa);
	}
	return Limit;
}

double Complementarity(const Embedding& E)
{
	return DotProduct(E.X, E.S) + E.Tau * E.Kappa;
}

double ComplementarityAfter(const Embedding& E, const Direction& Dir, double Step)
{
	const double* X = E.X.data();
	const double* S = E.S.data();
	const double* DX = Dir.X.data();
	const double* DS = Dir.S.data();
	double Sum = (E.Tau + Step * Dir.Tau) * (E.Kappa + Step * Dir.Kappa);
	for (Eigen::Index I = 0; I < E.X.size(); ++I)
	{
		Sum += (X[I] + Step * DX[I]) * (S[I] + Step * DS[I]);
	}
	return Sum;
}

Residuals ComputeResiduals(const Component& C, const Program& P, const Embedding& E, const Eigen::VectorXd& Bounds)
{
	Residuals R;
	R.Primal = Apply(C, P, E.X);
	for (Eigen::Index Row = 0; Row < P.RowCount; ++Row)
	{
		R.Primal.data()[Row] -= E.Tau * Bounds.data()[Row];
	}
	R.Reaction = ApplyTranspose(C, P, E.Y);
	R.Dual = Eigen::VectorXd(P.Variables);
	for (Eigen::Index I = 0; I < P.Variables; ++I)
	{
		R.Dual.data()[I] = R.Reaction.data()[I] + E.S.data()[I];
	}
	R.Dual.data()[P.LoadFactor] += E.Tau * ObjectiveNorm;
	R.Gap = ObjectiveNorm * E.X.data()[P.LoadFactor] + DotProduct(Bounds, E.Y) - E.Kappa;
	return R;
}

bool IsConverged(const Component& C, const Program& P, const Embedding& E, const Residuals& R, double Tolerance, bool& bFinite, double& Largest)
{
	const double T = E.X.data()[P.LoadFactor] / E.Tau;
	const double* Primal = R.Primal.data();
	const double EquilibriumScale = 1.0 + std::max(LargestMagnitude(P.Offset.data(), C.Rows), std::abs(T) * LargestMagnitude(C.Demand.data(), C.Rows));
	double PrimalRelative = std::max(LargestMagnitude(Primal, C.Rows) / EquilibriumScale, std::abs(Primal[P.CapRow]) / (1.0 + P.Cap));
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		PrimalRelative = std::max(PrimalRelative, std::abs(Primal[P.FirstGroupRow + static_cast<Eigen::Index>(G)]) / (1.0 + P.Groups[G].Bound));
	}
	for (const CapSite& Cap : P.CapSites)
	{
		for (std::size_t K = 0; K < Cap.Facets.size(); ++K)
		{
			PrimalRelative = std::max(PrimalRelative, std::abs(Primal[Cap.FirstRow + static_cast<Eigen::Index>(K)]) / (1.0 + Cap.Bound));
		}
	}
	PrimalRelative /= E.Tau;
	const double DualScale = 1.0 + std::max({ObjectiveNorm, LargestMagnitude(E.S.data(), P.Variables) / E.Tau, LargestMagnitude(R.Reaction.data(), P.Variables) / E.Tau});
	const double DualRelative = LargestMagnitude(R.Dual.data(), P.Variables) / E.Tau / DualScale;
	const double GapRelative = DotProduct(E.X, E.S) / (E.Tau * E.Tau) / (1.0 + std::abs(T));
	bFinite = IsFinite(PrimalRelative) && IsFinite(DualRelative) && IsFinite(GapRelative) && IsFinite(T);
	Largest = std::max({PrimalRelative, DualRelative, GapRelative});
	return PrimalRelative <= Tolerance && DualRelative <= Tolerance && GapRelative <= Tolerance;
}

void PrepareContext(const Component& C, const Program& P, const Embedding& E, NewtonContext& Ctx)
{
	const Normal& N = *Ctx.N;
	const double Dt = N.D.data()[P.LoadFactor];
	Eigen::VectorXd Rhs = *Ctx.Bounds;
	for (int Row = 0; Row < C.Rows; ++Row)
	{
		Rhs.data()[Row] += Dt * ObjectiveNorm * C.Demand.data()[Row];
	}
	Rhs.data()[P.CapRow] -= Dt * ObjectiveNorm;
	Ctx.Q = SolveNormal(C, P, N, Rhs);
	Ctx.U1 = ApplyTranspose(C, P, Ctx.Q);
	Ctx.U1.data()[P.LoadFactor] += ObjectiveNorm;
	for (Eigen::Index I = 0; I < P.Variables; ++I)
	{
		Ctx.U1.data()[I] *= N.D.data()[I];
	}
	Ctx.Denominator = ObjectiveNorm * Ctx.U1.data()[P.LoadFactor] + DotProduct(*Ctx.Bounds, Ctx.Q) + E.Kappa / E.Tau;
}

void Advance(Embedding& E, const Direction& Dir, double Step)
{
	for (Eigen::Index I = 0; I < E.X.size(); ++I)
	{
		E.X.data()[I] += Step * Dir.X.data()[I];
		E.S.data()[I] += Step * Dir.S.data()[I];
	}
	for (Eigen::Index Row = 0; Row < E.Y.size(); ++Row)
	{
		E.Y.data()[Row] += Step * Dir.Y.data()[Row];
	}
	E.Tau += Step * Dir.Tau;
	E.Kappa += Step * Dir.Kappa;
}

double Gamma(double Count)
{
	const double Product = Count * UnitRoundoff;
	return Product / (1.0 - Product);
}

Vec3 AbsPointVelocity(const double* Y, int Offset, const Vec3& Arm)
{
	if (Offset < 0)
	{
		return Vec3{};
	}
	const double* Row = Y + Offset;
	const double Wx = std::abs(Row[3]);
	const double Wy = std::abs(Row[4]);
	const double Wz = std::abs(Row[5]);
	const double Ax = std::abs(Arm.X);
	const double Ay = std::abs(Arm.Y);
	const double Az = std::abs(Arm.Z);
	return Vec3{std::abs(Row[0]) + Wy * Az + Wz * Ay, std::abs(Row[1]) + Wz * Ax + Wx * Az, std::abs(Row[2]) + Wx * Ay + Wy * Ax};
}

double AbsDot(const Vec3& A, const Vec3& B)
{
	return std::abs(A.X) * B.X + std::abs(A.Y) * B.Y + std::abs(A.Z) * B.Z;
}

Eigen::VectorXd ApplyAbsTranspose(const Component& C, const Program& P, const Eigen::VectorXd& Y)
{
	Eigen::VectorXd Out(P.Variables);
	double* O = Out.data();
	const Eigen::VectorXd Magnitude = Y.cwiseAbs();
	const double* Duals = Magnitude.data();
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		const Group& Gr = P.Groups[G];
		const Site& S = C.Sites[Gr.SiteIndex];
		const Vec3 V = statics::Add(AbsPointVelocity(Duals, S.OffsetA, S.ArmA), AbsPointVelocity(Duals, S.OffsetB, S.ArmB));
		const double GroupDual = Duals[P.FirstGroupRow + static_cast<Eigen::Index>(G)];
		for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
		{
			O[J] = AbsDot(P.Directions[J], V) + GroupDual;
		}
		O[P.FirstSlack + static_cast<Eigen::Index>(G)] = GroupDual;
	}
	double DemandSum = 0.0;
	for (int R = 0; R < C.Rows; ++R)
	{
		DemandSum += std::abs(C.Demand.data()[R]) * Duals[R];
	}
	O[P.LoadFactor] = DemandSum + Duals[P.CapRow] + ObjectiveNorm;
	O[P.CapSlack] = Duals[P.CapRow];
	for (const CapSite& Cap : P.CapSites)
	{
		Vec3 Facet;
		for (std::size_t K = 0; K < Cap.Facets.size(); ++K)
		{
			const Eigen::Index Offset = static_cast<Eigen::Index>(K);
			const Vec3& F = Cap.Facets[K];
			Facet = statics::Add(Facet, Scale(Duals[Cap.FirstRow + Offset], Vec3{std::abs(F.X), std::abs(F.Y), std::abs(F.Z)}));
			O[Cap.FirstSlack + Offset] = Duals[Cap.FirstRow + Offset];
		}
		for (const std::size_t G : Cap.Groups)
		{
			const Group& Gr = P.Groups[G];
			for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
			{
				O[J] += AbsDot(P.Directions[J], Facet);
			}
		}
	}
	return Out;
}

Eigen::VectorXd ColumnBoxes(const Program& P)
{
	Eigen::VectorXd Boxes(P.Variables);
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		const Group& Gr = P.Groups[G];
		for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
		{
			Boxes(static_cast<Eigen::Index>(J)) = Gr.Bound;
		}
		Boxes(P.FirstSlack + static_cast<Eigen::Index>(G)) = Gr.Bound;
	}
	Boxes(P.LoadFactor) = P.Cap;
	Boxes(P.CapSlack) = P.Cap;
	for (const CapSite& Cap : P.CapSites)
	{
		for (std::size_t K = 0; K < Cap.Facets.size(); ++K)
		{
			double Reach = Cap.Bound;
			for (const std::size_t G : Cap.Groups)
			{
				const Group& Gr = P.Groups[G];
				double Largest = 0.0;
				for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
				{
					Largest = std::max(Largest, std::abs(Dot(Cap.Facets[K], P.Directions[J])));
				}
				Reach += Gr.Bound * Largest;
			}
			Boxes(Cap.FirstSlack + static_cast<Eigen::Index>(K)) = Reach;
		}
	}
	return Boxes;
}

double BoundAtDuals(const Component& C, const Program& P, const Eigen::VectorXd& Bounds, const Eigen::VectorXd& Y)
{
	Eigen::VectorXd R = ApplyTranspose(C, P, Y);
	R.data()[P.LoadFactor] += ObjectiveNorm;
	const Eigen::VectorXd Allowance = ApplyAbsTranspose(C, P, Y);
	std::size_t MostFacets = 0;
	for (const CapSite& Cap : P.CapSites)
	{
		MostFacets = std::max(MostFacets, Cap.Facets.size());
	}
	const double ColumnGamma = Gamma(static_cast<double>(2 * DofPerBody + 2 + static_cast<int>(MostFacets)));
	const double LoadFactorGamma = Gamma(static_cast<double>(C.Rows + 2));
	double Upper = 0.0;
	double Magnitude = 0.0;
	for (Eigen::Index Row = 0; Row < P.RowCount; ++Row)
	{
		Upper -= Bounds.data()[Row] * Y.data()[Row];
		Magnitude += std::abs(Bounds.data()[Row] * Y.data()[Row]);
	}
	const Eigen::VectorXd Boxes = ColumnBoxes(P);
	for (Eigen::Index I = 0; I < P.Variables; ++I)
	{
		const double Rounding = (I == P.LoadFactor ? LoadFactorGamma : ColumnGamma) * Allowance.data()[I];
		const double Raw = R.data()[I] + Rounding;
		if (!IsFinite(Raw))
		{
			return Unbounded;
		}
		const double Excess = std::max(0.0, Raw);
		Upper += Excess * Boxes.data()[I];
		Magnitude += Excess * Boxes.data()[I];
	}
	Upper += Gamma(static_cast<double>(P.RowCount + P.Variables + 1)) * Magnitude;
	return IsFinite(Upper) ? Upper : Unbounded;
}

void PolishGroupsAndCap(const Component& C, const Program& P, Eigen::VectorXd& Y)
{
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		Y(P.FirstGroupRow + static_cast<Eigen::Index>(G)) = 0.0;
	}
	Y(P.CapRow) = 0.0;
	const Eigen::VectorXd R = ApplyTranspose(C, P, Y);
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		const Group& Gr = P.Groups[G];
		double Largest = 0.0;
		for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
		{
			Largest = std::max(Largest, R(static_cast<Eigen::Index>(J)));
		}
		Y(P.FirstGroupRow + static_cast<Eigen::Index>(G)) = -Largest;
	}
	Y(P.CapRow) = -std::max(0.0, R(P.LoadFactor) + ObjectiveNorm);
}

bool FacetDualSimplex(const Eigen::VectorXd& Cost, const Eigen::MatrixXd& A, Eigen::VectorXd B, Eigen::VectorXd& X, int PivotLimit, int BlandAfter)
{
	const Eigen::Index M = A.rows();
	const Eigen::Index N = A.cols();
	Eigen::MatrixXd T(M, N + M);
	T.leftCols(N) = A;
	T.rightCols(M).setIdentity();
	Eigen::VectorXd D(N + M);
	D.head(N) = Cost;
	D.tail(M).setZero();
	std::vector<Eigen::Index> Basis(static_cast<std::size_t>(M));
	for (Eigen::Index I = 0; I < M; ++I)
	{
		Basis[static_cast<std::size_t>(I)] = N + I;
	}
	const double Tol = 1.0e-13 * (1.0 + (M > 0 ? B.cwiseAbs().maxCoeff() : 0.0));
	bool bOptimal = false;
	for (int Pivots = 0; Pivots < PivotLimit; ++Pivots)
	{
		const bool bBland = Pivots >= BlandAfter;
		Eigen::Index Row = -1;
		double Worst = -Tol;
		for (Eigen::Index I = 0; I < M; ++I)
		{
			const bool bPick = bBland ? (B(I) < -Tol && (Row < 0 || Basis[static_cast<std::size_t>(I)] < Basis[static_cast<std::size_t>(Row)])) : B(I) < Worst;
			if (bPick)
			{
				Worst = B(I);
				Row = I;
			}
		}
		if (Row < 0)
		{
			bOptimal = true;
			break;
		}
		Eigen::Index Col = -1;
		double Best = Unbounded;
		for (Eigen::Index J = 0; J < N + M; ++J)
		{
			const double Entry = T(Row, J);
			if (Entry < -1.0e-12)
			{
				const double Ratio = std::max(0.0, D(J)) / -Entry;
				if (Ratio < Best)
				{
					Best = Ratio;
					Col = J;
				}
			}
		}
		if (Col < 0)
		{
			return false;
		}
		const double Pivot = T(Row, Col);
		T.row(Row) /= Pivot;
		B(Row) /= Pivot;
		for (Eigen::Index I = 0; I < M; ++I)
		{
			const double F = T(I, Col);
			if (I != Row && F != 0.0)
			{
				T.row(I) -= F * T.row(Row);
				B(I) -= F * B(Row);
			}
		}
		D -= D(Col) * T.row(Row).transpose();
		Basis[static_cast<std::size_t>(Row)] = Col;
	}
	if (!bOptimal)
	{
		return false;
	}
	X = Eigen::VectorXd::Zero(N);
	for (Eigen::Index I = 0; I < M; ++I)
	{
		if (Basis[static_cast<std::size_t>(I)] < N)
		{
			X(Basis[static_cast<std::size_t>(I)]) = std::max(0.0, B(I));
		}
	}
	return true;
}

void PolishFacets(const Component& C, const Program& P, Eigen::VectorXd& Y, int PivotLimit)
{
	const Eigen::VectorXd Boxes = ColumnBoxes(P);
	Eigen::VectorXd Base = Y;
	for (std::size_t G = 0; G < P.Groups.size(); ++G)
	{
		Base(P.FirstGroupRow + static_cast<Eigen::Index>(G)) = 0.0;
	}
	for (const CapSite& Cap : P.CapSites)
	{
		Base.segment(Cap.FirstRow, static_cast<Eigen::Index>(Cap.Facets.size())).setZero();
	}
	const Eigen::VectorXd Reduced = ApplyTranspose(C, P, Base);
	for (const CapSite& Cap : P.CapSites)
	{
		const Eigen::Index F = static_cast<Eigen::Index>(Cap.Facets.size());
		const Eigen::Index Gs = static_cast<Eigen::Index>(Cap.Groups.size());
		Eigen::Index Rows = 0;
		for (const std::size_t G : Cap.Groups)
		{
			Rows += static_cast<Eigen::Index>(P.Groups[G].Count);
		}
		const Eigen::Index N = 2 * F + Gs;
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(Rows, N);
		Eigen::VectorXd B(Rows);
		Eigen::VectorXd Cost(N);
		for (Eigen::Index K = 0; K < F; ++K)
		{
			Cost(K) = std::max(0.0, Boxes(Cap.FirstSlack + K) - Cap.Bound);
			Cost(F + K) = Cap.Bound;
		}
		Eigen::Index Row = 0;
		for (Eigen::Index W = 0; W < Gs; ++W)
		{
			const Group& Gr = P.Groups[Cap.Groups[static_cast<std::size_t>(W)]];
			Cost(2 * F + W) = Gr.Bound;
			for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
			{
				for (Eigen::Index K = 0; K < F; ++K)
				{
					const double Coefficient = Dot(Cap.Facets[static_cast<std::size_t>(K)], P.Directions[J]);
					A(Row, K) = Coefficient;
					A(Row, F + K) = -Coefficient;
				}
				A(Row, 2 * F + W) = -1.0;
				B(Row) = -Reduced(static_cast<Eigen::Index>(J));
				++Row;
			}
		}
		Eigen::VectorXd X;
		if (!FacetDualSimplex(Cost, A, B, X, PivotLimit, FacetBlandAfter))
		{
			continue;
		}
		for (Eigen::Index K = 0; K < F; ++K)
		{
			Y(Cap.FirstRow + K) = X(K) - X(F + K);
		}
	}
}

double RigorousDualBound(const Component& C, const Program& P, const Eigen::VectorXd& Bounds, const Embedding& E, int FacetLimit = FacetPivotLimit)
{
	if (!(E.Tau > 0.0))
	{
		return Unbounded;
	}
	const Eigen::VectorXd Given = E.Y / E.Tau;
	const double Unpolished = BoundAtDuals(C, P, Bounds, Given);
	if (!Given.allFinite())
	{
		return Unpolished;
	}
	Eigen::VectorXd Y = Given;
	PolishFacets(C, P, Y, FacetLimit);
	PolishGroupsAndCap(C, P, Y);
	return std::min(Unpolished, BoundAtDuals(C, P, Bounds, Y));
}

Outcome SolveProgram(const Component& C, const Program& P, const CertificateOptions& Options)
{
	Outcome Out;
	const Eigen::VectorXd Bounds = RowBounds(C, P);
	Embedding E = StartingPoint(C, P);
	const double PairCount = static_cast<double>(P.Variables + 1);
	SparseNormal Sparse;
	const bool bSparse = C.Rows > Options.DenseRowLimit;
	Normal N;
	double WindowBest = 0.0;
	double WindowMu = 0.0;
	int WindowStart = 0;
	const auto Finish = [&]() {
		Out.DualBound = RigorousDualBound(C, P, Bounds, E);
		if (E.Tau > 0.0)
		{
			Out.LastY = E.Y.head(C.Rows) / E.Tau;
			Out.X = E.X / E.Tau;
			Out.AllY = E.Y / E.Tau;
		}
		return Out;
	};
	for (int Iteration = 0;; ++Iteration)
	{
		const Residuals R = ComputeResiduals(C, P, E, Bounds);
		bool bFinite = false;
		double Largest = 0.0;
		const bool bConverged = IsConverged(C, P, E, R, Options.FeasibilityTolerance, bFinite, Largest);
		Out.Iterations = Iteration;
		if (!bFinite)
		{
			return Finish();
		}
		if (bConverged)
		{
			Out.bConverged = true;
			Out.LoadFactor = E.X.data()[P.LoadFactor] / E.Tau;
			Out.Y = E.Y.head(C.Rows) / E.Tau;
			return Finish();
		}
		if (Iteration >= Options.MaxIterations)
		{
			return Finish();
		}
		const double Mu = Complementarity(E) / PairCount;
		if (Iteration == 0 || Largest < Options.StallImprovement * WindowBest || Mu < ComplementarityRestart * WindowMu)
		{
			WindowBest = Largest;
			WindowMu = Mu;
			WindowStart = Iteration;
		}
		else if (Options.StallWindow > 0 && Iteration - WindowStart >= Options.StallWindow)
		{
			return Finish();
		}
		N.Sparse = bSparse ? &Sparse : nullptr;
		N.bShifted = false;
		N.D = Eigen::VectorXd(P.Variables);
		for (Eigen::Index I = 0; I < P.Variables; ++I)
		{
			N.D.data()[I] = E.X.data()[I] / E.S.data()[I];
		}
		if (!FactorNormal(C, P, N))
		{
			return Finish();
		}
		NewtonContext Ctx;
		Ctx.C = &C;
		Ctx.P = &P;
		Ctx.N = &N;
		Ctx.Bounds = &Bounds;
		PrepareContext(C, P, E, Ctx);
		Eigen::VectorXd R4(P.Variables);
		for (Eigen::Index I = 0; I < P.Variables; ++I)
		{
			R4.data()[I] = -E.X.data()[I] * E.S.data()[I];
		}
		const Direction Affine = SolveNewton(Ctx, E, R, 1.0, R4, -E.Tau * E.Kappa, N.bShifted ? ShiftedRefinementPasses : 0);
		const double AffineMu = ComplementarityAfter(E, Affine, std::min(1.0, MaxStep(E, Affine))) / PairCount;
		const double Ratio = AffineMu / Mu;
		const double Centring = Ratio * Ratio * Ratio;
		for (Eigen::Index I = 0; I < P.Variables; ++I)
		{
			R4.data()[I] += Centring * Mu - Affine.X.data()[I] * Affine.S.data()[I];
		}
		const double R5 = Centring * Mu - E.Tau * E.Kappa - Affine.Tau * Affine.Kappa;
		const Direction Dir = SolveNewton(Ctx, E, R, 1.0 - Centring, R4, R5, N.bShifted ? ShiftedRefinementPasses : RefinementPasses);
		const double Step = std::min(1.0, StepToBoundary * MaxStep(E, Dir));
		Advance(E, Dir, Step);
	}
}

void FillMechanism(const Component& C, const Eigen::VectorXd& Y, ComponentVerdict& V)
{
	Eigen::VectorXd Motion = -Y;
	for (int Offset = 0; Offset < C.Rows; Offset += DofPerBody)
	{
		Motion.segment(Offset + 3, 3) /= C.LengthM;
	}
	const double Largest = Motion.cwiseAbs().maxCoeff();
	if (!(Largest > 0.0))
	{
		return;
	}
	const Eigen::VectorXd Unit = Motion / Largest;
	std::vector<bool> Listed(C.FreeBodies.size(), false);
	for (std::size_t K = 0; K < C.FreeBodies.size(); ++K)
	{
		const int O = static_cast<int>(K) * DofPerBody;
		if (!(Unit.segment(O, DofPerBody).cwiseAbs().maxCoeff() > UnconstrainedComponent))
		{
			continue;
		}
		Listed[K] = true;
		V.Moving.push_back(C.FreeBodies[K]);
		V.Mechanism.push_back(MechanismMotion{C.FreeBodies[K], Vec3{Unit(O + 0), Unit(O + 1), Unit(O + 2)}, Vec3{Unit(O + 3), Unit(O + 4), Unit(O + 5)}});
	}
	std::vector<double> Speeds;
	Speeds.reserve(C.Sites.size());
	for (const Site& S : C.Sites)
	{
		Speeds.push_back(Length(RelativeVelocity(S, Y.data())));
	}
	double Fastest = 0.0;
	for (const double Speed : Speeds)
	{
		Fastest = std::max(Fastest, Speed);
	}
	const auto IsListed = [&](int Offset) { return Offset >= 0 && Listed[static_cast<std::size_t>(Offset / DofPerBody)]; };
	for (const JointData& J : C.Joints)
	{
		const Site& First = C.Sites[J.FirstSite];
		if (!IsListed(First.OffsetA) && !IsListed(First.OffsetB))
		{
			continue;
		}
		for (std::size_t Corner = 0; Corner < CornerCount; ++Corner)
		{
			if (Speeds[J.FirstSite + Corner] > UnconstrainedComponent * Fastest)
			{
				V.Hinging.push_back(J.Id);
				break;
			}
		}
	}
}

struct Selection
{
	const Component* C = nullptr;
	const Program* P = nullptr;
	Eigen::Index Velocities = 0;
	Eigen::Index GenRows = 0;
	Eigen::Index RowCount = 0;
	bool bOpening = false;
	double OpeningBound = 0.0;
};

Selection BuildSelection(const Component& C, const Program& P, double OpeningBound)
{
	Selection Sel;
	Sel.C = &C;
	Sel.P = &P;
	Sel.Velocities = C.Rows;
	Sel.GenRows = static_cast<Eigen::Index>(P.Directions.size());
	Sel.bOpening = P.Offset.cwiseAbs().maxCoeff() > 0.0;
	Sel.OpeningBound = OpeningBound;
	Sel.RowCount = Sel.GenRows + 2 * Sel.Velocities + (Sel.bOpening ? 1 : 0);
	return Sel;
}

Eigen::VectorXd Multiply(const Selection& Sel, const Eigen::VectorXd& Z)
{
	const Component& C = *Sel.C;
	const Program& P = *Sel.P;
	const Eigen::Index N = Sel.Velocities;
	Eigen::VectorXd Out(Sel.RowCount);
	double* O = Out.data();
	const double* Zp = Z.data();
	for (const Group& Gr : P.Groups)
	{
		const Vec3 V = RelativeVelocity(C.Sites[Gr.SiteIndex], Zp);
		for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
		{
			O[J] = Dot(P.Directions[J], V);
		}
	}
	for (Eigen::Index I = 0; I < N; ++I)
	{
		O[Sel.GenRows + I] = Zp[I] - Zp[N + I];
		O[Sel.GenRows + N + I] = -Zp[I] - Zp[N + I];
	}
	if (Sel.bOpening)
	{
		double Sum = 0.0;
		for (Eigen::Index I = 0; I < N; ++I)
		{
			Sum -= P.Offset.data()[I] * Zp[I];
		}
		O[Sel.RowCount - 1] = Sum;
	}
	return Out;
}

Eigen::VectorXd MultiplyTranspose(const Selection& Sel, const Eigen::VectorXd& L)
{
	const Component& C = *Sel.C;
	const Program& P = *Sel.P;
	const Eigen::Index N = Sel.Velocities;
	Eigen::VectorXd Out = Eigen::VectorXd::Zero(2 * N);
	double* O = Out.data();
	const double* Lp = L.data();
	for (const Group& Gr : P.Groups)
	{
		Vec3 Force;
		for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
		{
			Force = statics::Add(Force, Scale(Lp[J], P.Directions[J]));
		}
		ApplyCornerForce(O, C.Sites[Gr.SiteIndex], Force);
	}
	for (Eigen::Index I = 0; I < N; ++I)
	{
		const double Plus = Lp[Sel.GenRows + I];
		const double Minus = Lp[Sel.GenRows + N + I];
		O[I] += Plus - Minus;
		O[N + I] -= Plus + Minus;
	}
	if (Sel.bOpening)
	{
		const double Open = Lp[Sel.RowCount - 1];
		for (Eigen::Index I = 0; I < N; ++I)
		{
			O[I] -= Open * P.Offset.data()[I];
		}
	}
	return Out;
}

struct SelectionSystem
{
	const Selection* Sel = nullptr;
	Eigen::LLT<Eigen::MatrixXd> Factor;
	Eigen::VectorXd D;
	Eigen::VectorXd Coupling;
	Eigen::VectorXd Diagonal;
	Eigen::VectorXd MinvWork;
	double WorkMinvWork = 0.0;
};

bool FactorSelection(SelectionSystem& Sys)
{
	const Selection& Sel = *Sys.Sel;
	const Component& C = *Sel.C;
	const Program& P = *Sel.P;
	const Eigen::Index N = Sel.Velocities;
	const double* D = Sys.D.data();
	std::vector<Matrix3> SiteMatrix(C.Sites.size(), Matrix3{});
	for (const Group& Gr : P.Groups)
	{
		Matrix3& K = SiteMatrix[Gr.SiteIndex];
		for (std::size_t J = Gr.First; J < Gr.First + Gr.Count; ++J)
		{
			const std::array<double, 3> Dir{P.Directions[J].X, P.Directions[J].Y, P.Directions[J].Z};
			for (std::size_t R = 0; R < 3; ++R)
			{
				for (std::size_t Col = 0; Col < 3; ++Col)
				{
					K[R * 3 + Col] += D[J] * Dir[R] * Dir[Col];
				}
			}
		}
	}
	Eigen::MatrixXd M = AssembleSites(C, SiteMatrix);
	Sys.Coupling = Eigen::VectorXd(N);
	Sys.Diagonal = Eigen::VectorXd(N);
	for (Eigen::Index I = 0; I < N; ++I)
	{
		const double Plus = D[Sel.GenRows + I];
		const double Minus = D[Sel.GenRows + N + I];
		Sys.Coupling(I) = Minus - Plus;
		Sys.Diagonal(I) = Plus + Minus;
		M(I, I) += 4.0 * Plus * Minus / (Plus + Minus);
	}
	if (Sel.bOpening)
	{
		M.noalias() += D[Sel.RowCount - 1] * P.Offset * P.Offset.transpose();
	}
	Sys.Factor.compute(M);
	if (!HasPositivePivots(Sys.Factor) && !FactorWithRetryShifts(M, Sys.Factor))
	{
		return false;
	}
	return true;
}

Eigen::VectorXd SolveSelectionNormal(const SelectionSystem& Sys, const Eigen::VectorXd& Q)
{
	const Eigen::Index N = Sys.Sel->Velocities;
	Eigen::VectorXd Out(2 * N);
	Eigen::VectorXd Velocity(N);
	for (Eigen::Index I = 0; I < N; ++I)
	{
		Velocity(I) = Q(I) - Sys.Coupling(I) * Q(N + I) / Sys.Diagonal(I);
	}
	Sys.Factor.solveInPlace(Velocity);
	for (Eigen::Index I = 0; I < N; ++I)
	{
		Out(I) = Velocity(I);
		Out(N + I) = (Q(N + I) - Sys.Coupling(I) * Velocity(I)) / Sys.Diagonal(I);
	}
	return Out;
}

struct SelectionState
{
	Eigen::VectorXd Z;
	Eigen::VectorXd S;
	Eigen::VectorXd L;
	double Nu = 0.0;
	double Tau = 1.0;
	double Kappa = 1.0;
};

struct SelectionResiduals
{
	Eigen::VectorXd Dual;
	Eigen::VectorXd Primal;
	double Work = 0.0;
	double Gap = 0.0;
};

struct SelectionProblem
{
	const Selection* Sel = nullptr;
	Eigen::VectorXd H;
	Eigen::VectorXd Cost;
	Eigen::VectorXd Work;
};

SelectionResiduals SelectionResidualsAt(const SelectionProblem& Pr, const SelectionState& X)
{
	SelectionResiduals R;
	R.Dual = MultiplyTranspose(*Pr.Sel, X.L) - X.Nu * Pr.Work + X.Tau * Pr.Cost;
	R.Primal = -Multiply(*Pr.Sel, X.Z) + X.Tau * Pr.H - X.S;
	R.Work = DotProduct(Pr.Work, X.Z) - X.Tau;
	R.Gap = -DotProduct(Pr.Cost, X.Z) - DotProduct(Pr.H, X.L) + X.Nu - X.Kappa;
	return R;
}

struct SaddleSolution
{
	Eigen::VectorXd Z;
	double Nu = 0.0;
};

SaddleSolution SolveSaddle(const SelectionSystem& Sys, const Eigen::VectorXd& Work, const Eigen::VectorXd& Top, double Bottom)
{
	SaddleSolution Out;
	const Eigen::VectorXd MinvTop = SolveSelectionNormal(Sys, Top);
	Out.Nu = (Bottom - DotProduct(Work, MinvTop)) / Sys.WorkMinvWork;
	Out.Z = MinvTop + Out.Nu * Sys.MinvWork;
	return Out;
}

SelectionState SolveSelectionLinear(const SelectionSystem& Sys, const SelectionProblem& Pr, const SelectionState& X, const SelectionResiduals& Target, const Eigen::VectorXd& Rc, double Rk, const SaddleSolution& TauColumn, double TauDenominatorBase)
{
	const Selection& Sel = *Sys.Sel;
	const Eigen::VectorXd Shifted = (Rc + X.L.cwiseProduct(Target.Primal)).cwiseQuotient(X.S);
	const Eigen::VectorXd Top = Target.Dual - MultiplyTranspose(Sel, Shifted);
	const SaddleSolution Base = SolveSaddle(Sys, Pr.Work, Top, Target.Work);
	const Eigen::VectorXd DGBase = Sys.D.cwiseProduct(Multiply(Sel, Base.Z));
	const double Numerator = Target.Gap + DotProduct(Pr.Cost, Base.Z) + DotProduct(Pr.H, Shifted) + DotProduct(Pr.H, DGBase) - Base.Nu + Rk / X.Tau;
	SelectionState Step;
	Step.Tau = Numerator / (TauDenominatorBase + X.Kappa / X.Tau);
	Step.Z = Base.Z + Step.Tau * TauColumn.Z;
	Step.Nu = Base.Nu + Step.Tau * TauColumn.Nu;
	Step.L = Shifted + Sys.D.cwiseProduct(Multiply(Sel, Step.Z) - Step.Tau * Pr.H);
	Step.S = (Rc - X.S.cwiseProduct(Step.L)).cwiseQuotient(X.L);
	Step.Kappa = (Rk - X.Kappa * Step.Tau) / X.Tau;
	return Step;
}

SelectionState SolveSelectionNewton(const SelectionSystem& Sys, const SelectionProblem& Pr, const SelectionState& X, const SelectionResiduals& R, double Reduction, const Eigen::VectorXd& Rc, double Rk, const SaddleSolution& TauColumn, double TauDenominatorBase, int Passes)
{
	SelectionResiduals Target;
	Target.Dual = -Reduction * R.Dual;
	Target.Primal = -Reduction * R.Primal;
	Target.Work = -Reduction * R.Work;
	Target.Gap = -Reduction * R.Gap;
	SelectionState Step = SolveSelectionLinear(Sys, Pr, X, Target, Rc, Rk, TauColumn, TauDenominatorBase);
	const Eigen::VectorXd NoComplementarity = Eigen::VectorXd::Zero(Rc.size());
	for (int Pass = 0; Pass < Passes; ++Pass)
	{
		const SelectionResiduals Achieved = SelectionResidualsAt(Pr, Step);
		SelectionResiduals Error;
		Error.Dual = Target.Dual - Achieved.Dual;
		Error.Primal = Target.Primal - Achieved.Primal;
		Error.Work = Target.Work - Achieved.Work;
		Error.Gap = Target.Gap - Achieved.Gap;
		const SelectionState Fix = SolveSelectionLinear(Sys, Pr, X, Error, NoComplementarity, 0.0, TauColumn, TauDenominatorBase);
		Step.Z += Fix.Z;
		Step.S += Fix.S;
		Step.L += Fix.L;
		Step.Nu += Fix.Nu;
		Step.Tau += Fix.Tau;
		Step.Kappa += Fix.Kappa;
	}
	return Step;
}

double BoundaryStep(const Eigen::VectorXd& V, const Eigen::VectorXd& Delta)
{
	double Limit = Unbounded;
	for (Eigen::Index I = 0; I < V.size(); ++I)
	{
		if (Delta(I) < 0.0)
		{
			Limit = std::min(Limit, -V(I) / Delta(I));
		}
	}
	return Limit;
}

double SelectionMaxStep(const SelectionState& X, const SelectionState& Step)
{
	double Limit = std::min(BoundaryStep(X.S, Step.S), BoundaryStep(X.L, Step.L));
	if (Step.Tau < 0.0)
	{
		Limit = std::min(Limit, -X.Tau / Step.Tau);
	}
	if (Step.Kappa < 0.0)
	{
		Limit = std::min(Limit, -X.Kappa / Step.Kappa);
	}
	return Limit;
}

double SelectionComplementarity(const SelectionState& X, const SelectionState& Step, double Alpha)
{
	return DotProduct(X.S + Alpha * Step.S, X.L + Alpha * Step.L) + (X.Tau + Alpha * Step.Tau) * (X.Kappa + Alpha * Step.Kappa);
}

bool SolveSelection(const Selection& Sel, const CertificateOptions& Options, Eigen::VectorXd& Velocity)
{
	const Eigen::Index N = Sel.Velocities;
	const Eigen::Index RowCount = Sel.RowCount;
	SelectionProblem Pr;
	Pr.Sel = &Sel;
	Pr.H = Eigen::VectorXd::Zero(RowCount);
	if (Sel.bOpening)
	{
		Pr.H(RowCount - 1) = Sel.OpeningBound;
	}
	Pr.Cost = Eigen::VectorXd::Zero(2 * N);
	Pr.Cost.head(N) = -Sel.C->Demand;
	Pr.Work = Eigen::VectorXd::Zero(2 * N);
	Pr.Work.tail(N).setOnes();
	SelectionState X;
	X.Z = Eigen::VectorXd::Zero(2 * N);
	X.S = Eigen::VectorXd::Ones(RowCount);
	X.L = Eigen::VectorXd::Ones(RowCount);
	const double Tolerance = Options.FeasibilityTolerance;
	const double BoundScale = 1.0 + Sel.OpeningBound;
	const double WorkScale = 1.0 + LargestMagnitude(Pr.Cost.data(), Pr.Cost.size());
	const double PairCount = static_cast<double>(RowCount + 1);
	for (int Iteration = 0; Iteration <= Options.MaxIterations; ++Iteration)
	{
		const SelectionResiduals R = SelectionResidualsAt(Pr, X);
		const double PrimalRelative = LargestMagnitude(R.Primal.data(), RowCount) / X.Tau;
		const double DualRelative = LargestMagnitude(R.Dual.data(), R.Dual.size()) / X.Tau;
		const double Objective = DotProduct(Pr.Cost, X.Z) / X.Tau;
		const double GapRelative = DotProduct(X.S, X.L) / (X.Tau * X.Tau);
		if (!(IsFinite(PrimalRelative) && IsFinite(DualRelative) && IsFinite(Objective) && IsFinite(GapRelative)))
		{
			return false;
		}
		if (PrimalRelative <= Tolerance * BoundScale && DualRelative <= Tolerance * WorkScale && std::abs(R.Work) / X.Tau <= Tolerance && GapRelative <= Tolerance * (1.0 + std::abs(Objective)))
		{
			Velocity = X.Z.head(N) / X.Tau;
			return true;
		}
		if (Iteration == Options.MaxIterations || !(X.Tau > Tolerance * X.Kappa))
		{
			return false;
		}
		SelectionSystem Sys;
		Sys.Sel = &Sel;
		Sys.D = X.L.cwiseQuotient(X.S);
		if (!FactorSelection(Sys))
		{
			return false;
		}
		Sys.MinvWork = SolveSelectionNormal(Sys, Pr.Work);
		Sys.WorkMinvWork = DotProduct(Pr.Work, Sys.MinvWork);
		if (!(Sys.WorkMinvWork > 0.0))
		{
			return false;
		}
		const Eigen::VectorXd DH = Sys.D.cwiseProduct(Pr.H);
		const SaddleSolution TauColumn = SolveSaddle(Sys, Pr.Work, MultiplyTranspose(Sel, DH) - Pr.Cost, 1.0);
		const double TauDenominatorBase = -DotProduct(Pr.Cost, TauColumn.Z) - DotProduct(DH, Multiply(Sel, TauColumn.Z)) + DotProduct(Pr.H, DH) + TauColumn.Nu;
		const double Mu = (DotProduct(X.S, X.L) + X.Tau * X.Kappa) / PairCount;
		const Eigen::VectorXd Product = X.S.cwiseProduct(X.L);
		const SelectionState Affine = SolveSelectionNewton(Sys, Pr, X, R, 1.0, -Product, -X.Tau * X.Kappa, TauColumn, TauDenominatorBase, 0);
		const double AffineMu = SelectionComplementarity(X, Affine, std::min(1.0, SelectionMaxStep(X, Affine))) / PairCount;
		const double Ratio = AffineMu / Mu;
		const double Centring = Ratio * Ratio * Ratio;
		Eigen::VectorXd Rc = -Product - Affine.S.cwiseProduct(Affine.L);
		Rc.array() += Centring * Mu;
		const double Rk = -X.Tau * X.Kappa - Affine.Tau * Affine.Kappa + Centring * Mu;
		const SelectionState Dir = SolveSelectionNewton(Sys, Pr, X, R, 1.0 - Centring, Rc, Rk, TauColumn, TauDenominatorBase, SelectionRefinementPasses);
		const double Step = std::min(1.0, StepToBoundary * SelectionMaxStep(X, Dir));
		X.Z += Step * Dir.Z;
		X.S += Step * Dir.S;
		X.L += Step * Dir.L;
		X.Nu += Step * Dir.Nu;
		X.Tau += Step * Dir.Tau;
		X.Kappa += Step * Dir.Kappa;
	}
	return false;
}

bool IsGrounded(const Component& C)
{
	return std::any_of(C.Sites.begin(), C.Sites.end(), [](const Site& S) { return S.OffsetA < 0 || S.OffsetB < 0; });
}

bool FallRigid(const Component& C, ComponentVerdict& V)
{
	Vec3 Net;
	for (int Offset = 0; Offset < C.Rows; Offset += DofPerBody)
	{
		Net = statics::Add(Net, Vec3{-C.Demand(Offset + 0), -C.Demand(Offset + 1), -C.Demand(Offset + 2)});
	}
	const double Largest = std::max({std::abs(Net.X), std::abs(Net.Y), std::abs(Net.Z)});
	if (!(Largest > 0.0) || !IsFinite(Net))
	{
		return false;
	}
	const Vec3 Direction{Net.X / Largest, Net.Y / Largest, Net.Z / Largest};
	V.State = CertificateState::Collapses;
	V.Lower = 0.0;
	V.Upper = 0.0;
	for (const BodyId Id : C.FreeBodies)
	{
		V.Moving.push_back(Id);
		V.Mechanism.push_back(MechanismMotion{Id, Direction, Vec3{}});
	}
	return true;
}

ComponentVerdict DecideComponent(const Component& C, const CertificateOptions& Options, std::span<const double> Margins)
{
	ComponentVerdict V;
	V.Upper = Options.LoadFactorCap;
	if (!IsGrounded(C) && !C.bHasBodyLoad && FallRigid(C, V))
	{
		return V;
	}
	const Program InnerProgram = BuildProgram(C, Options.ConeGenerators, 1.0, Options.LoadFactorCap, true);
	Outcome Inner = SolveProgram(C, InnerProgram, Options);
	V.Iterations = Inner.Iterations;
	if (!Inner.bConverged && Options.bStallRecovery && Inner.DualBound > 0.0 && Inner.DualBound < Options.LoadFactorCap)
	{
		for (const double Margin : Margins)
		{
			const double CappedAt = Inner.DualBound * (1.0 - Margin);
			const Outcome Capped = SolveProgram(C, BuildProgram(C, Options.ConeGenerators, 1.0, CappedAt, true), Options);
			V.Iterations = std::max(V.Iterations, Capped.Iterations);
			if (Capped.bConverged)
			{
				Inner.bConverged = true;
				Inner.LoadFactor = Capped.LoadFactor;
				Inner.Y = Capped.LoadFactor >= CappedAt * (1.0 - AtCapRelative) ? Inner.LastY : Capped.Y;
				break;
			}
		}
	}
	const bool bInnerFailed = !Inner.bConverged;
	if (bInnerFailed)
	{
		if (!Options.bStallRecovery)
		{
			return V;
		}
		Inner.LoadFactor = 0.0;
	}
	double OuterLoadFactor = Options.LoadFactorCap;
	if (Inner.LoadFactor < Options.LoadFactorCap * (1.0 - Options.FeasibilityTolerance))
	{
		const double Circumscribed = 1.0 / std::cos(Pi / static_cast<double>(Options.ConeGenerators));
		const Outcome Outer = SolveProgram(C, BuildProgram(C, Options.ConeGenerators, Circumscribed, Options.LoadFactorCap, false), Options);
		V.Iterations = std::max(V.Iterations, Outer.Iterations);
		if (Outer.bConverged)
		{
			OuterLoadFactor = Outer.LoadFactor;
			if (OuterLoadFactor < 1.0 && !(Outer.DualBound < 1.0))
			{
				OuterLoadFactor = std::min(Options.LoadFactorCap, Outer.DualBound);
			}
		}
		else if (Options.bStallRecovery && Outer.DualBound < Options.LoadFactorCap)
		{
			OuterLoadFactor = Outer.DualBound;
		}
		else if (!(Inner.LoadFactor >= 1.0))
		{
			return V;
		}
	}
	V.Lower = Inner.LoadFactor;
	V.Upper = std::max(OuterLoadFactor, Inner.LoadFactor);
	if (V.Lower >= 1.0)
	{
		V.State = CertificateState::Stands;
	}
	else if (V.Upper < 1.0)
	{
		V.State = CertificateState::Collapses;
		Eigen::VectorXd Velocity = Inner.Y;
		if (Inner.LoadFactor < Options.FeasibilityTolerance)
		{
			if (!SolveSelection(BuildSelection(C, InnerProgram, Options.FeasibilityTolerance), Options, Velocity))
			{
				V.State = CertificateState::Undecided;
				return V;
			}
			if (bInnerFailed && !HasPositiveRelativeWork(C, Velocity.data()))
			{
				V.State = CertificateState::Undecided;
				return V;
			}
		}
		FillMechanism(C, Velocity, V);
	}
	return V;
}

int Severity(CertificateState State)
{
	switch (State)
	{
	case CertificateState::Stands: return 0;
	case CertificateState::Collapses: return 1;
	case CertificateState::Undecided: return 2;
	case CertificateState::InvalidInput: return 3;
	}
	return 3;
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

bool ProbedComponent(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::size_t Index, Component& Out)
{
	if (!IsInputValid(Model, Loads, Options))
	{
		return false;
	}
	std::vector<Component> Components = BuildComponents(Model, Loads, Options.Gravity, {});
	if (Index >= Components.size())
	{
		return false;
	}
	Out = std::move(Components[Index]);
	return true;
}

Program ProbedProgram(const Component& C, const CertificateOptions& Options, bool bInscribed)
{
	const double RadiusScale = bInscribed ? 1.0 : 1.0 / std::cos(Pi / static_cast<double>(Options.ConeGenerators));
	return BuildProgram(C, Options.ConeGenerators, RadiusScale, Options.LoadFactorCap, bInscribed);
}

std::vector<double> ToVector(const Eigen::VectorXd& V)
{
	return std::vector<double>(V.data(), V.data() + V.size());
}

CertificateResult DecideAll(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const BodyId> Only, std::span<const double> Margins)
{
	CertificateResult Result;
	std::vector<bool> Chosen;
	if (!IsInputValid(Model, Loads, Options) || !ChooseComponents(Model, Only, Chosen))
	{
		Result.State = CertificateState::InvalidInput;
		return Result;
	}
	Result.State = CertificateState::Stands;
	Result.LoadFactorLower = Options.LoadFactorCap;
	Result.LoadFactorUpper = Options.LoadFactorCap;
	const std::vector<Component> Components = BuildComponents(Model, Loads, Options.Gravity, Chosen);
	for (std::size_t K = 0; K < Components.size(); ++K)
	{
		if (!Chosen.empty() && !Chosen[K])
		{
			continue;
		}
		const Component& C = Components[K];
		const ComponentVerdict V = DecideComponent(C, Options, Margins);
		if (Severity(V.State) > Severity(Result.State))
		{
			Result.State = V.State;
		}
		Result.LoadFactorLower = std::min(Result.LoadFactorLower, V.Lower);
		Result.LoadFactorUpper = std::min(Result.LoadFactorUpper, V.Upper);
		Result.Iterations = std::max(Result.Iterations, V.Iterations);
		Result.Mechanism.insert(Result.Mechanism.end(), V.Mechanism.begin(), V.Mechanism.end());
		Result.UnconstrainedBodies.insert(Result.UnconstrainedBodies.end(), V.Moving.begin(), V.Moving.end());
		Result.HingingInterfaces.insert(Result.HingingInterfaces.end(), V.Hinging.begin(), V.Hinging.end());
		Result.Components.push_back(CertificateComponent{C.FreeBodies, V.State, std::min(Options.LoadFactorCap, V.Lower), std::min(Options.LoadFactorCap, V.Upper), V.Iterations, V.Mechanism, V.Moving, V.Hinging});
	}
	return Result;
}

class LimitAnalysisCertificate final : public ICollapseCertificate
{
public:
	CertificateResult Decide(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const BodyId> Only) const override
	{
		return DecideAll(Model, Loads, Options, Only, RecoveryMargins);
	}
};
}

namespace detail
{
ProgramProbe ProbeProgram(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::size_t Index, bool bInscribed)
{
	ProgramProbe Probe;
	Component C;
	if (!ProbedComponent(Model, Loads, Options, Index, C))
	{
		return Probe;
	}
	const Program P = ProbedProgram(C, Options, bInscribed);
	const Outcome Out = SolveProgram(C, P, Options);
	Probe.bConverged = Out.bConverged;
	Probe.LoadFactor = Out.LoadFactor;
	Probe.DualBound = Out.DualBound;
	Probe.Primal = ToVector(Out.X);
	Probe.Duals = ToVector(Out.AllY);
	Probe.Boxes = ToVector(ColumnBoxes(P));
	return Probe;
}

double ProbeDualBound(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::size_t Index, bool bInscribed, const std::vector<double>& Duals)
{
	Component C;
	if (!ProbedComponent(Model, Loads, Options, Index, C))
	{
		return Unbounded;
	}
	const Program P = ProbedProgram(C, Options, bInscribed);
	if (static_cast<Eigen::Index>(Duals.size()) != P.RowCount)
	{
		return Unbounded;
	}
	Embedding E;
	E.Y = Eigen::Map<const Eigen::VectorXd>(Duals.data(), P.RowCount);
	E.Tau = 1.0;
	return RigorousDualBound(C, P, RowBounds(C, P), E);
}

ExportedProgram ExportProgram(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::size_t Index, bool bInscribed)
{
	ExportedProgram Out;
	Component C;
	if (!ProbedComponent(Model, Loads, Options, Index, C))
	{
		return Out;
	}
	const Program P = ProbedProgram(C, Options, bInscribed);
	Out.Rows = static_cast<int>(P.RowCount);
	Out.Columns = static_cast<int>(P.Variables);
	Out.A.reserve(static_cast<std::size_t>(P.RowCount) * static_cast<std::size_t>(P.Variables));
	Eigen::VectorXd Unit = Eigen::VectorXd::Zero(P.Variables);
	for (Eigen::Index J = 0; J < P.Variables; ++J)
	{
		Unit(J) = 1.0;
		const Eigen::VectorXd Column = Apply(C, P, Unit);
		Out.A.insert(Out.A.end(), Column.data(), Column.data() + Column.size());
		Unit(J) = 0.0;
	}
	Out.B = ToVector(RowBounds(C, P));
	Out.Boxes = ToVector(ColumnBoxes(P));
	Out.LoadFactorColumn = static_cast<int>(P.LoadFactor);
	return Out;
}

bool FacetSimplexForTest(const std::vector<double>& Cost, const std::vector<double>& A, const std::vector<double>& B, int PivotLimit, int BlandAfter, std::vector<double>& X)
{
	const Eigen::Index N = static_cast<Eigen::Index>(Cost.size());
	const Eigen::Index M = static_cast<Eigen::Index>(B.size());
	if (A.size() != static_cast<std::size_t>(M * N))
	{
		return false;
	}
	Eigen::VectorXd Solution;
	if (!FacetDualSimplex(Eigen::Map<const Eigen::VectorXd>(Cost.data(), N), Eigen::Map<const Eigen::MatrixXd>(A.data(), M, N), Eigen::Map<const Eigen::VectorXd>(B.data(), M), Solution, PivotLimit, BlandAfter))
	{
		return false;
	}
	X = ToVector(Solution);
	return true;
}

double ProbeDualBoundWithFacetPivotLimit(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::size_t Index, bool bInscribed, const std::vector<double>& Duals, int PivotLimit)
{
	Component C;
	if (!ProbedComponent(Model, Loads, Options, Index, C))
	{
		return Unbounded;
	}
	const Program P = ProbedProgram(C, Options, bInscribed);
	if (static_cast<Eigen::Index>(Duals.size()) != P.RowCount)
	{
		return Unbounded;
	}
	Embedding E;
	E.Y = Eigen::Map<const Eigen::VectorXd>(Duals.data(), P.RowCount);
	E.Tau = 1.0;
	return RigorousDualBound(C, P, RowBounds(C, P), E, PivotLimit);
}

CertificateResult DecideWithRecoveryMargins(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const double> Margins)
{
	return DecideAll(Model, Loads, Options, {}, Margins);
}

bool FailedInnerMechanismIsAdmissible(const double* Demand, const double* Velocity, int Rows)
{
	double Work = 0.0;
	double Scale = 0.0;
	double MotionL1 = 0.0;
	for (int R = 0; R < Rows; ++R)
	{
		MotionL1 += std::abs(Velocity[R]);
		Work += Demand[R] * Velocity[R];
		Scale += std::abs(Demand[R] * Velocity[R]);
	}
	return MotionL1 >= FailedInnerMinimumMotionL1 && Scale > 0.0 && Work > FailedInnerMinimumRelativeWork * Scale;
}
}

std::unique_ptr<ICollapseCertificate> MakeLimitAnalysisCertificate()
{
	return std::make_unique<LimitAnalysisCertificate>();
}
}
