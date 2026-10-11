#include "StaticsCommon.h"

namespace loadpath::statics
{
namespace
{
constexpr std::array<double, CornerCount> CornerSignU{1.0, -1.0, -1.0, 1.0};
constexpr std::array<double, CornerCount> CornerSignV{1.0, 1.0, -1.0, -1.0};

std::size_t FindRoot(std::vector<std::size_t>& Parent, std::size_t Slot)
{
	while (Parent[Slot] != Slot)
	{
		Parent[Slot] = Parent[Parent[Slot]];
		Slot = Parent[Slot];
	}
	return Slot;
}

void Unite(std::vector<std::size_t>& Parent, std::size_t SlotA, std::size_t SlotB)
{
	const std::size_t RootA = FindRoot(Parent, SlotA);
	const std::size_t RootB = FindRoot(Parent, SlotB);
	Parent[std::max(RootA, RootB)] = std::min(RootA, RootB);
}
}

std::array<Vec3, CornerCount> CornerPoints(const InterfaceRecord& Record)
{
	std::array<Vec3, CornerCount> Corners{};
	for (std::size_t C = 0; C < CornerCount; ++C)
	{
		Corners[C] = Add(Record.CentreM, Add(Scale(CornerSignU[C] * Record.HalfExtentUM, Record.AxisU), Scale(CornerSignV[C] * Record.HalfExtentVM, Record.AxisV)));
	}
	return Corners;
}

JointGeometry MakeJointGeometry(const Structure& Model, InterfaceId Id, const std::vector<int>& OffsetOfSlot)
{
	const InterfaceRecord& Record = Model.Interface(Id);
	JointGeometry J;
	J.Id = Id;
	J.Normal = Record.NormalAToB;
	J.CentreM = Record.CentreM;
	J.CornerM = CornerPoints(Record);
	J.OffsetA = OffsetOfSlot[Record.BodyA.Index];
	J.OffsetB = OffsetOfSlot[Record.BodyB.Index];
	J.CentroidAM = Model.Body(Record.BodyA).CentroidM;
	J.CentroidBM = Model.Body(Record.BodyB).CentroidM;
	return J;
}

ComponentPartition PartitionFreeBodies(const Structure& Model)
{
	const std::vector<BodyId> Bodies = Model.LiveBodies();
	const std::size_t SlotCount = Bodies.empty() ? 0 : static_cast<std::size_t>(Bodies.back().Index) + 1;
	std::vector<std::size_t> Parent(SlotCount);
	for (std::size_t Slot = 0; Slot < SlotCount; ++Slot)
	{
		Parent[Slot] = Slot;
	}
	for (const InterfaceId Id : Model.LiveInterfaces())
	{
		const InterfaceRecord& Record = Model.Interface(Id);
		if (!Model.Body(Record.BodyA).bGrounded && !Model.Body(Record.BodyB).bGrounded)
		{
			Unite(Parent, Record.BodyA.Index, Record.BodyB.Index);
		}
	}
	ComponentPartition Partition;
	Partition.ComponentOfSlot.assign(SlotCount, -1);
	Partition.OffsetOfSlot.assign(SlotCount, -1);
	std::vector<int> ComponentOfRoot(SlotCount, -1);
	for (const BodyId Id : Bodies)
	{
		if (Model.Body(Id).bGrounded)
		{
			continue;
		}
		const std::size_t Root = FindRoot(Parent, Id.Index);
		if (ComponentOfRoot[Root] < 0)
		{
			ComponentOfRoot[Root] = static_cast<int>(Partition.FreeBodies.size());
			Partition.FreeBodies.emplace_back();
		}
		const int Component = ComponentOfRoot[Root];
		std::vector<BodyId>& Members = Partition.FreeBodies[static_cast<std::size_t>(Component)];
		Partition.ComponentOfSlot[Id.Index] = Component;
		Partition.OffsetOfSlot[Id.Index] = DofPerBody * static_cast<int>(Members.size());
		Members.push_back(Id);
	}
	return Partition;
}

int ComponentOf(const ComponentPartition& Partition, const InterfaceRecord& Record)
{
	return std::max(Partition.ComponentOfSlot[Record.BodyA.Index], Partition.ComponentOfSlot[Record.BodyB.Index]);
}

std::array<double, 18> CornerJacobianRows(const Vec3& ArmM)
{
	return std::array<double, 18>{
		1.0, 0.0, 0.0, 0.0, ArmM.Z, -ArmM.Y,
		0.0, 1.0, 0.0, -ArmM.Z, 0.0, ArmM.X,
		0.0, 0.0, 1.0, ArmM.Y, -ArmM.X, 0.0};
}

Eigen::Matrix<double, 3, 6> CornerJacobian(const Vec3& ArmM)
{
	const std::array<double, 18> Rows = CornerJacobianRows(ArmM);
	return Eigen::Map<const Eigen::Matrix<double, 3, 6, Eigen::RowMajor>>(Rows.data());
}

void AddForceAt(Eigen::VectorXd& Load, int Offset, const Vec3& CentroidM, const Vec3& PointM, const Vec3& ForceN)
{
	if (Offset < 0)
	{
		return;
	}
	const Vec3 MomentNm = Cross(Sub(PointM, CentroidM), ForceN);
	Load(Offset + 0) += ForceN.X;
	Load(Offset + 1) += ForceN.Y;
	Load(Offset + 2) += ForceN.Z;
	Load(Offset + 3) += MomentNm.X;
	Load(Offset + 4) += MomentNm.Y;
	Load(Offset + 5) += MomentNm.Z;
}

bool AreCertificateOptionsValid(const CertificateOptions& Options)
{
	const auto IsPositiveFinite = [](double X) { return X > 0.0 && IsFinite(X); };
	return IsFinite(Options.Gravity) && Options.ConeGenerators >= 4 && Options.ConeGenerators % 2 == 0 &&
		IsPositiveFinite(Options.LoadFactorCap) && IsPositiveFinite(Options.FeasibilityTolerance) && Options.MaxIterations >= 1;
}
}
