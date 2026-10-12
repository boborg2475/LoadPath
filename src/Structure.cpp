#include "loadpath/Structure.h"

#include "StaticsCommon.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>

namespace loadpath
{
namespace
{
constexpr std::uint32_t FirstGeneration = 1;
constexpr double AxisTolerance = 1.0e-9;
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

constexpr BodyDesc DeadBody{.MassKg = NaN};
constexpr InterfaceRecord DeadInterface{.State = InterfaceState::Removed};
constexpr StrengthProfile DeadProfile{
	.CompressiveStrengthPa = NaN,
	.ShearCohesionPa = NaN,
	.TensileStrengthPa = NaN,
	.FrictionCoefficient = NaN,
	.MaxShearStrengthPa = NaN,
	.NormalStiffnessPaPerM = NaN,
	.TangentialStiffnessPaPerM = NaN};

bool IsWithin(double Value, double Tolerance)
{
	return std::abs(Value) <= Tolerance;
}

bool IsFiniteOrPositiveInfinity(double X)
{
	return IsFinite(X) || X == std::numeric_limits<double>::infinity();
}

bool IsLiveState(InterfaceState State)
{
	return State == InterfaceState::Intact || State == InterfaceState::Cracked;
}

bool IsNotNegative(double X)
{
	return X >= 0.0;
}

Vec3 Divided(const Vec3& V, double Divisor)
{
	return Vec3{V.X / Divisor, V.Y / Divisor, V.Z / Divisor};
}

std::atomic<std::uint64_t> NextLineage{1};

std::uint64_t MintLineage()
{
	return NextLineage.fetch_add(1, std::memory_order_relaxed);
}
}

Structure::Structure()
	: LineageToken(MintLineage())
{
}

Structure::Structure(const Structure& Other)
	: Profiles(Other.Profiles)
	, Bodies(Other.Bodies)
	, Interfaces(Other.Interfaces)
	, HighestStamp(Other.HighestStamp)
	, CurrentRevision(Other.CurrentRevision)
	, LineageToken(MintLineage())
	, CachedComponents(Other.CachedComponents)
{
}

Structure& Structure::operator=(const Structure& Other)
{
	if (this != &Other)
	{
		Profiles = Other.Profiles;
		Bodies = Other.Bodies;
		Interfaces = Other.Interfaces;
		HighestStamp = Other.HighestStamp;
		CurrentRevision = Other.CurrentRevision;
		CachedComponents = Other.CachedComponents;
		LineageToken = MintLineage();
	}
	return *this;
}

std::uint64_t Structure::Lineage() const
{
	return LineageToken;
}

Result<ProfileId> Structure::AddProfile(const StrengthProfile& Desc)
{
	const bool bAllFinite = IsFinite(Desc.CompressiveStrengthPa) && IsFinite(Desc.ShearCohesionPa) &&
		IsFinite(Desc.TensileStrengthPa) && IsFinite(Desc.FrictionCoefficient) &&
		IsFiniteOrPositiveInfinity(Desc.MaxShearStrengthPa) && IsFinite(Desc.NormalStiffnessPaPerM) &&
		IsFinite(Desc.TangentialStiffnessPaPerM);
	if (!bAllFinite)
	{
		return {ProfileId::Invalid(), ModelError::NonFiniteInput};
	}
	const bool bNoneNegative = IsNotNegative(Desc.CompressiveStrengthPa) && IsNotNegative(Desc.ShearCohesionPa) &&
		IsNotNegative(Desc.TensileStrengthPa) && IsNotNegative(Desc.FrictionCoefficient) &&
		IsNotNegative(Desc.MaxShearStrengthPa);
	if (!bNoneNegative)
	{
		return {ProfileId::Invalid(), ModelError::NegativeStrength};
	}
	const ProfileId Id{static_cast<std::uint32_t>(Profiles.size()), FirstGeneration};
	Profiles.push_back(Desc);
	return {Id, ModelError::None};
}

Result<BodyId> Structure::AddBody(const BodyDesc& Desc)
{
	if (!IsFinite(Desc.MassKg) || !IsFinite(Desc.CentroidM))
	{
		return {BodyId::Invalid(), ModelError::NonFiniteInput};
	}
	if (!IsNotNegative(Desc.MassKg))
	{
		return {BodyId::Invalid(), ModelError::NegativeMass};
	}
	const BodyId Id{static_cast<std::uint32_t>(Bodies.size()), FirstGeneration};
	CachedComponents.reset();
	Bodies.push_back(BodySlot{Desc, true, ++CurrentRevision});
	return {Id, ModelError::None};
}

ModelError Structure::ValidateInterface(const InterfaceDesc& Desc, Vec3& UnitNormal) const
{
	if (!IsKnown(Desc.BodyA) || !IsKnown(Desc.BodyB))
	{
		return ModelError::UnknownBody;
	}
	if (!IsLive(Desc.BodyA) || !IsLive(Desc.BodyB))
	{
		return ModelError::BodyRemoved;
	}
	if (Desc.BodyA == Desc.BodyB)
	{
		return ModelError::SameBody;
	}
	if (!IsValid(Desc.Profile))
	{
		return ModelError::UnknownProfile;
	}
	if (!IsFinite(Desc.NormalAToB) || !IsFinite(Desc.CentreM) || !IsFinite(Desc.AxisU) || !IsFinite(Desc.AxisV) ||
		!IsFinite(Desc.HalfExtentUM) || !IsFinite(Desc.HalfExtentVM))
	{
		return ModelError::NonFiniteInput;
	}
	const Vec3& Normal = Desc.NormalAToB;
	const double LargestComponent = std::max({std::abs(Normal.X), std::abs(Normal.Y), std::abs(Normal.Z)});
	if (!(LargestComponent > 0.0))
	{
		return ModelError::DegenerateNormal;
	}
	const Vec3 Scaled = Divided(Normal, LargestComponent);
	UnitNormal = Divided(Scaled, Length(Scaled));
	if (!IsWithin(Length(UnitNormal) - 1.0, AxisTolerance))
	{
		return ModelError::DegenerateNormal;
	}
	const bool bOrthonormal = IsWithin(Length(Desc.AxisU) - 1.0, AxisTolerance) &&
		IsWithin(Length(Desc.AxisV) - 1.0, AxisTolerance) && IsWithin(Dot(Desc.AxisU, Desc.AxisV), AxisTolerance) &&
		IsWithin(Dot(Desc.AxisU, UnitNormal), AxisTolerance) && IsWithin(Dot(Desc.AxisV, UnitNormal), AxisTolerance) &&
		Dot(Cross(Desc.AxisU, Desc.AxisV), UnitNormal) > 0.0;
	if (!bOrthonormal)
	{
		return ModelError::AxesNotOrthonormal;
	}
	if (!(Desc.HalfExtentUM > 0.0) || !(Desc.HalfExtentVM > 0.0))
	{
		return ModelError::NonPositiveExtent;
	}
	const double AreaM2 = 4.0 * Desc.HalfExtentUM * Desc.HalfExtentVM;
	if (!IsFinite(AreaM2) || !(AreaM2 > 0.0))
	{
		return ModelError::NonPositiveExtent;
	}
	return ModelError::None;
}

Result<InterfaceId> Structure::AddInterface(const InterfaceDesc& Desc)
{
	Vec3 UnitNormal;
	const ModelError Error = ValidateInterface(Desc, UnitNormal);
	if (Error != ModelError::None)
	{
		return {InterfaceId::Invalid(), Error};
	}
	InterfaceRecord Record;
	Record.BodyA = Desc.BodyA;
	Record.BodyB = Desc.BodyB;
	Record.NormalAToB = UnitNormal;
	Record.CentreM = Desc.CentreM;
	Record.AxisU = Desc.AxisU;
	Record.AxisV = Desc.AxisV;
	Record.HalfExtentUM = Desc.HalfExtentUM;
	Record.HalfExtentVM = Desc.HalfExtentVM;
	Record.AreaM2 = 4.0 * Desc.HalfExtentUM * Desc.HalfExtentVM;
	Record.Profile = Desc.Profile;
	const InterfaceId Id{static_cast<std::uint32_t>(Interfaces.size()), FirstGeneration};
	CachedComponents.reset();
	Interfaces.push_back(InterfaceSlot{Record});
	++CurrentRevision;
	TouchInterface(Record);
	return {Id, ModelError::None};
}

bool Structure::RemoveBody(BodyId Id)
{
	if (!IsLive(Id))
	{
		return false;
	}
	CachedComponents.reset();
	Bodies[Id.Index].bLive = false;
	++CurrentRevision;
	for (InterfaceSlot& Each : Interfaces)
	{
		if ((Each.Record.BodyA == Id || Each.Record.BodyB == Id) && Each.Record.State != InterfaceState::Severed)
		{
			if (IsLiveState(Each.Record.State))
			{
				TouchInterface(Each.Record);
			}
			Each.Record.State = InterfaceState::Removed;
		}
	}
	return true;
}

bool Structure::Crack(InterfaceId Id, std::uint32_t Pass)
{
	if (Pass == 0 || !IsKnown(Id) || Interfaces[Id.Index].Record.State != InterfaceState::Intact)
	{
		return false;
	}
	InterfaceRecord& Record = Interfaces[Id.Index].Record;
	CachedComponents.reset();
	Record.State = InterfaceState::Cracked;
	Record.CrackPass = Pass;
	HighestStamp = std::max(HighestStamp, Pass);
	++CurrentRevision;
	TouchInterface(Record);
	return true;
}

bool Structure::Sever(InterfaceId Id, std::uint32_t Pass)
{
	if (Pass == 0 || !IsLive(Id) || Pass < Interfaces[Id.Index].Record.CrackPass)
	{
		return false;
	}
	InterfaceRecord& Record = Interfaces[Id.Index].Record;
	CachedComponents.reset();
	Record.State = InterfaceState::Severed;
	Record.SeverPass = Pass;
	HighestStamp = std::max(HighestStamp, Pass);
	++CurrentRevision;
	TouchInterface(Record);
	return true;
}

void Structure::TouchInterface(const InterfaceRecord& Record)
{
	for (const BodyId Each : {Record.BodyA, Record.BodyB})
	{
		if (IsLive(Each))
		{
			Bodies[Each.Index].Touched = CurrentRevision;
		}
	}
}

std::uint64_t Structure::Revision() const
{
	return CurrentRevision;
}

std::uint64_t Structure::LastTouched(BodyId Id) const
{
	return IsKnown(Id) ? Bodies[Id.Index].Touched : std::numeric_limits<std::uint64_t>::max();
}

std::uint32_t Structure::HighestPass() const
{
	return HighestStamp;
}

bool Structure::MarkPass(std::uint32_t Pass)
{
	if (Pass == 0)
	{
		return false;
	}
	HighestStamp = std::max(HighestStamp, Pass);
	return true;
}

StrengthProfile Structure::EffectiveProfile(InterfaceId Id) const
{
	const InterfaceRecord& Record = Interface(Id);
	StrengthProfile Effective = Profile(Record.Profile);
	if (IsKnown(Id) && Record.State != InterfaceState::Intact)
	{
		Effective.TensileStrengthPa = 0.0;
		Effective.ShearCohesionPa = 0.0;
	}
	return Effective;
}

bool Structure::IsKnown(BodyId Id) const
{
	return Id.Generation == FirstGeneration && Id.Index < Bodies.size();
}

bool Structure::IsKnown(InterfaceId Id) const
{
	return Id.Generation == FirstGeneration && Id.Index < Interfaces.size();
}

bool Structure::IsLive(BodyId Id) const
{
	return IsKnown(Id) && Bodies[Id.Index].bLive;
}

bool Structure::IsLive(InterfaceId Id) const
{
	return IsKnown(Id) && IsLiveState(Interfaces[Id.Index].Record.State);
}

bool Structure::IsValid(ProfileId Id) const
{
	return Id.Generation == FirstGeneration && Id.Index < Profiles.size();
}

const BodyDesc& Structure::Body(BodyId Id) const
{
	return IsLive(Id) ? Bodies[Id.Index].Desc : DeadBody;
}

const InterfaceRecord& Structure::Interface(InterfaceId Id) const
{
	return IsKnown(Id) ? Interfaces[Id.Index].Record : DeadInterface;
}

const StrengthProfile& Structure::Profile(ProfileId Id) const
{
	return IsValid(Id) ? Profiles[Id.Index] : DeadProfile;
}

std::size_t Structure::LiveBodyCount() const
{
	return LiveBodies().size();
}

std::size_t Structure::LiveInterfaceCount() const
{
	return LiveInterfaces().size();
}

std::vector<BodyId> Structure::LiveBodies() const
{
	std::vector<BodyId> Ids;
	for (std::size_t Index = 0; Index < Bodies.size(); ++Index)
	{
		if (Bodies[Index].bLive)
		{
			Ids.push_back(BodyId{static_cast<std::uint32_t>(Index), FirstGeneration});
		}
	}
	return Ids;
}

std::vector<InterfaceId> Structure::LiveInterfaces() const
{
	std::vector<InterfaceId> Ids;
	for (std::size_t Index = 0; Index < Interfaces.size(); ++Index)
	{
		if (IsLiveState(Interfaces[Index].Record.State))
		{
			Ids.push_back(InterfaceId{static_cast<std::uint32_t>(Index), FirstGeneration});
		}
	}
	return Ids;
}

std::vector<InterfaceId> Structure::InterfacesOf(BodyId Id) const
{
	std::vector<InterfaceId> Ids;
	for (const InterfaceId Each : LiveInterfaces())
	{
		const InterfaceRecord& Record = Interfaces[Each.Index].Record;
		if (Record.BodyA == Id || Record.BodyB == Id)
		{
			Ids.push_back(Each);
		}
	}
	return Ids;
}

const ComponentPartition& Structure::Components() const
{
	if (!CachedComponents)
	{
		CachedComponents = statics::PartitionFreeBodies(*this);
	}
	return *CachedComponents;
}
}
