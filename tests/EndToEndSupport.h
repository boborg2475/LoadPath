#pragma once

#include "ArchFixtures.h"
#include "CatalogueTestSupport.h"
#include "VisualSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/Catalogue.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/Structure.h"
#include "loadpath/StructureFile.h"
#include "loadpath/Units.h"
#include "loadpath/geometry/BoxContacts.h"
#include "loadpath/geometry/RunningBondWall.h"
#include "loadpath/geometry/Warehouse.h"

#include "doctest.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace loadpath::test::e2e
{
inline constexpr double BrickXM = 0.215;
inline constexpr double BrickYM = 0.1025;
inline constexpr double BrickZM = 0.065;
inline constexpr double BrickDensity = 2000.0;
inline constexpr double TimberDensity = 420.0;

using NameSet = std::vector<std::string>;

// What the visuals step draws; Rotation is a unit quaternion (w, x, y, z).
struct SceneBox
{
	std::string Name;
	std::string Material;
	Vec3 CentreM;
	Vec3 HalfExtentsM;
	std::array<double, 4> Rotation{1.0, 0.0, 0.0, 0.0};
	bool bGrounded = false;
	BodyId Id;
	double MassKg = 0.0;
};

struct Scene
{
	Structure Model;
	std::vector<SceneBox> Boxes;
	std::vector<BodyLoad> Loads;
	Vec3 Gravity{0.0, 0.0, -GravityMPerS2};
	NameSet RemovedBeforeRun;
	// Judged once before the first removal, and only when recording pictures.
	std::optional<ComponentVerdict> AsBuilt;
};

// One run kept whole (boxes, removals, gravity, verdict) so the visuals step can record it in one call.
struct CaseRun
{
	std::string_view Slug;
	std::vector<SceneBox> Boxes;
	NameSet RemovedBeforeRun;
	std::optional<ComponentVerdict> Before;
	Vec3 Gravity;
	CascadeVerdict Verdict;
	double ElapsedMs = 0.0;
};

inline const Catalogue& Rows()
{
	static const Catalogue Value = [] {
		const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
		CHECK(Loaded.Ok());
		return Loaded.Value;
	}();
	return Value;
}

inline StrengthProfile JointProfile(std::string_view Name) { return JointOrFail(Rows(), Name).Profile; }

inline StrengthProfile Bearing() { return JointProfile("c24_bearing_perp_to_grain"); }

inline StrengthProfile Ec6Bed() { return JointProfile("clay_gp_mortar_bed_ec6"); }

inline StrengthProfile Ec6Perpend() { return JointProfile("clay_gp_mortar_perpend_ec6"); }

inline StrengthProfile DryBrick()
{
	StrengthProfile Profile = Ec6Bed();
	Profile.ShearCohesionPa = 0.0;
	Profile.TensileStrengthPa = 0.0;
	Profile.FrictionCoefficient = 0.7;
	return Profile;
}

inline StrengthProfile Screwed(int Count, double AreaM2) { return FastenerProfile(FastenerOrFail(Rows(), "ec5_screw_8x60_steel_to_glulam"), Count, AreaM2); }

inline std::string NameIn(const Scene& S, BodyId Id)
{
	for (const SceneBox& B : S.Boxes)
	{
		if (B.Id.Index == Id.Index)
		{
			return B.Name;
		}
	}
	return "#" + std::to_string(Id.Index);
}

inline BodyId SceneBody(const Scene& S, std::string_view Name)
{
	for (const SceneBox& B : S.Boxes)
	{
		if (B.Name == Name)
		{
			return B.Id;
		}
	}
	INFO("no body named " << std::string(Name));
	CHECK(false);
	return BodyId{};
}

inline ComponentVerdict JudgeRemainder(const Scene& S);

inline void RemoveBodyOf(Scene& S, BodyId Id)
{
	if (!S.AsBuilt && VisualsDirFromEnvironment())
	{
		S.AsBuilt = JudgeRemainder(S);
	}
	const std::string Name = NameIn(S, Id);
	INFO("removing " << Name);
	CHECK(S.Model.RemoveBody(Id));
	S.RemovedBeforeRun.push_back(Name);
}

inline void RemoveNamed(Scene& S, std::string_view Name) { RemoveBodyOf(S, SceneBody(S, Name)); }

inline void Record(Scene& S, BodyId Id, std::string Name, std::string Material, const Vec3& HalfExtentsM)
{
	SceneBox B;
	B.Name = std::move(Name);
	B.Material = std::move(Material);
	B.CentreM = S.Model.Body(Id).CentroidM;
	B.HalfExtentsM = HalfExtentsM;
	B.bGrounded = S.Model.Body(Id).bGrounded;
	B.Id = Id;
	B.MassKg = S.Model.Body(Id).MassKg;
	S.Boxes.push_back(std::move(B));
}

struct BoxSpec
{
	std::string Name, Kind, Material;
	Vec3 LoM, HiM;
	double DensityKgPerM3 = 0.0;
	bool bGrounded = false;
};

using ProfilePicker = std::function<std::size_t(const std::string& LowerKind, const std::string& UpperKind, bool bBed)>;

// Joins every pair of touching boxes by FaceContact; Pick maps the two kinds (lower first) and whether the contact is horizontal to a profile index.
class BoxSceneBuilder
{
public:
	std::vector<StrengthProfile> Profiles;
	std::vector<BoxSpec> Specs;

	std::size_t Profile(const StrengthProfile& P)
	{
		Profiles.push_back(P);
		return Profiles.size() - 1;
	}

	void Add(BoxSpec Spec) { Specs.push_back(std::move(Spec)); }

	// Seed 0 keeps the order of Add; any other seed shuffles bodies and contact pairs deterministically.
	Scene Build(const ProfilePicker& Pick, unsigned Seed = 0) const
	{
		Scene S;
		std::vector<ProfileId> ProfileIds;
		for (const StrengthProfile& P : Profiles)
		{
			ProfileIds.push_back(AddProfileOk(S.Model, P));
		}
		std::vector<std::size_t> Order(Specs.size());
		std::iota(Order.begin(), Order.end(), std::size_t{0});
		if (Seed != 0)
		{
			std::mt19937 Rng(Seed);
			std::shuffle(Order.begin(), Order.end(), Rng);
		}
		std::vector<BodyId> Ids(Specs.size());
		std::vector<geometry::Box> Shapes(Specs.size());
		for (const std::size_t I : Order)
		{
			const BoxSpec& Spec = Specs[I];
			const geometry::Box Shape{Vec3{0.5 * (Spec.LoM.X + Spec.HiM.X), 0.5 * (Spec.LoM.Y + Spec.HiM.Y), 0.5 * (Spec.LoM.Z + Spec.HiM.Z)},
				Vec3{0.5 * (Spec.HiM.X - Spec.LoM.X), 0.5 * (Spec.HiM.Y - Spec.LoM.Y), 0.5 * (Spec.HiM.Z - Spec.LoM.Z)}};
			const double VolumeM3 = 8.0 * Shape.HalfExtentsM.X * Shape.HalfExtentsM.Y * Shape.HalfExtentsM.Z;
			const Result<BodyId> Added = S.Model.AddBody(BodyDesc{Spec.bGrounded ? 0.0 : Spec.DensityKgPerM3 * VolumeM3, Shape.CentreM, Spec.bGrounded});
			INFO("body " << Spec.Name);
			CHECK(Added.Ok());
			Ids[I] = Added.Value;
			Shapes[I] = Shape;
			Record(S, Added.Value, Spec.Name, Spec.Material, Shape.HalfExtentsM);
		}
		std::vector<std::pair<std::size_t, std::size_t>> Pairs;
		for (std::size_t I = 0; I < Specs.size(); ++I)
		{
			for (std::size_t J = I + 1; J < Specs.size(); ++J)
			{
				Pairs.emplace_back(I, J);
			}
		}
		if (Seed != 0)
		{
			std::mt19937 Rng(Seed * 7919u + 1u);
			std::shuffle(Pairs.begin(), Pairs.end(), Rng);
		}
		for (const auto& [I, J] : Pairs)
		{
			const bool bSwap = Shapes[J].CentreM.Z < Shapes[I].CentreM.Z;
			const std::size_t Lower = bSwap ? J : I;
			const std::size_t Upper = bSwap ? I : J;
			std::optional<InterfaceDesc> Desc = geometry::FaceContact(Ids[Lower], Shapes[Lower], Ids[Upper], Shapes[Upper], ProfileId{});
			if (!Desc)
			{
				continue;
			}
			Desc->Profile = ProfileIds[Pick(Specs[Lower].Kind, Specs[Upper].Kind, std::abs(Desc->NormalAToB.Z) > 0.5)];
			AddInterfaceOk(S.Model, *Desc);
		}
		return S;
	}
};

inline BoxSpec GroundSlab(double LengthM)
{
	return BoxSpec{"ground", "ground", "ground", Vec3{-5.0, -5.0, -1.0}, Vec3{LengthM + 5.0, 5.0, 0.0}, 0.0, true};
}

// Posts 0.2 x 0.2 x 2.0 m at 2.2 m centres, beams 2.199 x 0.2 x 0.3 m on half of each post top; Screws 0 makes every joint Bearing.
inline Scene BuildColonnade(int Bays, int Screws, unsigned Seed = 0)
{
	BoxSceneBuilder B;
	B.Add(GroundSlab(Bays * 2.2));
	for (int K = 0; K <= Bays; ++K)
	{
		B.Add(BoxSpec{"post" + std::to_string(K), "post", "C24 timber post", Vec3{2.2 * K - 0.1, -0.1, 0.0}, Vec3{2.2 * K + 0.1, 0.1, 2.0}, TimberDensity});
	}
	for (int K = 0; K < Bays; ++K)
	{
		B.Add(BoxSpec{"beam" + std::to_string(K), "beam", "C24 timber beam", Vec3{2.2 * K + 0.0005, -0.1, 2.0}, Vec3{2.2 * K + 2.2 - 0.0005, 0.1, 2.3}, TimberDensity});
	}
	const std::size_t Bear = B.Profile(Bearing());
	const std::size_t Screw = Screws > 0 ? B.Profile(Screwed(Screws, 0.0995 * 0.2)) : Bear;
	return B.Build([=](const std::string& Lower, const std::string& Upper, bool) { return Lower == "post" && Upper == "beam" ? Screw : Bear; }, Seed);
}

inline constexpr double PierBeamDepthM = 0.15;

// Two five-course EC6 piers 0.785 m apart under a 1.215 x 0.1025 x 0.15 m beam; the midspan load puts Factor of f_c,90 A on each bearing.
inline Scene BuildTimberOnPiers(double Factor, unsigned Seed = 0)
{
	BoxSceneBuilder B;
	B.Add(GroundSlab(1.0));
	for (int P = 0; P < 2; ++P)
	{
		for (int C = 0; C < 5; ++C)
		{
			B.Add(BoxSpec{"pier" + std::to_string(P) + "_c" + std::to_string(C), "brick", "clay brick, EC6", Vec3{P * 1.0, -BrickYM / 2, C * BrickZM}, Vec3{P * 1.0 + BrickXM, BrickYM / 2, (C + 1) * BrickZM}, BrickDensity});
		}
	}
	const double TopM = 5 * BrickZM + PierBeamDepthM;
	B.Add(BoxSpec{"beam", "beam", "C24 timber beam", Vec3{0.0, -BrickYM / 2, 5 * BrickZM}, Vec3{1.0 + BrickXM, BrickYM / 2, TopM}, TimberDensity});
	const std::size_t Bed = B.Profile(Ec6Bed());
	const std::size_t Bear = B.Profile(Bearing());
	Scene S = B.Build([=](const std::string&, const std::string& Upper, bool) { return Upper == "beam" ? Bear : Bed; }, Seed);
	const double CapacityN = Bearing().CompressiveStrengthPa * BrickXM * BrickYM;
	const double BeamWeightN = TimberDensity * (1.0 + BrickXM) * BrickYM * PierBeamDepthM * GravityMPerS2;
	const double LoadN = 2.0 * (Factor * CapacityN - 0.5 * BeamWeightN);
	S.Loads = {BodyLoad{SceneBody(S, "beam"), Vec3{0.0, 0.0, -LoadN}, Vec3{0.5 * (1.0 + BrickXM), 0.0, TopM}}};
	return S;
}

enum class Counterweight
{
	None,
	Heavy,
	Light
};

// A 1.6 x 0.2 x 0.1 m beam from x -0.4 to 1.2 m on a 0.2 m post at x 0 to 0.2 m; Heavy is a 16 kg cube on the back end, Light 4 kg.
inline Scene BuildCantilever(Counterweight Weight, unsigned Seed = 0)
{
	BoxSceneBuilder B;
	B.Add(GroundSlab(1.0));
	B.Add(BoxSpec{"post", "post", "C24 timber post", Vec3{0.0, -0.1, 0.0}, Vec3{0.2, 0.1, 1.0}, TimberDensity});
	B.Add(BoxSpec{"beam", "beam", "C24 timber beam", Vec3{-0.4, -0.1, 1.0}, Vec3{1.2, 0.1, 1.1}, TimberDensity});
	if (Weight != Counterweight::None)
	{
		B.Add(BoxSpec{"counterweight", "counterweight", "counterweight", Vec3{-0.4, -0.1, 1.1}, Vec3{-0.2, 0.1, 1.3}, Weight == Counterweight::Heavy ? 2000.0 : 500.0});
	}
	const std::size_t Bear = B.Profile(Bearing());
	return B.Build([=](const std::string&, const std::string&, bool) { return Bear; }, Seed);
}

inline std::string LintelBrickName(int Course, double MidBricks)
{
	char Text[32];
	std::snprintf(Text, sizeof Text, "c%dx%.2f", Course, MidBricks);
	return Text;
}

// The 10 x 8 wall brick by brick: an opening [OpenFrom, OpenTo) brick lengths through courses 0 to 4 and a timber lintel in course 5 bearing half a brick each side.
inline Scene BuildLintelWall(bool bBonded, int OpenFrom, int OpenTo)
{
	BoxSceneBuilder B;
	B.Add(GroundSlab(8 * BrickXM));
	const std::string Material = bBonded ? "clay brick, EC6" : "clay brick, dry";
	for (int C = 0; C < 10; ++C)
	{
		std::vector<std::pair<double, double>> Spans;
		if (C % 2 == 0)
		{
			for (int K = 0; K < 8; ++K)
			{
				Spans.emplace_back(K * 1.0, K + 1.0);
			}
		}
		else
		{
			Spans.emplace_back(0.0, 0.5);
			for (int K = 0; K < 7; ++K)
			{
				Spans.emplace_back(K + 0.5, K + 1.5);
			}
			Spans.emplace_back(7.5, 8.0);
		}
		for (const auto& [From, To] : Spans)
		{
			const double Mid = 0.5 * (From + To);
			if ((C < 5 && Mid > OpenFrom + 0.01 && Mid < OpenTo - 0.01) || (C == 5 && Mid > OpenFrom - 1 && Mid < OpenTo + 1))
			{
				continue;
			}
			B.Add(BoxSpec{LintelBrickName(C, Mid), "brick", Material, Vec3{From * BrickXM, -BrickYM / 2, C * BrickZM}, Vec3{To * BrickXM, BrickYM / 2, (C + 1) * BrickZM}, BrickDensity});
		}
		if (C == 5)
		{
			B.Add(BoxSpec{"lintel", "lintel", "C24 timber lintel", Vec3{(OpenFrom - 0.5) * BrickXM, -BrickYM / 2, C * BrickZM}, Vec3{(OpenTo + 0.5) * BrickXM, BrickYM / 2, (C + 1) * BrickZM}, TimberDensity});
		}
	}
	const std::size_t Bed = B.Profile(bBonded ? Ec6Bed() : DryBrick());
	const std::size_t Perpend = B.Profile(bBonded ? Ec6Perpend() : DryBrick());
	const std::size_t Bear = B.Profile(Bearing());
	return B.Build([=](const std::string& Lower, const std::string& Upper, bool bBed) { return Lower == "lintel" || Upper == "lintel" ? Bear : (bBed ? Bed : Perpend); }, 0);
}

// Two dry six-course walls at a right angle, a facade of 6 bricks along x and a return of 4 along +y, with a 10 mm gap or a course-by-course interlock.
inline Scene BuildCorner(bool bInterlock)
{
	BoxSceneBuilder B;
	B.Add(GroundSlab(6 * BrickXM));
	const double EndM = 6 * BrickXM;
	for (int C = 0; C < 6; ++C)
	{
		const double Z0 = C * BrickZM;
		const double Z1 = (C + 1) * BrickZM;
		std::vector<std::pair<double, double>> Facade, Return;
		double ReturnX0 = 0.0;
		if (!bInterlock)
		{
			if (C % 2 == 0)
			{
				for (int K = 0; K < 6; ++K)
				{
					Facade.emplace_back(K * BrickXM, (K + 1) * BrickXM);
				}
			}
			else
			{
				Facade.emplace_back(0.0, BrickXM / 2);
				for (int K = 0; K < 5; ++K)
				{
					Facade.emplace_back(BrickXM / 2 + K * BrickXM, BrickXM / 2 + (K + 1) * BrickXM);
				}
				Facade.emplace_back(EndM - BrickXM / 2, EndM);
			}
			ReturnX0 = EndM + 0.01;
			const double Y0 = -BrickYM / 2;
			if (C % 2 == 0)
			{
				for (int K = 0; K < 4; ++K)
				{
					Return.emplace_back(Y0 + K * BrickXM, Y0 + (K + 1) * BrickXM);
				}
			}
			else
			{
				Return.emplace_back(Y0, Y0 + BrickXM / 2);
				for (int K = 0; K < 3; ++K)
				{
					Return.emplace_back(Y0 + BrickXM / 2 + K * BrickXM, Y0 + BrickXM / 2 + (K + 1) * BrickXM);
				}
				Return.emplace_back(Y0 + 3.5 * BrickXM, Y0 + 4 * BrickXM);
			}
		}
		else
		{
			ReturnX0 = EndM - BrickYM;
			const double YEnd = BrickYM / 2 + 4 * BrickXM;
			if (C % 2 == 0)
			{
				for (int K = 0; K < 6; ++K)
				{
					Facade.emplace_back(K * BrickXM, (K + 1) * BrickXM);
				}
				for (int K = 0; K < 4; ++K)
				{
					Return.emplace_back(BrickYM / 2 + K * BrickXM, BrickYM / 2 + (K + 1) * BrickXM);
				}
			}
			else
			{
				Facade.emplace_back(0.0, BrickXM / 2);
				for (int K = 0; K < 4; ++K)
				{
					Facade.emplace_back(BrickXM / 2 + K * BrickXM, BrickXM / 2 + (K + 1) * BrickXM);
				}
				Facade.emplace_back(4.5 * BrickXM, EndM - BrickYM);
				Return.emplace_back(-BrickYM / 2, -BrickYM / 2 + BrickXM);
				for (int K = 0; K < 3; ++K)
				{
					Return.emplace_back(-BrickYM / 2 + BrickXM + K * BrickXM, -BrickYM / 2 + BrickXM + (K + 1) * BrickXM);
				}
				Return.emplace_back(-BrickYM / 2 + 4 * BrickXM, YEnd);
			}
		}
		for (std::size_t K = 0; K < Facade.size(); ++K)
		{
			B.Add(BoxSpec{"f_c" + std::to_string(C) + "_" + std::to_string(K), "brick", "clay brick, dry, facade", Vec3{Facade[K].first, -BrickYM / 2, Z0}, Vec3{Facade[K].second, BrickYM / 2, Z1}, BrickDensity});
		}
		for (std::size_t K = 0; K < Return.size(); ++K)
		{
			B.Add(BoxSpec{"r_c" + std::to_string(C) + "_" + std::to_string(K), "brick", "clay brick, dry, return", Vec3{ReturnX0, Return[K].first, Z0}, Vec3{ReturnX0 + BrickYM, Return[K].second, Z1}, BrickDensity});
		}
	}
	const std::size_t Dry = B.Profile(DryBrick());
	return B.Build([=](const std::string&, const std::string&, bool) { return Dry; }, 0);
}

struct WallScene
{
	Scene S;
	geometry::Wall Wall;
};

// BuildRunningBondWall with bricks named c<course>b<index>, each recorded at its true length.
inline WallScene BuildWall(bool bBonded, int Courses, int Bricks)
{
	WallScene W;
	geometry::WallSpec Spec;
	Spec.Courses = Courses;
	Spec.BricksPerCourse = Bricks;
	Spec.Bed = AddProfileOk(W.S.Model, bBonded ? Ec6Bed() : DryBrick());
	Spec.Perpend = AddProfileOk(W.S.Model, bBonded ? Ec6Perpend() : DryBrick());
	W.Wall = geometry::BuildRunningBondWall(W.S.Model, Spec);
	CHECK(W.Wall.bComplete);
	Record(W.S, W.Wall.Ground, "ground", "ground", Vec3{5.0, 5.0, 0.5});
	W.S.Boxes.back().CentreM.Z = -0.5;
	const double FullKg = BrickDensity * BrickXM * BrickYM * BrickZM;
	for (std::size_t C = 0; C < W.Wall.Courses.size(); ++C)
	{
		for (std::size_t K = 0; K < W.Wall.Courses[C].size(); ++K)
		{
			const BodyId Id = W.Wall.Courses[C][K];
			Record(W.S, Id, "c" + std::to_string(C) + "b" + std::to_string(K), bBonded ? "clay brick, EC6" : "clay brick, dry",
				Vec3{0.5 * BrickXM * W.S.Model.Body(Id).MassKg / FullKg, BrickYM / 2, BrickZM / 2});
		}
	}
	return W;
}

// Removes every brick of courses [FromCourse, ToCourse) whose centroid lies strictly inside (FromBricks, ToBricks) brick lengths.
inline void RemoveWallBricks(WallScene& W, double FromBricks, double ToBricks, int FromCourse, int ToCourse)
{
	for (int C = FromCourse; C < ToCourse; ++C)
	{
		for (const BodyId Id : W.Wall.Courses[static_cast<std::size_t>(C)])
		{
			const double X = W.S.Model.Body(Id).CentroidM.X / BrickXM;
			if (X > FromBricks && X < ToBricks)
			{
				RemoveBodyOf(W.S, Id);
			}
		}
	}
}

struct ArchScene
{
	Scene S;
	std::vector<BodyId> Voussoirs;
	std::vector<double> JointAnglesDeg;
	double ThicknessM = 0.0;
};

// MakeSemicircularArch with voussoirs named v<index>, each recorded as the largest box centred on its centroid that fits inside its wedge.
inline ArchScene BuildArch(double Eta, double Friction)
{
	SemicircularArch Arch = MakeSemicircularArch(Eta, DryJoint(Friction));
	ArchScene A;
	A.S.Model = std::move(Arch.Model);
	A.Voussoirs = Arch.Voussoirs;
	A.JointAnglesDeg = Arch.JointAnglesDeg;
	A.ThicknessM = Eta * ArchRadiusM;
	for (const BodyId Id : A.S.Model.LiveBodies())
	{
		if (A.S.Model.Body(Id).bGrounded)
		{
			Record(A.S, Id, A.S.Model.Body(Id).CentroidM.X < 0.0 ? "abutment_left" : "abutment_right", "ground", Vec3{0.5, 0.5, 0.5});
		}
	}
	for (std::size_t I = 0; I < A.Voussoirs.size(); ++I)
	{
		const double HalfRad = 0.5 * (A.JointAnglesDeg[I + 1] - A.JointAnglesDeg[I]) * RadPerDegree;
		const double BisectorRad = 0.5 * (A.JointAnglesDeg[I] + A.JointAnglesDeg[I + 1]) * RadPerDegree;
		const Vec3 C = A.S.Model.Body(A.Voussoirs[I]).CentroidM;
		const double CentroidRadiusM = std::sqrt(C.X * C.X + C.Z * C.Z);
		const double RadialHalfM = std::min(CentroidRadiusM - (ArchRadiusM - 0.5 * A.ThicknessM), (ArchRadiusM + 0.5 * A.ThicknessM) * std::cos(HalfRad) - CentroidRadiusM);
		Record(A.S, A.Voussoirs[I], "v" + std::to_string(I), "stone voussoir", Vec3{(CentroidRadiusM - RadialHalfM) * std::tan(HalfRad), 0.5 * ArchDepthM, RadialHalfM});
		A.S.Boxes.back().Rotation = {std::cos(0.5 * BisectorRad), 0.0, std::sin(0.5 * BisectorRad), 0.0};
	}
	return A;
}

struct ShedScene
{
	Scene S;
	geometry::Warehouse Shed;
};

// EC6 walls, Bearing posts and beams, planks screwed with two screws per joint; walls named w<wall>c<course>b<index>, the frame post<i>, beam<i>, plank<i>.
inline ShedScene BuildShed(const geometry::WarehouseSpec& Base)
{
	ShedScene H;
	geometry::WarehouseSpec Spec = Base;
	Spec.Bed = AddProfileOk(H.S.Model, Ec6Bed());
	Spec.Perpend = AddProfileOk(H.S.Model, Ec6Perpend());
	Spec.Bearing = AddProfileOk(H.S.Model, Bearing());
	Spec.Plank = AddProfileOk(H.S.Model, Screwed(2, Spec.PlankWidthM * Spec.BeamWidthM));
	H.Shed = geometry::BuildWarehouse(H.S.Model, Spec);
	CHECK(H.Shed.bComplete);
	Record(H.S, H.Shed.Ground, "ground", "ground", Vec3{5.0, 5.0, 0.5});
	H.S.Boxes.back().CentreM.Z = -0.5;
	const double FullKg = Spec.DensityKgPerM3 * Spec.BrickM.X * Spec.BrickM.Y * Spec.BrickM.Z;
	for (std::size_t Wall = 0; Wall < 4; ++Wall)
	{
		const std::vector<std::vector<BodyId>>& Courses = H.Shed.Walls[Wall].Courses;
		for (std::size_t C = 0; C < Courses.size(); ++C)
		{
			for (std::size_t K = 0; K < Courses[C].size(); ++K)
			{
				const BodyId Id = Courses[C][K];
				const double HalfLengthM = 0.5 * Spec.BrickM.X * H.S.Model.Body(Id).MassKg / FullKg;
				Record(H.S, Id, "w" + std::to_string(Wall) + "c" + std::to_string(C) + "b" + std::to_string(K), "clay brick, EC6",
					Wall % 2 == 0 ? Vec3{HalfLengthM, Spec.BrickM.Y / 2, Spec.BrickM.Z / 2} : Vec3{Spec.BrickM.Y / 2, HalfLengthM, Spec.BrickM.Z / 2});
			}
		}
	}
	const double LengthM = Spec.LengthBricks * Spec.BrickM.X;
	const double WidthM = Spec.WidthBricks * Spec.BrickM.X;
	const double SpanM = (LengthM - Spec.PostSideM) / Spec.Bays;
	const double PlankLengthM = WidthM - 2.0 * (Spec.BrickM.Y + Spec.PostSideM) + Spec.BeamWidthM;
	for (std::size_t I = 0; I < H.Shed.Posts.size(); ++I)
	{
		Record(H.S, H.Shed.Posts[I], "post" + std::to_string(I), "C24 timber post", Vec3{Spec.PostSideM / 2, Spec.PostSideM / 2, 0.5 * Spec.Courses * Spec.BrickM.Z});
	}
	for (std::size_t I = 0; I < H.Shed.Beams.size(); ++I)
	{
		Record(H.S, H.Shed.Beams[I], "beam" + std::to_string(I), "C24 timber beam", Vec3{SpanM / 2, Spec.BeamWidthM / 2, Spec.BeamDepthM / 2});
	}
	for (std::size_t I = 0; I < H.Shed.Planks.size(); ++I)
	{
		Record(H.S, H.Shed.Planks[I], "plank" + std::to_string(I), "C24 timber plank, screwed", Vec3{Spec.PlankWidthM / 2, PlankLengthM / 2, Spec.PlankThicknessM / 2});
	}
	return H;
}

inline AuthorityOptions OptionsFor(const Scene& S)
{
	AuthorityOptions Options;
	Options.Standing.Gravity = S.Gravity;
	Options.Certificate.Gravity = S.Gravity;
	return Options;
}

inline std::unique_ptr<IStructuralAuthority> CombinedAuthority()
{
	return MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate());
}

inline CaseRun Run(Scene& S, std::string_view Slug)
{
	CaseRun R;
	R.Slug = Slug;
	R.Boxes = S.Boxes;
	R.RemovedBeforeRun = S.RemovedBeforeRun;
	R.Before = S.AsBuilt;
	if (!R.Before && VisualsDirFromEnvironment())
	{
		R.Before = JudgeRemainder(S);
	}
	R.Gravity = S.Gravity;
	CascadeOptions Options;
	Options.Authority = OptionsFor(S);
	const auto Start = std::chrono::steady_clock::now();
	R.Verdict = MakeCascade(CombinedAuthority())->Run(S.Model, S.Loads, Options, nullptr);
	R.ElapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
	return R;
}

inline ComponentVerdict JudgeRemainder(const Scene& S) { return CombinedAuthority()->Judge(S.Model, S.Loads, OptionsFor(S), nullptr).Worst; }

inline NameSet Sorted(NameSet Names)
{
	std::sort(Names.begin(), Names.end());
	return Names;
}

inline std::vector<NameSet> SortedClusters(std::vector<NameSet> Clusters)
{
	for (NameSet& C : Clusters)
	{
		std::sort(C.begin(), C.end());
	}
	std::sort(Clusters.begin(), Clusters.end());
	return Clusters;
}

inline std::vector<NameSet> ClusterNames(const Scene& S, const CascadeVerdict& V)
{
	std::vector<NameSet> Out;
	for (const ReleasedCluster& C : V.Clusters)
	{
		NameSet Set;
		for (const BodyId Id : C.Bodies)
		{
			Set.push_back(NameIn(S, Id));
		}
		Out.push_back(std::move(Set));
	}
	return SortedClusters(std::move(Out));
}

inline NameSet ReleasedNames(const Scene& S, const CascadeVerdict& V)
{
	NameSet Out;
	for (const NameSet& C : ClusterNames(S, V))
	{
		Out.insert(Out.end(), C.begin(), C.end());
	}
	return Sorted(std::move(Out));
}

inline NameSet LiveFreeNames(const Scene& S)
{
	NameSet Out;
	for (const SceneBox& B : S.Boxes)
	{
		if (!B.bGrounded && S.Model.IsLive(B.Id))
		{
			Out.push_back(B.Name);
		}
	}
	return Sorted(std::move(Out));
}

inline std::string Names(const NameSet& Set)
{
	std::string Out = "{";
	for (std::size_t I = 0; I < Set.size(); ++I)
	{
		Out += (I > 0 ? ", " : "") + Set[I];
	}
	return Out + "}";
}

inline std::string Names(const std::vector<NameSet>& Clusters)
{
	std::string Out;
	for (const NameSet& C : Clusters)
	{
		Out += Names(C) + " ";
	}
	return Out;
}

inline const char* StateName(CascadeState State)
{
	switch (State)
	{
	case CascadeState::Stands: return "Stands";
	case CascadeState::Overstressed: return "Overstressed";
	case CascadeState::Undecided: return "Undecided";
	case CascadeState::InvalidInput: return "InvalidInput";
	case CascadeState::PassCapReached: return "PassCapReached";
	}
	return "CascadeState(?)";
}

// The outcome by name: state, passes, failure count and each cluster with its pass; body ids and insertion order cannot reach it.
inline std::string Canonical(const Scene& S, const CascadeVerdict& V)
{
	std::vector<std::string> Clusters;
	for (const ReleasedCluster& C : V.Clusters)
	{
		NameSet Set;
		for (const BodyId Id : C.Bodies)
		{
			Set.push_back(NameIn(S, Id));
		}
		Clusters.push_back("p" + std::to_string(C.Pass) + Names(Sorted(std::move(Set))));
	}
	std::sort(Clusters.begin(), Clusters.end());
	std::ostringstream Out;
	Out << StateName(V.State) << ", passes " << V.Passes << ", failures " << V.Failures.size() << ", clusters " << V.Clusters.size() << ":";
	for (const std::string& C : Clusters)
	{
		Out << " " << C;
	}
	return Out.str();
}

struct ExpectedOutcome
{
	std::vector<NameSet> Clusters;
	std::optional<std::uint32_t> ClusterPass;
	std::optional<int> Passes;
	CascadeState State = CascadeState::Stands;
	bool bNoFailures = false;
	NameSet Either;
};

// Clusters by name, survivors (every free body neither removed nor released is live), the final state, and a Stands state re-judging Stands.
inline void CheckOutcome(const Scene& S, const CaseRun& R, const ExpectedOutcome& Want)
{
	INFO("visual " << std::string(R.Slug) << ": " << Canonical(S, R.Verdict));
	const std::vector<NameSet> WantClusters = SortedClusters(Want.Clusters);
	const std::vector<NameSet> GotClusters = ClusterNames(S, R.Verdict);
	INFO("clusters got " << Names(GotClusters) << "want " << Names(WantClusters));
	CHECK(GotClusters == WantClusters);
	if (Want.ClusterPass)
	{
		for (const ReleasedCluster& C : R.Verdict.Clusters)
		{
			CHECK(C.Pass == *Want.ClusterPass);
		}
	}
	if (Want.Passes)
	{
		CHECK(R.Verdict.Passes == *Want.Passes);
	}
	if (Want.bNoFailures)
	{
		CHECK(R.Verdict.Failures.empty());
	}
	NameSet Released;
	for (const NameSet& C : WantClusters)
	{
		Released.insert(Released.end(), C.begin(), C.end());
	}
	NameSet WantLive;
	for (const SceneBox& B : R.Boxes)
	{
		const bool bRemoved = std::find(R.RemovedBeforeRun.begin(), R.RemovedBeforeRun.end(), B.Name) != R.RemovedBeforeRun.end();
		const bool bReleased = std::find(Released.begin(), Released.end(), B.Name) != Released.end();
		if (!B.bGrounded && !bRemoved && !bReleased)
		{
			WantLive.push_back(B.Name);
		}
	}
	CHECK(LiveFreeNames(S) == Sorted(WantLive));
	CHECK(std::string(StateName(R.Verdict.State)) == std::string(StateName(Want.State)));
	if (Want.State == CascadeState::Stands)
	{
		CHECK(JudgeRemainder(S) == ComponentVerdict::Stands);
	}
}

struct VisualCaption
{
	std::string Title, Summary;
};

// Wording from claude_plans/test_plans/2026-10-15-e2e-validation.md section 3; a slug with no caption fails the check.
inline VisualCaption CaptionFor(std::string_view Slug)
{
	static const std::vector<std::pair<std::string_view, VisualCaption>> Captions = {
		{"colonnade-dry-minus-post2", {"3.1 Dry colonnade, middle post removed", "Five C24 posts carry four beams on dry bearings. Losing post 2, the two beams it carried each tip off as their own cluster; nothing else moves."}},
		{"colonnade-dry-minus-post0", {"3.1 Dry colonnade, end post removed", "Five C24 posts carry four beams on dry bearings. Losing the end post, only the end beam tips."}},
		{"arch-keystone-intact", {"3.2 Dry semicircular arch, intact", "A 62-voussoir arch at t/R 0.15, friction 0.7, above its least thickness: it stands."}},
		{"arch-keystone-minus-crown-pair", {"3.2 Arch, two crown voussoirs removed", "Each half-arch becomes a free chain; the 12 voussoirs per side that stand by statics and friction stay, the 36 above fall."}},
		{"arch-keystone-minus-one-crown-voussoir", {"3.2 Arch, one crown voussoir removed", "As the crown pair row: 12 voussoirs stand per side and the 37 above fall."}},
		{"wall-ec6-minus-top-end-brick", {"3.3 EC6 wall, top-course end brick removed", "A 10 x 8 mortared running-bond wall loses a brick that carries nothing: nothing cracks or falls."}},
		{"wall-ec6-minus-mid-brick", {"3.3 EC6 wall, mid-height brick removed", "A 10 x 8 mortared wall loses a middle brick its neighbours bridge by bond: nothing cracks or falls."}},
		{"wall-dry-minus-top-end-brick", {"3.3 Dry wall, top-course end brick removed", "A 10 x 8 dry running-bond wall loses a brick that carries nothing: nothing falls."}},
		{"colonnade-screwed-2-screws-minus-post2", {"3.4 Screwed colonnade, two screws, middle post removed", "Two EC5 screws per beam-on-post joint cannot hold a 1 m cantilever: the joints by the gap crack in tension, then beams 1 and 2 fall."}},
		{"colonnade-screwed-3-screws-minus-post2", {"3.4 Screwed colonnade, three screws, middle post removed", "Three screws per joint hold the cantilever (2.08 needed): the frame bridges the lost post and nothing falls."}},
		{"two-bay-screwed-2-screws-minus-middle-post", {"3.5 Two-bay screwed frame, two screws, middle post removed", "The screwed joints hold, so each end post tips with its beam about its dry base."}},
		{"two-bay-screwed-3-screws-minus-middle-post", {"3.5 Two-bay screwed frame, three screws, middle post removed", "The screwed joints hold, so each end post tips with its beam about its dry base."}},
		{"two-bay-dry-minus-middle-post", {"3.5 Two-bay dry frame, middle post removed", "Contrast row: with dry bearings each beam tips alone and both end posts stand."}},
		{"bonded-wall-push-0-98", {"3.6 EC6 wall pushed at 0.98 of its limit", "A 5 x 4 mortared wall under a lateral push just below the bonded monolith limit: nothing cracks or falls."}},
		{"bonded-wall-push-1-02", {"3.6 EC6 wall pushed at 1.02 of its limit", "Just past the limit the base beds crack in tension, then the wall overturns as one slab."}},
		{"dry-wall-push-0-98", {"3.7 Dry wall pushed at 0.98 t/H", "A 5 x 4 dry wall under a lateral push just below t/H: nothing falls."}},
		{"dry-wall-push-1-05", {"3.7 Dry wall pushed at 1.05 t/H", "Past t/H every brick falls; the cluster partition is not asserted."}},
		{"timber-bearing-load-0-8", {"3.8 Timber beam on brick piers, bearing at 0.8", "A midspan load puts each timber bearing at 0.8 of f_c,90 A: nothing falls."}},
		{"timber-bearing-load-1-2", {"3.8 Timber beam on brick piers, bearing at 1.2", "At 1.2 of f_c,90 A the timber bearings crush: the beam falls and all ten pier bricks stand."}},
		{"shed-minus-side-posts", {"3.9 Shed, every post along one side removed", "Each bay's beams and screwed planks tip off the far posts as one cluster per bay; walls and far posts stand."}},
		{"shed-minus-middle-post", {"3.10 Shed, one middle post removed", "The screwed planks carry the beam ends that lost their post: nothing falls."}},
		{"shed-minus-corner-post", {"3.10 Shed, one corner post removed", "The screwed planks carry the beam ends that lost their post: nothing falls."}},
		{"cantilever-no-counterweight", {"3.11 Cantilever, no counterweight", "A C24 beam overhanging its post by 1.0 m with nothing on its back end tips off."}},
		{"cantilever-16kg-counterweight", {"3.11 Cantilever, 16 kg counterweight", "A 16 kg block keeps the combined centroid over the post: it stands."}},
		{"cantilever-4kg-counterweight", {"3.11 Cantilever, 4 kg counterweight", "A 4 kg block is short of the 5.36 kg needed: beam and block fall together and the post stands."}},
		{"cantilever-16kg-counterweight-removed", {"3.11 Cantilever, 16 kg counterweight removed", "With the counterweight taken off the beam tips."}},
		{"file-colonnade-intact", {"3.12 Colonnade loaded from a file, intact", "data/structures/colonnade.json, the dry colonnade of 3.1: it stands."}},
		{"file-colonnade-minus-post2", {"3.12 Colonnade loaded from a file, middle post removed", "Same answer as the built colonnade: beams 1 and 2 fall, each on its own."}},
		{"dry-lintel-intact", {"3.13 Dry wall with a door opening and timber lintel", "A 10 x 8 dry wall over a four-brick opening spanned by a timber lintel: it stands."}},
		{"dry-lintel-removed-expected", {"3.13 Dry wall, lintel removed", "The masonry over the opening falls; the bricks wholly on the piers stay. Bricks between are not asserted."}},
		{"bonded-lintel-intact", {"3.14 EC6 wall with an opening and lintel", "The mortared wall of 3.13: it stands."}},
		{"bonded-lintel-removed", {"3.14 EC6 wall, lintel removed", "The bond bridges the opening: nothing cracks or falls."}},
		{"dry-wall-minus-two-base-bricks", {"3.15 Dry wall, two base bricks removed", "A 10 x 8 dry wall with two adjacent base bricks knocked out: nothing falls."}},
		{"dry-two-brick-lintel-removed", {"3.15 Dry wall, two-brick opening, lintel removed", "The toothed jambs corbel over a two-brick opening: nothing falls."}},
		{"build-order-shuffled-screwed-colonnade", {"3.16 Shuffled build order", "A fixture rebuilt with its bodies and joints added in a seeded shuffled order gives the same outcome."}},
		{"corner-straight-joint-push", {"3.17 Dry corner with a straight joint, pushed", "A facade with a 10 mm gap at its corner topples under the outward push."}},
		{"corner-interlocked-push", {"3.17 Dry corner, interlocked, pushed", "The facade bonded into its return resists the push that topples the straight-jointed one."}},
	};
	for (const auto& [Key, Caption] : Captions)
	{
		if (Key == Slug)
		{
			return Caption;
		}
	}
	FAIL_CHECK("no caption for visual slug " << std::string(Slug));
	return {std::string(Slug), {}};
}

// Records the run's picture when LOADPATH_VISUALS is set; call it after Run and before the first CHECK.
inline void RecordVisual(const Scene& S, const CaseRun& R, const std::optional<ExpectedOutcome>& Expected = std::nullopt)
{
	if (!VisualsDirFromEnvironment())
	{
		return;
	}
	ViewerScene V;
	V.Slug = std::string(R.Slug);
	const VisualCaption Caption = CaptionFor(R.Slug);
	V.Title = Caption.Title;
	V.Summary = Caption.Summary;
	V.Before = R.Before;
	V.Model = S.Model;
	for (const SceneBox& B : R.Boxes)
	{
		V.Boxes.push_back(ViewerBox{B.Id, B.Name, B.Material, B.CentreM, B.HalfExtentsM, B.Rotation, B.bGrounded, B.MassKg});
	}
	for (const std::string& Name : R.RemovedBeforeRun)
	{
		V.RemovedBeforeRun.push_back(SceneBody(S, Name));
	}
	V.Gravity = R.Gravity;
	V.Loads = S.Loads;
	V.After = {R.Verdict};
	V.ElapsedMs = R.ElapsedMs;
	if (Expected)
	{
		ViewerExpected E;
		E.State = StateName(Expected->State);
		for (const NameSet& Cluster : Expected->Clusters)
		{
			std::vector<BodyId> Ids;
			for (const std::string& Name : Cluster)
			{
				Ids.push_back(SceneBody(S, Name));
			}
			E.Clusters.push_back(std::move(Ids));
		}
		for (const std::string& Name : Expected->Either)
		{
			E.Either.push_back(SceneBody(S, Name));
		}
		V.Expected = std::move(E);
	}
	test::RecordVisual(CurrentTestName(), V);
}
}
