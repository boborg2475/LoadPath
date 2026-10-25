// Writes the E2E plan's before/after scenario files for the viewer, from the fixtures the plan describes.
#include "loadpath/Cascade.h"
#include "loadpath/Catalogue.h"
#include "loadpath/CollapseCertificate.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/Structure.h"
#include "loadpath/Units.h"
#include "loadpath/geometry/BoxContacts.h"
#include "loadpath/geometry/RunningBondWall.h"
#include "loadpath/geometry/Warehouse.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <vector>
using namespace loadpath;
using geometry::Box;

static Catalogue Cat;
static const std::string OutDir = "C:/Users/bobby/AppData/Local/Temp/claude/C--Users-bobby-Documents-Unreal-Projects-DestructionGame/1c20c3b7-b4e9-40fb-a43d-64c43f1388e4/scratchpad/e2e_viz/data/";
static std::vector<std::string> Index;
static double gMu = 0.7;
static std::set<std::string> gFixed;
static const std::string OutPath = "C:/Users/bobby/AppData/Local/Temp/claude/C--Users-bobby-Documents-Unreal-Projects-DestructionGame/1c20c3b7-b4e9-40fb-a43d-64c43f1388e4/scratchpad/stress313/data.json";

const double Lb = 0.215, Tb = 0.1025, Hb = 0.065, Rt = 420.0, G0 = GravityMPerS2;

std::string F(double X)
{
	if (!std::isfinite(X)) return "null";
	if (std::fabs(X) < 1e-12) X = 0;
	char B[32]; std::snprintf(B, sizeof B, "%.6g", X); return B;
}
std::string V3(Vec3 V) { return "[" + F(V.X) + "," + F(V.Y) + "," + F(V.Z) + "]"; }
std::string Esc(const std::string& S)
{
	std::string O;
	for (char C : S) { if (C == '"' || C == '\\') { O += '\\'; O += C; } else if (C == '\n') O += "\\n"; else O += C; }
	return O;
}
std::string Q(const std::string& S) { return "\"" + Esc(S) + "\""; }
Vec3 Sub(Vec3 A, Vec3 B) { return {A.X - B.X, A.Y - B.Y, A.Z - B.Z}; }
Vec3 Add3(Vec3 A, Vec3 B) { return {A.X + B.X, A.Y + B.Y, A.Z + B.Z}; }
Vec3 Cr(Vec3 A, Vec3 B) { return {A.Y * B.Z - A.Z * B.Y, A.Z * B.X - A.X * B.Z, A.X * B.Y - A.Y * B.X}; }
double Len(Vec3 A) { return std::sqrt(A.X * A.X + A.Y * A.Y + A.Z * A.Z); }

const char* CS(CascadeState S) { switch (S) { case CascadeState::Stands: return "Stands"; case CascadeState::Overstressed: return "Overstressed"; case CascadeState::Undecided: return "Undecided"; case CascadeState::InvalidInput: return "InvalidInput"; case CascadeState::PassCapReached: return "PassCapReached"; } return "?"; }
const char* CV(ComponentVerdict V) { switch (V) { case ComponentVerdict::Stands: return "Stands"; case ComponentVerdict::Overstressed: return "Overstressed"; case ComponentVerdict::Collapses: return "Collapses"; case ComponentVerdict::Undecided: return "Undecided"; case ComponentVerdict::InvalidInput: return "InvalidInput"; } return "?"; }
const char* SS(StandingState V) { switch (V) { case StandingState::Stands: return "Stands"; case StandingState::Overstressed: return "Overstressed"; case StandingState::Mechanism: return "Mechanism"; case StandingState::Undecided: return "Undecided"; case StandingState::InvalidInput: return "InvalidInput"; } return "?"; }
const char* KS(CertificateState V) { switch (V) { case CertificateState::Stands: return "Stands"; case CertificateState::Collapses: return "Collapses"; case CertificateState::Undecided: return "Undecided"; case CertificateState::InvalidInput: return "InvalidInput"; } return "?"; }
const char* AX(GoverningAxis A) { switch (A) { case GoverningAxis::None: return "None"; case GoverningAxis::Compression: return "Compression"; case GoverningAxis::Tension: return "Tension"; case GoverningAxis::Shear: return "Shear"; } return "?"; }
const char* IS(InterfaceState S) { switch (S) { case InterfaceState::Intact: return "Intact"; case InterfaceState::Cracked: return "Cracked"; case InterfaceState::Severed: return "Severed"; case InterfaceState::Removed: return "Removed"; } return "?"; }

AuthorityOptions Opts(Vec3 G) { AuthorityOptions O; O.Standing.Gravity = G; O.Certificate.Gravity = G; return O; }
StrengthProfile Row(const char* Name) { auto* R = Cat.FindJoint(Name); if (!R) { std::printf("no row %s\n", Name); return {}; } return R->Profile; }
StrengthProfile Dry(double Mu) { StrengthProfile P = Row("clay_gp_mortar_bed_ec6"); P.ShearCohesionPa = 0; P.TensileStrengthPa = 0; P.FrictionCoefficient = Mu; P.MaxShearStrengthPa = std::numeric_limits<double>::infinity(); return P; }

struct VBody { std::uint32_t Id = 0; Vec3 C, H; double Qt[4] = {1, 0, 0, 0}; bool G = false; double Mass = 0; std::string Mat, Name; bool Slab = false; };

struct Scene
{
	Structure M;
	std::vector<VBody> VB;
	std::map<std::uint32_t, std::string> ProfileNames, Names;
	std::vector<InterfaceId> Ifaces;
	std::vector<BodyLoad> Loads;
	Vec3 Grav{0, 0, -G0};
	bool HasLat = false; double Lat[2] = {0, 0};
	std::vector<std::uint32_t> RemovedBefore;
	struct Spec { std::string Name, Kind, Mat; Vec3 Lo, Hi; double Rho; bool G; };
	std::vector<Spec> Specs; std::vector<BodyId> Ids; std::vector<Box> Boxes;

	ProfileId Prof(const StrengthProfile& P, const std::string& Nm) { auto R = M.AddProfile(P); if (!R.Ok()) std::printf("profile fail %s\n", Nm.c_str()); ProfileNames[R.Value.Index] = Nm; return R.Value; }
	int Add(const std::string& Name, const std::string& Kind, const std::string& Mat, Vec3 Lo, Vec3 Hi, double Rho, bool G = false)
	{
		Specs.push_back(Spec{Name, Kind, Mat, Lo, Hi, Rho, G || gFixed.count(Name) > 0}); return (int)Specs.size() - 1;
	}
	void Commit(const std::function<ProfileId(const std::string&, const std::string&)>& Pick, unsigned Seed = 0)
	{
		std::vector<std::size_t> Order(Specs.size()); std::iota(Order.begin(), Order.end(), 0);
		if (Seed) { std::mt19937 R(Seed); std::shuffle(Order.begin(), Order.end(), R); }
		Ids.assign(Specs.size(), BodyId{}); Boxes.assign(Specs.size(), Box{});
		for (std::size_t I : Order)
		{
			const Spec& S = Specs[I];
			Box B{Vec3{(S.Lo.X + S.Hi.X) / 2, (S.Lo.Y + S.Hi.Y) / 2, (S.Lo.Z + S.Hi.Z) / 2}, Vec3{(S.Hi.X - S.Lo.X) / 2, (S.Hi.Y - S.Lo.Y) / 2, (S.Hi.Z - S.Lo.Z) / 2}};
			double Vol = 8 * B.HalfExtentsM.X * B.HalfExtentsM.Y * B.HalfExtentsM.Z;
			auto R = M.AddBody(BodyDesc{S.G ? 0.0 : S.Rho * Vol, B.CentreM, S.G});
			if (!R.Ok()) std::printf("addbody fail %s\n", S.Name.c_str());
			Ids[I] = R.Value; Boxes[I] = B; Names[R.Value.Index] = S.Name;
			VBody V; V.Id = R.Value.Index; V.C = B.CentreM; V.H = B.HalfExtentsM; V.G = S.G; V.Mass = S.G ? 0.0 : S.Rho * Vol; V.Mat = S.Mat; V.Name = S.Name; V.Slab = S.G;
			VB.push_back(V);
		}
		std::vector<std::pair<std::size_t, std::size_t>> Pairs;
		for (std::size_t I = 0; I < Specs.size(); ++I) for (std::size_t J = I + 1; J < Specs.size(); ++J) Pairs.push_back({I, J});
		if (Seed) { std::mt19937 R(Seed * 7919u + 1u); std::shuffle(Pairs.begin(), Pairs.end(), R); }
		for (auto [I, J] : Pairs)
		{
			std::size_t A = I, B = J;
			if (Boxes[J].CentreM.Z < Boxes[I].CentreM.Z) { A = J; B = I; }
			auto D = geometry::FaceContact(Ids[A], Boxes[A], Ids[B], Boxes[B], ProfileId{});
			if (!D) continue;
			D->Profile = Pick(Specs[A].Kind, Specs[B].Kind);
			auto R = M.AddInterface(*D);
			if (!R.Ok()) std::printf("addinterface fail %s-%s err %d\n", Specs[A].Name.c_str(), Specs[B].Name.c_str(), (int)R.Error);
		}
		Snapshot();
	}
	BodyId Id(int I) const { return Ids[I]; }
	void Snapshot() { Ifaces = M.LiveInterfaces(); }
	void Remove(BodyId B) { if (M.RemoveBody(B)) RemovedBefore.push_back(B.Index); else std::printf("remove failed\n"); }
	std::string N(BodyId B) const { auto It = Names.find(B.Index); return It == Names.end() ? "#" + std::to_string(B.Index) : It->second; }
	std::string N(std::uint32_t I) const { auto It = Names.find(I); return It == Names.end() ? "#" + std::to_string(I) : It->second; }
	const VBody* Find(std::uint32_t I) const { for (auto& V : VB) if (V.Id == I) return &V; return nullptr; }
};

struct Meta { std::string Slug, Title, Test, Summary, Note; };
int Lintel(Scene& S, bool bBonded, int L0, int L1)
{
	S.Add("ground", "ground", "ground", {-5, -5, -1}, {5, 5, 0}, 0, true);
	int Lin = -1;
	const std::string Mat = bBonded ? "clay brick, EC6 mortar" : "clay brick, dry (mu 0.7)";
	for (int C = 0; C < 10; ++C)
	{
		std::vector<std::pair<double, double>> Spans;
		if (C % 2 == 0) for (int K = 0; K < 8; ++K) Spans.push_back({K * 1.0, K + 1.0});
		else { Spans.push_back({0, 0.5}); for (int K = 0; K < 7; ++K) Spans.push_back({K + 0.5, K + 1.5}); Spans.push_back({7.5, 8.0}); }
		for (auto [A, B] : Spans)
		{
			double Mid = 0.5 * (A + B);
			if (C < 5 && Mid > L0 + 0.01 && Mid < L1 - 0.01) continue;
			if (C == 5 && Mid > L0 - 1 && Mid < L1 + 1) continue;
			char Nm[32]; std::snprintf(Nm, sizeof Nm, "c%dx%.2f", C, Mid);
			S.Add(Nm, "brick", Mat, {A * Lb, -Tb / 2, C * Hb}, {B * Lb, Tb / 2, (C + 1) * Hb}, 2000);
		}
		if (C == 5)
		{
			double A = L0 - 0.5, B = L1 + 0.5;
			char Lm[64]; std::snprintf(Lm, sizeof Lm, "C24 timber lintel %.4g x 0.1025 x 0.065 m", (B - A) * Lb);
			Lin = S.Add("lintel", "lintel", Lm, {A * Lb, -Tb / 2, C * Hb}, {B * Lb, Tb / 2, (C + 1) * Hb}, Rt);
		}
	}
	ProfileId Bed = bBonded ? S.Prof(Row("clay_gp_mortar_bed_ec6"), "clay_gp_mortar_bed_ec6 (every brick joint)") : S.Prof(Dry(gMu), "dry brick mu 0.7 (every brick joint)");
	ProfileId Bear = S.Prof(Row("c24_bearing_perp_to_grain"), "c24_bearing_perp_to_grain (lintel joints)");
	S.Commit([&](const std::string& A, const std::string& B) { return (A == "lintel" || B == "lintel") ? Bear : Bed; });
	return Lin;
}

// Two dry walls at a right angle, 6 courses: a facade along x (6 bricks) and a return along +y (4 bricks).
struct Expected { std::vector<std::uint32_t> Released, Unasserted; std::string State = "Stands"; };
Expected LintelExpected(const Scene& S)
{
	Expected E;
	std::vector<std::size_t> Order;
	for (std::size_t I = 0; I < S.Specs.size(); ++I) if (!S.Specs[I].G && S.M.IsLive(S.Ids[I])) Order.push_back(I);
	std::sort(Order.begin(), Order.end(), [&](std::size_t A, std::size_t B) { return S.Specs[A].Lo.Z < S.Specs[B].Lo.Z; });
	auto Below = [&](std::size_t I, std::size_t J) { // J's top touches I's bottom; returns the x overlap length
		const auto &A = S.Specs[I], &B = S.Specs[J];
		if (std::fabs(B.Hi.Z - A.Lo.Z) > 1e-9) return 0.0;
		return std::max(0.0, std::min(A.Hi.X, B.Hi.X) - std::max(A.Lo.X, B.Lo.X));
	};
	std::set<std::size_t> Supported, AssertLive;
	for (std::size_t I : Order)
	{
		const auto& A = S.Specs[I];
		bool bSup = std::fabs(A.Lo.Z) < 1e-9, bAnyBed = bSup;
		double Covered = bSup ? (A.Hi.X - A.Lo.X) : 0.0;
		for (std::size_t J : Order)
		{
			double Ov = Below(I, J); if (Ov <= 1e-9) continue;
			bAnyBed = true;
			if (Supported.count(J)) bSup = true;
			if (AssertLive.count(J)) Covered += Ov;
		}
		int Course = (int)std::lround(A.Lo.Z / Hb);
		if (bSup) Supported.insert(I);
		if (Course <= 5 || Covered >= (A.Hi.X - A.Lo.X) - 1e-9) AssertLive.insert(I);
		bool bAssertReleased = Course == 6 && !bAnyBed;
		if (!bSup) E.Released.push_back(S.Ids[I].Index);
		if (!AssertLive.count(I) && !bAssertReleased) E.Unasserted.push_back(S.Ids[I].Index);
	}
	std::sort(E.Released.begin(), E.Released.end()); std::sort(E.Unasserted.begin(), E.Unasserted.end());
	std::printf("  3.13 expected released:"); for (auto I : E.Released) std::printf(" %s", S.N(I).c_str()); std::printf("\n");
	std::printf("  3.13 asserted live above course 5:"); for (std::size_t I : AssertLive) if (S.Specs[I].Lo.Z > 5.5 * Hb) std::printf(" %s", S.Specs[I].Name.c_str()); std::printf("\n");
	return E;
}

#include "helpers.inc"
#include "main.inc"
