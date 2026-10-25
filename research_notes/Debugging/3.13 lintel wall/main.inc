
// ---- 3.13 stress probe ----
#include "RbsmDiag.h"
double Dot3(Vec3 A, Vec3 B) { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }
std::string Num(double X) { return std::isfinite(X) ? F(X) : std::string("null"); }

// The elastic solver alone, through the instrumented copy: its state, why it stopped, its free motion and the corners that blocked it.
std::string ElasticJson(Scene& S, StandingResult* Out = nullptr)
{
	gDiag.clear(); gCounts.clear(); gFinalStatus.clear(); gFinalJoints.clear();
	StandingOptions O; O.Gravity = S.Grav;
	StandingResult R = MakeRbsmDiag()->Solve(S.M, S.Loads, O, {});
	if (Out) *Out = R;
	std::string J = ",\"elastic\":{\"state\":" + Q(SS(R.State)) + ",\"iterations\":" + std::to_string(R.ActiveSetIterations) + ",\"components\":[";
	for (std::size_t I = 0; I < R.Components.size(); ++I)
		J += std::string(I ? "," : "") + "{\"bodies\":" + std::to_string(R.Components[I].Bodies.size()) + ",\"state\":" + Q(SS(R.Components[I].State)) + ",\"iterations\":" + std::to_string(R.Components[I].ActiveSetIterations) + "}";
	J += "],\"events\":[";
	for (std::size_t E = 0; E < gDiag.size(); ++E)
	{
		auto& D = gDiag[E];
		J += std::string(E ? "," : "") + "{\"reason\":" + Q(D.Reason) + ",\"iteration\":" + std::to_string(D.Iteration) + ",\"bodies\":" + std::to_string(D.Bodies) + ",\"power\":" + Num(D.PowerW) + ",\"blockers\":[";
		for (std::size_t K = 0; K < D.Blockers.size(); ++K) J += std::string(K ? "," : "") + "{\"id\":" + std::to_string(D.Blockers[K].Id.Index) + ",\"corner\":" + std::to_string(D.Blockers[K].Corner) + "}";
		J += "],\"motion\":[";
		for (std::size_t K = 0; K < D.Motion.size(); ++K) J += std::string(K ? "," : "") + "{\"body\":" + std::to_string(D.Motion[K].Body.Index) + ",\"t\":" + V3(D.Motion[K].TranslationM) + ",\"r\":" + V3(D.Motion[K].RotationRad) + "}";
		J += "]}";
	}
	J += "],\"released\":[";
	for (std::size_t K = 0; K < R.UnconstrainedBodies.size(); ++K) J += std::string(K ? "," : "") + std::to_string(R.UnconstrainedBodies[K].Index);
	return J + "]}";
}

// Net horizontal force the bodies left of XCut put on the bodies right of it, through the given readings (positive = pushing right).
double ThrustAcross(const Scene& S, const std::vector<InterfaceReading>& Rs, double XCut, double ZMin)
{
	double H = 0;
	for (auto& R : Rs)
	{
		const InterfaceRecord& I = S.M.Interface(R.Id);
		Vec3 CA = S.M.Body(I.BodyA).CentroidM, CB = S.M.Body(I.BodyB).CentroidM;
		if (S.M.Body(I.BodyA).bGrounded || S.M.Body(I.BodyB).bGrounded) continue;
		if (CA.Z < ZMin && CB.Z < ZMin) continue;
		bool ALeft = CA.X < XCut, BLeft = CB.X < XCut;
		if (ALeft == BLeft) continue;
		double FxOnB = R.NormalForceN * I.NormalAToB.X + R.ShearForceN.X;
		H += ALeft ? FxOnB : -FxOnB;
	}
	return H;
}

std::string GeometryJson(const Scene& S)
{
	std::string O = "\"bodies\":[";
	bool First = true;
	for (auto& V : S.VB)
	{
		O += std::string(First ? "" : ",") + "{\"id\":" + std::to_string(V.Id) + ",\"name\":" + Q(V.Name) + ",\"c\":" + V3(V.C) + ",\"h\":" + V3(V.H) + ",\"ground\":" + (V.G ? "true" : "false") + ",\"mass\":" + F(V.Mass) + "}";
		First = false;
	}
	O += "],\"interfaces\":[";
	First = true;
	for (InterfaceId I : S.Ifaces)
	{
		const InterfaceRecord& R = S.M.Interface(I);
		O += std::string(First ? "" : ",") + "{\"id\":" + std::to_string(I.Index) + ",\"a\":" + std::to_string(R.BodyA.Index) + ",\"b\":" + std::to_string(R.BodyB.Index) + ",\"c\":" + V3(R.CentreM) + ",\"n\":" + V3(R.NormalAToB) +
			",\"u\":" + V3(R.AxisU) + ",\"v\":" + V3(R.AxisV) + ",\"hu\":" + F(R.HalfExtentUM) + ",\"hv\":" + F(R.HalfExtentVM) + ",\"area\":" + F(R.AreaM2) + ",\"profile\":" + Q(S.ProfileNames.count(R.Profile.Index) ? S.ProfileNames.at(R.Profile.Index) : "?") + "}";
		First = false;
	}
	return O + "]";
}

std::string ReadingsJson(const std::vector<InterfaceReading>& Rs)
{
	std::string O = "[";
	for (std::size_t I = 0; I < Rs.size(); ++I)
	{
		auto& R = Rs[I];
		O += std::string(I ? "," : "") + "{\"id\":" + std::to_string(R.Id.Index) + ",\"u\":" + Num(R.Utilisation) + ",\"axis\":" + Q(AX(R.Axis)) + ",\"uc\":" + Num(R.CompressionUtilisation) + ",\"ut\":" + Num(R.TensionUtilisation) + ",\"us\":" + Num(R.ShearUtilisation) +
			",\"N\":" + F(R.NormalForceN) + ",\"S\":" + V3(R.ShearForceN) + ",\"M\":" + V3(R.MomentNm) + ",\"corners\":[";
		for (int K = 0; K < 4; ++K)
		{
			auto& Cn = R.Corners[K];
			O += std::string(K ? "," : "") + "{\"p\":" + V3(Cn.PointM) + ",\"N\":" + F(Cn.NormalForceN) + ",\"S\":" + V3(Cn.ShearForceN) + ",\"open\":" + (Cn.bOpen ? "true" : "false") + ",\"slip\":" + (Cn.bSlipping ? "true" : "false") + "}";
		}
		O += "]}";
	}
	return O + "]";
}

// Relative motion of B against A at the interface centre: opening along the normal, and slip.
struct Rel { double Open = 0, Slip = 0; };
Rel RelativeAt(const Scene& S, const InterfaceRecord& R, const std::map<std::uint32_t, MechanismMotion>& Mo)
{
	auto Vel = [&](BodyId B) {
		auto It = Mo.find(B.Index);
		if (It == Mo.end()) return Vec3{0, 0, 0};
		Vec3 Arm = Sub(R.CentreM, S.M.Body(B).CentroidM);
		return Add3(It->second.TranslationM, Cr(It->second.RotationRad, Arm));
	};
	Vec3 D = Sub(Vel(R.BodyB), Vel(R.BodyA));
	Rel Out; Out.Open = Dot3(D, R.NormalAToB);
	Vec3 T = Sub(D, Vec3{R.NormalAToB.X * Out.Open, R.NormalAToB.Y * Out.Open, R.NormalAToB.Z * Out.Open});
	Out.Slip = Len(T);
	return Out;
}

std::map<std::uint32_t, MechanismMotion> MotionOf(const std::vector<ComponentReport>& Cs)
{
	std::map<std::uint32_t, MechanismMotion> Mo;
	for (auto& C : Cs) for (auto& M : C.Mechanism) Mo[M.Body.Index] = M;
	return Mo;
}

std::string MotionJson(const std::vector<ComponentReport>& Cs)
{
	std::string O = "[";
	bool First = true;
	for (auto& C : Cs) for (auto& M : C.Mechanism)
	{
		bool bRel = std::find(C.Release.begin(), C.Release.end(), M.Body) != C.Release.end();
		O += std::string(First ? "" : ",") + "{\"body\":" + std::to_string(M.Body.Index) + ",\"t\":" + V3(M.TranslationM) + ",\"r\":" + V3(M.RotationRad) + ",\"released\":" + (bRel ? "true" : "false") + "}";
		First = false;
	}
	return O + "]";
}

std::string JointMotionJson(const Scene& S, const std::map<std::uint32_t, MechanismMotion>& Mo)
{
	std::string O = "[";
	bool First = true;
	for (InterfaceId I : S.M.LiveInterfaces())
	{
		Rel X = RelativeAt(S, S.M.Interface(I), Mo);
		if (std::fabs(X.Open) < 1e-9 && X.Slip < 1e-9) continue;
		O += std::string(First ? "" : ",") + "{\"id\":" + std::to_string(I.Index) + ",\"open\":" + F(X.Open) + ",\"slip\":" + F(X.Slip) + "}";
		First = false;
	}
	return O + "]";
}

std::string IdList(const std::vector<std::uint32_t>& V)
{
	std::string O = "[";
	for (std::size_t I = 0; I < V.size(); ++I) O += (I ? "," : "") + std::to_string(V[I]);
	return O + "]";
}

std::string StateJson(Scene& S, const std::string& Key, const std::string& Label, ComponentVerdict Worst, const std::vector<InterfaceReading>& Readings, const std::vector<ComponentReport>& Cs, const std::vector<std::uint32_t>& Gone, const std::string& Extra)
{
	return "{\"key\":" + Q(Key) + ",\"label\":" + Q(Label) + ",\"worst\":" + Q(CV(Worst)) + ",\"components\":" + ComponentsJson(Cs) + ",\"gone\":" + IdList(Gone) +
		",\"readings\":" + ReadingsJson(Readings) + ",\"mechanism\":" + MotionJson(Cs) + ",\"jointMotion\":" + JointMotionJson(S, MotionOf(Cs)) + Extra + "}";
}

AuthorityResult Judge(Scene& S)
{
	auto A = MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate());
	return A->Judge(S.M, S.Loads, Opts(S.Grav), nullptr);
}

BodyId ById(const Scene& S, std::uint32_t Index) { for (auto B : S.Ids) if (B.Index == Index) return B; return {}; }

void Summarise(const char* Tag, Scene& S, const AuthorityResult& R)
{
	std::printf("[%s] worst %s, %zu readings\n", Tag, CV(R.Worst), R.Readings.size());
	for (auto& C : R.Components)
	{
		std::printf("  component %zu bodies: %s elastic %s kinematic %s lambda [%.3g, %.3g] mechanism %zu release %zu\n", C.Bodies.size(), CV(C.Verdict), SS(C.Elastic), KS(C.Kinematic), C.LoadFactorLower, C.LoadFactorUpper, C.Mechanism.size(), C.Release.size());
		if (!C.Release.empty()) { std::printf("   released:"); for (auto B : C.Release) std::printf(" %s", S.N(B).c_str()); std::printf("\n"); }
	}
}

void Diagnose(Scene& S, const AuthorityResult& B, const std::vector<std::uint32_t>& Expected)
{
	auto Mo = MotionOf(B.Components);
	std::set<std::uint32_t> Tri(Expected.begin(), Expected.end());
	double Work = 0, L1 = 0;
	for (auto& [K, M] : Mo)
	{
		const BodyDesc& D = S.M.Body(ById(S, K));
		Work += D.MassKg * G0 * -M.TranslationM.Z;
		L1 += std::fabs(M.TranslationM.X) + std::fabs(M.TranslationM.Y) + std::fabs(M.TranslationM.Z) + std::fabs(M.RotationRad.X) + std::fabs(M.RotationRad.Y) + std::fabs(M.RotationRad.Z);
		if (!Tri.count(K) && Len(M.TranslationM) + Len(M.RotationRad) > 1e-3)
			std::printf("   non-triangle mover %-10s t (%+.3f %+.3f %+.3f) r (%+.3f %+.3f %+.3f)\n", S.N(K).c_str(), M.TranslationM.X, M.TranslationM.Y, M.TranslationM.Z, M.RotationRad.X, M.RotationRad.Y, M.RotationRad.Z);
	}
	std::printf("  engine mechanism: %zu bodies listed, gravity work %.3f, L1 motion %.3f, work per L1 %.3f\n", Mo.size(), Work, L1, Work / L1);
	int Sliding = 0, Dilating = 0;
	for (InterfaceId I : S.M.LiveInterfaces())
	{
		Rel X = RelativeAt(S, S.M.Interface(I), Mo);
		if (X.Slip > 1e-3) { ++Sliding; if (X.Open >= gMu * X.Slip - 1e-3) ++Dilating; }
	}
	std::printf("  engine mechanism: %d joints slide, %d of them open by at least mu x slip (associative dilatancy)\n", Sliding, Dilating);
	// The expected outcome as a motion: the triangle drops straight down, everything else still.
	std::map<std::uint32_t, MechanismMotion> TriFall;
	double TWork = 0, TL1 = 0;
	for (auto K : Expected) { MechanismMotion M; M.Body = ById(S, K); M.TranslationM = {0, 0, -1}; TriFall[K] = M; TWork += S.M.Body(M.Body).MassKg * G0; TL1 += 1; }
	int Violate = 0;
	for (InterfaceId I : S.M.LiveInterfaces())
	{
		const InterfaceRecord& R = S.M.Interface(I);
		if (Tri.count(R.BodyA.Index) == Tri.count(R.BodyB.Index)) continue;
		Rel X = RelativeAt(S, R, TriFall);
		bool Bad = X.Open < -1e-9 || X.Open < gMu * X.Slip - 1e-9;
		if (Bad) { ++Violate; std::printf("   triangle-fall joint %-9s|%-9s open %+.3f slip %.3f, associative flow needs open >= %.3f\n", S.N(R.BodyA).c_str(), S.N(R.BodyB).c_str(), X.Open, X.Slip, gMu * X.Slip); }
	}
	std::printf("  triangle-only fall: work per L1 %.3f; %d boundary joints violate the associative flow rule\n", TWork / TL1, Violate);
}

static std::vector<std::string> TriNames;

// Per-brick vertical balance from readings: what holds the brick up, split by joint type.
void HoldUp(const Scene& S, const std::vector<InterfaceReading>& Rs, const std::string& Name)
{
	std::uint32_t Id = 0; for (auto& V : S.VB) if (V.Name == Name) Id = V.Id;
	double Below = 0, Above = 0, Side = 0, SideN = 0;
	for (auto& R : Rs)
	{
		const InterfaceRecord& I = S.M.Interface(R.Id);
		if (I.BodyA.Index != Id && I.BodyB.Index != Id) continue;
		Vec3 F{R.NormalForceN * I.NormalAToB.X + R.ShearForceN.X, R.NormalForceN * I.NormalAToB.Y + R.ShearForceN.Y, R.NormalForceN * I.NormalAToB.Z + R.ShearForceN.Z};
		if (I.BodyA.Index == Id) F = Vec3{-F.X, -F.Y, -F.Z};
		bool Bed = std::fabs(I.NormalAToB.Z) > 0.5;
		bool IsBelow = Bed && I.BodyB.Index == Id;
		if (!Bed) { Side += F.Z; SideN += std::fabs(R.NormalForceN); }
		else if (IsBelow) Below += F.Z; else Above += F.Z;
	}
	double W = 0; for (auto& V : S.VB) if (V.Id == Id) W = V.Mass * G0;
	std::printf("     %-9s weight %5.1f N | up from beds below %+7.1f, from side joints %+7.1f (side compression %6.1f N, friction limit %5.1f), from beds above %+7.1f\n", Name.c_str(), W, Below, Side, SideN, gMu * SideN, Above);
}

// The state the elastic-only cascade reaches: its pass 1 released c6x3.50 and c6x4.50, and pass 2 reads Undecided.
void SolverState(std::string& Out, const std::vector<std::uint32_t>& Expected)
{
	Scene S; int Lin = Lintel(S, false, 2, 6);
	S.Remove(S.Id(Lin));
	std::vector<std::uint32_t> Gone{S.Id(Lin).Index};
	for (auto& V : S.VB) if (V.Name == "c6x3.50" || V.Name == "c6x4.50") { S.Remove(ById(S, V.Id)); Gone.push_back(V.Id); }
	AuthorityResult J = Judge(S); Summarise("S solver-only cascade, pass 2", S, J);
	StandingResult R; std::string EJ = ElasticJson(S, &R);
	std::printf("  elastic %s; its last full-rank readings: thrust across the midline above course 5 = %.1f N\n", SS(R.State), ThrustAcross(S, R.Readings, 4 * Lb, 5.5 * Hb));
	std::printf("  what holds each remaining triangle brick in those readings:\n");
	for (auto& N : TriNames) if (N != "c6x3.50" && N != "c6x4.50") HoldUp(S, R.Readings, N);
	Out += "," + StateJson(S, "solver", "Solver-only cascade: pass 1 dropped c6x3.50 and c6x4.50, pass 2 is Undecided", J.Worst, R.Readings, J.Components, Gone, ",\"expectedReleased\":" + IdList(Expected) + EJ);
}

// Does the triangle stand if everything around it is fixed (ideal abutments), or if only the piers are fixed?
void FixedAbutments()
{
	for (double Mu : {0.7, 1.0})
	for (int Mode = 0; Mode < 2; ++Mode)
	{
		gMu = Mu; gFixed.clear();
		Scene Probe; int L0 = Lintel(Probe, false, 2, 6); (void)L0;
		for (auto& Sp : Probe.Specs)
		{
			if (Sp.G || Sp.Name == "lintel") continue;
			bool Tri = std::find(TriNames.begin(), TriNames.end(), Sp.Name) != TriNames.end();
			int Course = (int)std::lround(Sp.Lo.Z / Hb);
			if (Mode == 0 ? !Tri : Course <= 5) gFixed.insert(Sp.Name);
		}
		Scene S; int Lin = Lintel(S, false, 2, 6);
		S.Remove(S.Id(Lin));
		AuthorityResult R = Judge(S);
		std::size_t Rel = 0; for (auto& C : R.Components) Rel += C.Release.size();
		std::printf("[fixed %s, mu %.1f] worst %s", Mode == 0 ? "everything but the triangle" : "piers (courses 0-5)", Mu, CV(R.Worst));
		for (auto& C : R.Components) std::printf(" | %zu bodies: elastic %s, certificate %s lambda [%.3g, %.3g], release %zu", C.Bodies.size(), SS(C.Elastic), KS(C.Kinematic), C.LoadFactorLower, C.LoadFactorUpper, C.Release.size());
		std::printf("\n");
		if (Mode == 1 && Mu == 0.7) std::printf("   thrust across the midline above course 5 (elastic) = %.1f N\n", ThrustAcross(S, R.Readings, 4 * Lb, 5.5 * Hb));
	}
	gFixed.clear(); gMu = 0.7;
}int main()
{
	std::ifstream Fc("C:/Users/bobby/Documents/LoadPath/data/catalogue.json", std::ios::binary);
	std::string Jc((std::istreambuf_iterator<char>(Fc)), std::istreambuf_iterator<char>());
	auto Cc = LoadCatalogue(Jc); if (!Cc.Ok()) return 1; Cat = Cc.Value;
	std::string Out = "{";
	std::vector<std::uint32_t> ExpectedReleased;
	{
		Scene S; int Lin = Lintel(S, false, 2, 6);
		Out += GeometryJson(S) + ",\"lintel\":" + std::to_string(S.Id(Lin).Index) + ",\"states\":[";
		AuthorityResult A = Judge(S); Summarise("A intact", S, A);
		Out += StateJson(S, "intact", "Lintel in place", A.Worst, A.Readings, A.Components, {}, ElasticJson(S));
		S.Remove(S.Id(Lin));
		Expected E = LintelExpected(S); ExpectedReleased = E.Released;
		AuthorityResult B = Judge(S); Summarise("B lintel removed", S, B);
		Diagnose(S, B, E.Released);
		Out += "," + StateJson(S, "removed", "Lintel removed, before the cascade", B.Worst, B.Readings, B.Components, {S.Id(Lin).Index}, ",\"expectedReleased\":" + IdList(E.Released) + ElasticJson(S));
		TriNames.clear(); for (auto K : E.Released) TriNames.push_back(S.N(K));
		std::printf("  elastic readings, lintel removed: thrust across the midline above course 5 = %.1f N\n", ThrustAcross(S, B.Readings, 4 * Lb, 5.5 * Hb));
		SolverState(Out, E.Released);
		CascadeVerdict V = RunCascade(S);
		std::printf("[C cascade] %s passes %d clusters %zu failures %zu\n", CS(V.State), V.Passes, V.Clusters.size(), V.Failures.size());
		std::vector<std::uint32_t> Gone{S.Id(Lin).Index};
		std::string Cl = ",\"clusters\":[";
		for (std::size_t I = 0; I < V.Clusters.size(); ++I)
		{
			std::vector<std::uint32_t> Bs;
			for (auto B2 : V.Clusters[I].Bodies) { Bs.push_back(B2.Index); Gone.push_back(B2.Index); }
			Cl += std::string(I ? "," : "") + "{\"pass\":" + std::to_string(V.Clusters[I].Pass) + ",\"bodies\":" + IdList(Bs) + "}";
		}
		Cl += "],\"failures\":[";
		for (std::size_t I = 0; I < V.Failures.size(); ++I)
		{
			auto& Fl = V.Failures[I];
			Cl += std::string(I ? "," : "") + "{\"id\":" + std::to_string(Fl.Id.Index) + ",\"pass\":" + std::to_string(Fl.Pass) + ",\"axis\":" + Q(AX(Fl.Axis)) + ",\"ratio\":" + Num(Fl.Ratio) + ",\"severed\":" + (Fl.bSevered ? "true" : "false") + "}";
		}
		Cl += "],\"cascadeState\":" + Q(CS(V.State)) + ",\"passes\":" + std::to_string(V.Passes);
		ComponentVerdict W = ComponentVerdict::Stands;
		for (auto& C : V.Components) if ((int)C.Verdict > (int)W) W = C.Verdict;
		Out += "," + StateJson(S, "cascade", "After the engine's cascade", W, V.Readings, V.Components, Gone, Cl + ElasticJson(S));
	}
	{
		Scene S; int Lin = Lintel(S, false, 2, 6);
		S.Remove(S.Id(Lin));
		std::vector<std::uint32_t> Gone{S.Id(Lin).Index};
		for (auto K : ExpectedReleased) { S.Remove(ById(S, K)); Gone.push_back(K); }
		AuthorityResult D = Judge(S); Summarise("D lintel + 10 expected removed", S, D);
		Diagnose(S, D, {});
		Out += "," + StateJson(S, "expected", "Lintel and the 10 expected bricks removed", D.Worst, D.Readings, D.Components, Gone, ElasticJson(S));
		CascadeVerdict V = RunCascade(S);
		std::printf("[D cascade] %s passes %d clusters %zu\n", CS(V.State), V.Passes, V.Clusters.size());
	}
	Out += "]}";
	std::ofstream(OutPath) << Out;
	// Friction sweep: the dilatancy angle is atan(mu), so if dilatancy drives the over-release the count moves with mu.
	for (double Mu : {0.05, 0.2, 0.4, 0.7, 1.0, 1.5})
	{
		gMu = Mu;
		Scene S; int Lin = Lintel(S, false, 2, 6);
		S.Remove(S.Id(Lin));
		AuthorityResult B = Judge(S);
		std::size_t Rel = 0; for (auto& C : B.Components) Rel += C.Release.size();
		for (auto& C : B.Components) std::printf("  [mu %.2f] component %zu: elastic %s kinematic %s lambda [%.3g, %.3g]\n", Mu, C.Bodies.size(), SS(C.Elastic), KS(C.Kinematic), C.LoadFactorLower, C.LoadFactorUpper);
		CascadeVerdict V = RunCascade(S);
		std::size_t Fell = 0; for (auto& C : V.Clusters) Fell += C.Bodies.size();
		std::printf("[mu %.2f] pass-1 release %zu bodies; cascade %s, %d passes, %zu bricks fall in %zu clusters\n", Mu, Rel, CS(V.State), V.Passes, Fell, V.Clusters.size());
	}
	FixedAbutments();
	return 0;
}
