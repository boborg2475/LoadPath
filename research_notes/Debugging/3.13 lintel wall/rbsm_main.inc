
// ---- elastic-only probe: why RBSM reads Mechanism or Undecided on the 3.13 wall ----
#include "RbsmDiag.h"

BodyId ById(const Scene& S, std::uint32_t Index) { for (auto B : S.Ids) if (B.Index == Index) return B; return {}; }
const char* St(int S) { return S == 0 ? "stuck" : S == 1 ? "slipping" : S == 2 ? "open" : "none"; }
std::string Joint(const Scene& S, InterfaceId I) { const auto& R = S.M.Interface(I); return S.N(R.BodyA) + "|" + S.N(R.BodyB); }

StandingResult Elastic(Scene& S, const char* Tag, bool bVerbose = true)
{
	gDiag.clear(); gCounts.clear(); gFinalStatus.clear(); gFinalJoints.clear();
	StandingOptions O; O.Gravity = S.Grav;
	auto A = MakeRbsmDiag();
	StandingResult R = A->Solve(S.M, S.Loads, O, {});
	if (!bVerbose) return R;
	std::printf("\n[%s] elastic %s, %d iterations, %zu components\n", Tag, SS(R.State), R.ActiveSetIterations, R.Components.size());
	for (auto& C : R.Components) std::printf("  component of %zu bodies: %s after %d iterations\n", C.Bodies.size(), SS(C.State), C.ActiveSetIterations);
	// The corner-status trajectory: how many corners were open, stuck, slipping at the start of each iteration.
	std::size_t Last = 0;
	for (std::size_t I = 0; I < gCounts.size(); ++I)
	{
		auto& C = gCounts[I];
		bool bEdge = I + 1 == gCounts.size() || gCounts[I + 1].Iteration == 1;
		if (C.Iteration <= 3 || bEdge) std::printf("   it %2d (%zu bodies): open %3d stuck %3d slipping %3d\n", C.Iteration, C.Bodies, C.Open, C.Stuck, C.Slipping);
		else if (C.Iteration == 4) std::printf("   ...\n");
	}
	for (std::size_t E = 0; E < gDiag.size(); ++E)
	{
		auto& D = gDiag[E];
		std::printf("  outcome (%zu bodies, iteration %d): %s", D.Bodies, D.Iteration, D.Reason.c_str());
		if (D.Reason == "power" || D.Reason == "mechanism") std::printf("  power %.4g W vs cohesive dissipation %.4g W", D.PowerW, D.DissipationW);
		std::printf("\n");
		if (!D.Blockers.empty())
		{
			std::printf("   %zu corners are open but the null motion would push them closed (into each other), and the last full-rank solve had them open too:\n", D.Blockers.size());
			std::map<std::string, int> Per;
			for (auto& B : D.Blockers) Per[Joint(S, B.Id)]++;
			for (auto& [J, N] : Per) std::printf("     %-20s %d corner%s\n", J.c_str(), N, N > 1 ? "s" : "");
		}
		if (!D.Motion.empty())
		{
			std::printf("   null motion moves %zu bodies:", D.Motion.size());
			double Peak = 0; for (auto& M : D.Motion) Peak = std::max(Peak, std::max(Len(M.TranslationM), Len(M.RotationRad)));
			int Shown = 0;
			for (auto& M : D.Motion) { double Sz = std::max(Len(M.TranslationM), Len(M.RotationRad)); if (Sz > 0.05 * Peak) { std::printf(" %s", S.N(M.Body).c_str()); ++Shown; } }
			std::printf("  (%d above 5%% of peak)\n", Shown);
			for (auto& M : D.Motion) { double Sz = std::max(Len(M.TranslationM), Len(M.RotationRad)); if (Sz > 0.05 * Peak && bVerbose) std::printf("     %-9s t (%+.3f %+.3f %+.3f) r (%+.3f %+.3f %+.3f)\n", S.N(M.Body).c_str(), M.TranslationM.X, M.TranslationM.Y, M.TranslationM.Z, M.RotationRad.X, M.RotationRad.Y, M.RotationRad.Z); }
		}
		// Final corner status per joint: list joints with any open or slipping corner.
		if (E < gFinalStatus.size())
		{
			auto& Sts = gFinalStatus[E]; auto& Js = gFinalJoints[E];
			int Open = 0, Slip = 0, Stuck = 0;
			std::string OpenList, SlipList;
			for (std::size_t J = 0; J < Js.size(); ++J)
			{
				int O2 = 0, S2 = 0;
				for (int C = 0; C < 4; ++C) { int V = Sts[J * 4 + C]; if (V == 2) ++O2; else if (V == 1) ++S2; else ++Stuck; }
				Open += O2; Slip += S2;
				if (O2 == 4) OpenList += " " + Joint(S, Js[J]);
				else if (S2 > 0) SlipList += " " + Joint(S, Js[J]) + "(" + std::to_string(S2) + "s" + (O2 ? "," + std::to_string(O2) + "o" : "") + ")";
			}
			std::printf("   corners in the active set: %d open, %d slipping, %d stuck\n", Open, Slip, Stuck);
			std::printf("   fully open joints:%s\n", OpenList.c_str());
			std::printf("   joints with slipping corners:%s\n", SlipList.c_str());
		}
	}
	if (R.State == StandingState::Mechanism)
	{
		std::printf("  UnconstrainedBodies (%zu): ", R.UnconstrainedBodies.size());
		for (auto B : R.UnconstrainedBodies) std::printf(" %s", S.N(B).c_str());
		std::printf("\n");
	}
	return R;
}

int main()
{
	std::ifstream Fc("C:/Users/bobby/Documents/LoadPath/data/catalogue.json", std::ios::binary);
	std::string Jc((std::istreambuf_iterator<char>(Fc)), std::istreambuf_iterator<char>());
	auto Cc = LoadCatalogue(Jc); if (!Cc.Ok()) return 1; Cat = Cc.Value;
	std::vector<std::uint32_t> Tri;
	{
		Scene S; int Lin = Lintel(S, false, 2, 6);
		Elastic(S, "A intact, lintel in place");
		S.Remove(S.Id(Lin));
		Tri = LintelExpected(S).Released;
		Elastic(S, "B lintel removed");
		// Without the certificate: release the elastic UnconstrainedBodies each time it reads Mechanism, until it stops.
		std::printf("\n[elastic-only cascade from B]\n");
		for (int Pass = 1; Pass <= 20; ++Pass)
		{
			StandingResult R = Elastic(S, "", false);
			if (R.State != StandingState::Mechanism) { std::printf("  pass %d: %s, nothing more released; %zu free bodies remain\n", Pass, SS(R.State), S.M.LiveBodyCount() - 1); break; }
			std::printf("  pass %d: Mechanism, release %zu:", Pass, R.UnconstrainedBodies.size());
			for (auto B : R.UnconstrainedBodies) std::printf(" %s", S.N(B).c_str());
			std::printf("\n");
			for (auto B : R.UnconstrainedBodies) S.M.RemoveBody(B);
		}
	}
	{
		Scene S; int Lin = Lintel(S, false, 2, 6);
		S.Remove(S.Id(Lin));
		for (auto K : Tri) S.Remove(ById(S, K));
		Elastic(S, "D lintel and the 10 triangle bricks removed");
	}
	return 0;
}
