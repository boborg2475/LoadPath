std::string ComponentsJson(const std::vector<ComponentReport>& Cs)
{
	std::string O = "[";
	for (std::size_t I = 0; I < Cs.size(); ++I)
	{
		auto& C = Cs[I];
		O += std::string(I ? "," : "") + "{\"bodies\":" + std::to_string(C.Bodies.size()) + ",\"verdict\":" + Q(CV(C.Verdict)) + ",\"elastic\":" + Q(SS(C.Elastic)) + ",\"kinematic\":" + Q(KS(C.Kinematic)) +
			",\"consulted\":" + (C.bConsultedCertificate ? "true" : "false") + ",\"disagreed\":" + (C.bDisagreed ? "true" : "false") + ",\"lambda\":" +
			(std::isfinite(C.LoadFactorLower) ? "[" + F(C.LoadFactorLower) + "," + F(C.LoadFactorUpper) + "]" : std::string("null")) + "}";
	}
	return O + "]";
}

std::string NameList(const Scene& S, const std::vector<BodyId>& B)
{
	std::string O = "{";
	for (std::size_t I = 0; I < B.size(); ++I) O += (I ? ", " : "") + S.N(B[I]);
	return O + "}";
}

// The canonical outcome by name: state, then every cluster as a sorted name list, sorted; insertion order cannot reach it.
std::string Canon(const Scene& S, const CascadeVerdict& V)
{
	std::vector<std::string> Cl;
	for (auto& C : V.Clusters)
	{
		std::vector<std::string> Ns; for (auto B : C.Bodies) Ns.push_back(S.N(B));
		std::sort(Ns.begin(), Ns.end());
		std::string T = "p" + std::to_string(C.Pass) + "{"; for (auto& N : Ns) T += N + ","; Cl.push_back(T + "}");
	}
	std::sort(Cl.begin(), Cl.end());
	std::string O = std::string(CS(V.State)) + " passes " + std::to_string(V.Passes) + " ";
	for (auto& C : Cl) O += C + " ";
	return O;
}

CascadeVerdict RunCascade(Scene& S, double* Ms = nullptr)
{
	CascadeOptions O; O.Authority = Opts(S.Grav);
	auto C = MakeCascade(MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate()));
	auto T0 = std::chrono::steady_clock::now();
	CascadeVerdict V = C->Run(S.M, S.Loads, O, nullptr);
	if (Ms) *Ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - T0).count();
	return V;
}
