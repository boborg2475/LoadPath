local $/; $_ = <STDIN>;
my $n = 0;
s/#include "loadpath\/StandingAuthority.h"/#include "loadpath\/StandingAuthority.h"\n#include "RbsmDiag.h"/ and $n++;
# globals right after the anonymous namespace opens
s/(namespace loadpath\n\{\nnamespace\n\{\n)/namespace loadpath { std::vector<DiagEvent> gDiag; std::vector<DiagCount> gCounts; std::vector<std::vector<int>> gFinalStatus; std::vector<std::vector<InterfaceId>> gFinalJoints; static std::vector<DiagBlocker> gPending; }\n$1/ and $n++;
# record every closing corner instead of returning at the first
s/(Restoration RestoreInadmissibleReleases\([^\n]*\n\{\n[^\n]*\n\tRestoration Outcome = Restoration::None;\n)/$1\tbool bBlocked = false;\n/ and $n++;
s/\t\t\t\t\treturn Restoration::Undecided;\n/\t\t\t\t\tgPending.push_back(DiagBlocker{J.Id, (int)C, FullRankStates ? (int)(*FullRankStates)[I].Status[C] : -1, GapM}); bBlocked = true; continue;\n/ and $n++;
s/(\t\t\t\tOutcome = Restoration::Restored;\n\t\t\t\}\n\t\t\}\n\t\}\n)\treturn Outcome;\n\}/$1\treturn bBlocked ? Restoration::Undecided : Outcome;\n}/ and $n++;
# a helper after FillMechanism
s/(\nbool IsAllFinite\(const Eigen::MatrixXd& M\))/\nvoid Diag(const char* Why, int It, const Problem& P, const Eigen::VectorXd* Motion, const std::vector<JointState>& States, double PowerW = 0, double DissW = 0)\n{\n\tDiagEvent E; E.Reason = Why; E.Iteration = It; E.Bodies = P.FreeBodies.size(); E.PowerW = PowerW; E.DissipationW = DissW;\n\tif (Motion) { StandingResult R; FillMechanism(P, *Motion, R); E.Motion = R.Mechanism; }\n\tE.Blockers = gPending; gPending.clear(); gDiag.push_back(E);\n\tstd::vector<int> St; std::vector<InterfaceId> Js; for (std::size_t I = 0; I < P.Joints.size(); ++I) { Js.push_back(P.Joints[I].Id); for (std::size_t C = 0; C < CornerCount; ++C) St.push_back((int)States[I].Status[C]); }\n\tgFinalStatus.push_back(St); gFinalJoints.push_back(Js);\n}\n$1/ and $n++;
# per-iteration counts
s/(\t\tResult.ActiveSetIterations = Iteration;\n)/$1\t\t{ DiagCount Cn; Cn.Iteration = Iteration; Cn.Bodies = P.FreeBodies.size(); for (auto& S : States) for (auto St : S.Status) { if (St == CornerStatus::Open) ++Cn.Open; else if (St == CornerStatus::Stuck) ++Cn.Stuck; else ++Cn.Slipping; } gCounts.push_back(Cn); }\n/ and $n++;
# reasons
s/(\t\t\t\tif \(Outcome == Restoration::Undecided\)\n\t\t\t\t\{\n)/$1\t\t\t\t\tDiag("restore", Iteration, P, &Motion, States);\n/ and $n++;
s/(\t\t\t\t\tif \(!\(PowerW > CohesiveDissipationW\(P, States, Motion\)\)\)\n\t\t\t\t\t\{\n)/$1\t\t\t\t\t\tDiag("power", Iteration, P, &Motion, States, PowerW, CohesiveDissipationW(P, States, Motion));\n/ and $n++;
s/(\t\t\t\t\tResult.State = StandingState::Mechanism;\n)/\t\t\t\t\tDiag("mechanism", Iteration, P, &Motion, States, PowerW, CohesiveDissipationW(P, States, Motion));\n$1/ and $n++;
s/(\tResult.Readings = ReadAll\(P, States, X, bOverstressed, Timings\);\n\tResult.State = StandingState::Undecided;\n\treturn Result;\n\})/\tDiag("cap", Options.MaxActiveSetIterations, P, nullptr, States);\n$1/ and $n++;
s/std::unique_ptr<IStandingAuthority> MakeRbsmStandingAuthority\(/std::unique_ptr<IStandingAuthority> MakeRbsmDiag(/ and $n++;
print STDERR "patches applied: $n of 13\n";
print;
