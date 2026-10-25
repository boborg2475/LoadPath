#pragma once
#include "loadpath/StandingAuthority.h"
#include <memory>
#include <string>
#include <vector>

namespace loadpath
{
struct DiagBlocker { InterfaceId Id; int Corner = 0; int FullRankStatus = -1; double GapM = 0; };
struct DiagCount { int Iteration = 0; int Open = 0, Stuck = 0, Slipping = 0; std::size_t Bodies = 0; };
struct DiagEvent { std::string Reason; int Iteration = 0; std::size_t Bodies = 0; std::vector<DiagBlocker> Blockers; std::vector<MechanismMotion> Motion; double PowerW = 0, DissipationW = 0; };
extern std::vector<DiagEvent> gDiag;
extern std::vector<DiagCount> gCounts;
extern std::vector<std::vector<int>> gFinalStatus; // per component: status per joint corner, joint-major
extern std::vector<std::vector<InterfaceId>> gFinalJoints;
std::unique_ptr<IStandingAuthority> MakeRbsmDiag(const IParallelFor* Workers = nullptr);
}
