#pragma once

#include "loadpath/Cascade.h"
#include "loadpath/Interface.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"

namespace loadpath::detail
{
inline const char* CascadeStateText(CascadeState State)
{
	switch (State)
	{
	case CascadeState::Stands: return "Stands";
	case CascadeState::Overstressed: return "Overstressed";
	case CascadeState::Undecided: return "Undecided";
	case CascadeState::InvalidInput: return "InvalidInput";
	case CascadeState::PassCapReached: return "PassCapReached";
	}
	return "Unknown";
}

inline const char* VerdictText(ComponentVerdict Verdict)
{
	switch (Verdict)
	{
	case ComponentVerdict::Stands: return "Stands";
	case ComponentVerdict::Overstressed: return "Overstressed";
	case ComponentVerdict::Collapses: return "Collapses";
	case ComponentVerdict::Undecided: return "Undecided";
	case ComponentVerdict::InvalidInput: return "InvalidInput";
	}
	return "Unknown";
}

inline const char* InterfaceStateText(InterfaceState State)
{
	switch (State)
	{
	case InterfaceState::Intact: return "Intact";
	case InterfaceState::Cracked: return "Cracked";
	case InterfaceState::Severed: return "Severed";
	case InterfaceState::Removed: return "Removed";
	}
	return "Unknown";
}

inline const char* AxisText(GoverningAxis Axis)
{
	switch (Axis)
	{
	case GoverningAxis::None: return "None";
	case GoverningAxis::Compression: return "Compression";
	case GoverningAxis::Tension: return "Tension";
	case GoverningAxis::Shear: return "Shear";
	}
	return "Unknown";
}

inline const char* EndText(SolveEnd End)
{
	switch (End)
	{
	case SolveEnd::None: return "None";
	case SolveEnd::Converged: return "Converged";
	case SolveEnd::Mechanism: return "Mechanism";
	case SolveEnd::PowerCheckFailed: return "PowerCheckFailed";
	case SolveEnd::IterationCap: return "IterationCap";
	case SolveEnd::InvalidInput: return "InvalidInput";
	}
	return "Unknown";
}

inline const char* CornerStateText(CornerState State)
{
	switch (State)
	{
	case CornerState::Closed: return "closed";
	case CornerState::Open: return "open";
	case CornerState::Slipping: return "slipping";
	}
	return "unknown";
}
}
