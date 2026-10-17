#pragma once

#include "loadpath/Cascade.h"
#include "loadpath/Handles.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/Structure.h"
#include "loadpath/Units.h"
#include "loadpath/Vec3.h"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace loadpath
{
struct ViewerBox
{
	BodyId Id;
	std::string Name, Material;
	Vec3 CentreM, HalfExtentsM;
	// Quaternion (w, x, y, z), the viewer's component order.
	std::array<double, 4> Rotation{1.0, 0.0, 0.0, 0.0};
	bool bGrounded = false;
	double MassKg = 0.0;
};

// States are names ("Stands", "Collapses"); Either bodies are left out when comparing with the engine.
struct ViewerExpected
{
	std::string State = "Stands";
	std::vector<std::vector<BodyId>> Clusters;
	std::vector<BodyId> Either;
};

struct ViewerScene
{
	std::string Slug, Title, Summary, Test;
	Structure Model;
	std::vector<ViewerBox> Boxes;
	std::vector<BodyId> RemovedBeforeRun;
	Vec3 Gravity{0.0, 0.0, -GravityMPerS2};
	std::vector<BodyLoad> Loads;
	std::optional<ComponentVerdict> Before;
	// One verdict per cascade run: passes sum, clusters and failures concatenate, the state is the last one's.
	std::vector<CascadeVerdict> After;
	// Written only when set, so an untimed export is byte-identical run to run.
	std::optional<double> ElapsedMs;
	// Written only when it differs from the engine's after.
	std::optional<ViewerExpected> Expected;
};

// The one writer of viewer/index.html's data format.
std::string WriteViewerJson(const ViewerScene& Scene);
}
