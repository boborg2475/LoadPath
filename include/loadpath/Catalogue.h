#pragma once

#include "loadpath/StrengthProfile.h"
#include "loadpath/StructureFile.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace loadpath
{
enum class StrengthBasis : uint8_t { Characteristic, Mean };

// An area joint: masonry bed or perpend, timber bearing.
struct JointRow
{
	// Kind is "bed", "perpend" or "bearing".
	std::string Name, Kind, Source, Note;
	StrengthBasis Basis = StrengthBasis::Characteristic;
	// Pa and Pa/m, converted by the loader.
	StrengthProfile Profile;
	// The normal stress the friction value was measured at; NaN for a code value.
	double FrictionNormalStressPa = 0;
};

// A discrete fastener: nail, screw, bolt.
struct FastenerRow
{
	std::string Name, Source, Note;
	StrengthBasis Basis = StrengthBasis::Characteristic;
	// F_v,Rk per shear plane.
	double ShearPerPlaneN = 0;
	// F_ax,Rk; 0 when the source gives none (a bolt).
	double WithdrawalN = 0;
	// The timber's f_c,90 the fastener bears on, for the crushing bound.
	double BearingStrengthPa = 0;
	// EN 1995-1-1 Table 7.1 K_ser per shear plane per fastener (slip_modulus_mn_per_m × 1e6).
	double SlipModulusNPerM = 0;
	// The bearing member's normal stiffness (bearing_normal_stiffness_gpa_per_m × 1e9).
	double BearingNormalStiffnessPaPerM = 0;
};

struct MaterialRow
{
	std::string Name, Source, Note;
	double DensityKgPerM3 = 0;
};

struct Catalogue
{
	std::vector<JointRow> Joints;
	std::vector<FastenerRow> Fasteners;
	// Empty when the document has no materials table.
	std::vector<MaterialRow> Materials;
	// nullptr when absent.
	const JointRow* FindJoint(std::string_view Name) const;
	const FastenerRow* FindFastener(std::string_view Name) const;
	const MaterialRow* FindMaterial(std::string_view Name) const;
};

struct CatalogueResult
{
	Catalogue Value;
	FileError Error;

	bool Ok() const { return Error.Message.empty(); }
};

CatalogueResult LoadCatalogue(std::string_view Json);

// mu = 0 (independent axes, DESIGN §4): cohesion = cap = Count · ShearPerPlaneN / AreaM2, tension = Count · WithdrawalN / AreaM2, compression = BearingStrengthPa, kt = Count · SlipModulusNPerM / AreaM2, kn = BearingNormalStiffnessPaPerM.
// Count is the number of fasteners crossing THIS interface, which is one shear plane: a double-shear fastener crossing two interfaces is Count 1 on each, and its withdrawal is then counted on both (an over-count only when a multi-plane fastener has withdrawal).
StrengthProfile FastenerProfile(const FastenerRow& Row, int Count, double AreaM2);
}
