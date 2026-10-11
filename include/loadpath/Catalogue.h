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
};

struct Catalogue
{
	std::vector<JointRow> Joints;
	std::vector<FastenerRow> Fasteners;
	// nullptr when absent.
	const JointRow* FindJoint(std::string_view Name) const;
	const FastenerRow* FindFastener(std::string_view Name) const;
};

struct CatalogueResult
{
	Catalogue Value;
	FileError Error;

	bool Ok() const { return Error.Message.empty(); }
};

CatalogueResult LoadCatalogue(std::string_view Json);

// mu = 0 (independent axes, DESIGN §4): cohesion = cap = Count · ShearPerPlaneN / AreaM2, tension = Count · WithdrawalN / AreaM2, compression = BearingStrengthPa.
StrengthProfile FastenerProfile(const FastenerRow& Row, int Count, double AreaM2);
}
