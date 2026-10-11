# API contract

The public headers under `include/loadpath/`, specified a slice ahead of the code so test-expert
and dev-expert build to one shape. Namespace `loadpath`. SI base units, the unit in every name.
Plain structs, integer handles, `std::span` in, `std::vector` out. No exceptions, no RTTI, no
global state. A decision recorded here is the orchestrator's unless marked owner.

## Slice 1: the model

### Scalars and vectors (`Vec3.h`, `Units.h`)

- `struct Vec3 { double X = 0, Y = 0, Z = 0; }` with `Dot`, `Cross`, `Length`, `IsFinite`.
  Arithmetic operators are added when a test needs them. The engine's own type; Eigen stays
  behind the solver interface.
- `constexpr double GravityMPerS2 = 9.80665;`
- `bool IsFinite(double)` helper; every guard is written `!(x > 0.0)` style so NaN fails closed.

### Handles (`Handles.h`)

Generational handles: `struct BodyId { uint32_t Index; uint32_t Generation; }`, the same for
`InterfaceId` and `ProfileId`. `Generation == 0` is the invalid handle (`BodyId::Invalid()`),
so a value-initialised handle is invalid. Equality operators. Slots are never reused, so a
live slot's generation is always 1 today; the generation field exists so slot reuse can be
introduced later without changing the handle type (see "Structure" below for the ruling).

### Results and errors (`Result.h`)

```cpp
enum class ModelError : uint8_t {
	None, NonFiniteInput, NegativeMass, DegenerateNormal, AxesNotOrthonormal,
	NonPositiveExtent, UnknownBody, UnknownProfile, SameBody, BodyRemoved,
	NegativeStrength, NonPositiveStiffness
};
template <class T> struct Result { T Value{}; ModelError Error = ModelError::None;
	bool Ok() const { return Error == ModelError::None; } };
```

A refused add returns the invalid handle and the reason. There is no partial success: a refused
add changes nothing in the structure.

### Profiles (`StrengthProfile.h`)

```cpp
struct StrengthProfile {
	double CompressiveStrengthPa = 0;
	double ShearCohesionPa = 0;
	double TensileStrengthPa = 0;
	double FrictionCoefficient = 0;
	double MaxShearStrengthPa = std::numeric_limits<double>::infinity();  // unset = plain Mohr-Coulomb
	double NormalStiffnessPaPerM = 0;      // slice 2 uses these; slice 1 only stores and validates
	double TangentialStiffnessPaPerM = 0;
};
```

Validation on `AddProfile`: every field finite (`NonFiniteInput`), except that
`MaxShearStrengthPa` may be `+infinity` (its default); a NaN or `-infinity` cap is
`NonFiniteInput`. Strengths, friction and the cap not negative (`NegativeStrength`);
stiffnesses may be zero in slice 1 (slice 2 decides whether zero is refused). Zero tensile
strength and zero cohesion are the dry-joint profile and are valid. Non-finite is checked
across all fields before negativity is checked on any.

### Bodies (`Body.h`)

```cpp
struct BodyDesc { double MassKg = 0; Vec3 CentroidM; bool bGrounded = false; };
```

`AddBody` refuses a NaN or infinite mass or centroid (`NonFiniteInput`) and a negative mass
(`NegativeMass`). Zero mass is accepted: a massless body is meaningful (TRAPS). A grounded body
is part of the earth: it never moves and absorbs any load.

### Interfaces (`Interface.h`)

```cpp
struct InterfaceDesc {
	BodyId BodyA, BodyB;
	Vec3 NormalAToB;          // the axis of separation, pointing from A into B; normalised on add
	Vec3 CentreM;             // centre of the contact rectangle
	Vec3 AxisU, AxisV;        // in-plane unit axes; U x V must point along the normal
	double HalfExtentUM = 0, HalfExtentVM = 0;
	ProfileId Profile;
};
enum class InterfaceState : uint8_t { Intact, Cracked, Severed, Removed };
```

`AddInterface` validation, in this order, first failure wins: both bodies known (`UnknownBody`,
A then B), then both live (`BodyRemoved`, A then B); distinct (`SameBody`); profile known (`UnknownProfile`); every
scalar finite (`NonFiniteInput`); the normal's length `> 0` after which it is normalised
(`DegenerateNormal`; a NaN length fails this test because `!(len > 0)` is true for NaN); `U` and
`V` unit length within 1e-9, mutually orthogonal and orthogonal to the normal within 1e-9, and
`U x V . N > 0` (`AxesNotOrthonormal`); both half-extents `> 0` and the derived area
`4 * HalfExtentUM * HalfExtentVM` finite and `> 0` (`NonPositiveExtent`; half-extents of 1e-170
underflow the area to 0, which would be the 0/0 fail-open trap in slice 2).

Normalisation is robust to scale (review 2026-10-11): the normal is scaled by its largest
absolute component before its length is taken, so (0, 3e-161, 4e-161) stores exactly
(0, 0.6, 0.8) and (1e155, 0, 0) stores (1, 0, 0); the stored normal's length is within 1e-9 of
one or the add is refused as `DegenerateNormal`. A naive `sqrt(x²+y²+z²)` underflows and
accepted a normal of length 1.006.

Stored per interface, readable through accessors: the two body ids, the unit normal, the centre,
the axes, the half-extents, `AreaM2 = 4 * HalfExtentUM * HalfExtentVM`, the profile id, the
state, and the pass stamps `CrackPass` and `SeverPass` (0 = never; slice 3 writes them). A
removed body's interfaces go to `Removed` with no stamp (removal is the player's move, not a
failure).

### Structure (`Structure.h`)

```cpp
class Structure {
public:
	Result<ProfileId>   AddProfile(const StrengthProfile&);
	Result<BodyId>      AddBody(const BodyDesc&);
	Result<InterfaceId> AddInterface(const InterfaceDesc&);
	bool RemoveBody(BodyId);                 // false for an unknown or already removed id
	bool IsLive(BodyId) const;  bool IsLive(InterfaceId) const;  bool IsValid(ProfileId) const;
	const BodyDesc& Body(BodyId) const;      // a dead or unknown id reads MassKg = NaN (fail closed)
	const InterfaceRecord& Interface(InterfaceId) const;
	const StrengthProfile& Profile(ProfileId) const;
	size_t LiveBodyCount() const;  size_t LiveInterfaceCount() const;
	std::vector<BodyId> LiveBodies() const;  std::vector<InterfaceId> LiveInterfaces() const;  // ascending index order
	std::vector<InterfaceId> InterfacesOf(BodyId) const;  // live only, ascending
};
```

Handles are stable across removal: removing a body tombstones its slot; nothing renumbers and
**slots are never reused**, so the slot's generation stays at the value it was issued with (1)
and a stale handle is told apart by the tombstone, not by a generation bump (review 2026-10-11:
a bump was unobservable and made a never-issued `{i, 2}` read as known). A handle is *known*
when its index is in range and its generation equals the slot's; *live* when known and not
tombstoned. A never-issued generation is `UnknownBody`; a tombstoned slot is `BodyRemoved`. If
slot reuse is ever introduced, the bump returns with it and a test.

Interfaces are never removed from storage: a removed body's interfaces go to state `Removed`
and their handles keep reading the record, so `IsLive(InterfaceId)` means "state is not Removed"
(slice 3 decides whether `Cracked` and `Severed` count as live; the current reading is that they
do).

Dead reads fail closed (review 2026-10-11): `Body()` on an unknown or removed handle returns a
descriptor with `MassKg = NaN` (TRAPS: fail-closed for a mass is NaN, never zero); `Interface()`
on an unknown handle returns a record in state `Removed` with zero area; `Profile()` on an
unknown handle returns a profile whose strengths are all NaN. The sentinels differ on purpose:
a dead interface's zero area is safe because its `Removed` state governs and any demand over
zero area lands on the breaking side of `!(ratio <= 1)`; do not "fix" it to NaN or the body's
NaN to zero. The references the accessors return point into vectors and are invalidated by any
later `Add*`; copy before adding. Iteration order is ascending slot index, which makes it deterministic and independent
of removal history.

### Structure file (`StructureFile.h`), slice 1b

A hand-written JSON reader with no dependency. Format:

```json
{ "profiles":   [ { "name": "mortar", "compressive_pa": 1e7, "cohesion_pa": 9e5, "tensile_pa": 7e5,
                    "friction": 0.75, "max_shear_pa": 2e6, "normal_stiffness_pa_per_m": 0, "tangential_stiffness_pa_per_m": 0 } ],
  "bodies":     [ { "name": "b0", "mass_kg": 2.72, "centroid_m": [0, 0, 0.0325], "grounded": false } ],
  "interfaces": [ { "a": "b0", "b": "b1", "normal": [0, 0, 1], "centre_m": [0, 0, 0.065],
                    "u": [1, 0, 0], "v": [0, 1, 0], "half_u_m": 0.1075, "half_v_m": 0.05125, "profile": "mortar" } ] }
```

```cpp
struct FileError { int Line = 0; std::string Message; };   // Line 0 = no location (document is not an object)
struct LoadedStructure { Structure Model; std::vector<std::string> BodyNames, InterfaceNames, ProfileNames; };
struct LoadResult { LoadedStructure Value; FileError Error; bool Ok() const { return Error.Message.empty(); } };
LoadResult  LoadStructure(std::string_view Json);
std::string WriteStructure(const LoadedStructure&);
int RunCli(std::span<const std::string_view> Args, std::string& Out, std::string& Err);
```

The JSON subset: objects, arrays, numbers (sign, fraction, exponent), strings with the standard
escapes (`\" \\ \/ \b \f \n \r \t \uXXXX` for the Basic Multilingual Plane), `true`, `false`,
`null`, whitespace. Anything else is a syntax error with its 1-based line. Duplicate keys take
the last value. The reader is a small recursive descent over `std::string_view`; depth is capped
at 64 (a deeper document is a syntax error, never a stack overflow).

Semantics: declaration order is free within the file; the loader adds all profiles, then all
bodies, then all interfaces, each kind in file order. A duplicate name within a kind, an unknown name, a
missing required field (`mass_kg`, `a`, `b`, `normal`, `centre_m`, `u`, `v`, `half_u_m`,
`half_v_m`, `profile`), a wrong type, or a 3-vector of the wrong length is an error naming the
element (`Line` is the element's opening line). Missing optional fields take the struct defaults.
Every `Add*` refusal is surfaced as an error naming the element and the `ModelError` by name.
`WriteStructure` emits the same format with every field, bodies in slot order, so
load(write(load(x))) equals load(x) field for field. Removed bodies and their interfaces are
not written.

The CLI binary `loadpath_cli` is a `main` (in `cli/Main.cpp`, its own CMake target) that reads
the file named by its one argument into a string and calls `RunCli`, which does everything else
so tests cover it without a process. The signature above is replaced by:

```cpp
int RunCli(std::span<const std::string_view> Args, std::optional<std::string_view> FileContents,
           std::string& Out, std::string& Err);
```

`Args` excludes the program name; `FileContents` is `std::nullopt` when the file could not be
read. No argument or more than one prints usage to `Err` and returns 2; an unreadable file
prints `cannot read <name>` to `Err` and returns 2; a load error prints `line N: message` (or
just the message when `Line` is 0) to `Err` and returns 1; success prints
`profiles P  bodies B  interfaces I` to `Out` and returns 0. `Err` text ends in a newline, as
`Out` does.

Ratified after the 1b red set (2026-10-11): `BodyNames[Id.Index]` names body `Id`, and the same
for profiles and interfaces (slot-indexed, so a name list is as long as the slot vector).
Bodies and profiles require a `name`; interfaces have none and their errors say `interface`
plus the field. A field error's message contains the field key and the element name; a
duplicate name is reported at the second element's opening line. `Line` is 0 only for an error
with no location: a document that is not an object (empty input, a top-level array) reports
`Line 0`; every syntax, field, name and `Add*` error carries a line. Unknown keys are ignored.
A number that parses to a non-finite value (`1e999`) is a syntax error at its line. All
standard escapes decode, surrogate pairs combine, a lone surrogate is a syntax error.
`centroid_m` is optional (default origin). Nesting: 64 levels counting the root are accepted,
65 refused. The writer prints doubles in shortest round-trip form (`std::to_chars`) and omits
`max_shear_pa` when it is `+infinity`, which the reader's default restores.

Second ratification (2026-10-11, after the 1b green): a number that underflows (`1e-400`) reads
as the nearest representable value (zero or a subnormal), never an error; only overflow to a
non-finite value is the syntax error. Raw control characters (below U+0020) inside a string
are a syntax error at their line. `InterfaceNames` holds one empty string per interface, since
the format has no interface names. When a name vector is shorter than its slot vector, the
writer emits `b<Index>` or `p<Index>` for the missing names. A failed `LoadStructure` returns a
default-constructed `Value` (empty structure, empty names). The round trip compares a
re-normalised normal within 4 ulps per component (re-normalising an already-unit vector is not
bit-exact for arbitrary directions); every other field is compared exactly.

## Slice 2: the standing authority (statics v1)

The first physics. A swappable interface, one implementation: an elastic rigid-body-spring
field (RBSM) on small structures with a dense solve. It answers "does this structure stand
under these loads", gives every interface a utilisation with its governing axis, and when the
structure cannot stand it names the mechanism. Slice 3 adds cracking and the cascade on top;
slice 4 adds the collapse certificate beside it.

### The interface (`StandingAuthority.h`)

```cpp
struct BodyLoad { BodyId Body; Vec3 ForceN; Vec3 PointM; };   // a point force; moment = (Point - Centroid) x Force
struct StandingOptions {
	Vec3 Gravity = {0, 0, -GravityMPerS2};        // tilt tests rotate this; a lateral multiplier adds a horizontal component
	int MaxActiveSetIterations = 50;
	double PivotToleranceRelative = 1e-10;         // a pivot below this times the largest pivot is a mechanism
};
enum class StandingState : uint8_t { Stands, Overstressed, Mechanism, Undecided, InvalidInput };
enum class GoverningAxis : uint8_t { None, Compression, Tension, Shear };
struct CornerReading { Vec3 PointM; double NormalForceN; Vec3 ShearForceN; bool bOpen; bool bSlipping; };
struct InterfaceReading {
	InterfaceId Id; double Utilisation; GoverningAxis Axis;
	double NormalForceN; Vec3 ShearForceN; Vec3 MomentNm;      // resultants about the rectangle centre, force on B
	std::array<CornerReading, 4> Corners;                       // order: (+u,+v), (-u,+v), (-u,-v), (+u,-v)
};
struct MechanismMotion { BodyId Body; Vec3 TranslationM; Vec3 RotationRad; };   // the null vector, largest component scaled to 1
struct StandingResult {
	StandingState State = StandingState::Undecided;
	std::vector<InterfaceReading> Readings;        // every live interface, ascending slot order
	std::vector<MechanismMotion> Mechanism;        // non-empty only when State == Mechanism
	std::vector<BodyId> UnconstrainedBodies;       // bodies the mechanism moves
	int ActiveSetIterations = 0;
	InterfaceId InvalidInterface;                  // set when State == InvalidInput
};
class IStandingAuthority {
public:
	virtual ~IStandingAuthority() = default;
	virtual StandingResult Solve(const Structure&, std::span<const BodyLoad> Loads, const StandingOptions&) const = 0;
};
std::unique_ptr<IStandingAuthority> MakeRbsmStandingAuthority();
```

`Solve` is const, keeps no state between calls, is deterministic for a given structure, and
never mutates the structure. The host may call it on any thread.

### States

- **Stands**: the active set converged, the stiffness matrix is non-singular, and every
  interface reads utilisation ≤ 1. By the lower-bound theorem this is a standing certificate
  for a rigid-block structure whose corner forces are within every bound.
- **Overstressed**: converged and non-singular, but some interface reads > 1 on an axis the
  active set does not release (compression on any joint; tension or cohesive shear on a
  bonded joint). Slice 3's crack round acts on it. In slice 2 the readings are the answer.
- **Mechanism**: the factorisation met a pivot under tolerance, so some set of bodies is free
  to move within the current active set; `Mechanism` holds the null vector. This is the
  slice-2 collapse answer: a block pushed past its toe, a frictionless block on a slope, a
  body resting on a single point. It is fail-closed: a structure that is in fact balanced but
  whose stiffness is singular still reads Mechanism; slice 4's certificate refines it.
- **Undecided**: the active set did not converge within the cap. Fail closed; never Stands.
- **InvalidInput**: a live interface whose profile has a stiffness that is not finite and `> 0`;
  a load on a dead body; a non-finite load or gravity. `InvalidInterface` names the first.

### The model (orchestrator rulings, 2026-10-11)

- **Six degrees of freedom per free body** about its centroid; grounded bodies are fixed and
  their rows are removed. Unknowns are ordered by body slot index, six per body, which fixes
  the assembly order and makes the result bit-reproducible.
- **Four corner springs per interface**, at the rectangle's corners (±HalfExtentU, ±HalfExtentV
  about the centre), each with normal stiffness `kn · A/4` and tangential stiffness `kt · A/4`
  in the plane, where `kn` and `kt` are the profile's `NormalStiffnessPaPerM` and
  `TangentialStiffnessPaPerM`. Corner points, not Gauss points, because that is the
  admissibility the rigid-block limit theorems and every tier-1 benchmark assume: a hinge
  forms at the edge, so Housner's `a/g = b/h`, the incline's `tan θ = w/h` and the arch's
  `t/r = 0.1075` come out exactly. A Gauss-point lumping would open the joint at `e = a/√3`
  instead of `e = a`. The corner's tributary area is `A/4` for stress checks.
- **Relative displacement at a corner** `p`: `δ = (uB + θB × (p − cB)) − (uA + θA × (p − cA))`
  with `u` the translation and `θ` the small rotation of each body. Normal gap `g = δ · n`
  (positive opening, `n` from A to B). Corner normal force on B, compression positive,
  `N = −(kn A/4) g`; shear force on B `S = −(kt A/4) (δ − g n)`.
- **Free releases in the active set** (slice 2): a corner on an interface whose profile has
  `TensileStrengthPa == 0` opens when its trial `N < 0` and then carries nothing; a corner on an
  interface whose profile has `ShearCohesionPa == 0` slips when its trial `|S|` exceeds
  `μ N` (and the cap `MaxShearStrengthPa · A/4`), after which its shear is held at capacity
  opposite the trial slip direction with no stiffness along that direction and **no dilation**
  (non-associative, DESIGN §9 row 8). Transverse to the held direction, in the joint plane, a
  slipping corner carries the tangent of its held friction, `kt · capacity / |trial S|` (never
  above `kt`): the stiffness with which the friction vector rotates to follow the slip
  (dev-expert 2026-10-11, ratified by the orchestrator the same day, DESIGN §10). It adds no force at convergence, since the
  trial slip it was built from lies along the held direction, so the converged readings are
  the model above; it changes the iteration and the null space. Without it the held direction
  lags one pass, and where `capacity / |trial δ|` times the rotational compliance exceeds 1
  the antisymmetric mode grows by that factor every pass with alternating sign (×−5 to ×−12
  measured on a brick pushed into a wall), breaking symmetry out of rounding. A motion
  transverse to a slipping corner's held friction is therefore not free. A corner returns to
  closed when its gap would be `≤ 0` under the field solved without it, and to stick when its
  trial shear falls under capacity by a 1e-9 relative margin (hysteresis so it cannot
  chatter). The loop ends when no corner changes status; it is capped by
  `MaxActiveSetIterations`.
- **Bonded nonlinearities are not released here.** A corner with `TensileStrengthPa > 0`
  stays closed in tension and reports tension utilisation `N⁻ / (ft A/4)`; with cohesion it
  reports shear utilisation `|S| / min(c A/4 + μ N⁺, cap A/4)`. Over 1 is `Overstressed`.
  Slice 3's crack round zeroes the bond (ft and c) of a cracked interface and re-solves, at
  which point these corners enter the free active set above.
- **Utilisation per interface** is the worst corner on the worst axis:
  compression `N⁺ / (fc A/4)`, tension as above (on a dry joint an open corner reads 0), shear
  as above with `N⁺ = max(N, 0)`. A corner with zero force reads 0 on every axis. Non-finite
  anywhere makes the interface read `+inf` on that axis (fail closed) and the state cannot be
  Stands.
- **Solve**: assemble the symmetric stiffness, factor it with a right-looking dense LDLT with
  complete diagonal pivoting on the Schur complement, stopping at the first step whose largest
  remaining diagonal is not above `PivotToleranceRelative` times the largest pivot; the steps
  taken are the rank, the rest span the null space. Back-substitute on the kept block, recover
  corner forces. Eigen (vendored under `third_party/eigen`, `EIGEN_MPL2_ONLY`, single-threaded,
  included as a SYSTEM directory so its warnings do not trip `/WX`) supplies the dense storage
  and triangular solves, not the factorisation (dev-expert 2026-10-11, ratified by the orchestrator the same day, DESIGN §10):
  `Eigen::LDLT` is left-looking and picks each pivot from the diagonal before the Schur update,
  so a zero pivot can precede a non-zero one (measured D = 1.1e8, 1.1e8, 1.1e8, 289414, 0,
  289414) and it divides by any non-zero rounding residue. Dense is the slice-2 choice; slice 6
  swaps the factorisation behind the same interface.
- **Mechanism reporting**: when a pivot fails, the null vector is normalised so its largest
  absolute component is 1 and written per body as `TranslationM`/`RotationRad`;
  `UnconstrainedBodies` lists bodies with any component above 1e-6. Tests assert the motion
  shape: a Housner block reads rotation about the toe edge; a sliding block reads a pure
  translation down-slope. When the null space has more than one direction, the reported motion
  is the projection of the applied load onto the null space; a zero projection is neutral
  equilibrium and reads Stands. A singular active set with a non-zero projection is a Mechanism
  **only if the motion is admissible** (ratified 2026-10-11, driven by the Paterson–Zwick
  diamond): an open corner the motion would close re-closes, and a slipping corner the motion
  would carry along its held friction re-sticks, and the active-set loop continues; a Mechanism
  is returned only when nothing needs restoring. In a neutral solve an open corner whose gap
  moves along the null space is indeterminate and is not re-closed (otherwise a corner at
  exactly zero force cycles open/closed until the cap). **Restored corners re-enter as
  slipping, not stuck, on a joint with `c == 0`** (review ruling 2026-10-11): an open corner the
  motion would close re-closes with its shear held at capacity along the tangential component
  of the projected motion, because re-closing it stuck re-opens it on the next pass and the
  loop cycles (a squat dry block past both its sliding and tipping thresholds read Undecided
  instead of Mechanism). On a cohesive joint it re-closes stuck.
- **No status-only cycle detection** (review ruling 2026-10-11). A repeated status set while
  the held forces are still moving is normal convergence, not a cycle; treating it as one turned
  decided answers into Undecided. The iteration cap is the only early exit, and it reads
  Undecided.
- **Solves are per connected component** (review ruling 2026-10-11). Free bodies linked by
  live interfaces form a component (grounded bodies are sinks, not links). Each component is
  assembled, scaled, factored and judged on its own: its rank tolerance is relative to its own
  largest pivot and its neutral test to its own load norm, so adding bodies with no interface
  path to X never changes X's verdict or readings (a 1 cm cube read Mechanism once a 10 m slab
  stood elsewhere on the ground). Rotational unknowns are scaled by the component's
  characteristic length (the largest corner lever arm `|p − c|` in it) before factoring, so
  every pivot carries N/m and the tolerance compares like with like. The overall `State` is the
  worst over components in the order InvalidInput > Undecided > Mechanism > Overstressed >
  Stands; `Mechanism` and `UnconstrainedBodies` collect every mechanism component.
- **Readings on Mechanism and Undecided** (review ruling 2026-10-11): a standing component's
  readings are its converged readings whatever the overall state. A mechanism component's
  readings are the range-space solution of its last active set, with the released corners
  marked open or slipping; they are in equilibrium with the load's range component and are
  informative, not a certificate. An undecided component's readings are its last pass. Slice 3
  consumes readings only from standing components.
- **Options are validated** (review ruling 2026-10-11): `PivotToleranceRelative` must be finite
  and in `(0, 1)` and `MaxActiveSetIterations` at least 1, else `InvalidInput`. An assembled
  stiffness or load that is not finite is `InvalidInput`, never a release.
- **Stands** means every bonded axis and compression is at or under 1 and every free release is
  consistent; a slipping corner reads shear utilisation 1 by construction and may land a few
  ulp over it, which is not Overstressed.
- **Loads**: gravity `m · g⃗` at every free body's centroid (no moment); each `BodyLoad` adds
  its force and `(PointM − CentroidM) × ForceN` to its body. A tilt test rotates `g⃗`; a
  lateral load factor λ uses `g⃗ = (λ g, 0, −g)`. The test does any bisection; one solve is
  microseconds at this scale.

### Anchors slice 2 must pin

From `BENCHMARKS.md` group 2 (exact anchors, slice 2) and group 3 (friction switches), each with
its ANCHORS.md row: block on an incline tips at `tan θ = w/h` and slides at `tan θ = μ`, the
lower governing, with the mechanism shape asserted; Housner block `a/g = b/h` with rotation
about the toe; free wall `λ = t/H`; 3DEC's four regimes on a slope; the harmonic stack for
n = 2..4 (stands at overhang `(L/2) H_n`, falls one part in 1e6 beyond); the inverted
3-triangle falls and the 4-diamond stands; the 3DEC wedge on two planes at φ = 33.36°; the
Couplet–Heyman semicircular arch with radial joints stands at `t/r = 0.1075` and falls below
(voussoir count from the benchmark report's discretisation guidance; hinge positions asserted
within a half-voussoir of 54.5°); the friction switch on that arch (stands at φ = 22°, slides at
21°). Bracket: the Orduña hollow pile's non-associative load factor inside [0.427, 0.553].
