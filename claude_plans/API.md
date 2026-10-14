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
	// Stage 2: a revision per successful mutation, the revision at which a body was last touched (its own AddBody, an interface added/cracked/severed on it, a live neighbour removed; an unknown id reads the largest value), and a lineage token minted at construction and by every copy (a move keeps it in the destination and mints a fresh one for the source).
	std::uint64_t Revision() const;
	std::uint64_t LastTouched(BodyId Id) const;
	std::uint64_t Lineage() const;
	const ComponentPartition& Components() const;   // cached partition, reset by every successful mutation (stage 1b-ii)
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
and their handles keep reading the record, so `IsLive(InterfaceId)` means "state is `Intact` or `Cracked`"
(slice 3: a severed joint carries nothing and is not live; its record and stamps stay readable).

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
load(write(load(x))) equals load(x) field for field. Removed bodies are not written; an interface is written when both its bodies are live, in
any state but `Removed` (slice 3), with a top-level `highest_pass` when it is above 0.

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
are a syntax error at their line. `InterfaceNames` holds one string per interface, empty when
the file gave none (an optional `name` per interface since slice 3; duplicates are not checked). When a name vector is shorter than its slot vector, the
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
	std::vector<MechanismMotion> Mechanism;        // every mechanism component, whatever State
	std::vector<BodyId> UnconstrainedBodies;       // bodies every mechanism component moves
	std::vector<StandingComponent> Components;     // per component: bodies, State, ActiveSetIterations, ReadingSlots into Readings; empty on InvalidInput (stage 1b-ii)
	int ActiveSetIterations = 0;
	InterfaceId InvalidInterface;                  // set when State == InvalidInput
};
class IStandingAuthority {
public:
	virtual ~IStandingAuthority() = default;
	// Only: empty solves every component; otherwise the components containing a listed body, Readings and Components covering those only (plus interfaces joining two grounded bodies); an entry that is not a live free body is InvalidInput (stage 2).
	virtual StandingResult Solve(const Structure&, std::span<const BodyLoad> Loads, const StandingOptions&, std::span<const BodyId> Only) const = 0;
};
std::unique_ptr<IStandingAuthority> MakeRbsmStandingAuthority();
```

`Solve` is const, keeps no state between calls, is deterministic for a given structure, and
never mutates the structure. The host may call it on any thread.

### States

- **Stands**: the active set converged (full rank, or singular with a zero load projection,
  which is neutral equilibrium) and every interface reads utilisation ≤ 1 on every axis the
  active set does not release. By the lower-bound theorem this is a standing certificate for a
  rigid-block structure whose corner forces are within every bound, with the caveat that under
  non-associative friction the theorem bounds, not decides (see Mechanism below).
- **Overstressed**: converged and non-singular, but some interface reads > 1 on an axis the
  active set does not release (compression on any joint; tension or cohesive shear on a
  bonded joint). Slice 3's crack round acts on it. In slice 2 the readings are the answer.
- **Mechanism**: the factorisation met a pivot under tolerance and the null-space motion the
  load drives is admissible against the open and slipping corners as they stand, so some set
  of bodies is free to move; `Mechanism` holds the null vector. The motion must also be paid
  for: its load power (held friction on slipping corners included) must strictly exceed the
  cohesion it breaks, `min(c, shear cap) · A/4` times the slide of every open corner of a joint
  that holds slip (`c > 0`, uncracked); otherwise the component reads Undecided (DESIGN §10,
  the cohesive-wedge Mechanism), since an open cohesive corner keeps its cohesion intercept
  until a crack removes it. This is the slice-2 collapse
  answer: a block pushed past its toe, a frictionless block on a slope, a body resting on a
  single point. **It is a verdict, not a certificate** (third review, 2026-10-11): the elastic
  path can reach a singular admissible set on a structure for which a statically admissible
  field exists (the three-brick bridge at μ 5, λ = (2.5, 0.875), reads Mechanism after 5
  passes while an NNLS search finds an admissible field at 0.7 μ and 1.02 × the load). Under
  non-associative friction the lower-bound theorem is not sufficient; a probe DEM (2026-10-13)
  showed this bridge falling, so the elastic Mechanism was the physical answer there, and the
  combined authority now reports Undecided rather than the certificate's Stands in that case.
  The slice-4 kinematic certificate referees Mechanism verdicts as well as Undecided ones, and a
  host treats a combined Collapses as a release. A neutral singular set (zero load
  projection) reads Stands, so Stands does not require a non-singular stiffness.
- **Undecided**: the active set did not converge within the cap, a restoration needed a
  value the last full-rank solve does not supply (the fail-closed clause below), or a
  null-space motion's load power did not exceed the cohesion it would break (the readings are
  then the last full-rank solve's, and an interface's utilisation can read +inf there where the
  Mechanism path read a finite value). Never Stands, never a release.
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
  `N = −(kn A/4) g`; shear force on B `S = −(kt A/4) (δ − g n − Ref)`, where `Ref` is the
  corner's tangential reference: zero as built and for every corner that has not re-closed
  twice, so on a dry or bonded joint the shear is `kt A/4` times the whole tangential offset.
  A cohesive tensionless corner (`c > 0`, `f_t = 0`) that re-closes for the second or later time
  in one solve takes the tangential offset at that re-closure as `Ref` (slice 2e, DESIGN §10),
  carried into the load as a held pre-force the way a slipping corner's held friction is; the
  reference and the re-closure count live for one solve and start at zero in the next. The
  first re-closure keeps the full offset. A cohesive corner never slips. A slip-releasing corner
  (`c == 0`) that re-closes with its trial shear above `μN` re-enters Slipping holding `μN`
  (slice 2d) instead of re-closing Stuck. Both rules choose among equally admissible elastic
  fields: a Stands is certified by the readings it returns (equilibrium on every free body and
  the cone test on every closed corner), not by the path that reached them, so readings at a
  point that would otherwise cycle are path-dependent by up to about 1 W per interface.
- **Free releases in the active set** (slice 2): a corner on an interface whose profile has
  `TensileStrengthPa == 0` opens when its trial `N < 0` and then carries nothing, and (slice 2c,
  DESIGN §10) that opening threshold is `N < −floor`, the force-noise floor (one named constant
  relative to the component's force scale, of the order of the held-force tolerance), so a
  noise-level negative trial opens nothing; a corner on an interface with zero tension and zero
  cohesion whose trial `|N|` lies within the floor is open as well, reading `N = 0`, `S = 0` and zero
  utilisation, re-closing when its trial normal rises above twice the floor (hysteresis, so a
  corner at the floor cannot chatter) or when the restore step finds it closing under a motion,
  so a dry perpend carrying nothing cannot flip the active set (the slip test itself has no slack: it
  drove no test, DESIGN §10). A closed tensionless corner whose trial `N` lies in `[−floor, 0)`
  reports `N = 0`, so the interface's reported normal force, moment and tension demand agree at
  zero rather than reading noise over a zero tensile capacity as +inf. A component whose force
  scale (the sum of |weight| and |load| over its free bodies) is not finite reads Undecided,
  never Stands, with its readings filled from the overflowed solve (tensile normal forces reported as computed, not clamped as noise); a corner on an interface whose profile has `ShearCohesionPa == 0`
  slips when its trial `|S|` exceeds `μ N` (and the cap `MaxShearStrengthPa · A/4`), after which its shear is held at capacity
  opposite the trial slip direction with no stiffness along that direction and **no dilation**
  (non-associative, DESIGN §9 row 8). Transverse to the held direction, in the joint plane, a
  slipping corner carries the tangent of its held friction, `kt · capacity / |trial S|` (at most
  `kt / (1 − 1e-9)`, since a corner slips only while its trial is at least the capacity less
  the stick margin; the explicit clamp was withdrawn as moot): the stiffness with which the friction vector rotates to follow the slip
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
  exactly zero force cycles open/closed until the cap). Restored corners re-enter as slipping,
  not stuck, on a joint with `c == 0` (review ruling 2026-10-11), because re-closing them stuck
  re-opened them on the next pass and the loop cycled (a squat dry block past both its sliding
  and tipping thresholds read Undecided instead of Mechanism); the held force's magnitude and
  direction are ruled in "The elastic authority does not guess" below, which supersedes this
  sentence where they differ. On a cohesive joint it re-closes stuck.
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
  Stands; `Mechanism` and `UnconstrainedBodies` collect every mechanism component whatever the
  overall state (a host releases those bodies even while another component is undecided), each
  component's motion normalised on its own; `ActiveSetIterations` is the maximum over
  components.
- **The elastic authority does not guess** (orchestrator ruling 2026-10-11, DESIGN §10, after
  three heuristic rounds each regressed a fixture). `Stands` and `Overstressed` are certificates
  from a converged full-rank active set. `Mechanism` is reported only when the singular active
  set is admissible as it stands, with the held forces the last full-rank solve gave. When the
  projected motion would close an open corner, the corner is restored once: stuck on a cohesive
  joint, keeping its tangential reference and not counting as a re-closure (the restore step
  rebuilds the full-rank state, it is not an iterate); on a dry joint slipping with held force `μ · N⁺` taken from **that corner's own**
  normal in the last full-rank solve, opposite the motion's tangential part, and zero transverse
  tangent on the re-entry pass. If that corner was open in the last full-rank solve too (no
  physical value exists), the component is `Undecided`. Nothing is ever seeded from a singular
  solution. Three clauses drafted with this ruling are **withdrawn** (third review,
  2026-10-11), none can produce a false release: a second restoration of the same corner as
  `Undecided` is reachable at 34 of 4,488 probe points, 32 end Undecided anyway and the other
  two are correct verdicts it would spoil; a clamp of the transverse tangent at `kt` is moot
  (a slipping corner persists only while `|trial| ≥ cap (1 − 1e-9)` and a restored one
  re-enters at zero, so the ratio never exceeds `1 / (1 − 1e-9)`); `SolveInRange` on the load
  minus its projection would change only readings on a Mechanism, never a verdict (seeding
  reads the full-rank solve), and stating it correctly needs the scaling (the residual wrench
  is parallel to `D⁻² · motion`). Points the elastic field cannot certify (several provably
  standing three-brick-bridge points at μ 5, and a two-brick stack past friction that it cannot
  call a Mechanism) read Undecided and are owned by the slice-4 kinematic certificate; the
  default suite pins them as never-Mechanism or never-Stands, and the strong assertions live
  under the `Deferred.` test prefix, excluded from the default run by `Scripts/Test.ps1`.
- **Readings on Mechanism and Undecided** (review ruling 2026-10-11, restored after the third
  review): a standing component's readings are its converged readings whatever the overall
  state. A mechanism component's readings are the range-space solution of its last active set
  with the released corners marked; they are informative, not a certificate, and may show a
  dry slipping corner that re-entered with zero held force in tension, which reads `+inf` on
  the tension axis. An undecided component's readings come from the last full-rank solve on
  the fail-closed path (the current pass when there is none, which no fixture reaches) and
  from the last pass on the cap path; neither is an equilibrium field (slipping corners read
  the held force while the solve also carried the transverse spring). **Slice 3 consumes
  readings only from standing components.**
- **Options are validated** (review ruling 2026-10-11): `PivotToleranceRelative` must be finite
  and in `(0, 1)` and `MaxActiveSetIterations` at least 1, else `InvalidInput`. An assembled
  stiffness that is not finite is `InvalidInput` (it is a profile or geometry fault); a load
  that overflows in assembly keeps the overflow path already pinned, reading `+inf` utilisation
  and never Stands. Neither is ever a release. Rotational unknowns are stored as `L_c · θ`
  (the scaling matrix carries `1 / L_c` on rotations) so every pivot is in N/m.
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
3-triangle falls and the 4-diamond stands; the 3DEC wedge on two planes (physical φ = 33.36°; the associative certificate switches at 31.07°); the
Couplet–Heyman semicircular arch with radial joints stands at `t/r = 0.1075` and falls below
(voussoir count from the benchmark report's discretisation guidance; hinge positions asserted
within a half-voussoir of 54.5°); the friction switch on that arch (stands at φ = 22°, slides at
21°). Bracket: the Orduña hollow pile's non-associative load factor inside [0.427, 0.553].

## Slice 4: the collapse certificate (limit analysis, pulled forward 2026-10-11)

The kinematic authority. Where the elastic standing authority fails closed (Undecided) or
reports a Mechanism it cannot certify, this answers from the static theorem of limit analysis
over the same rigid blocks and interfaces (Livesley 1978; Gilbert & Melbourne 1994; Orduña &
Lourenço 2005; the formulation BENCHMARKS groups 4 and 5 were computed with). It is a second
implementation behind its own interface, swappable like the first (DESIGN §7). This section
is the contract as built and reviewed twice (2026-10-11); the dated rulings it absorbed are in
DESIGN §10.

### The interface (`CollapseCertificate.h`)

```cpp
enum class CertificateState : uint8_t { Stands, Collapses, Undecided, InvalidInput };

struct CertificateOptions {
	Vec3   Gravity = {0.0, 0.0, -GravityMPerS2};
	int    ConeGenerators = 16;            // even, >= 4; the friction circle is approximated by this polygon
	double LoadFactorCap = 1.0e3;          // the LP's upper bound on t; reaching it means "stands with margin >= cap"
	double FeasibilityTolerance = 1.0e-9;  // relative, on the equilibrium residual, dual residual and gap
	int    MaxIterations = 200;            // interior-point iterations per solve before it is declared failed
	bool   bStallRecovery = true;          // on a stalled or failed cone solve, bound t by the rigorous dual bound and re-solve under it
};

struct CertificateResult {
	CertificateState State = CertificateState::Undecided;
	double LoadFactorLower = 0.0;          // t_in: a statically admissible field exists up to here
	double LoadFactorUpper = 0.0;          // t_out: no statically admissible field exists beyond here
	std::vector<MechanismMotion> Mechanism;        // the selected mechanism of every Collapses component, whatever State
	std::vector<BodyId> UnconstrainedBodies;
	std::vector<InterfaceId> HingingInterfaces;    // interfaces the selected mechanism moves (kinematic)
	int Iterations = 0;                            // maximum over components
	InterfaceId InvalidInterface{};
	std::vector<CertificateComponent> Components;  // one per decided component in partition order, its Bodies as the partition lists them; empty on InvalidInput (carried from 6b, item 2 amendment)
};

struct CertificateComponent {                  // one decided component's verdict: the fields mean what CertificateResult's aggregate fields mean, restricted to its bodies; bounds capped at LoadFactorCap
	std::vector<BodyId> Bodies;
	CertificateState State;
	double LoadFactorLower, LoadFactorUpper;
	int Iterations;
	std::vector<MechanismMotion> Mechanism;
	std::vector<BodyId> UnconstrainedBodies;
	std::vector<InterfaceId> HingingInterfaces;
};

class ICollapseCertificate {
public:
	virtual ~ICollapseCertificate() = default;
	// Only: empty decides every component; otherwise the components containing a listed body, loads outside them without effect; an entry that is not a live free body is InvalidInput (stage 1b-ii).
	virtual CertificateResult Decide(const Structure&, std::span<const BodyLoad>, const CertificateOptions&, std::span<const BodyId> Only) const = 0;
};

std::unique_ptr<ICollapseCertificate> MakeLimitAnalysisCertificate();
```

### The problem it solves

**Loading is proportional.** `A α = t · b` with `b` the whole wrench, gravity and `BodyLoad`
together (a point force acts at its point, so it carries a moment about the centroid exactly as
in slice 2); the dead-plus-live form is not offered. `t = 0, α = 0` is always feasible, so there
is no infeasibility detection. Objective: maximise `t` subject to `t ≤ LoadFactorCap`.

**Unknowns.** For every corner of every live interface (slice 2's four corners), one
non-negative coefficient `α_j` per friction generator `g_j = n + r (cos θ_j u + sin θ_j v)`,
`θ_j = 2πj/G`, and, when the joint has cohesion, one `κ_j ≥ 0` per cohesion generator
`ρ (cos θ_j u + sin θ_j v)`. The corner force on B is `−T n + Σ α_j g_j + Σ κ_j c_j`: the tension
offset `T` is a fixed force on the right-hand side (not scaled by `t`), friction acts on the
compressive remainder, cohesion is a separate bounded polygon. Per-corner bounds:
`Σ α_j ≤ fc A/4 + T` (crushing: `N ≤ fc A/4`) and `Σ κ_j ≤ C`; a bound that is not `> 0` drops
its group (`!(x > 0)`), so dry joints carry no cohesion columns.

**Two regions, two solves.** The inscribed solve uses the **inner region**: `r = μ`, `ρ = 1`,
`T = min(ft, c/μ) · A/4` (`ft · A/4` when `μ = 0`), `C = (c − μ · min(ft, c/μ)) · A/4`. That is
`|S| ≤ c + μ N` for `N ≥ −T`: inside DESIGN's "only compression buys friction" envelope
everywhere, exact in compression, conservative in tension, and the largest valid region of this
family. The circumscribed solve uses the **hull**: `r = μ / cos(π/G)`, `ρ = 1 / cos(π/G)`,
`T = ft · A/4`, `C = c · A/4`, which contains every admissible field. Consequences: a joint with
`ft > 0`, `c = 0`, `μ > 0` gets `T = 0` on the inscribed side (a tension-only connection reads
Undecided at best; slice 4b decides whether such profiles need a cohesion floor); on bonded
joints the band is wider than the friction polygon alone. `MaxShearStrengthPa` is **not in the
LP** (a finite cap is ignored, which over-reads capacity and is fail-open for Stands; slice 4b
adds facet rows on the corner shear before a host trusts a Stands on a capped joint).

**Rows.** Six equilibrium rows per free body (force, and moment about the centroid), a corner
force acting on B and its negative on A; grounded bodies contribute nothing. Per component
(union-find over free-free interfaces, as slice 2; shared code in `src/StaticsCommon.*`) the
forces are divided by the component's total load magnitude and the arms by its characteristic
length, so verdicts are local and scale-invariant (pinned at s = 1/16 … 16).

**Solver.** The homogeneous self-dual embedding with Mehrotra predictor-corrector on the normal
equations, `6 × free bodies` square after Schur-eliminating the diagonal bound rows and the
rank-one cap row (the plain infeasible-start method stalls when `t* = 0`, where the feasible set
has no interior); Eigen LLT; on a failed pivot the diagonal is shifted by `1e-16`, then `1e-14`, then `1e-12 ×`
the largest diagonal (coupling term included, dense and bordered alike) until the pivots are
positive, a failure at `1e-12` reading Undecided; one iterative-refinement pass on the corrector,
or 20 on both the predictor and the corrector in an iteration whose factor was shifted (DESIGN
§10, the interior-optimum defect); step 0.995 to the boundary; a feasible deterministic start (`α = min(1, bound/2G)`, lowered to the body's own load where a group touches a body whose largest normalised `|Demand|` is under 1e-6,
`t = min(1, cap/2)`, complementarity products 1, duals 0, `τ = κ = 1`); termination when the
primal residual (per block, term-size scaled), the dual residual (relative to
`1 + max(1, |s|, |Aᵀy|)`) and the gap `xᵀs/τ²` are all under `FeasibilityTolerance`. Any
non-finite value or the iteration cap marks that solve failed. Every cone solve, converged or not, also returns a **rigorous dual bound** on its optimum: with `ŷ = y/τ` and `r = Aᵀŷ + e_t`, `U = −bᵀŷ + Σᵢ ubᵢ max(0, rᵢ + γₖ (|A|ᵀ|ŷ| + |c|)ᵢ)` over the box every variable has (a group's forces and slack by its bound, `t` and the cap slack by the cap, a cap site's facet slack `K` by its bound plus `Σ_G B_G max_J |f_K · d_J|` over the site's groups, which holds for any `μ`), weak duality for a box-bounded LP valid for any `ŷ` (Neumaier & Shcherbina 2004); a non-finite term (an overflowed `ŷ`) makes `U` unbounded rather than dropping the column; `γₖ = k u / (1 − k u)` with `u` the unit roundoff and `k` the column length (`6 × free bodies + 2` for `t`, `14 + facets` for the rest), plus `γ` over the bound's own sum. **Stall recovery** (`bStallRecovery`, DESIGN §10): an inscribed solve that stalls or fails with `0 < U < cap` is re-solved with the cap at `U (1 − 1e-5)`, then `U (1 − 1e-3)`; the first that converges gives `t_in` (its duals give the mechanism unless it reached that cap, when the stalled solve's last duals do), and if neither converges the component reads Undecided; a circumscribed solve that does not converge gives `t_out = min(cap, U)`. Each re-solve is its own solve for `Iterations`. Deterministic: ordered assembly,
no threads, Eigen storage and factorisation only. The circumscribed solve is skipped when the
inscribed one reaches the cap; `t_out` is clamped to at least `t_in` (exact, every inscribed
set sits inside its circumscribed one). `t_out` is today the converged primal value; the ruling
that it should be the circumscribed **dual objective** `bᵀy / τ` (an upper bound needs dual
feasibility) is **open**, logged in CURRENT_STATE; the two agree within the 1e-9 termination gap, so the reported bound can sit on the unsafe side only by that much.

**State.** `Stands` when `t_in ≥ 1` (whatever the circumscribed solve did: if it failed,
`LoadFactorUpper = min(LoadFactorCap, U)`, or the cap with recovery off); `Collapses` when `t_out < 1`; `Undecided` when
`t_in < 1 ≤ t_out` (the polygon band, narrowed by raising `ConeGenerators`; on a cone-vertex
direction a slide flips Stands → Undecided at `μ` and Undecided → Collapses at `μ / cos(π/G)`,
on a mid-edge direction at `μ cos(π/G)` and `μ`), when the inscribed solve failed (its recovery re-solves included; bounds `[0, cap]`), when
`t_in < 1` and the circumscribed solve failed with recovery off or with `U_out ≥ cap` (bounds
`[0, cap]`), or when the mechanism selection below failed; `InvalidInput` on the slice-2 rules (dead-body load,
non-finite load or gravity) plus `ConeGenerators` odd or under 4 and non-finite or non-positive
options, with `InvalidInterface` left default. On dry joints the cones are homogeneous, so `t`
is exactly 0 or the cap; finite load factors come only from bonded, cohesive or crushing bounds
(`ft A L / (W (λh − L))` for tension tipping, `c A / (W (sin θ − μ cos θ))` for cohesive sliding,
`fc A / W` for crushing). Load factors compare at 1e-7 relative plus 1e-6 absolute. A massless
free body with no load is Stands; a free body with no interface is Collapses at `t = 0` and is
listed, and so is an ungrounded component carrying no `BodyLoad` (the rigid fall, slice 6a). Across components the state is the worst in the order InvalidInput > Undecided >
Collapses > Stands, load factors are the minimum, the lists collect **every Collapses
component whatever the overall state**, and `Iterations` is the maximum. A single state hides
which component collapsed; the combined authority (slice 4b) reports per component.

**Mechanism.** Where `t*` is finite and the dual unique, the equilibrium-row duals at the
inscribed optimum are the body velocities (sign `−y`, rotations unscaled by the characteristic
length, normalised per component so the largest physical component is 1). At `t* = 0` the dual
optimal set is a cone and the interior point returns its analytic centre, which names every
body that can move; so on a Collapses with `t_in` under the selection gate (`FeasibilityTolerance`, 1e-9; the largest dry `t_in` seen is 5.5e-10 and `t_in` scales with the tolerance, so the margin is thin but real)
a **selection LP** runs per component: over body
velocities `y` and bounding variables `u`, with `±y ≤ u`, the kinematic admissibility rows
`gᵀ (v_B − v_A) ≤ 0` for every inscribed friction and cohesion generator, the opening row
`−Tᵀ y ≤ tol` where the inner tension offset is non-zero, and a motion normalisation, choose the
mechanism with the greatest external work per unit of L1 motion (maximise `bᵀ y` subject to
`Σ u = 1`; the earlier unit-work form `bᵀ y = 1, min Σ u` is the same mechanism up to scale
but fails 1e-6 past a threshold, where unit work needs velocities near 1e6 and `τ → 0`). The
bound-group dual variables are omitted: per corner the minimum dissipation is the support
function of a strength region containing the origin, hence non-negative, and zero exactly for
dilatant opening on a dry corner or zero relative motion where `T > 0`, which the rows above
express. The selected mechanism is reported; `UnconstrainedBodies` and `HingingInterfaces`
follow the listing threshold below. The selection factor's retry escalates like the cone solves' (`1e-16`, `1e-14`, `1e-12` of the largest diagonal, coupling and opening terms included), and 3 iterative-refinement passes on the selection corrector (DESIGN §10, the modified Cholesky reverted). "Minimal" means highest work per unit
motion, not fewest bodies: a cap on an exactly critical end brick is reported with that brick
(TRAPS: a cluster released as one body must be a certified mechanism; two bricks whose joint
centroid is past a pillar's edge are named together with one rigid rotation). **If the selection
LP does not converge the component reads Undecided with empty lists**: the analytic centre is
never handed to a host as a release set. On `Stands` every list is empty. The selection has a
**resolution limit**: a harmonic stack with every offset scaled by `1 + ε` has every level past
its threshold and the optimum scales with ε, so once it falls under the solver's absolute
termination gap (ε about 1e-7 for n ≥ 5, 1e-8 for n ≥ 3) the solve returns near the centre of
the near-optimal set and names every level, or reads Undecided; every body it names there is
past its own threshold, so this is not host-unsafe, and slice 4b's referee records it:
the tests pin only that the top block is named. The cone solves have a **threshold resolution
limit** too: within about 1e-6 relative of a load factor where the admissible set empties, the
inscribed solve can stall with its primal residual near 1e-4 and the component reads Undecided
[0, cap] (the Ferris & Tin-Loi wall at G = 64, λ 9/14 + 1e-6, while G = 16 stands); fail closed.
Near an interior optimum (`t*` strictly inside `(0, cap)`) a solve can stall with only its primal residual floored; stall recovery decides it (DESIGN §10).
**Listing
threshold (ruled 2026-10-11 after the third review):** a body is unconstrained when the largest
of its six scaled components exceeds 1e-3 of the component's largest, and an interface hinges
when a corner's relative speed exceeds 1e-3 of the fastest corner and one of its bodies is
listed; the earlier 1e-6 admitted interior-point residue (tied levels at about 1e-5, a stable
bystander lifting at 1.6e-6, a weakly bonded bystander's joint read hinging) and let the two
lists disagree. An Undecided after a failed selection still carries `LoadFactorUpper < 1`.

### The theorem ruling (settles the question slice 2 left open)

The certificate implements **associative** limit analysis: the inscribed solution is a genuine
statically admissible field, so `Stands` certifies that equilibrium within every strength bound
is possible; the circumscribed solution bounds every admissible field, so `Collapses` certifies
that none exists for the associative problem, and since the non-associative collapse load never
exceeds the associative one, `Collapses` holds for the physical joint too. The gap is the other
direction: under non-associative friction a structure with an admissible field can still fail
along a sliding path (Ferris & Tin-Loi; Orduña's pile reads [0.427, 0.553] for exactly this
reason). **Ruling:** static admissibility is the standing criterion for the host where the elastic
authority found no mechanism. **Amended 2026-10-13** (DESIGN §10, the 5-diamond with friction and
the false-Stands class): where slice 2 reports Mechanism and the certificate says `Stands`, the
combined authority reports `Undecided`, releases nothing, and counts the disagreement. A probe
DEM showed that case can fall under Coulomb friction (the three-brick bridge at μ 5 and λ (2.5,
0.875) in every variant, and 3 of 5 such random dry walls robustly): the certificate's field can
rest on associative dilatancy that a sliding joint does not supply. Undecided is a ceiling, not a
collapse claim: some of these cases stand or fall on damping, and 71 of the 80 such verdicts in
the random families are on cohesive structures that the probe DEM cannot model. A red on the oracle property's never-Mechanism direction is therefore not
automatically a slice-2 defect. Where the elastic authority reads Undecided and the certificate
Stands, the combined answer stays Stands; that residual is a known unsafe gap (about a fifth of
such dry random walls fall in the probe DEM), logged in CURRENT_STATE.
The pile's non-associative factor is pinned as a bracket on `[t_in, t_out]` containing the
associative value, not as the certificate's answer.

### Anchors slice 4 must pin

Pinned in 4a: the single-brick closed forms and the polygon band, bonded tension, cohesion and
crushing load factors, point-load tipping and eccentric crushing, the slice-2 anchors through
the certificate, the deferred stack and bridge rows, the inner-region cap at `c/μ`, mechanism
selection on a sliding cap, an overhanging cap, a two-brick cluster, a bonded pier and a
running-bond wall with an end brick removed, and agreement with the test-tier NNLS oracle (0
disagreements on 300 seeded draws). Still to pin (4b): the three `Deferred.Slice4.*` cases move
back under `Statics.*` through the combined authority; the Couplet–Heyman semicircular arch
(B-ARC-01, radial joints, stands at `t/r = 0.1075` and collapses below, hinge positions within a
half-voussoir of 54.5°, voussoir count from the report's discretisation guidance); its friction
switch (B-ARC-06, stands at φ = 22°, slides at 21°); the 3DEC wedge (B-WDG-01, physical φ_crit 33.36°, the associative certificate 31.07°
within 0.05°); the Orduña pile bracket (B-ORD-01); the shear cap.

## Slice 4b: the combined authority, the shear cap, and the kinematic anchors

### The combined authority (`StructuralAuthority.h`)

One call a host makes per structure and load. It owns nothing: it is handed the two
implementations and composes their answers under the theorem ruling above.

```cpp
enum class ComponentVerdict : uint8_t { Stands, Overstressed, Collapses, Undecided, InvalidInput };

struct ComponentReport {
	std::vector<BodyId> Bodies;                   // the free bodies of this connected component, ascending slot order
	ComponentVerdict Verdict = ComponentVerdict::Undecided;
	StandingState Elastic = StandingState::Undecided;        // what the elastic authority said for these bodies
	CertificateState Kinematic = CertificateState::Undecided; // what the certificate said, or Undecided if it was not consulted
	bool bConsultedCertificate = false;
	bool bDisagreed = false;                      // elastic Mechanism against certificate Stands
	double LoadFactorLower = 0.0;                 // from the certificate when consulted, else NaN
	double LoadFactorUpper = 0.0;
	std::vector<MechanismMotion> Mechanism;       // the certificate's selected mechanism on Collapses
	std::vector<BodyId> Release;                  // bodies the host releases: the certificate's UnconstrainedBodies on Collapses, else empty
	std::vector<InterfaceId> Hinging;             // the certificate's HingingInterfaces on Collapses, else empty
	std::vector<BodyLoad> Loads;                  // the loads on these bodies, as given; compared exactly for reuse (stage 2)
	bool bReused = false;                         // copied from the previous verdict rather than solved (stage 2)
};

struct AuthorityOptions {
	StandingOptions Standing;
	CertificateOptions Certificate;               // Gravity must equal Standing.Gravity or the call is InvalidInput
};

struct AuthorityResult {
	ComponentVerdict Worst = ComponentVerdict::Undecided;  // InvalidInput > Undecided > Collapses > Overstressed > Stands
	std::vector<InterfaceReading> Readings;      // the elastic authority's readings, every live interface, ascending slot order
	std::vector<ComponentReport> Components;     // ascending by lowest body slot
	int Disagreements = 0;                       // components where the elastic said Mechanism and the certificate Stands (reported Undecided)
	std::uint64_t Revision = 0;                  // Structure::Revision() when judged (stage 2)
	std::uint64_t Lineage = 0;                   // Structure::Lineage() of the judged structure; a Previous from another lineage is unmatched (stage 2 fix)
	AuthorityOptions Options;                    // the options judged under; a Previous under other options is unmatched (stage 2)
	InterfaceId InvalidInterface{};
};

class IStructuralAuthority {
public:
	virtual ~IStructuralAuthority() = default;
	// Previous: an earlier verdict on this structure's lineage, or nullptr; clean components (same bodies, no body touched since Previous->Revision, same loads, same options) are copied with bReused set, the rest solved through one filtered Solve (stage 2).
	virtual AuthorityResult Judge(const Structure&, std::span<const BodyLoad>, const AuthorityOptions&, const AuthorityResult* Previous) const = 0;
};

std::unique_ptr<IStructuralAuthority> MakeCombinedAuthority(std::unique_ptr<IStandingAuthority>, std::unique_ptr<ICollapseCertificate>);
```

**Rules.** Options are validated first: `Certificate.Gravity` must equal `Standing.Gravity`
and the certificate options must pass the slice-4 checks (`ConeGenerators` even and at least
4, finite positive cap, tolerance and iterations), or the result is `InvalidInput` with no
readings and no components. The elastic authority then runs once over the whole structure for
the result's readings, and once more per connected component on that component's
sub-structure (its free bodies, every grounded body, the loads on those bodies): **the
sub-solve's `State` is the component's elastic verdict, never inferred from the whole-structure
state.** One global state cannot say which component is Overstressed, and an elastic Undecided
coexists with listed bodies; the first version inferred it and relabelled an Overstressed brick
Undecided beside an unrelated pass-capped one, and read NaN readings as Stands. Locality
(slice 2) makes the sub-solve's readings equal the whole-structure ones, so the cost is a second
solve, logged for slice 6. Per component (the same partition both implementations use, the
shared partition code, so the mapping of bodies to components is one definition):
- elastic `Stands` → `Stands`; elastic `Overstressed` → `Overstressed` (slice 3's domain; the
  certificate is not consulted, its crushing bound would only restate the same joint);
- elastic `Mechanism` or `Undecided` → the certificate is consulted on that component alone (a
  structure holding only that component's bodies, grounded bodies kept, loads filtered):
  `Stands` → `Stands` when the elastic said Undecided, and `Undecided` with `bDisagreed` and
  `Disagreements + 1` when the elastic said Mechanism (nothing released; the certificate's
  `Stands` stays in `Kinematic`);
  `Collapses` → `Collapses` with the certificate's lists as `Release`, `Mechanism` and `Hinging`;
  `Undecided` → `Undecided`, nothing released;
- elastic `InvalidInput` → the whole result is `InvalidInput` with the elastic's interface.
The release set is always the certificate's (listing threshold 1e-3, rotations scaled by the
component's characteristic length); the elastic authority's own `Mechanism` lists stay
informational and are never released, which is the shared listing definition slice 4a asked for
(the elastic lists at 1e-6 from a direct null space and need not change). Deterministic; no
state between calls. The three `Deferred.Slice4.*` cases assert through this authority and move
back under `Statics.*`.

### The shear cap in the LP

`MaxShearStrengthPa · A/4` bounds the corner shear with facet rows: for each of the `G` facet
directions `d_j = cos θ_j u + sin θ_j v` the shear component along `d_j` of the corner force is
at most `cap · A/4 · cos(π/G)` on the inscribed solve and `cap · A/4` on the circumscribed one (a facet row bounds the apothem, so the plain cap is already the circumscribed polygon and the inscribed one is its cos(π/G) scaling)
(the same polygon pair as the friction cone, a bound on the force not on `Σ α`, which would turn
a shear cap into a compression cap). A dry brick with `μ` large and a small cap Collapses past lateral factor `λ = cap · A / W`
(every corner at its cap) and Stands below `cos(π/G)` of it, Undecided between; at tilt the
switch is `θ = asin(cap · A / W)`, which slice 2 already pins for the elastic authority. The
cap bounds shear only: a brick at rest under a cap far below its weight Stands. **A cap of 0 is
a real cap** (the joint carries no shear; `+inf` is the only "no cap"), so a cap-0 brick under
any lateral load Collapses at `t = 0`; in the mechanism selection a cap-0 corner contributes
only its normal row (tangential slip dissipates nothing there), and a finite positive cap adds
no selection rows (its prism contains the origin, so the zero-dissipation set is the friction
cone's).

### Anchors slice 4b must pin

- B-WDG-01, the 3DEC wedge, through the certificate with `ConeGenerators` high enough that the
  polygon band is under the 0.05° tolerance (`1/cos(π/G) − 1` times μ: G = 64 gives 0.12 % of
  μ, about 0.04° at 33°); the skipped elastic test stays skipped and says so.
- B-ARC-01 and B-ARC-02, the Couplet–Heyman semicircle: radial joints, centreline radius `r`,
  unit depth, density any, `μ ≥ 0.5` (use 0.7). Stands at `t/r = 0.1075 + 0.002`, Collapses at
  `0.1075 − 0.001`; the haunch hinge within one joint of 54.48° from the crown (joints every 5°
  plus a joint at ±54.484°, or voussoirs of 2°; B-ARC-09's "discretisation only raises the least
  thickness" is a DDA observation and is **false for exact sector voussoirs**: the joints are
  a subset of the continuous sections, so the discrete least thickness is at or below the
  continuous one (0.1074601 at 3° joints by a thrust-line oracle); with joints at ±54.484° it
  is exactly 0.107478, which is the pin). On Collapses the
  hinging set is the crown, the two haunches and the two springings (symmetric mechanism; the
  selection may return one side's four hinges, so assert the haunch and crown hinges and that
  every hinge is one of the seven joints 0°, ±54°, ±54.484°, ±90°, the haunch hinge accepted within
  one joint (±51°, ±57°) as B-ARC-02 allows; the 0.484° sliver between
  54° and 54.484° lists both of its joints, the selection mixing two admissible mechanisms
  below least thickness, not a physical event).
- B-ARC-06, the friction switch on that arch at least thickness: Collapses (sliding) at φ = 21°
  (μ 0.384) and Stands at 22° (μ 0.404) with `G = 32` so the band (0.5 % of μ) clears the 0.3 %
  gap between tan 22° and tan 21.57°.
- B-ORD-01, the Orduña & Lourenço hollow pile, if the block layout can be taken from the cited
  paper or the research notes: `[t_in, t_out]` must contain a value in [0.427, 0.553] and the
  elastic authority's verdict at λ = 0.4 must not be Mechanism; otherwise a `Deferred.Slice4b`
  case with the reason.
- The combined authority: the three deferred cases; a structure with one Overstressed and one
  Collapses component reports both per component with `Worst = Collapses`; the disagreement
  count on the dry bridge at μ 5, λ (2.5, 0.875) is 1 (elastic Mechanism, certificate Stands)
  and the release set is empty; a pure elastic Stands never consults the certificate.

## Slice 3: the cascade (crack, sever, release)

The brittle loop of DESIGN §5 on top of the combined authority: judge, release what the
certificate collapses as rigid clusters, crack the top band of over-capacity bonded joints where
the structure is Overstressed, re-judge, until a pass changes nothing. It completes the verdict
type of DESIGN §2.2. The host runs it on its snapshot (§2.4): `Run` mutates the structure it is
given (stamps, removals), and the verdict lists every change so a host can mirror it onto
another copy.

### Structure additions (`Structure.h`, `Interface.h`)

```cpp
bool Crack(InterfaceId, std::uint32_t Pass);   // Intact -> Cracked, stamps CrackPass; false otherwise
bool Sever(InterfaceId, std::uint32_t Pass);   // Intact or Cracked -> Severed, stamps SeverPass; false otherwise
std::uint32_t HighestPass() const;             // the pass counter: the largest stamp written or pass marked, 0 when none
bool MarkPass(std::uint32_t Pass);             // raises the counter to Pass without stamping (a counted pass that wrote no stamp); false for 0
StrengthProfile EffectiveProfile(InterfaceId) const;  // the profile with TensileStrengthPa = ShearCohesionPa = 0 when Cracked, Severed or Removed (a dead joint reads the weaker form, fail closed)
```

State is irreversible: `Crack` on anything but `Intact` and `Sever` on `Severed` or `Removed`
return false and change nothing; a pass of 0 is refused (0 means never). `IsLive(InterfaceId)`
is now **`Intact` or `Cracked`**: a severed joint carries exactly nothing (TRAPS), so it leaves
`LiveInterfaces`, `InterfacesOf` and both authorities, and a body whose joints are all severed
is a free body with no interface (Collapses at `t = 0`, listed). `RemoveBody` sets a body's
`Intact` and `Cracked` interfaces to `Removed` with no stamp, as before, and **leaves a
`Severed` interface `Severed` with its stamp**: the stamp is history and a removal never
overwrites history. Pass numbers belong to the structure: `HighestPass` survives across calls
and a later cascade continues from it.

**Both authorities read `EffectiveProfile`**, never the raw profile, so a cracked joint is a dry
joint to them: compression, friction and the shear cap kept, tension and cohesion gone,
stiffness unchanged. The crushing bound, `μ`, `MaxShearStrengthPa` and the two stiffnesses are
untouched by a crack.

### Per-axis readings (`StandingAuthority.h`)

`InterfaceReading` gains `CompressionUtilisation`, `TensionUtilisation` and
`ShearUtilisation`, the slice-2 ratios per axis as the elastic authority computes them today
(`Utilisation` is their maximum and `Axis` names the maximum, with the existing tie order), so
the cascade never recomputes a strength rule from corner forces (the first slice-4b round
showed what a second copy of that rule costs). A non-finite ratio stays non-finite.

### The interface (`Cascade.h`)

```cpp
enum class CascadeState : uint8_t { Stands, Overstressed, Undecided, InvalidInput, PassCapReached };

struct FailedInterface {
	InterfaceId Id;
	std::uint32_t Pass;
	GoverningAxis Axis;        // the crack: the larger of tension and shear; the sever: Tension if the mechanism opens it beyond friction dilatancy (none on a zero-cap joint), else Compression if it closes, else Shear if it slides, else the reading's axis (None when unread)
	double Ratio;              // the crack ratio, or the elastic reading's utilisation at release
	bool bSevered;             // false: cracked (bond lost, friction kept); true: severed with its cluster
};

struct ReleasedCluster {
	std::uint32_t Pass;
	std::vector<BodyId> Bodies;            // ascending slot order; released as one rigid body by the host
	MechanismMotion Motion;                // the certificate's motion of the lowest-slot body; the others share it
	std::vector<InterfaceId> Severed;      // every live interface that crossed the cluster boundary, now Severed
	double LoadFactorLower, LoadFactorUpper;  // the certificate's bracket for the component
};

struct CascadeOptions {
	AuthorityOptions Authority;
	double CrackBandFraction = 0.5;       // delta: joints with ratio >= R_max (1 - delta) crack this pass; 0 = one event per pass (ties together), 1 = all at once
	int MaxPasses = 32;                   // counted passes before PassCapReached
};

struct CascadeVerdict {
	CascadeState State = CascadeState::Undecided;
	std::uint32_t FirstPass = 0, LastPass = 0;   // stamps written by this run; FirstPass = LastPass + 1 of the previous run
	int Passes = 0;                              // counted passes: those that cracked or released something
	std::vector<FailedInterface> Failures;       // ascending (pass, slot)
	std::vector<ReleasedCluster> Clusters;       // ascending (pass, lowest body slot)
	std::vector<InterfaceReading> Readings;      // the final judge's readings, every live interface
	std::vector<ComponentReport> Components;     // the final judge's components
	InterfaceId InvalidInterface{};
};

class ICascade {
public:
	virtual ~ICascade() = default;
	// Previous: the caller's last verdict or nullptr; each pass's verdict is the next pass's Previous (stage 2).
	virtual CascadeVerdict Run(Structure&, std::span<const BodyLoad>, const CascadeOptions&, const AuthorityResult* Previous) const = 0;
};

std::unique_ptr<ICascade> MakeCascade(std::unique_ptr<IStructuralAuthority>);
```

### The pass

Options first: `CrackBandFraction` must be finite in `[0, 1]` and `MaxPasses ≥ 1`, else
`InvalidInput`; the authority options are validated by the authority. A load on a body dead at
entry is `InvalidInput` (the authority's rule). Then, with `Pass = HighestPass() + 1`:

1. **Judge** with the combined authority on the live structure and the loads of live bodies (a
   load on a released body left with it). `InvalidInput` ends the run as `InvalidInput`.
2. **Release.** For every component with Verdict `Collapses`: group its `Release` bodies into
   clusters, the connected components of the released bodies over the live interfaces **not** in
   the certificate's `Hinging` list (bodies joined by a non-hinging interface share the
   certified rigid motion; TRAPS: a cluster released as one body must be a certified mechanism,
   and this one is the certificate's). For each cluster, ascending by lowest body slot: sever
   every live interface between a cluster body and a body outside it (stamped `Pass`, recorded
   as a `FailedInterface` with `bSevered`, the elastic reading's utilisation as `Ratio`, and an `Axis` from the mechanism: relative velocity B minus A at the joint's four corners, a body outside the mechanism still; `Tension` if some corner's normal opening less μ times its tangential speed exceeds 1e-3 of the fastest corner's relative speed, else `Compression` if some corner closes faster than that threshold, else `Shear` if a tangential speed exceeds it, else the reading's axis; a zero-cap joint (`MaxShearStrengthPa = 0`, NormalOnly in the certificate) subtracts no friction lift), then
   `RemoveBody` every cluster body (its internal joints go with it, `Removed`, no stamp). The
   cluster's `Motion` is the certificate's motion of its lowest-slot body.
3. **Crack.** For every component with Verdict `Overstressed`: the eligible joints are its live
   `Intact` interfaces whose profile is bonded (`TensileStrengthPa > 0` or
   `ShearCohesionPa > 0`) with crack ratio `max(TensionUtilisation, ShearUtilisation)` not
   `≤ 1` (so NaN is over). `R_max` is the largest **finite** eligible ratio; a joint cracks when
   its ratio is NaN or `+inf` (every pass it is eligible) or `≥ R_max · (1 − δ) · (1 − 1e-9)`
   (the relative edge keeps two identical joints that differ by rounding in the same pass).
   Cracked joints are stamped `Pass` and recorded with the crack ratio and its axis. A joint over
   one on compression only (crushing) stands and logs: no crack action, as in the old project; a
   dry joint over one cannot be Overstressed on a crackable axis and is never eligible.
   `Undecided` and `Stands` components change nothing.
4. A pass that released or cracked anything is **counted** and the loop repeats with
   `Pass + 1`; a pass that changed nothing ends the run and is not counted. Termination is
   structural: joints never heal, every counted pass stamps at least one, so the run ends within
   the live interface count; `MaxPasses` is a budget, and reaching it after a counted pass ends
   the run as `PassCapReached` with the structure as it stands then (the host sees a partial
   cascade and calls again; the next run continues from `HighestPass`).

`FirstPass` is `HighestPass() + 1` at entry and `LastPass` the last counted pass; every counted
pass ends with `MarkPass(Pass)`, so a release that wrote no stamp (a free body, a body whose
last joints a neighbour severed) still advances the structure's counter and a later run never
reuses a pass number; a run that changed nothing has `LastPass = FirstPass − 1` and
`Passes = 0`. The counter is saved: the file writes `highest_pass` and the reader takes the
larger of that field and the stamps it reads, so a reloaded structure continues numbering.
`Sever` refuses a pass below the joint's `CrackPass` (the file reader refuses the same), and
`Run` reads `InvalidInput` when `HighestPass()` is `UINT32_MAX` (the counter cannot advance), and a run that exhausts the pass counter ends at the cap: `PassCapReached` with `LastPass == UINT32_MAX`.

**The structure file carries state** (slice 1b format, extended here): every interface whose
two bodies are live is written, `Intact`, `Cracked` or `Severed`, with `state`, `crack_pass`
and `sever_pass` (written only when not `Intact`, so a file of an intact structure is unchanged),
and an optional interface `name` the writer emits when one was given, so a saved joint can be
found again; a file without those fields reads `Intact`, unstamped and unnamed, so old files
load.

The final `State` is the last judge's `Worst` mapped to `CascadeState` (`Stands`,
`Overstressed`, `Undecided`), or `PassCapReached`; `Collapses` cannot be final because every
collapsing component was released in its pass, and a `Collapses` component with an empty
release list (the certificate named nothing) is reported `Undecided`, fail closed. On
`PassCapReached` the cascade judges once more after the capped pass, so `Readings` and
`Components` always describe the structure as it stands when the run returns.
`Readings` and `Components` are the final
judge's, so a host reading utilisations after a cascade reads the structure that remains. The
run is deterministic: ordered by slot everywhere, no state outside the structure.

### What slice 3 must pin

- Structure: the state machine (`Crack` only from `Intact`, `Sever` from `Intact` or `Cracked`,
  nothing from `Severed` or `Removed`, pass 0 refused, stamps written once), `HighestPass`,
  liveness of `Cracked` and not of `Severed`, `RemoveBody` keeping a `Severed` interface and
  its stamp, `EffectiveProfile` on each state, the file reader/writer round-tripping state and
  stamps or a recorded decision that they do not (CURRENT_STATE has the format note).
- Readings: the three per-axis ratios agree with `Utilisation` and `Axis` on the slice-2
  anchors (brick under uplift: tension; sliding brick: shear; crushed brick: compression).
- Both authorities on a cracked joint: a bonded brick that stands under an uplift below
  `ft · A` Collapses (certificate) and reads Mechanism or Undecided (elastic) once the bed is
  cracked; the cracked joint keeps friction (a cracked bed on a 20° slope at μ 0.7 still stands).
- Cascade: a dry brick past friction releases `{brick}` on pass 1 with the bed severed and
  stamped, the structure left empty and `State = Stands`; the two-brick stack past friction at
  the ground releases both as **one** cluster (the joint between them is not hinging); the
  harmonic three-stack with the top block pushed past its edge releases `{top}` alone; a bonded
  three-brick pier with a lateral point load that over-stresses the base joint in tension
  (ratio over one) and leaves the others under one cracks the base on pass 1, then on pass 2
  the dry base tips and the pier is released as one cluster (sever stamped 2), `Passes = 2`; the
  top band: three bonded joints at ratios about 3, 2 and 1.2 with `δ = 0.5` crack the first two
  on pass 1 and not the third, with `δ = 0` only the first, with `δ = 1` all three; a crushing
  overload cracks nothing and ends `Overstressed` with `Passes = 0`; an Undecided component
  ends `Undecided` with nothing stamped; `MaxPasses = 1` on the pier ends `PassCapReached`
  after pass 1 with the base cracked, and a second `Run` continues at pass 2 and finishes; a
  grounded-only structure is `Stands` with `Passes = 0`; `CrackBandFraction` NaN or 1.5 is
  `InvalidInput` with nothing stamped; a free body with no interface is released on pass 1 as
  its own cluster; two independent collapsing components in one pass give two clusters in slot
  order; running the same input twice gives identical verdicts.

## Slice 5: the catalogue, the geometry layer and the structural fuzz

Three producers above the engine (DESIGN §2.1: "a producer above the engine, like the test
fixtures, and the engine never depends on it"). Nothing in `src/` solver code includes them.

### 5a. The catalogue (`Catalogue.h`, `data/catalogue.json`)

Materials and connection types are data (CLAUDE.md). The catalogue is a JSON file in the repo
read by a loader that converts **once** (MPa → Pa, kN → N) and never again; every row cites its
source and says whether it is a **characteristic** (5 % fractile, code) or **mean** (measured)
value, because the mean/characteristic ratio is not uniform across axes (TRAPS "Strength data":
tension ×7, cohesion ×4.5, compression ×1 in the old project; these rows give ×6 and ×2), so a re-anchor re-measures and
never re-scales.

```cpp
enum class StrengthBasis : uint8_t { Characteristic, Mean };

struct JointRow {                       // an area joint: masonry bed or perpend, timber bearing
	std::string Name, Kind, Source, Note;   // Kind: "bed", "perpend", "bearing"
	StrengthBasis Basis;
	StrengthProfile Profile;                // Pa and Pa/m, converted by the loader
	double FrictionNormalStressPa;          // the normal stress the friction value was measured at; NaN for a code value
};

struct FastenerRow {                    // a discrete fastener: nail, screw, bolt
	std::string Name, Source, Note;
	StrengthBasis Basis;
	double ShearPerPlaneN;                  // F_v,Rk per shear plane
	double WithdrawalN;                     // F_ax,Rk; 0 when the source gives none (a bolt)
	double BearingStrengthPa;               // the timber's f_c,90 the fastener bears on, for the crushing bound
	double SlipModulusNPerM;                // EN 1995-1-1 Table 7.1 K_ser per shear plane per fastener (slip_modulus_mn_per_m × 1e6)
	double BearingNormalStiffnessPaPerM;    // the bearing member's normal stiffness (bearing_normal_stiffness_gpa_per_m × 1e9)
};

struct Catalogue {
	std::vector<JointRow> Joints;
	std::vector<FastenerRow> Fasteners;
	const JointRow* FindJoint(std::string_view Name) const;        // nullptr when absent
	const FastenerRow* FindFastener(std::string_view Name) const;
};

struct CatalogueResult { Catalogue Value; FileError Error; bool Ok() const; };   // FileError as StructureFile.h
CatalogueResult LoadCatalogue(std::string_view Json);

// μ = 0 (independent axes, DESIGN §4): cohesion = cap = Count · ShearPerPlaneN / AreaM2, tension = Count · WithdrawalN / AreaM2, compression = BearingStrengthPa,
// tangential stiffness = Count · SlipModulusNPerM / AreaM2, normal stiffness = BearingNormalStiffnessPaPerM (the area cancels in stiffness as in capacity).
// Count is the number of fasteners crossing THIS interface, which is one shear plane: a double-shear fastener crossing two interfaces is Count 1 on each; its withdrawal is then counted on both.
StrengthProfile FastenerProfile(const FastenerRow& Row, int Count, double AreaM2);
```

**File.** `{"version": 1, "joints": [...], "fasteners": [...]}`. A joint row carries `name`,
`kind`, `basis` ("characteristic" | "mean"), `compressive_mpa`, `cohesion_mpa`, `tensile_mpa`,
`friction`, `shear_cap_mpa` (absent = no cap), `normal_stiffness_gpa_per_m`,
`tangential_stiffness_gpa_per_m`, `friction_normal_stress_mpa` (optional), `source`, `note`.
A fastener row carries `name`, `basis`, `shear_per_plane_kn`, `withdrawal_kn`,
`bearing_mpa`, `slip_modulus_mn_per_m`, `bearing_normal_stiffness_gpa_per_m`, `source`, `note`. The loader refuses (with the line, as `LoadStructure` does) a
missing or mistyped field, an unknown key (a misspelt optional key would otherwise fail open), a
negative value, a value that is not finite after conversion, a non-positive stiffness, a duplicate
name, an unknown kind or basis, and a profile the `Structure` would refuse (it runs `AddProfile`
on a scratch structure); on any refusal the result carries no rows at all. Conversion:
`× 1e6` for MPa, `× 1e9` for GPa/m, `× 1e3` for kN, exactly once, in the loader; the tests
assert `Profile.TensileStrengthPa == tensile_mpa × 1e6` for a row read from the file, not
from a constant in the test.

**Rows the first file carries** (every number from BENCHMARKS §8 and the research notes, with
its URL in the row's `source`; a value with no source is omitted, never invented; the slice-5
close rulings in DESIGN §10 govern the conversions):
- `clay_gp_mortar_bed_ec6` (bed, characteristic): EN 1996-1-1 Table 3.4 `f_vk0 = 0.30` MPa
  (M10–M20), friction 0.4 (the `0.4 σ_d` term of eq. 3.5), cap `0.065 f_b` with `f_b = 20` MPa
  stated in the note (1.30 MPa), compressive strength from eq. 3.2 `f_k = K f_b^0.7 f_m^0.3`
  with `K = 0.55` (Table 3.3, clay Group 1, GP mortar), `f_b = 20`, `f_m = 10` MPa (M10):
  `0.55 · 20^0.7 · 10^0.3 = 8.93` MPa, derived in the note. **Tension is direct**: the code's
  `f_xk1 = 0.10` MPa (§3.6.3) is a flexural strength through `Z = b t² / 6`, and the engine's
  rigid-plastic hinge gives `M = f_t A t / 2 = 3 f_t Z`, so the row carries `f_xk1 / 3 =
  0.0333` MPa, which reproduces the code moment exactly. **Stiffness is a course's**: `E = K_E
  f_k`, `K_E = 1000` (§3.7.2) is the masonry modulus, unit and joint together, and a rigid
  unit puts a whole course's deformation into one spring: `8.93 GPa / (0.065 + 0.010) m = 119
  GPa/m` normal, `G = 0.4 E` (§3.7.3): 47.6 GPa/m tangential.
- `clay_gp_mortar_perpend_ec6` (perpend, characteristic): the **filled** perpend, the same
  unit–mortar bond as the bed, so every value is the bed row's. EN 1996-1-1 gives no strength
  for the perpend joint itself: eq. 3.6 (`0.5 f_vk0`, cap `0.045 f_b`) is the shear strength
  along the *bed* of a wall whose perpends are unfilled, and `f_xk2` a wall flexural strength
  whose failure plane runs through the units. An unfilled perpend is a dry profile.
- `clay_gp_mortar_bed_gooch_mean` (bed, mean): Gooch et al. 2023/2025: direct tension 0.20
  MPa (the middle of 0.15–0.25, a direct value so no conversion), cohesion 0.60 MPa (just above
  the low end of the couplet 0.58–1.04), friction 1.05 within the 1.04–1.16 measured at
  `friction_normal_stress_mpa = 0.22`, compression and stiffness as the code row (not measured
  there, the note says so).
- `lime_mortar_handmade_brick_upc_mean` (bed, mean): Segura et al.: `c = 0.16` MPa,
  `μ = tan 33.71° = 0.667`, tension absent in the source so `0` with the note "no tension
  measured; the dry-bond floor", compression and stiffness as the code row with the note.
- `c24_bearing_perp_to_grain` (bearing, characteristic): `f_c,90,k = 2.5` MPa × `k_c,90 =
  1.5` = 3.75 MPa compressive (Swedish Wood Vol. 3 Ex 3.2; EN 1995-1-1 §6.1.5), no tension,
  no cohesion, friction 0.3 (Wood Handbook, the low end of 0.3–0.5, snippet-matched, the row's
  caveat), stiffness over the 45 mm joist width of Ex 3.2, the one bearing dimension the example
  gives: `E_90,mean = 0.37` GPa (EN 338 C24) / 0.045 m = 8.2 GPa/m normal, `G_mean = 0.69` GPa
  (EN 338 C24) / 0.045 m = 15.3 GPa/m tangential; a different bearing depth re-derives both.
- Fasteners (Swedish Wood Vol. 3 Ex 4.1–4.3; EN 1995-1-1 §8.2.3(1) defines the Johansen
  expressions **per shear plane per fastener**): `ec5_screw_8x60_steel_to_glulam` 4.83 kN per
  plane (mode d), withdrawal by EN 1995-1-1/A1:2008 §8.7.2 with the example's own inputs
  (penetration 52 mm, `l_ef = 44` mm, GL30c `ρ_k = 390`): `f_ax,k = 0.52 d^-0.5 l_ef^-0.1
  ρ_k^0.8 = 14.89` MPa, `F_ax,Rk = 14.89 · 8 · 44 = 5.24` kN (the Structural Basics 1.74 kN is
  a 2.5 mm screw and is not this row); `ec5_ring_nail_4x40_steel_plate` 1.607 kN as published
  with the note "a recomputation gives about 1.48 kN", withdrawal 0.42 kN (the 3.1 mm nail
  datum, noted); `ec5_bolt_m16_double_shear` 12.9 kN **per shear plane** (mode k; 25.8 kN per
  bolt across its two planes). No "mean" fastener row: Blass & Colling's 1.074 / 1.073 is an
  EN 14358 *characteristic* ratio of test to the yield-model prediction with the 1.05 / 1.15
  factors removed, so a mean fastener row waits for a cited mean. Every fastener row carries
  EN 1995-1-1 Table 7.1's slip modulus `K_ser` per shear plane per fastener in `slip_modulus_mn_per_m`
  (`ρ_m^1.5 d / 23` for screws and bolts, `ρ_m^1.5 d^0.8 / 30` for nails without predrilling,
  ×2 steel-to-timber by §7.1(3), GL30c `ρ_m = 430`): screw 6.20, nail 1.80, bolt 12.4 MN/m; and
  the bearing member's `bearing_mpa` 3.75 and `bearing_normal_stiffness_gpa_per_m` 8.2 (the
  C24 row's, noted).

**Catalogue anchors.** A brick (0.215 × 0.1025 × 0.065 m, 2000 kg/m³) on the `..._bed_ec6`
profile: pure uplift Collapses at `ft · A` and Stands below (B-CAT-01 through the certificate,
the number written as `Row.Profile.TensileStrengthPa × AreaM2` in the test); the perpend row
carries the bed row's cohesion, tension and cap; the mean row's direct tension is `0.20 /
0.0333 = 6×` the code row's and its cohesion `2×` (B-CAT-02; TRAPS "Strength data" quotes the
old project's ×7 / ×4.5, the same order); `FastenerProfile(screw, 2, A)` on a joint of area `A`
under pure shear Collapses at `2 × 4.83` kN and under pure tension at `2 × 5.24` kN, whatever
`A` (the area cancels: B-CAT-06/08), **through `Judge` as well as the certificate**, so the
elastic authority accepts the fastened joint; the bolt row is 12.9 kN per plane (B-CAT-07);
the loader refuses a row whose tension is negative and a duplicate name with the line number,
and leaves nothing loaded on any refusal.

### 5b. The geometry layer (`geometry/BoxContacts.h`, `geometry/RunningBondWall.h`)

```cpp
struct Box { Vec3 CentreM; Vec3 HalfExtentsM; };
// The overlap rectangle of two axis-aligned boxes touching on one face within Tolerance; nullopt when they do not touch or overlap in volume.
std::optional<InterfaceDesc> FaceContact(BodyId A, const Box& BoxA, BodyId B, const Box& BoxB, ProfileId Profile, double ToleranceM = 1e-9);

struct WallSpec {
	int Courses = 10;  int BricksPerCourse = 8;
	Vec3 BrickM = {0.215, 0.1025, 0.065};     // length along x, thickness along y, height along z
	double DensityKgPerM3 = 2000.0;
	ProfileId Bed, Perpend;                   // the bed also serves the base course on the ground
	bool bRunningBond = true;                 // odd courses offset by half a brick, with half bricks at the ends
	double GroundZM = 0.0;
	Vec3 OriginM = {0, 0, 0};                 // shifts every brick and the base beds (slice 6a)
	bool bAlongY = false;                     // length along y, thickness along x; brick centre = origin + (0, s, z)
	BodyId Ground;                            // Invalid: make a new grounded ground; valid: build on it and add none
};
struct Wall { std::vector<std::vector<BodyId>> Courses; std::vector<InterfaceId> Beds, Perpends; BodyId Ground; double LengthM, HeightM; bool bComplete; };   // bComplete false when any AddBody/AddInterface was refused (an Invalid profile): the wall is then not a wall
Wall BuildRunningBondWall(Structure& Model, const WallSpec& Spec);
```

Bricks are bodies (mass = density × volume, centroid at the box centre); joints are
`FaceContact` results: beds between a brick and every brick below it that overlaps in plan (two
per brick in running bond, one at the ends), perpends between neighbours in a course, the base
course on a grounded body (`OnGroundPlane` semantics). Normals point from the lower or
left body to the upper or right one. Deterministic order: courses bottom-up, bricks left to
right. A wall of `n` courses and `m` bricks in stretcher bond has `n·m` bricks when `n` is
even-only or `bRunningBond` is false, and half bricks (half mass, half length) at the ends of
odd courses otherwise, so every course spans the same length.

**Geometry anchors.** `FaceContact` on two unit cubes stacked gives the full 1 × 1 face at the
shared plane with the normal +z from the lower to the upper; shifted half a cube gives a
0.5 × 1 rectangle at the overlap's centre; separated by `2 × Tolerance` gives nullopt;
overlapping in volume gives nullopt; a side-by-side pair gives the normal ±x. The wall: brick
and joint counts match the closed forms; every joint passes `AddInterface`; under gravity on the
dry profile the wall Stands, elastic and combined (since slice 2c the elastic authority settles every dry wall at rest in two iterations; laterally loaded dry walls can still read Undecided, CURRENT_STATE); a dry wall of 10 courses under a lateral
load along y overturns at `λ = t / H` (B-WAL-01, through the certificate bisection on λ, the
classical monolith value, since a running-bond wall with dry beds rocks as one block when
`μ > t/H`); the same wall with the `..._bed_ec6` profile stands past that λ (bond adds
tension) and overturns at the bonded monolith factor `λ_c = (t/H)(1 + f_t A_base / W)` exactly
(certificate Stands at 0.99 λ_c, Collapses at 1.01 λ_c; the combined authority reads Overstressed
at 1.01 λ_c, since the elastic field is over capacity and no mechanism has formed, and Stands at
0.99 λ_c: the four-corner lumping puts the elastic tension onset at the plastic moment, not
below it); removing the middle brick of
the base course leaves the wall standing with the beds above the gap reading higher
utilisation than before (the load arches), through `RemoveBody` and a re-judge.

### 5c. The structural fuzz (`tests/CascadeFuzzTest.cpp`)

Seeded random small structures (stacks, running-bond walls of 3–6 courses with 0–3 bricks
removed, dry or `..._bed_ec6` profiles, tilt 0–30° or lateral λ 0–1) run through the cascade,
with these invariants checked per draw and the failing seed printed:
- **every released cluster is certified**: on a copy of the structure taken before the pass,
  the certificate on that component reads Collapses with every cluster body in its release
  list (the cluster is a subset of the certificate's list, never a reachability weld);
- **stamps are consistent**: every `Failures` entry matches the structure's record (state,
  pass), passes are contiguous from `FirstPass` to `LastPass`, `HighestPass() == LastPass`
  when `Passes > 0`, and no stamp outside `[FirstPass, LastPass]` was written;
- **termination**: `Passes ≤ live interfaces at entry`;
- **the remainder is judged**: the final `Components` carry no Collapses, and a fresh judge
  of the remaining structure returns the same verdicts;
- **determinism**: the same seed twice gives identical verdicts;
- **the file round-trips the end state**: `LoadStructure(WriteStructure(end))` has the same
  live bodies, states, stamps and `HighestPass`.
Draw counts by build (`bOptimisedBuild`), under a second in Release; coverage guards that the
run saw at least one crack, one multi-body cluster, one Overstressed end and one Stands end.

## Slice 6: scale

DESIGN §8 row 6: "sparse solve, event region, panels; the old warehouse is the benchmark: 5,612
pieces, 15,042 joints, a decision inside 200 ms." The slice opens with a **measurement**, not a
design, because every cost item logged so far (the combined authority's per-component copies,
the cascade's quadratic scans, the dense per-component LDLT, the certificate's interior point)
is a guess until timed on structures ten and a hundred times the fixtures.

### 6a. The scale measurement (`geometry/Warehouse.h`, `tests/ScaleMeasurementTest.cpp`, `Measure.Scale.*`)

A generator a step up from the wall, built from the slice-5 pieces only (boxes, `FaceContact`,
running-bond walls), and timing rows that say what the engine costs before anything is
redesigned. Nothing in `src/` solver code includes the generator.

```cpp
struct Opening { int Wall; int StartBrick; int WidthBricks; int SillCourse; int LintelCourse; };   // Wall 0/2 the long walls at y = 0 and y = width, 1/3 the short walls at x = 0 and x = length
struct WarehouseSpec {
	int LengthBricks = 40; int WidthBricks = 20; int Courses = 30;
	Vec3 BrickM = {0.215, 0.1025, 0.065}; double DensityKgPerM3 = 2000.0;
	int Bays = 4; double PostSideM = 0.2; double BeamDepthM = 0.3; double BeamWidthM = 0.2;
	double PlankThicknessM = 0.05; double PlankWidthM = 0.2; int PlanksPerBay = 4; double TimberDensityKgPerM3 = 420.0;
	ProfileId Bed, Perpend, Bearing, Plank; std::vector<Opening> Openings; double GroundZM = 0.0; double CornerGapM = 0.01;
};
struct Warehouse {
	std::array<Wall, 4> Walls; std::vector<BodyId> Posts, Beams, Planks; std::vector<InterfaceId> Bearings, PlankJoints;
	BodyId Ground; bool bComplete; std::size_t LiveBodies, LiveInterfaces;   // counts taken when the build finishes
	std::size_t BodyCount() const; std::size_t InterfaceCount() const;
};
Warehouse BuildWarehouse(Structure& Model, const WarehouseSpec& Spec);
```

Layout (pinned by `tests/GeometryWarehouseTest.cpp`'s `ExpectedLayout`): one grounded ground
under the plan; wall 0 at `y ∈ [0, T]` and wall 2 at `y ∈ [W − T, W]`, both along `x ∈ [0, L]`;
walls 1 and 3 along y (`WallSpec::bAlongY`) outside the long walls by `CornerGapM`, so the four
walls are four components (the corners are not bonded and nothing in the frame bears on a
wall); `Bays + 1` posts per long wall at `x = P/2 + k (L − P) / Bays`, `y = T + P` and
`W − T − P`, standing on the ground through `Bearing`; one beam per bay per side from post
centre to post centre on half of each post top; `PlanksPerBay` planks per bay across the width
on both beams through `Plank` (a fastener profile made for the `PlankWidthM × BeamWidthM`
contact). Every contact is a `FaceContact`; mass is density × volume; the order is walls 0..3,
posts (side 0 then 2), beams, planks, deterministic. An opening removes (by `RemoveBody`,
after the wall is built) every brick whose span lies inside it along the wall and whose
course is in `[Sill, Lintel)`; the dead ids are filtered from `Courses` but remain in `Beds`
and `Perpends`. Closed forms: bricks per wall as 5b; posts `2(Bays + 1)`; beams `2 Bays`;
planks `Bays · PlanksPerBay`; bearings `2(Bays + 1) + 4 Bays`; plank joints
`2 · Bays · PlanksPerBay`; an opening away from a wall end removes `W` bricks on even courses
and `W − 1` on odd ones. Sizes as built: the shed (12 × 8 plan, 10 courses, 2 bays, a door and
a window) 420 live bodies; the half warehouse (30 × 15 × 20, 3 bays, two openings) 1,822; the
default warehouse 3,695 (3,660 bricks, 34 timbers, the ground), 34 % below the 5,612-piece
benchmark it stands in for.

The measurement rows, `Measure.Scale.*` (excluded from every tier by `Scripts/Test.ps1`, run
one per process from a copy of the Release exe, never as a tier, because two of them do not
finish on the dense solver): for each of the 10 × 8 wall, a 40 × 32 wall (1,296 bricks), the
shed, the half warehouse and the warehouse (Release only), build; time the elastic authority
alone, the certificate alone and `Judge` at rest; on copies time one `ICascade::Run` after
removing one post and after removing wall 0's base course; print one row. Asserted: the live
body count equals the closed form, `Judge` at rest reads Stands (not for the warehouse), two
judges agree. The table is `claude_plans/SCALE.md`, with the HEAD and the machine named; 6b's
contract is written against it. First findings: the hand-rolled rank-1 `Factorise` is 10–17×
slower than a blocked dense factorisation at the same size and stops finishing near 7,800 DoF;
the judge costs twice the elastic solve (the whole structure, then every component again); the
certificate is never called when the elastic field decides; and a floating component was cut
into bricks, which led to the rigid-fall rule (below).

**The rigid fall (slice 6a).** An ungrounded component (no live joint to a grounded body) that
carries no `BodyLoad` is Collapses at load factor 0/0 before any solve: every body translates
`(0, 0, −1)` (the header's normalisation, zero rotation) and no joint hinges, so the cascade
releases it as one cluster with nothing severed. That is exact under gravity alone. A loaded
ungrounded component, and a component hanging from or wedged under a grounded body through dry
contacts, takes the general selection, which today can split it or read Undecided
(`Deferred.Slice6.*`, DESIGN §10).

### 6b. Scale design, written against SCALE.md

The 6a rows (SCALE.md) say four things: the hand-rolled rank-1 `Factorise` is 10–17× slower
than a blocked dense factorisation at the same size and stops finishing at about 7,800 DoF
(the 40 × 32 wall); the dense factor is n³ whatever its constant; the combined authority
solves the whole structure and then every component again, copying the structure per
component; and the certificate's interior point factors dense normal equations (594 s on the
half warehouse) but is never called when the elastic field decides. Nothing in the rows says a
5,000-piece structure needs a different physics, so 6b changes the linear algebra and the
bookkeeping behind the same interfaces and measures after each stage. Panels (an uncracked
bonded run solved as one body) are contingent: built only if stages 1 and 2 leave the
warehouse outside its budget.

**Stage 1a, the factorisation itself.** `Factorise` becomes a blocked right-looking pivoted
LDLᵀ in the style of LAPACK `dpstrf` (the only factor since the 6b close; until then
`StandingOptions::FactorKind::Blocked` was the default and `Reference`, the rank-1 factor,
the referee): 64-column panels, the
largest signed updated diagonal as the pivot, the same `Pivot > PivotToleranceRelative × largest`
cutoff, the same `Factorisation` shape, so the rank test, the null vectors and the Schur
handling of slipping corners are untouched. (Eigen's own dense `LDLT` was tried and rejected:
left-looking, it picks pivots before the Schur update and never stops, so it does not reveal
rank; DESIGN §10.) Until the close the property `Statics.Factor.BlockedAgreesWithTheReference`
ran the seeded structures of the oracle test and named multi-panel fixtures (walls above the
64-column panel, Stands and Mechanism) through both kinds under the DESIGN §10 rulings; at the
close it became `Statics.Factor.TheBlockedFactorReadsItsPins` (and `...AcrossPanels`) against a
generated pins header measured against the reference one last time (worst certified reading
gap 5.8e-14). The rulings it ran under: `Stands`,
`Overstressed` and `InvalidInput` agree exactly; `Mechanism` and `Undecided` may trade places
on at most max(1, draws / 1000) of the seeded draws (borderline pivots) and never on a named
fixture; an `Undecided` pair compares nothing beyond its class; body lists agree when both
read `Mechanism`; the mechanism motions agree within 1e-6 when both read `Mechanism`; open
flags, iterations (within 2) and readings (forces at 1e-7 relative plus 1e-9 W absolute,
utilisations at 1e-7 relative plus 1e-9 absolute) are compared only when both kinds read
`Stands` or `Overstressed`. The `Measure.Scale.*` rows are re-run (SCALE.md: the half
warehouse's elastic solve 59 s to 5.9 s, the 32 × 40 wall finishing in 24 s).

**Stage 1b, the sparse solve and no copies.**
- `RbsmStandingAuthority` (stage 1b-i, 2026-10-12): for a component above `DenseDofLimit`
  (default 600 DoF, 100 bodies; the largest default-suite component is 510 DoF)
  `AssembleScaledSparseStiffness` builds the scaled stiffness `D K D` as an
  `Eigen::SparseMatrix<double>` from triplets (the same 6 × 6 blocks the dense
  `AssembleStiffness` sums, both triangles stored) and the factorisation is
  `Eigen::SimplicialLDLT` under AMD ordering; at or below the limit the path is the dense one
  with `Options.Factor`'s kind, so every anchor in the suite keeps its exact path and reading
  (`Statics.Sparse.ADenseFixtureKeepsItsPath` pins two anchors bit-for-bit against a forced
  dense limit; the limit boundary itself is pinned by an 8 × 12 wall of exactly 600 DoF).
  `Factor` therefore applies at or below the limit and to the fallback only. Acceptance of the
  sparse factor (DESIGN §10, stage 1b-i ruling, amendment and constants): the factor reports
  success, every pivot of `D` is finite and above `PivotToleranceRelative × max |D|`, and five
  inverse iterations with the factor from a fixed ±1 pattern put the Rayleigh quotient above
  the same bar; otherwise the iteration is redone through `Options.Factor`'s dense factor,
  whose rank test and null basis stage 1a verified, so mechanisms above the limit are decided
  by the same code as below it. The unpivoted pivot test alone is not a rank test (182 of
  214 hidden-pivot factorisations in a forced-sparse run of the suite were rank-deficient to
  the dense factor); the estimate has a known adversary (a soft mode orthogonal to the start
  vector with a second mode within 10³ of it) whose worst consequence is an equilibrium
  residual of 1e-3 of the load, not a false Stands, logged in CURRENT_STATE. The property
  `Statics.Sparse.AgreesWithTheDensePath` runs seeded walls of 60 to 200 bodies and the
  oracle's stacks and bridges through both paths (the limit forced to 0 and to `INT_MAX`)
  under the stage-1a comparison rulings (certified states exact, trades budgeted, motions
  within 1e-6 on Mechanism pairs, readings only on certified states), and
  `Statics.Sparse.AMechanismAboveTheLimitReadsTheSameAsBelowIt` pins a 630-DoF dry wall's
  Mechanism (105 moving bodies) across the limit.
- Done (stage 1b-ii, 2026-10-12): `StandingResult::Components` (`StandingComponent`: bodies in
  partition order, the component's state and iterations, its reading slots; empty on
  InvalidInput), the per-component answer the solver already computes, so `CombinedAuthority`
  solves once and copies nothing. `ICollapseCertificate::Decide` takes a fourth argument, the
  bodies whose components it decides (empty: all; loads outside them have no effect but are
  still validated; any entry that is not a live free body reads InvalidInput, DESIGN §10).
  `Structure::Components()` returns the cached `ComponentPartition`
  (include/loadpath/ComponentPartition.h), reset by every successful AddBody, AddInterface,
  RemoveBody, Crack and Sever; it is mutable state behind a const method, so one `Structure`
  shared across threads is not safe to query concurrently (nothing in the library promised
  that before either). Pinned by `Authority.Judge.SolvesTheElasticFieldOnceAndConsultsOnlyTheUndecided`,
  `Statics.StandingResult.ReportsEveryComponent`, `Certificate.Decide.AComponentFilterReadsAsTheComponentAlone`,
  `Structure.Components.IsCachedUntilAMutation` and the equivalence guard
  `Authority.Judge.MatchesSolvingEachComponentAlone` (40 seeded multi-part structures, 161
  components, Release).
- Done (stage 1b-iii, 2026-10-12): `CertificateOptions::DenseRowLimit` (600 rows); above it
  `FactorNormal` assembles `A D Aᵀ` as triplets (the site blocks plus the load-factor coupling
  on the demand's support, DESIGN §10) and factors with `SimplicialLLT` under AMD, the
  symbolic analysis once per component, a regularised retry on failure as on the dense path;
  at or below the limit the dense path is bit for bit. Pinned by `Certificate.Sparse.*`: the
  agreement property (64 cases, state exact, load factors 1e-7, motions 1e-6, hinging equal,
  iterations within 2), the arch, pile and wedge anchors either side of tolerance on the
  sparse path, the pile bit-identical below the limit, the inclusive 600-row boundary, a
  frictionless wall that needs the regularised retry, and the shed's certificate under 8.0 s
  (Slow). Measured in SCALE.md: 589 s to 152 s, 754 s to 114 s; the profile that follows says
  where the rest goes.
- Targets, asserted as `Slow.Scale.*` ceilings only after the first measurement and recorded in
  SCALE.md: the half warehouse judges at rest in under 2 s (from 91 s); the 40 × 32 wall
  judges; the warehouse (3,695 pieces) judges in under 10 s.

**Stage 2, the event region.** Done 2026-10-12 (0fc14a5 and its fix rounds, DESIGN §10 "stage 2
opening" and its amendments), second follow-up review in progress: cleanliness is derived from `Structure::Revision()` and per-body
`LastTouched` stamps, not a dirty set; `IStandingAuthority::Solve` takes the component
filter; `AuthorityResult` records its revision and `ComponentReport` its loads and `bReused`;
`Judge(..., Previous)` copies clean components and their readings forward and solves the
dirty ones through one filtered `Solve`; `Run(..., Previous)` passes each pass's verdict to
the next. Options are part of cleanliness (`AuthorityResult::Options`, every field compared
exactly); a verdict belongs to one lineage (`Structure::Lineage()`, `AuthorityResult::Lineage`);
the standing options are validated in `Judge` before any matching. The contract as first
written follows. A judge after a change re-solves only the components that
contain a changed body or joint. `Structure` carries a version counter and a dirty set (bodies
whose joints were added, removed, cracked or severed since the last judge); `IStructuralAuthority::Judge`
takes the last verdict and returns the clean components' entries unchanged (same verdict, same
readings, flagged `bReused`), solving only the dirty ones; `ICascade::Run` passes its own last
verdict between passes. Correctness guard: `Cascade.EventRegion.ReusedComponentsEqualAFreshJudge`
re-judges everything on a copy after every pass of the fuzz and requires equality. Target: one
post removed from the warehouse decided in under 200 ms when the touched component is the
frame; one brick removed from a wall decided in the time of that wall alone.

**Stage 3, panels (contingent).** Written only if stage 2's rows leave the warehouse outside
200 ms: an uncracked bonded run is solved as one rigid body against its supports until the
crack rule needs its per-joint readings, which are then recovered by a local solve of the
panel. Not contracted here.

**Measurement discipline.** Every stage ends with the `Slow.Scale.*` rows re-run (from a copy
of the Release exe, so the repo's exe stays free) and appended to SCALE.md with the HEAD, and
with the default suites green in both configurations; a stage that changes any anchor reading
is a stage that is wrong.

### 6b, stage 3: the certificate at scale (written against the profile in SCALE.md)

**3a, the structured cap-site factor (done 2026-10-12, merged at 2623138; reviewed, fix round merged at 0214d3e).**
The structured block is the only cap-site factor since the 6b close (until then
`CertificateOptions::CapSiteKind { Dense, Structured }`, default Structured, Dense the referee).
Per cap site
the Schur block `Mww` is factored by eliminating its diagonal group rows and factoring the
facet block `diag(D_facet) + Facets K′ Facetsᵀ` with a 3 × 3 core formed as a sum of PSD terms
(DESIGN §10); `Mww⁻¹Fᵀ` is cached per site; the constant geometry is cached per component and
the normal-equation buffers are reused across interior-point iterations. Every certificate
answer is unchanged: `Certificate.CapSite.TheCertificateReadsTheSameEverywhere` pins twelve
fixtures' iteration counts and 17-digit load factors (worst movement 3e-13 relative);
`StructuredSolveMatchesTheDenseSchurBlock` (200 seeded blocks, backward error 1.5e-16 at κ
up to 4e12) and `FailureIsReportedWhereTheDenseFactorFailed` (the same positivity verdict,
NaN failing closed; 160 cases in eight kinds after the review, one per guard) pin the
factor itself, with `StructuredSolveMatchesOnTheCapSiteShape` building the site the way
`BuildProgram` does; the Slow ratio test decides the shed D, S, S, D in one process and has
the structured minimum at most 0.85 of the dense minimum (measured 0.67–0.84). Measured: shed 6.65 s to 4.4 s; the
32 × 40 wall 106.8 s to 92.4 s.

**3b, the stall rule (done 2026-10-12, the default window 20 after its review; the
complementarity restart 2026-10-13; DESIGN §10).** `CertificateOptions::StallWindow` (20; 0
disables) and `StallImprovement` (0.5): a cone solve tracks its best largest relative
residual and stops when a window of iterations has not brought it below `StallImprovement`
times the best at the window's start, the window also restarting whenever the complementarity
`(xᵀs + τκ)/(n + 1)` falls below half its value at the window's start (DESIGN §10, the stall
defect), returning the iteration cap's outcome (no iterate is
kept: one within tolerance would already have ended the solve, DESIGN §10). The complementarity
restart resets the window's best residual to the current one, which raises iterations on stuck
solves (45 to 96 on the Ferris & Tin-Loi G = 64 near-threshold case, none at the cap). The selection
solve is untouched. Pinned by `Certificate.Stall.*`.

**3c, the coupling out of the factor (done 2026-10-12, 425506a, in review).**
The bordered system is the only coupling since the 6b close (until then
`CertificateOptions::CouplingKind { InFactor, Bordered }`, default Bordered, InFactor the
referee). Above `DenseRowLimit` the normal system
`M = M_sites + c·d dᵀ` is no longer assembled with its dense coupling block: the sparse path
factors the bordered symmetric indefinite system `[M_sites + ρI, d; dᵀ, −1/c]` with
`SimplicialLDLT` under an ordering that runs AMD on the site block and eliminates the border
last, accepts positive site pivots and a negative border pivot, retries with the same ρ as
before, and solves `[r; 0]` taking the site part. Answers unchanged: iteration counts equal and
bounds identical on the 66 agreement cases and the 12 pinned fixtures (`Certificate.Coupling.*`).
Measured: the 40 × 32 wall's certificate 118 s to 34 s, the half warehouse's 89 s to 59 s.

**3d, a verdict-only option.** As ruled in DESIGN §10 "stage 3 opening", last and only if the
warm-cascade target still needs it after 3a–3c (measurement in SCALE.md).
