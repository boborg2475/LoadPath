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
	std::vector<MechanismMotion> Mechanism;        // every mechanism component, whatever State
	std::vector<BodyId> UnconstrainedBodies;       // bodies every mechanism component moves
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
  of bodies is free to move; `Mechanism` holds the null vector. This is the slice-2 collapse
  answer: a block pushed past its toe, a frictionless block on a slope, a body resting on a
  single point. **It is a verdict, not a certificate** (third review, 2026-10-11): the elastic
  path can reach a singular admissible set on a structure for which a statically admissible
  field exists (the three-brick bridge at μ 5, λ = (2.5, 0.875), reads Mechanism after 5
  passes while an NNLS search finds an admissible field at 0.7 μ and 1.02 × the load). Under
  non-associative friction the lower-bound theorem is not sufficient, so which answer is
  physical is open; the slice-4 kinematic certificate referees Mechanism verdicts as well as
  Undecided ones, and a host treats Mechanism as a release. A neutral singular set (zero load
  projection) reads Stands, so Stands does not require a non-singular stiffness.
- **Undecided**: the active set did not converge within the cap, or a restoration needed a
  value the last full-rank solve does not supply (the fail-closed clause below). Never Stands,
  never a release.
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
  joint; on a dry joint slipping with held force `μ · N⁺` taken from **that corner's own**
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
3-triangle falls and the 4-diamond stands; the 3DEC wedge on two planes at φ = 33.36°; the
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
};

class ICollapseCertificate {
public:
	virtual ~ICollapseCertificate() = default;
	virtual CertificateResult Decide(const Structure&, std::span<const BodyLoad>, const CertificateOptions&) const = 0;
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
has no interior); Eigen LLT with one regularised refactor (`1e-12 ×` the largest diagonal) on a
failed pivot and a second failure reading Undecided; one iterative-refinement pass on the
corrector; step 0.995 to the boundary; a feasible deterministic start (`α = min(1, bound/2G)`,
`t = min(1, cap/2)`, complementarity products 1, duals 0, `τ = κ = 1`); termination when the
primal residual (per block, term-size scaled), the dual residual (relative to
`1 + max(1, |s|, |Aᵀy|)`) and the gap `xᵀs/τ²` are all under `FeasibilityTolerance`. Any
non-finite value or the iteration cap marks that solve failed. Deterministic: ordered assembly,
no threads, Eigen storage and factorisation only. The circumscribed solve is skipped when the
inscribed one reaches the cap; `t_out` is clamped to at least `t_in` (exact, every inscribed
set sits inside its circumscribed one). `t_out` is today the converged primal value; the ruling
that it should be the circumscribed **dual objective** `bᵀy / τ` (an upper bound needs dual
feasibility) is **open**, logged in CURRENT_STATE; the two agree within the 1e-9 termination gap, so the reported bound can sit on the unsafe side only by that much.

**State.** `Stands` when `t_in ≥ 1` (whatever the circumscribed solve did: if it failed,
`LoadFactorUpper = LoadFactorCap`); `Collapses` when `t_out < 1`; `Undecided` when
`t_in < 1 ≤ t_out` (the polygon band, narrowed by raising `ConeGenerators`; on a cone-vertex
direction a slide flips Stands → Undecided at `μ` and Undecided → Collapses at `μ / cos(π/G)`,
on a mid-edge direction at `μ cos(π/G)` and `μ`), when the inscribed solve failed, or when the
mechanism selection below failed; `InvalidInput` on the slice-2 rules (dead-body load,
non-finite load or gravity) plus `ConeGenerators` odd or under 4 and non-finite or non-positive
options, with `InvalidInterface` left default. On dry joints the cones are homogeneous, so `t`
is exactly 0 or the cap; finite load factors come only from bonded, cohesive or crushing bounds
(`ft A L / (W (λh − L))` for tension tipping, `c A / (W (sin θ − μ cos θ))` for cohesive sliding,
`fc A / W` for crushing). Load factors compare at 1e-7 relative plus 1e-6 absolute. A massless
free body with no load is Stands; a free body with no interface is Collapses at `t = 0` and is
listed. Across components the state is the worst in the order InvalidInput > Undecided >
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
follow the listing threshold below. "Minimal" means highest work per unit
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
the tests pin only that the top block is named. **Listing
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
reason). **Ruling:** static admissibility is the standing criterion for the host. Where slice 2
reports Mechanism and the certificate says `Stands`, the certificate wins, nothing is released,
and the disagreement is counted by the combined authority (slice 4b). A future red on the
oracle property's never-Mechanism direction is therefore a defect in slice 2, not a known limit.
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
switch (B-ARC-06, stands at φ = 22°, slides at 21°); the 3DEC wedge (B-WDG-01, φ_crit 33.36°
within 0.05°); the Orduña pile bracket (B-ORD-01); the shear cap.
