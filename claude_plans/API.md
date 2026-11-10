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
struct BodyDesc { double MassKg = 0; Vec3 CentroidM; bool bGrounded = false; double CompressiveStrengthPa = 0; };
```

`AddBody` refuses a NaN or infinite mass or centroid (`NonFiniteInput`) and a negative mass
(`NegativeMass`). Zero mass is accepted: a massless body is meaningful (TRAPS). A grounded body
is part of the earth: it never moves and absorbs any load. `CompressiveStrengthPa` is the body's
material strength, used only to pick which body of a crushing joint crushes (the cascade, step 3);
0 is unknown. The scenario loader fills it from the catalogue material row's optional
`compressive_strength_mpa` (2026-11-08).

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
	bool IsKnown(BodyId) const;  bool IsKnown(InterfaceId) const;  // issued by this structure, live or removed (public 2026-10-15)
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
	const ComponentPartition& Components() const;   // cached partition, reset by every successful mutation (stage 1b-ii); safe for concurrent const callers (2026-10-15)
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
	std::vector<StandingComponent> Components;     // per component: bodies, State, ActiveSetIterations, ReadingSlots into Readings, Timings of that component's solve; empty on InvalidInput (stage 1b-ii)
	int ActiveSetIterations = 0;
	InterfaceId InvalidInterface;                  // set when State == InvalidInput
	std::vector<std::size_t> StaleLoads;            // indices into the input loads whose body this structure issued but has removed: skipped, never InvalidInput (2026-10-15)
	PhaseTimings Timings;                          // field-wise sum of the Components' Timings, in component order; all 0.0 unless built with LOADPATH_PROFILE, and always 0.0 on InvalidInput (see "Profiling timers")
};
class IStandingAuthority {
public:
	virtual ~IStandingAuthority() = default;
	// Only: empty solves every component; otherwise the components containing a listed body, Readings and Components covering those only (plus interfaces joining two grounded bodies); an entry that is not a live free body is InvalidInput (stage 2).
	virtual StandingResult Solve(const Structure&, std::span<const BodyLoad> Loads, const StandingOptions&, std::span<const BodyId> Only) const = 0;
};
std::unique_ptr<IStandingAuthority> MakeRbsmStandingAuthority(const IParallelFor* Workers = nullptr);   // non-owning; null is serial (2026-10-15, "Parallel component solves")
```

`Solve` is const, keeps no state between calls, is deterministic for a given structure, and
never mutates the structure. The host may call it on any thread.

### Parallel component solves (`ParallelFor.h`, 2026-10-15, DESIGN §10 "parallel component solves")

```cpp
class IParallelFor {
public:
	virtual ~IParallelFor() = default;
	// Calls Body(I) once for every I in [0, Count), possibly concurrently and in any order, and returns when all have finished.
	// For itself may be called from several threads at once (an authority shares its hook across concurrent solves), and each Body must run with the calling thread's floating-point environment.
	virtual void For(std::size_t Count, const std::function<void(std::size_t)>& Body) const = 0;
};
```

- **A hook must accept concurrent `For` calls from any thread.** The hook lives inside the
  authority, and `Solve` may be called on several threads at once, so concurrent solves on one
  authority share it. Engine task systems (Unreal's `ParallelFor`, TBB) already do, with the
  calling thread helping. A pool that keeps one job in shared fields without a lock is not a
  conforming hook: a second `For` would overwrite the first and skip its indices.
- **Each `Body` must run with the calling thread's floating-point environment** (rounding mode,
  denormal flushing). The control word is per thread; a worker set to round toward zero gives a
  different answer (shown by the review's probe, 2026-10-15). The library does not enforce this
  yet; enforcement is logged for the Unreal adapter slice.
- **The library never calls `For` from inside a `Body`** today. Whether a hook must support
  nested calls is decided when the certificate's consults take the hook, since that would nest.
- **The host supplies the threads.** The library never creates a thread. The Unreal adapter
  will pass the engine's task system; the tests pass a small `std::thread` pool or a serial
  hook that runs the indices in a chosen order.
- **The factory parameter is non-owning.** `MakeRbsmStandingAuthority(&Workers)` stores the
  pointer; the host keeps the hook alive for the authority's lifetime. Null (the default) solves
  serially. It is not an option: `StandingOptions` and `SameOptions` are unchanged.
- **One `For` call per `Solve`**, with `Count` = 1 + the components solved: problem 0 (the
  interfaces joining two grounded bodies) is always one of the indices. Each index solves into
  its own slot; the merge runs afterwards on the calling thread in ascending component order.
  An `InvalidInput` component returns the same result as the serial path even though later
  components were also solved.
- **The answer is bit-identical on any worker count and in any execution order**, given the
  floating-point precondition above. Only the
  elastic `SolveComponents` uses the hook; the certificate's `DecideAll` and the combined
  authority's consults stay serial until a later slice.
- **Host rule: keep one judge in flight per structure** and merge new events into the next one.
  Two concurrent solves of one component are safe (both read the structure only) but waste work;
  the library adds no claim and no global state.
- **Timings under a hook** are per-component wall times, so their sum (`StandingResult::Timings`,
  `ElasticTimings`) can exceed the call's wall time.

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
  a load on a handle this structure never issued; a non-finite load or gravity. `InvalidInterface`
  names the first. A load on a body the structure issued but has removed is skipped and listed in
  `StaleLoads`; the result equals the solve without it (2026-10-15).

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
  noise-level negative trial opens nothing. On a non-dry joint (zero tension, cohesion above zero)
  the floor is replaced by two settings (DESIGN §10, 2026-11-10): a closed corner opens when its
  trial `N < −open_tension_tolerance_relative ×` the force scale, and an open corner re-closes
  when its trial `N > close_compression_tolerance_relative ×` the force scale (both default 1e-12;
  `StandingOptions::OpenTensionToleranceRelative`, `CloseCompressionToleranceRelative`); the
  re-closure still waits while the gap is indeterminate. Dry corners ignore both. A corner on an interface with zero tension and zero
  cohesion whose trial `|N|` lies within the floor is open as well, reading `N = 0`, `S = 0` and zero
  utilisation, re-closing when its trial normal rises above `dry_reclose_factor` (default 2) times the floor (hysteresis, so a
  corner at the floor cannot chatter) or when the restore step finds it closing under a motion,
  so a dry perpend carrying nothing cannot flip the active set (the slip test itself has no slack: it
  drove no test, DESIGN §10). A closed tensionless corner whose trial `N` lies in `[−floor, 0)`
  (`[−open tolerance, 0)` on a non-dry joint) reports `N = 0`, in the readings and the pass records alike, so the interface's reported normal force, moment and tension demand agree at
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

## The structural authority: elastic-only (`StructuralAuthority.h`, 2026-11-08)

The limit-analysis certificate was removed on 2026-11-08 (DESIGN §10, that date). Its code and
its former contract are on branch `certificate_checkpoint`. The structural authority now wraps
the elastic standing authority alone.

```cpp
enum class ComponentVerdict : uint8_t { Stands, Overstressed, Collapses, Undecided, InvalidInput };

struct ComponentReport {
	std::vector<BodyId> Bodies;                   // the free bodies of this component, ascending slot order
	ComponentVerdict Verdict = ComponentVerdict::Undecided;
	StandingState Elastic = StandingState::Undecided;
	std::vector<MechanismMotion> Mechanism;       // on Collapses: the elastic MechanismMotion of this component's bodies
	std::vector<BodyId> Release;                  // on Collapses: the elastic UnconstrainedBodies of this component; else empty
	bool bReused = false;                         // copied from the previous verdict rather than solved
	std::vector<BodyLoad> Loads;                  // the loads on these bodies as judged; a change dirties the component
};

struct AuthorityOptions { StandingOptions Standing; };

struct AuthorityResult {
	ComponentVerdict Worst;                       // InvalidInput > Undecided > Collapses > Overstressed > Stands
	std::vector<InterfaceReading> Readings;       // every live interface, ascending slot order
	std::vector<ComponentReport> Components;      // ascending by lowest body slot
	InterfaceId InvalidInterface{};
	std::uint64_t Revision, Lineage;
	AuthorityOptions Options;
	std::vector<std::size_t> StaleLoads;
	PhaseTimings ElasticTimings;
	double JudgeSeconds;                          // at least ElasticTimings.TotalSeconds when profiled
};

std::unique_ptr<IStructuralAuthority> MakeElasticAuthority(std::unique_ptr<IStandingAuthority> Standing);
```

Per component: elastic Stands reads Stands, Overstressed reads Overstressed, Mechanism reads
Collapses and releases the elastic list, Undecided reads Undecided and releases nothing. Invalid
standing options read InvalidInput. Previous-verdict reuse, reading merge and stale-load
handling are unchanged. The cascade releases each Collapses component's Release list; bodies of
it joined by any live interface fall as one cluster (the certificate's hinging split is gone),
and a cluster's Motion is the elastic motion of its lowest-slot body.

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
	bool bSevered;             // false: cracked (bond lost, friction kept) or crushed; true: severed with its cluster
	FailureKind Kind;          // Cracked, Severed (exactly when bSevered) or Crushed (2026-11-08)
};

enum class FailureKind : uint8_t { Cracked, Severed, Crushed };

struct CrushedBody {                       // 2026-11-08: the body is removed; the host turns it to rubble
	std::uint32_t Pass;
	BodyId Body;
	InterfaceId Interface;                 // the joint that crushed it (the worst, when several name it)
	double Ratio;                          // that joint's CompressionUtilisation
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
	double CrackBandEdgeRelative = 1e-9;  // the band's edge is lowered by this fraction
	double CrushEdgeRelative = 1e-9;      // a joint crushes only above compression utilisation 1 + this
	double MotionAxisThresholdRelative = 1e-3;  // a severed joint's axis comes from corners moving faster than this times the fastest
	int MaxPasses = 32;                   // counted passes before PassCapReached
};

struct CascadeVerdict {
	CascadeState State = CascadeState::Undecided;
	std::uint32_t FirstPass = 0, LastPass = 0;   // stamps written by this run; FirstPass = LastPass + 1 of the previous run
	int Passes = 0;                              // counted passes: those that cracked or released something
	std::vector<FailedInterface> Failures;       // ascending (pass, slot)
	std::vector<ReleasedCluster> Clusters;       // ascending (pass, lowest body slot)
	std::vector<CrushedBody> CrushedBodies;      // ascending (pass, body slot); separate from Clusters
	std::vector<InterfaceReading> Readings;      // the final judge's readings, every live interface
	std::vector<ComponentReport> Components;     // the final judge's components
	InterfaceId InvalidInterface{};
	std::vector<std::size_t> StaleLoads;            // indices into the input loads whose body this structure issued but has removed: skipped, never InvalidInput (2026-10-15)
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
entry is skipped and listed in `CascadeVerdict::StaleLoads`; a load on a body released during
the run is dropped from later passes and not listed. Then, with `Pass = HighestPass() + 1`:

1. **Judge** with the combined authority on the live structure and the loads of live bodies (a
   load on a released body left with it). `InvalidInput` ends the run as `InvalidInput`.
2. **Release.** For every component with Verdict `Collapses`: group its `Release` bodies into
   clusters, the connected components of the released bodies over the live interfaces **not** in
   the certificate's `Hinging` list (bodies joined by a non-hinging interface share the
   certified rigid motion; TRAPS: a cluster released as one body must be a certified mechanism,
   and this one is the certificate's). For each cluster, ascending by lowest body slot: sever
   every live interface between a cluster body and a body outside it (stamped `Pass`, recorded
   as a `FailedInterface` with `bSevered`, the elastic reading's utilisation as `Ratio`, and an `Axis` from the mechanism: relative velocity B minus A at the joint's four corners, a body outside the mechanism still; `Tension` if some corner's normal opening less μ times its tangential speed exceeds `motion_axis_threshold_relative` (default 1e-3) of the fastest corner's relative speed, else `Compression` if some corner closes faster than that threshold, else `Shear` if a tangential speed exceeds it, else the reading's axis; a zero-cap joint (`MaxShearStrengthPa = 0`, NormalOnly in the certificate) subtracts no friction lift), then
   `RemoveBody` every cluster body (its internal joints go with it, `Removed`, no stamp). The
   cluster's `Motion` is the certificate's motion of its lowest-slot body.
3. **Crush and crack, one band** (crushing as rubble, 2026-11-08, decisions
   2026-11-08-forced-collapse-and-core-suite.md §2). For every component with Verdict
   `Overstressed`: the crack-eligible joints are its live `Intact` interfaces whose profile is
   bonded (`TensileStrengthPa > 0` or `ShearCohesionPa > 0`) with crack ratio
   `max(TensionUtilisation, ShearUtilisation)` not `≤ 1` (so NaN is over); the crush-eligible
   joints are its live interfaces, `Intact` or `Cracked`, with `CompressionUtilisation` not `≤ 1`.
   `R_max` is the largest **finite** ratio over both lists; a joint fails when its ratio is NaN or
   `+inf` or `≥ R_max · (1 − δ) · (1 − 1e-9)` (the relative edge keeps two identical joints that
   differ by rounding in the same pass). Crushes go first, worst ratio first (non-finite first,
   then ascending slot): each names a body (not grounded; else the one of lower known
   `CompressiveStrengthPa` when both are known and differ; else the lower along gravity; else the
   lower slot) which is removed with `RemoveBody` (its joints go `Removed`, unstamped), listed in
   `CrushedBodies` and as a `FailedInterface` of kind `Crushed`, axis `Compression`, ratio the
   joint's compression utilisation; a body two joints name is crushed once, by the worse. Its loads
   are dropped from later passes, as a released body's are. Then the in-band crack joints still
   live are cracked: stamped `Pass` and recorded with the crack ratio and its axis. A crush is
   not a crack and not a release: what the crushed body carried falls by the ordinary rules at
   the next judge. A dry joint over one cannot be Overstressed on a crackable axis and is never
   crack-eligible.
   `Undecided` and `Stands` components change nothing.
4. A pass that released, crushed or cracked anything is **counted** and the loop repeats with
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

struct MaterialRow { std::string Name, Source, Note; double DensityKgPerM3 = 0; };   // kg/m³, published value unconverted

struct Catalogue {
	std::vector<JointRow> Joints;
	std::vector<FastenerRow> Fasteners;
	std::vector<MaterialRow> Materials;                            // the optional "materials" table: name, density_kg_per_m3 (> 0), source, note
	const JointRow* FindJoint(std::string_view Name) const;        // nullptr when absent
	const FastenerRow* FindFastener(std::string_view Name) const;
	const MaterialRow* FindMaterial(std::string_view Name) const;
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

### 6a. The scale measurement (`geometry/Warehouse.h`, `tests/ScaleTestSupport.h`, `tests/ScaleMeasurementTest.cpp`, `Measure.Scale.*`)

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
shed, the half warehouse and the warehouse (Release only), build (the subject builders and their
closed-form body counts live in `tests/ScaleTestSupport.h`, shared with
`Measure.Profile.ReadingsDigest`); time the elastic authority
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
  RemoveBody, Crack and Sever; the fill is guarded by an internal mutex, so any number of threads may make const calls on one
  `Structure` at once with identical answers; mutation still needs exclusive access (2026-10-15,
  pinned by `Concurrency.ConcurrentReadersOfOneStructureAgreeWithASerialRun`). Pinned by `Authority.Judge.SolvesTheElasticFieldOnceAndConsultsOnlyTheUndecided`,
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

## Profiling timers (`PhaseTimings.h`, 2026-10-15, DESIGN §10 "profiling timers")

```cpp
struct PhaseTimings {                     // seconds; every field 0.0 unless built with LOADPATH_PROFILE
	double AssemblySeconds = 0.0;         // stiffness and load assembly, the ungrounded finiteness check
	double AnalysisSeconds = 0.0;         // sparse symbolic analysis (ordering); 0.0 on the dense path
	double FactorizationSeconds = 0.0;    // the sparse numeric factor or the blocked dense factor
	double SolveSeconds = 0.0;            // back-substitution, null basis and load projection
	double CheckSeconds = 0.0;            // the sparse smallest-eigenvalue check; 0.0 on the dense path
	double ReadoutSeconds = 0.0;          // interface readings, the rigid fall and the mechanism fill
	double TotalSeconds = 0.0;            // the whole component solve
};
```

`PhaseTimings` sits on each solved `StandingComponent` and, summed field-wise in component
order, on `StandingResult::Timings`; `AuthorityResult` carries the elastic sum of the
components solved in that call as `ElasticTimings`, the time inside certificate consults as
`CertificateSeconds`, and the whole call as `JudgeSeconds`. A reused component adds nothing.

- **The phases cover only part of `TotalSeconds`.** The pivot rank scan, finiteness checks,
  active-set updates, release restoration, cohesive dissipation and the mechanism power test
  count only in the total, so the phase sum is at most the total.
- **An abandoned reuse attempt counts.** When a matchable `Previous` is tried, the dirty
  components are solved, and merging the readings then fails, the judge re-solves from scratch;
  `ElasticTimings` holds both solves, since both were spent in this call.
- **InvalidInput.** An elastic `InvalidInput` result carries zero `Timings` (it is built fresh,
  without the components' timings) even though time was spent. The combined authority keeps
  whatever it accumulated before that point, and `JudgeSeconds` always covers the whole call.
- **Profiled sparse factor time reads high.** The profiled build splits Eigen's `compute()`
  into `analyzePattern` and `factorize` to time the analysis apart; the ordering is computed
  once but the permuted copy of K is built twice, the second booked to `FactorizationSeconds`,
  so it overstates the normal build by one sparse-matrix copy.

**The `-Profile` build.** A compile-time switch, not a runtime option: CMake option
`LOADPATH_PROFILE` (off by default) defines `LOADPATH_PROFILE=1` for the library and the tests;
preset `msvc-profile` (build presets `profile-debug`, `profile-release`) builds into
`build/msvc-profile`; `-Profile` on `Scripts/Build.ps1` and `Scripts/Test.ps1` selects it. The
fields exist in both builds so the layout never changes. **The normal build reads no clock** and
leaves every field 0.0 (`src/PhaseClock.h`'s `ScopedPhase` is empty there). Timings feed no
decision and no reuse comparison, so **answers are bit-identical across the two builds**: proved
by the profiled build passing the default suite and by `Measure.Profile.ReadingsDigest` printing
the same digest of every bit of the `AuthorityResult` (except `Revision`, `Lineage`, `Options`
and `StaleLoads`) in both. Pinned by `Profile.Timings.*` (ANCHORS.md).

## The scenario workbench (`Scenario.h`, `Viewer.h`, `Cli.h`, 2026-10-17)

The file format, its fields, the rules and the expectations are specified in
[test_plans/2026-10-16-scenario-workbench.md](test_plans/2026-10-16-scenario-workbench.md) §1 and §5
(boxes take exactly one of `centre` and `min`). The library side:

```cpp
// Scenario.h
ScenarioLoadResult LoadScenario(std::string_view Json, const Catalogue& Rows);   // Value + FileError; a refused file loads nothing
ScenarioOutcome    RunScenario(Scenario& Value, const ICascade& Cascade, const IStructuralAuthority& Authority);
ScenarioOutcome    RunScenario(Scenario& Value);                                  // the standard combined authority and cascade
ExpectationReport  CompareExpectation(const ScenarioExpect& Expect, const ScenarioOutcome& Outcome);   // pure, over names
PatternExpansion   ExpandPattern(std::string_view Pattern, std::span<const std::string> Names);       // '*' and [a-b]; no match is an error
ViewerScene        ViewerSceneOf(const Scenario& Value, const ScenarioOutcome& Outcome, std::string_view Slug);
std::string        WriteViewerJson(const Scenario& Value, const ScenarioOutcome& Outcome, std::string_view Slug);

// Viewer.h: the one writer of viewer/index.html's data format
std::string WriteViewerJson(const ViewerScene& Scene);

// Cli.h
class IFileStore { Read(path) -> optional<string>; Write(path, contents) -> bool; List(dir) -> sorted names; };
int RunCli(std::span<const std::string_view> Args, IFileStore& Files, std::string& Out, std::string& Err);
```

- **Loading.** Bodies take slots in file order (a `repeat` makes `name.0` .. `name.<n-1>`; a wall's
  bricks follow in course then brick order, omitted bricks never built). A wall is built by
  `geometry::BuildRunningBondWall` into a scratch structure and copied across, so its joints are the
  generator's; every other face-touching pair (`geometry::FaceContact`, 1e-9 m) gets one joint from
  the first matching `between` rule, else the first matching `materials` rule. One profile per joint
  row and per (fastener row, count, area); `Scenario::ProfileNames` names each by row. Mass is
  density × 8 hx hy hz (grounded 0); `ground` is the built-in material of grounded boxes. An `expect`
  body removed by a step that comes before any `cascade` step is a load error (it can never be
  released); one removed later is a run error when the step runs.
- **Running.** `Before` judges the structure as the before view draws it: every `remove` step's bodies removed, every `load` and `lateral_g` step applied, no cascade run. Each step runs in
  order on `Value.Model`; a cascade gets the accumulated loads and the current gravity (both
  `Standing` and `Certificate`) with `Previous = nullptr`, exactly as a direct `ICascade::Run` call
  would. Clusters concatenate, passes sum, the state is the last cascade's (or judge's) name;
  `Cracked` lists `<a>/<b>` for each crack (not sever, not crush) event; `Crushed` names every body a
  cascade crushed (2026-11-08). A step that cannot run (removing or
  loading a body already gone) sets `RunError` and stops the run. `ElapsedMs` times the steps.
- **Comparing.** Lines, in this order when present: `run error: ...`, `state: expected X, engine Y`,
  `released, not expected: a, b`, `expected to fall, still standing: a, b`, the partition line
  `expected {a} {b, c}; engine {...}` (sorted, `either` bodies left out), `passes: expected N,
  engine M`, `cracked, not expected: ...`, `expected to crack, not cracked: ...`, `crushed: expected
  [a], engine [b]`. The expectation's `crushed` (body patterns) is compared as a set and absent means
  nothing crushes, so a crush the file did not expect fails it. Met means no lines.
  Then (2026-11-08, core case A) the three keys below, each checked only when present:
  `utilisation before: ...` / `utilisation after: ...` (`no readings`; `max U at <joint> (<axis>),
  expected [lo, hi)`; `max U at <joint>, not at <patterns>`; `governed by <axis> at <joint>, expected
  <axis>`), `forced: expected none, engine forced {..}` or `forced: expected a forced release, engine
  forced none`, and `expected to stand, not standing: a, b`.
- **The `arch` builder** (2026-11-08, core case A): `{ "arch": "v", "material", "centre": [x, y, z],
  "radius", "thickness", "depth", "voussoirs": N, "springings": [left wall, right wall], "joint": row }`.
  A semicircle of N annular-sector voussoirs `v.0` (left, x = cx − R) to `v.N−1`, centreline radius
  `radius`, radial depth `thickness` (< 2 × radius), barrel width `depth` along y, centred on `centre`
  (the springing plane). Mass ρ b δ (Rₒ² − Rᵢ²) and the exact sector centroid, δ = π/2N. Radial joints of
  the `joint` row between neighbours (normal along the ring, U radial, V = −y, half-extents t/2 and b/2).
  Each end voussoir's lower face (x ∈ ±R ± t/2) gets one horizontal `joint`-row contact per top-course
  brick of its springing wall that it overlaps (the overlap rectangle, brick below). Errors: a springing
  that names no wall, a springing wall that comes after the arch, a wall top not at `centre.z`, an end
  voussoir resting on no brick, `voussoirs` < 1, a bad `thickness`, an unknown joint row, a name
  (or a generated `v.k`) already used. Voussoirs take part in no `joints` rule and no overlap check.
  `Scenario::BodyBoxes` holds a box inscribed at each sector centroid, for drawing only, and
  `BodyRotations` its quaternion about y by the bisector angle (identity for every other body).
- **Expectation keys added 2026-11-08:**
  `"utilisation": { "before" | "after": { "max": [lo, hi], "at": ["<a>/<b>" patterns], "axis": "Tension" | "Compression" | "Shear" } }`
  holds when the highest `Utilisation` over the state's readings (`ScenarioOutcome::BeforeReadings`,
  the before judge's; `AfterReadings`, the last cascade's or judge's) lies in [lo, hi) (a NaN fails),
  an `at` joint attains it within 1e-9 relative, and `axis` governs at one such joint (`at` and `axis`
  optional). `"forced": false` means no released cluster has `bForced` (`ScenarioOutcome::ForcedClusters`
  is empty), `true` that one has; absent checks nothing. `"stands": [patterns]` lists bodies that must
  be live at the end: in no released cluster, not crushed, not removed. Unknown keys stay errors.
- **Viewer export.** Keys in a fixed order: `slug`, `title`, `test` (when set), `summary`, `units`,
  `gravity`, `lateral`, `loads`, `bodies` (`id`, `name`, `c`, `h`, `q` as (w, x, y, z), `grounded`,
  `mass_kg`, `material`), `interfaces` (every interface ever added, with `state_after`),
  `removed_before_run`, `before.verdict`, `before.readings` (2026-11-08: one joint-shaped entry per
  reading of the before judge, as in `pass_records[].readings`; written only when not empty), `after` (`state`, `passes`, `clusters` with `motion`,
  `failures` (each with `kind`: `cracked`, `severed` or `crushed`), `crushed` (`body`, `pass`,
  `interface`, `ratio`; always written), `removed` (released and crushed bodies), `elapsed_ms` only
  when timed), and `expected` (with `crushed` body ids) only when it differs from the engine's
  after. The page draws a crushed body in place with a rubble tint and dashed edges and lists it in
  the facts, the failures table and the body inspect panel; it reads a file without `crushed` as
  crushing nothing. Numbers are shortest round-trip, so the export of a run is byte-identical run to
  run. `ViewerSceneOf` leaves `ElapsedMs` unset and sets `Expected` only when the file's expectation
  is not met.
- **CLI.** `loadpath_cli scenario <file> [--viz <out.json>] [--catalogue <path>] [--settings <path>] [--overrides <path>]`: the catalogue
  defaults to `catalogue.json` in the scenario folder's parent; the settings default to
  `data/settings.json` (absent means the compiled defaults, which the shipped file equals) and the
  overrides to `data/settings.local.json` (absent is fine). Prints the title, `built B bodies (G
  grounded), J joints`, one `step N <kind>: ...` line per step, then `expectation met` or
  `expectation not met` and the report lines. Exit 0 met, 1 not met (or a run error), 2 the file
  cannot be run (usage, unreadable file or catalogue, an explicit `--settings` file that cannot be read,
  `<file>:<line>: <message>` for a load error or a bad settings or overrides key).
  The one-argument structure command is unchanged.

## Settings (`Settings.h`, `data/settings.json`, 2026-11-08)

The engine's tunables live in a committed defaults file plus an optional overrides file holding
only the keys a user changed; reset discards the overrides ([decisions/2026-11-08-forced-collapse-and-core-suite.md](decisions/2026-11-08-forced-collapse-and-core-suite.md) §5,
[test_plans/2026-11-08-settings-and-pass-recorder.md](test_plans/2026-11-08-settings-and-pass-recorder.md) A).

```cpp
struct SolverSettings  { int MaxActiveSetIterations = 50; double PivotToleranceRelative = 1e-10; int DenseDofLimit = 600;
                         double StickMarginRelative = 1e-9, HeldForceToleranceRelative = 1e-12, DryNoiseFloorRelative = 1e-12,
                         NeutralLoadToleranceRelative = 1e-12, AdmissibilityToleranceRelative = 1e-9, UnconstrainedComponent = 1e-6,
                         DryRecloseFactor = 2.0, SparseAcceptanceMargin = 1.0, OpenTensionToleranceRelative = 1e-12,
                         CloseCompressionToleranceRelative = 1e-12; };
struct CascadeSettings { double CrackBandFraction = 0.5, CrackBandEdgeRelative = 1e-9, CrushEdgeRelative = 1e-9,
                         MotionAxisThresholdRelative = 1e-3; int MaxPasses = 32; UndecidedResolution OnUndecided = UndecidedResolution::Collapse; };
struct DebugSettings   { bool bRecordPasses = false; };
struct Settings        { SolverSettings Solver; CascadeSettings Cascade; DebugSettings Debug; };
struct SettingsResult  { Settings Value; FileError Error; bool Ok() const; };

SettingsResult  LoadSettings(std::string_view DefaultsJson);                                  // every key required, no unknown keys
SettingsResult  ApplyOverrides(const Settings& Defaults, std::string_view OverridesJson);      // any subset, nested the same way
StandingOptions ToStandingOptions(const Settings&, Vec3 Gravity);                             // Debug points into the Settings
CascadeOptions  ToCascadeOptions(const Settings&, Vec3 Gravity);
```

- The file's keys are the struct fields in snake_case under `solver`, `cascade` and `debug`;
  `Settings.DefaultsFileLoadsAndMatchesTheCompiledDefaults` pins the shipped file to the C++ defaults.
- Fail closed: a missing key (defaults file only), an unknown key, a wrong type (an integer field
  given a fraction included) or a non-finite number is an error whose message names the key by its
  path (`solver.dense_dof_limit`) and whose `Line` is the offending line; a refused document loads
  nothing (`Value` is the compiled defaults).
- The six solver tolerances that were `constexpr` in `RbsmStandingAuthority.cpp` are now
  `StandingOptions` fields with the same defaults, so a default-options solve is bit-identical to
  before; `StandingOptions::Debug` (`const DebugSettings*`, null means no debug work) is the one hook
  the pass recorder (package B) reads. `RunScenario` takes a `Settings` (overloads without one use
  the compiled defaults) and builds every step's options through `ToStandingOptions` and `ToCascadeOptions`.

## Pass recorder (`StandingAuthority.h`, `Cascade.h`, 2026-11-08, build-order item 1 package B)

When `Options.Debug != nullptr && Options.Debug->bRecordPasses`, the elastic solver fills `StandingComponent::Passes`
(one `ElasticPassRecord` per active-set iteration) and the cascade fills `CascadeVerdict::PassRecords` (one per pass,
including the final judge that changed nothing); otherwise both stay empty and recording changes no answer
(`Statics.Recorder.RecordingChangesNoAnswer` is bit-identity). `End` is filled whether or not recording is on.

```cpp
enum class CornerState : uint8_t { Closed, Open, Slipping };
enum class SolveEnd : uint8_t { None, Converged, Mechanism, RestoreFailedClosed, PowerCheckFailed, IterationCap, InvalidInput };
struct JointPassRecord {
	InterfaceId Id; std::array<CornerState, 4> Corners;   // after this iteration's update
	std::array<bool, 4> bFlipped;                          // differs from the previous iteration; all false on iteration 1
	std::array<double, 4> CornerNormalN; std::array<Vec3, 4> CornerShearN;
	double NormalForceN; Vec3 ShearForceN; Vec3 MomentNm; double Utilisation; GoverningAxis Axis;
};
struct ElasticPassRecord { int Iteration; std::vector<JointPassRecord> Joints; };
// StandingComponent: std::vector<ElasticPassRecord> Passes; SolveEnd End; std::vector<InterfaceId> Blocking (RestoreFailedClosed only)
// ComponentReport:   SolveEnd End; std::vector<ElasticPassRecord> Passes (carried from the elastic component)
struct CascadePassRecord { uint32_t Pass; std::vector<ComponentReport> Components; std::vector<InterfaceReading> Readings;
                           std::vector<ReleasedCluster> Clusters; std::vector<FailedInterface> Failures;
                           std::vector<CrushedBody> Crushed; };   // the crushes of this pass (2026-11-08)
// CascadeVerdict:    std::vector<CascadePassRecord> PassRecords;   // `Passes` stays the counted-pass integer
```

- `SolveEnd` is `None` for one Undecided: the force scale overflowed inside `UpdateActiveSet` (no value yet, owner call).
- Viewer JSON: `after.pass_records` is written when any verdict has records and omitted otherwise; `after.passes` stays
  the integer count. Each record is `{pass, components: [{bodies, verdict, end, iterations, inner: [{iteration, joints:
  [{interface, corners: ["closed"|"open"|"slipping" x4], flipped: [bool x4], corner_n: [x4], corner_shear: [[x,y,z] x4],
  n, shear, moment, utilisation, axis}]}]}], readings: [joint-shaped, one per live interface], clusters: [as after.clusters],
  failures: [as after.failures], crushed: [as after.crushed, this pass only]}`; `iterations` is `inner.size()`. Interface ids are the viewer's existing numbers.
- The E2E harness (`OptionsFor`) and the scenario-file harness record whenever `LOADPATH_VISUALS` is set and never
  otherwise; `RunScenario` follows `Settings.Debug.bRecordPasses`; the CLI honours `debug.record_passes` and
  `--record-passes` sets it true.

## Forced collapse (`Cascade.h`, `StructuralAuthority.h`, 2026-11-08, build-order item 3a)

The owner's §1 ruling ([decisions/2026-11-08-forced-collapse-and-core-suite.md](decisions/2026-11-08-forced-collapse-and-core-suite.md)),
built as a setting: `cascade.undecided_resolution` in `data/settings.json`, `"collapse"` (default) or `"stands"`; any other
value is an error naming `cascade.undecided_resolution` and its line.

```cpp
enum class UndecidedResolution : uint8_t { Collapse, Stands };
// CascadeOptions / CascadeSettings: UndecidedResolution OnUndecided = UndecidedResolution::Collapse;
// ComponentReport: int PassCount;                         // the elastic ActiveSetIterations
//                  bool Forced; SolveEnd ForcedReason;    // IterationCap, RestoreFailedClosed, PowerCheckFailed, or None (non-finite force scale)
//                  std::vector<InterfaceId> UnsettledJoints;
// ReleasedCluster: bool bForced;                          // a forced release; Motion is zero
// CascadeVerdict:  std::array<uint32_t, SolveEndCount> ForcedByReason;  // forced components solved this run, by SolveEnd
```

- The cascade's judge step resolves every component the elastic authority reads Undecided. Under `Collapse` it reads
  `Collapses` and its whole `Bodies` list is released (not the elastic `UnconstrainedBodies`); under `Stands` it reads
  `Stands` and the cascade carries on, readings as read. Both set `Forced` and `ForcedReason`; a Mechanism component reads
  `Collapses` with `Forced` false. The authority alone (`IStructuralAuthority::Judge`) still reports Undecided.
- `UnsettledJoints` (filled by the authority on any Undecided component): the `Blocking` list at `RestoreFailedClosed`;
  at `IterationCap` with the pass recorder on, the joints with a corner flipped in the last iteration; otherwise empty.
- A reused component is not counted twice in `ForcedByReason`. The re-judge after `PassCapReached` is not resolved.
- Viewer JSON: `after.components` (the last verdict's components: `bodies`, `verdict`, `end`) and every
  `pass_records[].components[]` carry `forced`, `forced_reason`, `pass_count`, `unsettled_joints` when `Forced` is true;
  a forced cluster carries `forced: true`. The page tints forced releases with the undecided colour and the components
  table gains a Forced column; absent fields change nothing.
