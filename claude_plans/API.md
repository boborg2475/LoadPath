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

`LoadStructure(std::string_view json) -> Result<Structure>` with a `FileError` carrying the
line and a message for the first problem. Names resolve to handles; a duplicate or unknown name
is an error; missing optional fields take the struct defaults; every `Add*` refusal is
surfaced with the element's name. `WriteStructure(const Structure&) -> std::string` round-trips.
The CLI `loadpath_cli <file>` loads and prints profile, body and interface counts and the first
error if any; exit code 0 on success, 1 on a load error, 2 on usage.
