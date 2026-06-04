# Wave Provider Architecture (design)

**Status:** target design, not yet implemented. The plan is to **build it in VRX
first** behind these boundaries, get it right by testing, then lift the
Gazebo-bound pieces upstream. This is the companion to
[`wave_design_reference.md`](wave_design_reference.md), which documents the
*current* (pre-redesign) implementation.

---

## The big idea (plain English)

Gazebo defines a standard **"wave socket"** — a fixed set of questions anyone can
ask about the water. Different wave implementations **plug into that socket**.
Everything that needs waves (buoyancy, the renderer, sensors) talks to the
*socket*, never to a specific implementation — so you can swap the wave engine
underneath and nothing else changes.

Same idea as **printer drivers**: apps just press "Print"; you install whatever
printer's driver you own; the apps never know the brand.

```
            ┌──────────── the "wave socket" (the interface) ────────────┐
   ask it:  "water height / slope / velocity / foam at (x,y) right now?"
            └────────────────────────────────────────────────────────────┘
                    ▲                      ▲                       ▲
       plugs in     │                      │                       │
   ┌────────────────┴───┐    ┌─────────────┴──────┐    ┌───────────┴────────┐
   │ Gerstner provider  │    │ FFT/Encino provider │    │ (future provider)  │
   │ simple, w/ Gazebo  │    │ rich, ships w/ VRX  │    │                    │
   └────────────────────┘    └─────────────────────┘    └────────────────────┘

   who asks the socket:   buoyancy  ·  the water renderer  ·  sensors / scoring
   (none of them know or care which provider is plugged in)
```

In one line: *the water is asked questions through a standard socket;
interchangeable engines answer; buoyancy and rendering just ask and don't care
who answered; and because everyone works from the same little recipe (seed
included), the separate physics and graphics programs always agree.*

---

## The pieces

### 1. The interface — "the socket / the contract"

A short list of questions every wave implementation must answer, plus "advance
yourself to time T." Strawman:

```cpp
namespace gz::sim::waves
{
  /// What a provider can answer, so consumers degrade gracefully.
  struct Capabilities
  {
    bool queryable = true;   ///< CPU point queries (buoyancy needs this)
    bool grid      = false;  ///< produces a WaveGrid for the renderer
    bool velocity  = false;  ///< real particle velocity (else returns 0)
    bool foam      = false;  ///< real foam (else returns 0)
  };

  /// The "socket". Every wave engine implements this; consumers only see this.
  class IWaveField
  {
  public:
    virtual ~IWaveField() = default;

    // --- lifecycle ---
    /// (Re)configure from the recipe: the two universal knobs — the RNG `seed`
    /// and the target significant wave height `targetHs` (the sea state, in
    /// metres) — plus the provider's OWN `config` (its serialized SDF sub-tree).
    /// The provider realizes `targetHs` in its own terms, parses `config` itself
    /// (the core never interprets it), and decides whether to mutate or rebuild.
    /// Beyond the two universal knobs there is deliberately NO shared parameter
    /// struct; see "Parameters: provider-owned" and "Common config: sea state".
    virtual void SetParameters(std::uint32_t seed, double targetHs,
                               const std::string &config) = 0;
    /// Advance state to sim time t. CONTRACT: deterministic — same seed+targetHs
    /// +config+t must yield the same field in every process (see determinism).
    virtual void Update(double simTime) = 0;
    virtual Capabilities Caps() const = 0;

    // --- point queries (physics / logic / sensors) ---
    virtual double         Elevation(double x, double y) const = 0;
    virtual math::Vector3d Normal(double x, double y) const = 0;
    virtual math::Vector3d Velocity(double x, double y) const = 0;  // 0 if !Caps().velocity
    virtual double         Foam(double x, double y) const = 0;      // 0..1; 0 if !Caps().foam

    // --- grid (rendering) ---
    /// Null when !Caps().grid. Provider owns the memory; valid until next Update.
    virtual const WaveGrid *Grid() const = 0;
  };
}
```

### 2. The providers — "the plug-in engines"

Actual wave implementations, each in its **own package**, loaded **by name** with
`gz-plugin` (the same mechanism Gazebo already uses for systems/sensors):

```cpp
// in a provider package, e.g. gz_waves_gerstner:
class GerstnerProvider : public gz::sim::waves::IWaveField { /* ... */ };
GZ_ADD_PLUGIN(GerstnerProvider, gz::sim::waves::IWaveField)
```

- a **simple** provider (Gerstner — a few sine waves) ships **with Gazebo**, so a
  plain install has working water out of the box;
- a **rich** provider (FFT + Encino spectral ocean) ships **with VRX**.

More can be added later; they only have to fit the socket.

### 3. The component — "the recipe card on the world"

A small note attached to the world entity: which provider + its settings. It is
the **recipe**, *not* the water. Critically, it carries **no live simulation
object** — only data:

```cpp
struct WavefieldData
{
  std::string   provider;   ///< "gerstner" | "fft" | "vrx_encino" | ...
  std::uint32_t seed;       ///< universal: determinism (identical fields per process)
  double        targetHs;   ///< universal: significant wave height [m] (the sea
                            ///< state); the provider realizes it (see Sea state)
  std::string   config;     ///< the provider's SDF sub-tree, serialized; parsed
                            ///< by the provider itself — opaque to the core
  std::uint64_t revision;   ///< bumped on any change; consumers re-apply
};
using Wavefield = components::Component<WavefieldData, class WavefieldTag>;
```

This is replicated server→GUI like any ECM component (it's tiny — a couple of
strings and two integers).

#### Parameters: provider-owned, not a shared struct

There is deliberately **no shared `WaveParameters` struct** in the API. A single
struct holding every provider's knobs would be a **fat union** — most fields are
meaningless to any one provider (a grid size means nothing to an analytic
Gerstner sea) — and, living in the upstream core, **adding a knob for a
downstream provider** (say an Encino `fetch`) would force an **upstream change**.
That's exactly why our current Encino knobs are bolted on as `GZ_WAVES_ENCINO_*`
**environment variables**: `WaveParameters` had nowhere to put them.

Instead, **each provider owns its parameter surface** — its own SDF sub-tree,
parsed by the provider itself. The recipe carries that as a **serialized string**
(`config`) so it travels through the ECM and **both processes parse the identical
text**, preserving determinism. The core never looks inside it. There are only
**two** truly universal knobs — the **`seed`** (determinism) and a **sea-state
target** (how rough the sea is; next section). Almost nothing else is universal
(not every sea even has a single direction or period), so only those two are
promoted out of the opaque config.

Concretely, the Encino env vars become the FFT/Encino provider's own SDF, e.g.:

```xml
<provider>fft</provider>
<spectrum>jonswap</spectrum>   <depth>8</depth>   <swell>0.7</swell>
```

— parsed by `vrx_waves`, invisible to `gz_waves`.

#### Common config: sea state

The second universal knob is a **sea state** — how rough the sea is — because
every provider, whatever its math, can produce a sea of a given roughness. It's
the friendly front door: a user or a VRX task says "sea state 4" without learning
any provider's spectrum/fetch/dispersion knobs.

The input is the standard **WMO sea-state code** (0–9); the core resolves it to a
**significant wave height** `targetHs` [m] — the universal physical quantity
stored in the recipe and passed to providers:

| N | name | Hs |
|---|---|---|
| 0–1 | calm | 0–0.1 m |
| 2 | smooth | 0.1–0.5 m |
| 3 | slight | 0.5–1.25 m |
| 4 | moderate | 1.25–2.5 m |
| 5 | rough | 2.5–4 m |
| 6 | very rough | 4–6 m |
| 7–9 | high → phenomenal | 6 m → 14 m+ |

So the SDF accepts either the friendly integer or the precise height:

```xml
<sea_state>4</sea_state>                       <!-- resolves to targetHs ≈ 1.9 m -->
<!-- or, for precision the 0–9 buckets can't express: -->
<significant_wave_height>1.7</significant_wave_height>
```

**The core owns the table** (a WMO standard, one source of truth) and hands
`targetHs` to the provider via `SetParameters`. Each provider realizes it in its
own terms — Gerstner picks amplitude/period; FFT/Encino derives the wind and lets
its **amplitude calibration** (which already targets `Hs = 0.21·V²/g`) land on
it. So sea state simply becomes the target the calibration aims at.

This makes the recipe **two-tier**:

```
recipe = {
  provider,                  // who
  seed,                      // universal: determinism
  targetHs,                  // universal: sea state (how rough)
  config (opaque SDF),       // provider-specific: the power-user knobs
  revision
}
```

- **Beginner / VRX task:** `provider=fft`, `sea_state=3` — done.
- **Power user:** also supplies `config` to fine-tune spectrum/fetch/dispersion;
  `targetHs` sets the baseline energy, the SDF bends its character.

**Caveat:** sea-state → Hs is standardized for a *fully-developed, deep-water*
sea. If the provider config overrides `depth`/`fetch`, the realized Hs can drift
from the table — so `sea_state` is a baseline target the config refines, not a
hard guarantee. (Especially nice for VRX: tasks are often specified as sea
conditions, and this knob behaves the same across providers.)

### 4. The consumers — "the things that use water"

Buoyancy, hydrodynamics, the renderer, sensors, scoring. Each **holds its own
provider** (built from the recipe) and asks it questions through the socket.
Because they only speak "socket," they work with *any* provider unchanged.

---

## How buoyancy interacts (concrete)

Buoyancy is just a consumer. Each physics step it advances its provider and asks,
per hull point:

```cpp
field->Update(simTime);                 // advance the instance I hold
double h = field->Elevation(px, py);    // water height under this point
// ... compare to the point's Z, apply buoyant force where submerged ...
```

It never knows whether the answer came from Gerstner or Encino — only "the water
is *this* high here." Swap the engine and the buoy behaves the same, riding
different-looking waves. (It can also use `Velocity()`/`Foam()` when the
provider's `Caps()` advertise them — e.g. wave-induced drift, or foam-aware
drag.)

---

## The two-process model & the determinism contract

Gazebo runs as **two programs**: the physics **server** and the graphics **GUI**.
Both need the waves. Instead of streaming a grid of water heights between them,
**each program reads the recipe and builds its own provider**. Because the recipe
is identical — *including the seed* — they produce the **same waves**
independently. The server's buoy and the GUI's rendered crest line up for free,
with almost no network traffic.

This makes determinism a **hard contract on providers**:

> Given the same `seed`, `targetHs`, `config`, and `simTime`, `Update()` + the
> queries must produce the same field in every process.

It also bans the anti-pattern we hit during development: **a consumer must
advance the provider instance it holds — never assume another system did.** (The
old design shared a live simulation *pointer* through the component; after a
replication round-trip a consumer could end up holding a fresh, never-advanced
copy and see flat water. This design removes the shared pointer entirely.)

---

## Runtime parameters

Because the recipe is a small note on the world, a control surface can edit it
live:

```
service/topic  /world/<w>/wave/set_parameters
      │
      ▼  write the new config into the Wavefield component, bump `revision`
      │     (SceneBroadcaster replicates it as usual)
      ▼
 each process's wave system sees the new revision → field->SetParameters(seed, targetHs, config)
      │     (provider decides: cheap mutate, or internal rebuild)
      ▼
 next Update() reflects the change; server + GUI stay in lockstep via the seed
```

The provider should apply changes by **atomic swap** so a query never sees a
half-updated field.

---

## Where it lives (package split)

| Package | Contains | Future home |
|---|---|---|
| `gz_waves` (core) | `IWaveField`, `WaveGrid`, `Capabilities`, the `Wavefield` component (recipe: provider + seed + targetHs + config blob), query helpers, the `Waves` system, provider **discovery via gz-plugin**, consumer systems (`WaveBuoyancy`, hydro), **+ the simple Gerstner provider** | → gz-sim upstream |
| `gz_waves_rendering` | `WaterVisual` + the engine-specific heightmap→GPU bridge; consumes a `WaveGrid`, provider-agnostic | → gz-sim/rendering upstream |
| `vrx_waves` | the rich **FFT/Encino** provider (a gz-plugin) | stays in VRX |
| `encino_waves` | vendored Horvath library | dependency of `vrx_waves` only |

**Discipline that makes upstreaming a lift-and-shift:** the core package must
compile with nothing gz-sim doesn't already have — **no Encino, no Ogre, no
VRX-isms** leak into it.

---

## Mapping to today's code

The current code is ~80% of this already; the redesign is mostly repackaging +
one architectural change (plugin discovery), not new algorithms:

| Today | Becomes |
|---|---|
| `IWaveSimulation` (`WaveSimulation.hh`) | `IWaveField` (+ `SetParameters`, `Caps`, `Grid`, `Foam`) |
| `CreateWaveSimulation()` if/else factory | **gz-plugin** discovery (load provider by name) |
| `WavefieldData` carrying a live `shared_ptr<IWaveSimulation>` | recipe-only `WavefieldData` (no shared sim) + `revision` |
| `WaveParameters` (fat union of all backends' fields) | dropped — each provider parses its own SDF `config`; only `seed` is promoted to a universal field |
| Encino `GZ_WAVES_ENCINO_*` env vars | the FFT/Encino provider's own SDF tags (in `config`) |
| `GerstnerWaveSimulation` | the simple **Gerstner provider** (already self-contained) |
| `FFTWaveSimulation` + Encino + Ogre2 bridge | the **VRX FFT/Encino provider** + the rendering bridge |
| `Eval::*` free functions | thin helpers over `IWaveField` |
| `WaveBuoyancy` (already provider-agnostic) | upstream consumer system |

---

## Migration plan (staging)

Build in VRX, behind the boundaries above, then upstream the stable core. Each
PR keeps a working system:

1. **Carve `gz_waves`** — move interface/component/`Eval`/`Waves`/`WaveBuoyancy`;
   generalise `IWaveSimulation → IWaveField`.
2. **Introduce the gz-plugin seam** — wrap Gerstner and FFT as provider plugins;
   replace the hardcoded factory with name-based loading. (Everything still
   works, just loaded differently.)
3. **Split `gz_waves_rendering`** out of the monolith.
4. **Move FFT/Encino into `vrx_waves`** as the rich provider.
5. **Wire runtime params** (service + `revision` + `SetParameters`).
6. **Add a conformance test suite** every provider must pass (point-query
   sanity, determinism, grid bounds) — pays off once there are ≥2 providers.

**First upstream PR (Path B):** the **Gerstner-only** slice on this architecture
— the simple provider + the core + the rendering, with FFT and Encino landing as
clean *additive* provider PRs afterward (no refactor of the merged core).

---

## Open decisions

- **Replicate recipe, not pixels** (decided): the component carries the recipe
  (provider + seed + opaque config blob), never a height grid; determinism via
  shared seed + identical config keeps processes in sync cheaply. A
  future GPU-only provider that can't answer CPU point queries would advertise
  `Caps().queryable = false` and need a readback path for physics.
- **Rendering boundary is engine-specific:** standardise the CPU `WaveGrid`
  upstream; keep the GPU upload (Ogre2) behind the existing dlopen isolation so
  Ogre never leaks into the interface.
- **Capability matrix vs simplicity:** keep `Caps()` to a few flags with
  documented fallbacks (finite-diff normals, zero velocity/foam).
- **Runtime-update concurrency:** `SetParameters` builds new state and swaps
  atomically so the render thread never reads a half-updated field.
