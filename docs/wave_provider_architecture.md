# Wave Provider Architecture

How the VRX wave system is structured: one wave-field interface ("the socket"),
interchangeable engines behind it, and a recipe-style ECM component that keeps
the physics **server** and the render **GUI** in sync. This describes the system
as built. For the user-facing SDF knobs see
[`wave_design_reference.md`](wave_design_reference.md); for the broader
upstreaming roadmap (wind, currents, hydrodynamics) see
[`waves_integration_plan.md`](waves_integration_plan.md).

## The big idea (plain English)

There is a standard **"wave socket"** — a fixed set of questions anyone can ask
about the water. Different wave implementations **plug into that socket**.
Everything that needs waves (buoyancy, the renderer, sensors) talks to the
*socket*, never to a specific implementation — so the wave engine can be swapped
underneath and nothing else changes. Same idea as printer drivers: apps press
"Print"; the app never knows the brand.

```
            ┌──────────── the "wave socket" — IWaveField ────────────┐
   ask it:  "water height / normal / particle velocity / fold at (x,y,t)?"
            │                  "give me the height grid"               │
            └──────────────────────────────────────────────────────────┘
                    ▲                      ▲                       ▲
       plugs in     │                      │                       │
   ┌────────────────┴───┐    ┌─────────────┴──────┐    ┌───────────┴────────┐
   │ Gerstner engine    │    │ FFT / Encino engine │    │ (future engine)    │
   │ analytic, light    │    │ spectral, rich      │    │                    │
   └────────────────────┘    └─────────────────────┘    └────────────────────┘

   who asks the socket:   buoyancy  ·  the water renderer  ·  sensors / scoring
   (none of them know or care which engine is plugged in)
```

Because every consumer works from the same little **recipe** (seed included), the
separate physics and graphics processes always agree on the waves.

## The pieces

### 1. The interface — `IWaveField` (the socket)

The contract every engine implements; consumers only see this
(`gz_waves/include/gz/sim/waves/WaveSimulation.hh`):

```cpp
class IWaveField
{
public:
  virtual ~IWaveField() = default;

  // --- point queries (physics / logic / sensors) ---
  virtual double          Elevation(double x, double y, double t) const = 0;
  virtual Eigen::Vector3d ParticleVelocity(double x, double y, double t) const = 0;
  virtual Eigen::Vector3d Normal(double x, double y, double t) const = 0;
  virtual double          Jacobian(double x, double y, double t) const = 0;

  // --- configuration / lifecycle ---
  virtual void SetParameters(const WaveParameters &) = 0;
  virtual void Update(double simTime) {}                  // no-op for analytic engines

  // --- identity / rendering ---
  virtual std::string_view        Kind() const = 0;       // "gerstner" | "fft"
  virtual std::optional<TileSize> Bounds() const { return std::nullopt; }
  virtual const WaveField2D      *Field() const { return nullptr; }
};
```

- **Vectors are `Eigen`, not `gz::math`.** The engine layer (this interface plus
  both engines) carries **no Gazebo dependency** — only Eigen — so it can be
  reused outside a simulator (see the extraction note in
  [`waves_integration_plan.md`](waves_integration_plan.md)).
- **No `Capabilities` flags.** Optional features degrade via defaults: analytic
  engines no-op `Update` and return `nullopt` from `Bounds`; foam is the
  `Field()` grid's foam channel; the renderer finite-diffs the normal when an
  engine exposes no slope.
- **One shared `WaveParameters` recipe.** Both engines read the same struct
  (model / period / grid_size / seed / `sea_state` / `<spectrum>`/`<spreading>`/
  `<dispersion>` / the band-pass filter / …). Gravity is **not** an SDF knob —
  it is read from the world's `<gravity>` so the waves stay consistent with
  buoyancy and rigid-body dynamics.

### 2. The engines + the registry — "the plug-in engines"

Each engine (Gerstner = analytic sum-of-Gerstners; FFT = Encino spectral ocean)
is a **plain `IWaveField` library** with no `GZ_ADD_PLUGIN`. Engines are made
reachable by a token through an **in-process registry** — no dlopen engine
loader:

```cpp
RegisterWaveEngineFactory("gerstner", MakeGerstnerWaveField);   // engine self-registers
auto field = CreateWaveSimulation("fft", params);               // build + SetParameters
```

On the **server**, a per-engine gz-sim **system plugin**
(`gz-sim-waves-fft-system`, `gz-sim-waves-gerstner-system`) — a thin
`WavesSystemBase` subclass that overrides `EngineToken()` + `MakeEngine()` —
links its engine and registers it. The world selects the engine simply by the
plugin **filename** (no `<algorithm>` tag). On the **GUI**, the water visual
links and registers both engines and builds its own from the component.

### 3. The component — "the recipe card on the world"

A small note attached to the world entity: which engine + its settings. It is
the **recipe, not the water**:

```cpp
struct WavefieldData
{
  std::string                  algorithm;   // "gerstner" | "fft"
  WaveParameters               params;      // the full recipe (incl. world gravity)
  std::shared_ptr<IWaveField>  simulation;  // server-side live engine — NOT serialized
  std::uint64_t                generation;  // bumped on any change; consumers re-read
};
```

The stream operators serialize **only the recipe** (`algorithm` + `params` +
`generation`); `simulation` is reset to null on deserialize. Each process
rebuilds its own engine from the recipe via `CreateWaveSimulation`, so the server
and GUI agree without streaming any height grids — the component is tiny.

### 4. The consumers — "the things that use water"

Buoyancy, the renderer, sensors, scoring. Each asks the socket through the
`Eval::*` free-function helpers over `WavefieldData` (`WaveBuoyancy` uses
`SurfaceElevation`) or the `Field()` grid (`WaterVisual`). Where the engine
*comes from* differs by process (see Determinism): a **GUI** consumer builds and
owns a private engine; a **server** consumer reads the source system's
authoritative engine out of the component. Because they only speak "socket,"
they work with any engine unchanged.

## Determinism — one engine per process, advanced by its owner

Gazebo runs as two programs — the physics **server** and the graphics **GUI** —
and both need the waves. Instead of streaming a grid between them, **each process
builds its engine from the recipe**; because the recipe is identical (*including
the seed*), the two processes produce the same waves independently. The server's
buoy and the GUI's rendered crest line up for free.

This makes determinism a hard contract on engines:

> Given the same `params` (seed included) and `simTime`, `Update()` + the queries
> must produce the same field in every process.

Within a process there is exactly **one** engine, owned and advanced by one
system; a consumer must never assume some *other* process advanced it:

- **GUI:** there is no server engine to borrow, so `WaterVisual` builds and owns
  a private engine and advances it on the render clock. (An earlier design shared
  a live engine *pointer* through the component; after a replication round-trip
  the GUI could hold a fresh, never-advanced copy and see flat water, or race the
  server's engine on the render thread — hence: own it.)
- **Server:** the source system (`WavesSystemBase`) owns the authoritative engine
  and advances it each tick; same-thread consumers (`WaveBuoyancy`) read it out of
  the component. The component serializes recipe-only, so every ECM deserialize
  (a **reset** restore, a replication round-trip) nulls that live pointer — the
  source system re-points the component at its engine whenever the ECM has cleared
  it, otherwise buoyancy silently reverts to flat water after a reset.

## Runtime parameters

The recipe is a small note on the world, so a control surface can edit it live.
The source system advertises:

```
/world/<world>/wave/set_parameters     (gz.msgs.Param → gz.msgs.Boolean)
```

A caller sends a key→value map of `<wave>` tag names (a **partial** update —
omitted keys keep their value). The change is queued on the transport thread and
applied on the ECM thread: it re-runs `SetParameters`, bumps `generation`, and
re-marks the component changed, so every consumer re-reads with no extra wiring.

## Where it lives (packages)

| Package | Contains | Gazebo dep? |
|---|---|---|
| `gz_waves` | `IWaveField` + registry + `WaveParameters` + `Eval` (gz-free); the `Wavefield` ECM component + `WavesSystemBase` + `WaveBuoyancy` (gz-sim) | mixed |
| `gz_waves_provider_gerstner` | Gerstner engine (gz-free) + its system plugin | mixed |
| `gz_waves_provider_fft` | FFT/Encino engine (gz-free) + its system plugin | mixed |
| `gz_waves_rendering` | `WaterVisual` + the Ogre2 heightmap C-ABI bridge | gz-sim/Ogre |
| `encinowaves_vendor` | vendored Horvath spectrum library | none (Eigen/TBB/Imath) |

The **engine layer** (the interface, the registry, and both engines) depends only
on Eigen — no gz-sim, no gz-math — so it is extractable as a standalone,
simulator-agnostic library. The gz-sim integration (the component, the system
plugins, buoyancy, rendering) is the only part that needs Gazebo.
