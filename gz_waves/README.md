# gz_waves

The core of VRX's wave simulation: the engine-agnostic wave-field **contract**
shared by the wave engines and their consumers. It builds standalone and depends
on no particular wave backend.

It provides:

- **`IWaveField`** — the abstract wave-field interface (surface elevation,
  normal, particle velocity, Jacobian, …) that every wave engine implements.
- **An engine registry** (`RegisterWaveEngineFactory` / `CreateWaveSimulation`)
  — a process-wide token→factory map, so an engine can be reached by name
  without the core linking it.
- **`WaveParameters`** and the **`Wavefield` ECM component** (+ serialization) —
  the shared recipe and the world-entity channel that carries it (and the live
  engine) to every consumer.
- **`Eval`** — the null-safe free-function query API consumers use; the engine
  interface stays opaque behind it.
- **`WavesSystemBase`** — the base class each per-engine *source* system plugin
  builds on.

Wave **engines** — each a gz-sim system plugin you select by filename in the
world SDF (there is no `<algorithm>` tag; the plugin you load *is* the backend) —
and **consumers** (buoyancy, the water visual, any wave-aware plugin) live in
their own packages that depend on this one. It all shares a single Apache-2
codebase intended for upstreaming to `gz-sim`.
