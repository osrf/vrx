# gz_waves

Wave simulation for VRX with two backends, each a gz-sim system plugin you
select by filename in the world SDF (there is no `<algorithm>` tag — the
plugin you load *is* the backend):

- **`gz-sim-waves-gerstner-system`** (`gz::sim::systems::GerstnerWaves`) —
  analytic Gerstner (Tessendorf 2001) sum-of-sines, vertex-shader
  displacement. Fast to load, lower visual fidelity.
- **`gz-sim-waves-fft-system`** (`gz::sim::systems::FftWaves`) — stochastic FFT
  (EncinoWaves spectra by default, in-tree Phillips + Tessendorf choppy
  displacement otherwise), CPU IFFT each tick, GPU heightmap sampled in a
  custom vertex shader. Higher visual fidelity.

Both backends share the same Apache-2 codebase intended for upstreaming to
`gz-sim`. All Ogre Next access goes through a small dlopen'd bridge
(`libwaves-ogre2-bridge.so`) so the system plugin itself has no DT_NEEDED on
`libgz-rendering-ogre2.so` — that DT_NEEDED would interfere with
gz-rendering's engine-plugin loader.

## Known limitation: slow first-frame load with the FFT visual

On at least Ubuntu 24.04 + NVIDIA Blackwell + locally-built Gazebo Jetty +
system OgreNext 2.3.1, the FFT visual takes **~2 minutes** between
`gz sim` launch and the first frame of waves rendering. The Gerstner
visual on the same setup loads in ~5 seconds.

Empirically traced to the first-render-of-a-`HlmsLowLevel`-material-with-
custom-GLSL path inside OgreNext. The same hardware running asv_wave_sim
against Gazebo Harmonic renders waves in seconds; that project's
architecture (custom `RenderEngineExtension` + `Ogre2OceanVisual` +
`Ogre2DynamicMesh` with procedurally-built v1 vertex buffers and
`SubItem::setDatablock`) bypasses the slow code path. Reproducing that
architecture in VRX is tracked as a multi-week follow-up task.

Bypasses tried that did NOT fix it (kept for reference):
- `setNamedConstant("heightMap", &texIndex, 1, 1)` to bind the GLSL sampler
  index explicitly — applied as a keeper since it's the correct OpenGL
  pattern, but did not fix the stall.
- `SaveToSystemRam` + `notifyDataIsReady` after each upload + full mipmap
  chain (`getMaxMipmapCount`) + per-upload `scheduleTransitionTo(Resident)` —
  matches asv_wave_sim's streaming-texture contract; applied as keepers,
  but did not fix the stall on this hardware.
- Persistent staging texture (vs. per-frame `getStagingTexture` /
  `removeStagingTexture`) — fixed a real pool-churn issue and is a
  keeper, but did not fix the stall.
- Bumped `setStagingTextureMaxBudgetBytes(256 MB)` — kept as a keeper but
  did not fix the stall.
- Deduping the gz-sim double-loaded `WaterVisual` instance (the bug
  acknowledged in asv_wave_sim#177) — kept as a keeper but each
  individual instance still triggers the stall on its own.
- Skipping `gz::rendering::Visual::SetMaterial` and binding the material
  to a procedurally-built `Ogre::Item` via `SubItem::setDatablockOrMaterialName`
  — the procedural path was diagnostic; it did not fix the stall and the
  resulting render was empty, so the diagnostic was reverted.

Upstream references documenting the same class of issue:
- [`asv_wave_sim#152`](https://github.com/srmainwaring/asv_wave_sim/issues/152)
  — "Performance issues using FFT waves"
- [`asv_wave_sim#177`](https://github.com/srmainwaring/asv_wave_sim/issues/177)
  — gz-sim double-load of visual plugins
- [`asv_wave_sim#182`](https://github.com/srmainwaring/asv_wave_sim/issues/182)
  — maintainer's note on the architectural fix being a GPU FFT visual

Until the architectural port lands, users wanting fast load should use the
`gz-sim-waves-gerstner-system` plugin; the FFT system is worth the
first-launch wait when the visual quality justifies it.
