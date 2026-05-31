# Headless rendering benchmark: EGL vs. Xvfb (GLX) in CI

**Date:** 2026-05-29
**Branch:** `jrivero/vrx4_ci`
**Component:** `tools/smoke_test.sh`, `vrx_bringup/launch/sim_smoke.launch.xml`, `.github/workflows/ci.yml`
**Question:** The vrx4 smoke test renders the observation camera through two headless
backends. EGL headless was consistently ~6–8× slower than Xvfb/GLX. Why?

---

## TL;DR

- Both backends rasterize with the **identical** software driver: Mesa **llvmpipe**
  (LLVM 21.1.8, Mesa 26.0.3, OpenGL 4.5). Confirmed in the real CI container via
  `glxinfo -B` / `eglinfo -B`. The slowdown is **not** a different/slower rasterizer.
- The gap is **not** explained by launch confounds (`-v4` logging, composable vs.
  standalone) — it persists in a fully controlled comparison on CI.
- The gap is **not** llvmpipe thread contention — pinning `LP_NUM_THREADS=1`
  made **both** backends *slower* (egl collapsed to 1 frame), refuting the
  "free a core for readback" hypothesis.
- **Leading explanation (localized, not yet profiled):** the EGL software
  offscreen surface pays a costlier, less-pipelined **per-frame GPU→CPU
  readback/resolve** than the GLX drawable. This cost **scales with physical core
  count**, which is why it is far worse on the core-limited CI runner than on a
  many-core workstation.

---

## 1. Setup

### 1.1 What the smoke test measures

`tools/smoke_test.sh <xvfb|egl>` launches the simulation headless against
`open_water.sdf`, then over a 10 s window confirms:

1. `/clock` is advancing (sim is stepping), and
2. the observation camera is producing frames on `/observation_camera/image`,

reporting the achieved **camera FPS**. It requires `MIN_FRAMES=5` (sustained
rendering, not a single frame), so a backend that renders once and stalls fails.

### 1.2 The two backends

| Backend | Mechanism |
|---------|-----------|
| `xvfb` | Mesa software GL (llvmpipe) under a virtual X server (Xvfb), **GLX** path. `LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe`. |
| `egl`  | **EGL** headless rendering, no X server (`gz sim --headless-rendering`). gz selects an EGL device via `eglQueryDevicesEXT`; on a GPU-less host this resolves to Mesa's software device. |

### 1.3 Environment

- **Container:** `ros:lyrical-ros-base` (Ubuntu 24.04 "Noble"), ROS 2 Lyrical,
  Gazebo Jetty (`ros-lyrical-ros-gz-*` from `ros2-testing`).
- **CI runner:** GitHub-hosted `ubuntu-latest`. `osrf/vrx` is a **public** repo,
  so the larger free tier: **4 vCPU / 16 GB RAM / 14 GB SSD**, Ubuntu 24.04,
  ephemeral (one fresh VM per job). The 4 vCPUs are SMT threads on **~2 physical
  cores** — relevant because llvmpipe is FP/SIMD-saturating, so hyperthreads add
  little and the machine behaves close to 2 effective cores for this workload.
  *(Spec is GitHub's documented tier; not directly measured from our logs.)*
- **No GPU:** rendering is pure CPU software rasterization in both paths.

---

## 2. Renderer identity (the key control)

Queried in the actual CI container with no `/dev/dri` (GPU-less, exactly like the
runner):

```
# GLX (Xvfb)
OpenGL renderer string: llvmpipe (LLVM 21.1.8, 256 bits)
OpenGL core profile version string: 4.5 (Core Profile) Mesa 26.0.3-1ubuntu1

# EGL device platform (what gz picks via eglQueryDevicesEXT)
Device #0:  EGL_EXT_device_drm_render_node, EGL_MESA_device_software
OpenGL core profile renderer: llvmpipe (LLVM 21.1.8, 256 bits)
```

**Conclusion:** both backends bind the same llvmpipe. "swrast" (`swrast_dri.so` /
`kms_swrast_dri.so`) is only the DRI *loader* that hosts a Gallium pipe driver; the
pipe driver it instantiates is llvmpipe (softpipe only if LLVM were absent). There
is no separate "EGL software rasterizer" — EGL can and does use llvmpipe.

To make this verifiable on every run, the smoke test now prints the bound renderer
(`glxinfo -B` / `eglinfo -B`) and the effective `LP_NUM_THREADS` before launching.

---

## 3. Measurements

> ⚠️ **Absolute FPS is noisy.** Each matrix cell runs on a *separate* ephemeral VM,
> and the shared runner pool load varies. Same code has swung ~2× run-to-run.
> Trust the **direction and ratios**, not the absolute values. EGL's low frame
> counts (single digits) make its absolute numbers especially noisy.

### 3.1 Original (confounded) comparison

Before controlling the launch: `xvfb` used the composable production launch at
default verbosity; `egl` used a standalone launch at `-v4`.

First green run (`387f3e7d`, run `26601399247`):

| Backend | FPS | Frames/10 s |
|---------|----:|------------:|
| xvfb | 6.77 | 68 |
| egl  | 0.80 | 8 |

Five repeat dispatches (same confounded setup):

| Run | xvfb FPS | egl FPS |
|----:|---------:|--------:|
| 1 | 4.80 | 1.19 |
| 2 | 4.80 | 0.50 |
| 3 | 5.79 | 0.90 |
| 4 | 4.88 | 0.99 |
| 5 | 6.07 | 0.70 |
| **mean** | **5.27** | **0.86** |

Gap ≈ **6×**, ranking never crossed.

### 3.2 Controlled comparison

Unified `sim_smoke.launch.xml`: both backends use the same standalone `gz sim`
(via `ros_gz_sim`'s `gz_sim.launch.py`), `--render-engine-server ogre2`, `-v1`,
standalone bridge. **The only difference is the `--headless-rendering` flag.**

CI (`83487503`, run `26634188291`):

| Backend | Renderer | FPS | Frames/10 s |
|---------|----------|----:|------------:|
| xvfb | llvmpipe | 5.60 | 56 |
| egl  | llvmpipe | 0.70 | 7 |

Local container (many physical cores), same controlled launch:

| Backend | FPS |
|---------|----:|
| xvfb | 7.20 |
| egl  | 2.90 |

**Observation:** the gap *persisted* on CI (~8×) even after removing the confounds.
On the many-core local box the gap was only ~2.5×. xvfb barely changed across
machines (~1.3×); **egl dropped ~4× (2.90 → 0.70)** — i.e. egl scales much worse
with fewer physical cores.

### 3.3 `LP_NUM_THREADS` experiment

4-cell matrix (`8809ca1d`, run `26635828676`), both backends × `{auto, 1}`:

| Backend | LP_NUM_THREADS | Renderer | FPS | Frames/10 s | Result |
|---------|:--------------:|----------|----:|------------:|--------|
| xvfb | auto | llvmpipe | 2.59 | 26 | ✅ |
| xvfb | 1    | llvmpipe | 1.80 | 18 | ✅ |
| egl  | auto | llvmpipe | 0.80 | 8  | ✅ |
| egl  | 1    | llvmpipe | 0.10 | 1  | ❌ (< 5-frame min) |

Pinning to a single thread made **both** backends slower, and egl collapsed to a
single frame. (The ❌ is the smoke test behaving correctly — it demands sustained
rendering — not a code bug.)

---

## 4. Hypotheses tested

| # | Hypothesis | Verdict | Evidence |
|---|-----------|---------|----------|
| 1 | EGL uses a different/slower software rasterizer (softpipe/swrast) | **Refuted** | `glxinfo`/`eglinfo` show identical llvmpipe in both. |
| 2 | The gap is the launch confounds (`-v4` logging, composable vs. standalone) | **Refuted as primary cause** | Controlled `-v1`/identical-composition still showed the gap on CI. (Confounds *did* matter on the local many-core box.) |
| 3 | llvmpipe rasterizer threads starve the readback path; freeing a core (`LP_NUM_THREADS=1`) will speed egl up | **Refuted** | `LP=1` made both *slower*; egl collapsed 0.80 → 0.10. The prediction's sign was wrong. |
| 4 | The camera is render-rate-throttled | **Ruled out** | Both run far below the world's 30 Hz cap → compute-bound. |

---

## 5. Leading explanation (localized, not yet profiled)

With the rasterizer, the launch confounds, thread contention, and throttling all
eliminated, the only remaining difference between the two runs is the
**window-system binding of the render target** — the EGL surfaceless/software-device
surface vs. the GLX drawable — and therefore the **per-frame GPU→CPU readback**
(`glReadPixels` → publish) the camera sensor performs on top of it. Both paths do
the same `gz sim -s`, same scene, same resolution, same Ogre2 render-to-texture,
same llvmpipe — so per-frame *rasterization work* is equal.

The behavior most consistent with the data is a **per-frame readback
serialization** in the EGL software path: a flush/finish plus a tiled→linear
resolve (possibly with a format/swizzle conversion if the EGL config doesn't match
the readback format) that does not pipeline as well as the GLX drawable's. That
predicts exactly the observed **core-count scaling**: more physical cores partially
hide the per-frame resolve (local ~2.5× gap), few cores cannot (CI ~8× gap), and
`LP_NUM_THREADS=1` removes the last of the overlap (egl craters to 1 frame).

**One-sentence summary:** *both paths rasterize identically; EGL is slower because
its software offscreen surface pays a costlier, less-pipelined per-frame
readback/resolve than GLX's drawable, and that cost dominates on a core-starved
runner.*

This last step is **not yet confirmed by profiling** — it is the best-supported
explanation after eliminating the alternatives, not a measured mechanism.

---

## 6. How to confirm the mechanism (future work)

Any of, runnable in the same container:

- `GALLIUM_HUD=frametime,fps` (or `cpu`) on both backends → per-frame timing.
- A **readback micro-benchmark**: render a fixed FBO and loop `glReadPixels` under
  GLX vs. EGL, with no sim, comparing pure readback throughput.
- `apitrace` / `MESA_DEBUG` on a single camera frame each → compare
  flush/finish/resolve call counts.
- Compare the chosen EGL config's surface format against the `glReadPixels` format
  to detect a CPU-side conversion.
- Add a `Runner info` CI step (`nproc; lscpu; free -h`) to record the exact host,
  removing the remaining assumption about runner specs.

---

## 7. What landed in the repo

Permanent, useful artifacts from this investigation:

- **`vrx_bringup/launch/sim_smoke.launch.xml`** — one controlled smoke launch used
  by both backends; only `headless_rendering` (→ `--headless-rendering`) differs.
  (`simulation.launch.xml` remains the production composable/GUI launch.)
- **`tools/smoke_test.sh`** — both backends use the unified launch and print the
  bound GL renderer (`glxinfo -B` / `eglinfo -B`) and effective `LP_NUM_THREADS`
  next to the FPS, so every job log is self-describing.
- **`.github/workflows/ci.yml`** — installs `mesa-utils` (provides `glxinfo`/
  `eglinfo`).

The `LP_NUM_THREADS` matrix axis was a temporary experiment; since it forces CI
red (egl `lp=1` always renders below the 5-frame threshold) it should be reverted
to the clean 2-job CI now that it has served its purpose.

---

## 8. Run index

| Purpose | Commit | Run ID |
|---------|--------|--------|
| Rolling → Lyrical, first green | `387f3e7d` | `26601399247` |
| 5 repeat dispatches (confounded) | `387f3e7d` | `26601990288`, `26601995794`, `26602000846`, `26602006255`, `26602011186` |
| Controlled launch + renderer diagnostics | `83487503` | `26634188291` |
| `LP_NUM_THREADS` matrix (auto vs 1) | `8809ca1d` | `26635828676` |
