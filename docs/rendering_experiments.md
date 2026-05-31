# Rendering experiments: faster CPU rasterizer & Xvfb replacement

**Date:** 2026-05-31
**Branch:** `jrivero/vrx4_ci` (experiments only — production CI lives on
`jrivero/vrx4_ci_clean`, which is xvfb-only)
**Harness:** `.github/workflows/experiments.yml` (manual `workflow_dispatch`,
non-blocking) + `tools/smoke_test.sh <xvfb|egl|vulkan>`

This investigates two questions raised after the EGL-vs-Xvfb benchmark
(`docs/egl_vs_xvfb_rendering_report.md`):

1. Can **llvmpipe** be replaced by a faster CPU rasterizer (a Rust one was hinted)?
2. Can **Xvfb** be replaced by a better CPU/headless alternative (Rust hinted)?

Both run with **no GPU** (GitHub-hosted `ubuntu-latest`, ~2 physical cores behind
4 SMT vCPUs), so "faster" means faster *software* rendering.

---

## TL;DR

- **There is no production Rust software rasterizer that beats llvmpipe today.**
  Rust rasterizers in the wild (wgpu/Vello/hobby projects) are not Mesa drivers
  and cannot be consumed by Gazebo's Ogre2. Rust *is* entering Mesa (the NVK
  hardware driver, Rusticl, gallium-rs glue) but **not** as a faster software
  rasterizer.
- **The real faster-CPU path is Vulkan on lavapipe.** Mesa's `lavapipe`
  (software Vulkan ICD, shares llvmpipe's LLVM JIT backend) beats Google's
  SwiftShader (~42% in vkmark) and is actively developed. Gazebo's Ogre2 has a
  **Vulkan** render backend, so we can run the camera headless on lavapipe.
- **The best "Xvfb replacement" is no display server at all.** EGL surfaceless
  and Vulkan headless both render offscreen with no X server. The Vulkan/lavapipe
  path needs *neither* Xvfb *nor* the EGL-GL surface.
- **Both questions converge on one experiment** — *Ogre2 Vulkan on lavapipe,
  fully headless* — and it is **viable**: verified rendering the VRX observation
  camera in a GPU-less `ros:lyrical-ros-base` container.

---

## 1. Investigation A — replace llvmpipe with a faster CPU rasterizer

### Candidates and verdicts

| Candidate | Language | Verdict for VRX/Ogre2 |
|-----------|----------|-----------------------|
| **Rust GL/Vulkan software rasterizer** (drop-in for llvmpipe) | Rust | **Does not exist.** No Rust software OpenGL/Vulkan ICD that Mesa loads / Ogre2 can use. Rust rasterizers (wgpu-based, Vello, etc.) are app-level renderers, not drivers. |
| **lavapipe** (Mesa software Vulkan ICD) | C + LLVM JIT | **Viable & promising.** Full Vulkan 1.4 frontend on the llvmpipe backend; faster than SwiftShader; Ogre2 supports a Vulkan backend. ← chosen experiment |
| **SwiftShader** (Google) | C++ | Possible (Vulkan/GLES ICD), but lavapipe now matches/beats it and isn't packaged in Ubuntu; extra build burden. Parked. |
| **OpenSWR / `swr`** (Intel threaded SW rasterizer) | C++ | Removed from modern Mesa. Dead end. |
| **softpipe** (`GALLIUM_DRIVER=softpipe`) | C | The slow reference Gallium driver — strictly *slower* than llvmpipe; useful only as a control. |

**Note:** Mesa 26 (we run 26.0.3) landed a rewritten llvmpipe rasterizer reported
~2–3× faster for 2D; llvmpipe itself is improving, which further narrows the case
for a hypothetical replacement.

### The chosen experiment

Run gz with the Vulkan backend on lavapipe, fully headless:

```
gz sim -r -s -v1 --render-engine-server ogre2 \
       --headless-rendering --render-engine-server-api-backend vulkan <world>
```

On a GPU-less host the only Vulkan ICD installed (`mesa-vulkan-drivers`) is
lavapipe, so Ogre2's Vulkan RenderSystem runs on it. `vulkaninfo --summary`
confirms the device.

---

## 2. Investigation B — replace Xvfb with a better headless alternative

### Candidates and verdicts

| Candidate | Language | Verdict |
|-----------|----------|---------|
| **EGL surfaceless / headless** | (Mesa) | Already exercised as the `egl` backend. No X server, but the GL readback path is slower (see EGL-vs-Xvfb report). |
| **Vulkan headless** (VK null-window, lavapipe) | (Mesa) | **Best fit.** No display server *and* no EGL-GL surface; renders straight to an offscreen Vulkan image. This is the `vulkan` backend. |
| **Smithay** (Rust Wayland compositor toolkit) | Rust | The Rust hint, but it is a *library* (sample compositor `anvil`), not a turnkey headless display like Xvfb. Would mean writing/operating a compositor + EGL-on-Wayland — high effort, no expected win over going display-server-less. Parked as a research curiosity. |
| **Xdummy** (Xorg dummy video driver) | C | A real X server with a dummy GPU; still software GL via llvmpipe, still a display server to manage. No advantage over Xvfb for our purpose. |
| **weston `--backend=headless`** + Xwayland | C | Adds a Wayland compositor layer; more moving parts, no rendering speedup. Parked. |

**Conclusion:** the strongest "replacement" for Xvfb is to *remove the display
server entirely*. Vulkan-headless on lavapipe does exactly that, which is why B
collapses into the same experiment as A.

---

## 3. Viability result (measured)

Local probe in a GPU-less `ros:lyrical-ros-base` container (no `/dev/dri`),
controlled launch (`sim_smoke.launch.xml`, ogre2, `-v1`):

| Backend | Stack | Device | Result | Camera FPS* |
|---------|-------|--------|--------|------------:|
| xvfb | GLX + llvmpipe (under Xvfb) | `llvmpipe (LLVM 21.1.8)` | PASS | ~7 |
| egl | EGL headless + llvmpipe | `llvmpipe` (EGL device sw) | PASS | ~3 |
| **vulkan** | **Ogre2 Vulkan + lavapipe, headless** | `llvmpipe (LLVM 21.1.8)` CPU Vulkan 1.4 | **PASS** | **~3.2** |

\* Single-sample, many-core local box; **absolute values are noisy and not
comparable across machines/runs.** The trustworthy claims are: (a) the Vulkan
backend *works* on lavapipe with no GPU and no X server, and (b) it lands in the
same ballpark as — and on this sample slightly above — the EGL-GL headless path,
while eliminating the display server. Whether it beats Xvfb/GLX on the CI runner
is exactly what `experiments.yml` is there to measure.

---

## 4. How to run

The non-blocking comparison (xvfb / egl / vulkan, isolated jobs so the numbers
are same-config and the leftover-`gz` collision can't happen) runs automatically
on every push to `jrivero/vrx4_ci`. Once `experiments.yml` exists on the default
branch it can also be dispatched manually against any ref:

```
gh workflow run experiments.yml --ref jrivero/vrx4_ci
```

Each job logs its bound renderer (`glxinfo`/`eglinfo`/`vulkaninfo`) next to the
camera FPS, so a silent softpipe/SwiftShader/llvmpipe fallback is always visible.
`continue-on-error` means a backend that can't initialize won't fail the others.

---

## 5. Recommendations & next steps

- **Production stays on xvfb (GLX + llvmpipe).** It is the fastest and most
  robust path on the GPU-less runner today; that is what `vrx4_ci_clean` ships.
- **Track Vulkan/lavapipe headless as the strategic direction.** It removes the X
  server dependency and rides Mesa's actively-optimized Vulkan path; if/when CI
  runners gain GPUs, the same Vulkan backend uses the hardware with no launch
  change.
- **Drop the "Rust replaces llvmpipe" line of inquiry** unless a Mesa-loadable
  Rust software driver appears — none exists now.
- **To turn the FPS numbers into evidence** (not single samples): run
  `experiments.yml` several times and average per backend, and/or add a
  `Runner info` step (`nproc; lscpu; free -h`) to record the exact host.
- **Possible follow-ups:** `GALLIUM_DRIVER=softpipe` as a slow control;
  measuring the Mesa-26 new-rasterizer effect; a pure `glReadPixels`
  GLX-vs-EGL-vs-Vulkan readback micro-benchmark to isolate the surface cost.

---

## 6. What landed in the repo (this branch)

- **`.github/workflows/experiments.yml`** — manual, non-blocking 3-way backend
  comparison.
- **`tools/smoke_test.sh`** — adds the `vulkan` backend (lavapipe, headless)
  alongside `xvfb` and `egl`.
- **`vrx_bringup/launch/sim_smoke.launch.xml`** — adds an `api_backend`
  (`opengl`|`vulkan`) arg to the headless path.
- **`docs/rendering_experiments.md`** — this document.
- **`docs/egl_vs_xvfb_rendering_report.md`** — the prior EGL-vs-Xvfb findings.

The blocking `ci.yml` on this branch is a single xvfb sanity smoke; the
rendering comparison is intentionally isolated in `experiments.yml`.
