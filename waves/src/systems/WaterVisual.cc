/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "WaterVisual.hh"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <list>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Color.hh>
#include <gz/math/Vector2.hh>
#include <gz/plugin/Register.hh>
#include <filesystem>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Mesh.hh>
#include <gz/rendering/MeshDescriptor.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ShaderParams.hh>
#include <gz/rendering/Visual.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SourceFilePath.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/sim/Util.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/waves/FFTWaveSimulation.hh"
#include "gz/sim/waves/GerstnerWaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"

#include "HeightMapTexture.hh"

namespace gz::sim::systems
{

class WaterVisual::Implementation
{
  /// \brief Re-resolve the visual pointer; runs on the render thread.
  public: bool ResolveVisual();

  /// \brief Upload uniforms; runs on the render thread under the cache mutex.
  public: void UploadUniforms();

  /// \brief Render-thread entry point (SceneUpdate event).
  public: void OnSceneUpdate();

  /// \brief Render-thread entry point (RenderTeardown event). Resets render
  /// resources so the next OnSceneUpdate rebuilds them from scratch.
  public: void OnRenderTeardown();

  // ---- Configuration (set once at Configure) ----
  public: std::string vertexShaderUri;        ///< Gerstner vertex shader
  public: std::string fftVertexShaderUri;     ///< FFT vertex shader (optional)
  public: std::string fragmentShaderUri;      ///< Shared fragment shader
  public: std::string computeShaderUri;       ///< GPU-FFT compute shader (optional)
  public: std::string evolveShaderUri;        ///< Stage 2 evolve shader (optional)
  public: std::string evolveDyShaderUri;      ///< Stage 2 evolve Dy companion (optional)
  public: bool         spectrumUploaded{false}; ///< Stage 2 one-shot init
  public: std::string bitrevShaderUri;        ///< Stage 3 IFFT bit-reverse pass (optional)
  public: std::string butterShaderUri;        ///< Stage 3 IFFT butterfly stage (optional)
  public: std::string combineEtaDxShaderUri;  ///< Stage 4 combine η+Dx (optional)
  public: std::string combineDyShaderUri;     ///< Stage 4 combine Dy (optional)
  public: std::string naiveShaderUri;         ///< Diagnostic naive O(N²) IFFT reference
  public: std::string testPatternShaderUri;   ///< Diagnostic test-pattern fill (optional)
  public: std::string viewHktShaderUri;       ///< Diagnostic hktTex viewer (optional)
  public: std::string bumpMapPath;
  public: std::string cubeMapPath;
  public: float rescale{0.125f};
  // Baseline parameters — defaults aligned with asv_wave_sim's
  // reference scene so a fresh setup gets the same visual neighbourhood.
  // Each can still be overridden in the model.sdf <parameters> block.
  public: gz::math::Vector2d bumpScale{64.0, 64.0};
  public: gz::math::Vector2d bumpSpeed{0.01, 0.01};
  // Strict asv_wave_sim defaults — match their literal values so a
  // visual comparison reflects the simulation/normal-computation
  // difference, not parameter divergence.
  public: float hdrMultiplier{0.4f};
  public: float fresnelPower{5.0f};
  public: float roughness{0.0f};
  public: float foamStrength{0.7f};     ///< Blend amount at J ≤ 0
  public: float foamThreshold{0.25f};   ///< Foam ramps in below this J
  // asv_wave_sim's exact default colours.
  public: gz::math::Color shallowColor{0.0f, 0.1f, 0.3f, 1.0f};
  public: gz::math::Color deepColor{0.0f, 0.05f, 0.2f, 1.0f};
  public: std::string visualName;
  public: Entity visualEntity{kNullEntity};
  public: std::string modelPath;

  // Tile instancing. The procedural water mesh is 200m × 200m; to
  // cover the visible horizon we render the same mesh at
  // (2·radius+1)² offset positions, each sharing the central tile's
  // material so they all sample the same heightmap and follow the
  // continuous periodic wavefield. 0 disables instancing.
  public: int tilesRadius{2};
  public: double tileMeshSize{200.0};
  public: std::vector<gz::rendering::VisualPtr> tileVisuals;

  // ---- Cross-thread cache, guarded by mutex_ ----
  // The cache is pre-sized to 3 components (matching the shader's vec3
  // layout) and zero-initialized so UploadUniforms can run safely before
  // PreUpdate has read the Wavefield component. PreUpdate later overwrites
  // these with real values.
  public: std::mutex mutex_;
  public: bool haveWavefield{false};
  public: std::uint64_t cachedGeneration{0};
  public: std::uint64_t lastUploadedGeneration{0};
  public: int cachedNwaves{3};
  public: std::vector<float> cachedAmplitudes{0.0f, 0.0f, 0.0f};
  public: std::vector<float> cachedWavenumbers{0.0f, 0.0f, 0.0f};
  public: std::vector<float> cachedOmegas{0.0f, 0.0f, 0.0f};
  public: std::vector<float> cachedSteepnesses{0.0f, 0.0f, 0.0f};
  public: std::vector<gz::math::Vector2f> cachedDirections{
    {1.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 0.0f}};
  public: float cachedTau{2.0f};
  public: float currentSimTime{0.0f};

  // ---- FFT path state ----
  public: bool useFft{false};
  /// \brief True iff the currently-bound material was created with the
  /// FFT vertex shader. Set when `ResolveVisual` builds the material;
  /// `UploadUniforms` keys off this (not `useFft`) when deciding
  /// whether to bind FFT-only uniforms like `world_matrix`. Prevents
  /// a startup race where the visual resolves before the wavefield
  /// component arrives, building a Gerstner material that later
  /// receives FFT-only uniforms.
  public: bool materialIsFft{false};
  public: std::shared_ptr<gz::sim::waves::FFTWaveSimulation> fftSim;
  public: std::unique_ptr<HeightMapTexture> heightMap;
  public: float cachedTileSize{200.0f};
  public: int   cachedGridSize{128};
  public: float cachedChopFactor{-1.0f};

  // ---- Render-thread state ----
  public: gz::rendering::ScenePtr scene;
  public: gz::rendering::VisualPtr visual;
  public: gz::rendering::MaterialPtr material;
  public: gz::common::ConnectionPtr sceneUpdateConn;
  public: gz::common::ConnectionPtr teardownConn;

  /// \brief gz-sim's `GuiRunner` loads this system *twice* for the same
  /// entity (once via the model SDF, once via the visual SDF — known
  /// upstream issue mentioned in asv_wave_sim#177). The two instances
  /// race on SetMaterial / heightmap creation. Dedupe at our level: the
  /// first `Configure` for an entity claims it; the second becomes a
  /// no-op so the render thread only does one round of material setup.
  public: bool active{true};
};

namespace
{
  std::mutex &VisualClaimMutex()
  {
    static std::mutex m;
    return m;
  }
  std::set<gz::sim::Entity> &VisualClaimSet()
  {
    static std::set<gz::sim::Entity> s;
    return s;
  }
}

bool WaterVisual::Implementation::ResolveVisual()
{
  if (this->visual)
    return true;
  if (!this->scene)
    this->scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!this->scene)
    return false;

  // BFS for the visual matching our entity id.
  auto root = this->scene->RootVisual();
  std::list<gz::rendering::NodePtr> queue{root};
  while (!queue.empty())
  {
    auto node = queue.front();
    queue.pop_front();
    if (node && node->HasUserData("gazebo-entity"))
    {
      auto var = node->UserData("gazebo-entity");
      if (auto *value = std::get_if<uint64_t>(&var);
          value && *value == static_cast<uint64_t>(this->visualEntity))
      {
        this->visual = std::dynamic_pointer_cast<gz::rendering::Visual>(node);
        break;
      }
    }
    for (unsigned int i = 0; i < node->ChildCount(); ++i)
      queue.push_back(node->ChildByIndex(i));
  }

  if (!this->visual)
    return false;

  // Stage 6 step 6.0: when GZ_WAVES_HLMS_PBS=1 and the FFT path is
  // active, hide the SDF visual and build a procedural HlmsPbs Item via
  // the bridge instead. The aim is to verify whether moving off
  // HlmsLowLevel alone fixes the 2-min first-frame stall.
  const char *pbsEnv = std::getenv("GZ_WAVES_HLMS_PBS");
  const bool useHlmsPbs = pbsEnv && std::string(pbsEnv) == "1" &&
                          this->useFft;

  if (!this->material)
  {
    // Defer material creation until the wavefield component has been
    // parsed and `useFft` has settled. Otherwise we'd build a Gerstner
    // material now, and a later UploadUniforms() (with useFft now
    // true) would try to bind FFT-only uniforms like `world_matrix`
    // and throw Ogre::ItemIdentityException. The visual just stays
    // unrendered for a few ticks at startup, which is harmless.
    if (!this->haveWavefield)
      return false;

    const std::string &vsUri = this->useFft && !this->fftVertexShaderUri.empty()
      ? this->fftVertexShaderUri
      : this->vertexShaderUri;
    gzmsg << "[WaterVisual] creating material with shaders ("
          << (this->useFft ? "fft" : "gerstner")
          << (useHlmsPbs ? ", HLMS_PBS path" : "") << ")" << std::endl;

    if (useHlmsPbs)
    {
      // Hide the SDF visual (it would otherwise render with its default
      // material on top of our procedural Item).
      this->visual->SetVisible(false);

      this->heightMap.reset();
      static std::atomic<std::uint64_t> seq{0};
      this->heightMap = std::make_unique<HeightMapTexture>(
          this->scene, gz::rendering::MaterialPtr{},
          static_cast<std::size_t>(this->cachedGridSize),
          "wavefield_heightmap_pbs_" +
              std::to_string(this->visualEntity) + "_" +
              std::to_string(seq.fetch_add(1)));

      const double planeSize = this->cachedTileSize;
      const int planeSegments =
          std::min(this->cachedGridSize, 200);
      const auto wpos = this->visual->WorldPosition();
      const bool ok = this->heightMap->CreatePbsVisual(
          planeSize, planeSegments,
          wpos.X(), wpos.Y(), wpos.Z(),
          "waves_pbs_" + std::to_string(this->visualEntity));
      if (!ok)
      {
        gzerr << "[WaterVisual] CreatePbsVisual failed; falling back to "
              << "HlmsLowLevel path" << std::endl;
        this->visual->SetVisible(true);
        // Fall through to the normal SetMaterial path below.
      }
      else
      {
        this->material = nullptr;
        return true;
      }
    }

    auto mat = this->scene->CreateMaterial();
    mat->SetVertexShader(vsUri);
    mat->SetFragmentShader(this->fragmentShaderUri);

    // Inherit CastShadows from the visual's existing material (mirrors the
    // built-in shader_param plugin's behavior). Without this Ogre Next may
    // refuse to apply our datablock and the mesh stays invisible.
    gz::rendering::MaterialPtr oldMat;
    if (this->visual->GeometryCount() > 0u)
      oldMat = this->visual->GeometryByIndex(0u)->Material();
    else
      oldMat = this->visual->Material();
    if (oldMat)
      mat->SetCastShadows(oldMat->CastShadows());

    this->visual->SetMaterial(mat);
    this->scene->DestroyMaterial(mat);
    this->material = this->visual->Material();
    if (!this->material)
    {
      gzerr << "[WaterVisual] failed to attach material to visual" << std::endl;
      return false;
    }
    // Latch which shader the material was actually created with — see
    // the `materialIsFft` doc comment for why we don't just trust
    // `useFft` later.
    this->materialIsFft = this->useFft;

    // Critical: upload all uniforms NOW, in the same render-thread call as
    // material creation. Ogre Next compiles/caches the material the first
    // time it's used; uniforms must be present at that point.
    this->UploadUniforms();

    // FFT path also needs a dynamic heightmap texture bound to the material.
    if (this->useFft)
    {
      // Destroy any previous instance first so its GPU texture frees up
      // before we ask Ogre Next to create the new one. Otherwise the new
      // `createOrRetrieveTexture` returns the still-Resident texture and
      // `setResolution` asserts (`mResidencyStatus == OnStorage`).
      this->heightMap.reset();
      // Use a process-unique texture name so we never collide with a
      // stale entry in `TextureGpuManager` left behind by a previous
      // engine teardown/reload cycle.
      static std::atomic<std::uint64_t> heightMapCounter{0};
      const auto seq = heightMapCounter.fetch_add(1);
      this->heightMap = std::make_unique<HeightMapTexture>(
        this->scene, this->material,
        static_cast<std::size_t>(this->cachedGridSize),
        "wavefield_heightmap_" + std::to_string(this->visualEntity) +
            "_" + std::to_string(seq));
      if (!this->heightMap->Ready())
      {
        gzerr << "[WaterVisual] heightmap texture failed to initialize"
              << std::endl;
      }
    }

    // Spawn tile copies. Each one shares the central material (so the
    // dynamic heightmap binding applies to all of them) and the same
    // mesh resource (Ogre caches the COLLADA load by URI). The FFT
    // wavefield is periodic in world XY, so neighbour tiles continue
    // the same wave pattern without seams.
    if (this->useFft && this->tilesRadius > 0 && !this->modelPath.empty())
    {
      // modelPath is the path to the model.sdf itself; take its parent
      // directory to anchor the mesh URI.
      const std::string meshPath =
          std::filesystem::path(this->modelPath).parent_path().string() +
          "/meshes/water.dae";
      const auto basePos = this->visual->WorldPosition();
      const int r = this->tilesRadius;
      gzmsg << "[WaterVisual] spawning tile instances: radius=" << r
            << " mesh_size=" << this->tileMeshSize << "m → "
            << ((2 * r + 1) * (2 * r + 1) - 1) << " extra tiles"
            << std::endl;
      for (int j = -r; j <= r; ++j)
      {
        for (int i = -r; i <= r; ++i)
        {
          if (i == 0 && j == 0) continue;  // center is the existing visual
          const std::string tileName =
              "water_tile_" + std::to_string(this->visualEntity) + "_" +
              std::to_string(i) + "_" + std::to_string(j);
          gz::rendering::VisualPtr tile = this->scene->CreateVisual(tileName);
          if (!tile) continue;
          gz::rendering::MeshDescriptor desc(meshPath);
          gz::rendering::MeshPtr mesh = this->scene->CreateMesh(desc);
          if (!mesh)
          {
            this->scene->DestroyVisual(tile);
            continue;
          }
          tile->AddGeometry(mesh);
          tile->SetMaterial(this->material, false);  // share, don't clone
          tile->SetWorldPosition(
              basePos.X() + i * this->tileMeshSize,
              basePos.Y() + j * this->tileMeshSize,
              basePos.Z());
          this->scene->RootVisual()->AddChild(tile);
          this->tileVisuals.push_back(tile);
        }
      }
    }
  }
  return this->material != nullptr || useHlmsPbs;
}

void WaterVisual::Implementation::UploadUniforms()
{
  if (!this->material)
    return;
  if (this->useFft)
  {
    gzmsg << "[WaterVisual] uploading uniforms (fft): tileSize="
          << this->cachedTileSize << " gridSize=" << this->cachedGridSize
          << " tau=" << this->cachedTau << std::endl;
  }
  else
  {
    gzmsg << "[WaterVisual] uploading uniforms (gerstner): Nwaves="
          << std::min(this->cachedNwaves, 3)
          << " a0=" << (this->cachedAmplitudes.size() > 0 ? this->cachedAmplitudes[0] : 0.0)
          << " k0=" << (this->cachedWavenumbers.size() > 0 ? this->cachedWavenumbers[0] : 0.0)
          << " w0=" << (this->cachedOmegas.size() > 0 ? this->cachedOmegas[0] : 0.0)
          << " tau=" << this->cachedTau << std::endl;
  }

  auto vsParams = this->material->VertexShaderParams();
  auto fsParams = this->material->FragmentShaderParams();

  // Engine-auto bindings (sentinel value = "auto from Ogre").
  (*vsParams)["worldviewproj_matrix"] = 1;
  (*vsParams)["camera_position_object_space"] = 1;

  // Static scalars/vec2s shared by both shaders.
  (*vsParams)["rescale"] = this->rescale;
  {
    float v[2] = {static_cast<float>(this->bumpScale.X()),
                  static_cast<float>(this->bumpScale.Y())};
    (*vsParams)["bumpScale"].InitializeBuffer(2);
    (*vsParams)["bumpScale"].UpdateBuffer(v);
  }
  {
    float v[2] = {static_cast<float>(this->bumpSpeed.X()),
                  static_cast<float>(this->bumpSpeed.Y())};
    (*vsParams)["bumpSpeed"].InitializeBuffer(2);
    (*vsParams)["bumpSpeed"].UpdateBuffer(v);
  }
  (*vsParams)["tau"] = this->cachedTau;

  // Gate FFT-only uniforms on the material's actual shader, not the
  // current `useFft` flag — if the visual resolved before the
  // wavefield component arrived, the material was built with the
  // Gerstner VS and any FFT-only binding (world_matrix, tileSize,
  // ...) would throw Ogre::ItemIdentityException.
  if (this->materialIsFft)
  {
    // FFT vertex shader uses world_matrix to compute world-space XY
    // for the periodic heightmap sample, so tile instances at
    // different world offsets each render their own piece of the
    // continuous wavefield (rather than each tile showing the same
    // patch in local model space). The Gerstner VS doesn't declare
    // this uniform, so binding it unconditionally throws an
    // ItemIdentityException there.
    (*vsParams)["world_matrix"] = 1;

    // FFT shader: heightmap texture (bound separately by HeightMapTexture)
    // plus the geometry of the periodic tile and the choppiness factor.
    (*vsParams)["tileSize"]   = this->cachedTileSize;
    (*vsParams)["gridSize"]   = this->cachedGridSize;
    (*vsParams)["chopFactor"] = this->cachedChopFactor;
    // CPU FFT uploads a slope map; GPU FFT doesn't (yet). The
    // useGpu env var gates the GPU pipeline, so default the uniform
    // accordingly. If the slope upload fails the VS still falls
    // back to finite differences when this is 0.
    const char *gpuEnv = std::getenv("GZ_WAVES_GPU_FFT");
    const char *stage2 = std::getenv("GZ_WAVES_GPU_FFT_STAGE2");
    const bool gpuPath = (gpuEnv && std::string(gpuEnv) == "1") ||
                         (stage2 && std::string(stage2) == "1");
    const bool encinoPath = this->fftSim && this->fftSim->UseEncino();
    (*vsParams)["useSlopeMap"] = (gpuPath || encinoPath) ? 0 : 1;
  }
  else
  {
    // Pack up to 3 components into vec3 / per-direction vec2 uniforms,
    // matching the conservative GLSL layout that Ogre Next compiles
    // reliably. Extra components beyond 3 are dropped on the visual side;
    // physics consumers can still see them via the component arrays.
    float amp[3]   = {0.0f, 0.0f, 0.0f};
    float knum[3]  = {0.0f, 0.0f, 0.0f};
    float om[3]    = {0.0f, 0.0f, 0.0f};
    float steep[3] = {0.0f, 0.0f, 0.0f};
    float d0[2]    = {1.0f, 0.0f};
    float d1[2]    = {1.0f, 0.0f};
    float d2[2]    = {1.0f, 0.0f};
    const int n = std::min(this->cachedNwaves, 3);
    for (int i = 0; i < n; ++i)
    {
      amp[i]   = this->cachedAmplitudes[i];
      knum[i]  = this->cachedWavenumbers[i];
      om[i]    = this->cachedOmegas[i];
      steep[i] = this->cachedSteepnesses[i];
    }
    if (n > 0) { d0[0] = this->cachedDirections[0].X();
                 d0[1] = this->cachedDirections[0].Y(); }
    if (n > 1) { d1[0] = this->cachedDirections[1].X();
                 d1[1] = this->cachedDirections[1].Y(); }
    if (n > 2) { d2[0] = this->cachedDirections[2].X();
                 d2[1] = this->cachedDirections[2].Y(); }

    (*vsParams)["Nwaves"] = n;
    (*vsParams)["amplitude"].InitializeBuffer(3);
    (*vsParams)["amplitude"].UpdateBuffer(amp);
    (*vsParams)["wavenumber"].InitializeBuffer(3);
    (*vsParams)["wavenumber"].UpdateBuffer(knum);
    (*vsParams)["omega"].InitializeBuffer(3);
    (*vsParams)["omega"].UpdateBuffer(om);
    (*vsParams)["steepness"].InitializeBuffer(3);
    (*vsParams)["steepness"].UpdateBuffer(steep);
    (*vsParams)["dir0"].InitializeBuffer(2);
    (*vsParams)["dir0"].UpdateBuffer(d0);
    (*vsParams)["dir1"].InitializeBuffer(2);
    (*vsParams)["dir1"].UpdateBuffer(d1);
    (*vsParams)["dir2"].InitializeBuffer(2);
    (*vsParams)["dir2"].UpdateBuffer(d2);
  }

  // Fragment shader: colours + lighting params + textures.
  (*fsParams)["hdrMultiplier"] = this->hdrMultiplier;
  (*fsParams)["fresnelPower"]  = this->fresnelPower;
  (*fsParams)["roughness"]     = this->roughness;
  // Foam path. FFT mode reads the heightmap for the Tessendorf
  // Jacobian; gerstner keeps foamStrength=0 so the FS short-circuits
  // before sampling an unbound heightMap.
  if (this->useFft)
  {
    (*fsParams)["chopFactor"]    = this->cachedChopFactor;
    (*fsParams)["tileSize"]      = this->cachedTileSize;
    (*fsParams)["foamStrength"]  = this->foamStrength;
    (*fsParams)["foamThreshold"] = this->foamThreshold;
  }
  else
  {
    (*fsParams)["chopFactor"]    = 0.0f;
    (*fsParams)["tileSize"]      = 1.0f;
    (*fsParams)["foamStrength"]  = 0.0f;
    (*fsParams)["foamThreshold"] = 1.0f;
  }
  {
    float v[4] = {this->shallowColor.R(), this->shallowColor.G(),
                  this->shallowColor.B(), this->shallowColor.A()};
    (*fsParams)["shallowColor"].InitializeBuffer(4);
    (*fsParams)["shallowColor"].UpdateBuffer(v);
  }
  {
    float v[4] = {this->deepColor.R(), this->deepColor.G(),
                  this->deepColor.B(), this->deepColor.A()};
    (*fsParams)["deepColor"].InitializeBuffer(4);
    (*fsParams)["deepColor"].UpdateBuffer(v);
  }
  if (!this->bumpMapPath.empty())
  {
    (*fsParams)["bumpMap"].SetTexture(
      this->bumpMapPath,
      gz::rendering::ShaderParam::ParamType::PARAM_TEXTURE);
  }
  if (!this->cubeMapPath.empty())
  {
    (*fsParams)["cubeMap"].SetTexture(
      this->cubeMapPath,
      gz::rendering::ShaderParam::ParamType::PARAM_TEXTURE_CUBE, 1u);
  }
  // Patch the bump/cube samplers to use anisotropic trilinear
  // filtering. gz::rendering's ShaderParam binding installs default
  // bilinear-without-mips, which aliases badly when the bumpmap is
  // tiled densely (see fft_water_vs_330.glsl's bumpResolution).
  if (this->heightMap)
  {
    if (!this->bumpMapPath.empty())
      this->heightMap->SetTexFiltering("bumpMap");
    if (!this->cubeMapPath.empty())
      this->heightMap->SetTexFiltering("cubeMap");
  }

  this->lastUploadedGeneration = this->cachedGeneration;
}

void WaterVisual::Implementation::OnSceneUpdate()
{
  if (!this->active)
    return;
  if (this->visualName.empty())
    return;
  if (!this->ResolveVisual())
    return;

  std::lock_guard<std::mutex> lock(this->mutex_);
  if (this->haveWavefield &&
      this->cachedGeneration != this->lastUploadedGeneration)
  {
    this->UploadUniforms();
  }
  if (this->material)
  {
    auto vsParams = this->material->VertexShaderParams();
    (*vsParams)["t"] = this->currentSimTime;
  }

  // FFT visual path: either dispatch the GPU compute shader (Stage 1+ of
  // the GPU-FFT plan) when GZ_WAVES_GPU_FFT=1 and a compute shader URI
  // was configured, or fall back to the CPU IFFT + upload.
  if (this->useFft && this->fftSim && this->heightMap &&
      this->heightMap->Ready())
  {
    const char *gpuEnv = std::getenv("GZ_WAVES_GPU_FFT");
    const bool useGpu = gpuEnv && std::string(gpuEnv) == "1" &&
                        !this->computeShaderUri.empty();
    const char *stage2Env = std::getenv("GZ_WAVES_GPU_FFT_STAGE2");
    const bool useStage2 = stage2Env && std::string(stage2Env) == "1" &&
                           !this->evolveShaderUri.empty();
    const char *cpuFeedEnv = std::getenv("GZ_WAVES_GPU_FFT_CPU_FEED");
    const bool useCpuFeed = cpuFeedEnv && std::string(cpuFeedEnv) == "1";
    bool ok = false;

    // Diagnostic: feed CPU's IFFT output straight into the GPU
    // visual texture, bypassing evolve+IFFT entirely. If this looks
    // identical to the standard CPU path, the visual sampling layer
    // is correct and the GPU compute chain is the bug. If it still
    // looks "fast", the visual sampling layer itself is wrong.
    if (useCpuFeed)
    {
      this->fftSim->Update(static_cast<double>(this->currentSimTime));
      ok = this->heightMap->CpuFeed(this->fftSim->HeightGrid(),
                                     this->fftSim->DispXGrid(),
                                     this->fftSim->DispYGrid());
      static bool loggedCpuFeed = false;
      if (ok && !loggedCpuFeed)
      {
        loggedCpuFeed = true;
        gzmsg << "[WaterVisual] CPU-feed diagnostic active — feeding "
              << "the CPU FFT output into ifftFinalTex; evolve+IFFT "
              << "bypassed. If this looks like CPU, the GPU compute "
              << "chain is the bug." << std::endl;
      }
    }
    else if (useStage2)
    {
      // Stage 2 of the GPU-FFT plan: dispatch the evolve compute
      // shader so h(k, t) is recomputed on the GPU each frame from
      // the once-uploaded h0 / omega textures. No spatial output
      // yet — Stage 3 (IFFT) is what produces the heightmap the
      // visual samples. Until then this path produces no visual
      // change; we're just exercising the compute pipeline.
      if (!this->spectrumUploaded)
      {
        // Convert Eigen complex matrices (column-major) to flat
        // row-major double buffers the bridge expects. Build them
        // with an explicit element-by-element loop — we previously
        // used `RowMatrix x = mxcd.real()` and similar Eigen Block
        // conversions, but the resulting GPU texture content
        // mismatched CPU's spectrum (verified by replacing the
        // upload path with a shader-synthesised Phillips spectrum).
        const auto &h0 = this->fftSim->H0();
        const auto &hc = this->fftSim->H0Conj();
        const int N = static_cast<int>(this->fftSim->GridSize());
        std::vector<double> h0Re(static_cast<std::size_t>(N) * N);
        std::vector<double> h0Im(static_cast<std::size_t>(N) * N);
        std::vector<double> hcRe(static_cast<std::size_t>(N) * N);
        std::vector<double> hcIm(static_cast<std::size_t>(N) * N);
        for (int i = 0; i < N; ++i)
        {
          for (int j = 0; j < N; ++j)
          {
            const std::size_t idx =
                static_cast<std::size_t>(i) * N + j;
            h0Re[idx] = h0(i, j).real();
            h0Im[idx] = h0(i, j).imag();
            hcRe[idx] = hc(i, j).real();
            hcIm[idx] = hc(i, j).imag();
          }
        }

        if (this->heightMap->UploadSpectrum(
                h0Re.data(), h0Im.data(),
                hcRe.data(), hcIm.data(), N))
        {
          this->spectrumUploaded = true;
        }

        // GPU readback diagnostics (gated by GZ_WAVES_GPU_FFT_DEBUG=1).
        const char *dbgEnv = std::getenv("GZ_WAVES_GPU_FFT_DEBUG");
        const bool debugOn =
            dbgEnv && std::string(dbgEnv) == "1";
        if (debugOn && this->spectrumUploaded)
        {
          // Magnitude statistics so we can spot upload-side scaling
          // problems. Compute |h0(1, 0)| and the cell with the
          // largest |h0| as ground-truth references.
          double maxAbsH0 = 0.0;
          int maxI = 0, maxJ = 0;
          for (int i = 0; i < N; ++i)
          {
            for (int j = 0; j < N; ++j)
            {
              const double mag = std::abs(h0(i, j));
              if (mag > maxAbsH0)
              {
                maxAbsH0 = mag;
                maxI = i;
                maxJ = j;
              }
            }
          }
          gzmsg << "[WaterVisual] spectrum stats: |h0(1,0)|="
                << std::abs(h0(1, 0))
                << " |h0|_max=" << maxAbsH0
                << " @ (i=" << maxI << ", j=" << maxJ << ")"
                << std::endl;
          const auto dump = [&](int i, int j)
          {
            float gpuRe = 0, gpuIm = 0, gpuConjRe = 0, gpuConjIm = 0;
            const bool ok = this->heightMap->ReadbackH0Cell(
                i, j, &gpuRe, &gpuIm, &gpuConjRe, &gpuConjIm);
            const double cpuRe = h0(i, j).real();
            const double cpuIm = h0(i, j).imag();
            const double cpuMag = std::abs(h0(i, j));
            const double gpuMag =
                std::sqrt(static_cast<double>(gpuRe) * gpuRe +
                           static_cast<double>(gpuIm) * gpuIm);
            const double cpuConjRe = hc(i, j).real();
            const double cpuConjIm = hc(i, j).imag();
            gzmsg << "[WaterVisual] h0(" << i << "," << j << ")  "
                  << "CPU:(re=" << cpuRe << " im=" << cpuIm
                  << " |h0|=" << cpuMag << ")  "
                  << "GPU:(re=" << gpuRe << " im=" << gpuIm
                  << " |h0|=" << gpuMag << ")  "
                  << "ok=" << ok << std::endl;
            gzmsg << "[WaterVisual] h0Conj(" << i << "," << j << ") "
                  << "CPU:(re=" << cpuConjRe << " im=" << cpuConjIm
                  << ")  "
                  << "GPU:(re=" << gpuConjRe << " im=" << gpuConjIm
                  << ")" << std::endl;
          };
          dump(0, 0);
          dump(1, 0);
          dump(0, 1);
          dump(maxI, maxJ);
          dump(N / 2, N / 2);
          dump(N - 1, 0);
          dump(N - 1, N - 1);
        }
      }
      if (this->spectrumUploaded)
      {
        ok = this->heightMap->EvolveDispatch(this->evolveShaderUri,
                                             this->currentSimTime,
                                             this->cachedTau,
                                             this->cachedTileSize);
        if (ok && !this->evolveDyShaderUri.empty())
        {
          this->heightMap->EvolveDyDispatch(this->evolveDyShaderUri,
                                            this->currentSimTime,
                                            this->cachedTau,
                                            this->cachedTileSize);
        }
        static bool loggedStage2 = false;
        if (ok && !loggedStage2)
        {
          loggedStage2 = true;
          gzmsg << "[WaterVisual] GPU-FFT Stage 2 (evolve) online — "
                << "h(k, t) computed on GPU. Visual is unchanged "
                << "until Stage 3 (IFFT) lands." << std::endl;
        }
      }
      // Stage 3: 2D IFFT over h(k, t) on the GPU. Produces the spatial
      // η(x, y, t) in a ping-pong texture. Stage 3 still doesn't bind
      // that texture to the visual material — Stage 4 will. For now
      // this exercises the full Cooley-Tukey pipeline end-to-end.
      const char *stage3Env = std::getenv("GZ_WAVES_GPU_FFT_STAGE3");
      const bool useStage3 = stage3Env && std::string(stage3Env) == "1" &&
                             !this->bitrevShaderUri.empty() &&
                             !this->butterShaderUri.empty() &&
                             this->spectrumUploaded;
      if (useStage3)
      {
        const char *naiveEnv = std::getenv("GZ_WAVES_GPU_FFT_NAIVE");
        const bool useNaive = naiveEnv && std::string(naiveEnv) == "1"
                              && !this->naiveShaderUri.empty();
        const bool ifftOk = useNaive
            ? this->heightMap->IfftNaiveDispatch(this->naiveShaderUri)
            : this->heightMap->IfftDispatch(this->bitrevShaderUri,
                                            this->butterShaderUri);
        static bool loggedStage3 = false;
        if (ifftOk && !loggedStage3)
        {
          loggedStage3 = true;
          gzmsg << "[WaterVisual] GPU-FFT Stage 3 (IFFT) online — "
                << "η(x, t) computed on GPU. Visual stays on CPU "
                << "upload path until Stage 4 binds the GPU output."
                << std::endl;
        }

        // Stage 4: assemble (η, Dx, Dy, _) into the visual texture.
        // Only runs when both the Dy evolve and the combine shaders
        // are configured. Without combine, ifft_dispatch's fallback
        // binds the packed η+Dx texture directly (degraded mode, no
        // chop displacement).
        if (ifftOk && !useNaive &&
            !this->evolveDyShaderUri.empty() &&
            !this->combineEtaDxShaderUri.empty() &&
            !this->combineDyShaderUri.empty())
        {
          const bool combineOk = this->heightMap->CombineDispatch(
              this->combineEtaDxShaderUri, this->combineDyShaderUri);
          static bool loggedStage4 = false;
          if (combineOk && !loggedStage4)
          {
            loggedStage4 = true;
            gzmsg << "[WaterVisual] GPU-FFT Stage 4 (combine) online — "
                  << "(η, Dx, Dy) bound to material; CPU upload path "
                  << "retired." << std::endl;
          }

          // Diagnostic: scan combinedTex once for non-finite cells +
          // per-channel ranges. Gated by GZ_WAVES_GPU_FFT_DEBUG=1.
          const char *scanEnv = std::getenv("GZ_WAVES_GPU_FFT_DEBUG");
          static int scanFrameCounter = 0;
          static bool ranScan = false;
          if (combineOk && scanEnv && std::string(scanEnv) == "1"
              && !ranScan)
          {
            ++scanFrameCounter;
            if (scanFrameCounter == 60)  // ~1s after combine online
            {
              ranScan = true;
              int badCount = 0;
              float mn[4] = {0, 0, 0, 0}, mx[4] = {0, 0, 0, 0};
              int bi = -1, bj = -1;
              float br[4] = {0, 0, 0, 0};
              if (this->heightMap->ReadbackCombinedScan(
                      &badCount, mn, mx, &bi, &bj, br))
              {
                gzmsg << "[WaterVisual] combinedTex scan: bad="
                      << badCount
                      << "  η[min,max]=[" << mn[0] << "," << mx[0]
                      << "]  Dx[min,max]=[" << mn[1] << "," << mx[1]
                      << "]  Dy[min,max]=[" << mn[2] << "," << mx[2]
                      << "]  a[min,max]=[" << mn[3] << "," << mx[3]
                      << "]" << std::endl;
                if (badCount > 0)
                {
                  gzwarn << "[WaterVisual] first bad cell @ ("
                         << bi << "," << bj << ") = ("
                         << br[0] << "," << br[1] << ","
                         << br[2] << "," << br[3] << ")"
                         << std::endl;
                }
              }
            }
          }
        }

        // Diagnostic: ~5s after Stage 3 comes online, read back the
        // GPU's ifftFinalTex and compare cell-by-cell to CPU's
        // heightGrid_ at the same simTime. Gated by
        // GZ_WAVES_GPU_FFT_DEBUG=1.
        const char *ifftDbgEnv = std::getenv("GZ_WAVES_GPU_FFT_DEBUG");
        const bool ifftDebugOn =
            ifftDbgEnv && std::string(ifftDbgEnv) == "1";
        static int diagFrameCounter = 0;
        static bool ranIfftDiag = false;
        if (ifftDebugOn && ifftOk && this->heightMap->GpuOutputBound())
        {
          ++diagFrameCounter;
          if (!ranIfftDiag && diagFrameCounter == 300)
          {
            ranIfftDiag = true;
            const float t = this->currentSimTime;
            this->fftSim->Update(static_cast<double>(t));
            const auto &eta = this->fftSim->HeightGrid();
            // Also compute expected h(k, t) directly for comparison
            // against GPU's hktTex readback at the same cells.
            const auto &h0  = this->fftSim->H0();
            const auto &hc  = this->fftSim->H0Conj();
            const auto &om  = this->fftSim->OmegaGrid();
            // Compute ramp at this t exactly like CPU does.
            const double rampVal = (this->cachedTau > 0.0)
                ? (1.0 - std::exp(-t / this->cachedTau)) : 1.0;
            auto cpuHkt = [&](int i, int j) -> std::complex<double>
            {
              const double w = om(i, j);
              const std::complex<double> e_plus(std::cos(w * t),
                                                  std::sin(w * t));
              const std::complex<double> e_minus = std::conj(e_plus);
              const std::complex<double> h =
                  h0(i, j) * e_plus + hc(i, j) * e_minus;
              return h * rampVal;
            };
            gzmsg << "[WaterVisual] IFFT η comparison at t=" << t
                  << " (300 frames after Stage 3 online)" << std::endl;
            auto cmp = [&](int i, int j)
            {
              float gpuEta = 0.0f;
              const bool ok =
                  this->heightMap->ReadbackIfftCell(i, j, &gpuEta);
              const double cpuEta = eta(i, j);
              const double diff =
                  static_cast<double>(gpuEta) - cpuEta;
              const double ratio = (std::abs(cpuEta) > 1e-9)
                  ? gpuEta / cpuEta : 0.0;
              gzmsg << "[WaterVisual] η(" << i << "," << j << ")  "
                    << "CPU=" << cpuEta << "  GPU=" << gpuEta
                    << "  diff=" << diff
                    << "  ratio=" << ratio
                    << "  ok=" << ok << std::endl;
            };
            cmp(0, 0);
            cmp(1, 1);
            cmp(32, 32);
            cmp(64, 0);
            cmp(64, 64);
            cmp(100, 50);
            cmp(127, 0);
            cmp(127, 127);

            // Compare evolve's hktTex output to CPU's expected
            // h(k, t) cell-by-cell.
            auto cmpHkt = [&](int i, int j)
            {
              float gpuRe = 0.0f, gpuIm = 0.0f;
              const bool ok =
                  this->heightMap->ReadbackHktCell(i, j, &gpuRe, &gpuIm);
              const std::complex<double> hh = cpuHkt(i, j);
              gzmsg << "[WaterVisual] hkt(" << i << "," << j << ")  "
                    << "CPU=(" << hh.real() << "," << hh.imag() << ")  "
                    << "GPU=(" << gpuRe << "," << gpuIm << ")  "
                    << "ok=" << ok << std::endl;
            };
            cmpHkt(0, 0);
            cmpHkt(1, 0);
            cmpHkt(126, 127);
            cmpHkt(64, 64);
            cmpHkt(127, 127);

            // Magnitude statistics across the whole CPU heightGrid.
            double cpuMin = 1e9, cpuMax = -1e9, cpuSumSq = 0.0;
            for (int i = 0; i < eta.rows(); ++i)
            {
              for (int j = 0; j < eta.cols(); ++j)
              {
                const double v = eta(i, j);
                if (v < cpuMin) cpuMin = v;
                if (v > cpuMax) cpuMax = v;
                cpuSumSq += v * v;
              }
            }
            gzmsg << "[WaterVisual] CPU η stats: min=" << cpuMin
                  << "  max=" << cpuMax
                  << "  rms=" << std::sqrt(cpuSumSq /
                                             (eta.rows() * eta.cols()))
                  << std::endl;
          }
        }

        // Diagnostic: after the IFFT, optionally view hktTex (Stage
        // 2's output) directly. Distinguishes "evolve produces zero"
        // from "IFFT loses evolve's output".
        const char *vhEnv = std::getenv("GZ_WAVES_GPU_FFT_VIEW_HKT");
        if (vhEnv && std::string(vhEnv) == "1" &&
            !this->viewHktShaderUri.empty() &&
            this->heightMap->GpuOutputBound())
        {
          const float scale = 0.01f;
          const bool vhOk =
              this->heightMap->ViewHktDispatch(
                  this->viewHktShaderUri, scale);
          static bool loggedVh = false;
          if (vhOk && !loggedVh)
          {
            loggedVh = true;
            gzmsg << "[WaterVisual] GPU-FFT view-hkt ENABLED — "
                  << "ifftFinalTex overwritten with |h(k,t)| · "
                  << scale << ". If patterns are visible, evolve "
                  << "produced data and the IFFT is what's broken; "
                  << "if still flat, evolve/upload is what's broken."
                  << std::endl;
          }
        }

        // Diagnostic: after the IFFT, optionally overwrite
        // ifftFinalTex with a known sine pattern to isolate
        // binding/sampling bugs from compute bugs. If waves appear
        // with this flag set but not without, the compute pipeline
        // is broken; if still flat, the binding path is broken.
        const char *tpEnv =
            std::getenv("GZ_WAVES_GPU_FFT_TEST_PATTERN");
        if (tpEnv && std::string(tpEnv) == "1" &&
            !this->testPatternShaderUri.empty() &&
            this->heightMap->GpuOutputBound())
        {
          const float amplitude = 1.5f;
          const bool tpOk = this->heightMap->TestPatternDispatch(
              this->testPatternShaderUri,
              this->currentSimTime, amplitude);
          static bool loggedTp = false;
          if (tpOk && !loggedTp)
          {
            loggedTp = true;
            gzmsg << "[WaterVisual] GPU-FFT test pattern ENABLED — "
                  << "ifftFinalTex overwritten with a moving sine "
                  << "(amp=" << amplitude << " m). If waves are now "
                  << "visible, the IFFT compute is at fault; if "
                  << "still flat, the binding/sampling path is."
                  << std::endl;
          }
        }
      }
      // Stage 2/3 don't update the spatial heightmap the visual reads
      // from, so we still need *something* there; fall through to the
      // standard CPU/Stage-1 path below.
    }
    if (useGpu)
    {
      ok = this->heightMap->Dispatch(this->computeShaderUri,
                                     this->currentSimTime,
                                     this->cachedTileSize);
      static bool loggedGpu = false;
      if (ok && !loggedGpu)
      {
        loggedGpu = true;
        gzmsg << "[WaterVisual] GPU-FFT dispatch online — compute shader "
              << this->computeShaderUri << std::endl;
      }
    }
    else if (this->heightMap->GpuOutputBound())
    {
      // Stage 4: GPU IFFT output is bound to the visual material.
      // CPU `fftSim->Update + Upload` is no longer needed on the
      // render path. The CPU FFTWaveSimulation still runs server-side
      // for buoyancy queries (Stage 5).
      ok = true;
      static bool loggedStage4 = false;
      if (!loggedStage4)
      {
        loggedStage4 = true;
        gzmsg << "[WaterVisual] GPU-FFT Stage 4 online — visual now "
              << "samples the GPU IFFT output directly; CPU heightmap "
              << "upload skipped on the render thread." << std::endl;
      }
    }
    else
    {
      // Server and GUI run in separate processes (gz_server composable
      // node vs gz-sim -g executable); the Wavefield component carries
      // only the parameters, and each side instantiates its own
      // FFTWaveSimulation from them. So the visual MUST drive its own
      // Update each frame — there is no shared grid to read from.
      //
      // Run at full render rate (60 Hz). A throttle here would only
      // save CPU in the GUI process — which doesn't enter the server's
      // RTF accounting — at the cost of a stale-grid stutter every
      // time the render rate is faster than the throttle period.
      this->fftSim->Update(static_cast<double>(this->currentSimTime));
      ok = this->heightMap->Upload(this->fftSim->HeightGrid(),
                                   this->fftSim->DispXGrid(),
                                   this->fftSim->DispYGrid());
      // Upload the slope and chop-derivative grids so the VS can
      // build the full Tessendorf chop-aware tangent + normal per
      // vertex instead of finite-differencing the displaced surface.
      // Skip on the Encino path — it only fills Height/Dx/Dy; the VS
      // falls back to finite-diff normals (gated by useSlopeMap=0).
      if (ok && !this->fftSim->UseEncino())
      {
        this->heightMap->UploadSlope(this->fftSim->SlopeXGrid(),
                                      this->fftSim->SlopeYGrid());
        this->heightMap->UploadChopDerivatives(
            this->fftSim->DispDxDxGrid(),
            this->fftSim->DispDyDyGrid(),
            this->fftSim->DispDxDyGrid());
      }
    }
    // One-shot diagnostic on the very first successful CPU upload so we
    // can see the actual amplitudes the GPU is sampling. Helps
    // distinguish "upload silently failing" from "Phillips spectrum is
    // tiny". (Skipped on the GPU-FFT path; that path has its own log.)
    static bool logged = false;
    if (ok && !useGpu && !logged)
    {
      logged = true;
      const auto &eta = this->fftSim->HeightGrid();
      const auto &dx  = this->fftSim->DispXGrid();
      const auto &dy  = this->fftSim->DispYGrid();
      gzmsg << "[WaterVisual] first FFT upload: η range=["
            << eta.minCoeff() << ", " << eta.maxCoeff()
            << "] m, |Dx|max=" << dx.cwiseAbs().maxCoeff()
            << " m, |Dy|max=" << dy.cwiseAbs().maxCoeff()
            << " m, chopFactor=" << this->cachedChopFactor << std::endl;
    }
  }
}

void WaterVisual::Implementation::OnRenderTeardown()
{
  // Destroy the FFT heightmap texture BEFORE the scene/material handles
  // go away, so the bridge can still walk Ogre's TextureGpuManager to
  // release it. After teardown the next ResolveVisual will rebuild it.
  this->heightMap.reset();
  // Drop our tile-instance handles. The scene owns the actual
  // Visuals; resetting our shared pointers here lets it clean up.
  if (this->scene)
  {
    for (auto &tile : this->tileVisuals)
    {
      if (tile)
        this->scene->DestroyVisual(tile);
    }
  }
  this->tileVisuals.clear();
  this->visual.reset();
  this->material.reset();
  this->scene.reset();
  // Force re-upload after the scene rebuilds.
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->lastUploadedGeneration = 0;
}

WaterVisual::WaterVisual()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

WaterVisual::~WaterVisual() = default;

void WaterVisual::Configure(
  const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &_eventMgr)
{
  GZ_PROFILE("WaterVisual::Configure");
  auto sdf = std::const_pointer_cast<sdf::Element>(_sdf);

  if (!sdf->HasElement("shader"))
  {
    gzerr << "[WaterVisual] <shader> element is required" << std::endl;
    return;
  }

  this->dataPtr->visualEntity = _entity;
  if (auto *name = _ecm.Component<components::Name>(_entity))
    this->dataPtr->visualName = name->Data();

  // Dedupe: only the first instance for this entity does real work.
  {
    std::lock_guard<std::mutex> lock(VisualClaimMutex());
    auto &claims = VisualClaimSet();
    if (claims.count(_entity))
    {
      gzmsg << "[WaterVisual] entity " << _entity
            << " already claimed by another WaterVisual instance — "
            << "this one will be inactive" << std::endl;
      this->dataPtr->active = false;
      return;
    }
    claims.insert(_entity);
  }

  // Resolve the model path for relative shader / texture URIs.
  const auto modelEnt = topLevelModel(_entity, _ecm);
  if (auto *src = _ecm.Component<components::SourceFilePath>(modelEnt))
    this->dataPtr->modelPath = src->Data();

  auto resolve = [&](const std::string &uri) {
    return gz::common::findFile(asFullPath(uri, this->dataPtr->modelPath));
  };

  auto shader = sdf->GetElement("shader");
  this->dataPtr->vertexShaderUri =
    resolve(shader->GetElement("vertex")->Get<std::string>());
  this->dataPtr->fragmentShaderUri =
    resolve(shader->GetElement("fragment")->Get<std::string>());
  if (shader->HasElement("fft_vertex"))
  {
    this->dataPtr->fftVertexShaderUri =
      resolve(shader->GetElement("fft_vertex")->Get<std::string>());
  }
  if (shader->HasElement("gpu_compute"))
  {
    this->dataPtr->computeShaderUri =
      resolve(shader->GetElement("gpu_compute")->Get<std::string>());
  }
  if (shader->HasElement("gpu_ifft_bitreverse"))
  {
    this->dataPtr->bitrevShaderUri =
      resolve(shader->GetElement("gpu_ifft_bitreverse")->Get<std::string>());
  }
  if (shader->HasElement("gpu_ifft_butterfly"))
  {
    this->dataPtr->butterShaderUri =
      resolve(shader->GetElement("gpu_ifft_butterfly")->Get<std::string>());
  }
  if (shader->HasElement("gpu_ifft_naive"))
  {
    this->dataPtr->naiveShaderUri =
      resolve(shader->GetElement("gpu_ifft_naive")->Get<std::string>());
  }
  if (shader->HasElement("gpu_test_pattern"))
  {
    this->dataPtr->testPatternShaderUri =
      resolve(shader->GetElement("gpu_test_pattern")->Get<std::string>());
  }
  if (shader->HasElement("gpu_view_hkt"))
  {
    this->dataPtr->viewHktShaderUri =
      resolve(shader->GetElement("gpu_view_hkt")->Get<std::string>());
  }
  if (shader->HasElement("gpu_evolve"))
  {
    this->dataPtr->evolveShaderUri =
      resolve(shader->GetElement("gpu_evolve")->Get<std::string>());
  }
  if (shader->HasElement("gpu_evolve_dy"))
  {
    this->dataPtr->evolveDyShaderUri =
      resolve(shader->GetElement("gpu_evolve_dy")->Get<std::string>());
  }
  if (shader->HasElement("gpu_combine_eta_dx"))
  {
    this->dataPtr->combineEtaDxShaderUri = resolve(
        shader->GetElement("gpu_combine_eta_dx")->Get<std::string>());
  }
  if (shader->HasElement("gpu_combine_dy"))
  {
    this->dataPtr->combineDyShaderUri =
      resolve(shader->GetElement("gpu_combine_dy")->Get<std::string>());
  }

  if (shader->HasElement("parameters"))
  {
    auto p = shader->GetElement("parameters");
    if (p->HasElement("rescale"))
      this->dataPtr->rescale = p->Get<float>("rescale");
    if (p->HasElement("bumpScale"))
      this->dataPtr->bumpScale = p->Get<gz::math::Vector2d>("bumpScale");
    if (p->HasElement("bumpSpeed"))
      this->dataPtr->bumpSpeed = p->Get<gz::math::Vector2d>("bumpSpeed");
    if (p->HasElement("hdrMultiplier"))
      this->dataPtr->hdrMultiplier = p->Get<float>("hdrMultiplier");
    if (p->HasElement("fresnelPower"))
      this->dataPtr->fresnelPower = p->Get<float>("fresnelPower");
    if (p->HasElement("roughness"))
      this->dataPtr->roughness = p->Get<float>("roughness");
    if (p->HasElement("foamStrength"))
      this->dataPtr->foamStrength = p->Get<float>("foamStrength");
    if (p->HasElement("foamThreshold"))
      this->dataPtr->foamThreshold = p->Get<float>("foamThreshold");
    if (p->HasElement("shallowColor"))
      this->dataPtr->shallowColor = p->Get<gz::math::Color>("shallowColor");
    if (p->HasElement("deepColor"))
      this->dataPtr->deepColor = p->Get<gz::math::Color>("deepColor");
  }

  if (sdf->HasElement("textures"))
  {
    auto t = sdf->GetElement("textures");
    if (t->HasElement("bumpMap"))
      this->dataPtr->bumpMapPath = resolve(t->Get<std::string>("bumpMap"));
    if (t->HasElement("cubeMap"))
      this->dataPtr->cubeMapPath = resolve(t->Get<std::string>("cubeMap"));
  }

  // Tile instancing radius: render the water mesh at
  // (2·radius+1)² offsets around the central visual. 0 disables.
  if (sdf->HasElement("tiles_radius"))
    this->dataPtr->tilesRadius = sdf->Get<int>("tiles_radius");
  if (sdf->HasElement("tile_mesh_size"))
    this->dataPtr->tileMeshSize = sdf->Get<double>("tile_mesh_size");

  // Connect to render-thread events.
  this->dataPtr->sceneUpdateConn =
    _eventMgr.Connect<events::SceneUpdate>(
      std::bind(&Implementation::OnSceneUpdate, this->dataPtr.get()));
  this->dataPtr->teardownConn =
    _eventMgr.Connect<events::RenderTeardown>(
      std::bind(&Implementation::OnRenderTeardown, this->dataPtr.get()));

  // Subscribe to the wavefield topic. SceneBroadcaster doesn't replicate
  // the Wavefield ECM component to the GUI process, so the GUI-side plugin
  // gets parameters via this topic instead. The Waves system publishes on
  // it from Configure.
}

void WaterVisual::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  GZ_PROFILE("WaterVisual::PreUpdate");

  if (!this->dataPtr->active)
    return;

  // Snapshot sim time for the render thread.
  const float t = std::chrono::duration<float>(_info.simTime).count();

  const Entity worldEnt = worldEntity(_ecm);
  const auto *wfComp =
    (worldEnt != kNullEntity)
      ? _ecm.Component<components::Wavefield>(worldEnt)
      : nullptr;


  std::lock_guard<std::mutex> lock(this->dataPtr->mutex_);
  this->dataPtr->currentSimTime = t;
  if (!wfComp)
  {
    // ECM component is absent on the GUI side (SceneBroadcaster doesn't
    // replicate it). The OnWavefieldMsg topic callback is the backup
    // channel; don't touch haveWavefield here.
    return;
  }
  const auto &data = wfComp->Data();

  // Dispatch on backend type. The FFT path samples a heightmap texture
  // uploaded each frame; the Gerstner path uploads per-component vec3
  // uniforms.
  if (auto fft = std::dynamic_pointer_cast<waves::FFTWaveSimulation>(
        data.simulation))
  {
    if (!this->dataPtr->haveWavefield)
    {
      gzmsg << "[WaterVisual] Wavefield component found (algorithm=fft, "
            << "tile=" << fft->TileSizeMeters() << " m, "
            << "grid=" << fft->GridSize() << ", generation="
            << data.generation << ")" << std::endl;
    }
    this->dataPtr->useFft = true;
    this->dataPtr->fftSim = fft;
    // NOTE: the visual's update period is intentionally NOT synced to
    // the server's <update_rate>. The two rates do different jobs:
    // the server only needs Update for buoyancy queries (15 Hz is
    // fine — water at sea evolves slowly), but the displayed surface
    // needs ≥30 Hz refresh to avoid the stroboscopic stagger that
    // the eye reads as "waves moving slower". An earlier attempt to
    // sync them made waves visually drag at update_rate<30.
    // Slope/chop-deriv grids are only consumed when the VS reads them
    // (useSlopeMap=1). The GPU-FFT path and the Encino path both run
    // with useSlopeMap=0 (finite-diff normals in the VS), so skipping
    // the 5 derivative IFFTs cuts ~60% off each Update on Phillips.
    // Encino's own Update branch already bypasses them — flag is a
    // no-op there but harmless.
    const char *gpuEnv = std::getenv("GZ_WAVES_GPU_FFT");
    const char *stage2 = std::getenv("GZ_WAVES_GPU_FFT_STAGE2");
    const bool gpuPath = (gpuEnv && std::string(gpuEnv) == "1") ||
                         (stage2 && std::string(stage2) == "1");
    fft->SetComputeDerivatives(!gpuPath && !fft->UseEncino());
    this->dataPtr->cachedTileSize = static_cast<float>(fft->TileSizeMeters());
    this->dataPtr->cachedGridSize = static_cast<int>(fft->GridSize());
    this->dataPtr->cachedTau = static_cast<float>(data.params.tau);
    this->dataPtr->cachedChopFactor =
        static_cast<float>(data.params.choppiness);
    this->dataPtr->haveWavefield = true;
    this->dataPtr->cachedGeneration = data.generation;
    return;
  }

  const auto *gerstner =
    dynamic_cast<const waves::GerstnerWaveSimulation *>(data.simulation.get());
  if (!gerstner)
  {
    if (this->dataPtr->haveWavefield)
      gzwarn << "[WaterVisual] backend '" << data.algorithm
             << "' has no shader path yet" << std::endl;
    this->dataPtr->haveWavefield = false;
    return;
  }

  if (!this->dataPtr->haveWavefield)
  {
    gzmsg << "[WaterVisual] Wavefield component found (algorithm="
          << data.algorithm
          << ", N=" << gerstner->Amplitudes().size()
          << ", generation=" << data.generation << ")" << std::endl;
  }
  this->dataPtr->haveWavefield = true;
  if (data.generation == this->dataPtr->cachedGeneration)
    return;

  this->dataPtr->cachedGeneration = data.generation;
  this->dataPtr->cachedTau = static_cast<float>(gerstner->Tau());
  this->dataPtr->cachedNwaves =
    static_cast<int>(gerstner->Amplitudes().size());
  this->dataPtr->cachedAmplitudes.assign(
    gerstner->Amplitudes().begin(), gerstner->Amplitudes().end());
  this->dataPtr->cachedWavenumbers.assign(
    gerstner->Wavenumbers().begin(), gerstner->Wavenumbers().end());
  this->dataPtr->cachedOmegas.assign(
    gerstner->AngularFrequencies().begin(),
    gerstner->AngularFrequencies().end());
  this->dataPtr->cachedSteepnesses.assign(
    gerstner->Steepnesses().begin(), gerstner->Steepnesses().end());
  this->dataPtr->cachedDirections.clear();
  for (const auto &d : gerstner->Directions())
  {
    this->dataPtr->cachedDirections.emplace_back(
      static_cast<float>(d.X()), static_cast<float>(d.Y()));
  }
}

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::WaterVisual,
              gz::sim::System,
              gz::sim::systems::WaterVisual::ISystemConfigure,
              gz::sim::systems::WaterVisual::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::WaterVisual,
                    "gz::sim::systems::WaterVisual")
