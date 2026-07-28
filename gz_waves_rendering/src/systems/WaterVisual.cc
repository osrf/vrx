/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "WaterVisual.hh"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <list>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
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

#include <gz/msgs/serialized_map.pb.h>
#include <gz/transport/Node.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SourceFilePath.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/sim/Util.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/waves/WaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"

#include "HeightMapTexture.hh"

namespace gz::sim::systems
{

/// \brief Pimpl for WaterVisual: holds the GUI-side wave engine, the rendering
/// material + heightmap texture, and the ECM/scene wiring that drives the
/// per-frame uploads.
class WaterVisual::Implementation
{
  /// \brief Destructor — releases this entity's dedup claim (see `active`).
  public: ~Implementation();

  /// \brief Re-resolve the visual pointer; runs on the render thread.
  public: bool ResolveVisual();

  /// \brief Upload uniforms; runs on the render thread under the cache mutex.
  public: void UploadUniforms();

  /// \brief (Re)create the dynamic heightmap texture at `_gridSize` and bind
  /// it to the material; runs on the render thread under the cache mutex.
  /// Called at material setup and again whenever a runtime reconfigure
  /// changes the wave grid resolution.
  /// \param[in] _gridSize Texture resolution per axis.
  /// \return True when the new texture initialized and bound.
  public: bool CreateHeightMapTexture(std::size_t _gridSize);

  /// \brief Render-thread entry point (SceneUpdate event).
  public: void OnSceneUpdate();

  /// \brief Render-thread entry point (RenderTeardown event). Resets render
  /// resources so the next OnSceneUpdate rebuilds them from scratch.
  public: void OnRenderTeardown();

  /// \brief Async handler for the world `state` service pull (runs on a
  /// transport thread). Deserialises the snapshot in a throwaway ECM and caches
  /// the Wavefield recipe so PreUpdate can seed a GUI that joined at any time.
  public: void OnStateResponse(
      const gz::msgs::SerializedStepMap &_reply, bool _result);

  // Fields are ordered by descending alignment (8-byte handles/strings,
  // then 4-byte scalars, then 1-byte flags) to minimise struct padding
  // (clang-tidy optin.performance.Padding). Logical grouping is preserved
  // within each alignment band, and the cross-thread cache fields — written
  // on the render thread and read back under mutex_ — are tagged inline.

  // ---- 8-byte-aligned: configuration, handles, sync ----
  // Configuration (set once at Configure).
  public: std::string fftVertexShaderUri;   ///< Grid/displacement vertex shader
  public: std::string fragmentShaderUri;    ///< Water surface fragment shader
  public: std::string bumpMapPath;
  public: std::string cubeMapPath;
  public: std::string visualName;
  public: std::string modelPath;
  // Baseline parameters — defaults aligned with asv_wave_sim's reference
  // scene so a fresh setup gets the same visual neighbourhood. Each can
  // still be overridden in the model.sdf <parameters> block.
  public: gz::math::Vector2d bumpScale{64.0, 64.0};
  public: gz::math::Vector2d bumpSpeed{0.01, 0.01};
  public: Entity visualEntity{kNullEntity};

  // Tile instancing. The procedural water mesh is 200m × 200m; to cover the
  // visible horizon we render the same mesh at (2·radius+1)² offset positions
  // (see tilesRadius below), each sharing the central tile's material so they
  // all sample the same heightmap and follow the continuous periodic wavefield.
  public: double tileMeshSize{200.0};
  public: std::vector<gz::rendering::VisualPtr> tileVisuals;

  // Cross-thread cache. mutex_ guards every member tagged "guarded by mutex_"
  // here and in the 4-/1-byte bands below.
  public: std::mutex mutex_;
  public: std::uint64_t cachedGeneration{0};        ///< guarded by mutex_
  public: std::uint64_t lastUploadedGeneration{0};  ///< guarded by mutex_

  // Grid/displacement path state (engine + heightmap; its scalar cache is in
  // the 4-byte band). The engine is this visual's private instance, written by
  // PreUpdate and read by OnSceneUpdate — both under mutex_.
  public: std::shared_ptr<gz::sim::waves::IWaveField> sim;  ///< guarded by mutex_
  public: std::unique_ptr<HeightMapTexture> heightMap;

  // Render-thread state.
  public: gz::rendering::ScenePtr scene;
  public: gz::rendering::VisualPtr visual;
  public: gz::rendering::MaterialPtr material;
  public: gz::common::ConnectionPtr sceneUpdateConn;
  public: gz::common::ConnectionPtr teardownConn;

  /// \brief Transport node for the one-time on-demand `state` pull-on-ready.
  public: gz::transport::Node node;

  /// \brief Recipe lifted from a `state` snapshot, used to seed a GUI that
  /// joined after the server's startup re-broadcast window. Guarded by mutex_.
  public: std::optional<gz::sim::waves::WavefieldData> pulled;

  // ---- 4-byte-aligned: scalar parameters + scalar cache ----
  public: float rescale{0.125f};
  // Strict asv_wave_sim defaults — match their literal values so a visual
  // comparison reflects the simulation/normal-computation difference, not
  // parameter divergence.
  public: float hdrMultiplier{0.4f};
  public: float fresnelPower{5.0f};
  public: float roughness{0.0f};
  public: float foamStrength{0.7f};       ///< Foam blend toward white (grid foam)
  public: float foamThreshold{0.25f};     ///< Foam ramp half-width (grid foam)
  public: float cachedTau{2.0f};          ///< guarded by mutex_
  public: float currentSimTime{0.0f};     ///< guarded by mutex_
  public: float cachedTileSize{200.0f};   ///< grid cache, guarded by mutex_
  public: int tilesRadius{2};             ///< (2·r+1)² tiles; 0 disables
  public: int cachedGridSize{128};        ///< grid cache, guarded by mutex_
  // asv_wave_sim's exact default colours.
  public: gz::math::Color shallowColor{0.0f, 0.1f, 0.3f, 1.0f};
  public: gz::math::Color deepColor{0.0f, 0.05f, 0.2f, 1.0f};

  // ---- 1-byte-aligned: flags ----
  public: bool haveWavefield{false};  ///< guarded by mutex_
  /// \brief gz-sim's `GuiRunner` loads this system *twice* for the same
  /// entity (once via the model SDF, once via the visual SDF — known
  /// upstream issue mentioned in asv_wave_sim#177). The two instances
  /// race on SetMaterial / heightmap creation. Dedupe at our level: the
  /// first `Configure` for an entity claims it; the second becomes a
  /// no-op so the render thread only does one round of material setup.
  public: bool active{true};

  /// \brief Whether the one-time `state` pull has been kicked off (GUI thread).
  public: bool triedPull{false};

  /// \brief Whether the one-shot "first Field upload" message was logged.
  /// Guarded by mutex_ (only touched in OnSceneUpdate).
  public: bool firstUploadLogged{false};
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

//////////////////////////////////////////////////
WaterVisual::Implementation::~Implementation()
{
  // Release the dedup claim so a later WaterVisual for the same entity (e.g.
  // after a model reload) can become active again. Only the instance that
  // actually claimed the entity (active) erases it; the deduped no-op instance
  // leaves the live claim alone.
  if (this->active && this->visualEntity != kNullEntity)
  {
    const std::lock_guard<std::mutex> lock(VisualClaimMutex());
    VisualClaimSet().erase(this->visualEntity);
  }
}

//////////////////////////////////////////////////
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

  if (!this->material)
  {
    // Defer material creation until the wavefield component has been
    // parsed (haveWavefield). The visual just stays unrendered for a
    // few ticks at startup, which is harmless.
    if (!this->haveWavefield)
      return false;

    const std::string &vsUri = this->fftVertexShaderUri;
    gzmsg << "[WaterVisual] creating material with shaders" << '\n';

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
      gzerr << "[WaterVisual] failed to attach material to visual" << '\n';
      return false;
    }
    // Critical: upload all uniforms NOW, in the same render-thread call as
    // material creation. Ogre Next compiles/caches the material the first
    // time it's used; uniforms must be present at that point.
    this->UploadUniforms();

    // The material needs a dynamic heightmap texture bound to it.
    this->CreateHeightMapTexture(
        static_cast<std::size_t>(this->cachedGridSize));

    // Spawn tile copies. Each one shares the central material (so the
    // dynamic heightmap binding applies to all of them) and the same
    // mesh resource (Ogre caches the COLLADA load by URI). The
    // wavefield is periodic in world XY, so neighbour tiles continue
    // the same wave pattern without seams.
    if (this->tilesRadius > 0 && !this->modelPath.empty())
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
            << '\n';
      for (int j = -r; j <= r; ++j)
      {
        for (int i = -r; i <= r; ++i)
        {
          if (i == 0 && j == 0) continue;  // center is the existing visual
          const std::string tileName =
              "water_tile_" + std::to_string(this->visualEntity) + "_" +
              std::to_string(i) + "_" + std::to_string(j);
          const gz::rendering::VisualPtr tile = this->scene->CreateVisual(tileName);
          if (!tile) continue;
          const gz::rendering::MeshDescriptor desc(meshPath);
          const gz::rendering::MeshPtr mesh = this->scene->CreateMesh(desc);
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
  return this->material != nullptr;
}

//////////////////////////////////////////////////
void WaterVisual::Implementation::UploadUniforms()
{
  if (!this->material)
    return;
  gzmsg << "[WaterVisual] uploading uniforms: tileSize=" << this->cachedTileSize
        << " gridSize=" << this->cachedGridSize << " tau=" << this->cachedTau
        << '\n';

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

  // Grid-path uniforms. The material always uses the grid vertex shader,
  // so these bindings are always valid.
  {
    // The grid vertex shader uses world_matrix to compute world-space XY
    // for the periodic heightmap sample, so tile instances at
    // different world offsets each render their own piece of the
    // continuous wavefield (rather than each tile showing the same
    // patch in local model space).
    (*vsParams)["world_matrix"] = 1;

    // Grid shader: heightmap texture (bound separately by HeightMapTexture)
    // plus the geometry of the periodic tile. Dx/Dy in the heightmap are the
    // final displacement (WaveField2D contract) — no chop factor here.
    (*vsParams)["tileSize"]   = this->cachedTileSize;
    (*vsParams)["gridSize"]   = this->cachedGridSize;
  }

  // Fragment shader: colours + lighting params + textures.
  (*fsParams)["hdrMultiplier"] = this->hdrMultiplier;
  (*fsParams)["fresnelPower"]  = this->fresnelPower;
  (*fsParams)["roughness"]     = this->roughness;
  // Foam: the grid path packs a folding metric into the heightmap's alpha
  // channel, so the FS reads it directly (one sample) instead of
  // finite-differencing the displacement. foamThreshold is the smoothstep
  // half-width around J = 0 (Encino's MinE; the Phillips path leaves it ≈1).
  (*fsParams)["tileSize"]      = this->cachedTileSize;
  // Foam is engine-specific. The analytic Gerstner engine's Jacobian
  // determinant barely leaves 1.0, so it carries no usable folding signal and
  // foam is unsupported for it (foamStrength=0 makes the FS skip the whole foam
  // path). The FFT/Encino engine writes a real min-eigenvalue metric and keeps
  // foam. (`sim` is this visual's private, render-thread engine.)
  const bool gerstner = this->sim && this->sim->Kind() == "gerstner";
  (*fsParams)["foamStrength"]  = gerstner ? 0.0f : this->foamStrength;
  (*fsParams)["foamThreshold"] = this->foamThreshold;
  (*fsParams)["useFoamMap"]    = 1;
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

//////////////////////////////////////////////////
bool WaterVisual::Implementation::CreateHeightMapTexture(std::size_t _gridSize)
{
  // Destroy any previous instance first so its GPU texture frees up
  // before we ask Ogre Next to create the new one. Otherwise the new
  // `createOrRetrieveTexture` returns the still-Resident texture and
  // `setResolution` asserts (`mResidencyStatus == OnStorage`).
  this->heightMap.reset();
  // Use a process-unique texture name so we never collide with a
  // stale entry in `TextureGpuManager` left behind by a previous
  // engine teardown/reload cycle (or by the texture this one replaces).
  static std::atomic<std::uint64_t> heightMapCounter{0};
  const auto seq = heightMapCounter.fetch_add(1);
  this->heightMap = std::make_unique<HeightMapTexture>(
    this->scene, this->material, _gridSize,
    "wavefield_heightmap_" + std::to_string(this->visualEntity) +
        "_" + std::to_string(seq));
  if (!this->heightMap->Ready())
  {
    gzerr << "[WaterVisual] heightmap texture failed to initialize" << '\n';
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
void WaterVisual::Implementation::OnSceneUpdate()
{
  if (!this->active)
    return;
  if (this->visualName.empty())
    return;

  // Hold the cache mutex across ResolveVisual too: its first-pass material
  // setup calls UploadUniforms and reads the cross-thread cache (haveWavefield,
  // cachedGridSize, …) that PreUpdate writes on the ECM thread. Locking only
  // afterwards left that first upload racing PreUpdate. The lock can't move
  // inside UploadUniforms (it's also called below under this same lock, and
  // std::mutex isn't recursive), so it lives here.
  const std::lock_guard<std::mutex> lock(this->mutex_);
  if (!this->ResolveVisual())
    return;

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

  // Drive the wave field's own Update each frame (the GUI process holds its
  // own instance, rebuilt from the replicated parameters) and upload the
  // resulting grid as the heightmap the surface shader samples. One path for
  // every engine -- they all expose the same WaveField2D via Field().
  // This runs under mutex_ on purpose: `sim` is shared with PreUpdate, so the
  // Update()/Field() pair (an IFFT + readback for grid engines) and the GPU
  // upload must be serialised against the ECM thread rebuilding the engine.
  if (this->sim && this->heightMap && this->heightMap->Ready())
  {
    this->sim->Update(static_cast<double>(this->currentSimTime));
    const auto *f = this->sim->Field();
    bool ok = false;
    if (f && f->n > 0 && f->dz)
    {
      // A runtime set_parameters reconfigure can change grid_size. The GPU
      // texture is sized at creation, so when the rebuilt engine's grid no
      // longer matches, recreate it here (render thread, under mutex_) —
      // otherwise every subsequent Upload fails its size check and the
      // surface freezes at the old field.
      if (f->n != this->heightMap->GridSize())
      {
        gzmsg << "[WaterVisual] wave grid changed "
              << this->heightMap->GridSize() << " -> " << f->n
              << "; recreating the heightmap texture" << '\n';
        if (!this->CreateHeightMapTexture(f->n))
          return;
      }
      // Hand the raw column-major WaveField2D grids straight to the
      // heightmap/bridge (no reflow; the bridge's texel pack preserves the
      // physics orientation). Null displacement channels (dx/dy) upload as
      // zeros; a null folding metric leaves the alpha channel at 0. The FS
      // reads alpha as foam when useFoamMap=1 — the grid engines that
      // compute folding fill it, the analytic ones don't.
      ok = this->heightMap->Upload(f->dz, f->dx, f->dy, f->foam, f->n);
      if (ok && !this->firstUploadLogged)
      {
        this->firstUploadLogged = true;
        gzmsg << "[WaterVisual] first Field upload: grid=" << f->n
              << " tile=" << f->tile << " m" << '\n';
      }
    }
  }
}

//////////////////////////////////////////////////
void WaterVisual::Implementation::OnRenderTeardown()
{
  // Destroy the heightmap texture BEFORE the scene/material handles
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
  const std::lock_guard<std::mutex> lock(this->mutex_);
  this->lastUploadedGeneration = 0;
}

//////////////////////////////////////////////////
WaterVisual::WaterVisual()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

WaterVisual::~WaterVisual() = default;

//////////////////////////////////////////////////
void WaterVisual::Configure(
  const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &_eventMgr)
{
  GZ_PROFILE("WaterVisual::Configure");
  // const_cast only to satisfy sdf::Element::Get/GetElement (non-const); the
  // SDF is read, never mutated.
  auto sdf = std::const_pointer_cast<sdf::Element>(_sdf);

  if (!sdf->HasElement("shader"))
  {
    gzerr << "[WaterVisual] <shader> element is required" << '\n';
    return;
  }

  this->dataPtr->visualEntity = _entity;
  if (auto *name = _ecm.Component<components::Name>(_entity))
    this->dataPtr->visualName = name->Data();

  // Dedupe: only the first instance for this entity does real work.
  {
    const std::lock_guard<std::mutex> lock(VisualClaimMutex());
    auto &claims = VisualClaimSet();
    if (claims.count(_entity))
    {
      gzmsg << "[WaterVisual] entity " << _entity
            << " already claimed by another WaterVisual instance — "
            << "this one will be inactive" << '\n';
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
  this->dataPtr->fragmentShaderUri =
    resolve(shader->GetElement("fragment")->Get<std::string>());
  this->dataPtr->fftVertexShaderUri =
    resolve(shader->GetElement("fft_vertex")->Get<std::string>());

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
}

//////////////////////////////////////////////////
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

  // Pull-on-ready: the first tick we run without the replicated component,
  // request the current world state once and seed the recipe from it. The
  // server's startup re-broadcast only blankets its first seconds, so a GUI
  // joining later would otherwise never get the recipe; an on-demand pull
  // covers any join time. Strictly fallback-forward: if the pull yields nothing
  // we still just wait for the component to replicate, exactly as before. Fired
  // outside the lock — the request is non-blocking.
  if (!wfComp && worldEnt != kNullEntity && !this->dataPtr->triedPull)
  {
    this->dataPtr->triedPull = true;
    std::string worldName;
    if (const auto *nameComp = _ecm.Component<components::Name>(worldEnt))
      worldName = nameComp->Data();
    if (!worldName.empty())
    {
      this->dataPtr->node.Request("/world/" + worldName + "/state",
          &Implementation::OnStateResponse, this->dataPtr.get());
    }
  }

  const std::lock_guard<std::mutex> lock(this->dataPtr->mutex_);
  this->dataPtr->currentSimTime = t;

  // Prefer the replicated component — it carries the live generation bumps from
  // runtime set_parameters. Fall back to the pulled snapshot until the
  // component reaches our ECM; the generation counter reconciles the two (once
  // the component shows up with an equal-or-newer generation it takes over).
  const gz::sim::waves::WavefieldData *data =
      wfComp ? &wfComp->Data()
             : (this->dataPtr->pulled ? &*this->dataPtr->pulled : nullptr);
  if (!data)
    return;   // neither replicated nor pulled yet — try again next tick

  if (!this->dataPtr->haveWavefield)
  {
    gzmsg << "[WaterVisual] wavefield recipe acquired (algorithm="
          << data->algorithm << ", generation=" << data->generation
          << (wfComp ? ", via component" : ", via state pull") << ")" << '\n';
  }

  // Build and OWN a private engine instance from the replicated parameters,
  // rebuilt only when the wavefield generation changes. We deliberately do NOT
  // alias data.simulation: that is a process-global engine shared by every
  // consumer (gz-sim's GuiRunner loads this system twice, and the deserialize
  // path drives the same instance on another thread). The per-instance mutex_
  // below cannot serialise a shared instance, so sharing it races
  // OnSceneUpdate's Update()/Field() on the render thread against this thread —
  // the cause of the intermittent PreUpdate segfault. A private instance is
  // touched only by PreUpdate + OnSceneUpdate, both under mutex_, so it is
  // fully serialised. One render path for every engine: OnSceneUpdate pulls
  // the grid via Field().
  if (!this->dataPtr->sim ||
      this->dataPtr->cachedGeneration != data->generation)
  {
    this->dataPtr->sim =
        gz::sim::waves::CreateWaveSimulation(data->algorithm, data->params);
  }
  if (!this->dataPtr->sim)
  {
    this->dataPtr->haveWavefield = false;
    return;
  }
  if (const auto *f = this->dataPtr->sim->Field())
  {
    this->dataPtr->cachedTileSize = static_cast<float>(f->tile);
    this->dataPtr->cachedGridSize = static_cast<int>(f->n);
  }
  this->dataPtr->cachedTau = static_cast<float>(data->params.tau);
  this->dataPtr->haveWavefield = true;
  this->dataPtr->cachedGeneration = data->generation;
}

//////////////////////////////////////////////////
void WaterVisual::Implementation::OnStateResponse(
    const gz::msgs::SerializedStepMap &_reply, bool _result)
{
  if (!_result)
    return;
  // Deserialise the snapshot into a throwaway ECM — our component type is
  // registered in this process now, so it parses cleanly — and lift out just
  // the Wavefield recipe, without touching the live GUI ECM.
  gz::sim::EntityComponentManager ecm;
  ecm.SetState(_reply.state());
  ecm.Each<components::Wavefield>(
      [this](const Entity &, const components::Wavefield *_wf)
      {
        const std::lock_guard<std::mutex> lock(this->mutex_);
        this->pulled = _wf->Data();
        return false;   // the recipe lives on the single world entity
      });
}

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::WaterVisual,
              gz::sim::System,
              gz::sim::systems::WaterVisual::ISystemConfigure,
              gz::sim::systems::WaterVisual::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::WaterVisual,
                    "gz::sim::systems::WaterVisual")
