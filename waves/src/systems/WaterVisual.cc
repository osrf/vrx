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
#include <list>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Color.hh>
#include <gz/math/Vector2.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Material.hh>
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
  public: std::string bumpMapPath;
  public: std::string cubeMapPath;
  public: float rescale{0.125f};
  public: gz::math::Vector2d bumpScale{75.0, 75.0};
  public: gz::math::Vector2d bumpSpeed{0.01, 0.0};
  public: float hdrMultiplier{0.4f};
  public: float fresnelPower{5.0f};
  public: gz::math::Color shallowColor{0.0f, 0.1f, 0.2f, 1.0f};
  public: gz::math::Color deepColor{0.0f, 0.05f, 0.2f, 1.0f};
  public: std::string visualName;
  public: Entity visualEntity{kNullEntity};
  public: std::string modelPath;

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
};

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
    const std::string &vsUri = this->useFft && !this->fftVertexShaderUri.empty()
      ? this->fftVertexShaderUri
      : this->vertexShaderUri;
    gzmsg << "[WaterVisual] creating material with shaders ("
          << (this->useFft ? "fft" : "gerstner") << ")" << std::endl;
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
  }
  return this->material != nullptr;
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

  if (this->useFft)
  {
    // FFT shader: heightmap texture (bound separately by HeightMapTexture)
    // plus the geometry of the periodic tile and the choppiness factor.
    (*vsParams)["tileSize"]   = this->cachedTileSize;
    (*vsParams)["gridSize"]   = this->cachedGridSize;
    (*vsParams)["chopFactor"] = this->cachedChopFactor;
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

  // Fragment shader: colors + lighting params + textures.
  (*fsParams)["hdrMultiplier"] = this->hdrMultiplier;
  (*fsParams)["fresnelPower"] = this->fresnelPower;
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

  this->lastUploadedGeneration = this->cachedGeneration;
}

void WaterVisual::Implementation::OnSceneUpdate()
{
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

  // FFT visual path: re-evaluate the height field for the current sim time
  // on the GUI side, then upload it to the GPU heightmap texture. The GUI
  // and server own independent FFTWaveSimulation instances seeded from the
  // same `<seed>` in SDF, so the two grids agree bit-for-bit at each t.
  if (this->useFft && this->fftSim && this->heightMap &&
      this->heightMap->Ready())
  {
    this->fftSim->Update(static_cast<double>(this->currentSimTime));
    const bool ok = this->heightMap->Upload(this->fftSim->HeightGrid(),
                                            this->fftSim->DispXGrid(),
                                            this->fftSim->DispYGrid());
    // One-shot diagnostic on the very first successful upload so we can
    // see the actual amplitudes the GPU is sampling. Helps distinguish
    // "upload silently failing" from "Phillips spectrum is tiny".
    static bool logged = false;
    if (ok && !logged)
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
