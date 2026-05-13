/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "Ogre2HeightMapBridge.hh"

#include <cmath>
#include <cstring>
#include <limits>
#include <dlfcn.h>

#include <gz/common/Console.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ogre2/Ogre2Material.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>

#include <filesystem>

#include <OgreAsyncTextureTicket.h>
#include <OgreCommon.h>
#include <OgreDescriptorSetTexture.h>
#include <OgreHlmsCompute.h>
#include <OgreHlmsComputeJob.h>
#include <OgreHlmsManager.h>
#include <OgreHlmsSamplerblock.h>
#include <OgreItem.h>
#include <OgreMesh2.h>
#include <OgreMeshManager.h>
#include <OgreMeshManager2.h>
#include <OgrePass.h>
#include <OgrePixelFormatGpu.h>
#include <OgrePixelFormatGpuUtils.h>
#include <OgreRenderSystem.h>
#include <OgreResourceGroupManager.h>
#include <OgreResourceTransition.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreStagingTexture.h>
#include <OgreSubItem.h>
#include <OgreTechnique.h>
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>
#include <OgreTextureUnitState.h>
#include <Hlms/Pbs/OgreHlmsPbs.h>
#include <Hlms/Pbs/OgreHlmsPbsDatablock.h>
#include <Vao/OgreConstBufferPacked.h>
#include <Vao/OgreVaoManager.h>

namespace
{
  // std140-packed uniform layout matching the GLSL `Params` block.
  // Must stay in sync with shaders/compute/*.glsl.
  struct alignas(16) ComputeParams
  {
    float t;
    float tileSize;
    int   gridSize;
    float _pad;
  };
  static_assert(sizeof(ComputeParams) == 16,
                "ComputeParams must be a single std140 vec4 block");

  struct HeightMap
  {
    Ogre::TextureGpu *texture{nullptr};
    Ogre::TextureGpuManager *manager{nullptr};
    // Persistent staging texture, reused on every Upload(). Acquiring a
    // fresh one per frame from Ogre's pool was causing rapid pool growth
    // and "Texture memory budget exceeded. Stalling GPU." stalls that
    // visibly froze the GUI for a couple of minutes.
    Ogre::StagingTexture *staging{nullptr};
    Ogre::HlmsSamplerblock samplerblock;
    std::size_t gridSize{0};
    bool ready{false};

    // GPU-FFT compute pipeline state. Initialised lazily on the first
    // `compute_dispatch` call (so the CPU upload path doesn't pay any
    // setup cost when GPU-FFT is disabled).
    Ogre::SceneManager     *sceneManager{nullptr};
    Ogre::HlmsCompute      *hlmsCompute{nullptr};
    Ogre::HlmsComputeJob   *computeJob{nullptr};
    Ogre::ConstBufferPacked *paramsBuffer{nullptr};
    std::string             computeShaderName;  // basename, not full path

    // Stage 6 procedural-Item state (HlmsPbs path).
    Ogre::SceneNode        *pbsNode{nullptr};
    Ogre::Item             *pbsItem{nullptr};
    Ogre::MeshPtr           pbsMeshV2;
    Ogre::v1::MeshPtr       pbsMeshV1;
    Ogre::HlmsPbsDatablock *pbsDatablock{nullptr};
    std::string             pbsMeshNameV1;
    std::string             pbsMeshNameV2;
    std::string             pbsDatablockName;

    // Stage 2 (Phillips on GPU): persistent spectrum textures and the
    // evolve compute job that reads from them each frame.
    // ω(k) is computed in-shader from gridSize + tileSize, so no
    // omegaTex is needed.
    //
    // Two evolve jobs run per frame:
    //   - evolveJob writes hktTex = (η.re, η.im, Dx.re, Dx.im).
    //     The radix-2 butterfly carries both complex signals so the
    //     IFFT cost is one pipeline.
    //   - evolveDyJob writes hktTexDy = (Dy.re, Dy.im, 0, 0). A third
    //     complex signal won't fit in RGBA32F so Dy gets its own
    //     pipeline.
    Ogre::TextureGpu       *h0Tex{nullptr};        // RGBA32F (re, im, re, im)
    Ogre::TextureGpu       *hktTex{nullptr};       // packed (η, Dx)
    Ogre::TextureGpu       *hktTexDy{nullptr};     // Dy alone
    Ogre::HlmsComputeJob   *evolveJob{nullptr};
    Ogre::HlmsComputeJob   *evolveDyJob{nullptr};
    Ogre::ConstBufferPacked *evolveParams{nullptr};
    Ogre::ConstBufferPacked *evolveDyParams{nullptr};
    std::string             evolveShaderName;
    std::string             evolveDyShaderName;
    std::string             h0TexName;
    std::string             hktTexName;
    std::string             hktTexDyName;

    // Stage 3 (GPU IFFT): two ping-pong textures, one bitreverse job and
    // one butterfly job (both reused across all 2·(log2N+1) passes), plus
    // one tiny ConstBufferPacked per pass with the params baked at init.
    // Per-pass params are time-invariant (axis, stage, log2N) so we never
    // touch the buffers after init — no per-frame upload-then-dispatch
    // synchronisation to worry about.
    Ogre::TextureGpu       *ifftBufA{nullptr};
    Ogre::TextureGpu       *ifftBufB{nullptr};
    // Dy IFFT pipeline mirrors the η+Dx one, with its own ping-pong
    // textures and pass jobs.
    Ogre::TextureGpu       *ifftDyBufA{nullptr};
    Ogre::TextureGpu       *ifftDyBufB{nullptr};
    Ogre::TextureGpu       *ifftDyFinalTex{nullptr};
    std::vector<Ogre::HlmsComputeJob *> ifftDyPassJobs;
    std::vector<std::pair<Ogre::TextureGpu *, Ogre::TextureGpu *>>
        ifftDyPassSrcDst;
    /// One HlmsComputeJob per IFFT pass, with stable UAV+texture
    /// bindings at init time. Avoids the binding-cache and barrier
    /// quirks we observed when reusing one job across many dispatches.
    std::vector<Ogre::HlmsComputeJob *> ifftPassJobs;
    /// (src, dst) per pass, for runtime BarrierSolver transitions.
    std::vector<std::pair<Ogre::TextureGpu *, Ogre::TextureGpu *>>
        ifftPassSrcDst;
    std::vector<Ogre::ConstBufferPacked *> ifftParams;
    std::string             bitrevShaderName;
    std::string             butterShaderName;
    // Legacy fields, left for ABI compatibility — no longer used after
    // the per-pass-job refactor. Will be removed.
    Ogre::HlmsComputeJob   *bitrevJob{nullptr};
    Ogre::HlmsComputeJob   *butterJob{nullptr};
    std::string             ifftBufAName;
    std::string             ifftBufBName;
    int                     ifftLog2N{0};
    /// After the last pass, this aliases whichever of bufA/bufB ended
    /// up holding the spatial η output. Exposed for Stage 4's binding.
    Ogre::TextureGpu       *ifftFinalTex{nullptr};

    // Stage 4: gz::rendering Material we're bound to, saved at create
    // time so combine_dispatch can swap its "heightMap"
    // TextureUnitState from the CPU-uploaded `texture` to the assembled
    // `combinedTex`. Cleared on null-material (HlmsPbs) creates.
    Ogre::Material         *ogreMaterial{nullptr};
    bool                    ifftBoundToMaterial{false};

    // Stage 4: combine pass assembles (η, Dx, Dy, _) from the two
    // IFFT outputs into `combinedTex`, which is what the visual
    // material samples once GPU FFT is fully wired up.
    Ogre::TextureGpu       *combinedTex{nullptr};
    Ogre::HlmsComputeJob   *combineEtaDxJob{nullptr};
    Ogre::HlmsComputeJob   *combineDyJob{nullptr};
    std::string             combinedTexName;
    std::string             combineEtaDxShaderName;
    std::string             combineDyShaderName;

    // Diagnostic: a single-pass compute job that overwrites
    // ifftFinalTex with a known sine pattern, bypassing the IFFT. Used
    // to isolate binding/sampling bugs from compute bugs.
    Ogre::HlmsComputeJob   *debugJob{nullptr};
    Ogre::ConstBufferPacked *debugParams{nullptr};
    std::string             debugShaderName;

    // Diagnostic: copies a scaled view of hktTex into ifftFinalTex,
    // bypassing the IFFT. Used to determine whether evolve produces
    // non-zero output.
    Ogre::HlmsComputeJob   *viewHktJob{nullptr};
    Ogre::ConstBufferPacked *viewHktParams{nullptr};
    std::string             viewHktShaderName;
  };

  // std140 layout for debug_view_hkt.glsl's `Params` block.
  struct alignas(16) ViewHktParams
  {
    float scale;
    float _pad0;
    int   gridSize;
    int   _pad1;
  };
  static_assert(sizeof(ViewHktParams) == 16,
                "ViewHktParams must be a vec4");

  // std140 layout for debug_pattern.glsl's `Params` block.
  struct alignas(16) DebugParams
  {
    float t;
    float amplitude;
    int   gridSize;
    int   _pad;
  };
  static_assert(sizeof(DebugParams) == 16, "DebugParams must be a vec4");
}
namespace
{

  // std140-packed uniform layout matching evolve.glsl's `Params` block.
  struct alignas(16) EvolveParams
  {
    float t;
    float tau;
    int   gridSize;
    float tileSize;   // L, in meters
  };
  static_assert(sizeof(EvolveParams) == 16,
                "EvolveParams must be a single std140 vec4 block");

  // std140-packed uniform layouts matching fft_bitreverse.glsl and
  // fft_butterfly.glsl's `Params` blocks. Both fit in a single vec4.
  struct alignas(16) BitrevParams
  {
    int gridSize;
    int axis;
    int log2N;
    int _pad;
  };
  static_assert(sizeof(BitrevParams) == 16, "BitrevParams must be a vec4");

  // Two std140 vec4 slots: (gridSize, stage, axis, invertSign) and
  // (extraScale, _pad, _pad, _pad). extraScale folds Eigen FFT's
  // default 1/N normalisation into the last butterfly stage of each
  // axis so the 2D IFFT total scaling is 1/N², matching the CPU
  // FFTWaveSimulation convention.
  struct alignas(16) ButterParams
  {
    int   gridSize;
    int   stage;
    int   axis;
    float invertSign;
    float extraScale;
    float _pad0;
    float _pad1;
    float _pad2;
  };
  static_assert(sizeof(ButterParams) == 32,
                "ButterParams must occupy two std140 vec4 slots");
}

extern "C"
{

waves_heightmap_t waves_ogre2_heightmap_create(
    void *_scene, void *_material, std::size_t _gridSize, const char *_name)
{
  // `_material` is optional: on the Stage 6 HlmsPbs path the caller
  // passes nullptr because the texture will be bound to an
  // HlmsPbsDatablock slot (added in step 6.1) rather than to a v1
  // material's TextureUnitState.
  if (!_scene || !_name || _gridSize == 0)
    return nullptr;

  auto *scene = static_cast<gz::rendering::Scene *>(_scene);
  auto *material = static_cast<gz::rendering::Material *>(_material);

  auto *ogreScene = dynamic_cast<gz::rendering::Ogre2Scene *>(scene);
  if (!ogreScene)
  {
    gzerr << "[waves_ogre2_heightmap] scene is not an Ogre2Scene" << std::endl;
    return nullptr;
  }
  auto *sceneManager = ogreScene->OgreSceneManager();
  if (!sceneManager)
  {
    gzerr << "[waves_ogre2_heightmap] no Ogre scene manager" << std::endl;
    return nullptr;
  }

  auto *manager =
      sceneManager->getDestinationRenderSystem()->getTextureGpuManager();
  if (!manager)
  {
    gzerr << "[waves_ogre2_heightmap] no Ogre TextureGpuManager" << std::endl;
    return nullptr;
  }

  // Bump Ogre Next's staging-texture budget so multiple streaming textures
  // (heightMap, bumpMap, cubeMap) can coexist without tripping
  // "Texture memory budget exceeded. Stalling GPU." early in scene load.
  // Default in 2.3.x is conservative for a scene with several streaming
  // textures.
  manager->setStagingTextureMaxBudgetBytes(256u * 1024u * 1024u);  // 256 MB

  auto *hm = new HeightMap();
  hm->gridSize = _gridSize;
  hm->manager = manager;
  hm->sceneManager = sceneManager;
  // SaveToSystemRam (matches asv_wave_sim's Ogre2DisplacementMap): Ogre Next
  // retains a CPU-side copy of the texture so it doesn't have to round-trip
  // the GPU to validate residency state during streaming uploads.
  hm->texture = manager->createOrRetrieveTexture(
      _name,
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::ManualTexture,
      Ogre::TextureTypes::Type2D);
  hm->texture->setResolution(
      static_cast<Ogre::uint32>(_gridSize),
      static_cast<Ogre::uint32>(_gridSize));
  // Single mip: we only upload mip 0 each frame, and the engine doesn't
  // auto-generate the rest (would need RenderToTexture + allowsAutoMipmaps,
  // which conflict with our ManualTexture upload contract). With a full
  // mip chain in place but only mip 0 populated, the FS would sample
  // garbage from higher mips at distance — visible as fine dotted lines
  // that drift with the waves. Matches `combinedTex` on the GPU path.
  hm->texture->setNumMipmaps(1u);
  // RGBA32F so we can pack (η, Dx, Dy, α) per texel. Alpha is unused for
  // now (reserved for a Jacobian/foam mask in a future stage).
  hm->texture->setPixelFormat(Ogre::PFG_RGBA32_FLOAT);
  // Don't schedule residency at creation time — asv_wave_sim does it from
  // the upload path so the call is repeated every frame, which seems to
  // keep the engine's state machine "alive" through the streaming flow.

  hm->samplerblock.setFiltering(Ogre::TFO_BILINEAR);
  hm->samplerblock.mU = Ogre::TAM_WRAP;
  hm->samplerblock.mV = Ogre::TAM_WRAP;
  hm->samplerblock.mW = Ogre::TAM_WRAP;

  // Bind to a gz::rendering Material's v1 pass IFF a material was
  // supplied. The Stage 6 HlmsPbs path passes nullptr because the
  // heightmap texture will be slot-bound to an HlmsPbsDatablock later
  // (step 6.1), not to a v1 TextureUnitState.
  if (material)
  {
    auto *ogreMat = dynamic_cast<gz::rendering::Ogre2Material *>(material);
    if (!ogreMat || !ogreMat->Material())
    {
      gzerr << "[waves_ogre2_heightmap] material is not an Ogre2Material"
            << std::endl;
      manager->destroyTexture(hm->texture);
      delete hm;
      return nullptr;
    }
    auto mat = ogreMat->Material();
    hm->ogreMaterial = mat.get();
    auto *pass = mat->getTechnique(0u)->getPass(0u);
    Ogre::TextureUnitState *texUnit = nullptr;
    for (unsigned int i = 0; i < pass->getNumTextureUnitStates(); ++i)
    {
      auto *u = pass->getTextureUnitState(i);
      if (u->getName() == "heightMap")
      {
        texUnit = u;
        break;
      }
    }
    if (!texUnit)
    {
      texUnit = pass->createTextureUnitState();
      texUnit->setName("heightMap");
    }
    texUnit->setTexture(hm->texture);
    texUnit->setTextureCoordSet(0);
    texUnit->setSamplerblock(hm->samplerblock);

    // On OpenGL: explicitly bind the GLSL `sampler2D heightMap` uniform
    // to the texture unit index. Without this, Ogre Next's HlmsLowLevel
    // path can fall back to a slow program-introspection step on first
    // render to figure out the binding. Mirrors asv_wave_sim's pattern.
    const int texIndex =
        static_cast<int>(pass->getTextureUnitStateIndex(texUnit));
    auto ogreParams = pass->getVertexProgramParameters();
    if (ogreParams)
    {
      ogreParams->setNamedConstant("heightMap", &texIndex, 1, 1);
    }
  }
  hm->ready = true;

  // Allocate one persistent staging texture and reuse it on every Upload().
  // Acquiring a fresh staging texture per frame leaks them into Ogre's
  // pool, which trips the engine's "Texture memory budget exceeded" path.
  hm->staging = hm->manager->getStagingTexture(
      static_cast<Ogre::uint32>(_gridSize),
      static_cast<Ogre::uint32>(_gridSize),
      1u, 1u, Ogre::PFG_RGBA32_FLOAT);
  return hm;
}

int waves_ogre2_heightmap_upload(
    waves_heightmap_t _handle, const double *_eta, const double *_dx,
    const double *_dy, int _rows, int _cols)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->ready || !hm->texture || !hm->manager ||
      !_eta || !_dx || !_dy)
    return 0;
  const int N = static_cast<int>(hm->gridSize);
  if (_rows != N || _cols != N)
  {
    gzerr << "[waves_ogre2_heightmap] grid size mismatch (" << _rows << "x"
          << _cols << ", expected " << N << "x" << N << ")" << std::endl;
    return 0;
  }
  if (!hm->staging)
    return 0;

  // Schedule residency on every upload (asv_wave_sim's pattern). No-op
  // once the texture is already Resident, but the repeated call appears
  // to be what keeps the engine's state machine "alive" through the
  // first-frame streaming flow on Jetty + NVIDIA.
  hm->texture->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);

  hm->staging->startMapRegion();
  Ogre::TextureBox box =
      hm->staging->mapRegion(N, N, 1u, 1u, Ogre::PFG_RGBA32_FLOAT);

  // Pack one RGBA32F texel per (row, col): (η, Dx, Dy, 0).
  for (int row = 0; row < N; ++row)
  {
    auto *dst = reinterpret_cast<float *>(box.at(0, row, 0));
    const double *seta = _eta + static_cast<std::size_t>(row) * N;
    const double *sdx  = _dx  + static_cast<std::size_t>(row) * N;
    const double *sdy  = _dy  + static_cast<std::size_t>(row) * N;
    for (int col = 0; col < N; ++col)
    {
      dst[col * 4 + 0] = static_cast<float>(seta[col]);
      dst[col * 4 + 1] = static_cast<float>(sdx[col]);
      dst[col * 4 + 2] = static_cast<float>(sdy[col]);
      dst[col * 4 + 3] = 0.0f;
    }
  }

  hm->staging->stopMapRegion();
  hm->staging->upload(box, hm->texture, 0u, nullptr, nullptr);

  // Tell Ogre Next the texture data has arrived (asv_wave_sim's
  // contract). Without this the engine can keep the texture in an
  // "awaiting data" state on the render thread.
  if (!hm->texture->isDataReady())
    hm->texture->notifyDataIsReady();
  return 1;
}

int waves_ogre2_heightmap_compute_dispatch(
    waves_heightmap_t _handle, const char *_shaderAbsPath,
    float _simTimeS, float _tileSizeM)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->ready || !hm->texture || !hm->sceneManager ||
      !_shaderAbsPath)
    return 0;

  // Schedule residency every dispatch — same reasoning as the upload
  // path. No-op once already Resident.
  hm->texture->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);

  const int N = static_cast<int>(hm->gridSize);

  // First-call init: register the shader directory as a resource
  // location, create the HlmsComputeJob, create the params ConstBuffer,
  // wire up the UAV binding and thread groups.
  if (!hm->computeJob)
  {
    try
    {
      auto &rgMgr = Ogre::ResourceGroupManager::getSingleton();
      const std::filesystem::path absPath(_shaderAbsPath);
      const std::string dir = absPath.parent_path().string();
      // OgreNext's HlmsCompute appends the render-system extension
      // (.glsl on GL, .hlsl on D3D, .metal on Metal) to the source name
      // automatically — passing "foo.glsl" makes it look for
      // "foo.glsl.glsl". So we strip our own .glsl extension before
      // handing the name to createComputeJob, matching the convention
      // used by the bundled `ClearUav_cs.glsl` shader (registered as
      // "ClearUav_cs", not "ClearUav_cs.glsl").
      hm->computeShaderName = absPath.stem().string();

      // Only add the location once per process. addResourceLocation will
      // throw if the same path is re-added; we tolerate that.
      try
      {
        rgMgr.addResourceLocation(
            dir, "FileSystem",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            false);
      }
      catch (const Ogre::Exception &)
      {
        // Already registered — fine.
      }

      auto *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
      hm->hlmsCompute = hlmsManager->getComputeHlms();
      if (!hm->hlmsCompute)
      {
        gzerr << "[waves_ogre2_heightmap] no HlmsCompute available; "
              << "is the engine ogre2?" << std::endl;
        return 0;
      }

      // Process-unique job name so multiple bridge instances can coexist.
      const std::string jobName =
          "WavesGpuFft_" +
          std::to_string(reinterpret_cast<std::uintptr_t>(hm));
      hm->computeJob = hm->hlmsCompute->createComputeJob(
          jobName, jobName, hm->computeShaderName, Ogre::StringVector{});
      if (!hm->computeJob)
      {
        gzerr << "[waves_ogre2_heightmap] createComputeJob failed for "
              << hm->computeShaderName << std::endl;
        return 0;
      }

      // Workgroup is 16x16 in the shader, so we need ceil(N/16) groups
      // per axis. With N=128 that's 8x8 groups (one thread per texel).
      // setThreadsPerGroup is REQUIRED even though our GLSL already
      // declares `layout(local_size_x = 16, ...)` — OgreNext insists on
      // knowing the workgroup size at C++ level (so it can substitute
      // into Metal/HLSL templates and dispatch on those backends).
      // Without it, dispatch throws "Shader or C++ must set
      // threads_per_group_x, ...".
      hm->computeJob->setThreadsPerGroup(16u, 16u, 1u);
      const Ogre::uint32 groups =
          static_cast<Ogre::uint32>((N + 15) / 16);
      hm->computeJob->setNumThreadGroups(groups, groups, 1u);

      // One UAV (the heightmap output) and one const buffer (params).
      hm->computeJob->setNumUavUnits(1u);

      Ogre::DescriptorSetUav::TextureSlot uavSlot =
          Ogre::DescriptorSetUav::TextureSlot::makeEmpty();
      uavSlot.texture = hm->texture;
      uavSlot.access  = Ogre::ResourceAccess::Write;
      uavSlot.pixelFormat = Ogre::PFG_RGBA32_FLOAT;
      hm->computeJob->_setUavTexture(0u, uavSlot);

      // Const buffer for the (t, tileSize, gridSize) uniforms.
      auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
      auto *vaoManager   = renderSystem->getVaoManager();
      hm->paramsBuffer = vaoManager->createConstBuffer(
          sizeof(ComputeParams),
          Ogre::BT_DYNAMIC_PERSISTENT,
          nullptr, false);
      hm->computeJob->setConstBuffer(0u, hm->paramsBuffer);

      gzmsg << "[waves_ogre2_heightmap] compute pipeline initialised: "
            << "shader=" << hm->computeShaderName
            << " thread_groups=" << groups << "x" << groups << "x1"
            << std::endl;
    }
    catch (const Ogre::Exception &e)
    {
      gzerr << "[waves_ogre2_heightmap] compute setup threw: "
            << e.getDescription() << std::endl;
      hm->computeJob = nullptr;
      return 0;
    }
  }

  if (!hm->computeJob || !hm->paramsBuffer)
    return 0;

  // Update the params buffer with the current frame's uniforms and
  // dispatch. Anything throwing from inside Ogre here would otherwise
  // escape the extern "C" boundary and abort the process; swallow it.
  try
  {
    ComputeParams params{};
    params.t        = _simTimeS;
    params.tileSize = _tileSizeM;
    params.gridSize = N;
    params._pad     = 0.0f;
    hm->paramsBuffer->upload(&params, 0u, sizeof(params));

    // Camera arg is null — our shader doesn't reference per-camera state.
    hm->hlmsCompute->dispatch(hm->computeJob, hm->sceneManager, nullptr);

    if (!hm->texture->isDataReady())
      hm->texture->notifyDataIsReady();
  }
  catch (const Ogre::Exception &e)
  {
    static bool logged = false;
    if (!logged)
    {
      logged = true;
      gzerr << "[waves_ogre2_heightmap] compute dispatch threw: "
            << e.getDescription() << " (further failures suppressed)"
            << std::endl;
    }
    return 0;
  }
  return 1;
}

int waves_ogre2_heightmap_create_pbs_visual(
    waves_heightmap_t _handle,
    double _planeSizeM, int _planeSegments,
    double _wx, double _wy, double _wz, const char *_name)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->sceneManager || !_name)
    return 0;

  try
  {
    // Build a v1 plane mesh, import to v2 (same pattern as Path 1).
    hm->pbsMeshNameV1 = std::string(_name) + "_pbsPlaneV1";
    hm->pbsMeshNameV2 = std::string(_name) + "_pbsPlaneV2";
    const Ogre::Real size = static_cast<Ogre::Real>(_planeSizeM);
    const int segs = std::clamp(_planeSegments, 1, 200);  // v1 cap

    hm->pbsMeshV1 = Ogre::v1::MeshManager::getSingleton().createPlane(
        hm->pbsMeshNameV1,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::Plane(Ogre::Vector3::UNIT_Z, 0.0f),
        size, size,
        segs, segs,
        true,                                       // generate normals
        1u, 1.0f, 1.0f, Ogre::Vector3::UNIT_Y,
        Ogre::v1::HardwareBuffer::HBU_STATIC,
        Ogre::v1::HardwareBuffer::HBU_STATIC);

    hm->pbsMeshV2 = Ogre::MeshManager::getSingleton().createByImportingV1(
        hm->pbsMeshNameV2,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        hm->pbsMeshV1.get(),
        true, true, true);

    hm->pbsItem = hm->sceneManager->createItem(
        hm->pbsMeshV2, Ogre::SceneMemoryMgrTypes::SCENE_DYNAMIC);

    // Create an HlmsPbsDatablock with sensible "deep water" defaults.
    // Step 6.0: no custom shader piece, no heightmap binding — we're
    // checking whether HlmsPbs alone avoids the 2-minute load stall.
    // Step 6.1 will add vertex displacement via a custom piece.
    auto *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
    auto *hlmsPbs = static_cast<Ogre::HlmsPbs *>(
        hlmsManager->getHlms(Ogre::HLMS_PBS));
    if (!hlmsPbs)
    {
      gzerr << "[waves_ogre2_heightmap] HLMS_PBS unavailable; "
            << "cannot create PBS datablock" << std::endl;
      hm->sceneManager->destroyItem(hm->pbsItem);
      hm->pbsItem = nullptr;
      return 0;
    }

    hm->pbsDatablockName = std::string(_name) + "_pbsDb";
    hm->pbsDatablock = static_cast<Ogre::HlmsPbsDatablock *>(
        hlmsPbs->createDatablock(
            hm->pbsDatablockName,
            hm->pbsDatablockName,
            Ogre::HlmsMacroblock{},
            Ogre::HlmsBlendblock{},
            Ogre::HlmsParamVec{}));
    // Deep ocean blue, somewhat glossy. Tuned to roughly match the
    // current fragment shader's deepColor; full PBS reflection control
    // comes back in step 6.1+. Don't call setMetalness — it asserts in
    // SpecularWorkflow (the default), and 0 is the implicit default
    // anyway.
    hm->pbsDatablock->setDiffuse(Ogre::Vector3(0.0f, 0.10f, 0.25f));
    hm->pbsDatablock->setRoughness(0.10f);

    hm->pbsItem->getSubItem(0u)->setDatablock(hm->pbsDatablock);
    hm->pbsItem->setCastShadows(false);

    hm->pbsNode = hm->sceneManager->getRootSceneNode(
        Ogre::SceneMemoryMgrTypes::SCENE_DYNAMIC)->createChildSceneNode(
            Ogre::SceneMemoryMgrTypes::SCENE_DYNAMIC);
    hm->pbsNode->setPosition(static_cast<Ogre::Real>(_wx),
                             static_cast<Ogre::Real>(_wy),
                             static_cast<Ogre::Real>(_wz));
    hm->pbsNode->attachObject(hm->pbsItem);

    gzmsg << "[waves_ogre2_heightmap] PBS visual built: mesh="
          << hm->pbsMeshNameV2 << " size=" << size << "m segs=" << segs
          << " datablock=" << hm->pbsDatablockName << std::endl;
    return 1;
  }
  catch (const Ogre::Exception &e)
  {
    gzerr << "[waves_ogre2_heightmap] PBS visual setup threw: "
          << e.getDescription() << std::endl;
    return 0;
  }
}

namespace
{
  // Helper: create a manual TextureGpu of the given pixel format at
  // gridSize × gridSize, ready for compute-shader UAV use.
  //
  // For manually-created (not file-backed) textures, OgreNext's docs
  // are explicit: use `_transitionTo` + `_setNextResidencyStatus`
  // directly — `scheduleTransitionTo` is async and intended for the
  // file-loading path. If we use `scheduleTransitionTo` here, the
  // residency transition may still be in flight when our subsequent
  // staging-texture upload fires; the upload then silently fails and
  // the texture stays zero. (This was the root cause of "view-hkt
  // shows flat" in early Stage 4 testing: spectrum upload was a
  // no-op.)
  Ogre::TextureGpu *MakeSpectrumTexture(
      Ogre::TextureGpuManager *manager,
      const std::string &name,
      std::size_t gridSize,
      Ogre::PixelFormatGpu format)
  {
    Ogre::TextureGpu *tex = manager->createOrRetrieveTexture(
        name,
        Ogre::GpuPageOutStrategy::SaveToSystemRam,
        Ogre::TextureFlags::ManualTexture | Ogre::TextureFlags::Uav,
        Ogre::TextureTypes::Type2D);
    tex->setResolution(static_cast<Ogre::uint32>(gridSize),
                       static_cast<Ogre::uint32>(gridSize));
    tex->setNumMipmaps(1u);  // spectrum textures aren't mipmapped
    tex->setPixelFormat(format);
    tex->_setNextResidencyStatus(Ogre::GpuResidency::Resident);
    tex->_transitionTo(Ogre::GpuResidency::Resident, nullptr);
    return tex;
  }
}

int waves_ogre2_heightmap_upload_spectrum(
    waves_heightmap_t _handle,
    const double *_h0Re, const double *_h0Im,
    const double *_h0ConjRe, const double *_h0ConjIm,
    int _gridSize)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->manager || !hm->sceneManager || _gridSize <= 0)
    return 0;
  if (!_h0Re || !_h0Im || !_h0ConjRe || !_h0ConjIm)
    return 0;
  if (static_cast<std::size_t>(_gridSize) != hm->gridSize)
  {
    gzerr << "[waves_ogre2_heightmap] upload_spectrum size mismatch ("
          << _gridSize << " vs handle's " << hm->gridSize << ")"
          << std::endl;
    return 0;
  }
  if (hm->h0Tex)
    return 1;  // already uploaded

  try
  {
    const std::string base =
        "WavesSpec_" +
        std::to_string(reinterpret_cast<std::uintptr_t>(hm));
    hm->h0TexName    = base + "_h0";
    hm->hktTexName   = base + "_hkt";
    hm->hktTexDyName = base + "_hktDy";

    hm->h0Tex    = MakeSpectrumTexture(hm->manager, hm->h0TexName,
                                        hm->gridSize,
                                        Ogre::PFG_RGBA32_FLOAT);
    hm->hktTex   = MakeSpectrumTexture(hm->manager, hm->hktTexName,
                                        hm->gridSize,
                                        Ogre::PFG_RGBA32_FLOAT);
    hm->hktTexDy = MakeSpectrumTexture(hm->manager, hm->hktTexDyName,
                                        hm->gridSize,
                                        Ogre::PFG_RGBA32_FLOAT);

    // Pack spectrum into RGBA32F via a staging texture.
    auto upload_rgba = [&](Ogre::TextureGpu *dst,
                           const double *re0, const double *im0,
                           const double *re1, const double *im1)
    {
      Ogre::StagingTexture *staging = hm->manager->getStagingTexture(
          static_cast<Ogre::uint32>(hm->gridSize),
          static_cast<Ogre::uint32>(hm->gridSize),
          1u, 1u, Ogre::PFG_RGBA32_FLOAT);
      staging->startMapRegion();
      Ogre::TextureBox box = staging->mapRegion(
          static_cast<Ogre::uint32>(hm->gridSize),
          static_cast<Ogre::uint32>(hm->gridSize),
          1u, 1u, Ogre::PFG_RGBA32_FLOAT);
      const int N = static_cast<int>(hm->gridSize);
      for (int row = 0; row < N; ++row)
      {
        auto *outRow = reinterpret_cast<float *>(box.at(0, row, 0));
        for (int col = 0; col < N; ++col)
        {
          const std::size_t idx =
              static_cast<std::size_t>(row) * N + col;
          outRow[col * 4 + 0] = static_cast<float>(re0[idx]);
          outRow[col * 4 + 1] = static_cast<float>(im0[idx]);
          outRow[col * 4 + 2] = static_cast<float>(re1[idx]);
          outRow[col * 4 + 3] = static_cast<float>(im1[idx]);
        }
      }
      staging->stopMapRegion();
      staging->upload(box, dst, 0u, nullptr, nullptr);
      hm->manager->removeStagingTexture(staging);
      if (!dst->isDataReady())
        dst->notifyDataIsReady();
    };

    upload_rgba(hm->h0Tex, _h0Re, _h0Im, _h0ConjRe, _h0ConjIm);
    gzmsg << "[waves_ogre2_heightmap] spectrum uploaded ("
          << hm->gridSize << "×" << hm->gridSize << " h0)"
          << std::endl;
    return 1;
  }
  catch (const Ogre::Exception &e)
  {
    gzerr << "[waves_ogre2_heightmap] upload_spectrum threw: "
          << e.getDescription() << std::endl;
    return 0;
  }
}

int waves_ogre2_heightmap_evolve_dispatch(
    waves_heightmap_t _handle, const char *_shaderAbsPath,
    float _simTimeS, float _tauS, float _tileSizeM)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->h0Tex || !hm->hktTex ||
      !hm->sceneManager || !_shaderAbsPath)
    return 0;

  // Make sure the spectrum textures are addressable each frame
  // (same defensive pattern as the compute_dispatch path).
  hm->h0Tex->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);
  hm->hktTex->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);

  // Lazy init of the evolve compute job (mirrors compute_dispatch).
  if (!hm->evolveJob)
  {
    try
    {
      auto &rgMgr = Ogre::ResourceGroupManager::getSingleton();
      const std::filesystem::path absPath(_shaderAbsPath);
      const std::string dir = absPath.parent_path().string();
      hm->evolveShaderName = absPath.stem().string();
      try
      {
        rgMgr.addResourceLocation(
            dir, "FileSystem",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            false);
      }
      catch (const Ogre::Exception &) { /* already added — fine */ }

      auto *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
      hm->hlmsCompute = hlmsManager->getComputeHlms();
      if (!hm->hlmsCompute)
      {
        gzerr << "[waves_ogre2_heightmap] no HlmsCompute for evolve"
              << std::endl;
        return 0;
      }

      const std::string jobName =
          "WavesEvolve_" +
          std::to_string(reinterpret_cast<std::uintptr_t>(hm));
      hm->evolveJob = hm->hlmsCompute->createComputeJob(
          jobName, jobName, hm->evolveShaderName, Ogre::StringVector{});
      if (!hm->evolveJob)
      {
        gzerr << "[waves_ogre2_heightmap] createComputeJob(evolve) failed"
              << std::endl;
        return 0;
      }

      hm->evolveJob->setThreadsPerGroup(16u, 16u, 1u);
      const Ogre::uint32 groups =
          static_cast<Ogre::uint32>((hm->gridSize + 15) / 16);
      hm->evolveJob->setNumThreadGroups(groups, groups, 1u);

      // 1 UAV slot (write target) + 1 texture slot (h0 read input).
      // In OgreNext's OpenGL compute path, UAVs at slot 1+ are
      // unreliable. The supported idiom is: writes via UAV, reads
      // via regular texture samplers (texelFetch in GLSL).
      // ω(k) is computed in-shader, so no omegaTex is bound — slot-1
      // texture samplers are also unreliable in OgreNext compute.
      hm->evolveJob->setNumUavUnits(1u);
      hm->evolveJob->setNumTexUnits(1u);

      Ogre::DescriptorSetUav::TextureSlot uavSlot =
          Ogre::DescriptorSetUav::TextureSlot::makeEmpty();
      uavSlot.texture = hm->hktTex;
      uavSlot.access  = Ogre::ResourceAccess::Write;
      uavSlot.pixelFormat = Ogre::PFG_RGBA32_FLOAT;
      hm->evolveJob->_setUavTexture(0u, uavSlot);

      auto bindTex = [&](Ogre::uint8 slot, Ogre::TextureGpu *tex)
      {
        Ogre::DescriptorSetTexture2::TextureSlot s =
            Ogre::DescriptorSetTexture2::TextureSlot::makeEmpty();
        s.texture = tex;
        hm->evolveJob->setTexture(slot, s, &hm->samplerblock);
      };
      bindTex(0u, hm->h0Tex);

      // Params const buffer.
      auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
      auto *vaoManager   = renderSystem->getVaoManager();
      hm->evolveParams = vaoManager->createConstBuffer(
          sizeof(EvolveParams),
          Ogre::BT_DYNAMIC_PERSISTENT,
          nullptr, false);
      hm->evolveJob->setConstBuffer(0u, hm->evolveParams);

      gzmsg << "[waves_ogre2_heightmap] evolve pipeline initialised: "
            << "shader=" << hm->evolveShaderName << " "
            << "thread_groups=" << groups << "x" << groups << "x1"
            << std::endl;
    }
    catch (const Ogre::Exception &e)
    {
      gzerr << "[waves_ogre2_heightmap] evolve setup threw: "
            << e.getDescription() << std::endl;
      hm->evolveJob = nullptr;
      return 0;
    }
  }

  if (!hm->evolveJob || !hm->evolveParams)
    return 0;

  try
  {
    // Tell the BarrierSolver what the evolve pass is about to do.
    // h0Tex is bound as a texture (not a UAV) so it transitions to
    // ResourceLayout::Texture.
    auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
    auto &solver = renderSystem->getBarrierSolver();
    auto &transitions = solver.getNewResourceTransitionsArrayTmp();
    solver.resolveTransition(transitions, hm->h0Tex,
        Ogre::ResourceLayout::Texture, Ogre::ResourceAccess::Read,
        Ogre::c_computeStageMask);
    solver.resolveTransition(transitions, hm->hktTex,
        Ogre::ResourceLayout::Uav, Ogre::ResourceAccess::Write,
        Ogre::c_computeStageMask);
    renderSystem->executeResourceTransition(transitions);

    EvolveParams p{};
    p.t        = _simTimeS;
    p.tau      = _tauS;
    p.gridSize = static_cast<int>(hm->gridSize);
    p.tileSize = _tileSizeM;
    hm->evolveParams->upload(&p, 0u, sizeof(p));
    hm->hlmsCompute->dispatch(hm->evolveJob, hm->sceneManager, nullptr);
    if (!hm->hktTex->isDataReady())
      hm->hktTex->notifyDataIsReady();
  }
  catch (const Ogre::Exception &e)
  {
    static bool logged = false;
    if (!logged)
    {
      logged = true;
      gzerr << "[waves_ogre2_heightmap] evolve dispatch threw: "
            << e.getDescription() << " (further failures suppressed)"
            << std::endl;
    }
    return 0;
  }
  return 1;
}

int waves_ogre2_heightmap_evolve_dy_dispatch(
    waves_heightmap_t _handle, const char *_shaderAbsPath,
    float _simTimeS, float _tauS, float _tileSizeM)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->h0Tex || !hm->hktTexDy ||
      !hm->sceneManager || !_shaderAbsPath)
    return 0;

  hm->h0Tex->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);
  hm->hktTexDy->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);

  if (!hm->evolveDyJob)
  {
    try
    {
      auto &rgMgr = Ogre::ResourceGroupManager::getSingleton();
      const std::filesystem::path absPath(_shaderAbsPath);
      const std::string dir = absPath.parent_path().string();
      hm->evolveDyShaderName = absPath.stem().string();
      try
      {
        rgMgr.addResourceLocation(
            dir, "FileSystem",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            false);
      }
      catch (const Ogre::Exception &) { /* already added */ }

      auto *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
      hm->hlmsCompute = hlmsManager->getComputeHlms();
      if (!hm->hlmsCompute)
        return 0;

      const std::string jobName =
          "WavesEvolveDy_" +
          std::to_string(reinterpret_cast<std::uintptr_t>(hm));
      hm->evolveDyJob = hm->hlmsCompute->createComputeJob(
          jobName, jobName, hm->evolveDyShaderName, Ogre::StringVector{});
      if (!hm->evolveDyJob)
        return 0;

      hm->evolveDyJob->setThreadsPerGroup(16u, 16u, 1u);
      const Ogre::uint32 groups =
          static_cast<Ogre::uint32>((hm->gridSize + 15) / 16);
      hm->evolveDyJob->setNumThreadGroups(groups, groups, 1u);
      hm->evolveDyJob->setNumUavUnits(1u);
      hm->evolveDyJob->setNumTexUnits(1u);

      Ogre::DescriptorSetUav::TextureSlot uavSlot =
          Ogre::DescriptorSetUav::TextureSlot::makeEmpty();
      uavSlot.texture = hm->hktTexDy;
      uavSlot.access  = Ogre::ResourceAccess::Write;
      uavSlot.pixelFormat = Ogre::PFG_RGBA32_FLOAT;
      hm->evolveDyJob->_setUavTexture(0u, uavSlot);

      Ogre::DescriptorSetTexture2::TextureSlot texSlot =
          Ogre::DescriptorSetTexture2::TextureSlot::makeEmpty();
      texSlot.texture = hm->h0Tex;
      hm->evolveDyJob->setTexture(0u, texSlot, &hm->samplerblock);

      auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
      auto *vaoManager   = renderSystem->getVaoManager();
      hm->evolveDyParams = vaoManager->createConstBuffer(
          sizeof(EvolveParams),
          Ogre::BT_DYNAMIC_PERSISTENT,
          nullptr, false);
      hm->evolveDyJob->setConstBuffer(0u, hm->evolveDyParams);

      gzmsg << "[waves_ogre2_heightmap] evolve_dy pipeline initialised: "
            << "shader=" << hm->evolveDyShaderName << std::endl;
    }
    catch (const Ogre::Exception &e)
    {
      gzerr << "[waves_ogre2_heightmap] evolve_dy setup threw: "
            << e.getDescription() << std::endl;
      hm->evolveDyJob = nullptr;
      return 0;
    }
  }

  if (!hm->evolveDyJob || !hm->evolveDyParams)
    return 0;

  try
  {
    auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
    auto &solver = renderSystem->getBarrierSolver();
    auto &transitions = solver.getNewResourceTransitionsArrayTmp();
    solver.resolveTransition(transitions, hm->h0Tex,
        Ogre::ResourceLayout::Texture, Ogre::ResourceAccess::Read,
        Ogre::c_computeStageMask);
    solver.resolveTransition(transitions, hm->hktTexDy,
        Ogre::ResourceLayout::Uav, Ogre::ResourceAccess::Write,
        Ogre::c_computeStageMask);
    renderSystem->executeResourceTransition(transitions);

    EvolveParams p{};
    p.t        = _simTimeS;
    p.tau      = _tauS;
    p.gridSize = static_cast<int>(hm->gridSize);
    p.tileSize = _tileSizeM;
    hm->evolveDyParams->upload(&p, 0u, sizeof(p));
    hm->hlmsCompute->dispatch(hm->evolveDyJob, hm->sceneManager, nullptr);
    if (!hm->hktTexDy->isDataReady())
      hm->hktTexDy->notifyDataIsReady();
  }
  catch (const Ogre::Exception &e)
  {
    static bool logged = false;
    if (!logged)
    {
      logged = true;
      gzerr << "[waves_ogre2_heightmap] evolve_dy dispatch threw: "
            << e.getDescription() << " (further failures suppressed)"
            << std::endl;
    }
    return 0;
  }
  return 1;
}

namespace
{
  /// Returns true iff n is a positive power of two.
  bool IsPow2(std::size_t n) { return n > 0 && (n & (n - 1)) == 0; }

  /// log2(n) for n a power of two. UB if not.
  int Log2Pow2(std::size_t n)
  {
    int r = 0;
    while ((static_cast<std::size_t>(1u) << r) < n) ++r;
    return r;
  }
}

int waves_ogre2_heightmap_ifft_dispatch(
    waves_heightmap_t _handle,
    const char *_bitrevShaderAbsPath,
    const char *_butterShaderAbsPath)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->hktTex || !hm->sceneManager ||
      !_bitrevShaderAbsPath || !_butterShaderAbsPath)
    return 0;
  if (!IsPow2(hm->gridSize))
  {
    gzerr << "[waves_ogre2_heightmap] IFFT requires power-of-two gridSize ("
          << hm->gridSize << ")" << std::endl;
    return 0;
  }

  // Lazy init: create the ping-pong textures, the per-pass ConstBuffers
  // (params are time-invariant, so one buffer per pass at init), and
  // **one HlmsComputeJob per IFFT pass** with stable UAV+texture
  // bindings. This avoids the binding-cache and barrier-timing quirks
  // we hit when reusing a single job across 16 dispatches with
  // rebinding between each one.
  if (hm->ifftPassJobs.empty())
  {
    try
    {
      const int N     = static_cast<int>(hm->gridSize);
      const int log2N = Log2Pow2(hm->gridSize);
      hm->ifftLog2N   = log2N;

      auto &rgMgr = Ogre::ResourceGroupManager::getSingleton();
      const std::filesystem::path bitrevPath(_bitrevShaderAbsPath);
      const std::filesystem::path butterPath(_butterShaderAbsPath);
      hm->bitrevShaderName = bitrevPath.stem().string();
      hm->butterShaderName = butterPath.stem().string();
      for (const auto &dir : {bitrevPath.parent_path().string(),
                              butterPath.parent_path().string()})
      {
        try
        {
          rgMgr.addResourceLocation(
              dir, "FileSystem",
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
              false);
        }
        catch (const Ogre::Exception &) { /* already added */ }
      }

      // Ping-pong textures.
      const std::string base =
          "WavesIfft_" +
          std::to_string(reinterpret_cast<std::uintptr_t>(hm));
      hm->ifftBufAName = base + "_A";
      hm->ifftBufBName = base + "_B";
      hm->ifftBufA = MakeSpectrumTexture(hm->manager, hm->ifftBufAName,
                                          hm->gridSize,
                                          Ogre::PFG_RGBA32_FLOAT);
      hm->ifftBufB = MakeSpectrumTexture(hm->manager, hm->ifftBufBName,
                                          hm->gridSize,
                                          Ogre::PFG_RGBA32_FLOAT);

      auto *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
      hm->hlmsCompute = hlmsManager->getComputeHlms();
      if (!hm->hlmsCompute)
      {
        gzerr << "[waves_ogre2_heightmap] no HlmsCompute for IFFT"
              << std::endl;
        return 0;
      }

      // Pre-bake one ConstBuffer per pass. Layout (per axis a in
      // {0, 1}): 1 bitrev params + log2N butterfly params.
      auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
      auto *vaoManager   = renderSystem->getVaoManager();
      hm->ifftParams.reserve(
          static_cast<std::size_t>(2 * (log2N + 1)));
      for (int axis = 0; axis < 2; ++axis)
      {
        BitrevParams bp{};
        bp.gridSize = N;
        bp.axis     = axis;
        bp.log2N    = log2N;
        bp._pad     = 0;
        auto *buf = vaoManager->createConstBuffer(
            sizeof(BitrevParams), Ogre::BT_DEFAULT, &bp, false);
        hm->ifftParams.push_back(buf);

        for (int s = 0; s < log2N; ++s)
        {
          ButterParams btp{};
          btp.gridSize   = N;
          btp.stage      = s;
          btp.axis       = axis;
          btp.invertSign = +1.0f;  // IFFT
          btp.extraScale = (s == log2N - 1)
              ? 1.0f / static_cast<float>(N)
              : 1.0f;
          auto *bbuf = vaoManager->createConstBuffer(
              sizeof(ButterParams), Ogre::BT_DEFAULT, &btp, false);
          hm->ifftParams.push_back(bbuf);
        }
      }

      // Build a deterministic pass schedule of (isBitrev, paramIdx,
      // src, dst). Output of each pass becomes input to the next.
      struct PassDesc
      {
        bool isBitrev;
        std::size_t paramIdx;
        Ogre::TextureGpu *src;
        Ogre::TextureGpu *dst;
      };
      std::vector<PassDesc> plan;
      plan.reserve(static_cast<std::size_t>(2 * (log2N + 1)));

      const std::size_t perAxis =
          static_cast<std::size_t>(log2N + 1);

      // Row pass: bitrev hkt→A, then butters alternating A↔B.
      plan.push_back({true, 0u, hm->hktTex, hm->ifftBufA});
      Ogre::TextureGpu *cur = hm->ifftBufA;
      Ogre::TextureGpu *next = hm->ifftBufB;
      for (int s = 0; s < log2N; ++s)
      {
        plan.push_back({false,
                        1u + static_cast<std::size_t>(s),
                        cur, next});
        std::swap(cur, next);
      }
      // After row pass, cur holds the row-IFFT result.

      // Column pass: bitrev cur → other, then butters alternating.
      Ogre::TextureGpu *other =
          (cur == hm->ifftBufA) ? hm->ifftBufB : hm->ifftBufA;
      plan.push_back({true, perAxis, cur, other});
      cur  = other;
      next = (cur == hm->ifftBufA) ? hm->ifftBufB : hm->ifftBufA;
      for (int s = 0; s < log2N; ++s)
      {
        plan.push_back({false,
                        perAxis + 1u + static_cast<std::size_t>(s),
                        cur, next});
        std::swap(cur, next);
      }

      // The final pass writes the spatial η output. `cur` after the
      // last swap holds what would be the next dst, so the actual
      // last-written texture is plan.back().dst.
      hm->ifftFinalTex = plan.back().dst;

      // Create one job per pass with stable bindings.
      hm->ifftPassJobs.reserve(plan.size());
      hm->ifftPassSrcDst.reserve(plan.size());
      const Ogre::uint32 groups =
          static_cast<Ogre::uint32>((N + 15) / 16);
      for (std::size_t i = 0; i < plan.size(); ++i)
      {
        const PassDesc &p = plan[i];
        const std::string jobName = base + "_pass" + std::to_string(i);
        Ogre::HlmsComputeJob *job = hm->hlmsCompute->createComputeJob(
            jobName, jobName,
            p.isBitrev ? hm->bitrevShaderName : hm->butterShaderName,
            Ogre::StringVector{});
        if (!job)
        {
          gzerr << "[waves_ogre2_heightmap] createComputeJob(ifft pass "
                << i << ") failed" << std::endl;
          return 0;
        }
        job->setThreadsPerGroup(16u, 16u, 1u);
        job->setNumThreadGroups(groups, groups, 1u);
        job->setNumUavUnits(1u);
        job->setNumTexUnits(1u);

        Ogre::DescriptorSetUav::TextureSlot uavSlot =
            Ogre::DescriptorSetUav::TextureSlot::makeEmpty();
        uavSlot.texture     = p.dst;
        uavSlot.access      = Ogre::ResourceAccess::Write;
        uavSlot.pixelFormat = Ogre::PFG_RGBA32_FLOAT;
        job->_setUavTexture(0u, uavSlot);

        Ogre::DescriptorSetTexture2::TextureSlot texSlot =
            Ogre::DescriptorSetTexture2::TextureSlot::makeEmpty();
        texSlot.texture = p.src;
        job->setTexture(0u, texSlot, &hm->samplerblock);

        job->setConstBuffer(0u, hm->ifftParams[p.paramIdx]);

        hm->ifftPassJobs.push_back(job);
        hm->ifftPassSrcDst.emplace_back(p.src, p.dst);
      }

      gzmsg << "[waves_ogre2_heightmap] IFFT pipeline initialised: N=" << N
            << " log2N=" << log2N
            << " passes_per_frame=" << hm->ifftPassJobs.size()
            << " bitrev=" << hm->bitrevShaderName
            << " butter=" << hm->butterShaderName
            << " ifftFinalTex=" << hm->ifftFinalTex->getNameStr()
            << std::endl;

      // Parallel Dy IFFT pipeline. Same shaders, same params (the
      // butterfly's behaviour is independent of which RGBA32F texture
      // it reads), but its own ping-pong textures and pass jobs.
      // Only built when hktTexDy is present — i.e. the Dy chop path
      // is enabled.
      if (hm->hktTexDy)
      {
        const std::string dyBase = base + "_dy";
        hm->ifftDyBufA = MakeSpectrumTexture(hm->manager, dyBase + "_A",
                                              hm->gridSize,
                                              Ogre::PFG_RGBA32_FLOAT);
        hm->ifftDyBufB = MakeSpectrumTexture(hm->manager, dyBase + "_B",
                                              hm->gridSize,
                                              Ogre::PFG_RGBA32_FLOAT);

        std::vector<PassDesc> dyPlan;
        dyPlan.reserve(static_cast<std::size_t>(2 * (log2N + 1)));
        dyPlan.push_back({true, 0u, hm->hktTexDy, hm->ifftDyBufA});
        Ogre::TextureGpu *dCur  = hm->ifftDyBufA;
        Ogre::TextureGpu *dNext = hm->ifftDyBufB;
        for (int s = 0; s < log2N; ++s)
        {
          dyPlan.push_back({false,
                            1u + static_cast<std::size_t>(s),
                            dCur, dNext});
          std::swap(dCur, dNext);
        }
        Ogre::TextureGpu *dOther =
            (dCur == hm->ifftDyBufA) ? hm->ifftDyBufB : hm->ifftDyBufA;
        dyPlan.push_back({true, perAxis, dCur, dOther});
        dCur  = dOther;
        dNext = (dCur == hm->ifftDyBufA) ? hm->ifftDyBufB : hm->ifftDyBufA;
        for (int s = 0; s < log2N; ++s)
        {
          dyPlan.push_back({false,
                            perAxis + 1u + static_cast<std::size_t>(s),
                            dCur, dNext});
          std::swap(dCur, dNext);
        }
        hm->ifftDyFinalTex = dyPlan.back().dst;

        hm->ifftDyPassJobs.reserve(dyPlan.size());
        hm->ifftDyPassSrcDst.reserve(dyPlan.size());
        for (std::size_t i = 0; i < dyPlan.size(); ++i)
        {
          const PassDesc &p = dyPlan[i];
          const std::string jobName =
              dyBase + "_pass" + std::to_string(i);
          Ogre::HlmsComputeJob *job = hm->hlmsCompute->createComputeJob(
              jobName, jobName,
              p.isBitrev ? hm->bitrevShaderName : hm->butterShaderName,
              Ogre::StringVector{});
          if (!job)
            return 0;
          job->setThreadsPerGroup(16u, 16u, 1u);
          job->setNumThreadGroups(groups, groups, 1u);
          job->setNumUavUnits(1u);
          job->setNumTexUnits(1u);

          Ogre::DescriptorSetUav::TextureSlot uavSlot =
              Ogre::DescriptorSetUav::TextureSlot::makeEmpty();
          uavSlot.texture     = p.dst;
          uavSlot.access      = Ogre::ResourceAccess::Write;
          uavSlot.pixelFormat = Ogre::PFG_RGBA32_FLOAT;
          job->_setUavTexture(0u, uavSlot);

          Ogre::DescriptorSetTexture2::TextureSlot texSlot =
              Ogre::DescriptorSetTexture2::TextureSlot::makeEmpty();
          texSlot.texture = p.src;
          job->setTexture(0u, texSlot, &hm->samplerblock);

          job->setConstBuffer(0u, hm->ifftParams[p.paramIdx]);
          hm->ifftDyPassJobs.push_back(job);
          hm->ifftDyPassSrcDst.emplace_back(p.src, p.dst);
        }

        gzmsg << "[waves_ogre2_heightmap] Dy IFFT pipeline initialised: "
              << "passes=" << hm->ifftDyPassJobs.size()
              << " ifftDyFinalTex="
              << hm->ifftDyFinalTex->getNameStr() << std::endl;
      }
    }
    catch (const Ogre::Exception &e)
    {
      gzerr << "[waves_ogre2_heightmap] IFFT setup threw: "
            << e.getDescription() << std::endl;
      hm->ifftPassJobs.clear();
      hm->ifftPassSrcDst.clear();
      hm->ifftDyPassJobs.clear();
      hm->ifftDyPassSrcDst.clear();
      return 0;
    }
  }

  if (hm->ifftPassJobs.empty() || hm->ifftParams.empty())
    return 0;

  hm->ifftBufA->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);
  hm->ifftBufB->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);

  try
  {
    auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
    auto &solver = renderSystem->getBarrierSolver();

    // Belt-and-braces explicit memory barrier between dispatches.
    // OgreNext's BarrierSolver/executeResourceTransition should emit
    // the right glMemoryBarrier on GL3+, but if its tracking of
    // compute-to-compute UAV/Texture transitions is incomplete, the
    // chain reads stale data and produces wrong output. We look up
    // glMemoryBarrier at runtime (the symbol is already loaded by
    // the GL3+ render system) and call it ourselves with
    // GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT
    // between dispatches.
    static constexpr unsigned GL_SHADER_IMAGE_ACCESS_BARRIER_BIT_VAL =
        0x00000020u;
    static constexpr unsigned GL_TEXTURE_FETCH_BARRIER_BIT_VAL =
        0x00000008u;
    using PFN_glMemoryBarrier = void (*)(unsigned);
    static auto glMemoryBarrierPtr =
        reinterpret_cast<PFN_glMemoryBarrier>(
            dlsym(RTLD_DEFAULT, "glMemoryBarrier"));

    auto runPipeline = [&](
        const std::vector<Ogre::HlmsComputeJob *> &jobs,
        const std::vector<std::pair<Ogre::TextureGpu *, Ogre::TextureGpu *>>
            &srcDst)
    {
      for (std::size_t i = 0; i < jobs.size(); ++i)
      {
        Ogre::TextureGpu *src = srcDst[i].first;
        Ogre::TextureGpu *dst = srcDst[i].second;

        auto &transitions = solver.getNewResourceTransitionsArrayTmp();
        solver.resolveTransition(transitions, src,
            Ogre::ResourceLayout::Texture, Ogre::ResourceAccess::Read,
            Ogre::c_computeStageMask);
        solver.resolveTransition(transitions, dst,
            Ogre::ResourceLayout::Uav, Ogre::ResourceAccess::Write,
            Ogre::c_computeStageMask);
        renderSystem->executeResourceTransition(transitions);

        hm->hlmsCompute->dispatch(jobs[i], hm->sceneManager, nullptr);

        if (glMemoryBarrierPtr)
        {
          glMemoryBarrierPtr(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT_VAL |
                             GL_TEXTURE_FETCH_BARRIER_BIT_VAL);
        }
      }
    };

    runPipeline(hm->ifftPassJobs, hm->ifftPassSrcDst);
    if (!hm->ifftDyPassJobs.empty())
      runPipeline(hm->ifftDyPassJobs, hm->ifftDyPassSrcDst);

    if (!hm->ifftFinalTex->isDataReady())
      hm->ifftFinalTex->notifyDataIsReady();
    if (hm->ifftDyFinalTex && !hm->ifftDyFinalTex->isDataReady())
      hm->ifftDyFinalTex->notifyDataIsReady();

    // Transition the final IFFT output(s) from Uav to Texture so the
    // combine pass / visual can sample them. On GL this emits
    // glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT). Next frame's
    // first IFFT pass will transition it back to Uav.
    {
      auto &t2 = solver.getNewResourceTransitionsArrayTmp();
      solver.resolveTransition(t2, hm->ifftFinalTex,
          Ogre::ResourceLayout::Texture,
          Ogre::ResourceAccess::Read,
          Ogre::c_allGraphicStagesMask);
      if (hm->ifftDyFinalTex)
      {
        solver.resolveTransition(t2, hm->ifftDyFinalTex,
            Ogre::ResourceLayout::Texture,
            Ogre::ResourceAccess::Read,
            Ogre::c_allGraphicStagesMask);
      }
      renderSystem->executeResourceTransition(t2);
    }

    // When the Dy pipeline is active, the combine pass owns the
    // material binding (it writes into combinedTex). Otherwise bind
    // ifftFinalTex directly — degraded mode without chop, but a
    // useful fallback during bring-up.
    if (!hm->ifftBoundToMaterial && hm->ogreMaterial && hm->ifftFinalTex
        && !hm->hktTexDy)
    {
      auto *pass = hm->ogreMaterial->getTechnique(0u)->getPass(0u);
      Ogre::TextureUnitState *texUnit = nullptr;
      for (unsigned int i = 0; i < pass->getNumTextureUnitStates(); ++i)
      {
        auto *u = pass->getTextureUnitState(i);
        if (u->getName() == "heightMap")
        {
          texUnit = u;
          break;
        }
      }
      if (texUnit)
      {
        texUnit->setTexture(hm->ifftFinalTex);
        texUnit->setSamplerblock(hm->samplerblock);
        hm->ifftBoundToMaterial = true;
        gzmsg << "[waves_ogre2_heightmap] heightMap sampler swapped to "
              << "GPU IFFT output (" << hm->ifftFinalTex->getNameStr()
              << "); CPU upload path retired" << std::endl;
      }
    }
  }
  catch (const Ogre::Exception &e)
  {
    static bool logged = false;
    if (!logged)
    {
      logged = true;
      gzerr << "[waves_ogre2_heightmap] IFFT dispatch threw: "
            << e.getDescription()
            << " (further failures suppressed)" << std::endl;
    }
    return 0;
  }
  return 1;
}

int waves_ogre2_heightmap_ifft_naive_dispatch(
    waves_heightmap_t _handle, const char *_naiveShaderAbsPath)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->hktTex || !hm->sceneManager || !_naiveShaderAbsPath)
    return 0;

  // We reuse `bitrevJob` as the storage slot for the naive job —
  // when the naive path is active, the radix-2 path is unused.
  if (!hm->bitrevJob)
  {
    // Self-allocate ifftBufB if the radix-2 path hasn't created it.
    if (!hm->ifftBufB)
    {
      try
      {
        const std::string baseName =
            "WavesIfft_" +
            std::to_string(reinterpret_cast<std::uintptr_t>(hm));
        hm->ifftBufBName = baseName + "_B";
        hm->ifftBufB = MakeSpectrumTexture(
            hm->manager, hm->ifftBufBName,
            hm->gridSize, Ogre::PFG_RGBA32_FLOAT);
      }
      catch (const Ogre::Exception &e)
      {
        gzerr << "[waves_ogre2_heightmap] naive IFFT: bufB alloc failed: "
              << e.getDescription() << std::endl;
        return 0;
      }
    }
    try
    {
      auto &rgMgr = Ogre::ResourceGroupManager::getSingleton();
      const std::filesystem::path absPath(_naiveShaderAbsPath);
      const std::string dir = absPath.parent_path().string();
      const std::string shaderName = absPath.stem().string();
      try
      {
        rgMgr.addResourceLocation(
            dir, "FileSystem",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            false);
      }
      catch (const Ogre::Exception &) { /* already added */ }

      auto *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
      hm->hlmsCompute = hlmsManager->getComputeHlms();
      if (!hm->hlmsCompute)
        return 0;

      const std::string jobName =
          "WavesIfftNaive_" +
          std::to_string(reinterpret_cast<std::uintptr_t>(hm));
      hm->bitrevJob = hm->hlmsCompute->createComputeJob(
          jobName, jobName, shaderName, Ogre::StringVector{});
      if (!hm->bitrevJob)
        return 0;

      const int N = static_cast<int>(hm->gridSize);
      const Ogre::uint32 groups =
          static_cast<Ogre::uint32>((N + 15) / 16);
      hm->bitrevJob->setThreadsPerGroup(16u, 16u, 1u);
      hm->bitrevJob->setNumThreadGroups(groups, groups, 1u);
      hm->bitrevJob->setNumUavUnits(1u);
      hm->bitrevJob->setNumTexUnits(1u);

      Ogre::DescriptorSetUav::TextureSlot uavSlot =
          Ogre::DescriptorSetUav::TextureSlot::makeEmpty();
      uavSlot.texture     = hm->ifftBufB;
      uavSlot.access      = Ogre::ResourceAccess::Write;
      uavSlot.pixelFormat = Ogre::PFG_RGBA32_FLOAT;
      hm->bitrevJob->_setUavTexture(0u, uavSlot);

      Ogre::DescriptorSetTexture2::TextureSlot texSlot =
          Ogre::DescriptorSetTexture2::TextureSlot::makeEmpty();
      texSlot.texture = hm->hktTex;
      hm->bitrevJob->setTexture(0u, texSlot, &hm->samplerblock);

      // Reuse the first ConstBufferPacked from the IFFT params, but
      // patch the contents — BitrevParams has gridSize in slot 0
      // which the naive shader also reads as the first int. Simpler:
      // create our own const buffer.
      auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
      auto *vaoManager   = renderSystem->getVaoManager();
      struct alignas(16) NaiveParams
      {
        int gridSize;
        int _pad0;
        int _pad1;
        int _pad2;
      } np{};
      np.gridSize = N;
      auto *buf = vaoManager->createConstBuffer(
          sizeof(NaiveParams), Ogre::BT_DEFAULT, &np, false);
      hm->ifftParams.push_back(buf);  // keep alive
      hm->bitrevJob->setConstBuffer(0u, buf);

      hm->ifftFinalTex = hm->ifftBufB;

      gzmsg << "[waves_ogre2_heightmap] naive IFFT pipeline online — "
            << "single dispatch of " << shaderName
            << " (hktTex → " << hm->ifftFinalTex->getNameStr() << ")"
            << std::endl;
    }
    catch (const Ogre::Exception &e)
    {
      gzerr << "[waves_ogre2_heightmap] naive IFFT setup threw: "
            << e.getDescription() << std::endl;
      hm->bitrevJob = nullptr;
      return 0;
    }
  }

  if (!hm->bitrevJob)
    return 0;

  hm->ifftBufB->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);

  try
  {
    auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
    auto &solver = renderSystem->getBarrierSolver();

    auto &t1 = solver.getNewResourceTransitionsArrayTmp();
    solver.resolveTransition(t1, hm->hktTex,
        Ogre::ResourceLayout::Texture, Ogre::ResourceAccess::Read,
        Ogre::c_computeStageMask);
    solver.resolveTransition(t1, hm->ifftBufB,
        Ogre::ResourceLayout::Uav, Ogre::ResourceAccess::Write,
        Ogre::c_computeStageMask);
    renderSystem->executeResourceTransition(t1);

    hm->hlmsCompute->dispatch(hm->bitrevJob, hm->sceneManager, nullptr);

    auto &t2 = solver.getNewResourceTransitionsArrayTmp();
    solver.resolveTransition(t2, hm->ifftFinalTex,
        Ogre::ResourceLayout::Texture, Ogre::ResourceAccess::Read,
        Ogre::c_allGraphicStagesMask);
    renderSystem->executeResourceTransition(t2);

    if (!hm->ifftBoundToMaterial && hm->ogreMaterial && hm->ifftFinalTex)
    {
      auto *pass = hm->ogreMaterial->getTechnique(0u)->getPass(0u);
      Ogre::TextureUnitState *texUnit = nullptr;
      for (unsigned int i = 0; i < pass->getNumTextureUnitStates(); ++i)
      {
        auto *u = pass->getTextureUnitState(i);
        if (u->getName() == "heightMap")
        {
          texUnit = u;
          break;
        }
      }
      if (texUnit)
      {
        texUnit->setTexture(hm->ifftFinalTex);
        texUnit->setSamplerblock(hm->samplerblock);
        hm->ifftBoundToMaterial = true;
        gzmsg << "[waves_ogre2_heightmap] heightMap sampler swapped to "
              << "naive IFFT output (" << hm->ifftFinalTex->getNameStr()
              << ")" << std::endl;
      }
    }
  }
  catch (const Ogre::Exception &e)
  {
    static bool logged = false;
    if (!logged)
    {
      logged = true;
      gzerr << "[waves_ogre2_heightmap] naive IFFT dispatch threw: "
            << e.getDescription() << std::endl;
    }
    return 0;
  }
  return 1;
}

/// Diagnostic: upload CPU's (eta, dx, dy) grids directly into
/// ifftFinalTex, bypassing the GPU evolve+IFFT pipeline entirely.
/// Used to test whether the visual sampling path itself is correct
/// (if the visual then looks identical to CPU, the compute chain is
/// what's wrong; if it still looks "fast", the binding path is what's
/// wrong). Allocates ifftBufB if the radix-2 path hasn't run yet.
int waves_ogre2_heightmap_cpu_feed(
    waves_heightmap_t _handle,
    const double *_eta, const double *_dx, const double *_dy,
    int _rows, int _cols)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->manager || !hm->sceneManager || !_eta || !_dx || !_dy)
    return 0;
  const int N = static_cast<int>(hm->gridSize);
  if (_rows != N || _cols != N)
    return 0;

  // Lazy-allocate ifftBufB / ifftFinalTex if the IFFT path didn't.
  if (!hm->ifftBufB)
  {
    try
    {
      const std::string baseName =
          "WavesIfft_" +
          std::to_string(reinterpret_cast<std::uintptr_t>(hm));
      hm->ifftBufBName = baseName + "_B";
      hm->ifftBufB = MakeSpectrumTexture(hm->manager, hm->ifftBufBName,
                                          hm->gridSize,
                                          Ogre::PFG_RGBA32_FLOAT);
      hm->ifftFinalTex = hm->ifftBufB;
    }
    catch (const Ogre::Exception &e)
    {
      gzerr << "[waves_ogre2_heightmap] cpu_feed: bufB alloc failed: "
            << e.getDescription() << std::endl;
      return 0;
    }
  }
  if (!hm->ifftFinalTex)
    hm->ifftFinalTex = hm->ifftBufB;

  try
  {
    Ogre::StagingTexture *staging = hm->manager->getStagingTexture(
        static_cast<Ogre::uint32>(hm->gridSize),
        static_cast<Ogre::uint32>(hm->gridSize),
        1u, 1u, Ogre::PFG_RGBA32_FLOAT);
    staging->startMapRegion();
    Ogre::TextureBox box = staging->mapRegion(
        static_cast<Ogre::uint32>(hm->gridSize),
        static_cast<Ogre::uint32>(hm->gridSize),
        1u, 1u, Ogre::PFG_RGBA32_FLOAT);
    for (int row = 0; row < N; ++row)
    {
      auto *outRow = reinterpret_cast<float *>(box.at(0, row, 0));
      const double *eta = _eta + static_cast<std::size_t>(row) * N;
      const double *dx  = _dx  + static_cast<std::size_t>(row) * N;
      const double *dy  = _dy  + static_cast<std::size_t>(row) * N;
      for (int col = 0; col < N; ++col)
      {
        outRow[col * 4 + 0] = static_cast<float>(eta[col]);
        outRow[col * 4 + 1] = static_cast<float>(dx[col]);
        outRow[col * 4 + 2] = static_cast<float>(dy[col]);
        outRow[col * 4 + 3] = 0.0f;
      }
    }
    staging->stopMapRegion();
    staging->upload(box, hm->ifftFinalTex, 0u, nullptr, nullptr);
    hm->manager->removeStagingTexture(staging);
    if (!hm->ifftFinalTex->isDataReady())
      hm->ifftFinalTex->notifyDataIsReady();

    // Make sure Stage 4's swap pointed at this texture (or do it now).
    if (!hm->ifftBoundToMaterial && hm->ogreMaterial)
    {
      auto *pass = hm->ogreMaterial->getTechnique(0u)->getPass(0u);
      Ogre::TextureUnitState *texUnit = nullptr;
      for (unsigned int i = 0; i < pass->getNumTextureUnitStates(); ++i)
      {
        auto *u = pass->getTextureUnitState(i);
        if (u->getName() == "heightMap") { texUnit = u; break; }
      }
      if (texUnit)
      {
        texUnit->setTexture(hm->ifftFinalTex);
        texUnit->setSamplerblock(hm->samplerblock);
        hm->ifftBoundToMaterial = true;
        gzmsg << "[waves_ogre2_heightmap] cpu_feed: heightMap sampler "
              << "swapped to " << hm->ifftFinalTex->getNameStr()
              << "; this is the CPU-feed diagnostic path."
              << std::endl;
      }
    }
  }
  catch (const Ogre::Exception &e)
  {
    static bool logged = false;
    if (!logged)
    {
      logged = true;
      gzerr << "[waves_ogre2_heightmap] cpu_feed upload threw: "
            << e.getDescription() << std::endl;
    }
    return 0;
  }
  return 1;
}

/// Async readback of a single cell from h0Tex. Used to verify what
/// the GPU sees vs the CPU values that were uploaded.
/// On return, *_re and *_im receive h0Tex(i_row, j_col) interpreted
/// as the original (h0.re, h0.im) pair. Returns 1 on success, 0 if
/// the texture isn't allocated or the readback fails.
int waves_ogre2_heightmap_readback_h0(
    waves_heightmap_t _handle, int _i, int _j,
    float *_outRe, float *_outIm,
    float *_outConjRe, float *_outConjIm)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->h0Tex || !hm->manager || _outRe == nullptr ||
      _outIm == nullptr)
    return 0;
  const int N = static_cast<int>(hm->gridSize);
  if (_i < 0 || _i >= N || _j < 0 || _j >= N)
    return 0;

  try
  {
    Ogre::AsyncTextureTicket *ticket =
        hm->manager->createAsyncTextureTicket(
            static_cast<Ogre::uint32>(N),
            static_cast<Ogre::uint32>(N),
            1u, Ogre::TextureTypes::Type2D,
            Ogre::PFG_RGBA32_FLOAT);
    if (!ticket)
      return 0;
    ticket->download(hm->h0Tex, 0u, true /*accurateTracking*/,
                     nullptr, true /*bImmediate*/);

    // Poll for completion. The transfer is typically immediate on GL3+
    // since we requested bImmediate=true and the texture is Resident,
    // but be defensive.
    int spins = 0;
    while (!ticket->queryIsTransferDone() && spins < 1000)
      ++spins;

    const Ogre::TextureBox box = ticket->map(0u);
    // Pixel at (col=_j, row=_i) holds h0(_i, _j) per the upload layout.
    const std::uint8_t *base = static_cast<const std::uint8_t *>(box.data);
    const std::uint8_t *pixelPtr =
        base + static_cast<std::size_t>(_i) * box.bytesPerRow +
        static_cast<std::size_t>(_j) * 16u;
    const float *p = reinterpret_cast<const float *>(pixelPtr);
    *_outRe = p[0];
    *_outIm = p[1];
    if (_outConjRe) *_outConjRe = p[2];
    if (_outConjIm) *_outConjIm = p[3];
    ticket->unmap();
    hm->manager->destroyAsyncTextureTicket(ticket);
    return 1;
  }
  catch (const Ogre::Exception &e)
  {
    gzerr << "[waves_ogre2_heightmap] h0 readback threw: "
          << e.getDescription() << std::endl;
    return 0;
  }
}

int waves_ogre2_heightmap_readback_ifft(
    waves_heightmap_t _handle, int _i, int _j, float *_outEta)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->ifftFinalTex || !hm->manager || _outEta == nullptr)
    return 0;
  const int N = static_cast<int>(hm->gridSize);
  if (_i < 0 || _i >= N || _j < 0 || _j >= N)
    return 0;
  try
  {
    Ogre::AsyncTextureTicket *ticket =
        hm->manager->createAsyncTextureTicket(
            static_cast<Ogre::uint32>(N),
            static_cast<Ogre::uint32>(N),
            1u, Ogre::TextureTypes::Type2D,
            Ogre::PFG_RGBA32_FLOAT);
    if (!ticket)
      return 0;
    ticket->download(hm->ifftFinalTex, 0u, true, nullptr, true);
    int spins = 0;
    while (!ticket->queryIsTransferDone() && spins < 10000)
      ++spins;
    const Ogre::TextureBox box = ticket->map(0u);
    const std::uint8_t *base = static_cast<const std::uint8_t *>(box.data);
    // CPU heightGrid_(i, j) is written by GPU thread (texel.x=i,
    // texel.y=j) to pixel(col=i, row=j). Memory: row j starts at
    // j*bytesPerRow; col i is at i*16 inside the row.
    const float *p = reinterpret_cast<const float *>(
        base + static_cast<std::size_t>(_j) * box.bytesPerRow +
        static_cast<std::size_t>(_i) * 16u);
    *_outEta = p[0];   // .r = η
    ticket->unmap();
    hm->manager->destroyAsyncTextureTicket(ticket);
    return 1;
  }
  catch (const Ogre::Exception &e)
  {
    gzerr << "[waves_ogre2_heightmap] ifft readback threw: "
          << e.getDescription() << std::endl;
    return 0;
  }
}

int waves_ogre2_heightmap_readback_hkt(
    waves_heightmap_t _handle, int _i, int _j,
    float *_outRe, float *_outIm)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->hktTex || !hm->manager || !_outRe || !_outIm)
    return 0;
  const int N = static_cast<int>(hm->gridSize);
  if (_i < 0 || _i >= N || _j < 0 || _j >= N)
    return 0;
  try
  {
    Ogre::AsyncTextureTicket *ticket =
        hm->manager->createAsyncTextureTicket(
            static_cast<Ogre::uint32>(N),
            static_cast<Ogre::uint32>(N),
            1u, Ogre::TextureTypes::Type2D,
            Ogre::PFG_RGBA32_FLOAT);
    if (!ticket)
      return 0;
    ticket->download(hm->hktTex, 0u, true, nullptr, true);
    int spins = 0;
    while (!ticket->queryIsTransferDone() && spins < 10000)
      ++spins;
    const Ogre::TextureBox box = ticket->map(0u);
    const std::uint8_t *base = static_cast<const std::uint8_t *>(box.data);
    // Evolve writes hkt at pixel(col=texel.x=i, row=texel.y=j). So
    // h(k=(i, j), t) is at memory offset j*bytesPerRow + i*16.
    const float *p = reinterpret_cast<const float *>(
        base + static_cast<std::size_t>(_j) * box.bytesPerRow +
        static_cast<std::size_t>(_i) * 16u);
    *_outRe = p[0];
    *_outIm = p[1];
    ticket->unmap();
    hm->manager->destroyAsyncTextureTicket(ticket);
    return 1;
  }
  catch (const Ogre::Exception &e)
  {
    gzerr << "[waves_ogre2_heightmap] hkt readback threw: "
          << e.getDescription() << std::endl;
    return 0;
  }
}

int waves_ogre2_heightmap_ifft_bound(waves_heightmap_t _handle)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  return (hm && hm->ifftBoundToMaterial) ? 1 : 0;
}

int waves_ogre2_heightmap_readback_combined_scan(
    waves_heightmap_t _handle,
    int *_outBadCount,
    float *_outMinRgba,
    float *_outMaxRgba,
    int *_outFirstBadI,
    int *_outFirstBadJ,
    float *_outFirstBadRgba)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->combinedTex || !hm->manager)
    return 0;
  const int N = static_cast<int>(hm->gridSize);
  try
  {
    Ogre::AsyncTextureTicket *ticket =
        hm->manager->createAsyncTextureTicket(
            static_cast<Ogre::uint32>(N),
            static_cast<Ogre::uint32>(N),
            1u, Ogre::TextureTypes::Type2D,
            Ogre::PFG_RGBA32_FLOAT);
    if (!ticket)
      return 0;
    ticket->download(hm->combinedTex, 0u, true, nullptr, true);
    int spins = 0;
    while (!ticket->queryIsTransferDone() && spins < 1000)
      ++spins;
    const Ogre::TextureBox box = ticket->map(0u);
    const std::uint8_t *base = static_cast<const std::uint8_t *>(box.data);

    float mn[4] = { std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity() };
    float mx[4] = { -std::numeric_limits<float>::infinity(),
                    -std::numeric_limits<float>::infinity(),
                    -std::numeric_limits<float>::infinity(),
                    -std::numeric_limits<float>::infinity() };
    int badCount = 0;
    int firstBadI = -1, firstBadJ = -1;
    float firstBadRgba[4] = { 0, 0, 0, 0 };

    for (int row = 0; row < N; ++row)
    {
      const float *p = reinterpret_cast<const float *>(
          base + static_cast<std::size_t>(row) * box.bytesPerRow);
      for (int col = 0; col < N; ++col)
      {
        const float r = p[col * 4 + 0];
        const float g = p[col * 4 + 1];
        const float b = p[col * 4 + 2];
        const float a = p[col * 4 + 3];
        const bool bad = !std::isfinite(r) || !std::isfinite(g) ||
                         !std::isfinite(b) || !std::isfinite(a);
        if (bad)
        {
          if (firstBadI < 0)
          {
            firstBadI = row;
            firstBadJ = col;
            firstBadRgba[0] = r;
            firstBadRgba[1] = g;
            firstBadRgba[2] = b;
            firstBadRgba[3] = a;
          }
          ++badCount;
        }
        else
        {
          if (r < mn[0]) mn[0] = r; if (r > mx[0]) mx[0] = r;
          if (g < mn[1]) mn[1] = g; if (g > mx[1]) mx[1] = g;
          if (b < mn[2]) mn[2] = b; if (b > mx[2]) mx[2] = b;
          if (a < mn[3]) mn[3] = a; if (a > mx[3]) mx[3] = a;
        }
      }
    }

    ticket->unmap();
    hm->manager->destroyAsyncTextureTicket(ticket);

    if (_outBadCount) *_outBadCount = badCount;
    if (_outMinRgba) std::memcpy(_outMinRgba, mn, sizeof(mn));
    if (_outMaxRgba) std::memcpy(_outMaxRgba, mx, sizeof(mx));
    if (_outFirstBadI) *_outFirstBadI = firstBadI;
    if (_outFirstBadJ) *_outFirstBadJ = firstBadJ;
    if (_outFirstBadRgba)
      std::memcpy(_outFirstBadRgba, firstBadRgba, sizeof(firstBadRgba));
    return 1;
  }
  catch (const Ogre::Exception &e)
  {
    gzerr << "[waves_ogre2_heightmap] combined scan threw: "
          << e.getDescription() << std::endl;
    return 0;
  }
}

int waves_ogre2_heightmap_debug_dispatch(
    waves_heightmap_t _handle, const char *_shaderAbsPath,
    float _simTimeS, float _amplitude)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->ifftFinalTex || !hm->sceneManager || !_shaderAbsPath)
    return 0;

  if (!hm->debugJob)
  {
    try
    {
      auto &rgMgr = Ogre::ResourceGroupManager::getSingleton();
      const std::filesystem::path absPath(_shaderAbsPath);
      const std::string dir = absPath.parent_path().string();
      hm->debugShaderName = absPath.stem().string();
      try
      {
        rgMgr.addResourceLocation(
            dir, "FileSystem",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            false);
      }
      catch (const Ogre::Exception &) { /* already added — fine */ }

      auto *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
      hm->hlmsCompute = hlmsManager->getComputeHlms();
      if (!hm->hlmsCompute)
        return 0;

      const std::string jobName =
          "WavesDebug_" +
          std::to_string(reinterpret_cast<std::uintptr_t>(hm));
      hm->debugJob = hm->hlmsCompute->createComputeJob(
          jobName, jobName, hm->debugShaderName, Ogre::StringVector{});
      if (!hm->debugJob)
        return 0;

      hm->debugJob->setThreadsPerGroup(16u, 16u, 1u);
      const Ogre::uint32 groups =
          static_cast<Ogre::uint32>((hm->gridSize + 15) / 16);
      hm->debugJob->setNumThreadGroups(groups, groups, 1u);
      hm->debugJob->setNumUavUnits(1u);

      auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
      auto *vaoManager   = renderSystem->getVaoManager();
      hm->debugParams = vaoManager->createConstBuffer(
          sizeof(DebugParams),
          Ogre::BT_DYNAMIC_PERSISTENT,
          nullptr, false);
      hm->debugJob->setConstBuffer(0u, hm->debugParams);

      gzmsg << "[waves_ogre2_heightmap] debug pattern pipeline online — "
            << "writing directly to ifftFinalTex ("
            << hm->ifftFinalTex->getNameStr() << ")" << std::endl;
    }
    catch (const Ogre::Exception &e)
    {
      gzerr << "[waves_ogre2_heightmap] debug setup threw: "
            << e.getDescription() << std::endl;
      hm->debugJob = nullptr;
      return 0;
    }
  }

  if (!hm->debugJob || !hm->debugParams)
    return 0;

  try
  {
    auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
    auto &solver = renderSystem->getBarrierSolver();

    // Bind ifftFinalTex as the sole UAV and transition it to Uav-Write.
    auto &t1 = solver.getNewResourceTransitionsArrayTmp();
    solver.resolveTransition(t1, hm->ifftFinalTex,
        Ogre::ResourceLayout::Uav, Ogre::ResourceAccess::Write,
        Ogre::c_computeStageMask);
    renderSystem->executeResourceTransition(t1);

    Ogre::DescriptorSetUav::TextureSlot s =
        Ogre::DescriptorSetUav::TextureSlot::makeEmpty();
    s.texture     = hm->ifftFinalTex;
    s.access      = Ogre::ResourceAccess::Write;
    s.pixelFormat = Ogre::PFG_RGBA32_FLOAT;
    hm->debugJob->_setUavTexture(0u, s);

    DebugParams p{};
    p.t         = _simTimeS;
    p.amplitude = _amplitude;
    p.gridSize  = static_cast<int>(hm->gridSize);
    hm->debugParams->upload(&p, 0u, sizeof(p));
    hm->hlmsCompute->dispatch(hm->debugJob, hm->sceneManager, nullptr);

    // Transition back to Texture so the visual can sample it.
    auto &t2 = solver.getNewResourceTransitionsArrayTmp();
    solver.resolveTransition(t2, hm->ifftFinalTex,
        Ogre::ResourceLayout::Texture, Ogre::ResourceAccess::Read,
        Ogre::c_allGraphicStagesMask);
    renderSystem->executeResourceTransition(t2);
  }
  catch (const Ogre::Exception &e)
  {
    static bool logged = false;
    if (!logged)
    {
      logged = true;
      gzerr << "[waves_ogre2_heightmap] debug dispatch threw: "
            << e.getDescription() << std::endl;
    }
    return 0;
  }
  return 1;
}

int waves_ogre2_heightmap_view_hkt_dispatch(
    waves_heightmap_t _handle, const char *_shaderAbsPath, float _scale)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->ifftFinalTex || !hm->hktTex || !hm->sceneManager ||
      !_shaderAbsPath)
    return 0;

  if (!hm->viewHktJob)
  {
    try
    {
      auto &rgMgr = Ogre::ResourceGroupManager::getSingleton();
      const std::filesystem::path absPath(_shaderAbsPath);
      const std::string dir = absPath.parent_path().string();
      hm->viewHktShaderName = absPath.stem().string();
      try
      {
        rgMgr.addResourceLocation(
            dir, "FileSystem",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            false);
      }
      catch (const Ogre::Exception &) { /* already added */ }

      auto *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
      hm->hlmsCompute = hlmsManager->getComputeHlms();
      if (!hm->hlmsCompute)
        return 0;

      const std::string jobName =
          "WavesViewHkt_" +
          std::to_string(reinterpret_cast<std::uintptr_t>(hm));
      hm->viewHktJob = hm->hlmsCompute->createComputeJob(
          jobName, jobName, hm->viewHktShaderName,
          Ogre::StringVector{});
      if (!hm->viewHktJob)
        return 0;

      hm->viewHktJob->setThreadsPerGroup(16u, 16u, 1u);
      const Ogre::uint32 groups =
          static_cast<Ogre::uint32>((hm->gridSize + 15) / 16);
      hm->viewHktJob->setNumThreadGroups(groups, groups, 1u);
      hm->viewHktJob->setNumUavUnits(1u);
      hm->viewHktJob->setNumTexUnits(1u);

      auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
      auto *vaoManager   = renderSystem->getVaoManager();
      hm->viewHktParams = vaoManager->createConstBuffer(
          sizeof(ViewHktParams),
          Ogre::BT_DYNAMIC_PERSISTENT,
          nullptr, false);
      hm->viewHktJob->setConstBuffer(0u, hm->viewHktParams);

      gzmsg << "[waves_ogre2_heightmap] view-hkt pipeline online — "
            << "copies |h(k,t)| · " << _scale << " into ifftFinalTex"
            << std::endl;
    }
    catch (const Ogre::Exception &e)
    {
      gzerr << "[waves_ogre2_heightmap] view-hkt setup threw: "
            << e.getDescription() << std::endl;
      hm->viewHktJob = nullptr;
      return 0;
    }
  }

  if (!hm->viewHktJob || !hm->viewHktParams)
    return 0;

  try
  {
    auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
    auto &solver = renderSystem->getBarrierSolver();

    auto &t1 = solver.getNewResourceTransitionsArrayTmp();
    solver.resolveTransition(t1, hm->hktTex,
        Ogre::ResourceLayout::Texture, Ogre::ResourceAccess::Read,
        Ogre::c_computeStageMask);
    solver.resolveTransition(t1, hm->ifftFinalTex,
        Ogre::ResourceLayout::Uav, Ogre::ResourceAccess::Write,
        Ogre::c_computeStageMask);
    renderSystem->executeResourceTransition(t1);

    // UAV slot 0 = output, texture slot 0 = input.
    Ogre::DescriptorSetUav::TextureSlot uavSlot =
        Ogre::DescriptorSetUav::TextureSlot::makeEmpty();
    uavSlot.texture     = hm->ifftFinalTex;
    uavSlot.access      = Ogre::ResourceAccess::Write;
    uavSlot.pixelFormat = Ogre::PFG_RGBA32_FLOAT;
    hm->viewHktJob->_setUavTexture(0u, uavSlot);

    Ogre::DescriptorSetTexture2::TextureSlot texSlot =
        Ogre::DescriptorSetTexture2::TextureSlot::makeEmpty();
    texSlot.texture = hm->hktTex;
    hm->viewHktJob->setTexture(0u, texSlot, &hm->samplerblock);

    ViewHktParams p{};
    p.scale    = _scale;
    p.gridSize = static_cast<int>(hm->gridSize);
    hm->viewHktParams->upload(&p, 0u, sizeof(p));
    hm->hlmsCompute->dispatch(hm->viewHktJob, hm->sceneManager, nullptr);

    auto &t2 = solver.getNewResourceTransitionsArrayTmp();
    solver.resolveTransition(t2, hm->ifftFinalTex,
        Ogre::ResourceLayout::Texture, Ogre::ResourceAccess::Read,
        Ogre::c_allGraphicStagesMask);
    renderSystem->executeResourceTransition(t2);
  }
  catch (const Ogre::Exception &e)
  {
    static bool logged = false;
    if (!logged)
    {
      logged = true;
      gzerr << "[waves_ogre2_heightmap] view-hkt dispatch threw: "
            << e.getDescription() << std::endl;
    }
    return 0;
  }
  return 1;
}

int waves_ogre2_heightmap_combine_dispatch(
    waves_heightmap_t _handle,
    const char *_combineEtaDxShaderAbsPath,
    const char *_combineDyShaderAbsPath)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->sceneManager || !hm->ifftFinalTex || !hm->ifftDyFinalTex
      || !_combineEtaDxShaderAbsPath || !_combineDyShaderAbsPath)
    return 0;

  if (!hm->combinedTex)
  {
    try
    {
      hm->combinedTexName =
          "WavesCombined_" +
          std::to_string(reinterpret_cast<std::uintptr_t>(hm));
      hm->combinedTex = MakeSpectrumTexture(
          hm->manager, hm->combinedTexName,
          hm->gridSize, Ogre::PFG_RGBA32_FLOAT);
    }
    catch (const Ogre::Exception &e)
    {
      gzerr << "[waves_ogre2_heightmap] combinedTex alloc threw: "
            << e.getDescription() << std::endl;
      hm->combinedTex = nullptr;
      return 0;
    }
  }

  if (!hm->combineEtaDxJob || !hm->combineDyJob)
  {
    try
    {
      auto &rgMgr = Ogre::ResourceGroupManager::getSingleton();
      const std::filesystem::path etaDxPath(_combineEtaDxShaderAbsPath);
      const std::filesystem::path dyPath(_combineDyShaderAbsPath);
      hm->combineEtaDxShaderName = etaDxPath.stem().string();
      hm->combineDyShaderName    = dyPath.stem().string();
      for (const auto &dir : {etaDxPath.parent_path().string(),
                              dyPath.parent_path().string()})
      {
        try
        {
          rgMgr.addResourceLocation(
              dir, "FileSystem",
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
              false);
        }
        catch (const Ogre::Exception &) {}
      }

      auto *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
      hm->hlmsCompute = hlmsManager->getComputeHlms();
      if (!hm->hlmsCompute)
        return 0;

      const Ogre::uint32 groups =
          static_cast<Ogre::uint32>((hm->gridSize + 15) / 16);

      auto makeCombineJob = [&](const std::string &shaderName,
                                 const std::string &jobBaseName,
                                 Ogre::TextureGpu *src,
                                 Ogre::ResourceAccess::ResourceAccess uavAccess)
          -> Ogre::HlmsComputeJob *
      {
        const std::string jobName =
            jobBaseName + "_" +
            std::to_string(reinterpret_cast<std::uintptr_t>(hm));
        Ogre::HlmsComputeJob *job = hm->hlmsCompute->createComputeJob(
            jobName, jobName, shaderName, Ogre::StringVector{});
        if (!job) return nullptr;
        job->setThreadsPerGroup(16u, 16u, 1u);
        job->setNumThreadGroups(groups, groups, 1u);
        job->setNumUavUnits(1u);
        job->setNumTexUnits(1u);
        Ogre::DescriptorSetUav::TextureSlot uavSlot =
            Ogre::DescriptorSetUav::TextureSlot::makeEmpty();
        uavSlot.texture     = hm->combinedTex;
        uavSlot.access      = uavAccess;
        uavSlot.pixelFormat = Ogre::PFG_RGBA32_FLOAT;
        job->_setUavTexture(0u, uavSlot);
        Ogre::DescriptorSetTexture2::TextureSlot texSlot =
            Ogre::DescriptorSetTexture2::TextureSlot::makeEmpty();
        texSlot.texture = src;
        job->setTexture(0u, texSlot, &hm->samplerblock);
        return job;
      };

      // First pass writes (η, Dx, 0, 0) — UAV writeonly.
      hm->combineEtaDxJob = makeCombineJob(
          hm->combineEtaDxShaderName, "WavesCombineEtaDx",
          hm->ifftFinalTex, Ogre::ResourceAccess::Write);
      // Second pass reads+writes combinedTex to add Dy.
      hm->combineDyJob = makeCombineJob(
          hm->combineDyShaderName, "WavesCombineDy",
          hm->ifftDyFinalTex, Ogre::ResourceAccess::ReadWrite);

      if (!hm->combineEtaDxJob || !hm->combineDyJob)
      {
        gzerr << "[waves_ogre2_heightmap] combine job creation failed"
              << std::endl;
        return 0;
      }

      gzmsg << "[waves_ogre2_heightmap] combine pipeline initialised "
            << "(eta+Dx=" << hm->combineEtaDxShaderName
            << ", Dy=" << hm->combineDyShaderName << ")" << std::endl;
    }
    catch (const Ogre::Exception &e)
    {
      gzerr << "[waves_ogre2_heightmap] combine setup threw: "
            << e.getDescription() << std::endl;
      return 0;
    }
  }

  hm->combinedTex->scheduleTransitionTo(
      Ogre::GpuResidency::Resident, nullptr);

  try
  {
    auto *renderSystem = Ogre::Root::getSingleton().getRenderSystem();
    auto &solver = renderSystem->getBarrierSolver();

    static constexpr unsigned GL_SHADER_IMAGE_ACCESS_BARRIER_BIT_VAL =
        0x00000020u;
    static constexpr unsigned GL_TEXTURE_FETCH_BARRIER_BIT_VAL =
        0x00000008u;
    using PFN_glMemoryBarrier = void (*)(unsigned);
    static auto glMemoryBarrierPtr =
        reinterpret_cast<PFN_glMemoryBarrier>(
            dlsym(RTLD_DEFAULT, "glMemoryBarrier"));

    // Pass 1: read ifftFinalTex, write (η, Dx, 0, 0) to combinedTex.
    {
      auto &t = solver.getNewResourceTransitionsArrayTmp();
      solver.resolveTransition(t, hm->ifftFinalTex,
          Ogre::ResourceLayout::Texture,
          Ogre::ResourceAccess::Read,
          Ogre::c_computeStageMask);
      solver.resolveTransition(t, hm->combinedTex,
          Ogre::ResourceLayout::Uav,
          Ogre::ResourceAccess::Write,
          Ogre::c_computeStageMask);
      renderSystem->executeResourceTransition(t);
      hm->hlmsCompute->dispatch(hm->combineEtaDxJob,
                                hm->sceneManager, nullptr);
      if (glMemoryBarrierPtr)
      {
        glMemoryBarrierPtr(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT_VAL |
                           GL_TEXTURE_FETCH_BARRIER_BIT_VAL);
      }
    }

    // Pass 2: read ifftDyFinalTex + combinedTex (RMW), write Dy
    // into combinedTex.b.
    {
      auto &t = solver.getNewResourceTransitionsArrayTmp();
      solver.resolveTransition(t, hm->ifftDyFinalTex,
          Ogre::ResourceLayout::Texture,
          Ogre::ResourceAccess::Read,
          Ogre::c_computeStageMask);
      solver.resolveTransition(t, hm->combinedTex,
          Ogre::ResourceLayout::Uav,
          Ogre::ResourceAccess::ReadWrite,
          Ogre::c_computeStageMask);
      renderSystem->executeResourceTransition(t);
      hm->hlmsCompute->dispatch(hm->combineDyJob,
                                hm->sceneManager, nullptr);
      if (glMemoryBarrierPtr)
      {
        glMemoryBarrierPtr(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT_VAL |
                           GL_TEXTURE_FETCH_BARRIER_BIT_VAL);
      }
    }

    if (!hm->combinedTex->isDataReady())
      hm->combinedTex->notifyDataIsReady();

    // Hand combinedTex back to the graphics pipeline as a sampler.
    {
      auto &t = solver.getNewResourceTransitionsArrayTmp();
      solver.resolveTransition(t, hm->combinedTex,
          Ogre::ResourceLayout::Texture,
          Ogre::ResourceAccess::Read,
          Ogre::c_allGraphicStagesMask);
      renderSystem->executeResourceTransition(t);
    }

    // First successful combine: swap the material's heightMap sampler
    // from the (now retired) CPU upload texture to combinedTex.
    if (!hm->ifftBoundToMaterial && hm->ogreMaterial && hm->combinedTex)
    {
      auto *pass = hm->ogreMaterial->getTechnique(0u)->getPass(0u);
      Ogre::TextureUnitState *texUnit = nullptr;
      for (unsigned int i = 0; i < pass->getNumTextureUnitStates(); ++i)
      {
        auto *u = pass->getTextureUnitState(i);
        if (u->getName() == "heightMap") { texUnit = u; break; }
      }
      if (texUnit)
      {
        texUnit->setTexture(hm->combinedTex);
        texUnit->setSamplerblock(hm->samplerblock);
        hm->ifftBoundToMaterial = true;
        gzmsg << "[waves_ogre2_heightmap] heightMap sampler swapped to "
              << "combinedTex (" << hm->combinedTex->getNameStr()
              << "); GPU chop displacement online" << std::endl;
      }
    }
  }
  catch (const Ogre::Exception &e)
  {
    static bool logged = false;
    if (!logged)
    {
      logged = true;
      gzerr << "[waves_ogre2_heightmap] combine dispatch threw: "
            << e.getDescription() << " (further failures suppressed)"
            << std::endl;
    }
    return 0;
  }
  return 1;
}

int waves_ogre2_heightmap_ready(waves_heightmap_t _handle)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  return (hm && hm->ready) ? 1 : 0;
}

int waves_ogre2_heightmap_set_tex_filtering(
    waves_heightmap_t _handle, const char *_texUnitName)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->ogreMaterial || !_texUnitName)
    return 0;
  try
  {
    auto *pass = hm->ogreMaterial->getTechnique(0u)->getPass(0u);
    Ogre::TextureUnitState *texUnit = nullptr;
    for (unsigned int i = 0; i < pass->getNumTextureUnitStates(); ++i)
    {
      auto *u = pass->getTextureUnitState(i);
      if (u->getName() == _texUnitName)
      {
        texUnit = u;
        break;
      }
    }
    if (!texUnit)
      return 0;
    Ogre::HlmsSamplerblock sb;
    sb.setFiltering(Ogre::TFO_ANISOTROPIC);
    sb.mMaxAnisotropy = 16.0f;
    sb.mU = Ogre::TAM_WRAP;
    sb.mV = Ogre::TAM_WRAP;
    sb.mW = Ogre::TAM_WRAP;
    texUnit->setSamplerblock(sb);
    gzmsg << "[waves_ogre2_heightmap] tex unit '" << _texUnitName
          << "' filtering set to anisotropic+trilinear" << std::endl;
    return 1;
  }
  catch (const Ogre::Exception &e)
  {
    gzerr << "[waves_ogre2_heightmap] set_tex_filtering threw: "
          << e.getDescription() << std::endl;
    return 0;
  }
}

void waves_ogre2_heightmap_destroy(waves_heightmap_t _handle)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm)
    return;
  // Tolerate Ogre having already torn the texture down (e.g. on Ctrl-C the
  // engine shuts down its TextureGpuManager before our owners run their
  // destructors). Otherwise the unhandled exception escapes through the
  // extern "C" boundary and aborts the process.
  if (hm->hlmsCompute && hm->computeJob)
  {
    try { hm->hlmsCompute->destroyComputeJob(hm->computeJob->getName()); }
    catch (const Ogre::Exception &) {}
    hm->computeJob = nullptr;
  }
  if (hm->paramsBuffer)
  {
    try
    {
      auto *renderSystem = Ogre::Root::getSingletonPtr() ?
          Ogre::Root::getSingleton().getRenderSystem() : nullptr;
      if (renderSystem && renderSystem->getVaoManager())
        renderSystem->getVaoManager()->destroyConstBuffer(hm->paramsBuffer);
    }
    catch (const Ogre::Exception &) {}
    hm->paramsBuffer = nullptr;
  }

  // Stage 2 evolve job + spectrum textures cleanup.
  for (auto **job : {&hm->evolveJob, &hm->evolveDyJob,
                      &hm->combineEtaDxJob, &hm->combineDyJob})
  {
    if (hm->hlmsCompute && *job)
    {
      try { hm->hlmsCompute->destroyComputeJob((*job)->getName()); }
      catch (const Ogre::Exception &) {}
      *job = nullptr;
    }
  }
  {
    auto *renderSystem = Ogre::Root::getSingletonPtr() ?
        Ogre::Root::getSingleton().getRenderSystem() : nullptr;
    auto *vaoManager = renderSystem ? renderSystem->getVaoManager() : nullptr;
    for (auto **buf : {&hm->evolveParams, &hm->evolveDyParams})
    {
      if (*buf && vaoManager)
      {
        try { vaoManager->destroyConstBuffer(*buf); }
        catch (const Ogre::Exception &) {}
        *buf = nullptr;
      }
    }
  }
  for (auto **tex : {&hm->h0Tex, &hm->hktTex, &hm->hktTexDy,
                      &hm->combinedTex})
  {
    if (*tex && hm->manager)
    {
      try { hm->manager->destroyTexture(*tex); }
      catch (const Ogre::Exception &) {}
      *tex = nullptr;
    }
  }

  // Stage 3 IFFT pipeline teardown (both η+Dx and Dy pipelines).
  if (hm->hlmsCompute)
  {
    for (auto *jobs : {&hm->ifftPassJobs, &hm->ifftDyPassJobs})
    {
      for (auto *job : *jobs)
      {
        if (!job) continue;
        try { hm->hlmsCompute->destroyComputeJob(job->getName()); }
        catch (const Ogre::Exception &) {}
      }
      jobs->clear();
    }
    hm->ifftPassSrcDst.clear();
    hm->ifftDyPassSrcDst.clear();
  }
  if (!hm->ifftParams.empty())
  {
    try
    {
      auto *renderSystem = Ogre::Root::getSingletonPtr() ?
          Ogre::Root::getSingleton().getRenderSystem() : nullptr;
      auto *vaoManager = renderSystem ? renderSystem->getVaoManager()
                                       : nullptr;
      if (vaoManager)
      {
        for (auto *buf : hm->ifftParams)
        {
          if (buf)
            vaoManager->destroyConstBuffer(buf);
        }
      }
    }
    catch (const Ogre::Exception &) {}
    hm->ifftParams.clear();
  }
  for (auto **tex : {&hm->ifftBufA, &hm->ifftBufB,
                      &hm->ifftDyBufA, &hm->ifftDyBufB})
  {
    if (*tex && hm->manager)
    {
      try { hm->manager->destroyTexture(*tex); }
      catch (const Ogre::Exception &) {}
      *tex = nullptr;
    }
  }
  hm->ifftFinalTex   = nullptr;
  hm->ifftDyFinalTex = nullptr;
  if (hm->manager)
  {
    if (hm->staging)
    {
      try { hm->manager->removeStagingTexture(hm->staging); }
      catch (const Ogre::Exception &) { /* engine already torn down */ }
      hm->staging = nullptr;
    }
    if (hm->texture)
    {
      try
      {
        hm->manager->destroyTexture(hm->texture);
      }
      catch (const Ogre::Exception &e)
      {
        gzdbg << "[waves_ogre2_heightmap] destroyTexture threw during "
              << "shutdown (likely already destroyed): "
              << e.getDescription() << std::endl;
      }
    }
  }

  // Stage 6 procedural-Item + HlmsPbs datablock teardown.
  if (hm->sceneManager)
  {
    try
    {
      if (hm->pbsItem && hm->pbsNode)
        hm->pbsNode->detachObject(hm->pbsItem);
      if (hm->pbsItem)
      {
        hm->sceneManager->destroyItem(hm->pbsItem);
        hm->pbsItem = nullptr;
      }
      if (hm->pbsNode)
      {
        hm->sceneManager->destroySceneNode(hm->pbsNode);
        hm->pbsNode = nullptr;
      }
    }
    catch (const Ogre::Exception &) {}
  }
  if (hm->pbsDatablock)
  {
    try
    {
      auto *hlmsManager = Ogre::Root::getSingleton().getHlmsManager();
      auto *hlmsPbs = static_cast<Ogre::HlmsPbs *>(
          hlmsManager->getHlms(Ogre::HLMS_PBS));
      if (hlmsPbs)
        hlmsPbs->destroyDatablock(hm->pbsDatablockName);
    }
    catch (const Ogre::Exception &) {}
    hm->pbsDatablock = nullptr;
  }
  if (hm->pbsMeshV2)
  {
    try { Ogre::MeshManager::getSingleton().remove(hm->pbsMeshNameV2); }
    catch (const Ogre::Exception &) {}
    hm->pbsMeshV2.reset();
  }
  if (hm->pbsMeshV1)
  {
    try { Ogre::v1::MeshManager::getSingleton().remove(hm->pbsMeshNameV1); }
    catch (const Ogre::Exception &) {}
    hm->pbsMeshV1.reset();
  }

  delete hm;
}

}  // extern "C"
