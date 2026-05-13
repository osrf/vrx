/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "Ogre2HeightMapBridge.hh"

#include <cstring>

#include <gz/common/Console.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ogre2/Ogre2Material.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>

#include <filesystem>

#include <OgreHlmsCompute.h>
#include <OgreHlmsComputeJob.h>
#include <OgreHlmsManager.h>
#include <OgreHlmsSamplerblock.h>
#include <OgrePass.h>
#include <OgrePixelFormatGpu.h>
#include <OgrePixelFormatGpuUtils.h>
#include <OgreRenderSystem.h>
#include <OgreResourceGroupManager.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreStagingTexture.h>
#include <OgreTechnique.h>
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>
#include <OgreTextureUnitState.h>
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
  };
}

extern "C"
{

waves_heightmap_t waves_ogre2_heightmap_create(
    void *_scene, void *_material, std::size_t _gridSize, const char *_name)
{
  if (!_scene || !_material || !_name || _gridSize == 0)
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
  // Full mipmap chain matches asv_wave_sim's configuration; the engine
  // appears to take a slow init path on textures with setNumMipmaps(1).
  hm->texture->setNumMipmaps(
      Ogre::PixelFormatGpuUtils::getMaxMipmapCount(
          hm->texture->getWidth(), hm->texture->getHeight()));
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

  // On OpenGL: explicitly bind the GLSL `sampler2D heightMap` uniform to
  // the texture unit index. Without this, Ogre Next's HlmsLowLevel path
  // can fall back to a slow program-introspection step on first render to
  // figure out the binding. Mirrors asv_wave_sim's pattern.
  const int texIndex =
      static_cast<int>(pass->getTextureUnitStateIndex(texUnit));
  auto ogreParams = pass->getVertexProgramParameters();
  if (ogreParams)
  {
    ogreParams->setNamedConstant("heightMap", &texIndex, 1, 1);
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
      const std::string dir  = absPath.parent_path().string();
      hm->computeShaderName  = absPath.filename().string();

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

  // Update the params buffer with the current frame's uniforms.
  ComputeParams params{};
  params.t        = _simTimeS;
  params.tileSize = _tileSizeM;
  params.gridSize = N;
  params._pad     = 0.0f;
  hm->paramsBuffer->upload(&params, 0u, sizeof(params));

  // Dispatch. Camera arg is null because our shader doesn't reference
  // any per-camera state.
  hm->hlmsCompute->dispatch(hm->computeJob, hm->sceneManager, nullptr);

  // Tell the engine the texture's data is now fresh.
  if (!hm->texture->isDataReady())
    hm->texture->notifyDataIsReady();
  return 1;
}

int waves_ogre2_heightmap_ready(waves_heightmap_t _handle)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  return (hm && hm->ready) ? 1 : 0;
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
  delete hm;
}

}  // extern "C"
