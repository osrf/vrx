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

#include <OgreHlmsSamplerblock.h>
#include <OgrePass.h>
#include <OgrePixelFormatGpu.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreStagingTexture.h>
#include <OgreTechnique.h>
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>
#include <OgreTextureUnitState.h>

namespace
{
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

  auto *hm = new HeightMap();
  hm->gridSize = _gridSize;
  hm->manager = manager;
  hm->texture = manager->createOrRetrieveTexture(
      _name,
      Ogre::GpuPageOutStrategy::Discard,
      Ogre::TextureFlags::ManualTexture,
      Ogre::TextureTypes::Type2D);
  hm->texture->setResolution(
      static_cast<Ogre::uint32>(_gridSize),
      static_cast<Ogre::uint32>(_gridSize));
  hm->texture->setNumMipmaps(1u);
  // RGBA32F so we can pack (η, Dx, Dy, α) per texel. Alpha is unused for
  // now (reserved for a Jacobian/foam mask in a future stage).
  hm->texture->setPixelFormat(Ogre::PFG_RGBA32_FLOAT);
  hm->texture->scheduleTransitionTo(Ogre::GpuResidency::Resident);

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
  texUnit->setSamplerblock(hm->samplerblock);
  hm->ready = true;

  // Allocate one persistent staging texture sized for the full heightmap
  // and reuse it on every Upload(). Acquiring a fresh staging texture per
  // frame leaks them into Ogre's pool, which then trips the engine's
  // "Texture memory budget exceeded" stall and visibly hangs the GUI.
  hm->staging = hm->manager->getStagingTexture(
      static_cast<Ogre::uint32>(_gridSize),
      static_cast<Ogre::uint32>(_gridSize),
      1u, 1u, Ogre::PFG_RGBA32_FLOAT);

  // Zero the heightmap immediately so the shader doesn't sample garbage
  // GPU memory in the brief window between the texture going Resident and
  // the first frame's CPU upload. Without this the surface can flash to
  // arbitrary positions for a frame or two.
  if (hm->texture->getResidencyStatus() == Ogre::GpuResidency::Resident &&
      hm->staging)
  {
    hm->staging->startMapRegion();
    Ogre::TextureBox box =
        hm->staging->mapRegion(static_cast<Ogre::uint32>(_gridSize),
                               static_cast<Ogre::uint32>(_gridSize),
                               1u, 1u, Ogre::PFG_RGBA32_FLOAT);
    const std::size_t bytes =
        static_cast<std::size_t>(_gridSize) * _gridSize * 4u * sizeof(float);
    std::memset(box.at(0, 0, 0), 0, bytes);
    hm->staging->stopMapRegion();
    hm->staging->upload(box, hm->texture, 0u, nullptr, nullptr);
  }
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
  if (hm->texture->getResidencyStatus() != Ogre::GpuResidency::Resident)
    return 0;
  if (!hm->staging)
    return 0;

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
