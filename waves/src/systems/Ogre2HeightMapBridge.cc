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
  hm->texture->setPixelFormat(Ogre::PFG_R32_FLOAT);
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
  return hm;
}

int waves_ogre2_heightmap_upload(
    waves_heightmap_t _handle, const double *_grid, int _rows, int _cols)
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->ready || !hm->texture || !hm->manager || !_grid)
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

  Ogre::StagingTexture *staging = hm->manager->getStagingTexture(
      N, N, 1u, 1u, Ogre::PFG_R32_FLOAT);
  staging->startMapRegion();
  Ogre::TextureBox box =
      staging->mapRegion(N, N, 1u, 1u, Ogre::PFG_R32_FLOAT);

  for (int row = 0; row < N; ++row)
  {
    auto *dst = reinterpret_cast<float *>(box.at(0, row, 0));
    const double *src = _grid + static_cast<std::size_t>(row) * N;
    for (int col = 0; col < N; ++col)
      dst[col] = static_cast<float>(src[col]);
  }

  staging->stopMapRegion();
  staging->upload(box, hm->texture, 0u, nullptr, nullptr);
  hm->manager->removeStagingTexture(staging);
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
  if (hm->manager && hm->texture)
    hm->manager->destroyTexture(hm->texture);
  delete hm;
}

}  // extern "C"
