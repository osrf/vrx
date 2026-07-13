/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

// Acknowledgement: the Ogre Next displacement-map streaming approach used here
// (a CPU-retained RGBA32F texture, per-frame residency scheduling, manual mip-0
// upload) is adapted from the technique in asv_wave_sim by Rhys Mainwaring
// (github.com/srmainwaring/asv_wave_sim).

#include "Ogre2HeightMapBridge.hh"


#include <gz/common/Console.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ogre2/Ogre2Material.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>

#include <OgreCommon.h>
#include <OgreHlmsSamplerblock.h>
#include <OgrePass.h>
#include <OgrePixelFormatGpu.h>
#include <OgreRenderSystem.h>
#include <OgreSceneManager.h>
#include <OgreStagingTexture.h>
#include <OgreTechnique.h>
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>
#include <OgreTextureUnitState.h>

namespace
{
  /// \brief Opaque handle behind waves_heightmap_t: the CPU-uploaded heightmap
  /// texture and its reused staging buffer, bound to a gz::rendering Material.
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

    // gz::rendering Material we're bound to, saved at create time so
    // set_tex_filtering can find the heightMap TextureUnitState and switch
    // its sampler to anisotropic filtering.
    Ogre::Material         *ogreMaterial{nullptr};
  };
}

extern "C"
{

//////////////////////////////////////////////////
waves_heightmap_t waves_ogre2_heightmap_create(
    void *_scene, void *_material, std::size_t _gridSize, const char *_name)
// Function-try block: Ogre calls (createOrRetrieveTexture, setResolution,
// getStagingTexture, ...) can throw, and an exception escaping this
// extern "C" boundary is undefined behaviour. Failing the create (the caller
// logs and runs without the water heightmap) beats aborting the GUI.
try
{
  // `_material` is the gz::rendering Material to bind the heightmap to. A null
  // material is tolerated (the bind below is skipped), but every live caller
  // supplies one.
  if (!_scene || !_name || _gridSize == 0)
    return nullptr;

  auto *scene = static_cast<gz::rendering::Scene *>(_scene);
  auto *material = static_cast<gz::rendering::Material *>(_material);

  auto *ogreScene = dynamic_cast<gz::rendering::Ogre2Scene *>(scene);
  if (!ogreScene)
  {
    gzerr << "[waves_ogre2_heightmap] scene is not an Ogre2Scene" << '\n';
    return nullptr;
  }
  auto *sceneManager = ogreScene->OgreSceneManager();
  if (!sceneManager)
  {
    gzerr << "[waves_ogre2_heightmap] no Ogre scene manager" << '\n';
    return nullptr;
  }

  auto *manager =
      sceneManager->getDestinationRenderSystem()->getTextureGpuManager();
  if (!manager)
  {
    gzerr << "[waves_ogre2_heightmap] no Ogre TextureGpuManager" << '\n';
    return nullptr;
  }

  // Bump Ogre Next's staging-texture budget so multiple streaming textures
  // (heightMap, bumpMap, cubeMap) can coexist without tripping
  // "Texture memory budget exceeded. Stalling GPU." early in scene load.
  // Default in 2.3.x is conservative for a scene with several streaming
  // textures.
  manager->setStagingTextureMaxBudgetBytes(std::size_t{256} * 1024 * 1024);  // 256 MB

  auto *hm = new HeightMap();
  hm->gridSize = _gridSize;
  hm->manager = manager;
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
  // RGBA32F so we can pack (η, Dx, Dy, foam) per texel. Alpha carries the
  // displacement-Jacobian folding/foam metric, written in the upload path.
  hm->texture->setPixelFormat(Ogre::PFG_RGBA32_FLOAT);
  // Don't schedule residency at creation time — asv_wave_sim does it from
  // the upload path so the call is repeated every frame, which seems to
  // keep the engine's state machine "alive" through the streaming flow.

  hm->samplerblock.setFiltering(Ogre::TFO_BILINEAR);
  hm->samplerblock.mU = Ogre::TAM_WRAP;
  hm->samplerblock.mV = Ogre::TAM_WRAP;
  hm->samplerblock.mW = Ogre::TAM_WRAP;

  // Bind the heightmap to the gz::rendering Material's v1 pass when one was
  // supplied.
  if (material)
  {
    auto *ogreMat = dynamic_cast<gz::rendering::Ogre2Material *>(material);
    if (!ogreMat || !ogreMat->Material())
    {
      gzerr << "[waves_ogre2_heightmap] material is not an Ogre2Material"
            << '\n';
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
catch (const Ogre::Exception &e)
{
  gzerr << "[waves_ogre2_heightmap] create threw: " << e.getDescription()
        << '\n';
  return nullptr;
}
catch (const std::exception &e)
{
  gzerr << "[waves_ogre2_heightmap] create threw: " << e.what() << '\n';
  return nullptr;
}

//////////////////////////////////////////////////
int waves_ogre2_heightmap_upload(
    waves_heightmap_t _handle, const double *_eta, const double *_dx,
    const double *_dy, const double *_foam, int _rows, int _cols)
// Function-try block: see waves_ogre2_heightmap_create. A failed upload
// returns 0 and the frame simply keeps the previous heightmap contents.
try
{
  auto *hm = static_cast<HeightMap *>(_handle);
  if (!hm || !hm->ready || !hm->texture || !hm->manager || !_eta)
    return 0;
  const int N = static_cast<int>(hm->gridSize);
  if (_rows != N || _cols != N)
  {
    gzerr << "[waves_ogre2_heightmap] grid size mismatch (" << _rows << "x"
          << _cols << ", expected " << N << "x" << N << ")" << '\n';
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
  const Ogre::TextureBox box =
      hm->staging->mapRegion(N, N, 1u, 1u, Ogre::PFG_RGBA32_FLOAT);

  // Pack one RGBA32F texel per (row, col): (η, Dx, Dy, foam). Inputs are the
  // column-major WaveField2D grids (element (i, j) = world (x_i, y_j) at index
  // i + j*N), so the contiguous slice [row*N, row*N + N) is the fixed-y_row
  // line of the field, and texel (u=col, v=row) receives grid value
  // (x_col, y_row) — the orientation the vertex shader's
  // uv = worldXY / tileSize sampling expects. Null dx/dy/foam channels pack
  // as zeros.
  for (int row = 0; row < N; ++row)
  {
    auto *dst = reinterpret_cast<float *>(box.at(0, row, 0));
    const double *seta = _eta + static_cast<std::size_t>(row) * N;
    const double *sdx  = _dx
        ? _dx + static_cast<std::size_t>(row) * N : nullptr;
    const double *sdy  = _dy
        ? _dy + static_cast<std::size_t>(row) * N : nullptr;
    const double *sfoam = _foam
        ? _foam + static_cast<std::size_t>(row) * N : nullptr;
    for (int col = 0; col < N; ++col)
    {
      dst[col * 4 + 0] = static_cast<float>(seta[col]);
      dst[col * 4 + 1] = sdx ? static_cast<float>(sdx[col]) : 0.0f;
      dst[col * 4 + 2] = sdy ? static_cast<float>(sdy[col]) : 0.0f;
      dst[col * 4 + 3] = sfoam ? static_cast<float>(sfoam[col]) : 0.0f;
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
catch (const Ogre::Exception &e)
{
  gzerr << "[waves_ogre2_heightmap] upload threw: " << e.getDescription()
        << '\n';
  return 0;
}
catch (const std::exception &e)
{
  gzerr << "[waves_ogre2_heightmap] upload threw: " << e.what() << '\n';
  return 0;
}

//////////////////////////////////////////////////
int waves_ogre2_heightmap_ready(waves_heightmap_t _handle)
{
  const auto *hm = static_cast<const HeightMap *>(_handle);
  return (hm && hm->ready) ? 1 : 0;
}

//////////////////////////////////////////////////
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
          << "' filtering set to anisotropic+trilinear" << '\n';
    return 1;
  }
  catch (const Ogre::Exception &e)
  {
    gzerr << "[waves_ogre2_heightmap] set_tex_filtering threw: "
          << e.getDescription() << '\n';
    return 0;
  }
}

//////////////////////////////////////////////////
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
      try
      {
        hm->manager->removeStagingTexture(hm->staging);
      }
      catch (const Ogre::Exception &e)
      {
        gzdbg << "[waves_ogre2_heightmap] removeStagingTexture threw during "
              << "shutdown (likely already destroyed): "
              << e.getDescription() << '\n';
      }
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
              << e.getDescription() << '\n';
      }
    }
  }

  delete hm;
}

}  // extern "C"
