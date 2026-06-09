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

#include <gz/common/Console.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ogre2/Ogre2Material.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>

#include <OgreCommon.h>
#include <OgreHlmsManager.h>
#include <OgreHlmsSamplerblock.h>
#include <OgreItem.h>
#include <OgreMesh2.h>
#include <OgreMeshManager.h>
#include <OgreMeshManager2.h>
#include <OgrePass.h>
#include <OgrePixelFormatGpu.h>
#include <OgreRenderSystem.h>
#include <OgreResourceGroupManager.h>
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

namespace
{
  // Opaque handle behind waves_heightmap_t: the CPU-uploaded heightmap
  // texture, its reused staging buffer, and the procedural HlmsPbs water
  // Item the visual renders.
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

    // Scene manager owning the procedural water Item below.
    Ogre::SceneManager     *sceneManager{nullptr};

    // Procedural-Item state (HlmsPbs path): the mesh, Item, scene node and
    // datablock created by create_pbs_visual and torn down by destroy.
    Ogre::SceneNode        *pbsNode{nullptr};
    Ogre::Item             *pbsItem{nullptr};
    Ogre::MeshPtr           pbsMeshV2;
    Ogre::v1::MeshPtr       pbsMeshV1;
    Ogre::HlmsPbsDatablock *pbsDatablock{nullptr};
    std::string             pbsMeshNameV1;
    std::string             pbsMeshNameV2;
    std::string             pbsDatablockName;

    // gz::rendering Material we're bound to, saved at create time so
    // set_tex_filtering can find the heightMap TextureUnitState and switch
    // its sampler to anisotropic filtering.
    Ogre::Material         *ogreMaterial{nullptr};
  };
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
    const double *_dy, const double *_foam, int _rows, int _cols)
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

  // Pack one RGBA32F texel per (row, col): (η, Dx, Dy, foam). `foam` carries
  // the displacement-Jacobian folding metric when supplied, else 0.
  for (int row = 0; row < N; ++row)
  {
    auto *dst = reinterpret_cast<float *>(box.at(0, row, 0));
    const double *seta = _eta + static_cast<std::size_t>(row) * N;
    const double *sdx  = _dx  + static_cast<std::size_t>(row) * N;
    const double *sdy  = _dy  + static_cast<std::size_t>(row) * N;
    const double *sfoam = _foam
        ? _foam + static_cast<std::size_t>(row) * N : nullptr;
    for (int col = 0; col < N; ++col)
    {
      dst[col * 4 + 0] = static_cast<float>(seta[col]);
      dst[col * 4 + 1] = static_cast<float>(sdx[col]);
      dst[col * 4 + 2] = static_cast<float>(sdy[col]);
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
