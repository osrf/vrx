/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "HeightMapTexture.hh"

#include <dlfcn.h>

#include <mutex>
#include <vector>

#include <gz/common/Console.hh>

#include "GridReflow.hh"
#include "Ogre2HeightMapBridge.hh"

namespace gz::sim::systems
{

namespace
{
  // Function-pointer typedefs matching the bridge's C-ABI exactly.
  using CreateFn = waves_heightmap_t (*)(void *, void *, std::size_t,
                                          const char *);
  using UploadFn = int (*)(waves_heightmap_t,
                            const double *, const double *, const double *,
                            const double *, int, int);
  using ReadyFn  = int (*)(waves_heightmap_t);
  using SetTexFilteringFn = int (*)(waves_heightmap_t, const char *);
  using DestroyFn = void (*)(waves_heightmap_t);

  /// \brief The dlopen'd waves-ogre2-bridge C-ABI: the library handle plus the
  /// resolved function pointers used to drive the heightmap texture.
  struct BridgeApi
  {
    /// \brief dlopen handle for libwaves-ogre2-bridge.so.
    void      *handle{nullptr};
    /// \brief Creates an Ogre2 heightmap bound to a scene + material.
    CreateFn   create{nullptr};
    /// \brief Uploads the latest height/displacement grids to the GPU texture.
    UploadFn   upload{nullptr};
    /// \brief Reports whether the GPU texture is resident and bound.
    ReadyFn    ready{nullptr};
    /// \brief Patches a named tex unit to anisotropic trilinear filtering.
    SetTexFilteringFn setTexFiltering{nullptr};
    /// \brief Destroys the Ogre2 heightmap.
    DestroyFn  destroy{nullptr};
    /// \brief True once the library and all required symbols resolved.
    bool       loaded{false};
  };

  /// Load libwaves-ogre2-bridge.so once, on demand. This is the only point
  /// where Ogre2/OgreNext symbols enter the process from our side; they
  /// stay confined to the bridge library so the WaterVisual plugin itself
  /// has no DT_NEEDED on libgz-rendering-ogre2.so.
  const BridgeApi &LoadBridge()
  {
    static BridgeApi api;
    static std::once_flag flag;
    std::call_once(flag, [&]() {
      api.handle = dlopen("libwaves-ogre2-bridge.so",
                          RTLD_NOW | RTLD_LOCAL);
      if (!api.handle)
      {
        gzerr << "[HeightMapTexture] failed to dlopen "
              << "libwaves-ogre2-bridge.so: "
              << (dlerror() ? dlerror() : "unknown") << '\n';
        return;
      }
      api.create  = reinterpret_cast<CreateFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_create"));
      api.upload  = reinterpret_cast<UploadFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_upload"));
      api.ready   = reinterpret_cast<ReadyFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_ready"));
      api.setTexFiltering = reinterpret_cast<SetTexFilteringFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_set_tex_filtering"));
      api.destroy = reinterpret_cast<DestroyFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_destroy"));
      api.loaded = api.create && api.upload && api.ready && api.destroy;
      if (!api.loaded)
      {
        gzerr << "[HeightMapTexture] bridge is missing one or more C-ABI "
              << "symbols" << '\n';
      }
    });
    return api;
  }
}

/// \brief Pimpl for HeightMapTexture: owns the bridge handle and forwards the
/// public calls across the C-ABI to the Ogre2 implementation.
class HeightMapTexture::Impl
{
  /// \brief Opaque handle to the Ogre2-side heightmap, owned across the C-ABI
  /// bridge; null until created, released in HeightMapTexture's destructor.
  public: waves_heightmap_t handle{nullptr};
};

//////////////////////////////////////////////////
HeightMapTexture::HeightMapTexture(const gz::rendering::ScenePtr &_scene,
                                   const gz::rendering::MaterialPtr &_material,
                                   std::size_t _gridSize,
                                   const std::string &_textureName)
  : impl(std::make_unique<Impl>())
  , gridSize(_gridSize)
{
  const auto &api = LoadBridge();
  if (!api.loaded)
    return;
  this->impl->handle = api.create(
      _scene.get(), _material.get(), _gridSize, _textureName.c_str());
  this->ready = (this->impl->handle != nullptr) &&
                 (api.ready(this->impl->handle) != 0);
}

//////////////////////////////////////////////////
HeightMapTexture::~HeightMapTexture()
{
  if (!this->impl || !this->impl->handle)
    return;
  const auto &api = LoadBridge();
  if (api.loaded)
    api.destroy(this->impl->handle);
}

//////////////////////////////////////////////////
bool HeightMapTexture::Upload(const double *_eta, const double *_dispX,
                              const double *_dispY, const double *_foam,
                              std::size_t _n)
{
  if (!this->ready || !this->impl->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded)
    return false;
  if (_n != this->gridSize)
  {
    gzerr << "[HeightMapTexture] grid size mismatch (expected "
          << this->gridSize << ", got " << _n << ")" << '\n';
    return false;
  }
  if (!_eta)
  {
    gzerr << "[HeightMapTexture] null elevation grid" << '\n';
    return false;
  }
  // Transpose each column-major grid into a local row-major copy (the bridge
  // expects rows contiguous). Absent displacement channels reflow to zeros.
  const std::vector<double> eta   = ToRowMajor(_eta, _n);
  const std::vector<double> dispX = ToRowMajor(_dispX, _n);
  const std::vector<double> dispY = ToRowMajor(_dispY, _n);
  // Optional folding / foam metric → the texture's alpha channel.
  std::vector<double> foam;
  const double *foamData = nullptr;
  if (_foam)
  {
    foam = ToRowMajor(_foam, _n);
    foamData = foam.data();
  }
  const int N = static_cast<int>(_n);
  return api.upload(this->impl->handle,
                    eta.data(), dispX.data(), dispY.data(), foamData,
                    N, N) != 0;
}

//////////////////////////////////////////////////
bool HeightMapTexture::SetTexFiltering(const std::string &_texUnitName)
{
  if (!this->impl->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.setTexFiltering)
    return false;
  return api.setTexFiltering(this->impl->handle,
                             _texUnitName.c_str()) != 0;
}

}  // namespace gz::sim::systems
