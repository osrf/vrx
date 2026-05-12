/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "HeightMapTexture.hh"

#include <dlfcn.h>

#include <mutex>

#include <gz/common/Console.hh>

#include "Ogre2HeightMapBridge.hh"

namespace gz::sim::systems
{

namespace
{
  // Function-pointer typedefs matching the bridge's C-ABI exactly.
  using CreateFn = waves_heightmap_t (*)(void *, void *, std::size_t,
                                          const char *);
  using UploadFn = int (*)(waves_heightmap_t, const double *, int, int);
  using ReadyFn  = int (*)(waves_heightmap_t);
  using DestroyFn = void (*)(waves_heightmap_t);

  struct BridgeApi
  {
    void      *handle{nullptr};
    CreateFn   create{nullptr};
    UploadFn   upload{nullptr};
    ReadyFn    ready{nullptr};
    DestroyFn  destroy{nullptr};
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
              << (dlerror() ? dlerror() : "unknown") << std::endl;
        return;
      }
      api.create  = reinterpret_cast<CreateFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_create"));
      api.upload  = reinterpret_cast<UploadFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_upload"));
      api.ready   = reinterpret_cast<ReadyFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_ready"));
      api.destroy = reinterpret_cast<DestroyFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_destroy"));
      api.loaded = api.create && api.upload && api.ready && api.destroy;
      if (!api.loaded)
      {
        gzerr << "[HeightMapTexture] bridge is missing one or more C-ABI "
              << "symbols" << std::endl;
      }
    });
    return api;
  }
}

class HeightMapTexture::Impl
{
public:
  waves_heightmap_t handle{nullptr};
};

HeightMapTexture::HeightMapTexture(gz::rendering::ScenePtr _scene,
                                   gz::rendering::MaterialPtr _material,
                                   std::size_t _gridSize,
                                   const std::string &_textureName)
  : impl_(std::make_unique<Impl>())
  , gridSize_(_gridSize)
{
  const auto &api = LoadBridge();
  if (!api.loaded)
    return;
  this->impl_->handle = api.create(
      _scene.get(), _material.get(), _gridSize, _textureName.c_str());
  this->ready_ = (this->impl_->handle != nullptr) &&
                 (api.ready(this->impl_->handle) != 0);
}

HeightMapTexture::~HeightMapTexture()
{
  if (!this->impl_ || !this->impl_->handle)
    return;
  const auto &api = LoadBridge();
  if (api.loaded)
    api.destroy(this->impl_->handle);
}

bool HeightMapTexture::Upload(const Eigen::MatrixXd &_grid)
{
  if (!this->ready_ || !this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded)
    return false;
  const int N = static_cast<int>(this->gridSize_);
  if (_grid.rows() != N || _grid.cols() != N)
  {
    gzerr << "[HeightMapTexture] grid size mismatch (got "
          << _grid.rows() << "x" << _grid.cols()
          << ", expected " << N << "x" << N << ")" << std::endl;
    return false;
  }
  // Eigen defaults to column-major storage. The bridge expects a
  // row-major view of the matrix (rows contiguous in memory). Copy into a
  // local row-major buffer first so the bridge can iterate naturally.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      rowMajor = _grid;
  return api.upload(this->impl_->handle, rowMajor.data(), N, N) != 0;
}

}  // namespace gz::sim::systems
