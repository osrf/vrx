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
  using UploadFn = int (*)(waves_heightmap_t,
                            const double *, const double *, const double *,
                            int, int);
  using ReadyFn  = int (*)(waves_heightmap_t);
  using ComputeFn = int (*)(waves_heightmap_t, const char *, float, float);
  using PbsVisualFn = int (*)(waves_heightmap_t, double, int,
                               double, double, double, const char *);
  using UploadSpectrumFn = int (*)(waves_heightmap_t,
                                    const double *, const double *,
                                    const double *, const double *,
                                    const double *, int);
  using EvolveFn = int (*)(waves_heightmap_t, const char *, float);
  using DestroyFn = void (*)(waves_heightmap_t);

  struct BridgeApi
  {
    void      *handle{nullptr};
    CreateFn   create{nullptr};
    UploadFn   upload{nullptr};
    ReadyFn    ready{nullptr};
    ComputeFn  compute{nullptr};
    PbsVisualFn pbsVisual{nullptr};
    UploadSpectrumFn uploadSpectrum{nullptr};
    EvolveFn   evolve{nullptr};
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
      api.compute = reinterpret_cast<ComputeFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_compute_dispatch"));
      api.pbsVisual = reinterpret_cast<PbsVisualFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_create_pbs_visual"));
      api.uploadSpectrum = reinterpret_cast<UploadSpectrumFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_upload_spectrum"));
      api.evolve = reinterpret_cast<EvolveFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_evolve_dispatch"));
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

bool HeightMapTexture::Upload(const Eigen::MatrixXd &_eta,
                              const Eigen::MatrixXd &_dispX,
                              const Eigen::MatrixXd &_dispY)
{
  if (!this->ready_ || !this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded)
    return false;
  const int N = static_cast<int>(this->gridSize_);
  if (_eta.rows() != N || _eta.cols() != N ||
      _dispX.rows() != N || _dispX.cols() != N ||
      _dispY.rows() != N || _dispY.cols() != N)
  {
    gzerr << "[HeightMapTexture] grid size mismatch (expected "
          << N << "x" << N << ")" << std::endl;
    return false;
  }
  // Eigen defaults to column-major storage. The bridge expects row-major
  // views (rows contiguous in memory), so reflow each matrix once into a
  // local row-major copy before forwarding.
  using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                  Eigen::RowMajor>;
  RowMatrix eta   = _eta;
  RowMatrix dispX = _dispX;
  RowMatrix dispY = _dispY;
  return api.upload(this->impl_->handle,
                    eta.data(), dispX.data(), dispY.data(), N, N) != 0;
}

bool HeightMapTexture::Dispatch(const std::string &_shaderAbsPath,
                                float _simTimeS, float _tileSizeM)
{
  if (!this->ready_ || !this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.compute)
    return false;
  return api.compute(this->impl_->handle, _shaderAbsPath.c_str(),
                     _simTimeS, _tileSizeM) != 0;
}

bool HeightMapTexture::CreatePbsVisual(double _planeSizeM,
                                       int _planeSegments,
                                       double _wx, double _wy, double _wz,
                                       const std::string &_name)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.pbsVisual)
    return false;
  return api.pbsVisual(this->impl_->handle, _planeSizeM, _planeSegments,
                       _wx, _wy, _wz, _name.c_str()) != 0;
}

bool HeightMapTexture::UploadSpectrum(
    const double *_h0Re, const double *_h0Im,
    const double *_h0ConjRe, const double *_h0ConjIm,
    const double *_omega, int _gridSize)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.uploadSpectrum)
    return false;
  return api.uploadSpectrum(this->impl_->handle,
                            _h0Re, _h0Im, _h0ConjRe, _h0ConjIm, _omega,
                            _gridSize) != 0;
}

bool HeightMapTexture::EvolveDispatch(const std::string &_shaderAbsPath,
                                      float _simTimeS)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.evolve)
    return false;
  return api.evolve(this->impl_->handle, _shaderAbsPath.c_str(),
                    _simTimeS) != 0;
}

}  // namespace gz::sim::systems
