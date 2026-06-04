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
                            const double *, int, int);
  using ReadyFn  = int (*)(waves_heightmap_t);
  using UploadSlopeFn = int (*)(waves_heightmap_t,
                                 const double *, const double *,
                                 int, int);
  using UploadChopDerivsFn = int (*)(waves_heightmap_t,
                                      const double *, const double *,
                                      const double *, int, int);
  using ComputeFn = int (*)(waves_heightmap_t, const char *, float, float);
  using PbsVisualFn = int (*)(waves_heightmap_t, double, int,
                               double, double, double, const char *);
  using UploadSpectrumFn = int (*)(waves_heightmap_t,
                                    const double *, const double *,
                                    const double *, const double *,
                                    int);
  using EvolveFn = int (*)(waves_heightmap_t, const char *,
                            float, float, float);
  using CombineFn = int (*)(waves_heightmap_t, const char *, const char *);
  using IfftFn = int (*)(waves_heightmap_t, const char *, const char *);
  using IfftNaiveFn = int (*)(waves_heightmap_t, const char *);
  using IfftBoundFn = int (*)(waves_heightmap_t);
  using CpuFeedFn = int (*)(waves_heightmap_t,
                             const double *, const double *,
                             const double *, int, int);
  using DebugFn = int (*)(waves_heightmap_t, const char *, float, float);
  using ViewHktFn = int (*)(waves_heightmap_t, const char *, float);
  using ReadbackH0Fn = int (*)(waves_heightmap_t, int, int,
                                float *, float *, float *, float *);
  using ReadbackIfftFn  = int (*)(waves_heightmap_t, int, int, float *);
  using ReadbackHktFn   = int (*)(waves_heightmap_t, int, int,
                                   float *, float *);
  using ReadbackCombinedScanFn = int (*)(waves_heightmap_t,
                                          int *, float *, float *,
                                          int *, int *, float *);
  using SetTexFilteringFn = int (*)(waves_heightmap_t, const char *);
  using DestroyFn = void (*)(waves_heightmap_t);

  struct BridgeApi
  {
    void      *handle{nullptr};
    CreateFn   create{nullptr};
    UploadFn   upload{nullptr};
    UploadSlopeFn uploadSlope{nullptr};
    UploadChopDerivsFn uploadChopDerivs{nullptr};
    ReadyFn    ready{nullptr};
    ComputeFn  compute{nullptr};
    PbsVisualFn pbsVisual{nullptr};
    UploadSpectrumFn uploadSpectrum{nullptr};
    EvolveFn   evolve{nullptr};
    EvolveFn   evolveDy{nullptr};
    CombineFn  combine{nullptr};
    IfftFn     ifft{nullptr};
    IfftNaiveFn ifftNaive{nullptr};
    IfftBoundFn ifftBound{nullptr};
    CpuFeedFn  cpuFeed{nullptr};
    DebugFn    debug{nullptr};
    ViewHktFn  viewHkt{nullptr};
    ReadbackH0Fn readbackH0{nullptr};
    ReadbackIfftFn  readbackIfft{nullptr};
    ReadbackHktFn   readbackHkt{nullptr};
    ReadbackCombinedScanFn readbackCombinedScan{nullptr};
    SetTexFilteringFn setTexFiltering{nullptr};
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
      api.uploadSlope = reinterpret_cast<UploadSlopeFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_upload_slope"));
      api.uploadChopDerivs = reinterpret_cast<UploadChopDerivsFn>(
          dlsym(api.handle,
                "waves_ogre2_heightmap_upload_chop_derivatives"));
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
      api.evolveDy = reinterpret_cast<EvolveFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_evolve_dy_dispatch"));
      api.combine = reinterpret_cast<CombineFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_combine_dispatch"));
      api.ifft = reinterpret_cast<IfftFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_ifft_dispatch"));
      api.ifftNaive = reinterpret_cast<IfftNaiveFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_ifft_naive_dispatch"));
      api.ifftBound = reinterpret_cast<IfftBoundFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_ifft_bound"));
      api.cpuFeed = reinterpret_cast<CpuFeedFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_cpu_feed"));
      api.debug = reinterpret_cast<DebugFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_debug_dispatch"));
      api.viewHkt = reinterpret_cast<ViewHktFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_view_hkt_dispatch"));
      api.readbackH0 = reinterpret_cast<ReadbackH0Fn>(
          dlsym(api.handle, "waves_ogre2_heightmap_readback_h0"));
      api.readbackIfft = reinterpret_cast<ReadbackIfftFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_readback_ifft"));
      api.readbackHkt = reinterpret_cast<ReadbackHktFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_readback_hkt"));
      api.readbackCombinedScan =
          reinterpret_cast<ReadbackCombinedScanFn>(dlsym(
              api.handle,
              "waves_ogre2_heightmap_readback_combined_scan"));
      api.setTexFiltering = reinterpret_cast<SetTexFilteringFn>(
          dlsym(api.handle, "waves_ogre2_heightmap_set_tex_filtering"));
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
                              const Eigen::MatrixXd &_dispY,
                              const Eigen::MatrixXd *_foam)
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
  // Optional folding / foam metric → the texture's alpha channel.
  RowMatrix foam;
  const double *foamData = nullptr;
  if (_foam && _foam->rows() == N && _foam->cols() == N)
  {
    foam = *_foam;
    foamData = foam.data();
  }
  return api.upload(this->impl_->handle,
                    eta.data(), dispX.data(), dispY.data(), foamData,
                    N, N) != 0;
}

bool HeightMapTexture::UploadSlope(const Eigen::MatrixXd &_slopeX,
                                    const Eigen::MatrixXd &_slopeY)
{
  if (!this->ready_ || !this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.uploadSlope)
    return false;
  const int N = static_cast<int>(this->gridSize_);
  if (_slopeX.rows() != N || _slopeX.cols() != N ||
      _slopeY.rows() != N || _slopeY.cols() != N)
    return false;
  using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                  Eigen::RowMajor>;
  RowMatrix sx = _slopeX;
  RowMatrix sy = _slopeY;
  return api.uploadSlope(this->impl_->handle,
                          sx.data(), sy.data(), N, N) != 0;
}

bool HeightMapTexture::UploadChopDerivatives(
    const Eigen::MatrixXd &_dDxDx,
    const Eigen::MatrixXd &_dDyDy,
    const Eigen::MatrixXd &_dDxDy)
{
  if (!this->ready_ || !this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.uploadChopDerivs)
    return false;
  const int N = static_cast<int>(this->gridSize_);
  if (_dDxDx.rows() != N || _dDxDx.cols() != N ||
      _dDyDy.rows() != N || _dDyDy.cols() != N ||
      _dDxDy.rows() != N || _dDxDy.cols() != N)
    return false;
  using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                  Eigen::RowMajor>;
  RowMatrix xx = _dDxDx;
  RowMatrix yy = _dDyDy;
  RowMatrix xy = _dDxDy;
  return api.uploadChopDerivs(this->impl_->handle,
                                xx.data(), yy.data(), xy.data(),
                                N, N) != 0;
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
    int _gridSize)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.uploadSpectrum)
    return false;
  return api.uploadSpectrum(this->impl_->handle,
                            _h0Re, _h0Im, _h0ConjRe, _h0ConjIm,
                            _gridSize) != 0;
}

bool HeightMapTexture::EvolveDispatch(const std::string &_shaderAbsPath,
                                      float _simTimeS, float _tauS,
                                      float _tileSizeM)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.evolve)
    return false;
  return api.evolve(this->impl_->handle, _shaderAbsPath.c_str(),
                    _simTimeS, _tauS, _tileSizeM) != 0;
}

bool HeightMapTexture::EvolveDyDispatch(const std::string &_shaderAbsPath,
                                        float _simTimeS, float _tauS,
                                        float _tileSizeM)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.evolveDy)
    return false;
  return api.evolveDy(this->impl_->handle, _shaderAbsPath.c_str(),
                      _simTimeS, _tauS, _tileSizeM) != 0;
}

bool HeightMapTexture::CombineDispatch(
    const std::string &_etaDxShaderAbsPath,
    const std::string &_dyShaderAbsPath)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.combine)
    return false;
  return api.combine(this->impl_->handle,
                     _etaDxShaderAbsPath.c_str(),
                     _dyShaderAbsPath.c_str()) != 0;
}

bool HeightMapTexture::IfftDispatch(
    const std::string &_bitrevShaderAbsPath,
    const std::string &_butterShaderAbsPath)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.ifft)
    return false;
  return api.ifft(this->impl_->handle,
                  _bitrevShaderAbsPath.c_str(),
                  _butterShaderAbsPath.c_str()) != 0;
}

bool HeightMapTexture::IfftNaiveDispatch(
    const std::string &_naiveShaderAbsPath)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.ifftNaive)
    return false;
  return api.ifftNaive(this->impl_->handle,
                       _naiveShaderAbsPath.c_str()) != 0;
}

bool HeightMapTexture::CpuFeed(const Eigen::MatrixXd &_eta,
                                const Eigen::MatrixXd &_dispX,
                                const Eigen::MatrixXd &_dispY)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.cpuFeed)
    return false;
  const int N = static_cast<int>(this->gridSize_);
  using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                  Eigen::RowMajor>;
  RowMatrix eta   = _eta;
  RowMatrix dispX = _dispX;
  RowMatrix dispY = _dispY;
  return api.cpuFeed(this->impl_->handle,
                     eta.data(), dispX.data(), dispY.data(), N, N) != 0;
}

bool HeightMapTexture::ReadbackH0Cell(int _i, int _j,
                                       float *_outRe, float *_outIm,
                                       float *_outConjRe,
                                       float *_outConjIm) const
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.readbackH0)
    return false;
  return api.readbackH0(this->impl_->handle, _i, _j,
                        _outRe, _outIm,
                        _outConjRe, _outConjIm) != 0;
}

bool HeightMapTexture::ReadbackIfftCell(int _i, int _j,
                                         float *_outEta) const
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.readbackIfft)
    return false;
  return api.readbackIfft(this->impl_->handle, _i, _j, _outEta) != 0;
}

bool HeightMapTexture::ReadbackHktCell(int _i, int _j,
                                        float *_outRe,
                                        float *_outIm) const
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.readbackHkt)
    return false;
  return api.readbackHkt(this->impl_->handle, _i, _j,
                         _outRe, _outIm) != 0;
}

bool HeightMapTexture::SetTexFiltering(const std::string &_texUnitName)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.setTexFiltering)
    return false;
  return api.setTexFiltering(this->impl_->handle,
                             _texUnitName.c_str()) != 0;
}

bool HeightMapTexture::ReadbackCombinedScan(
    int *_outBadCount,
    float *_outMinRgba, float *_outMaxRgba,
    int *_outFirstBadI, int *_outFirstBadJ,
    float *_outFirstBadRgba) const
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.readbackCombinedScan)
    return false;
  return api.readbackCombinedScan(this->impl_->handle,
                                   _outBadCount,
                                   _outMinRgba, _outMaxRgba,
                                   _outFirstBadI, _outFirstBadJ,
                                   _outFirstBadRgba) != 0;
}

bool HeightMapTexture::GpuOutputBound() const
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.ifftBound)
    return false;
  return api.ifftBound(this->impl_->handle) != 0;
}

bool HeightMapTexture::TestPatternDispatch(
    const std::string &_shaderAbsPath, float _simTimeS, float _amplitude)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.debug)
    return false;
  return api.debug(this->impl_->handle, _shaderAbsPath.c_str(),
                   _simTimeS, _amplitude) != 0;
}

bool HeightMapTexture::ViewHktDispatch(
    const std::string &_shaderAbsPath, float _scale)
{
  if (!this->impl_->handle)
    return false;
  const auto &api = LoadBridge();
  if (!api.loaded || !api.viewHkt)
    return false;
  return api.viewHkt(this->impl_->handle, _shaderAbsPath.c_str(),
                     _scale) != 0;
}

}  // namespace gz::sim::systems
