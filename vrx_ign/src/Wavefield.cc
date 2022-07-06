/*
 * Copyright (C) 2019  Rhys Mainwaring
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "Wavefield.hh"

using namespace ignition;
using namespace gazebo;

///////////////////////////////////////////////////////////////////////////////
// Utilities
std::ostream& operator<<(std::ostream &_os, const std::vector<double> &_vec)
{
  for (auto&& v : _vec ) // NOLINT
    _os << v << ", ";
  return _os;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Private data for the WavefieldParameters.
class ignition::gazebo::WavefieldPrivate
{
  /// \brief Constructor.
  public: WavefieldPrivate():
    size({1000, 1000}),
    cellCount({50, 50}),
    model("PMS"),
    number(1),
    scale(2),
    angle(2.0*M_PI/10.0),
    steepness(1.0),
    amplitude(0.0),
    period(1.0),
    phase(0.0),
    direction(1, 0),
    angularFrequency(2.0*M_PI),
    wavelength(2 * M_PI / this->DeepWaterDispersionToWavenumber(2.0 * M_PI)),
    wavenumber(this->DeepWaterDispersionToWavenumber(2.0 * M_PI)),
    tau(1.0),
    gain(1.0)
  {
  }

  /// \brief The size of the wavefield.
  public: ignition::math::Vector2d size;

  /// \brief The number of grid cells in the wavefield.
  public: ignition::math::Vector2d cellCount;

  /// \brief Name of wavefield model to use - must be "PMS" or "CWR"
  public: std::string model;

  /// \brief The number of component waves.
  public: size_t number;

  /// \brief Set the scale of the largest and smallest waves.
  public: double scale;

  /// \brief Set the angle between component waves and the mean direction.
  public: double angle;

  /// \brief Control the wave steepness. 0 is sine waves, 1 is Gerstner waves.
  public: double steepness;

  /// \brief The mean wave amplitude [m].
  public: double amplitude;

  /// \brief The mean wave period [s]
  public: double period;

  /// \brief The mean wave phase (not currently enabled).
  public: double phase;

  /// \brief The mean wave direction.
  public: ignition::math::Vector2d direction;

  /// \brief The time constant for exponential increasing waves on startup
  public: double tau;

  /// \brief The multiplier applied to PM spectra
  public: double gain;

  /// \brief The mean wave angular frequency (derived).
  public: double angularFrequency;

  /// \brief The mean wavelength (derived).
  public: double wavelength;

  /// \brief The mean wavenumber (derived).
  public: double wavenumber;

  /// \brief The component wave angular frequencies (derived).
  public: std::vector<double> angularFrequencies;

  /// \brief The component wave amplitudes (derived).
  public: std::vector<double> amplitudes;

  /// \brief The component wave phases (derived).
  public: std::vector<double> phases;

  /// \brief The component wave steepness factors (derived).
  public: std::vector<double> steepnesses;

  /// \brief The component wavenumbers (derived).
  public: std::vector<double> wavenumbers;

  /// \brief The component wave dirctions (derived).
  public: std::vector<ignition::math::Vector2d> directions;

  /// \brief Recalculate for constant wavelength-amplitude ratio
  public: void RecalculateCwr()
  {
    // Normalize direction
    this->direction.Normalize();

    // Derived mean values
    this->angularFrequency = 2.0 * M_PI / this->period;
    this->wavenumber = \
      this->DeepWaterDispersionToWavenumber(this->angularFrequency);
    this->wavelength = 2.0 * M_PI / this->wavenumber;

    // Update components
    this->angularFrequencies.clear();
    this->amplitudes.clear();
    this->phases.clear();
    this->wavenumbers.clear();
    this->steepnesses.clear();
    this->directions.clear();

    for (size_t i = 0; i < this->number; ++i)
    {
      const int n = i - this->number / 2;
      const double scaleFactor = std::pow(this->scale, n);
      const double a = scaleFactor * this->amplitude;
      const double k = this->wavenumber / scaleFactor;
      const double omega = this->DeepWaterDispersionToOmega(k);
      const double phi = this->phase;
      double q = 0.0;
      if (!ignition::math::equal(a, 0.0))
      {
        q = std::min(1.0, this->steepness / (a * k * this->number));
      }

      this->amplitudes.push_back(a);
      this->angularFrequencies.push_back(omega);
      this->phases.push_back(phi);
      this->steepnesses.push_back(q);
      this->wavenumbers.push_back(k);

      // Direction
      const double c = std::cos(n * this->angle);
      const double s = std::sin(n * this->angle);

      const ignition::math::Vector2d d(
        c * this->direction.X() - s * this->direction.Y(),
        s * this->direction.X() + c * this->direction.Y());
      directions.push_back(d);
    }
  }

  // \brief Pierson-Moskowitz wave spectrum
  public: double pm(double _omega, double _omegaP)
  {
    double alpha = 0.0081;
    double g = 9.81;
    return alpha * std::pow(g, 2.0) / std::pow(_omega, 5.0) * \
      std::exp(-(5.0 / 4.0) * std::pow(_omegaP / _omega, 4.0));
  }

  /// \brief Recalculate for Pierson-Moskowitz spectrum sampling model
  public: void RecalculatePms()
  {
    // Normalize direction
    this->direction.Normalize();

    // Derived mean values
    this->angularFrequency = 2.0 * M_PI / this->period;
    this->wavenumber = \
      this->DeepWaterDispersionToWavenumber(this->angularFrequency);
    this->wavelength = 2.0 * M_PI / this->wavenumber;

    // Update components
    this->angularFrequencies.clear();
    this->amplitudes.clear();
    this->phases.clear();
    this->wavenumbers.clear();
    this->steepnesses.clear();
    this->directions.clear();

    // Vector for spaceing
    std::vector<double> omegaSpacing;
    omegaSpacing.push_back(this->angularFrequency * (1.0 - 1.0 / this->scale));
    omegaSpacing.push_back(this->angularFrequency * \
                            (this->scale - 1.0 / this->scale) / 2.0);
    omegaSpacing.push_back(this->angularFrequency * (this->scale - 1.0));

    for (size_t i = 0; i < this->number; ++i)
    {
      const int n = i - 1;
      const double scaleFactor = std::pow(this->scale, n);
      const double omega = this->angularFrequency * scaleFactor;
      const double pms = pm(omega, this->angularFrequency);
      const double a = this->gain * std::sqrt(2.0 * pms * omegaSpacing[i]);
      const double k = this->DeepWaterDispersionToWavenumber(omega);
      const double phi = this->phase;
      double q = 0.0;
      if (!ignition::math::equal(a, 0.0))
      {
        q = std::min(1.0, this->steepness / (a * k * this->number));
      }

      this->amplitudes.push_back(a);
      this->angularFrequencies.push_back(omega);
      this->phases.push_back(phi);
      this->steepnesses.push_back(q);
      this->wavenumbers.push_back(k);

      // Direction
      const double c = std::cos(n * this->angle);
      const double s = std::sin(n * this->angle);

      const ignition::math::Vector2d d(
        c * this->direction.X() - s * this->direction.Y(),
        s * this->direction.X() + c * this->direction.Y());
      directions.push_back(d);
    }
  }

  /// \brief Recalculate all derived quantities from inputs.
  public: void Recalculate()
  {
    if (!this->model.compare("PMS"))
    {
      ignmsg << "Using Pierson-Moskowitz spectrum sampling wavefield model "
            << std::endl;
      this->RecalculatePms();
    }
    else if (!this->model.compare("CWR"))
    {
      ignmsg << "Using Constant wavelength-ampltude ratio wavefield model "
            << std::endl;
      this->RecalculateCwr();
    }
    else
    {
      ignwarn<< "Wavefield model specified as <" << this->model
            << "> which is not one of the two supported wavefield models: "
            << "PMS or CWR!!!" << std::endl;
    }
  }

  /////////////////////////////////////////////////
  private: double DeepWaterDispersionToOmega(double _wavenumber)
  {
    const double g = std::fabs(-9.8);
    return std::sqrt(g * _wavenumber);
  }

  /////////////////////////////////////////////////
  private: double DeepWaterDispersionToWavenumber(double _omega)
  {
    const double g = std::fabs(-9.8);
    return _omega * _omega / g;
  }
};

/////////////////////////////////////////////////////////////////////////////
Wavefield::Wavefield()
  : data(std::make_unique<WavefieldPrivate>())
{
  this->data->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
Wavefield::~Wavefield()
{
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::Load(const std::shared_ptr<const sdf::Element> &_sdf)
{
  if (!_sdf->HasElement("wavefield"))
    return;

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  auto sdfWavefield = ptr->GetElement("wavefield");

  this->data->size = sdfWavefield->Get<ignition::math::Vector2d>("size",
    this->data->size).first;
  this->data->cellCount = sdfWavefield->Get<ignition::math::Vector2d>("cell_count",
    this->data->cellCount).first;
  if (sdfWavefield->HasElement("wave"))
  {
    auto sdfWave = sdfWavefield->GetElement("wave");

    this->data->model = sdfWave->Get<std::string>("model", "PMS").first;
    this->data->number =
      sdfWave->Get<double>("number", this->data->number).first;
    this->data->amplitude =
      sdfWave->Get<double>("amplitude", this->data->amplitude).first;
    this->data->period =
      sdfWave->Get<double>("period", this->data->period).first;
    this->data->phase = sdfWave->Get<double>("phase", this->data->phase).first;
    this->data->direction =
      sdfWave->Get<ignition::math::Vector2d>("direction",
        this->data->direction).first;
    this->data->scale = sdfWave->Get<double>("scale", this->data->scale).first;
    this->data->angle = sdfWave->Get<double>("angle", this->data->angle).first;
    this->data->steepness =
      sdfWave->Get<double>("steepness", this->data->steepness).first;
    this->data->tau = sdfWave->Get<double>("tau", this->data->tau).first;
    this->data->gain = sdfWave->Get<double>("gain", this->data->gain).first;

    this->data->Recalculate();
  }
  this->DebugPrint();
}

///////////////////////////////////////////////////////////////////////////////
size_t Wavefield::Number() const
{
  return this->data->number;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Angle() const
{
  return this->data->angle;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Scale() const
{
  return this->data->scale;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Steepness() const
{
  return this->data->steepness;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::AngularFrequency() const
{
  return this->data->angularFrequency;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Amplitude() const
{
  return this->data->amplitude;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Period() const
{
  return this->data->period;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Phase() const
{
  return this->data->phase;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Wavelength() const
{
  return this->data->wavelength;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::Wavenumber() const
{
  return this->data->wavenumber;
}

///////////////////////////////////////////////////////////////////////////////
float Wavefield::Tau() const
{
  return this->data->tau;
}

///////////////////////////////////////////////////////////////////////////////
float Wavefield::Gain() const
{
  return this->data->gain;
}

///////////////////////////////////////////////////////////////////////////////
ignition::math::Vector2d Wavefield::Direction() const
{
  return this->data->direction;
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetNumber(size_t _number)
{
  this->data->number = _number;
  this->data->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetAngle(double _angle)
{
  this->data->angle = _angle;
  this->data->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetScale(double _scale)
{
  this->data->scale = _scale;
  this->data->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetSteepness(double _steepness)
{
  this->data->steepness = _steepness;
  this->data->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetAmplitude(double _amplitude)
{
  this->data->amplitude = _amplitude;
  this->data->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetPeriod(double _period)
{
  this->data->period = _period;
  this->data->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetPhase(double _phase)
{
  this->data->phase = _phase;
  this->data->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetTau(double _tau)
{
  this->data->tau = _tau;
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetGain(double _gain)
{
  this->data->gain = _gain;
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::SetDirection(const ignition::math::Vector2d &_direction)
{
  this->data->direction = _direction;
  this->data->Recalculate();
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<double> &Wavefield::AngularFrequency_V() const
{
  return this->data->angularFrequencies;
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<double> &Wavefield::Amplitude_V() const
{
  return this->data->amplitudes;
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<double> &Wavefield::Phase_V() const
{
  return this->data->phases;
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<double> &Wavefield::Steepness_V() const
{
  return this->data->steepnesses;
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<double> &Wavefield::Wavenumber_V() const
{
  return this->data->wavenumbers;
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<ignition::math::Vector2d> &Wavefield::Direction_V() const
{
  return this->data->directions;
}

///////////////////////////////////////////////////////////////////////////////
void Wavefield::DebugPrint() const
{
  ignmsg << "Input Parameters:" << std::endl;
  ignmsg << "model:     " << this->data->model << std::endl;
  ignmsg << "number:     " << this->data->number << std::endl;
  ignmsg << "scale:      " << this->data->scale << std::endl;
  ignmsg << "angle:      " << this->data->angle << std::endl;
  ignmsg << "steepness:  " << this->data->steepness << std::endl;
  ignmsg << "amplitude:  " << this->data->amplitude << std::endl;
  ignmsg << "period:     " << this->data->period << std::endl;
  ignmsg << "direction:  " << this->data->direction << std::endl;
  ignmsg << "tau:  " << this->data->tau << std::endl;
  ignmsg << "gain:  " << this->data->gain << std::endl;
  ignmsg << "Derived Parameters:" << std::endl;
  ignmsg << "amplitudes:  " << this->data->amplitudes << std::endl;
  ignmsg << "wavenumbers: " << this->data->wavenumbers << std::endl;
  ignmsg << "omegas:      " << this->data->angularFrequencies << std::endl;
  ignmsg << "periods:     ";
  for (auto&& omega : this->data->angularFrequencies) // NOLINT
  {
    ignmsg << 2.0 * M_PI / omega <<", ";
  }
  ignmsg << std::endl;
  ignmsg << "phases:      " << this->data->phases << std::endl;
  ignmsg << "steepnesses: " << this->data->steepnesses << std::endl;
  ignmsg << "directions:  ";
  for (auto&& d : this->data->directions) // NOLINT
  {
    ignmsg << d << "; ";
  }
  ignmsg << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::ComputeDepthSimply(const ignition::math::Vector3d &_point,
  double _time, double _timeInit)
{
  double h = 0.0;
  for (std::size_t i = 0; i < this->Number(); ++i)
  {
    double k = this->Wavenumber_V()[i];
    double a = this->Amplitude_V()[i];
    double dx =  this->Direction_V()[i].X();
    double dy =  this->Direction_V()[i].Y();
    double dot = _point.X() * dx + _point.Y() * dy;
    double omega = this->AngularFrequency_V()[i];
    double theta = k * dot - omega * _time;
    double c = cos(theta);
    h += a*c;
  }

  // Exponentially grow the waves
  return h * (1 - exp(-1.0 * (_time - _timeInit) / this->Tau()));
}

///////////////////////////////////////////////////////////////////////////////
double Wavefield::ComputeDepthDirectly(const ignition::math::Vector3d &_point,
  double _time, double _timeInit)
{
  // Struture for passing wave parameters to lambdas
  struct WaveParams
  {
    WaveParams(
      const std::vector<double>& _a,
      const std::vector<double>& _k,
      const std::vector<double>& _omega,
      const std::vector<double>& _phi,
      const std::vector<double>& _q,
      const std::vector<ignition::math::Vector2d>& _dir) :
      a(_a), k(_k), omega(_omega), phi(_phi), q(_q), dir(_dir) {}

    const std::vector<double>& a;
    const std::vector<double>& k;
    const std::vector<double>& omega;
    const std::vector<double>& phi;
    const std::vector<double>& q;
    const std::vector<ignition::math::Vector2d>& dir;
  };

  // Compute the target function and Jacobian. Also calculate pz,
  // the z-component of the Gerstner wave, which we essentially get for free.
  // cppcheck-suppress constParameter
  auto wave_fdf = [=](auto x, auto p, auto t, auto &wp, auto &F, auto &J)
  {
    double pz = 0;
    F(0) = p.x() - x.x();
    F(1) = p.y() - x.y();
    J(0, 0) = -1;
    J(0, 1) =  0;
    J(1, 0) =  0;
    J(1, 1) = -1;
    const size_t n = wp.a.size();
    for (auto&& i = 0; i < n; ++i) // NOLINT
    {
      const double dx = wp.dir[i].X();
      const double dy = wp.dir[i].Y();
      const double q = wp.q[i];
      const double a = wp.a[i];
      const double k = wp.k[i];
      const double dot = x.x() * dx + x.y() * dy;
      const double theta = k * dot - wp.omega[i] * t;
      const double s = std::sin(theta);
      const double c = std::cos(theta);
      const double qakc = q * a * k * c;
      const double df1x = qakc * dx * dx;
      const double df1y = qakc * dx * dy;
      const double df2x = df1y;
      const double df2y = qakc * dy * dy;
      pz += a * c;
      F(0) += a * dx * s;
      F(1) += a * dy * s;
      J(0, 0) += df1x;
      J(0, 1) += df1y;
      J(1, 0) += df2x;
      J(1, 1) += df2y;
    }
    // Exponentially grow the waves
    return pz * (1 - exp(-1.0 * (_time - _timeInit) / this->Tau()));
  };

  // Simple multi-variate Newton solver -
  // this version returns the z-component of the
  // wave field at the desired point p.
  // cppcheck-suppress constParameter
  auto solver = [=](auto& fdfunc, auto x0, auto p, auto t, \
                    auto& wp, auto tol, auto nmax)
  {
    int n = 0;
    double err = 1;
    double pz = 0;
    auto xn = x0;
    Eigen::Vector2d F;
    Eigen::Matrix2d J;
    while (std::abs(err) > tol && n < nmax)
    {
      pz = fdfunc(x0, p, t, wp, F, J);
      xn = x0 - J.inverse() * F;
      x0 = xn;
      err = F.norm();
      n++;
    }
    return pz;
  };

  // Set up parameter references
  WaveParams wp(
    this->Amplitude_V(),
    this->Wavenumber_V(),
    this->AngularFrequency_V(),
    this->Phase_V(),
    this->Steepness_V(),
    this->Direction_V());

  // Tolerances etc.
  const double tol = 1.0E-10;
  const double nmax = 30;

  // Use the target point as the initial guess
  // (this is within sum{amplitudes} of the solution)
  Eigen::Vector2d p2(_point.X(), _point.Y());
  const double pz = solver(wave_fdf, p2, p2, _time, wp, tol, nmax);
  // Removed so that height is reported relative to mean water level
  // const double h = pz - _point.Z();
  const double h = pz;
  return h;
}
