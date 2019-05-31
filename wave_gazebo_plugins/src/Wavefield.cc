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

#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/Geometry.hh"
#include "wave_gazebo_plugins/Physics.hh"
#include "wave_gazebo_plugins/Utilities.hh"

#include <Eigen/Dense>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <array>
#include <iostream>
#include <cmath>
#include <string>

namespace asv 
{
///////////////////////////////////////////////////////////////////////////////
// Utilities

  std::ostream& operator<<(std::ostream& os, const std::vector<double>& _vec)
  { 
    for (auto&& v : _vec )
      os << v << ", ";
    return os;
  }

///////////////////////////////////////////////////////////////////////////////
// WaveParametersPrivate

  /// \internal
  /// \brief Private data for the WavefieldParameters.
  class WaveParametersPrivate
  {
    /// \brief Constructor.
    public: WaveParametersPrivate():
      number(1), 
      scale(2.0),
      angle(2.0*M_PI/10.0),
      steepness(1.0),
      amplitude(0.0), 
      period(1.0), 
      phase(0.0), 
      direction(1, 0),
      angularFrequency(2.0*M_PI),
      wavelength(2*M_PI/Physics::DeepWaterDispersionToWavenumber(2.0*M_PI)), 
      wavenumber(Physics::DeepWaterDispersionToWavenumber(2.0*M_PI))
    {
    }

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

    /// \brief The mean wve phase (not currently enabled).
    public: double phase;

    /// \brief The mean wave direction.
    public: ignition::math::Vector2d direction;

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

    /// \brief Recalculate all derived quantities from inputs.
    public: void Recalculate()
    {
      // Normalize direction
      this->direction = Geometry::Normalize(this->direction);

      // Derived mean values
      this->angularFrequency = 2.0 * M_PI / this->period;
      this->wavenumber = Physics::DeepWaterDispersionToWavenumber(this->angularFrequency);
      this->wavelength = 2.0 * M_PI / this->wavenumber;

      // Update components
      this->angularFrequencies.clear();
      this->amplitudes.clear();
      this->phases.clear();
      this->wavenumbers.clear();
      this->steepnesses.clear();
      this->directions.clear();

      for (size_t i=0; i<this->number; ++i)
      {
        const int n = i - this->number/2;
        const double scaleFactor = std::pow(this->scale, n);
        const double a = scaleFactor * this->amplitude;
        const double k = this->wavenumber / scaleFactor;
        const double omega = Physics::DeepWaterDispersionToOmega(k);
        const double phi = this->phase;
        double q = 0.0;
        if (a != 0)
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
        // const TransformMatrix T(
        //   c, -s,
        //   s,  c
        // );
        // const ignition::math::Vector2d d = T(this->direction);
        const ignition::math::Vector2d d(
          c * this->direction.X() - s * this->direction.Y(),
          s * this->direction.X() + c * this->direction.Y()
        );
        directions.push_back(d);
      }
    }
  };

///////////////////////////////////////////////////////////////////////////////
// WaveParameters

  WaveParameters::~WaveParameters()
  {
  }

  WaveParameters::WaveParameters()
    : data(new WaveParametersPrivate())
  {
    this->data->Recalculate();
  }

  void WaveParameters::FillMsg(gazebo::msgs::Param_V& _msg) const
  {
    // Clear 
    _msg.mutable_param()->Clear();

    // "number"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("number");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::INT32);
      nextParam->mutable_value()->set_int_value(this->data->number);
    }
    // "scale"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("scale");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->scale);
    }
    // "angle"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("angle");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->angle);
    }
    // "steepness"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("steepness");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->steepness);
    }
    // "amplitude"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("amplitude");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->amplitude);
    }
    // "period"
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("period");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->period);
    }
    // "direction"
    {
      const auto& direction = this->data->direction;
      auto nextParam = _msg.add_param();
      nextParam->set_name("direction");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::VECTOR3D);
      nextParam->mutable_value()->mutable_vector3d_value()->set_x(direction.X());
      nextParam->mutable_value()->mutable_vector3d_value()->set_y(direction.Y());
      nextParam->mutable_value()->mutable_vector3d_value()->set_z(0);
    }
  }

  void WaveParameters::SetFromMsg(const gazebo::msgs::Param_V& _msg)
  {
    this->data->number    = Utilities::MsgParamSizeT(_msg,    "number",     this->data->number);
    this->data->amplitude = Utilities::MsgParamDouble(_msg,   "amplitude",  this->data->amplitude);
    this->data->period    = Utilities::MsgParamDouble(_msg,   "period",     this->data->period);
    this->data->phase     = Utilities::MsgParamDouble(_msg,   "phase",      this->data->phase);
    this->data->direction = Utilities::MsgParamVector2(_msg,  "direction",  this->data->direction);
    this->data->scale     = Utilities::MsgParamDouble(_msg,   "scale",      this->data->scale);
    this->data->angle     = Utilities::MsgParamDouble(_msg,   "angle",      this->data->angle);
    this->data->steepness = Utilities::MsgParamDouble(_msg,   "steepness",  this->data->steepness);

    this->data->Recalculate();
  }

  void WaveParameters::SetFromSDF(sdf::Element& _sdf)
  {
    this->data->number    = Utilities::SdfParamSizeT(_sdf,    "number",     this->data->number);
    this->data->amplitude = Utilities::SdfParamDouble(_sdf,   "amplitude",  this->data->amplitude);
    this->data->period    = Utilities::SdfParamDouble(_sdf,   "period",     this->data->period);
    this->data->phase     = Utilities::SdfParamDouble(_sdf,   "phase",      this->data->phase);
    this->data->direction = Utilities::SdfParamVector2(_sdf,  "direction",  this->data->direction);
    this->data->scale     = Utilities::SdfParamDouble(_sdf,   "scale",      this->data->scale);
    this->data->angle     = Utilities::SdfParamDouble(_sdf,   "angle",      this->data->angle);
    this->data->steepness = Utilities::SdfParamDouble(_sdf,   "steepness",  this->data->steepness);

    this->data->Recalculate();
  }

  size_t WaveParameters::Number() const
  {
    return this->data->number;
  }

  double WaveParameters::Angle() const
  {
    return this->data->angle;
  }

  double WaveParameters::Scale() const
  {
    return this->data->scale;
  }

  double WaveParameters::Steepness() const
  {
    return this->data->steepness;
  }

  double WaveParameters::AngularFrequency() const
  {
    return this->data->angularFrequency;
  }

  double WaveParameters::Amplitude() const
  {
    return this->data->amplitude;
  }
  
  double WaveParameters::Period() const
  {
    return this->data->period;
  }
  
  double WaveParameters::Phase() const
  {
    return this->data->phase;
  }

  double WaveParameters::Wavelength() const
  {
    return this->data->wavelength;
  }

  double WaveParameters::Wavenumber() const
  {
    return this->data->wavenumber;
  }    

  ignition::math::Vector2d WaveParameters::Direction() const
  {
    return this->data->direction;
  }
  
  void WaveParameters::SetNumber(size_t _number)
  {
    this->data->number = _number;
    this->data->Recalculate();
  }

  void WaveParameters::SetAngle(double _angle)
  {
    this->data->angle = _angle;
    this->data->Recalculate();
  }

  void WaveParameters::SetScale(double _scale)
  {
    this->data->scale = _scale;
    this->data->Recalculate();
  }

  void WaveParameters::SetSteepness(double _steepness)
  {
    this->data->steepness = _steepness;
    this->data->Recalculate();
  }

  void WaveParameters::SetAmplitude(double _amplitude)
  {
    this->data->amplitude = _amplitude;
    this->data->Recalculate();
  }
  
  void WaveParameters::SetPeriod(double _period)
  {
    this->data->period = _period;
    this->data->Recalculate();
  }
    
  void WaveParameters::SetPhase(double _phase)
  {
    this->data->phase = _phase;
    this->data->Recalculate();
  }
  
  void WaveParameters::SetDirection(const ignition::math::Vector2d& _direction)
  {
    this->data->direction = _direction;
    this->data->Recalculate();
  }

  const std::vector<double>& WaveParameters::AngularFrequency_V() const
  {
    return this->data->angularFrequencies;
  }

  const std::vector<double>& WaveParameters::Amplitude_V() const
  {
    return this->data->amplitudes;
  }
  
  const std::vector<double>& WaveParameters::Phase_V() const
  {
    return this->data->phases;
  }
  
  const std::vector<double>& WaveParameters::Steepness_V() const
  {
    return this->data->steepnesses;
  }

  const std::vector<double>& WaveParameters::Wavenumber_V() const
  {
    return this->data->wavenumbers;
  }

  const std::vector<ignition::math::Vector2d>& WaveParameters::Direction_V() const
  {
    return this->data->directions;
  }
 
  void WaveParameters::DebugPrint() const
  {
    gzmsg << "number:     " << this->data->number << std::endl;
    gzmsg << "scale:      " << this->data->scale << std::endl;
    gzmsg << "angle:      " << this->data->angle << std::endl;
    gzmsg << "period:     " << this->data->period << std::endl;
    gzmsg << "amplitude:  " << this->data->amplitudes << std::endl;
    gzmsg << "wavenumber: " << this->data->wavenumbers << std::endl;
    gzmsg << "omega:      " << this->data->angularFrequencies << std::endl;
    gzmsg << "phase:      " << this->data->phases << std::endl;
    gzmsg << "steepness:  " << this->data->steepnesses << std::endl;
    for (auto&& d : this->data->directions)
    {
      gzmsg << "direction:  " << d << std::endl;
    }
  }

///////////////////////////////////////////////////////////////////////////////    
// WavefieldSampler

  double WavefieldSampler::ComputeDepthDirectly(  
    const WaveParameters& _waveParams,
    const ignition::math::Vector3d& _point,
    double time
  )
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
    auto wave_fdf = [=](auto x, auto p, auto t, auto& wp, auto& F, auto& J)
    {
      double pz = 0;
      F(0) = p.x() - x.x();
      F(1) = p.y() - x.y();
      J(0, 0) = -1;
      J(0, 1) =  0;
      J(1, 0) =  0;
      J(1, 1) = -1;
      const size_t n = wp.a.size();
      for (auto&& i=0; i<n; ++i)
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
      return pz;
    };

    // Simple multi-variate Newton solver - this version returns the z-component of the
    // wave field at the desired point p.
    auto solver = [=](auto& fdfunc, auto x0, auto p, auto t, auto& wp, auto tol, auto nmax)
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
      _waveParams.Amplitude_V(),
      _waveParams.Wavenumber_V(),
      _waveParams.AngularFrequency_V(),
      _waveParams.Phase_V(),
      _waveParams.Steepness_V(),
      _waveParams.Direction_V()
    );

    // Tolerances etc.
    const double tol = 1.0E-10;
    const double nmax = 30;

    // Use the target point as the initial guess (this is within sum{amplitudes} of the solution)
    Eigen::Vector2d p2(_point.X(), _point.Y());
    const double pz = solver(wave_fdf, p2, p2, time, wp, tol, nmax);
    const double h = pz - _point.Z();
    return h;
  }

///////////////////////////////////////////////////////////////////////////////

} // namespace asv
