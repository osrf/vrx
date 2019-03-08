// Copyright (C) 2019  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/// \file Utilities.hh
/// \brief This file defines utilities for extracting parameters from SDF.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_UTILITIES_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_UTILITIES_HH_

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <string>

///////////////////////////////////////////////////////////////////////////////
// Forward Declarations

namespace gazebo
{
  namespace msgs
  {
    class Param_V;
  }
}

namespace sdf
{
  class Element;
}

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// Utilities

  /// \brief A collection of static methods for common tasks.
  class Utilities
  {
    /// \brief Extract a named bool parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static bool SdfParamBool(sdf::Element& _sdf,
      const std::string &_paramName, const bool _defaultVal);

    /// \brief Extract a named size_t parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static size_t SdfParamSizeT(sdf::Element& _sdf,
      const std::string &_paramName, const size_t _defaultVal);

    /// \brief Extract a named double parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static double SdfParamDouble(sdf::Element& _sdf,
      const std::string &_paramName, const double _defaultVal);

    /// \brief Extract a named string parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static std::string SdfParamString(sdf::Element& _sdf,
      const std::string &_paramName, const std::string _defaultVal);

    /// \brief Extract a named Vector2 parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static ignition::math::Vector2d SdfParamVector2(sdf::Element& _sdf,
      const std::string &_paramName, const ignition::math::Vector2d _defaultVal);

    /// \brief Extract a named Vector3 parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static ignition::math::Vector3d SdfParamVector3(sdf::Element& _sdf,
      const std::string &_paramName, const ignition::math::Vector3d _defaultVal);

    /// \brief Extract a named bool parameter from a Param_V message.
    ///
    /// \param[in] _msg         A reference to the Param_V message.
    /// \param[in] _paramName   The parameter name as it appears in message.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static bool MsgParamBool(const gazebo::msgs::Param_V& _msg,
      const std::string &_paramName, const bool _defaultVal);

    /// \brief Extract a named size_t parameter from a Param_V message.
    ///
    /// \param[in] _msg         A reference to the Param_V message.
    /// \param[in] _paramName   The parameter name as it appears in message.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static size_t MsgParamSizeT(const gazebo::msgs::Param_V& _msg,
      const std::string &_paramName, const size_t _defaultVal);

    /// \brief Extract a named double parameter from a Param_V message.
    ///
    /// \param[in] _msg         A reference to the Param_V message.
    /// \param[in] _paramName   The parameter name as it appears in message.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static double MsgParamDouble(const gazebo::msgs::Param_V& _msg,
      const std::string &_paramName, const double _defaultVal);

    /// \brief Extract a named std::string parameter from a Param_V message.
    ///
    /// \param[in] _msg         A reference to the Param_V message.
    /// \param[in] _paramName   The parameter name as it appears in message.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static std::string MsgParamString(const gazebo::msgs::Param_V& _msg,
      const std::string &_paramName, const std::string _defaultVal);

    /// \brief Extract a named Vector2 parameter from a Param_V message.
    ///
    /// \param[in] _msg         A reference to the Param_V message.
    /// \param[in] _paramName   The parameter name as it appears in message.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static ignition::math::Vector2d MsgParamVector2(const gazebo::msgs::Param_V& _msg,
      const std::string &_paramName, const ignition::math::Vector2d _defaultVal);

    /// \brief Extract a named Vector3 parameter from a Param_V message.
    ///
    /// \param[in] _msg         A reference to the Param_V message.
    /// \param[in] _paramName   The parameter name as it appears in message.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value (or default value if not found).
    public: static ignition::math::Vector3d MsgParamVector3(const gazebo::msgs::Param_V& _msg,
      const std::string &_paramName, const ignition::math::Vector3d _defaultVal);
  };

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_UTILITIES_HH_
