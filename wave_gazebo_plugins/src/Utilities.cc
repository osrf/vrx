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

#include <algorithm>
#include <iostream>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <sdf/sdf.hh>

#include "wave_gazebo_plugins/Utilities.hh"

namespace asv 
{
///////////////////////////////////////////////////////////////////////////////
// Templates

// This code adapted vmrc/usv_gazebo_plugins/usv_gazebo_dynamics_plugin.cc
template <typename T>T
  SdfParam(sdf::Element& _sdf, const std::string &_paramName, \
					 const T _defaultVal)
{
  if (!_sdf.HasElement(_paramName))
  {
    gzmsg << "Parameter <" << _paramName << "> not found: " 
      <<  "Using default value of <" << _defaultVal << ">." << std::endl;
    return _defaultVal;
  }

  T val = _sdf.Get<T>(_paramName);
  gzmsg << "Parameter found - setting <" << _paramName 
    << "> to <" << val << ">." << std::endl;
  return val;
}

/// \brief Template function for extracting a value from a parameter message.
template <typename T>
T MsgParamGetValue(const gazebo::msgs::Param& _msg)
{
  gzwarn << "Using default template for MsgParamGetValue" << std::endl;
  return T();  
}

/// \brief Template specialization for
/// extracting a bool from a parameter message.
template <>
bool MsgParamGetValue<bool>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  return paramValue.bool_value();
}

/// \brief Template specialization for extracting
/// an int from a parameter message.
template <>
int MsgParamGetValue<int>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  return paramValue.int_value();
}

/// \brief Template specialization for extracting
/// a size_t from a parameter message.
template <>
size_t MsgParamGetValue<size_t>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  return paramValue.int_value();
}

/// \brief Template specialization for extracting
/// a double from a parameter message.
template <>
double MsgParamGetValue<double>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  return paramValue.double_value();
}

/// \brief Template specialization for extracting
/// a string from a parameter message.
template <> std::string
MsgParamGetValue<std::string>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  return paramValue.string_value();
}

/// \brief Template specialization for extracting
/// a Vector2 from a parameter message.
template <> ignition::math::Vector2d
MsgParamGetValue<ignition::math::Vector2d>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  auto vec = paramValue.vector3d_value();
  return ignition::math::Vector2d(vec.x(), vec.y());
}

/// \brief Template specialization for extracting
/// a Vector3 from a parameter message.
template <>
ignition::math::Vector3d
MsgParamGetValue<ignition::math::Vector3d>(const gazebo::msgs::Param& _msg)
{ 
  auto paramValue = _msg.value();
  auto vec = paramValue.vector3d_value();
  return ignition::math::Vector3d(vec.x(), vec.y(), vec.z());
}

/// \brief Template for extracting a
/// named parameter from a parameter vector message.
template <typename T> T
MsgParam(const gazebo::msgs::Param_V& _msg, const std::string \
				 &_paramName, const T _defaultVal)
{
  // Custom compare for params
  auto compare = [=](auto& _param)
  {
    return _param.name() == _paramName;
  };

  auto it = std::find_if(std::begin(_msg.param()), std::end(_msg.param()), \
												 compare);
  
  // Not found
  if (it == std::end(_msg.param()))
  {
  // @DEBUG_INFO
    // gzmsg << "Parameter <" << _paramName << "> not found: " 
    //   <<  "Using default value of <" << _defaultVal << ">." << std::endl;
    return _defaultVal;
  }

  // Found
  auto& param = *it;
  T val = MsgParamGetValue<T>(param);

  // @DEBUG_INFO
  // gzmsg << "Parameter found - setting <" << _paramName 
  //   << "> to <" << val << ">." << std::endl;
  return val;
}


///////////////////////////////////////////////////////////////////////////////
// Utilities

bool Utilities::SdfParamBool(sdf::Element& _sdf,
  const std::string& _paramName, const bool _defaultVal)
{
  return SdfParam<bool>(_sdf, _paramName, _defaultVal);
}

size_t Utilities::SdfParamSizeT(sdf::Element& _sdf,
  const std::string& _paramName, const size_t _defaultVal)
{
  return SdfParam<double>(_sdf, _paramName, _defaultVal);
}

double Utilities::SdfParamDouble(sdf::Element& _sdf,
  const std::string& _paramName, const double _defaultVal)
{
  return SdfParam<double>(_sdf, _paramName, _defaultVal);
}

std::string Utilities::SdfParamString(sdf::Element& _sdf,
  const std::string& _paramName, const std::string &_defaultVal)
{
  return SdfParam<std::string>(_sdf, _paramName, _defaultVal);
}

ignition::math::Vector2d Utilities::SdfParamVector2(sdf::Element& _sdf,
  const std::string& _paramName, const ignition::math::Vector2d _defaultVal)
{
  return SdfParam<ignition::math::Vector2d>(_sdf, _paramName, _defaultVal);
}

ignition::math::Vector3d Utilities::SdfParamVector3(sdf::Element& _sdf,
  const std::string& _paramName, const ignition::math::Vector3d _defaultVal)
{
  return SdfParam<ignition::math::Vector3d>(_sdf, _paramName, _defaultVal);
}

bool Utilities::MsgParamBool(const gazebo::msgs::Param_V& _msg,
  const std::string &_paramName, const bool _defaultVal)
{
  return MsgParam<bool>(_msg, _paramName, _defaultVal);
}

size_t Utilities::MsgParamSizeT(const gazebo::msgs::Param_V& _msg,
  const std::string &_paramName, const size_t _defaultVal)
{
  return MsgParam<size_t>(_msg, _paramName, _defaultVal);
}

double Utilities::MsgParamDouble(const gazebo::msgs::Param_V& _msg,
  const std::string &_paramName, const double _defaultVal)
{
  return MsgParam<double>(_msg, _paramName, _defaultVal);
}

std::string Utilities::MsgParamString(const gazebo::msgs::Param_V& _msg,
  const std::string &_paramName, const std::string &_defaultVal)
{
  return MsgParam<std::string>(_msg, _paramName, _defaultVal);
}

ignition::math::Vector2d
Utilities::MsgParamVector2(const gazebo::msgs::Param_V& _msg, \
													 const std::string &_paramName, \
													 const ignition::math::Vector2d _defaultVal)
{
  return MsgParam<ignition::math::Vector2d>(_msg, _paramName, _defaultVal);
}

ignition::math::Vector3d
Utilities::MsgParamVector3(const gazebo::msgs::Param_V& _msg, \
													 const std::string &_paramName, \
													 const ignition::math::Vector3d _defaultVal)
{
  return MsgParam<ignition::math::Vector3d>(_msg, _paramName, _defaultVal);
}


///////////////////////////////////////////////////////////////////////////////

}  // namespace asv
