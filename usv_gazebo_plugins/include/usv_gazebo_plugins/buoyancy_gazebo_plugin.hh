/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef USV_GAZEBO_PLUGINS_BUOYANCY_GAZEBO_PLUGIN_HH_
#define USV_GAZEBO_PLUGINS_BUOYANCY_GAZEBO_PLUGIN_HH_

#include <map>
#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief A class for storing the volume properties of a link.
  class VolumeProperties
  {
    /// \brief Default constructor.
    public: VolumeProperties()
      : area(0), height(0)
    {
    }

    /// \brief Center of volume in the link frame.
    public: ignition::math::Vector3d cov;

    /// \brief Horizontal area of this link.
    public: double area;

    /// \brief Vertical height for this link.
    public: double height;
  };

  enum class ShapeType {
    None,
    Box,
    Sphere,
    Cylinder
  };

  class Shape {
  public:
    explicit Shape(ShapeType type) : type(type) {}
    virtual ~Shape() = default;

    ShapeType type;

    virtual std::string print() {
      switch(type) {
        case ShapeType::None:
          return "None";
        case ShapeType::Box:
          return "Box";
        case ShapeType::Cylinder:
          return "Cylinder";
        case ShapeType::Sphere:
          return "Sphere";
      }
    }

    static Shape* make_shape(const sdf::ElementPtr geometry);
  };

  class BoxShape : public Shape {
  public:
    explicit BoxShape(double x, double y, double z) : Shape(ShapeType::Box), x(x), y(y), z(z) {}

    explicit BoxShape(const sdf::ElementPtr boxElem) : Shape(ShapeType::Box) {
      if (boxElem->HasElement("size")) {
        ignition::math::Vector3d cov = boxElem->GetElement("size")
            ->Get<ignition::math::Vector3d>();
        if (cov[0] > 1e-9 && cov[1] > 1e-9 && cov[2] > 1e-9) {
          x = cov[0];
          y = cov[1];
          z = cov[2];
        } else {
          std::stringstream errMsg;
          errMsg << "Incorrect dimensions " << cov << " for 'box' geometry "
                 << "for buoyancy element number in SDF";
          throw std::runtime_error(errMsg.str().c_str());
        }
      } else {
        throw std::runtime_error("Error parsing box");
      }
    }

    double x;
    double y;
    double z;

    std::string print() override {
      return Shape::print();
    }
  };

  class CylinderShape : public Shape {
  public:
    explicit CylinderShape(double r, double h) : Shape(ShapeType::Cylinder), r(r), h(h) {}

    explicit CylinderShape(const sdf::ElementPtr cylinderElem) : Shape(ShapeType::Cylinder) {
      if (cylinderElem->HasElement("radius")) {
        r = cylinderElem->GetElement("radius")->Get<double>();
      } else {
        throw std::runtime_error("Error parsing cylinder");
      }

      if (cylinderElem->HasElement("length")) {
        h = cylinderElem->GetElement("length")->Get<double>();
      } else {
        throw std::runtime_error("Error parsing cylinder");
      }

      if (r < 1e-9 || h < 1e-9) {
        throw std::runtime_error("Error parsing cylinder");
      }
    }

    double r;
    double h;

    std::string print() override {
      return Shape::print();
    }
  };

  class SphereShape : public Shape {
  public:
    explicit SphereShape(double r) : Shape(ShapeType::Sphere), r(r) {}

    double r;

    explicit SphereShape(const sdf::ElementPtr sphereElem) : Shape(ShapeType::Cylinder) {
      if (sphereElem->HasElement("radius")) {
        r = sphereElem->GetElement("radius")->Get<double>();
      } else {
        throw std::runtime_error("Error parsing sphere");
      }

      if (r < 1e-9) {
        throw std::runtime_error("Error parsing sphere");
      }
    }

    std::string print() override {
      return Shape::print();
    }
  };

  Shape* Shape::make_shape(const sdf::ElementPtr geometry) {
    int counter = 0;
    if (geometry->HasElement("box")) {
      return dynamic_cast<Shape*>(new BoxShape(geometry->GetElement("box")));
    } else if (geometry->HasElement("sphere")) {
      return dynamic_cast<Shape*>(new SphereShape(geometry->GetElement("sphere")));
    } else if (geometry->HasElement("cylinder")) {
      return dynamic_cast<Shape*>(new CylinderShape(geometry->GetElement("cylinder")));
    } else {
      std::stringstream errMsg;
      errMsg << "Invalid 'geometry' element within buoyancy element number ["
             << counter << "] in SDF" << std::endl;
      throw std::runtime_error(errMsg.str().c_str());
    }
  }

  /// \brief A class for storing buoyancy object properties
  class BuoyancyObject {

    /// \brief Default constructor
  public:
    BuoyancyObject()
      : linkId(-1),
        linkName(""),
        shape(nullptr) {}

    /// \brief associated link ID
    int linkId;

    /// \brief associated link name
    std::string linkName;

    /// \brief buoyancy object pose relative to link

    /// \brief buoyancy object shape
    Shape* shape;

    // Factory Method
    static BuoyancyObject *parseBuoyancyObject(const physics::ModelPtr model, const sdf::ElementPtr elem, int counter) {
      auto *output = new BuoyancyObject();
      gzmsg << "Buoyancy element number " << counter << std::endl;

      // parse link
      if (elem->HasElement("link_name")) {
        output->linkName = elem->GetElement("link_name")->Get<std::string>();
        gzmsg << "\tFound link name in SDF [" << output->linkName << "]" << std::endl;
        physics::LinkPtr link = model->GetLink(output->linkName);
        if (!link) {
          std::stringstream errMsg;
          errMsg << "\tSpecified link [" << output->linkName << "] not found.";
          throw std::runtime_error(errMsg.str().c_str());
        }
        output->linkId = link->GetId();
      } else {
        std::stringstream errMsg;
        errMsg << "Missing 'link_name' element within buoyancy element number ["
                << counter << "] in SDF";
        throw std::runtime_error(errMsg.str().c_str());
      }

      // parse geometry
      if (elem->HasElement("geometry")) {
        sdf::ElementPtr geometry = elem->GetElement("geometry");
        output->shape = Shape::make_shape(geometry);
      } else {
        std::stringstream errMsg;
        errMsg << "Missing 'geometry' element within buoyancy element number ["
               << counter << "] in SDF" << std::endl;
        throw std::runtime_error(errMsg.str().c_str());
      }
      return output;
    }

  };

  /// \brief A plugin that simulates buoyancy of an object immersed in fluid.
  /// All SDF parameters are optional.
  ///   <fluid_density>: Sets the density of the fluid that surrounds the
  ///                    buoyant object [kg/m^3].
  ///                    This paramater is optional.
  ///
  ///   <fluid_level>:   The height of the fluid/air interface [m].
  ///                    This parameter is optional.
  ///
  ///   <fluid_drag>:    Quadratic drag generally applied to Z velocity.
  ///                    This parameter is optional.
  ///
  ///   <link>:          Describe the volume properties of individual links in
  ///                    the model.
  ///                    For example:
  ///
  ///                    <link name="body">
  ///                      <center_of_volume>1 2 3</center_of_volume>
  ///                      <area>10</volume>
  ///                      <height>5</height>
  ///                    </link>
  ///
  ///     <center_of_volume>: A point representing the volumetric center of the
  ///                         link in the link frame. This is where the buoyancy
  ///                         force will be applied. This field is required.
  ///
  ///     <area>:             Horizontal area of this link.
  ///                         This field is required
  ///
  ///     <height>:           Vertical height of this link.
  ///                         This field is required.
  class BuoyancyPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: BuoyancyPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief The density of the fluid in which the object is submerged in
    /// kg/m^3. Defaults to 1000, the fluid density of water at 15 Celsius.
    protected: double fluidDensity;

    /// \brief The height of the fluid/air interface [m]. Defaults to 0.
    protected: double fluidLevel;

    /// \brief Quadratic drag generally applied to Z velocity. Defaults to 0.
    protected: double fluidDrag;

    /// \brief Map of <link ID, point> pairs mapping link IDs to the CoV
    /// (center of volume) and volume of the link.
    protected: std::map<int, VolumeProperties> volPropsMap;

    /// \brief Vector of links in the model for which we will apply buoyancy
    /// forces.
    protected: physics::Link_V buoyancyLinks;
  };
}

#endif
