#include <string>
#include <Eigen/Eigen>
#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Entity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/float.pb.h>
#include "gz/msgs/Utility.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "USVWind.hh"

using namespace gz;
using namespace vrx;

struct WindObj
{
  bool init = false;
  /// \brief The link information.
  std::string modelName;
  std::string linkName;
  sim::Entity linkEntity = sim::kNullEntity;
  math::Vector3d windCoeff;
};

class USVWind::Implementation
{

  /// \brief vector of simple objects effected by the wind
public:
  std::vector<WindObj> windObjs;

  /// \brief Bool to keep track if ALL of the windObjs have been initialized
public:
  bool windObjsInit = false;

  /// \brief Bool to keep track of the number of windObjs initialized
public:
  unsigned int windObjsInitCount = 0;

  /// \brief Wind velocity unit vector in Gazebo coordinates [m/s]
public:
  math::Vector3d windDirection;

  /// \brief Average wind velocity
public:
  double windMeanVelocity;

  /// \brief User specified gain constant
public:
  double gainConstant = 0;

  /// \brief Calculated filter gain constant
public:
  double filterGain = 0;

  /// \brief Time constant
public:
  double timeConstant = 0.1;

  /// \brief Previous time
public:
  std::chrono::duration<double> previousTime = std::chrono::duration<double>(0);

  /// \brief Variable velocity component
public:
  double varVel = 0;
  /// \brief Topic where the wind speed is published
public:
  std::string topicWindSpeed = "/vrx/debug/wind/speed";

  /// \brief Topic where the wind direction is published
public:
  std::string topicWindDirection = "/vrx/debug/wind/direction";
  /// \brief Message to store wind vector (constant)
public:
  msgs::Float windDirMsg;

  /// \brief Last time wind speed and direction was published
public:
  double lastPublishTime = 0;

  /// \brief Transport node
public:
  transport::Node node;

  /// \brief Transport node publisher for wind speed
public:
  transport::Node::Publisher windSpeedPub;

  /// \brief Transport node publisher for wind direction
public:
  transport::Node::Publisher windDirectionPub;

  /// \brief Update rate buffer for wind speed and direction
public:
  double updateRate = 10;

  /// \def Random generator
public:
  std::unique_ptr<std::mt19937> randGenerator;
};

//////////////////////////////////////////////////
USVWind::USVWind() : dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void USVWind::Configure(const sim::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        sim::EntityComponentManager &_ecm,
                        sim::EventManager & /*_eventMgr*/)
{
  sdf::ElementPtr sdf = _sdf->Clone();

  // Retrieve models' parameters from SDF
  if (!sdf->HasElement("wind_obj"))
  {
    gzerr << "Did not find SDF parameter wind_obj" << std::endl;
  }
  else
  {
    sdf::ElementPtr windObjSDF = sdf->GetElement("wind_obj");
    while (windObjSDF)
    {
      WindObj obj;
      if (!windObjSDF->HasElement("name"))
      {
        gzerr << ("Did not find SDF parameter name") << std::endl;
      }
      else
      {
        obj.modelName = windObjSDF->Get<std::string>("name");
      }

      if (!windObjSDF->HasElement("link_name"))
      {
        gzerr << ("Did not find SDF parameter link_name") << std::endl;
      }
      else
      {
        obj.linkName = windObjSDF->Get<std::string>("link_name");
      }

      if (!windObjSDF->HasElement("coeff_vector"))
      {
        gzerr << ("Did not find SDF parameter coeff_vector") << std::endl;
      }
      else
      {
        obj.windCoeff = windObjSDF->Get<math::Vector3d>("coeff_vector");
      }
      this->dataPtr->windObjs.push_back(obj);
      gzdbg << obj.modelName << " loaded" << std::endl;
      windObjSDF = windObjSDF->GetNextElement("wind_obj");
    }
  }

  if (sdf->HasElement("wind_direction"))
  {
    double windAngle = sdf->Get<double>("wind_direction");
    this->dataPtr->windDirection.X(cos(windAngle * M_PI / 180));
    this->dataPtr->windDirection.Y(sin(windAngle * M_PI / 180));
    this->dataPtr->windDirection.Z(0);
    this->dataPtr->windDirMsg.set_data(windAngle);
  }

  gzmsg << "Wind direction unit vector = " << this->dataPtr->windDirection 
        << std::endl;

  if (sdf->HasElement("wind_mean_velocity"))
  {
    this->dataPtr->windMeanVelocity =
        sdf->Get<double>("wind_mean_velocity");
  }

  gzmsg << "Wind mean velocity = " << this->dataPtr->windMeanVelocity 
        << std::endl;

  if (sdf->HasElement("var_wind_gain_constants"))
  {
    this->dataPtr->gainConstant =
        sdf->Get<double>("var_wind_gain_constants");
  }

  gzmsg << "var wind gain constants = " << this->dataPtr->gainConstant 
        << std::endl;

  if (sdf->HasElement("var_wind_time_constants"))
  {
    this->dataPtr->timeConstant =
        sdf->Get<double>("var_wind_time_constants");
  }

  gzmsg << "var wind time constants = " << this->dataPtr->timeConstant 
        << std::endl;

  if (sdf->HasElement("update_rate"))
  {
    this->dataPtr->updateRate =
        sdf->Get<double>("update_rate");
  }

  gzmsg << "update rate  = " << this->dataPtr->updateRate << std::endl;

  if (sdf->HasElement("topic_wind_speed"))
  {
    this->dataPtr->topicWindSpeed =
        sdf->Get<std::string>("topic_wind_speed");
  }

  gzmsg << "topic wind speed  = " << this->dataPtr->topicWindSpeed << std::endl;

  if (sdf->HasElement("topic_wind_direction"))
  {
    this->dataPtr->topicWindDirection =
        sdf->Get<std::string>("topic_wind_direction");
  }

  gzmsg << "topic wind direction = " << 
      this->dataPtr->topicWindDirection << std::endl;

  // Setting the  seed for the random generator.
  unsigned int seed = std::random_device{}();
  if (sdf->HasElement("random_seed") &&
      sdf->Get<unsigned int>("random_seed") != 0)
  {
    seed = sdf->Get<unsigned int>("random_seed");
  }

  gzmsg << "Random seed value = " << seed << std::endl;
  this->dataPtr->randGenerator.reset(new std::mt19937(seed));

  // Calculate filter constant
  this->dataPtr->filterGain = this->dataPtr->gainConstant * 
      sqrt(2.0 * this->dataPtr->timeConstant);
  gzmsg << "Var wind filter gain = " << this->dataPtr->filterGain << std::endl;

  // Set up publishers
  transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(this->dataPtr->updateRate);

  this->dataPtr->windSpeedPub = this->dataPtr->node.Advertise<msgs::Float>(
      this->dataPtr->topicWindSpeed, opts);

  this->dataPtr->windDirectionPub = this->dataPtr->node.Advertise<msgs::Float>(
      this->dataPtr->topicWindDirection, opts);
}

//////////////////////////////////////////////////
void USVWind::PreUpdate(
    const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("USVWind::PreUpdate");
  if (_info.paused)
    return;
  auto time = std::chrono::duration<double>(_info.simTime);
  if (this->dataPtr->previousTime == std::chrono::duration<double>(0))
  {
    this->dataPtr->previousTime = time - std::chrono::duration<double>(0.001);
  }

  if (!this->dataPtr->windObjsInit)
  {
    for (auto &i : this->dataPtr->windObjs)
    {

      if (!i.init)
      {
        {
          auto entity = sim::entitiesFromScopedName(i.linkName, _ecm);
          if (!entity.empty())
          {
            i.linkEntity = *entity.begin();
            gzdbg << i.modelName << " initialized" << std::endl;
            ++this->dataPtr->windObjsInitCount;
            i.init = true;
            sim::enableComponent<sim::components::WorldLinearVelocity>
              (_ecm, i.linkEntity, true);
            sim::enableComponent<sim::components::WorldPose>
              (_ecm, i.linkEntity, true);
            sim::enableComponent<sim::components::Inertial>
              (_ecm, i.linkEntity, true);
          }
        }
      }
    }
    if (this->dataPtr->windObjsInitCount == this->dataPtr->windObjs.size())
    {
      this->dataPtr->windObjsInit = true;
    }
  }

  auto dT = time - this->dataPtr->previousTime;

  std::normal_distribution<double> dist(0, 1);
  double randomDist = dist(*this->dataPtr->randGenerator);

  // Current variable wind velocity
  this->dataPtr->varVel += 1.0 / this->dataPtr->timeConstant *
                           (-1.0 * this->dataPtr->varVel + 
                           this->dataPtr->filterGain / sqrt(dT.count()) * 
                           randomDist) * dT.count();
  // Current wind velocity
  double velocity = this->dataPtr->varVel + this->dataPtr->windMeanVelocity;
  msgs::Float windVelMsg;
  windVelMsg.set_data(velocity);
  this->dataPtr->windSpeedPub.Publish(windVelMsg);
  this->dataPtr->windDirectionPub.Publish(this->dataPtr->windDirMsg);
  for (auto &windObj : this->dataPtr->windObjs)
  {
    // Apply the forces of the wind to all wind objects only if they have been
    // initialized
    if (!windObj.init)
    {
      continue;
    }
    // get world linear velocity of link
    auto worldVel = 
        *std::move(_ecm.ComponentData<sim::components::WorldLinearVelocity>
        (windObj.linkEntity));
    math::Vector3d relativeWind =
        this->dataPtr->windDirection * velocity - worldVel;
    math::Vector3d windForce(
        windObj.windCoeff.X() * relativeWind.X() * abs(relativeWind.X()),
        windObj.windCoeff.Y() * relativeWind.Y() * abs(relativeWind.Y()), 0);
    math::Vector3d windTorque(0.0, 0.0,
                              -2.0 * windObj.windCoeff.Z() * relativeWind.X() * 
                              relativeWind.Y());
    auto linkWrenchComp =
        _ecm.Component<sim::components::ExternalWorldWrenchCmd>
        (windObj.linkEntity);
    sim::components::ExternalWorldWrenchCmd wrench;

    if (!linkWrenchComp)
    {
      msgs::Set(wrench.Data().mutable_force(), windForce);
      msgs::Set(wrench.Data().mutable_torque(), windTorque);
      _ecm.CreateComponent(windObj.linkEntity, wrench);
    }
    else
    {
      msgs::Set(linkWrenchComp->Data().mutable_force(),
                msgs::Convert(linkWrenchComp->Data().force()) + windForce);

      msgs::Set(linkWrenchComp->Data().mutable_torque(),
                msgs::Convert(linkWrenchComp->Data().torque()) + windTorque);
    }
  }
  this->dataPtr->previousTime = time;
}
GZ_ADD_PLUGIN(vrx::USVWind,
              sim::System,
              USVWind::ISystemConfigure,
              USVWind::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::USVWind,
                    "vrx::USVWind")
