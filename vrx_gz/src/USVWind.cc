#include <string>
#include <Eigen/Eigen>
#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/double.pb.h>
#include "USVWind.hh"

using namespace gz;
using namespace sim;
using namespace vrx;

    struct WindObj
    {
        bool init = false;
        /// \brief The link information.
        std::string modelName;
        std::string linkName;
        sim::Entity linkEntity;
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
    double timeConstant = 0;

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
void USVWind::Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager & /*_eventMgr*/)
{
    sdf::ElementPtr sdf = _sdf->Clone();;

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
    }

    gzmsg << "Wind direction unit vector = " << this->dataPtr->windDirection << std::endl;

    if (sdf->HasElement("wind_mean_velocity"))
    {
        this->dataPtr->windMeanVelocity =
            sdf->Get<double>("wind_mean_velocity");
    }

    gzmsg << "Wind mean velocity = " << this->dataPtr->windMeanVelocity << std::endl;

    if (sdf->HasElement("var_wind_gain_constants"))
    {
        this->dataPtr->gainConstant =
            sdf->Get<double>("var_wind_gain_constants");
    }

    gzmsg << "var wind gain constants = " << this->dataPtr->gainConstant << std::endl;

    if (sdf->HasElement("var_wind_time_constants"))
    {
        this->dataPtr->timeConstant =
            sdf->Get<double>("var_wind_time_constants");
    }

    gzmsg << "var wind time constants = " << this->dataPtr->timeConstant << std::endl;

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

    gzmsg << "topic wind direction  = " << this->dataPtr->topicWindDirection << std::endl;

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
    this->dataPtr->filterGain = this->dataPtr->gainConstant * sqrt(2.0 * this->dataPtr->timeConstant);
    gzmsg << "Var wind filter gain = " << this->dataPtr->filterGain << std::endl;

    // TODO: Publishers
    this->dataPtr->windSpeedPub = this->dataPtr->node.Advertise<msgs::Double>(
        this->dataPtr->topicWindSpeed);

    this->dataPtr->windDirectionPub = this->dataPtr->node.Advertise<msgs::Vector3d>(
        this->dataPtr->topicWindDirection);
}

//////////////////////////////////////////////////
void USVWind::PreUpdate(
    const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm)
{
    GZ_PROFILE("USVWind::PreUpdate");

    if (this->dataPtr->previousTime == std::chrono::duration<double>(0))
    {
        this->dataPtr->previousTime = _info.simTime;
    }
   gzmsg << "PreUpdate" << std::endl;
 /*  if (this->dataPtr->vehicleModel == sim::kNullEntity)
    {
        auto entity = sim::entitiesFromScopedName(this->dataPtr->vehicleName, _ecm);
    }
    // Transform the force and torque to the world frame.
    math::Vector3d forceWorld = (*comPose).Rot().RotateVector(
        math::Vector3d(kForceSum(0), kForceSum(1), kForceSum(2)));
    math::Vector3d torqueWorld = (*comPose).Rot().RotateVector(
        math::Vector3d(kForceSum(3), kForceSum(4), kForceSum(5)));*/

    // Apply the force and torque at COM.
    //this->dataPtr->link.AddWorldWrench(_ecm, forceWorld, torqueWorld);
}

GZ_ADD_PLUGIN(vrx::USVWind,
              sim::System,
              USVWind::ISystemConfigure,
              USVWind::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::USVWind,
                    "vrx::USVWind")
