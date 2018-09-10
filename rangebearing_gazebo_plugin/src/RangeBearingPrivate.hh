#ifndef _RANGEBEARING_PRIVATE_HH_
#define _RANGEBEARING_PRIVATE_HH_

#include <mutex>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Ray sensor private data.
    class RangeBearingPrivate
    {
      
      /// \brief Mutex to protect laserMsg
      public: std::mutex mutex;

    };
  }
}

#endif
