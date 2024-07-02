#include <iostream>

#include <mujoco/mujoco.h>

#include "obelisk_sim_robot.h"

namespace obelisk {

    template <typename ControlMessageT> class ObeliskMujocoRobot : public ObeliskSimRobot<ControlMessageT> {
      public:
        ObeliskMujocoRobot(const std::string& name) : ObeliskSimRobot<ControlMessageT>(name) {}

      protected:
      private:
        // MuJoCo data structures
        mjModel* m = NULL; // MuJoCo model
        mjData* d  = NULL; // MuJoCo data
        mjvCamera cam;     // abstract camera
        mjvOption opt;     // visualization options
        mjvScene scn;      // abstract scene
        mjrContext con;    // custom GPU context
    };

} // namespace obelisk
