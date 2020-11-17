// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}}); //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.cpp#L18
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}}); //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.cpp#L19

    //TODO: be careful with this mode!
    robot.setCollisionBehavior(
        {{2000.0, 2000.0, 1800.0, 1800.0, 1600.0, 1400.0, 1200.0}}, {{2000.0, 2000.0, 1800.0, 1800.0, 1600.0, 1400.0, 1200.0}},
        {{2000.0, 2000.0, 1800.0, 1800.0, 1600.0, 1400.0, 1200.0}}, {{2000.0, 2000.0, 1800.0, 1800.0, 1600.0, 1400.0, 1200.0}},
        {{2000.0, 2000.0, 2000.0, 2500.0, 2500.0, 2500.0}}, {{2000.0, 2000.0, 2000.0, 2500.0, 2500.0, 2500.0}},
        {{2000.0, 2000.0, 2000.0, 2500.0, 2500.0, 2500.0}}, {{2000.0, 2000.0, 2000.0, 2500.0, 2500.0, 2500.0}});
    
    franka::Model model = robot.loadModel();

    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&model](
                const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {

      const std::array<double, 7> coriolis = model.coriolis(state);

      // Compute torque command
      std::array<double, 7> tau_d_calculated;
      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] = coriolis[i];
      }

      // The following line is only necessary for printing the rate limited torque. As we activated
      // rate limiting for the control loop (activated by default), the torque would anyway be
      // adjusted!
      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

      // Send torque command.
      return tau_d_rate_limited;
    };

    // Start real-time control loop.
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}