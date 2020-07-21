#include <mc_control/mc_global_controller.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <franka/exception.h>
#include <franka/robot.h>

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

namespace mc_time
{

using duration_us = std::chrono::duration<double, std::micro>;
/** Always pick a steady clock */
using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                        std::chrono::high_resolution_clock,
                                        std::chrono::steady_clock>::type;

} // namespace mc_time

#include "PandaControlType.h"

std::condition_variable sensors_cv;
std::mutex sensors_mutex;
std::condition_variable command_cv;
std::mutex command_mutex;

template<ControlMode cm>
struct PandaControlLoop
{
  PandaControlLoop(const std::string & name, const std::string & ip, size_t steps)
  : name(name), robotFE(ip), stateFE(robotFE.readOnce()), controlFE(stateFE), steps(steps)
  {
  }

  void updateSensors(mc_rbdyn::Robot & robotMC, mc_rbdyn::Robot & realMC)
  {
    using get_sensor_t = const std::vector<double> & (mc_rbdyn::Robot::*)() const;
    using set_sensor_t = void (mc_rbdyn::Robot::*)(const std::vector<double> &);
    auto updateSensor = [&](get_sensor_t get, set_sensor_t set, const std::array<double, 7> & value) {
      auto sensor = (robotMC.*get)();
      if(sensor.size() != robotMC.refJointOrder().size())
      {
        sensor.resize(robotMC.refJointOrder().size());
      }
      for(size_t i = 0; i < value.size(); ++i)
      {
        sensor[i] = value[i];
      }
      (robotMC.*set)(sensor);
      (realMC.*set)(sensor);
    };
    updateSensor(&mc_rbdyn::Robot::encoderValues, &mc_rbdyn::Robot::encoderValues, stateFE.q);
    updateSensor(&mc_rbdyn::Robot::encoderVelocities, &mc_rbdyn::Robot::encoderVelocities, stateFE.dq);
    updateSensor(&mc_rbdyn::Robot::jointTorques, &mc_rbdyn::Robot::jointTorques, stateFE.tau_J);
    commandMC = robotMC.mbc();
  }

  void updateSensors(mc_control::MCGlobalController & controllerMC)
  {
    auto & robotMC = controllerMC.controller().robots().robot(name);
    auto & realMC = controllerMC.controller().realRobots().robot(name);
    updateSensors(robotMC, realMC);
  }

  void init(mc_control::MCGlobalController & controllerMC)
  {
    auto & robotMC = controllerMC.controller().robots().robot(name);
    auto & realMC = controllerMC.controller().realRobots().robot(name);
    updateSensors(robotMC, realMC);
    const auto & rjo = robotMC.refJointOrder();
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      auto jIndex = robotMC.jointIndexByName(rjo[i]);
      robotMC.mbc().q[jIndex][0] = stateFE.q[i];
      robotMC.mbc().jointTorque[jIndex][0] = stateFE.tau_J[i];
    }
    robotMC.forwardKinematics();
    realMC.mbc() = robotMC.mbc();
  }

  void control_thread(mc_control::MCGlobalController & controllerMC)
  {
    controlFE.control(robotFE,
                    [ this, &controllerMC ](const franka::RobotState & stateIn, franka::Duration) ->
                    typename PandaControlType<cm>::ReturnT {
                      this->stateFE = stateIn;
                      sensor_id += 1;
                      auto & robotMC = controllerMC.controller().robots().robot(name);
                      //auto & realMC = controllerMC.controller().realRobots().robot(name); //TODO not required?
                      if(sensor_id % steps == 0)
                      {
                        sensors_cv.notify_all();
                        std::unique_lock<std::mutex> command_lock(command_mutex);
                        command_cv.wait(command_lock);
                      }
                      if(controllerMC.running)
                      {
                        return controlFE.update(robotMC, commandMC, sensor_id % steps, steps);
                      }
                      return franka::MotionFinished(controlFE);
                    });
  }

  std::string name;
  franka::Robot robotFE;
  franka::RobotState stateFE;
  PandaControlType<cm> controlFE;
  size_t sensor_id = 0;
  size_t steps = 1;
  rbd::MultiBodyConfig commandMC;
};

template<ControlMode cm>
void global_thread(mc_control::MCGlobalController::GlobalConfiguration & gconfig)
{
  auto frankaConfig = gconfig.config("Franka");
  auto ignoredRobots = frankaConfig("ignored", std::vector<std::string>{});
  mc_control::MCGlobalController controllerMC(gconfig);
  if(controllerMC.controller().timeStep < 0.001)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("mc_rtc cannot run faster than 1kHz with mc_franka");
  }
  size_t n_steps = std::ceil(controllerMC.controller().timeStep / 0.001);
  size_t freq = std::ceil(1 / controllerMC.controller().timeStep);
  mc_rtc::log::info("mc_rtc running at {}Hz, will interpolate every {} panda control step", freq, n_steps);
  auto & robotsMC = controllerMC.controller().robots();
  // Initialize all real robots
  for(size_t i = controllerMC.realRobots().size(); i < robotsMC.size(); ++i)
  {
    controllerMC.realRobots().robotCopy(robotsMC.robot(i));
    controllerMC.realRobots().robots().back().name(robotsMC.robot(i).name());
  }
  // Initialize controlled panda robot
  std::vector<std::pair<PandaControlLoop<cm>, size_t>> pandas;
  for(auto & robotMC : robotsMC)
  {
    if(robotMC.mb().nrDof() == 0)
    {
      continue;
    }
    if(std::find(ignoredRobots.begin(), ignoredRobots.end(), robotMC.name()) != ignoredRobots.end())
    {
      continue;
    }
    if(frankaConfig.has(robotMC.name()))
    {
      std::string ip = frankaConfig(robotMC.name())("ip");
      pandas.emplace_back(std::make_pair<PandaControlLoop<cm>, size_t>({robotMC.name(), ip, n_steps}, 0));
      pandas.back().first.init(controllerMC);
    }
    else
    {
      mc_rtc::log::warning("The loaded controllerMC uses an actuated robot that is not configured and not ignored: {}",
                           robotMC.name());
    }
  }
  for(auto & panda : pandas)
  {
    controllerMC.controller().logger().addLogEntry(panda.first.name + "_sensors_id", [&panda]() { return panda.second; });
  }
  controllerMC.init(robotsMC.robot().encoderValues());
  controllerMC.running = true;
  controllerMC.controller().gui()->addElement(
      {"Franka"}, mc_rtc::gui::Button("Stop controllerMC", [&controllerMC]() { controllerMC.running = false; }));
  // Start panda control loops
  std::vector<std::thread> panda_threads;
  for(auto & panda : pandas)
  {
    panda_threads.emplace_back([&panda, &controllerMC]() { panda.first.control_thread(controllerMC); });
  }
  size_t iter = 0;
  while(controllerMC.running)
  {
    {
      std::unique_lock<std::mutex> sensors_lock(sensors_mutex);
      bool start_measure = false;
      std::chrono::time_point<mc_time::clock> start;
      sensors_cv.wait(sensors_lock, [&]() {
        if(!start_measure)
        {
          start_measure = true;
          start = mc_time::clock::now();
        }
        for(const auto & panda : pandas)
        {
          if(panda.first.sensor_id % n_steps != 0 || panda.first.sensor_id == panda.second)
          {
            return false;
          }
        }
        return true;
      });
      if(iter++ % 5 * freq == 0 && pandas.size() > 1)
      {
        mc_time::duration_us delay = mc_time::clock::now() - start;
        mc_rtc::log::info("[mc_franka] Measured delay between the pandas: {}us", delay.count());
      }
      for(auto & panda : pandas)
      {
        panda.first.updateSensors(controllerMC);
        panda.second = panda.first.sensor_id;
      }
      command_cv.notify_all();
    }
    controllerMC.run();
  }
  for(auto & th : panda_threads)
  {
    th.join();
  }
}

//TODO is this struct required?
// template<ControlMode cm>
// struct GlobalControlLoop
// {
//   mc_control::MCGlobalController & controllerMC;
//   std::vector<PandaControlLoop<cm>> robotsFE;
// };

int main(int argc, char * argv[])
{
  std::string conf_file = "";
  po::options_description desc("MCUDPControl options");
  // clang-format off
  desc.add_options()
    ("help", "Display help message")
    ("conf,f", po::value<std::string>(&conf_file), "Configuration file");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << "\n";
    std::cout << "see etc/sample.yaml for libfranka configuration\n";
    return 1;
  }

  mc_control::MCGlobalController::GlobalConfiguration gconfig(conf_file, nullptr);
  if(!gconfig.config.has("Franka"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No Franka section in the configuration, see etc/sample.yaml for an example");
  }
  auto frankaConfig = gconfig.config("Franka");
  ControlMode cm = frankaConfig("ControlMode", ControlMode::Velocity);
  try
  {
    switch(cm)
    {
      case ControlMode::Position:
        global_thread<ControlMode::Position>(gconfig);
        break;
      case ControlMode::Velocity:
        global_thread<ControlMode::Velocity>(gconfig);
        break;
      case ControlMode::Torque:
        global_thread<ControlMode::Torque>(gconfig);
        break;
    }
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return 1;
  }
  return 0;
}
