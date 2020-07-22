#include <mc_control/mc_global_controller.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/vacuum_gripper.h>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

// mc_panda
#include <mc_panda/devices/Pump.h>
#include <mc_panda/devices/PandaSensor.h>

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
  //TODO handle failure of robotFE(ip) with wrong ip
  PandaControlLoop(const std::string & name, const std::string & ip, size_t steps)
  : name(name), robotFE(ip), stateRobotFE(robotFE.readOnce()), controlPandaFE(stateRobotFE), steps(steps)
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
    updateSensor(&mc_rbdyn::Robot::encoderValues, &mc_rbdyn::Robot::encoderValues, stateRobotFE.q);
    updateSensor(&mc_rbdyn::Robot::encoderVelocities, &mc_rbdyn::Robot::encoderVelocities, stateRobotFE.dq);
    updateSensor(&mc_rbdyn::Robot::jointTorques, &mc_rbdyn::Robot::jointTorques, stateRobotFE.tau_J);
    commandMC = robotMC.mbc();
  }

  void updateSensors(mc_control::MCGlobalController & controllerMC)
  {
    auto & robotMC = controllerMC.controller().robots().robot(name);
    auto & realMC = controllerMC.controller().realRobots().robot(name);

    // update endeffector wrench //TODO generalize for multiple robots
    sva::ForceVecd wrench = sva::ForceVecd(Eigen::Vector6d::Zero());
    std::map<std::string, sva::ForceVecd> wrenches;
    wrenches.insert(std::make_pair("LeftHandForceSensor", wrench));
    wrench.force().x() = stateRobotFE.K_F_ext_hat_K[0];
    wrench.force().y() = stateRobotFE.K_F_ext_hat_K[1];
    wrench.force().z() = stateRobotFE.K_F_ext_hat_K[2];
    wrench.moment().x() = stateRobotFE.K_F_ext_hat_K[3];
    wrench.moment().y() = stateRobotFE.K_F_ext_hat_K[4];
    wrench.moment().z() = stateRobotFE.K_F_ext_hat_K[5];
    wrenches.find("LeftHandForceSensor")->second = wrench;
    controllerMC.setWrenches(wrenches);

    // update pandasensor-device //TODO check
    const std::string pandasensorDeviceName = "PandaSensor";
    if(controllerMC.controller().robots().robot(name).hasDevice<mc_panda::PandaSensor>(pandasensorDeviceName))
    {
      controllerMC.controller().robots().robot(name).device<mc_panda::PandaSensor>(pandasensorDeviceName).set_tau_ext_hat_filtered(stateRobotFE.tau_ext_hat_filtered);
      controllerMC.controller().robots().robot(name).device<mc_panda::PandaSensor>(pandasensorDeviceName).set_O_F_ext_hat_K(stateRobotFE.O_F_ext_hat_K);
      controllerMC.controller().robots().robot(name).device<mc_panda::PandaSensor>(pandasensorDeviceName).set_control_command_success_rate(stateRobotFE.control_command_success_rate);
      controllerMC.controller().robots().robot(name).device<mc_panda::PandaSensor>(pandasensorDeviceName).set_m_ee(stateRobotFE.m_ee);
      controllerMC.controller().robots().robot(name).device<mc_panda::PandaSensor>(pandasensorDeviceName).set_m_load(stateRobotFE.m_load);
      controllerMC.controller().robots().robot(name).device<mc_panda::PandaSensor>(pandasensorDeviceName).set_joint_contact(stateRobotFE.joint_contact);
      controllerMC.controller().robots().robot(name).device<mc_panda::PandaSensor>(pandasensorDeviceName).set_cartesian_contact(stateRobotFE.cartesian_contact);
    }

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
      robotMC.mbc().q[jIndex][0] = stateRobotFE.q[i];
      robotMC.mbc().jointTorque[jIndex][0] = stateRobotFE.tau_J[i];
    }
    robotMC.forwardKinematics();
    realMC.mbc() = robotMC.mbc();
  }

  void control_thread(mc_control::MCGlobalController & controllerMC)
  {
    controlPandaFE.control(robotFE,
                    [ this, &controllerMC ](const franka::RobotState & stateRobotIn, franka::Duration) ->
                    typename PandaControlType<cm>::ReturnT {
                      const std::string pandasensorDeviceName = "PandaSensor";
                      if(controllerMC.controller().robots().robot(name).hasDevice<mc_panda::PandaSensor>(pandasensorDeviceName))
                      {
                        if(controllerMC.controller().robots().robot(name).device<mc_panda::PandaSensor>(pandasensorDeviceName).stopRequested())
                        {
                          //TODO at this point also stop the real pump!
                          return franka::MotionFinished(controlPandaFE);
                        }
                      }

                      this->stateRobotFE = stateRobotIn;
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
                        return controlPandaFE.update(robotMC, commandMC, sensor_id % steps, steps);
                      }
                      return franka::MotionFinished(controlPandaFE);
                    });
  }

  std::string name;
  franka::Robot robotFE;
  franka::RobotState stateRobotFE;
  PandaControlType<cm> controlPandaFE;
  size_t sensor_id = 0;
  size_t steps = 1;
  rbd::MultiBodyConfig commandMC;
};

struct PumpControlLoop
{
  //TODO handle failure of suckerFE(ip) with wrong ip
  PumpControlLoop(const std::string & name, const std::string & ip, size_t steps)
  : name(name), suckerFE(ip), stateSuckerFE(suckerFE.readOnce()), steps(steps)
  {
  }

  void updateSensors(mc_control::MCGlobalController & controllerMC) //TODO check
  {
    // update pump-device
    const std::string pumpDeviceName = "Pump";
    if(controllerMC.controller().robots().robot(name).hasDevice<mc_panda::Pump>(pumpDeviceName))
    {
      controllerMC.controller().robots().robot(name).device<mc_panda::Pump>(pumpDeviceName).set_in_control_range(stateSuckerFE.in_control_range);
      controllerMC.controller().robots().robot(name).device<mc_panda::Pump>(pumpDeviceName).set_part_detached(stateSuckerFE.part_detached);
      controllerMC.controller().robots().robot(name).device<mc_panda::Pump>(pumpDeviceName).set_part_present(stateSuckerFE.part_present);
      if(stateSuckerFE.device_status==franka::VacuumGripperDeviceStatus::kGreen){
        controllerMC.controller().robots().robot(name).device<mc_panda::Pump>(pumpDeviceName).set_device_status_ok(true);
      }
      else{
        controllerMC.controller().robots().robot(name).device<mc_panda::Pump>(pumpDeviceName).set_device_status_ok(false);
      }
      controllerMC.controller().robots().robot(name).device<mc_panda::Pump>(pumpDeviceName).set_actual_power(stateSuckerFE.actual_power);
      controllerMC.controller().robots().robot(name).device<mc_panda::Pump>(pumpDeviceName).set_vacuum(stateSuckerFE.vacuum);
    }
  }

  void init(mc_control::MCGlobalController & controllerMC)
  {
    updateSensors(controllerMC); //TODO
  }

  void control_thread(mc_control::MCGlobalController & controllerMC) //TODO check
  {
    this->stateSuckerFE = suckerFE.readOnce();
    sensor_id += 1;
    if(sensor_id % steps == 0)
    {
      sensors_cv.notify_all();
      std::unique_lock<std::mutex> command_lock(command_mutex);
      command_cv.wait(command_lock);
    }

    const std::string pumpDeviceName = "Pump";
    if(controllerMC.controller().robots().robot(name).hasDevice<mc_panda::Pump>(pumpDeviceName))
    {
      auto & pumpDevice = controllerMC.controller().robots().robot(name).device<mc_panda::Pump>(pumpDeviceName);
      mc_panda::NextPumpCommand nc;
      nc = pumpDevice.NextPumpCommandRequested();
      switch(nc)
      {
        case mc_panda::NextPumpCommand::None:
        {
          mc_rtc::log::info("no command requested");
          break;
        }
        case mc_panda::NextPumpCommand::Vacuum:
        {
          mc_rtc::log::info("vacuum command requested");
          uint8_t vacuum;
          std::chrono::milliseconds timeout;
          pumpDevice.getVacuumCommandParams(vacuum, timeout);
          const bool vacuumOK = suckerFE.vacuum(vacuum, timeout);
          pumpDevice.setVacuumCommandResult(vacuumOK);
          mc_rtc::log::info("PUMP-CONTROL: vacuum command applied with the params vacuum {} and timeout {}, result: {}", std::to_string(vacuum), std::to_string(timeout.count()), vacuumOK);
          break;
        }
        case mc_panda::NextPumpCommand::Dropoff:
        {
          mc_rtc::log::info("dropoff command requested");
          std::chrono::milliseconds timeout;
          pumpDevice.getDropoffCommandParam(timeout);
          const bool dropoffOK = suckerFE.dropOff(timeout);
          pumpDevice.setDropoffCommandResult(dropoffOK);
          mc_rtc::log::info("PUMP-CONTROL: dropoff command applied with the param timeout {}, result: {}", std::to_string(timeout.count()), dropoffOK);
          break;
        }
        case mc_panda::NextPumpCommand::Stop:
        {
          mc_rtc::log::info("stop command requested");
          const bool stopOK = suckerFE.stop();
          pumpDevice.setStopCommandResult(stopOK);
          mc_rtc::log::info("PUMP-CONTROL: stop command applied, result: {}", stopOK);
          break;
        }
        default:
        {
          mc_rtc::log::error_and_throw<std::runtime_error>("PUMP-CONTROL: next command has unexpected value");
        }
      }
    }
  }

  std::string name;
  franka::VacuumGripper suckerFE;
  franka::VacuumGripperState stateSuckerFE;
  size_t sensor_id = 0;
  size_t steps = 1;
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

  // Initialize controlled panda robot and pump device
  std::vector<std::pair<PandaControlLoop<cm>, size_t>> pandas;
  std::vector<std::pair<PumpControlLoop, size_t>> pumps;
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
      pumps.emplace_back(std::make_pair<PumpControlLoop, size_t>({robotMC.name(), ip, n_steps}, 0)); //TODO which name?
      pumps.back().first.init(controllerMC);

      //start to log data for all devices //TODO check 
      const std::string pandasensorDeviceName = "PandaSensor";
      if(controllerMC.controller().robots().robot(robotMC.name()).hasDevice<mc_panda::PandaSensor>(pandasensorDeviceName))
      {
        controllerMC.controller().robots().robot(robotMC.name()).device<mc_panda::PandaSensor>(pandasensorDeviceName).addToLogger(controllerMC.controller().logger());
      }
      const std::string pumpDeviceName = "Pump";
      if(controllerMC.controller().robots().robot(robotMC.name()).hasDevice<mc_panda::Pump>(pumpDeviceName))
      {
        controllerMC.controller().robots().robot(robotMC.name()).device<mc_panda::Pump>(pumpDeviceName).addToLogger(controllerMC.controller().logger());
      }
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
  for(auto & pump : pumps)
  {
    controllerMC.controller().logger().addLogEntry(pump.first.name + "_pumpsensors_id", [&pump]() { return pump.second; });
  }
  controllerMC.init(robotsMC.robot().encoderValues());
  controllerMC.running = true;
  controllerMC.controller().gui()->addElement(
      {"Franka"}, mc_rtc::gui::Button("Stop controllerMC", [&controllerMC]() { controllerMC.running = false; }));
  
  // Start panda control loops and pump control loops
  std::vector<std::thread> panda_threads;
  std::vector<std::thread> pump_threads;
  for(auto & panda : pandas)
  {
    panda_threads.emplace_back([&panda, &controllerMC]() { panda.first.control_thread(controllerMC); });
  }
  for(auto & pump : pumps)
  {
    pump_threads.emplace_back([&pump, &controllerMC]() { pump.first.control_thread(controllerMC); });
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
        for(const auto & pump : pumps) //TODO
        {
          if(pump.first.sensor_id % n_steps != 0 || pump.first.sensor_id == pump.second)
          {
            return false;
          }
        }
        return true;
      });
      if(iter++ % 5 * freq == 0 && pandas.size() > 1)
      {
        mc_time::duration_us delay = mc_time::clock::now() - start;
        mc_rtc::log::info("[mc_franka] Measured delay between the pandas: {}us", delay.count()); //TODO we expect large delay for pumps
      }
      for(auto & panda : pandas)
      {
        panda.first.updateSensors(controllerMC);
        panda.second = panda.first.sensor_id;
      }
      for(auto & pump : pumps)
      {
        pump.first.updateSensors(controllerMC);
        pump.second = pump.first.sensor_id;
      }
      command_cv.notify_all();
    }
    controllerMC.run();
  }
  for(auto & th : panda_threads)
  {
    th.join();
  }
  for(auto & th : pump_threads)
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
//   std::vector<PumpControlLoop> pumpsFE;
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
