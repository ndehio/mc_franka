#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syscall.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

// Taken from /usr/include/linux/sched/types.h
struct sched_attr {
  uint32_t size;

  uint32_t sched_policy;
  uint64_t sched_flags;

  /* SCHED_NORMAL, SCHED_BATCH */
  int32_t sched_nice;

  /* SCHED_FIFO, SCHED_RR */
  uint32_t sched_priority;

  /* SCHED_DEADLINE */
  uint64_t sched_runtime;
  uint64_t sched_deadline;
  uint64_t sched_period;
};


#include <mc_control/mc_global_controller.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <franka/exception.h>
#include <franka/robot.h>

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags)
{
  return syscall(__NR_sched_setattr, pid, attr, flags);
}

namespace mc_time
{

using duration_us = std::chrono::duration<double, std::micro>;
/** Always pick a steady clock */
using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                        std::chrono::high_resolution_clock,
                                        std::chrono::steady_clock>::type;

} // namespace mc_time

#include "PandaControlType.h"

std::condition_variable start_control_cv;
std::mutex start_control_mutex;
bool start_control_ready = false;
size_t control_id = 0;

template<ControlMode cm>
struct PandaControlLoop
{
  PandaControlLoop(const std::string & name, const std::string & ip, size_t steps, bool leader)
  : name(name), robot(ip), state(robot.readOnce()), control(state), steps(steps), leader(leader)
  {
    if(leader)
    {
      mc_rtc::log::info("[mc_franka] {} is the panda squad leader", name);
    }
    else
    {
      mc_rtc::log::info("[mc_franka] {} is a panda squad follower", name);
    }
    static auto panda_init_t = mc_time::clock::now();
    auto now = mc_time::clock::now();
    mc_time::duration_us dt = now - panda_init_t;
    mc_rtc::log::info("[mc_franka] Elapsed time since the creation of another PandaControlLoop: {}us", dt.count());
    panda_init_t = now;
  }

  void updateSensors(mc_rbdyn::Robot & robot, mc_rbdyn::Robot & real)
  {
    using get_sensor_t = const std::vector<double> & (mc_rbdyn::Robot::*)() const;
    using set_sensor_t = void (mc_rbdyn::Robot::*)(const std::vector<double> &);
    auto updateSensor = [&](get_sensor_t get, set_sensor_t set, const std::array<double, 7> & value) {
      auto sensor = (robot.*get)();
      if(sensor.size() != robot.refJointOrder().size())
      {
        sensor.resize(robot.refJointOrder().size());
      }
      for(size_t i = 0; i < value.size(); ++i)
      {
        sensor[i] = value[i];
      }
      (robot.*set)(sensor);
      (real.*set)(sensor);
    };
    updateSensor(&mc_rbdyn::Robot::encoderValues, &mc_rbdyn::Robot::encoderValues, state.q);
    updateSensor(&mc_rbdyn::Robot::encoderVelocities, &mc_rbdyn::Robot::encoderVelocities, state.dq);
    updateSensor(&mc_rbdyn::Robot::jointTorques, &mc_rbdyn::Robot::jointTorques, state.tau_J);
    command = robot.mbc();
  }

  void updateSensors(mc_control::MCGlobalController & controller)
  {
    auto & robot = controller.controller().robots().robot(name);
    auto & real = controller.controller().realRobots().robot(name);
    updateSensors(robot, real);
  }

  void init(mc_control::MCGlobalController & controller)
  {
    auto & robot = controller.controller().robots().robot(name);
    auto & real = controller.controller().realRobots().robot(name);
    updateSensors(robot, real);
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      auto jIndex = robot.jointIndexByName(rjo[i]);
      robot.mbc().q[jIndex][0] = state.q[i];
      robot.mbc().jointTorque[jIndex][0] = state.tau_J[i];
    }
    robot.forwardKinematics();
    real.mbc() = robot.mbc();
  }

  void control_thread(mc_control::MCGlobalController & controller)
  {
    {
      std::unique_lock<std::mutex> start_control_lock(start_control_mutex);
      start_control_cv.wait(start_control_lock, []() { return start_control_ready; });
    }
    control.control(robot, [
      this, &controller
    ](const franka::RobotState & stateIn, franka::Duration dt) -> typename PandaControlType<cm>::ReturnT {
      timespec tv;
      clock_gettime(CLOCK_REALTIME, &tv);
      control_t = tv.tv_sec + tv.tv_nsec * 1e-9;
      if(!started)
      {
        mc_rtc::log::info("[mc_franka] {} control loop started at {}", name, control_t);
        started = true;
      }
      this->state = stateIn;
      sensor_id += dt.toMSec();
      auto & robot = controller.controller().robots().robot(name);
      auto & real = controller.controller().realRobots().robot(name);
      if(sensor_id % steps == 0)
      {
        if(control_id != prev_control_id + dt.toMSec())
        {
          mc_rtc::log::warning("[mc_franka] {} missed control data (previous control id: {}, control id: {})", name,
                               prev_control_id, control_id);
        }
        updateSensors(controller);
        prev_control_id = control_id;
      }
      if(controller.running)
      {
        return control.update(robot, command, sensor_id % steps, steps);
      }
      return franka::MotionFinished(control);
    });
  }

  std::string name;
  franka::Robot robot;
  franka::RobotState state;
  PandaControlType<cm> control;
  size_t sensor_id = 0;
  size_t steps = 1;
  rbd::MultiBodyConfig command;
  bool started = false;
  double control_t = 0;
  size_t prev_control_id = 0;
  bool leader = false;
};

template<ControlMode cm>
void global_thread(mc_control::MCGlobalController::GlobalConfiguration & gconfig)
{
  /* Lock memory */
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
  {
    printf("mlockall failed: %m\n");
    std::exit(-2);
  }

  auto frankaConfig = gconfig.config("Franka");
  auto ignoredRobots = frankaConfig("ignored", std::vector<std::string>{});
  mc_control::MCGlobalController controller(gconfig);
  if(controller.controller().timeStep < 0.001)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("mc_rtc cannot run faster than 1kHz with mc_franka");
  }
  size_t n_steps = std::ceil(controller.controller().timeStep / 0.001);
  size_t freq = std::ceil(1 / controller.controller().timeStep);
  mc_rtc::log::info("mc_rtc running at {}Hz, will interpolate every {} panda control step", freq, n_steps);
  auto & robots = controller.controller().robots();
  // Initialize all real robots
  for(size_t i = controller.realRobots().size(); i < robots.size(); ++i)
  {
    controller.realRobots().robotCopy(robots.robot(i));
    controller.realRobots().robots().back().name(robots.robot(i).name());
  }
  // Initialize controlled panda robot
  std::vector<std::pair<PandaControlLoop<cm>, size_t>> pandas;
  {
    std::vector<std::thread> panda_init_threads;
    std::mutex pandas_init_mutex;
    std::condition_variable pandas_init_cv;
    bool pandas_init_ready = false;
    bool leader = true;
    for(auto & robot : robots)
    {
      if(robot.mb().nrDof() == 0)
      {
        continue;
      }
      if(std::find(ignoredRobots.begin(), ignoredRobots.end(), robot.name()) != ignoredRobots.end())
      {
        continue;
      }
      if(frankaConfig.has(robot.name()))
      {
        std::string ip = frankaConfig(robot.name())("ip");
        panda_init_threads.emplace_back([&, ip, leader]() {
          {
            std::unique_lock<std::mutex> lock(pandas_init_mutex);
            pandas_init_cv.wait(lock, [&pandas_init_ready]() { return pandas_init_ready; });
          }
          auto pair = std::make_pair<PandaControlLoop<cm>, size_t>({robot.name(), ip, n_steps, leader}, 0);
          std::unique_lock<std::mutex> lock(pandas_init_mutex);
          pandas.emplace_back(std::move(pair));
        });
        leader = false;
      }
      else
      {
        mc_rtc::log::warning("The loaded controller uses an actuated robot that is not configured and not ignored: {}",
                             robot.name());
      }
    }
    pandas_init_ready = true;
    pandas_init_cv.notify_all();
    for(auto & th : panda_init_threads)
    {
      th.join();
    }
  }
  for(auto & panda : pandas)
  {
    panda.first.init(controller);
    controller.controller().logger().addLogEntry(panda.first.name + "_sensors_id", [&panda]() { return panda.second; });
    controller.controller().logger().addLogEntry(panda.first.name + "_control_t",
                                                 [&panda]() { return panda.first.control_t; });
  }
  controller.init(robots.robot().encoderValues());
  controller.running = true;
  controller.controller().gui()->addElement(
      {"Franka"}, mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));
  // Start panda control loops
  std::vector<std::thread> panda_threads;
  for(auto & panda : pandas)
  {
    panda_threads.emplace_back([&panda, &controller]() { panda.first.control_thread(controller); });
  }
  start_control_ready = true;
  start_control_cv.notify_all();
  /** Setup scheduler to wake-up this thread at the required frequency */
  uint64_t cycle_ns = 1e9 * controller.timestep();
  struct sched_attr attr;
  attr.sched_policy = SCHED_DEADLINE;
  attr.sched_runtime = attr.sched_deadline = attr.sched_period = cycle_ns; // nanoseconds
  /* Set scheduler policy */
  if(sched_setattr(0, &attr, 0) < 0)
  {
    printf("sched_setattr failed: %m\n");
    std::exit(2);
  }
  size_t iter = 0;
  while(controller.running)
  {
    controller.run();
    control_id++;
    // Sleep until the next cycle
    sched_yield();
  }
  for(auto & th : panda_threads)
  {
    th.join();
  }
}

template<ControlMode cm>
struct GlobalControlLoop
{
  mc_control::MCGlobalController & controller;
  std::vector<PandaControlLoop<cm>> robots;
};

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
