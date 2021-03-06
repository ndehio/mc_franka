#include <mc_control/mc_global_controller.h>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/vacuum_gripper.h>
#include <iostream>

// mc_panda
#include <mc_panda/devices/PandaSensor.h>

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

void usage(const char * prog)
{
  std::cerr << "[usage] " << prog << " <fci-ip>\n";
}

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    usage(argv[0]);
    return 1;
  }

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
    sva::ForceVecd wrench;
    bool is_singular;
  } print_data{};
  std::atomic_bool running{true};
  const double print_rate = 2.0; //number of prints per second
  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          std::cout << "end-effector force= " <<  print_data.wrench.force().transpose() << " and moment =" << print_data.wrench.moment().transpose() << " and is_singular: "<< print_data.is_singular << std::endl;
          print_data.has_data = false;
        }
        print_data.mutex.unlock();
      }
    }
  });

  try
  {
    mc_rtc::log::success("executing with ip={}", argv[1]);

    // Initialize the robot
    franka::Robot robot(argv[1]);
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}}); //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.cpp#L18
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}}); //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.cpp#L19
    // robot.setJointImpedance({{100,100,100,100,100,100,100}}); 
    // robot.setCartesianImpedance({{100,100,100,10,10,10}});
    robot.setCollisionBehavior( //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/generate_joint_velocity_motion.cpp#L39
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    
    
    robot.setCollisionBehavior( //TODO: be careful with this mode!
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});

    // Get the initial state of the robot
    const auto state = robot.readOnce();
    const int dof = state.q.size();

    // Initialize mc_rtc
    mc_control::MCGlobalController controller;
    if(controller.controller().timeStep != 0.001)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("mc_rtc must be configured to run at 1kHz");
    }
    sva::ForceVecd wrench = sva::ForceVecd(Eigen::Vector6d::Zero());
    std::map<std::string, sva::ForceVecd> wrenches;
    wrenches.insert(std::make_pair("LeftHandForceSensor", wrench));
    std::vector<double> init_q_vector;
    std::vector<double> q_vector;
    std::vector<double> dq_vector;
    std::vector<double> tau_vector;
    std::vector<double> dtau_vector;
    init_q_vector.resize(dof);
    q_vector.resize(dof);
    dq_vector.resize(dof);
    tau_vector.resize(dof);
    dtau_vector.resize(dof);
    for(size_t i = 0; i < dof; ++i)
    {
      init_q_vector.at(i)=state.q[i];
      q_vector.at(i)=state.q[i];
      dq_vector.at(i)=state.dq[i];
      tau_vector.at(i)=state.tau_J[i];
      dtau_vector.at(i)=state.tau_J[i];
    }
    // FIXME Temporary work-around until we handle the gripper
    int counter=dof-1;
    while(controller.robot().refJointOrder().size() > init_q_vector.size())
    {
      counter++;
      init_q_vector.at(counter)=0;
    }
    controller.setEncoderValues(q_vector);
    controller.setEncoderVelocities(dq_vector);
    controller.setJointTorques(tau_vector);
    // controller.setJointTorques(dtau_vector); //TODO extend interface
    controller.setWrenches(wrenches);
    controller.init(init_q_vector);
    controller.running = true;
    controller.controller().gui()->addElement({"Franka"},
                                              mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));

    // Initialize the the mc_panda PandaSensor as 'sensor' if the robot-module has such a device
    bool sensorAvailable = false;
    std::string sensorDeviceName = "PandaSensor";
    if(controller.robot().hasDevice<mc_panda::PandaSensor>(sensorDeviceName))
    {
      controller.robot().device<mc_panda::PandaSensor>(sensorDeviceName).addToLogger(controller.controller().logger());
      sensorAvailable = true;
      mc_rtc::log::info("RobotModule has a PandaSensor named {}", sensorDeviceName);
    }
    else{
      mc_rtc::log::warning("RobotModule does not have a PandaSensor named {}", sensorDeviceName);
      mc_rtc::log::warning("PandaSensor functionality will not be available");
    }
    
    // Start the control loop in velocity-control
    bool is_singular = false;;
    franka::Model model = robot.loadModel();
    franka::JointVelocities output_dq(state.dq);
    robot.control([&print_data,&model,&controller,&sensorAvailable,&sensorDeviceName,&q_vector,&dq_vector,&tau_vector,&dtau_vector,&wrench,&wrenches,&is_singular,&output_dq](const franka::RobotState & state, franka::Duration period) -> franka::JointVelocities
    {
      if(sensorAvailable)
      {
        if(controller.robot().device<mc_panda::PandaSensor>(sensorDeviceName).stopRequested())
        {
          mc_rtc::log::warning("PandaSensor device requests to stop the robot");
          return franka::MotionFinished(output_dq);
        }
      }

      for(size_t i = 0; i < state.q.size(); ++i)
      {
        q_vector[i] = state.q[i];
        dq_vector[i] = state.dq[i];
        tau_vector[i] = state.tau_J[i];
        dtau_vector[i] = state.dtau_J[i];
      }

      wrench.force().x() = state.K_F_ext_hat_K[0];
      wrench.force().y() = state.K_F_ext_hat_K[1];
      wrench.force().z() = state.K_F_ext_hat_K[2];
      wrench.moment().x() = state.K_F_ext_hat_K[3];
      wrench.moment().y() = state.K_F_ext_hat_K[4];
      wrench.moment().z() = state.K_F_ext_hat_K[5];
      
      is_singular = false;
      if(wrench.force().x()==0 && wrench.force().y()==0 && wrench.force().z()==0 && wrench.moment().x()==0 && wrench.moment().y()==0 && wrench.moment().z()==0 ){
        // mc_rtc::log::warning("libfranka wrench is zero"); //NOTE this happens if smallest singular value of the jacobian is lower than 0.08...
        is_singular = true;
        const std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
        const Eigen::Matrix<double, 6, 7> jacobian(jacobian_array.data());
        const Eigen::Matrix<double, 7, 1> torques(state.tau_ext_hat_filtered.data());

        Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>> svdT = Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>>();
        svdT.compute(jacobian.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
        // const Eigen::VectorXd wrench_vector = svdT.solve(torques); //this solution maybe faster, but does not correspond to the libfranka wrenches for full-rank Jacobian
        const double rank_ = svdT.rank();
        mc_rtc::log::warning("rank_ = {}", rank_);
        const Eigen::MatrixXd U = svdT.matrixU();
        const Eigen::MatrixXd V = svdT.matrixV();
        const Eigen::VectorXd Svec  = svdT.singularValues();
        Eigen::Matrix<double, 6, 6> Sinv = Eigen::Matrix<double, 6, 6>::Zero();
        for(int i=0; i<6; i++){
          if (Svec(i)>=0.08) //important threshold, identical to the libfranka threshold!
          {
            Sinv(i,i) = 1.0 / Svec(i);
          }
        }
        const Eigen::VectorXd wrench_vector = V * Sinv * U.transpose() * torques;

        wrench.force() = wrench_vector.head<3>();
        wrench.moment() = wrench_vector.tail<3>();

        if(sensorAvailable)
        {
          auto & pandaSensor = controller.robot().device<mc_panda::PandaSensor>(sensorDeviceName);
          pandaSensor.set_singular_values(Svec);
        }
        // LOG_SUCCESS("special force = [" << wrench.force().x() << ", " << wrench.force().y() << ", " << wrench.force().z() << "] and moment =[" << wrench.moment().x() << ", " << wrench.moment().y() << ", " << wrench.moment().z() << "]")
      }

      wrenches.find("LeftHandForceSensor")->second = wrench;

      // int randomNumber = rand() % 500 + 1; //generate number between 1 and 1000
      // if (randomNumber==500){
      //   LOG_INFO("force = [" << wrench.force().x() << ", " << wrench.force().y() << ", " << wrench.force().z() << "] and moment =[" << wrench.moment().x() << ", " << wrench.moment().y() << ", " << wrench.moment().z() << "]")
      // }

      if(wrench.force().norm() > 230 && is_singular==false){
        mc_rtc::log::error("stopping because wrench.force().norm() = {}", wrench.force().norm())
        LOG_ERROR("force = [" << wrench.force().x() << ", " << wrench.force().y() << ", " << wrench.force().z() << "] and moment =[" << wrench.moment().x() << ", " << wrench.moment().y() << ", " << wrench.moment().z() << "]")
        return franka::MotionFinished(output_dq);
      }
      
      controller.setEncoderValues(q_vector);
      controller.setEncoderVelocities(dq_vector);
      controller.setJointTorques(tau_vector);
      // controller.setJointDTorques(ddtau_vector); //TODO extend interface
      controller.setWrenches(wrenches);
      if(sensorAvailable)
      {
        auto & pandaSensor = controller.robot().device<mc_panda::PandaSensor>(sensorDeviceName);
        pandaSensor.set_tau_ext_hat_filtered(state.tau_ext_hat_filtered);
        pandaSensor.set_O_F_ext_hat_K(state.O_F_ext_hat_K);
        pandaSensor.set_K_F_ext_hat_K(state.K_F_ext_hat_K);
        pandaSensor.set_control_command_success_rate(state.control_command_success_rate);
        pandaSensor.set_m_ee(state.m_ee);
        pandaSensor.set_m_load(state.m_load);
        pandaSensor.set_joint_contact(state.joint_contact);
        pandaSensor.set_cartesian_contact(state.cartesian_contact);
        // pandaSensor.set_singular_values();
        pandaSensor.set_joint_positions(state.q);
        pandaSensor.set_joint_velocities(state.dq);
        pandaSensor.set_joint_torques(state.tau_J);
        pandaSensor.set_joint_dtorques(state.dtau_J);
      }

      if(controller.running && controller.run())
      {
        const auto & rjo = controller.robot().refJointOrder();
        for(size_t i = 0; i < output_dq.dq.size(); ++i)
        {
          const auto & j = rjo[i];
          output_dq.dq[i] = controller.robot().mbc().alpha[controller.robot().jointIndexByName(j)][0];
        }

        // Update data to print.
        if (print_data.mutex.try_lock()) {
          print_data.has_data = true;
          print_data.robot_state = state;
          print_data.wrench = wrench;
          print_data.is_singular = is_singular;
          print_data.mutex.unlock();
        }
        return output_dq;
      }
      output_dq.dq = state.dq;
      return franka::MotionFinished(output_dq);
    // }, franka::ControllerMode::kJointImpedance, true, 100); //default parameters
    }, franka::ControllerMode::kJointImpedance, true, 1000); //increased kDefaultCutoffFrequency 

    mc_rtc::log::info("libfranka control loop has ended, stopping the robot now.");
    robot.stop();
    mc_rtc::log::info("panda succesfully stopped");
  }
  catch(const franka::Exception & e)
  {
    running = false;
    std::cerr << "franka::Exception " << e.what() << "\n";
    // return 1;
  }
  mc_rtc::log::info("stop the print thread");
  if (print_thread.joinable()) {
    print_thread.join();
  }
  mc_rtc::log::info("Good bye");
  return 0;
}
