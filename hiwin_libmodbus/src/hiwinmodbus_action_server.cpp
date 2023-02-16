#include <iostream>
#include <functional>
#include <memory>
#include <thread>

#include "hiwin_action_interfaces/action/hiwinmodbus.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "hiwin_libmodbus/hiwin_libmodbus.hpp"

#include "hiwin_libmodbus//visibility_control.h"

namespace hiwinmodbus_action_server_cpp
{

class HiwinmodbusActionServer : public rclcpp::Node
{
public:
  using Hiwinmodbus = hiwin_action_interfaces::action::Hiwinmodbus;
  using GoalHandleHiwinmodbus = rclcpp_action::ServerGoalHandle<Hiwinmodbus>;

  explicit HiwinmodbusActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("Hiwinmodbus_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Hiwinmodbus>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "Hiwinmodbus",
      std::bind(&HiwinmodbusActionServer::handle_goal, this, _1, _2),
      std::bind(&HiwinmodbusActionServer::handle_cancel, this, _1),
      std::bind(&HiwinmodbusActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Hiwinmodbus>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Hiwinmodbus::Goal> command)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with mode %s", command->mode);
    (void)uuid;
    // reject if no mode
    // HiwinLibmodbus hiwinlibmodbus;

    std::cout<<command->mode<<std::endl;

    if (command->mode=="") {
      std::cout<<"--------------------"<<std::endl;
      return rclcpp_action::GoalResponse::REJECT;
    }

//     if (command->mode == "connect") {
//       std::cout<<"afjioaljfl;kjaklfaf;"<<std::endl;
//         hiwinlibmodbus.libModbus_Connect(command->ip_address);
//         hiwinlibmodbus.Holding_Registers_init();
//         hiwinlibmodbus.MOTOR_EXCITE();
//       }

//     else if (command->mode == "PTP"){
//         hiwinlibmodbus.PTP(command->type, command->vel, command->acc, command->tool, command->base, command->angle);    
//     }
//     else if (command->mode == "LIN"){
//         hiwinlibmodbus.LIN(command->type, command->vel, command->acc, command->tool, command->base, command->xyz);    
//     }
//     else if (command->mode == "CIRC"){
//         hiwinlibmodbus.CIRC(command->vel, command->acc, command->tool, command->base, command->circ_s, command->circ_end);    
//     }
//     else if (command->mode == "DO"){
//         hiwinlibmodbus.DO(command->digital_output, command->onoff);    
//     }
// /*********************
//   value   joint 
//   0~5 -> A1~A6 
//   value  Cartesian
//   6~11 -> XYZABC 
// *********************/
//     else if (command->mode == "JOG"){
//         hiwinlibmodbus.JOG(command->joint, command->dir);    
//     }
//     else if (command->mode == "HOME"){
//         hiwinlibmodbus.HOME();
//     }
//     else if (command->mode == "close"){
//         hiwinlibmodbus.Modbus_Close();
//     }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleHiwinmodbus> command_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)command_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleHiwinmodbus> command_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&HiwinmodbusActionServer::execute, this, _1), command_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleHiwinmodbus> command_handle)
  {
    HiwinLibmodbus hiwinlibmodbus;
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto command = command_handle->get_goal();
    auto feedback = std::make_shared<Hiwinmodbus::Feedback>();
    
    auto & sequence = feedback->partial_sequence;
    // sequence.push_back(0);
    // sequence.push_back(1);
    auto result = std::make_shared<Hiwinmodbus::Result>();

    if (command->mode == "connect") {
      // std::cout<<"afjioaljfl;kjaklfaf;"<<std::endl;
        hiwinlibmodbus.libModbus_Connect(command->ip_address);
        hiwinlibmodbus.Holding_Registers_init();
        hiwinlibmodbus.MOTOR_EXCITE();
      }

    else if (command->mode == "PTP"){
        hiwinlibmodbus.PTP(command->type, command->vel, command->acc, command->tool, command->base, command->angle);    
    }
    else if (command->mode == "LIN"){
        hiwinlibmodbus.LIN(command->type, command->vel, command->acc, command->tool, command->base, command->xyz);    
    }
    else if (command->mode == "CIRC"){
        hiwinlibmodbus.CIRC(command->vel, command->acc, command->tool, command->base, command->circ_s, command->circ_end);    
    }
    else if (command->mode == "DO"){
        hiwinlibmodbus.DO(command->digital_output, command->onoff);    
    }
/*********************
  value   joint 
  0~5 -> A1~A6 
  value  Cartesian
  6~11 -> XYZABC 
*********************/
    else if (command->mode == "JOG"){
        hiwinlibmodbus.JOG(command->joint, command->dir);    
    }
    else if (command->mode == "HOME"){
        std::cout<<"afjioaljfl;kjaklfaf;"<<std::endl;
        hiwinlibmodbus.HOME();
    }
    else if (command->mode == "close"){
        hiwinlibmodbus.Modbus_Close();
    }


    // for (int i = 1; (i < goal->type) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     result->sequence = sequence;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal Canceled");
    //     return;
    //   }
    // }


    //   // Update sequence
    //   sequence.push_back(sequence[i] + sequence[i - 1]);
    //   // Publish feedback
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish Feedback");

    loop_rate.sleep();
    // }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      command_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

};  // class HiwinmodbusActionServer
};// namespace hiwinmodbus_action_server_cpp


// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);

//   auto action_server = std::make_shared<hiwinmodbus_action_server_cpp::HiwinmodbusActionServer>();

//   rclcpp::spin(action_server);

//   rclcpp::shutdown();
//   return 0;
// }


RCLCPP_COMPONENTS_REGISTER_NODE(hiwinmodbus_action_server_cpp::HiwinmodbusActionServer)