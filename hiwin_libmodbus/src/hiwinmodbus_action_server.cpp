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

namespace hiwin_modbus_cpp
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
    std::shared_ptr<const Hiwinmodbus::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with mode %s", goal->mode);
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->type==NULL) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleHiwinmodbus> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void convert_double(const double * temp){

    double *convert_temp = const_cast<double *>(temp);

  }

  void execute(const std::shared_ptr<GoalHandleHiwinmodbus> goal_handle)
  {
    HiwinLibmodbus hiwinlibmodbus;
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Hiwinmodbus::Feedback>();
    auto & sequence = feedback->partial_sequence;
    // sequence.push_back(0);
    // sequence.push_back(1);
    auto result = std::make_shared<Hiwinmodbus::Result>();

    // for (int i = 1; (i < goal->type) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     result->sequence = sequence;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal Canceled");
    //     return;
    //   }
    // }


    if (goal->mode == "connect") {
        hiwinlibmodbus.libModbus_Connect(goal->ip_address);
        hiwinlibmodbus.Holding_Registers_init();
        hiwinlibmodbus.MOTOR_EXCITE();
      }

    // else if (goal->mode == 'MOTOR_EXCITE'){
    // }
    else if (goal->mode == "PTP"){
      // double *angle = goal->angle
        // double* angle = &goal->angle[0];
        // double a = &goal->angle[0];
        double a = &goal->angle;
        convert_double(&a);
        double *angle = &a;
        hiwinlibmodbus.PTP(goal->type, goal->vel, goal->acc, goal->tool, goal->base, angle);    
    }
    // else if (goal->mode == "LIN"){
    //   const double* xyz = &goal->xyz[0];
    //     hiwinlibmodbus.LIN(goal->type, goal->vel, goal->acc, goal->tool, goal->base, xyz);    
    // }
    // else if (goal->mode == "CIRC"){
    //     double* circ_s = &goal->circ_s[0];
    //     double* circ_end = &goal->circ_end[0];
    //     hiwinlibmodbus.CIRC(goal->vel, goal->acc, goal->tool, goal->base, circ_s, circ_end);    
    // }
/*********************
  value   joint 
  0~5 -> A1~A6 
  value  Cartesian
  6~11 -> XYZABC 
*********************/
    else if (goal->mode == "JOG"){
        hiwinlibmodbus.JOG(goal->joint, goal->dir);    
    }
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
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleHiwinmodbus> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&HiwinmodbusActionServer::execute, this, _1), goal_handle}.detach();
  }
};  // class HiwinmodbusActionServer
};// namespace hiwin_libmodbus_cpp

// HiwinLibmodbus* HiwinLibmodbusConstructor()
// {
//   return new HiwinLibmodbus();
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // auto action_server = std::make_shared<HiwinmodbusActionServer>();

  // rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}


// RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)