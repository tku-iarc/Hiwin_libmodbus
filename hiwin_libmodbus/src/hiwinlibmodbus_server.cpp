#include <iostream>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "hiwin_interfaces/srv/robot_command.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"
#include "hiwin_libmodbus/hiwin_libmodbus.hpp"

#include "hiwin_libmodbus//visibility_control.h"

HiwinLibmodbus hiwinlibmodbus;
class HiwinlibmodbusServiceServer : public rclcpp::Node
{
    
    public:
    HiwinlibmodbusServiceServer() : Node("hiwinmodbus_server")
    {
        server_ = create_service<hiwin_interfaces::srv::RobotCommand>(
                "hiwinmodbus_service", std::bind(&HiwinlibmodbusServiceServer::hiwinmodbus_execute, this,
                                        std::placeholders::_1, std::placeholders::_2));

    }

    private:
                
        int arm_state;
        int digital_state;
        std::vector<double> current_pos;
        std::vector<double> command;
        int command_type;
        int digital_output;
        // std::array<double, 6> command;
        // double command[6];
        rclcpp::Service<hiwin_interfaces::srv::RobotCommand>::SharedPtr server_;
        void hiwinmodbus_execute(const std::shared_ptr<hiwin_interfaces::srv::RobotCommand::Request> request,    
                  std::shared_ptr<hiwin_interfaces::srv::RobotCommand::Response>     response)  
        {
            // std::cout<<request->holding<<std::endl;
            if (request->cmd_mode == 1){
                hiwinlibmodbus.MOTOR_EXCITE();
            }
            else if (request->cmd_mode == 2){
                if(request->cmd_type==0){
                    command = {request->joints[0], request->joints[1], request->joints[2],
                    request->joints[3], request->joints[4], request->joints[5]};
                }
                else if(request->cmd_type==1){
                    command={request->pose.linear.x, request->pose.linear.y, request->pose.linear.z,
                    request->pose.angular.x, request->pose.angular.y, request->pose.angular.z};
                }

                hiwinlibmodbus.PTP(request->cmd_type, request->velocity, request->acceleration, request->tool, request->base, command);
            }
            else if (request->cmd_mode == 3){
                if(request->cmd_type==0){
                    command = {request->joints[0], request->joints[1], request->joints[2],
                    request->joints[3], request->joints[4], request->joints[5]};
                }
                else if(request->cmd_type==1){
                    command={request->pose.linear.x, request->pose.linear.y, request->pose.linear.z,
                    request->pose.angular.x, request->pose.angular.y, request->pose.angular.z};
                }                hiwinlibmodbus.LIN(request->cmd_type, request->velocity, request->acceleration, request->tool, request->base, command);
            }
            else if (request->cmd_mode == 4){
                hiwinlibmodbus.CIRC(request->velocity, request->acceleration, request->tool, request->base, request->circ_s, request->circ_end); 
            }
            /************* Discret_e Input *************/
              /*********************************
                    S0
              value:0 ~ 255 -> S0[1] ~ [256]
                0 or 1    -> R 
              ----------------------------------
                    DI
              value:300 ~ 555 -> DI[1] ~ [256]
                0 or 1      -> R
              **********************************/
         
              /************* Coil *************/
              /*********************************
                    DI
              value:0 ~ 255  -> DI[1] ~ [256]
                0 or 65280 -> R/W 
              ----------------------------------
                    DO
              value:300 ~ 555 -> DO[1] ~ [256]
                0 or 65280  -> R/W 
              **********************************/
            else if (request->cmd_mode == 5){
                digital_output =request->digital_output_pin+299;
                hiwinlibmodbus.DO(digital_output, request->digital_output_cmd);  
            }
        /*********************
          value   joint 
          0~5 -> A1~A6 
          value  Cartesian
          6~11 -> XYZABC 
        *********************/
            else if (request->cmd_mode == 6){
                hiwinlibmodbus.HOME();
            }
            else if (request->cmd_mode == 7){
                hiwinlibmodbus.JOG(request->jog_joint, request->jog_dir); 
            }
            else if (request->cmd_mode == 8){
                hiwinlibmodbus.getArmJoints(current_pos); 
                response->current_position = current_pos;
            }
            else if (request->cmd_mode == 9){
                hiwinlibmodbus.getArmPose(current_pos); 
                std::cout<<"----------------------------------"<<std::endl;
                response->current_position = current_pos;
            }
            else if (request->cmd_mode == 10){
                hiwinlibmodbus.Modbus_Close();
                rclcpp::shutdown();
            }
            else if (request->cmd_mode == 11){
                request->holding = true;
            }
            else if (request->cmd_mode == 12){
                hiwinlibmodbus.Read_DI(299+request->digital_input_pin, digital_state);
                response->digital_state = digital_state;
                request->holding == false;
            }
            else if (request->cmd_mode == 13){
                command={request->pose.linear.x, request->pose.linear.y, request->pose.linear.z,
                request->pose.angular.x, request->pose.angular.y, request->pose.angular.z};
                hiwinlibmodbus.SET_BASE(request->base_num, command);
                request->holding == false;
            }
            else if (request->cmd_mode == 14){
                command={request->pose.linear.x, request->pose.linear.y, request->pose.linear.z,
                request->pose.angular.x, request->pose.angular.y, request->pose.angular.z};
                hiwinlibmodbus.SET_TOOL(request->tool_num, command);
                request->holding == false;
            }
            else if (request->cmd_mode == 15){
                hiwinlibmodbus.Motion_Stop();
                request->holding == false;
            }
            if (request->holding == true){
                while(1){
                    hiwinlibmodbus.Arm_State_REGISTERS(arm_state); // return arm_state
                    // int arm_state = hiwinlibmodbus.Check_Arm_State();
                    if (arm_state == 1){
                        response->arm_state = arm_state;
                        break;
                    }

                }
            }
            else{
                hiwinlibmodbus.Arm_State_REGISTERS(arm_state); // return arm_state{
                response->arm_state = arm_state;
                }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response");
        }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  hiwinlibmodbus.libModbus_Connect("192.168.0.1");
  hiwinlibmodbus.Holding_Registers_init();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to recieve commands.");                 

  rclcpp::spin(std::make_shared<HiwinlibmodbusServiceServer>());
  // rclcpp::shutdown();
}
