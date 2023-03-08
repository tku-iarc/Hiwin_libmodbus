#include <iostream>
#include <functional>
#include <memory>
#include <thread>

#include "hiwin_interfaces/srv/hiwinmodbus.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"
#include "hiwin_libmodbus/hiwin_libmodbus.hpp"

#include "hiwin_libmodbus//visibility_control.h"

#include <memory>

HiwinLibmodbus hiwinlibmodbus;
int commands_size;
std::vector<std::string> mode;
std::vector<int> type;
std::vector<int> vel;
std::vector<int> acc;
std::vector<int> tool;
std::vector<int> base;
std::vector<int> digital_output;
std::vector<int> onoff;
std::vector<double> ptp_command;
std::vector<double> lin_command;
std::vector<double> circ_s_command;
std::vector<double> circ_end_command;
std::vector<int> joint;
std::vector<int> dir;
void hiwinmodbus_execute(){
    for (int i=0; i<commands_size; i++){
        std::cout<<mode[i]<<std::endl;
        if (mode[i] == "Excite"){
            hiwinlibmodbus.Holding_Registers_init();
        }
        else if (mode[i] == "PTP"){
            std::vector<double> ptp_pose;
            for (int temp_count= i; temp_count<i+6; temp_count++){
                ptp_pose.push_back(ptp_command[temp_count]);
            }
            hiwinlibmodbus.PTP(type[i], vel[i], acc[i], tool[i], base[i], ptp_pose);
        }
        else if (mode[i] == "LIN"){
            std::vector<double> lin_pose;
            for (int temp_count= i; temp_count<i+6; temp_count++){
                lin_pose.push_back(lin_command[temp_count]);
            }
            hiwinlibmodbus.LIN(type[i], vel[i], acc[i], tool[i], base[i], lin_pose);
        }
        else if (mode[i] == "CIRC"){
            std::vector<double> circ_s;
            std::vector<double> circ_end;
            for (int temp_count= i; temp_count<i+6; temp_count++){
                circ_s.push_back(circ_s_command[temp_count]);
                circ_end.push_back(circ_end_command[temp_count]);
            }
            hiwinlibmodbus.CIRC(vel[i], acc[i], tool[i], base[i], circ_s, circ_end);
        }
        else if (mode[i] == "DO"){
            hiwinlibmodbus.DO(digital_output[i], onoff[i]);
        }
        else if (mode[i] == "JOG"){
            hiwinlibmodbus.JOG(joint[i], dir[i]); 
        }
        else if (mode[i] == "HOME"){
            hiwinlibmodbus.HOME();
        }


    }
}
void commands_reserve(const std::shared_ptr<hiwin_interfaces::srv::Hiwinmodbus::Request> request,    
          std::shared_ptr<hiwin_interfaces::srv::Hiwinmodbus::Response>     response)  
{
    // std::cout<<"----------------------------------"<<std::endl;
    // std::vector<double> a = request->ptp_pose;
    // std::cout<<a<<std::endl;

    // if (request->mode == "Connect" && rclcpp::ok()) {
    //     if(hiwinlibmodbus.libModbus_Connect(request->ip_address)&& rclcpp::ok()){
    //       std::cout<<"----------------------------------"<<std::endl;
    //       hiwinlibmodbus.Holding_Registers_init();
    //     }
    //   }
    if (request->mode == "check"){
        hiwinmodbus_execute();
        while (1){
            int state =hiwinlibmodbus.Arm_State_REGISTERS();
            std::cout<<state<<std::endl;
            if (state == 1){
                response->arm_state = state;
                break;
            }
        }  
    }
    else if (request->mode == "Close"){
        hiwinlibmodbus.Modbus_Close();
        rclcpp::shutdown();
    }
    else{
        mode.push_back(request->mode);
        commands_size += 1;
        if (request->mode == "Excite"){
            type.push_back(0);
            vel.push_back(0);
            acc.push_back(0);
            tool.push_back(0);
            base.push_back(0);
            digital_output.push_back(0);
            onoff.push_back(0);
            ptp_command.push_back(0.00);
            lin_command.push_back(0.00);
            circ_s_command.push_back(0.00);
            circ_end_command.push_back(0.00);
            joint.push_back(0);
            dir.push_back(0);
            // hiwinlibmodbus.Holding_Registers_init();
            // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();  
        }
        else if (request->mode == "PTP"){
            type.push_back(request->type);
            vel.push_back(request->vel);
            acc.push_back(request->acc);
            tool.push_back(request->tool);
            base.push_back(request->base);
            digital_output.push_back(0);
            onoff.push_back(0);
            for (const auto& pose :  request->ptp_pose) {
                ptp_command.push_back(pose);
            }
            lin_command.push_back(0.00);
            circ_s_command.push_back(0.00);
            circ_end_command.push_back(0.00);
            joint.push_back(0);
            dir.push_back(0);
            // hiwinlibmodbus.PTP(request->type, request->vel, request->acc, request->tool, request->base, request->angle);
            // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();  
        }
        else if (request->mode == "LIN"){
            type.push_back(request->type);
            vel.push_back(request->vel);
            acc.push_back(request->acc);
            tool.push_back(request->tool);
            base.push_back(request->base);
            digital_output.push_back(0);
            onoff.push_back(0);
            ptp_command.push_back(0.00);
            for (const auto& pose : request->lin_pose) {
                lin_command.push_back(pose);
            }
            circ_s_command.push_back(0.00);
            circ_end_command.push_back(0.00);
            joint.push_back(0);
            dir.push_back(0);
            // hiwinlibmodbus.LIN(request->type, request->vel, request->acc, request->tool, request->base, request->xyz);
            // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();    
        }
        else if (request->mode == "CIRC"){
            type.push_back(0);
            vel.push_back(request->vel);
            acc.push_back(request->acc);
            tool.push_back(request->tool);
            base.push_back(request->base);
            digital_output.push_back(0);
            onoff.push_back(0);
            ptp_command.push_back(0.00);
            lin_command.push_back(0.00);
            for (const auto& pose : request->circ_s) {
                circ_s_command.push_back(pose);
            }
            for (const auto& pose : request->circ_end) {
                circ_end_command.push_back(pose);
            }
            joint.push_back(0);
            dir.push_back(0);
            // hiwinlibmodbus.CIRC(request->vel, request->acc, request->tool, request->base, request->circ_s, request->circ_end); 
            // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();   
        }
        else if (request->mode == "DO"){
            type.push_back(0);
            vel.push_back(0);
            acc.push_back(0);
            tool.push_back(0);
            base.push_back(0);
            digital_output.push_back(request->digital_output);
            onoff.push_back(request->onoff);
            ptp_command.push_back(0.00);
            lin_command.push_back(0.00);
            circ_s_command.push_back(0.00);
            circ_end_command.push_back(0.00);
            joint.push_back(0);
            dir.push_back(0);
            // hiwinlibmodbus.DO(request->digital_output, request->onoff);  
            // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();  
        }
        /*********************
          value   joint 
          0~5 -> A1~A6 
          value  Cartesian
          6~11 -> XYZABC 
        *********************/
        else if (request->mode == "JOG"){
            type.push_back(0);
            vel.push_back(0);
            acc.push_back(0);
            tool.push_back(0);
            base.push_back(0);
            digital_output.push_back(0);
            onoff.push_back(0);
            ptp_command.push_back(0.00);
            lin_command.push_back(0.00);
            circ_s_command.push_back(0.00);
            circ_end_command.push_back(0.00);
            joint.push_back(request->joint);
            dir.push_back(request->dir);
            // hiwinlibmodbus.JOG(request->joint, request->dir); 
            // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();   
        }
        else if (request->mode == "HOME"){
            type.push_back(0);
            vel.push_back(0);
            acc.push_back(0);
            tool.push_back(0);
            base.push_back(0);
            digital_output.push_back(0);
            onoff.push_back(0);
            ptp_command.push_back(0.00);
            lin_command.push_back(0.00);
            circ_s_command.push_back(0.00);
            circ_end_command.push_back(0.00);
            joint.push_back(0);
            dir.push_back(0);
            hiwinlibmodbus.HOME();
            // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();
        }
    }

    // if (request->mode == "Excite"){
    //     commands_size += 1;
    //     hiwinlibmodbus.Holding_Registers_init();
    //     // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();  
    // }
    // else if (request->mode == "PTP"){
    //     commands_size += 1;
    //     hiwinlibmodbus.PTP(request->type, request->vel, request->acc, request->tool, request->base, request->angle);
    //     // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();  
    // }
    // else if (request->mode == "LIN"){
    //     commands_size += 1;
    //     hiwinlibmodbus.LIN(request->type, request->vel, request->acc, request->tool, request->base, request->xyz);
    //     // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();    
    // }
    // else if (request->mode == "CIRC"){
    //     commands_size += 1;
    //     hiwinlibmodbus.CIRC(request->vel, request->acc, request->tool, request->base, request->circ_s, request->circ_end); 
    //     // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();   
    // }
    // else if (request->mode == "DO"){
    //     commands_size += 1;
    //     hiwinlibmodbus.DO(request->digital_output, request->onoff);  
    //     // response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();  
    // }
    // /*********************
    //   value   joint 
    //   0~5 -> A1~A6 
    //   value  Cartesian
    //   6~11 -> XYZABC 
    // *********************/
    // else if (request->mode == "JOG"){
    //     commands_size += 1;
    //     hiwinlibmodbus.JOG(request->joint, request->dir); 
    //     response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();   
    // }
    // else if (request->mode == "HOME"){
    //     commands_size += 1;
    //     hiwinlibmodbus.HOME();
    //     response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();
    // }
    // else if (request->mode == "Close"){
    //     commands_size += 1;
    //     hiwinlibmodbus.Modbus_Close();
    //     rclcpp::shutdown();
    // }
                                    
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "execurting mode\na: %s",  request->mode);                                         
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
//   hiwinlibmodbus.libModbus_Connect("192.168.0.1");
//   hiwinlibmodbus.Holding_Registers_init();
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hiwinmodbus_service");   // CHANGE

  rclcpp::Service<hiwin_interfaces::srv::Hiwinmodbus>::SharedPtr service =               // CHANGE
    node->create_service<hiwin_interfaces::srv::Hiwinmodbus>("hiwinmodbus_service",  &commands_reserve);   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to recieve commands.");                     // CHANGE

  rclcpp::spin(node);
  // rclcpp::shutdown();
}
