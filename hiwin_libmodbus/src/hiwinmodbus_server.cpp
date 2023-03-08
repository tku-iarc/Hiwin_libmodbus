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

// For time delay
#include <chrono>
#include <thread>

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
        std::cout<<i<<std::endl;
        std::cout<<mode[i]<<std::endl;
        if (mode[i] == "Excite"){
            hiwinlibmodbus.Holding_Registers_init();
        }
        else if (mode[i] == "PTP"){
            std::vector<double> ptp_pose;
            for (int temp_count= i+5*i; temp_count<i+5*i+6; temp_count++){
                ptp_pose.push_back(ptp_command[temp_count]);
            }
            for (const auto& elem :  ptp_pose) {
                std::cout<<"=================="<<std::endl;
                std::cout<<elem<<std::endl;
                std::cout<<"=================="<<std::endl;
            }
            hiwinlibmodbus.PTP(type[i], vel[i], acc[i], tool[i], base[i], ptp_pose);
            ptp_pose.clear();
            // for (const auto& elem :  ptp_pose) {
            //     std::cout<<"-------------------"<<std::endl;
            //     std::cout<<elem<<std::endl;
            //     std::cout<<"-------------------"<<std::endl;
            // }
        }
        else if (mode[i] == "LIN"){
            std::vector<double> lin_pose;
            for (int temp_count= i+5*i; temp_count<i+5*i+6; temp_count++){
                lin_pose.push_back(lin_command[temp_count]);
            }
            hiwinlibmodbus.LIN(type[i], vel[i], acc[i], tool[i], base[i], lin_pose);
            lin_pose.clear();
        }
        else if (mode[i] == "CIRC"){
            std::vector<double> circ_s;
            std::vector<double> circ_end;
            for (int temp_count= i+5*i; temp_count<i+5*i+6; temp_count++){
                circ_s.push_back(circ_s_command[temp_count]);
                circ_end.push_back(circ_end_command[temp_count]);
            }
            hiwinlibmodbus.CIRC(vel[i], acc[i], tool[i], base[i], circ_s, circ_end);
            circ_s.clear();
            circ_end.clear();
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

    if (request->mode == "check"){
        hiwinmodbus_execute();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // sleep for 500 milliseconds
        while (1){
            int state =hiwinlibmodbus.Arm_State_REGISTERS();
            std::cout<<state<<std::endl;
            if (state == -1){
                commands_size = 0;
                mode.clear();
                type.clear();
                vel.clear();
                acc.clear();
                tool.clear();
                base.clear();
                digital_output.clear();
                onoff.clear();
                ptp_command.clear();
                lin_command.clear();
                circ_s_command.clear();
                circ_end_command.clear();
                joint.clear();
                dir.clear();
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
            std::cout<<"----------------------------------"<<std::endl;
            type.push_back(0);
            vel.push_back(0);
            acc.push_back(0);
            tool.push_back(0);
            base.push_back(0);
            digital_output.push_back(0);
            onoff.push_back(0);
            for (int count = 0; count < 6; count++){
                ptp_command.push_back(0.00);
                lin_command.push_back(0.00);
                circ_s_command.push_back(0.00);
                circ_end_command.push_back(0.00);
            }
            joint.push_back(0);
            dir.push_back(0);
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
            for (int count = 0; count < 6; count++){
                lin_command.push_back(0.00);
                circ_s_command.push_back(0.00);
                circ_end_command.push_back(0.00);
            }
            joint.push_back(0);
            dir.push_back(0);  
        }
        else if (request->mode == "LIN"){
            type.push_back(request->type);
            vel.push_back(request->vel);
            acc.push_back(request->acc);
            tool.push_back(request->tool);
            base.push_back(request->base);
            digital_output.push_back(0);
            onoff.push_back(0);
            for (const auto& pose : request->lin_pose) {
                lin_command.push_back(pose);
            }
            for (int count = 0; count < 6; count++){
                ptp_command.push_back(0.00);
                circ_s_command.push_back(0.00);
                circ_end_command.push_back(0.00);
            }
            joint.push_back(0);
            dir.push_back(0);
        }
        else if (request->mode == "CIRC"){
            type.push_back(0);
            vel.push_back(request->vel);
            acc.push_back(request->acc);
            tool.push_back(request->tool);
            base.push_back(request->base);
            digital_output.push_back(0);
            onoff.push_back(0);
            for (int count = 0; count < 6; count++){
                ptp_command.push_back(0.00);
                lin_command.push_back(0.00);
            }
            for (const auto& pose : request->circ_s) {
                circ_s_command.push_back(pose);
            }
            for (const auto& pose : request->circ_end) {
                circ_end_command.push_back(pose);
            }
            joint.push_back(0);
            dir.push_back(0);
        }
        else if (request->mode == "DO"){
            type.push_back(0);
            vel.push_back(0);
            acc.push_back(0);
            tool.push_back(0);
            base.push_back(0);
            digital_output.push_back(request->digital_output);
            onoff.push_back(request->onoff);
            for (int count = 0; count < 6; count++){
                ptp_command.push_back(0.00);
                lin_command.push_back(0.00);
                circ_s_command.push_back(0.00);
                circ_end_command.push_back(0.00);
            }
            joint.push_back(0);
            dir.push_back(0);
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
            for (int count = 0; count < 6; count++){
                ptp_command.push_back(0.00);
                lin_command.push_back(0.00);
                circ_s_command.push_back(0.00);
                circ_end_command.push_back(0.00);
            }
            joint.push_back(request->joint);
            dir.push_back(request->dir);  
        }
        else if (request->mode == "HOME"){
            type.push_back(0);
            vel.push_back(0);
            acc.push_back(0);
            tool.push_back(0);
            base.push_back(0);
            digital_output.push_back(0);
            onoff.push_back(0);
            for (int count = 0; count < 6; count++){
                ptp_command.push_back(0.00);
                lin_command.push_back(0.00);
                circ_s_command.push_back(0.00);
                circ_end_command.push_back(0.00);
            }
            joint.push_back(0);
            dir.push_back(0);
            hiwinlibmodbus.HOME();
        }
    }
                                    
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "execurting mode\na: %s",  request->mode);                                         
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  hiwinlibmodbus.libModbus_Connect("192.168.0.1");
  hiwinlibmodbus.Holding_Registers_init();
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hiwinmodbus_service");   // CHANGE

  rclcpp::Service<hiwin_interfaces::srv::Hiwinmodbus>::SharedPtr service =               // CHANGE
    node->create_service<hiwin_interfaces::srv::Hiwinmodbus>("hiwinmodbus_service",  &commands_reserve);   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to recieve commands.");                     // CHANGE

  rclcpp::spin(node);
  // rclcpp::shutdown();
}
