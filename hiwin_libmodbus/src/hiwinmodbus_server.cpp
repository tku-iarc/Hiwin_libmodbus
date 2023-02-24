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
void hiwinmodbus_execute(const std::shared_ptr<hiwin_interfaces::srv::Hiwinmodbus::Request> request,    
          std::shared_ptr<hiwin_interfaces::srv::Hiwinmodbus::Response>     response)  
{
    if (request->mode == "connect" && rclcpp::ok()) {
        if(hiwinlibmodbus.libModbus_Connect(request->ip_address)&& rclcpp::ok()){
          std::cout<<"----------------------------------"<<std::endl;
          hiwinlibmodbus.Holding_Registers_init();
        }
      }

    else if (request->mode == "PTP"){
        hiwinlibmodbus.PTP(request->type, request->vel, request->acc, request->tool, request->base, request->angle);
        std::cout<<"afjioaljfl;kjaklfaf;"<<std::endl;  
        response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();  
    }
    else if (request->mode == "LIN"){
        hiwinlibmodbus.LIN(request->type, request->vel, request->acc, request->tool, request->base, request->xyz);
        response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();    
    }
    else if (request->mode == "CIRC"){
        hiwinlibmodbus.CIRC(request->vel, request->acc, request->tool, request->base, request->circ_s, request->circ_end); 
        response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();   
    }
    else if (request->mode == "DO"){
        hiwinlibmodbus.DO(request->digital_output, request->onoff);  
        response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();  
    }
/*********************
  value   joint 
  0~5 -> A1~A6 
  value  Cartesian
  6~11 -> XYZABC 
*********************/
    else if (request->mode == "JOG"){
        hiwinlibmodbus.JOG(request->joint, request->dir); 
        response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();   
    }
    else if (request->mode == "HOME"&& rclcpp::ok()){
        hiwinlibmodbus.HOME();
        response->arm_state =hiwinlibmodbus.Arm_State_REGISTERS();
    }
    else if (request->mode == "close"){
        hiwinlibmodbus.Modbus_Close();
    }
                                    
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "execurting mode\na: %s",  request->mode);                                         
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hiwinmodbus_service");   // CHANGE

  rclcpp::Service<hiwin_interfaces::srv::Hiwinmodbus>::SharedPtr service =               // CHANGE
    node->create_service<hiwin_interfaces::srv::Hiwinmodbus>("hiwinmodbus_service",  &hiwinmodbus_execute);   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");                     // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}
