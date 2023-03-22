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
class HiwinmodbusServiceServer : public rclcpp::Node
{
    
    public:
    HiwinmodbusServiceServer() : Node("hiwinmodbus_server")
    {
        server_ = create_service<hiwin_interfaces::srv::Hiwinmodbus>(
                "hiwinmodbus_service", std::bind(&HiwinmodbusServiceServer::hiwinmodbus_execute, this,
                                        std::placeholders::_1, std::placeholders::_2));

    }

    private:
                
        int tool = 1;
        int base = 0;
        int arm_state;
        rclcpp::Service<hiwin_interfaces::srv::Hiwinmodbus>::SharedPtr server_;
        void hiwinmodbus_execute(const std::shared_ptr<hiwin_interfaces::srv::Hiwinmodbus::Request> request,    
                  std::shared_ptr<hiwin_interfaces::srv::Hiwinmodbus::Response>     response)  
        {
        
            if (request->mode == "Excite"){
                hiwinlibmodbus.MOTOR_EXCITE();
            }
            else if (request->mode == "PTP"){
                hiwinlibmodbus.PTP(request->type, request->vel, request->acc, tool, base, request->pose);
            }
            else if (request->mode == "LIN"){
                hiwinlibmodbus.LIN(request->type, request->vel, request->acc, tool, base, request->pose);
            }
            else if (request->mode == "CIRC"){
                hiwinlibmodbus.CIRC(request->vel, request->acc, tool, base, request->circ_s, request->circ_end); 
            }
            else if (request->mode == "DO"){
                hiwinlibmodbus.DO(request->digital_output, request->onoff);  
            }
        /*********************
          value   joint 
          0~5 -> A1~A6 
          value  Cartesian
          6~11 -> XYZABC 
        *********************/
            else if (request->mode == "JOG"){
                hiwinlibmodbus.JOG(request->joint, request->dir); 
            }
            else if (request->mode == "HOME"&& rclcpp::ok()){
                hiwinlibmodbus.HOME();
            }
            else if (request->mode == "Close"){
                hiwinlibmodbus.Modbus_Close();
                rclcpp::shutdown();
            }

            if (request->holding == true){
                while(1){
                    std::cout<<"----------------------------------"<<std::endl;
                    hiwinlibmodbus.Arm_State_REGISTERS(arm_state); // return arm_state
                    // int arm_state = hiwinlibmodbus.Check_Arm_State();
                    if (arm_state == 1){
                        response->arm_state = arm_state;
                        break;
                    }

                }
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

  rclcpp::spin(std::make_shared<HiwinmodbusServiceServer>());
  // rclcpp::shutdown();
}
