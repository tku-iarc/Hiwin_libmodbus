#!/usr/bin/env python3


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from hiwin_action_interfaces.action import Hiwinmodbus
'''
    DO(int DO_Num, int x)                                                         # 1 -> on ; 0 -> off                                          
    HOME(int state)                                                               # 1 RUN
    PTP(int type, int vel, int acc, int TOOL, int BASE, double *Angle)            # 0 -> joint ; 1 -> coordinate
    LIN(int type,double *XYZ, int vel, int acc, int TOOL, int BASE)               # 0 -> joint ; 1 -> coordinate
    CIRC(double *CIRC_s, double *CIRC_end, int vel, int acc, int TOOL, int BASE) 
    JOG(int joint,int dir)
'''


class HiwinmodbusActionClient(Node):

    def __init__(self):
        super().__init__('hiwinmodbus_action_client')
        self._action_client = ActionClient(self, Hiwinmodbus, 'Hiwinmodbus')
        self.command_msg = Hiwinmodbus.Goal()
        self.command_msg.ip_address = '192.168.0.1'

    def send_command(self):
        
        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(self.command_msg)
    
    def call_Connect(self):

        self.command_msg.mode  ='connect'
        return self.send_command()    

    def call_MOTOR_EXCITE(self):

        self.command_msg.mode  ='MOTOR_EXCITE'
        return self.send_command()    


    def call_PTP(self, type, vel, acc, tool, base, angle):

        self.command_msg.mode  ='PTP'
        self.command_msg.type  = type 
        self.command_msg.vel   = vel
        self.command_msg.acc   = acc
        self.command_msg.tool  = tool
        self.command_msg.base  = base
        self.command_msg.angle = angle
        return self.send_command()

    def call_LIN(self, type, vel, acc, tool, base, xyz):

        self.command_msg.mode  ='LIN'
        self.command_msg.type = type 
        self.command_msg.vel  = vel
        self.command_msg.acc  = acc
        self.command_msg.tool = tool
        self.command_msg.base = base
        self.command_msg.xyz  = xyz
        return self.send_command()


    def call_CIRC(self, vel, acc, tool, base, circ_s, circ_end):

        self.command_msg.mode  ='CIRC'
        self.command_msg.vel      = vel
        self.command_msg.acc      = acc
        self.command_msg.tool     = tool
        self.command_msg.base     = base
        self.command_msg.circ_s   = circ_s
        self.command_msg.circ_end = circ_end
        return self.send_command()


    def call_DO(self, digital_output, onoff):

        self.command_msg.mode  ='DO'
        self.command_msg.digital_output = digital_output
        self.command_msg.onoff          = onoff
        return self.send_command()


    def call_HOME(self):
        self.command_msg.mode  ='HOME'
        self.command_msg.type = 0
        return self.send_command()


    def call_JOG(self, joint, dir):

        self.command_msg.mode  ='JOG'
        self.command_msg.joint = joint
        self.command_msg.dir   = dir
        return self.send_command()

    def call_Modbus_Close(self):

        self.command_msg.mode  ='close'
        print("Modbus Close") 
        return self.send_command()

def main(args=None):
    rclpy.init(args=args)

    action_client = HiwinmodbusActionClient()

    
    action_client.call_Connect()
    input()
    # action_client.call_MOTOR_EXCITE()
    # input()
    action_client.call_HOME()
    input()
    action_client.call_Modbus_Close()

    
    # rclpy.spin(action_client)
    

    # future = action_client.call_HOME()

    # rclpy.spin_until_future_complete(action_client, future)

if __name__ == "__main__":
    main()
     

    

