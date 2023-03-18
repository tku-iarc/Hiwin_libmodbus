#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from hiwin_interfaces.srv import Hiwinmodbus
'''
    DO(int DO_Num, int x)                                                         # 1 -> on ; 0 -> off                                          
    HOME(int state)                                                               # 1 RUN
    PTP(int type, int vel, int acc, int TOOL, int BASE, double *Angle)            # 0 -> joint ; 1 -> coordinate
    LIN(int type,double *XYZ, int vel, int acc, int TOOL, int BASE)               # 0 -> joint ; 1 -> coordinate
    CIRC(double *CIRC_s, double *CIRC_end, int vel, int acc, int TOOL, int BASE) 
    JOG(int joint,int dir)
'''


class HiwinmodbusClient(Node):

    def __init__(self):
        super().__init__('hiwinmodbus_client')
        self.cli = self.create_client(Hiwinmodbus, 'hiwinmodbus_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.command_req = Hiwinmodbus.Request()

    def send_command_callback(self):
        self.future = self.cli.call_async(self.command_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def Holding_command(self, holding):

        self.command_req.holding = holding

    def call_MOTOR_EXCITE(self):

        self.command_req.mode  ='Excite'
        return self.send_command_callback() 

    def call_PTP(self, type, vel, acc, pose):

        self.command_req.mode  ='PTP'
        self.command_req.type  = type 
        self.command_req.vel   = vel
        self.command_req.acc   = acc
        self.command_req.pose = pose
        return self.send_command_callback()

    def call_LIN(self, type, vel, acc, pose):

        self.command_req.mode  ='LIN'
        self.command_req.type = type 
        self.command_req.vel  = vel
        self.command_req.acc  = acc
        self.command_req.pose  = pose
        return self.send_command_callback()


    def call_CIRC(self, vel, acc, circ_s, circ_end):

        self.command_req.mode  ='CIRC'
        self.command_req.vel      = vel
        self.command_req.acc      = acc
        self.command_req.circ_s   = circ_s
        self.command_req.circ_end = circ_end
        return self.send_command_callback()


    def call_DO(self, digital_output, onoff):

        self.command_req.mode  ='DO'
        self.command_req.digital_output = digital_output
        self.command_req.onoff          = onoff
        return self.send_command_callback()


    def call_HOME(self):
        self.command_req.mode  ='HOME'
        self.command_req.type = 0
        return self.send_command_callback()


    def call_JOG(self, joint, dir):

        self.command_req.mode  ='JOG'
        self.command_req.joint = joint
        self.command_req.dir   = dir
        return self.send_command_callback()

    def call_Modbus_Close(self):

        self.command_req.mode  ='Close'
        print("Modbus Close") 
        return self.send_command_callback()

def main(args=None):
    PTP_Angle = [0.00, 0.00, 0.00, 0.00, -90.00, 0.00]                 # ANGLE
    PTP_Angle2 = [20.00, 0.00, 0.00, 0.00, -90.00, 0.00]  
    PTP_Angle3 = [-20.00, 0.00, 0.00, 0.00, -90.00, 0.00]
    rclpy.init(args=args)

    hiwinmodbus_client = HiwinmodbusClient()

    hiwinmodbus_client.Holding_command(False)
    hiwinmodbus_client.call_MOTOR_EXCITE()
    
    # input()
    hiwinmodbus_client.Holding_command(False)
    hiwinmodbus_client.call_PTP(0,200,10,PTP_Angle)
    # input()
    hiwinmodbus_client.Holding_command(True)
    hiwinmodbus_client.call_PTP(0,200,10,PTP_Angle2)
    # hiwinmodbus_client.call_PTP(0,200,10,1,0,PTP_Angle3)
    # hiwinmodbus_client.call_PTP(0,200,10,1,0,PTP_Angle)
    input()
    hiwinmodbus_client.call_HOME()
    hiwinmodbus_client.call_Modbus_Close()


    hiwinmodbus_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
     

    

