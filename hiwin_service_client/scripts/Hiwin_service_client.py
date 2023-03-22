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
    
    # def Holding_command(self, holding):

    #     self.command_req.holding = holding

    def call_motor_excite(self, holding=False):

        self.command_req.mode  ='Excite'
        self.command_req.holding = holding
        return self.send_command_callback() 

    def call_ptp(self, type, vel, acc, pose, holding=False):
        
        self.command_req.mode  ='PTP'
        self.command_req.type  = type 
        self.command_req.vel   = vel
        self.command_req.acc   = acc
        self.command_req.pose = pose
        self.command_req.holding = holding
        return self.send_command_callback()

    def call_lin(self, type, vel, acc, pose, holding=False):

        self.command_req.mode  ='LIN'
        self.command_req.type = type 
        self.command_req.vel  = vel
        self.command_req.acc  = acc
        self.command_req.pose  = pose
        self.command_req.holding = holding
        return self.send_command_callback()


    def call_circ(self, vel, acc, circ_s, circ_end, holding=False):

        self.command_req.mode  ='CIRC'
        self.command_req.vel      = vel
        self.command_req.acc      = acc
        self.command_req.circ_s   = circ_s
        self.command_req.circ_end = circ_end
        self.command_req.holding = holding
        return self.send_command_callback()


    def call_do(self, digital_output, onoff, holding=False):

        self.command_req.mode  ='DO'
        self.command_req.digital_output = digital_output
        self.command_req.onoff          = onoff
        self.command_req.holding = holding
        return self.send_command_callback()


    def call_home(self, holding=False):
        self.command_req.mode  ='HOME'
        self.command_req.type = 0
        self.command_req.holding = holding
        return self.send_command_callback()


    def call_jog(self, joint, dir, holding=False):

        self.command_req.mode  ='JOG'
        self.command_req.joint = joint
        self.command_req.dir   = dir
        self.command_req.holding = holding
        return self.send_command_callback()

    def call_modbus_close(self):

        self.command_req.mode  ='Close'
        print("Modbus Close") 
        return self.send_command_callback()

def main(args=None):
    PTP_Angle = [0.00, 0.00, 0.00, 0.00, -90.00, 0.00]                 # ANGLE
    PTP_Angle2 = [20.00, 0.00, 0.00, 0.00, -90.00, 0.00]  
    PTP_Angle3 = [-20.00, 0.00, 0.00, 0.00, -90.00, 0.00]
    OBJECT_POSE = [-67.517, 361.753, 293.500, 180.00, 0.00, 100.572]
    rclpy.init(args=args)

    hiwinmodbus_client = HiwinmodbusClient()

    hiwinmodbus_client.call_motor_excite()
    
    hiwinmodbus_client.call_ptp(0,200,10,PTP_Angle)
    hiwinmodbus_client.call_ptp(0,200,10,PTP_Angle3)
    hiwinmodbus_client.call_ptp(0,200,10,PTP_Angle)
    hiwinmodbus_client.call_ptp(0,200,10,PTP_Angle2, holding= True)
    hiwinmodbus_client.call_lin(1,200,10,OBJECT_POSE, holding= True)

    hiwinmodbus_client.call_ptp(0,200,10,PTP_Angle3)
    hiwinmodbus_client.call_ptp(0,200,10,PTP_Angle, holding= True)
    # hiwinmodbus_client.call_HOME()
    hiwinmodbus_client.call_modbus_close()


    hiwinmodbus_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
     

    

