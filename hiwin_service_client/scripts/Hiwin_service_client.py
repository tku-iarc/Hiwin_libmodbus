#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.task import Future
import time
from threading import Thread

from hiwin_interfaces.srv import RobotCommand
from geometry_msgs.msg import Twist
'''
    DO(int DO_Num, int x)                                                         # 1 -> on ; 0 -> off                                          
    HOME(int state)                                                               # 1 RUN
    PTP(int type, int vel, int acc, int TOOL, int BASE, double *Angle)            # 0 -> joint ; 1 -> coordinate
    LIN(int type,double *XYZ, int vel, int acc, int TOOL, int BASE)               # 0 -> joint ; 1 -> coordinate
    CIRC(double *CIRC_s, double *CIRC_end, int vel, int acc, int TOOL, int BASE) 
    JOG(int joint,int dir)
'''

DEFAULT_VELOCITY = 10
DEFAULT_ACCELERATION = 10

VACUUM_PIN = 3

PHOTO_POSE = [0.00, 0.00, 0.00, 0.00, -90.00, 0.00]
# OBJECT_POSE = [20.00, 0.00, 0.00, 0.00, -90.00, 0.00]
OBJECT_POSE = [-67.517, 361.753, 293.500, 180.00, 0.00, 100.572]
PLACE_POSE = [-20.00, 0.00, 0.00, 0.00, -90.00, 0.00]

# only for example as we don't use yolo here
# assume NUM_OBJECTS=5, then this process will loop 5 times
NUM_OBJECTS = 5

class HiwinmodbusClient(Node):

    def __init__(self):
        super().__init__('hiwinmodbus_client')
        self.hiwin_client = self.create_client(RobotCommand, 'hiwinmodbus_service')

    def _wait_for_future_done(self, future: Future, timeout=-1):
        time_start = time.time()
        while not future.done():
            time.sleep(0.01)
            if timeout > 0 and time.time() - time_start > timeout:
                self.get_logger().error('Wait for service timeout!')
                return False
        print("+++++++++++++++++=")
        return True

    def call_hiwin(self, request):
        while not self.hiwin_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.hiwin_client.call_async(request)
        # if self._wait_for_future_done(future):
        #     res = future.result()
        #     print(res)
        # else:
        #     res = None
        res = future.result().arm_state
        return res
        # self.future = self.hiwin_client.call_async(request)
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()

    def generate_robot_request(
            self, 
            holding=True,
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.POSE_CMD,
            velocity=DEFAULT_VELOCITY,
            acceleration=DEFAULT_ACCELERATION,
            digital_output_pin=0,
            digital_output_cmd=RobotCommand.Request.DIGITAL_OFF,
            poses=Twist(),
            joints=[float('inf')]*6,
            circ_s=[],
            circ_end=[],
            jog_joint=6,
            jog_dir=0
            ):
        request = RobotCommand.Request()
        request.digital_output_pin = digital_output_pin
        request.digital_output_cmd = digital_output_cmd
        request.acceleration = acceleration
        request.jog_joint = jog_joint
        request.velocity = velocity
        request.cmd_mode = cmd_mode
        request.cmd_type = cmd_type
        request.circ_end = circ_end
        request.jog_dir = jog_dir
        request.holding = holding
        request.joints = joints
        request.circ_s = circ_s
        request.pose = poses

        return request

    def call_motor_excite(self, holding=False):

        req = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.EXCITE,
            holding=holding
        )
        return self.call_hiwin(req) 

    def call_ptp(self, pose, cmd_type=RobotCommand.Request.POSE_CMD, 
    holding=False,velocity=DEFAULT_VELOCITY, acceleration=DEFAULT_ACCELERATION):
        
        # print(pose)
        # print(cmd_type)
        # print(holding)
        # print(velocity)
        # print(acceleration)
        if cmd_type == 0:
            print("-------------")
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=cmd_type,
                velocity=velocity,
                acceleration=acceleration, 
                joints=pose,
                holding=holding
            )
        if cmd_type == 1:
            poses = Twist()
            [poses.linear.x, poses.linear.y, poses.linear.z] = pose[0:3]
            [poses.angular.x, poses.angular.y, poses.angular.z] = pose[3:6]
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=cmd_type,
                velocity=velocity,
                acceleration=acceleration, 
                poses=poses,
                holding=holding
            )
        # print(req)
        return self.call_hiwin(req) 

    def call_line(self, pose, cmd_type=RobotCommand.Request.POSE_CMD, 
    holding=False,  velocity=DEFAULT_VELOCITY, acceleration=DEFAULT_ACCELERATION):

        if cmd_type == 0:
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=cmd_type,
                velocity=velocity,
                acceleration=acceleration, 
                joints=pose,
                holding=holding
            )
        if cmd_type == 1:
            poses = Twist()
            [poses.linear.x, poses.linear.y, poses.linear.z] = pose[0:3]
            [poses.angular.x, poses.angular.y, poses.angular.z] = pose[3:6]
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=cmd_type,
                velocity=velocity,
                acceleration=acceleration, 
                poses=poses,
                holding=holding
            )
        return self.call_hiwin(req) 


    def call_circ(self, circ_s, circ_end,holding=False, 
    velocity=DEFAULT_VELOCITY, acceleration=DEFAULT_ACCELERATION):

        req = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.CIRC,
            velocity=velocity,
            acceleration=acceleration, 
            circ_s=circ_s,
            circ_end=circ_end,
            holding=holding
        )
        return self.call_hiwin(req)


    def call_do(self, digital_output_pin, digital_output_cmd=RobotCommand.Request.DIGITAL_OFF, holding=False):

        req = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.DIGITAL_OUTPUT,
            digital_output_cmd=digital_output_cmd,
            digital_output_pin=digital_output_pin,
            holding=holding
        )
        return self.call_hiwin(req)



    def call_home(self, holding=True):
        req = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.HOME,
            holding=holding
        )
        return self.call_hiwin(req)


    def call_jog(self, jog_joint=6, jog_dir=0, holding=False):

        req = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.JOG,
            jog_joint=jog_joint,
            jog_dir=jog_dir,
            holding=holding
        )
        return self.call_hiwin(req)

    def call_modbus_close(self):

        req = self.generate_robot_request(
            cmd_mode=RobotCommand.Request.CLOSE,
        )
        return self.call_hiwin(req)

    # def start_main_loop_thread(self):
    #     self.main_loop_thread = Thread(target=self._main_loop)
    #     self.main_loop_thread.daemon = True
    #     self.main_loop_thread.start()

def main(args=None):
    PTP_Angle = [0.00, 0.00, 0.00, 0.00, -90.00, 0.00]                 # ANGLE
    PTP_Angle2 = [20.00, 0.00, 0.00, 0.00, -90.00, 0.00]  
    PTP_Angle3 = [-20.00, 0.00, 0.00, 0.00, -90.00, 0.00]
    OBJECT_POSE = [-67.517, 361.753, 293.500, 180.00, 0.00, 100.572]
    rclpy.init(args=args)

    hiwinmodbus_client = HiwinmodbusClient()
    # hiwinmodbus_client.start_main_loop_thread()

    # hiwinmodbus_client.call_motor_excite()
    
    res =hiwinmodbus_client.call_ptp(PTP_Angle, RobotCommand.Request.JOINTS_CMD)
    print("pppppppp")
    print(res)
    hiwinmodbus_client.call_ptp(PTP_Angle3, RobotCommand.Request.JOINTS_CMD)
    hiwinmodbus_client.call_ptp(PTP_Angle, RobotCommand.Request.JOINTS_CMD)
    hiwinmodbus_client.call_ptp(PTP_Angle2, RobotCommand.Request.JOINTS_CMD, True)
    hiwinmodbus_client.call_ptp(OBJECT_POSE, RobotCommand.Request.POSE_CMD, True)

    # hiwinmodbus_client.call_ptp(0,200,10,PTP_Angle3)
    # hiwinmodbus_client.call_ptp(0,200,10,PTP_Angle, holding= True)
    # hiwinmodbus_client.call_HOME()
    hiwinmodbus_client.call_modbus_close()


    hiwinmodbus_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
     

    

