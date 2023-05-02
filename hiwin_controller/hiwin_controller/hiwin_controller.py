#!/usr/bin/env python3
import time
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from rclpy.task import Future
from typing import NamedTuple
from geometry_msgs.msg import Twist

from hiwin_interfaces.srv import RobotCommand
# from YoloDetector import YoloDetectorActionClient

DEFAULT_VELOCITY = 20
DEFAULT_ACCELERATION = 20

VACUUM_PIN = 3

PHOTO_POSE = [0.00, 0.00, 0.00, 0.00, -90.00, 0.00]
# OBJECT_POSE = [20.00, 0.00, 0.00, 0.00, -90.00, 0.00]
OBJECT_POSE = [-67.517, 361.753, 293.500, 180.00, 0.00, 100.572]
PLACE_POSE = [-20.00, 0.00, 0.00, 0.00, -90.00, 0.00]

# only for example as we don't use yolo here
# assume NUM_OBJECTS=5, then this process will loop 5 times
NUM_OBJECTS = 5



class HIWIN_Controller():

    def __init__(self, node):
        self.hiwin_client = node.create_client(RobotCommand, 'hiwinmodbus_service')
        self.object_pose = None
        self.object_cnt = 0
        self.node = node
        self.logger = node.get_logger()

    def set_speedANDacceleration(speed, acceleration):
        DEFAULT_VELOCITY = speed
        DEFAULT_ACCELERATION = acceleration

    def JOINT_move(self):
        self.logger.info('JOINT_move')
        req = self.generate_robot_request(
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                holding=True,
                joints=PHOTO_POSE
                )
        res = self.call_hiwin(req)
        # self.logger().info(res)
    
    def PTP_move(self):
        self.logger.info('PTP_move')
        req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                holding=True,
                velocity=DEFAULT_VELOCITY,
                joints=PLACE_POSE
                )
        res = self.call_hiwin(req)
    
    def LINE_move(self):
        req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                joints=PHOTO_POSE
                )
        res = self.call_hiwin(req)
    
    def Arm_close():
        self.logger.info('CLOSE_ROBOT')
        req = self.generate_robot_request(cmd_mode=RobotCommand.Request.CLOSE)
        res = self.call_hiwin(req)


    def _wait_for_future_done(self, future: Future, timeout=-1):
        time_start = time.time()
        while not future.done():
            time.sleep(0.01)
            if timeout > 0 and time.time() - time_start > timeout:
                self.logger.error('Wait for service timeout!')
                return False
        return True
    
    def generate_robot_request(
            self, 
            holding=True,
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.POSE_CMD,
            velocity=DEFAULT_VELOCITY,
            acceleration=DEFAULT_ACCELERATION,
            digital_output_pin=0,
            digital_output_cmd=RobotCommand.Request.DIGITAL_OFF,
            pose=Twist(),
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
        request.pose = pose
        return request

    def call_hiwin(self, req):
        while not self.hiwin_client.wait_for_service(timeout_sec=2.0):
            self.logger.info('service not available, waiting again...')
        # future = self.hiwin_client.call_async(req)
        # rclpy.spin_until_future_complete(self.node,future)
        # print('dghgj')
        # res = future.result()
        # # else:
        # #     res = None
        # return res

        future = self.hiwin_client.call_async(req)
        if self._wait_for_future_done(future):
            res = future.result()
        else:
            res = None
        return res
    # def execute(self):
    #     self.JOINT_move()
    #     print("-------------------------")

    #     self.JOINT_move()

    #     self.PTP_move()

    # def start_main_loop_thread(self):
    #     self.main_loop_thread = Thread(target=self.execute)
    #     self.main_loop_thread.daemon = True
    #     self.main_loop_thread.start()


def execute(node):
    rclpy.spin(node)
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.node.Node('test')
    stratery = HIWIN_Controller(node)
    
    main_loop_thread = Thread(target=execute, args=(node,))
    # main_loop_thread.daemon = True
    main_loop_thread.start()
    

    stratery.JOINT_move()

    stratery.JOINT_move()
    
    stratery.PTP_move()
    main_loop_thread.join()
    # rclpy.spin(node)
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
