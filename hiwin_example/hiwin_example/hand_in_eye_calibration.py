#!/usr/bin/env python3
import time
import keyboard
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from rclpy.task import Future
from typing import NamedTuple
from geometry_msgs.msg import Twist

from hiwin_interfaces.srv import RobotCommand
# from ros2_aruco_interfaces.srv import ArucoMarkerInfo
# from YoloDetector import YoloDetectorActionClient

from math import *
from scipy.spatial.transform import Rotation as R
import numpy as np
import quaternion as qtn
from hiwin_example import transformations

from tf2_ros import TransformBroadcaster

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

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


CALI_POSE = [[0.0, 368.0, 293.5, -180.0, 0.0, 90.0],
[169.087, 367.999, 293.499, 179.999, 0.0, 89.999],
[-124.949, 367.999, 293.499, -179.999, 0.0, 89.999],
[-124.95, 389.112, 293.5, -179.999, 0.0, 89.999],
[-6.375, 343.099, 293.499, -179.999, 0.0, 114.776],
[11.587, 345.349, 293.5, -179.999, 0.0, 141.454],
[43.537, 350.037, 293.499, 178.254, 15.194, 173.366],
[43.537, 491.112, 293.499, 178.254, 15.194, 173.366],
[-67.059, 549.197, 293.109, 178.246, 15.183, 173.348],
[-67.059, 475.135, 293.109, 178.246, 15.183, 43.696],
[-126.309, 514.613, 290.709, 178.246, 15.183, 43.696],
[47.54, 512.325, 290.71, 178.246, 15.183, 10.671],
[127.938, 500.858, 221.335, 174.706, 29.131, 76.06],
[-113.189, 465.175, 238.726, 153.091, 17.557, 68.415],
[-19.107, 484.406, 222.025, 162.894, 14.341, 16.293],
[-14.25, 255.246, 159.015, 178.03, -17.123, 89.891],
[-14.25, 255.246, 93.877, 178.03, -17.123, 89.891],
[-14.195, 431.346, 78.277, -179.687, 13.316, 89.095],
[-14.195, 431.346, 167.19, -179.746, 16.908, 89.079],
[94.141, 504.729, 167.19, -179.746, 16.908, 89.079],
[140.191, 504.729, 167.19, -179.746, 16.908, 89.079],
[-63.208, 386.342, 167.19, 158.774, 2.216, 88.169]]


class States(Enum):
    INIT = 0
    FINISH = 1
    BROADCASTE_POSE = 2
    MOVE_TO_CALI_POSE = 3
    MOVE_TO_CALI_POINT = 4
    CALCULATE_COORDINATE = 5
    MOVE_TO_PLACE_POSE = 6
    CHECK_POSE = 7
    CLOSE_ROBOT = 8

class HandInEyeCalibration(Node):

    def __init__(self):
        super().__init__('hand_in_eye_calibration')
        self.hiwin_client = self.create_client(RobotCommand, 'hiwinmodbus_service')
        self.cali_pose_cnt = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    def _state_machine(self, state: States) -> States:
        if state == States.INIT:
            self.get_logger().info('INIT')
            
            # t = TransformStamped()

            # t.header.stamp = self.get_clock().now().to_msg()
            # t.header.frame_id = 'world'
            # t.child_frame_id = 'base_link'

            # t.transform.translation.x = 0.0
            # t.transform.translation.y = 0.0
            # t.transform.translation.z = 0.0
            
            # t.transform.rotation.x = 0.0
            # t.transform.rotation.y = 0.0
            # t.transform.rotation.z = 0.0
            # t.transform.rotation.w = 1.0

            # self.tf_static_broadcaster.sendTransform(t)

            nest_state = States.BROADCASTE_POSE

        elif state == States.BROADCASTE_POSE:
            self.get_logger().info('BROADCASTE_POSE')
            pose = Twist()
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.CHECK_POSE,
                holding=False
                )
            res = self.call_hiwin(req)

            pose = TransformStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'base_link'
            pose.child_frame_id = 'tool0'
            pose.transform.translation.x = res.current_position[0]/1000.0
            pose.transform.translation.y = res.current_position[1]/1000.0
            pose.transform.translation.z = res.current_position[2]/1000.0
        
            quat = transformations.quaternion_from_euler(res.current_position[3]*3.14/180,
                                                         res.current_position[4]*3.14/180,
                                                         res.current_position[5]*3.14/180,axes= "sxyz")
            pose.transform.rotation.x = quat[0]
            pose.transform.rotation.y = quat[1]
            pose.transform.rotation.z = quat[2]
            pose.transform.rotation.w = quat[3]

            self.tf_broadcaster.sendTransform(pose)

            if self.cali_pose_cnt != 22:
                nest_state = States.MOVE_TO_CALI_POSE
            else:
                nest_state = States.CLOSE_ROBOT

        elif state == States.MOVE_TO_CALI_POSE:
            self.get_logger().info('MOVE_TO_CALI_POSE')
            pose = Twist()
            [pose.linear.x, pose.linear.y, pose.linear.z] = CALI_POSE[self.cali_pose_cnt][0:3]
            [pose.angular.x, pose.angular.y, pose.angular.z] = CALI_POSE[self.cali_pose_cnt][3:6]
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=pose
                )
            res = self.call_hiwin(req)
            self.cali_pose_cnt += 1
            if res.arm_state == RobotCommand.Response.IDLE:
                nest_state = States.BROADCASTE_POSE
            else:
                nest_state = None

        elif state == States.CLOSE_ROBOT:
            self.get_logger().info('CLOSE_ROBOT')
            req = self.generate_robot_request(cmd_mode=RobotCommand.Request.CLOSE)
            res = self.call_hiwin(req)
            nest_state = States.FINISH

        else:
            nest_state = None
            self.get_logger().error('Input state not supported!')
            # return
        return nest_state

    def _main_loop(self):
        state = States.INIT
        while state != States.FINISH:
            state = self._state_machine(state)
            if state == None:
                break
        self.destroy_node()

    def _wait_for_future_done(self, future: Future, timeout=-1):
        time_start = time.time()
        while not future.done():
            time.sleep(0.01)
            if timeout > 0 and time.time() - time_start > timeout:
                self.get_logger().error('Wait for service timeout!')
                return False
        return True
    
    def generate_robot_request(
            self, 
            holding=True,
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.POSE_CMD,
            velocity=DEFAULT_VELOCITY,
            acceleration=DEFAULT_ACCELERATION,
            tool=1,
            base=0,
            digital_input_pin=0,
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
        request.digital_input_pin = digital_input_pin
        request.digital_output_pin = digital_output_pin
        request.digital_output_cmd = digital_output_cmd
        request.acceleration = acceleration
        request.jog_joint = jog_joint
        request.velocity = velocity
        request.tool = tool
        request.base = base
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
            self.get_logger().info('service not available, waiting again...')
        future = self.hiwin_client.call_async(req)
        if self._wait_for_future_done(future):
            res = future.result()
        else:
            res = None
        return res


    def start_calibration_thread(self):
        self.main_loop_thread = Thread(target=self._main_loop)
        self.main_loop_thread.daemon = True
        self.main_loop_thread.start()



def main(args=None):
    rclpy.init(args=args)

    stratery = HandInEyeCalibration()
    stratery.start_calibration_thread()

    rclpy.spin(stratery)
    rclpy.shutdown()

if __name__ == "__main__":
    main()