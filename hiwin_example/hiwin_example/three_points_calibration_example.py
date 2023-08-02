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
from ros2_aruco_interfaces.srv import ArucoMarkerInfo
# from YoloDetector import YoloDetectorActionClient

from math import *
from scipy.spatial.transform import Rotation as R
import numpy as np
import quaternion as qtn
from hiwin_example import transformations

from tf2_ros import TransformBroadcaster

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
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


class States(Enum):
    INIT = 0
    FINISH = 1
    MOVE_TO_PHOTO_POSE = 2
    GET_CALI_POINT = 3
    MOVE_TO_CALI_POINT = 4
    CALCULATE_COORDINATE = 5
    MOVE_TO_PLACE_POSE = 6
    CHECK_POSE = 7
    CLOSE_ROBOT = 8

class ThreePointsCalibration(Node):

    def __init__(self):
        super().__init__('three_points_calibration')
        self.calibration_server = self.create_client(RobotCommand, 
                                                    'calibration_points')
        self.hiwin_client = self.create_client(RobotCommand, 'hiwinmodbus_service')
        self.object_pose = None
        self.object_cnt = 0
        self.cali_cnt = 0
        self.marker_ids = []
        self.cali_pose = []
        self.final_cali_pose = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

    def _state_machine(self, state: States) -> States:
        if state == States.INIT:
            self.get_logger().info('INIT')
            nest_state = States.MOVE_TO_PHOTO_POSE

        elif state == States.MOVE_TO_PHOTO_POSE:
            self.get_logger().info('MOVE_TO_PHOTO_POSE')
            pose = Twist()
            req = self.generate_robot_request(
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                joints=PHOTO_POSE
                )
            res = self.call_hiwin(req)
            if res.arm_state == RobotCommand.Response.IDLE:
                nest_state = States.GET_CALI_POINT
            else:
                nest_state = None

        elif state == States.GET_CALI_POINT:
            self.get_logger().info('GET_CALI_POINT')
            aruco_req = ArucoMarkerInfo()
            aruco_req.process = True
            self.call_for_aruco(aruco_req)
            if len(res.markers) != 0:
                self.marker_ids = res.markers.marker_ids
                for i in range(len(res.markers)):
                    # self.cali_points = len(res.markers)
                    cali_pose = self.convert_arm_pose(res.markers.poses[i])
                    if res.markers[0] == 1:
                        self.cali_pose.append(cali_pose)
                    else:
                        self.cali_pose.insert(0, cali_pose)
                    self.cali_pose.append(cali_pose+[0.0, 150.0, 0.0])
                # self.object_pose = res.pose_array
                nest_state = States.MOVE_TO_CALI_POINT
            else:
                if self.cali_cnt > 5:
                    self.get_logger().info('get aruco marker failed')
                    self.cali_cnt = 0
                    nest_state = States.CLOSE_ROBOT
                else:
                    self.cali_cnt += 1
                    nest_state = States.GET_CALI_POINT
                # nest_state = States.CLOSE_ROBOT

        elif state == States.MOVE_TO_CALI_POINT:
            self.get_logger().info('MOVE_TO_CALI_POINT')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.CHECK_POSE)
            res = self.call_hiwin(req)
            pose = Twist()
            for i in range(len(self.marker_ids)+1):
                [pose.linear.x, pose.linear.y, pose.linear.z] = self.cali_pose[i][0:3]
                [pose.angular.x, pose.angular.y, pose.angular.z] = res.current_position[3:6]
                # pose.linear.z += 10
                req = self.generate_robot_request(
                    cmd_mode=RobotCommand.Request.PTP,
                    pose=pose)
                res = self.call_hiwin(req)
                if res.arm_state == RobotCommand.Response.IDLE:
                    while 1:
                        pose.linear.z += 0.001
                        req = self.generate_robot_request(
                            cmd_mode=RobotCommand.Request.LINE,
                            pose=pose)
                        res = self.call_hiwin(req)
                        req = self.generate_robot_request(
                            cmd_mode=RobotCommand.Request.READ_DI,
                            digital_input_pin=3,
                            holding=False)
                        res = self.call_hiwin(req)
                        if res.digital_state:
                            break
                    req = self.generate_robot_request(
                        cmd_mode=RobotCommand.Request.CHECK_POSE)
                    res = self.call_hiwin(req)
                    self.final_cali_pose.append(res.current_position)
                    nest_state = States.CALCULATE_COORDINATE
                else:
                    nest_state = None

        elif state == States.CALCULATE_COORDINATE:
            self.get_logger().info('CALCULATE_COORDINATE')
            self.get_vector()
            if res.arm_state == RobotCommand.Response.IDLE:
                nest_state = States.MOVE_TO_PLACE_POSE
            else:
                nest_state = None

        elif state == States.MOVE_TO_PLACE_POSE:
            self.get_logger().info('MOVE_TO_PLACE_POSE')
            pose = Twist()
            req = self.generate_robot_request(
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                joints=PLACE_POSE,
                pose=pose)
            res = self.call_hiwin(req)

            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.DIGITAL_OUTPUT,
                digital_output_cmd=RobotCommand.Request.DIGITAL_OFF,
                digital_output_pin=VACUUM_PIN,
                holding=True,
                pose=pose
            )
            res = self.call_hiwin(req)
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.WAITING
            )
            res = self.call_hiwin(req)
            if res.arm_state == RobotCommand.Response.IDLE:
                nest_state = States.MOVE_TO_PHOTO_POSE
            else:
                nest_state = None

        elif state == States.CHECK_POSE:
            self.get_logger().info('CHECK_POSE')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.CHECK_JOINT)
            res = self.call_hiwin(req)
            print(res.current_position)
            if res.arm_state == RobotCommand.Response.IDLE:
                nest_state = States.CLOSE_ROBOT
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

    def call_for_aruco(self, req):
        while not self.calibration_server.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.calibration_server.call_async(req)
        if self._wait_for_future_done(future):
            res = future.result()
        else:
            res = None
        return res

    def convert_arm_pose(self, aruco_pose):


        cam2tool_rot = qtn.as_rotation_matrix(np.quaternion(transform.orientation.w, 
                                                            transform.orientation.x, 
                                                            transform.orientation.y, 
                                                            transform.orientation.z))
        cam2tool_trans = np.array([[transform.position.x],
                                   [transform.position.y],
                                   [transform.position.z]])
        
        cam2tool_mat = np.append(cam2tool_rot, cam2tool_trans, axis=1)
        cam2tool_mat = np.append(cam2tool_mat, np.array([[0., 0., 0., 1.]]), axis=0)
        # transform_mat = np.linalg.inv(transform_mat)

        base2tool_rot = qtn.as_rotation_matrix(np.quaternion(transform.orientation.w, 
                                                             transform.orientation.x, 
                                                             transform.orientation.y, 
                                                             transform.orientation.z))
        base2tool_trans = np.array([[transform.position.x],
                                    [transform.position.y],
                                    [transform.position.z]])
        
        base2tool_mat = np.append(base2tool_rot, base2tool_trans, axis=1)
        base2tool_mat = np.append(base2tool_mat, np.array([[0., 0., 0., 1.]]), axis=0)



        cam2aruco_rot = qtn.as_rotation_matrix(np.quaternion(aruco_pose.orientation.w, 
                                                             aruco_pose.orientation.x, 
                                                             aruco_pose.orientation.y, 
                                                             aruco_pose.orientation.z))
        cam2aruco_trans = np.array([[aruco_pose.position.x],
                                    [aruco_pose.position.y],
                                    [aruco_pose.position.z]])
        
        cam2aruco_mat = np.append(cam2aruco_rot, cam2aruco_trans, axis=1)
        cam2aruco_mat = np.append(cam2aruco_mat, np.array([[0., 0., 0., 1.]]), axis=0)

        base2cam_mat = np.matmul(base2tool_rot, cam2tool_mat)
        base2aruco_mat = np.matmul(base2cam_mat, cam2aruco_mat)


        ax, ay, az = transformations.euler_from_matrix(base2aruco_mat)
        base2aruco_translation = transformations.translation_from_matrix(base2aruco_mat)*100.0
        # print(angle[0]*180/pi)
        # print(angle[1]*180/pi)
        # print(angle[2]*180/pi)
        calibrate_pose = [base2aruco_translation[0], 
                       base2aruco_translation[1], 
                       base2aruco_translation[2],
                       ax, ay, az]

        return calibrate_pose
        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         'base',
        #         'ar_marker',
        #         rclpy.time.Time())
        #     print(t)
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform base to ar_marker: {ex}')
        #     return
    def get_vector(self):
        #
        # self.final_cali_pose[0][0:3]
        # if self.marker_ids[0] == 1:
        #     new_base_point = self.final_cali_pose[0][0:3]
        #     x_extend_point = self.final_cali_pose[1][0:3]
        # else:
        #     new_base_point = self.final_cali_pose[1][0:3]
        #     x_extend_point = self.final_cali_pose[0][0:3]

        new_base_point = self.final_cali_pose[0][0:3]
        x_extend_point = self.final_cali_pose[1][0:3]
        third_point = self.final_cali_pose[2][0:3]
        new_base_point = np.array(new_base_point)
        x_extend_point = np.array(x_extend_point)
        third_point = np.array(third_point)
        
        Vector_X = x_extend_point - new_base_point     # X軸向量
        Vector_other = third_point - new_base_point    # base and other 兩點向量
        Vector_Z = np.cross(Vector_X, Vector_other) # 外積出Z軸向量
        # print(Vector_AZ)
        Vector_Y = np.cross(Vector_X, Vector_Z)  # 外積出Y軸向量
        # print(Vector_AY)

        vx = Vector_X / np.linalg.norm(Vector_X)    # 轉成單位向量
        vy = Vector_Y / np.linalg.norm(Vector_Y)
        vz = Vector_Z / np.linalg.norm(Vector_Z)

        Transpose = np.array([vx, vy, vz]).T
        r = R.from_matrix(Transpose)
        euler_angle = r.as_euler('xyz', degrees=True)
        print(euler_angle)

        NEW_BASE = [new_base_point[0], 
                  new_base_point[1], 
                  new_base_point[2], 
                  euler_angle[0], 
                  euler_angle[1], 
                  euler_angle[2]]
        print(NEW_BASE)

        return NEW_BASE

    def start_calibration_thread(self):
        self.main_loop_thread = Thread(target=self._main_loop)
        self.main_loop_thread.daemon = True
        self.main_loop_thread.start()



def main(args=None):
    rclpy.init(args=args)

    stratery = ThreePointsCalibration()
    stratery.start_calibration_thread()

    rclpy.spin(stratery)
    rclpy.shutdown()

if __name__ == "__main__":
    main()