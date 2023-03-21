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

DEFAULT_VELOCITY = 10
DEFAULT_ACCELERATION = 10

VACUUM_PIN = 3

PHOTO_POSE = [0.00, 0.00, 0.00, 0.00, -90.00, 0.00]
OBJECT_POSE = [20.00, 0.00, 0.00, 0.00, -90.00, 0.00]
PLACE_POSE = [-20.00, 0.00, 0.00, 0.00, -90.00, 0.00]

# only for example as we don't use yolo here
# assume NUM_OBJECTS=5, then this process will loop 5 times
NUM_OBJECTS = 5


class States(Enum):
    INIT = 0
    FINISH = 1
    MOVE_TO_PHOTO_POSE = 2
    YOLO_DETECT = 3
    MOVE_TO_OPJECT_TOP = 4
    PICK_OBJECT = 5
    MOVE_TO_PLACE_POSE = 6
    CLOSE_ROBOT = 7

class ExampleStrategy(Node):

    def __init__(self):
        super().__init__('example_strategy')
        self.hiwin_client = self.create_client(RobotCommand, 'hiwinmodbus_service')
        self.object_pose = None
        self.object_cnt = 0

    def _state_machine(self, state: States) -> States:
        if state == States.INIT:
            self.get_logger().info('INIT')
            nest_state = States.MOVE_TO_PHOTO_POSE

        elif state == States.MOVE_TO_PHOTO_POSE:
            self.get_logger().info('MOVE_TO_PHOTO_POSE')
            pose = Twist()
            [pose.linear.x, pose.linear.y, pose.linear.z] = PHOTO_POSE[0:3]
            [pose.angular.x, pose.angular.y, pose.angular.z] = PHOTO_POSE[3:6]
            req = self.generate_robot_request(pose=pose)
            res = self.call_hiwin(req)
            if res == RobotCommand.Response.IDLE:
                nest_state = States.YOLO_DETECT
            else:
                nest_state = None

        elif state == States.YOLO_DETECT:
            self.get_logger().info('YOLO_DETECT')
            res = self.call_yolo()
            # OBJECT_POSE here for example, should get obj pose according to yolo result
            if res.has_object:
                self.object_pose = res.object_pose
                nest_state = States.MOVE_TO_OPJECT_TOP
            else:
                nest_state = States.CLOSE_ROBOT

        elif state == States.MOVE_TO_OPJECT_TOP:
            self.get_logger().info('MOVE_TO_OPJECT_TOP')
            pose = Twist()
            [pose.linear.x, pose.linear.y, pose.linear.z] = self.object_pose[0:3]
            [pose.angular.x, pose.angular.y, pose.angular.z] = self.object_pose[3:6]
            pose.linear.z += 10
            req = self.generate_robot_request(pose=pose)
            res = self.call_hiwin(req)
            if res == RobotCommand.Response.IDLE:
                nest_state = States.PICK_OBJECT
            else:
                nest_state = None

        elif state == States.PICK_OBJECT:
            self.get_logger().info('PICK_OBJECT')
            pose = Twist()
            [pose.linear.x, pose.linear.y, pose.linear.z] = self.object_pose[0:3]
            [pose.angular.x, pose.angular.y, pose.angular.z] = self.object_pose[3:6]

            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                holding=False,
                velocity=5,
                pose=pose
            )
            res = self.call_hiwin(req)

            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.DIGITAL_OUTPUT,
                digital_output_cmd=RobotCommand.Request.DIGITAL_ON,
                digital_output_pin=VACUUM_PIN,
                holding=False,
                pose=pose
            )
            res = self.call_hiwin(req)

            pose.linear.z += 10
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                pose=pose
            )
            res = self.call_hiwin(req)

            if res == RobotCommand.Response.IDLE:
                nest_state = States.MOVE_TO_PLACE_POSE
            else:
                nest_state = None

        elif state == States.MOVE_TO_PLACE_POSE:
            self.get_logger().info('MOVE_TO_PLACE_POSE')
            pose = Twist()
            [pose.linear.x, pose.linear.y, pose.linear.z] = PLACE_POSE[0:3]
            [pose.angular.x, pose.angular.y, pose.angular.z] = PLACE_POSE[3:6]
            req = self.generate_robot_request(pose=pose)
            res = self.call_hiwin(req)
            if res == RobotCommand.Response.IDLE:
                nest_state = States.YOLO_DETECT
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
        return nest_state

    def _main_loop(self):
        state = States.INIT
        while state != States.FINISH:
            state = self._state_machine(state)
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
        while not self.hiwin_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.hiwin_client.call_async(req)
        if self._wait_for_future_done(future):
            res = future.result()
        else:
            res = None
        return res

    def call_yolo(self):
        class YoloResponse(NamedTuple):
            has_object: bool
            object_pose: list
        res = YoloResponse()
        res.has_object = True if self.object_cnt < 5 else False
        res.object_pose = OBJECT_POSE
        self.object_cnt += 1
        return res

    def start_main_loop_thread(self):
        self.main_loop_thread = Thread(target=self._main_loop)
        self.main_loop_thread.daemon = True
        self.main_loop_thread.start()

def main(args=None):
    rclpy.init(args=args)

    stratery = ExampleStrategy()
    stratery.start_main_loop_thread()

    rclpy.spin(stratery)
    rclpy.shutdown()

if __name__ == "__main__":
    main()