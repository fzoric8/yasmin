#!/usr/bin/env python3

from asyncio import create_subprocess_exec
import time
import rclpy
from math import sqrt

from simple_node import Node

from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

from geometry_msgs.msg import Pose, PoseStamped


# define state Foo
class MoveUAV(State):
    def __init__(self, cmd_pose, curr_pose, timeout):
        super().__init__(["successful", "failed", "moving"])
        self.counter = 0
        #self.cmd_pose_ = PoseStamped(); 
        #self.curr_pose_ = PoseStamped(); 
        self.cmd_pose_ = cmd_pose.pose
        self.curr_pose_ = curr_pose.pose
        self.timeout = timeout
        self.time_elapsed = False
        self.start_time = time.time()

    def check_pose_diff(self): 

        x_diff = float(self.cmd_pose_.position.x - self.curr_pose_.position.x) 
        y_diff = float(self.cmd_pose_.position.y - self.curr_pose_.position.y)
        z_diff = float(self.cmd_pose_.position.z - self.curr_pose_.position.z)

        return sqrt(x_diff**2 + y_diff**2 + z_diff**2)


    def execute(self, blackboard):
        print("Executing state FOO")
        time.sleep(3)

        # TODO: Add timeout
        if not self.time_elapsed:
            self.counter += 1

            # Check if timeout reached
            current_time = time.time()
            self.time_elapsed = (current_time - self.start_time) > self.timeout

            # Check current pose 
            pose_diff = self.check_pose_diff()
            
            print("Pose command is: {}".format(self.cmd_pose_))
            print("Current pose is: {}".format(self.curr_pose_))
            print("Current pose diff is: {}".format(pose_diff))
            blackboard.foo_str = "Counter: " + str(self.counter)

            if (pose_diff <  0.001):
                return "successful"

            else: 
                print("Waiting for it to reach position")
                return "moving"
        else:
            print("Timeout reached!")
            return "failed"


# define state Bar
class BarState(State):
    def __init__(self):
        super().__init__(outcomes=["outcome3"])

    # TODO: Add in execute conditions for failure or successful approach
    def execute(self, blackboard):
        print("Executing state BAR")
        time.sleep(3)

        print(blackboard.foo_str)
        return "outcome3"


class DemoNode(Node):

    def __init__(self, uav_ns):
        super().__init__("yasmin_node")

        # add neccessary topics for sending pose
        self.current_pose = PoseStamped()
        self.cmd_pose = PoseStamped()
        #self.pose_pub = self.create_publisher(Pose, "{}/pose_ref".format(uav_ns), 1)
        self.pose_sub = self.create_subscription(PoseStamped,
                                                 "{}/pose_gt".format(uav_ns),
                                                 self.pose_cb, 1)
        self.pose_cmd_sub = self.create_subscription(PoseStamped,
                                                     "{}/pose_ref".format(uav_ns),
                                                     self.pose_ref_cb, 1)

        self.pose_reciv = False
        self.pose_cmd_reciv = False

        while not (self.pose_reciv and self.pose_cmd_reciv): 
            time.sleep(1)
            print("Waiting for first cmd...")

        # create a state machine
        sm = StateMachine(outcomes=["outcome4"])

        # add states
        # FooState becomes class for itself and it's not able to refresh current_pose
        # It's neccessary to refresh current pose to be able to move in our state machine 
        sm.add_state("MOVEUAV", MoveUAV(self.cmd_pose, self.current_pose, 20),
                     transitions={"successful": "BAR",
                                  "failed": "BAR", 
                                  "moving": "MOVEUAV"})

        sm.add_state("BAR", BarState(),
                     transitions={"outcome3": "outcome4"})

        # pub
        YasminViewerPub(self, "YASMIN_DEMO", sm)

        # execute
        outcome = sm()
        print(outcome)

    def pose_cb(self, msg): 

        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose
        self.pose_reciv = True
        print(self.current_pose)
        

    def pose_ref_cb(self, msg): 
        self.cmd_pose.header = msg.header
        self.cmd_pose.pose = msg.pose
        self.pose_cmd_reciv = True
        print(self.cmd_pose)


# main
def main(args=None):

    print("yasmin_demo")
    rclpy.init(args=args)
    node = DemoNode("uav1")
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
