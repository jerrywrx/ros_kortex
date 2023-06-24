#!/usr/bin/env python

import rospy
import sys
import os
from std_msgs.msg import Float64
import utilities
import argparse
from kortex_driver.msg import TwistCommand

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

class SpacemouseToKinova:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('spacemouse_to_kinova')

        # # Parse arguments
        # parser = argparse.ArgumentParser()
        # args = utilities.parseConnectionArguments(parser)

        # self.router = utilities.DeviceConnection.createTcpConnection(args)
        # self.base = BaseClient(self.router)

        self.cmd = TwistCommand()
        self.cmd.reference_frame = 0
        self.cmd.twist.linear_x = 0.0
        self.cmd.twist.linear_y = 0.0
        self.cmd.twist.linear_z = 0.0
        self.cmd.twist.angular_x = 0.0
        self.cmd.twist.angular_y = 0.0
        self.cmd.twist.angular_z = 0.0
        self.cmd.duration = 0

        self.grasp = 0

        # Initialize publisher for kinova robot
        self.pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=10)

        # Subscribe to spacemouse linear velocity commands
        rospy.Subscriber("spacemouse_state/dx", Float64, self.spacemouse_linear_x_callback)
        rospy.Subscriber("spacemouse_state/dy", Float64, self.spacemouse_linear_y_callback)
        rospy.Subscriber("spacemouse_state/dz", Float64, self.spacemouse_linear_z_callback)

        # Subscribe to spacemouse angular velocity commands
        rospy.Subscriber("spacemouse_state/dpitch", Float64, self.spacemouse_angular_x_callback)
        rospy.Subscriber("spacemouse_state/dyaw", Float64, self.spacemouse_angular_y_callback)
        rospy.Subscriber("spacemouse_state/droll", Float64, self.spacemouse_angular_z_callback)

        rospy.Subscriber("spacemouse_state/grasp", Float64, self.spacemouse_grasp_callback)

    def spacemouse_linear_x_callback(self, msg):
        self.cmd.twist.linear_x = -msg.data * 40
        

    def spacemouse_linear_y_callback(self, msg):
        self.cmd.twist.linear_y = -msg.data * 40
        

    def spacemouse_linear_z_callback(self, msg):
        self.cmd.twist.linear_z = msg.data * 20

    def spacemouse_angular_x_callback(self, msg):
        self.cmd.twist.angular_x = -msg.data * 60

    def spacemouse_angular_y_callback(self, msg):
        self.cmd.twist.angular_y = -msg.data * 60

    def spacemouse_angular_z_callback(self, msg):
        self.cmd.twist.angular_z = -msg.data * 60

    def spacemouse_grasp_callback(self, msg):
        self.grasp = msg.data

    def send_home_command(self):
        # Send the robot to Home Position
        print("Sending robot to Home position...")
        self.base.SendHomeCommand()
        print("Waiting for robot to reach Home position...")
        rospy.sleep(10.0)  # Wait for robot to complete the move
        print("Robot is now at Home position.")

    def run(self):
        # Import the utilities helper module
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

        # Parse arguments
        parser = argparse.ArgumentParser()
        args = utilities.parseConnectionArguments(parser)

        rate = rospy.Rate(10)  # 10Hz

        # self.send_home_command()

        # self.router.Close()

        with utilities.DeviceConnection.createTcpConnection(args) as router:
            example = GripperCommandExample(router)
            while not rospy.is_shutdown():
                print("linear_x: ", self.cmd.twist.linear_x)
                print("linear_y: ", self.cmd.twist.linear_y)
                print("linear_z: ", self.cmd.twist.linear_z)
                print("angular_x: ", self.cmd.twist.angular_x)
                print("angular_y: ", self.cmd.twist.angular_y)
                print("angular_z: ", self.cmd.twist.angular_z)

                self.pub.publish(self.cmd)
                example.ExampleSendGripperCommands(self.grasp)
                print("")

                rate.sleep()

class GripperCommandExample:
    def __init__(self, router, proportional_gain = 2.0):

        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)
        self.last_grasp = 0
        self.actual_grasp = 0

    def ExampleSendGripperCommands(self, grasp):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        
        # detect positive edge of grasp button
        if not self.last_grasp and grasp:
            self.actual_grasp = not self.actual_grasp
        self.last_grasp = grasp

        print("Grasp: ", self.actual_grasp)
        finger.value = self.actual_grasp
        self.base.SendGripperCommand(gripper_command)
            

if __name__ == "__main__":
    controller = SpacemouseToKinova()
    controller.run()
