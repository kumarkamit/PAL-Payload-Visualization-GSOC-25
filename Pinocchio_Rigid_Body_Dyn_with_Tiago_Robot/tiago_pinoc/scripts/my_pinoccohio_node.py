#!/usr/bin/env python3

'''
*******************************
*  Filename:			my_pinoccohio_node.py
*  Created:				04/04/2025
*  Last Modified:	    08/04/2025
*  Modified by:         Amit Kumar
*  Author:				Amit Kumar
*******************************
Ref- https://auctus-team.github.io/pycapacity/examples/ROS.html
     https://github.com/pal-robotics/tiago_robot/tree/humble-devel
     https://github.com/stack-of-tasks/pinocchio/issues/2156
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import pinocchio as pin
import numpy as np

from urdf_parser_py.urdf import URDF
import tempfile
import os

class PinocchioNode(Node):
    def __init__(self):
        super().__init__('my_pinocchio_node')

        # Subscriber to receive robot description
        self.subscription = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            10
        )
        # Subscriber to receive joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # Pinocchio model and data
        self.robot_mod = None
        self.robot_geom = None
        self.robot = None
        self.data = None

    def joint_states_callback(self, joint_data):
        if self.robot==None:
            return

        arm_7_joint_frame = self.robot.getFrameId('arm_7_joint')
        universe_frame = self.robot.getFrameId('universe')
        self.robot.frames[arm_7_joint_frame]

        joint_positions = np.array(joint_data.position)
        
        # Match Joints as per robot description
        joint_positions = np.insert(joint_positions,0, np.array([0.0, 0.0]))
        
        # Calculate forward kinematics of the robot
        self.robot_mod.framesForwardKinematics(joint_positions)        

        oMarm7 = self.data.oMf[arm_7_joint_frame]
        oMuniv = self.data.oMf[universe_frame]

        print("Arm7:\n",oMarm7)
        
        # Calculate Jacobian of Arm7 Frame
        Jarm7 = self.robot_mod.computeFrameJacobian(joint_positions,arm_7_joint_frame)
        
        print("Jacobian of Arm7:\n",Jarm7)

        # Check collision
        pin.updateGeometryPlacements(self.robot, self.data, self.collision_model, self.collision_data)

        # Check for self-collisions
        for k, collision_pair in enumerate(self.collision_model.collisionPairs):
            collision_result = pin.computeCollision(self.collision_model, self.collision_data, k)
            if collision_result:
                obj1 = self.collision_model.geometryObjects[collision_pair.first].name
                obj2 = self.collision_model.geometryObjects[collision_pair.second].name
                print(f"Collision detected between: {obj1} and {obj2}")
        print("---------")

    def robot_description_callback(self, msg: String):
        self.robot_mod = pin.RobotWrapper(model=pin.buildModelFromXML(msg.data),collision_model=pin.buildModelFromXML(msg.data))
        self.robot = self.robot_mod.model
        self.data = self.robot_mod.data

        print("Robot successfully loaded.")

        urdf_robot = URDF.from_xml_string(msg.data)

        # Save to temporary URDF file
        with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as urdf_file:
            urdf_file.write(msg.data.encode("utf-8"))
            urdf_path = urdf_file.name

        # Load the robot in Pinocchio
        model_path = os.path.dirname(urdf_path)
        robot_urdf = pin.RobotWrapper.BuildFromURDF(urdf_path)#, package_dirs, pin.JointModelFreeFlyer())

        # Create collision model and data
        self.collision_model = robot_urdf.collision_model
        self.collision_data = robot_urdf.collision_data

        # Add all unique pairs of geometry objects (excluding self-pairs)
        for i in range(len(self.collision_model.geometryObjects)):
            for j in range(i + 1, len(self.collision_model.geometryObjects)):
                pair = pin.CollisionPair(i, j)
                if not self.collision_model.existCollisionPair(pair):
                    self.collision_model.addCollisionPair(pair)

        self.collision_data = pin.GeometryData(self.collision_model)



def main(args=None):
    rclpy.init(args=args)
    node = PinocchioNode()

    # Running and spinning the node to handle messages
    rclpy.spin(node)

    # Shutdown and cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()
