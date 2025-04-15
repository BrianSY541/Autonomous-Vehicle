#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import numpy as np

from scipy.spatial.transform import Rotation as R

from typing import List

class PIDcontroller(Node):
    def __init__(self, Kp, Ki, Kd, waypoints: List[np.ndarray]):
        super().__init__('pid_controller')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.waypoints = waypoints
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.maximumValue = 0.02
        self.pose = Pose()
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.position.z = 0.0
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = 1.0
        self.publisher = self.create_publisher(Twist, '/twist_world', 1)
        self.subscription = self.create_subscription(Pose, '/pose', self.pose_callback, 1)
        self.subscription
        self.previous_time = time.time()
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        if len(self.waypoints) == 0 and self.target is None:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            raise SystemExit
        if self.target is None:
            self.setTarget(self.waypoints.pop(0))
        else:
            rot = R.from_quat([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
            yaw = rot.as_euler('zyx')[0]
            if np.linalg.norm(self.getError(np.array([self.pose.position.x, self.pose.position.y, yaw]), self.target)) < 0.1:
                self.setTarget(self.waypoints.pop(0))
        if self.target is not None:
            twist = Twist()
            rot = R.from_quat([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
            yaw = rot.as_euler('zyx')[0]
            twist.linear.x, twist.linear.y, twist.angular.z = self.update(np.array([self.pose.position.x, self.pose.position.y, yaw]))
            self.publisher.publish(twist)
    
    def pose_callback(self, pose):
        self.pose = pose

    def setTarget(self, target):
        """
        Set the target pose.
        """
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
        self.target = np.array(target)

    def getError(self, currentState, targetState):
        """
        Return the difference between two states.
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result

    def setMaximumUpdate(self, mv):
        """
        Set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        Calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)
        P = self.Kp * e
        self.I += self.Ki * e * (time.time() - self.previous_time)
        self.previous_time = time.time()
        D = self.Kd * (e - self.lastError)
        result = P + self.I + D
        self.lastError = e

        # Scale down the twist if its norm is more than the maximum value
        resultNorm = np.linalg.norm(result)
        if resultNorm > self.maximumValue:
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

if __name__ == "__main__":
    rclpy.init()

    # Square waypoints
    waypoints_square = [
        np.array([1.0, 0.0, 0.0]),
        np.array([1.0, 0.0, np.pi/2]),
        np.array([1.0, 1.0, np.pi/2]),
        np.array([1.0, 1.0, np.pi]),
        np.array([0.0, 1.0, np.pi]),
        np.array([0.0, 1.0, -np.pi/2]),
        np.array([0.0, 0.0, -np.pi/2]),
        np.array([0.0, 0.0, 0.0])
    ]

    # Octagon waypoints
    factor = 1.0/(1 + np.sqrt(2))
    shift = factor / np.sqrt(2)
    waypoints_octagon = [
        np.array([shift, 0.0, 0.0]), # Start from side
        np.array([shift + factor, 0.0, 0.0]), # Move to corner
        np.array([shift + factor, 0.0, np.pi/4]),
        np.array([2*shift + factor, shift, np.pi/4]),
        np.array([2*shift + factor, shift, np.pi/2]),
        np.array([2*shift+factor, shift+factor, np.pi/2]),
        np.array([2*shift+factor, shift+factor, 3*np.pi/4]),
        np.array([shift+factor, 2*shift+factor, 3*np.pi/4]),
        np.array([shift+factor, 2*shift+factor, np.pi]),
        np.array([shift, 2*shift+factor, np.pi]),
        np.array([shift, 2*shift+factor, -np.pi/4]),
        np.array([0.0, shift+factor, -np.pi/4]),
        np.array([0.0, shift+factor, -np.pi/2]),
        np.array([0.0, shift, -np.pi/2]),
        np.array([0.0, shift, -3*np.pi/4]),
        np.array([shift, 0.0, -3*np.pi/4]),
        np.array([shift, 0.0, 0.0]) # Return to start
    ]

    # Two rounds of square and octagon waypoints
    waypoints = waypoints_square*2 + waypoints_octagon*2

    pid_controller_node = PIDcontroller(0.5, 0.01, 0.005, waypoints = waypoints)
    
    try:
        rclpy.spin(pid_controller_node) # Spin for until shutdown
    except SystemExit:
        pid_controller_node.destroy_node()

    rclpy.shutdown()