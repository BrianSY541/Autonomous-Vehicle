#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Twist
import numpy as np

from threading import Lock
from scipy.spatial.transform import Rotation as Rot

def scope_angle(angle):
    if angle > np.pi:
        angle -= 2 * np.pi
    elif angle <= -np.pi:
        angle += 2 * np.pi
    return angle


class KalmanFilter(Node):
    def __init__(self, Q, R, Qv, Rv, covariance=None, use_ekf=True):
        super().__init__('kalman_filter')
        self.april_map = {}
        self.april_counter = 1
        self.Q = Q
        self.R = R
        self.Qv = Qv
        self.Rv = Rv
        self.twist = np.zeros(3)
        self.use_ekf = use_ekf
        self.state = np.zeros(3)
        self.covariance = covariance if covariance is not None else np.diag([1.0, 1.0, 0.3])
        self.sigma = np.eye(3)*0.01
        self.publisher = self.create_publisher(Pose, '/pose', 1)
        self.april_sub = self.create_subscription(PoseArray, '/april_poses', self.update, 1)
        self.april_sub
        self.twist_pub = self.create_publisher(Twist, '/twist', 1)
        self.lock = Lock()
        self.previous_time = time.time()
        self.twist_sub = self.create_subscription(Twist, '/twist_world', self.predict, 1)
        self.twist_sub
        
    # Prediction step
    def predict(self, twist):
        dt = time.time() - self.previous_time
        self.previous_time = time.time()
        self.twist[0] = twist.linear.x
        self.twist[1] = twist.linear.y
        self.twist[2] = twist.angular.z
        self.lock.acquire()
        # Predict the next state
        self.state[:3] +=  dt*np.array([twist.linear.x, twist.linear.y, twist.angular.z])
        self.state[2] = scope_angle(self.state[2])
        # Predict the next covariance
        Q = self.Q + dt*self.Qv*(self.twist.reshape(-1, 1))
        for i in range(len(self.state)//3):
            self.sigma[i*3:i*3+3, i*3:i*3+3] += Q
        self.lock.release()
        # Rotate twist to body frame
        twist_body = Twist()
        twist_body.linear.x = twist.linear.x*np.cos(self.state[2]) + twist.linear.y*np.sin(self.state[2])
        twist_body.linear.y = -twist.linear.x*np.sin(self.state[2]) + twist.linear.y*np.cos(self.state[2])
        twist_body.angular.z = twist.angular.z
        self.twist_pub.publish(twist_body)
        # Publish the predicted state
        pose = Pose()
        pose.position.x = self.state[0]
        pose.position.y = self.state[1]
        pose.position.z = 0.0
        rot = Rot.from_rotvec([0, 0, self.state[2]])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rot.as_quat()
        self.publisher.publish(pose)
    
    def update(self, measurement: PoseArray):
        detections = measurement.header.frame_id.split(',')[:-1]
        known_detections = [detection for detection in detections if detection in self.april_map]
        length = len(known_detections)
        z = np.zeros((length, 3))
        H = np.zeros((length*3, len(self.state)))

        self.lock.acquire()
        # Matrix to substract relative robot position in homogeneous coordinates
        robot_mat = np.array([
            [np.cos(self.state[2]), np.sin(self.state[2]), 0],
            [-np.sin(self.state[2]), np.cos(self.state[2]), 0],
            [0, 0, 1]
        ])
        lm_mat = np.array([
            [np.cos(self.state[2]), np.sin(self.state[2]), 0],
            [-np.sin(self.state[2]), np.cos(self.state[2]), 0],
            [0, 0, 1]
        ])
        if length > 0:
            for i, detection in enumerate(detections):
                if detection not in self.april_map:
                    continue
                # Correction for coordinates
                z[i][0] = measurement.poses[i].position.z
                z[i][1] = -measurement.poses[i].position.x
                # Rotation along y-axis
                rot = Rot.from_quat([measurement.poses[i].orientation.x, measurement.poses[i].orientation.y, measurement.poses[i].orientation.z, measurement.poses[i].orientation.w])
                z[i][2] = -rot.as_rotvec()[1]
                # Calculate the Jacobian
                j = self.april_map[detection]
                # Jacbian based on the state
                H[i*3:i*3+3, j*3:j*3+3] = lm_mat
                H[i*3:i*3+3, 0:3] = -robot_mat
                if self.use_ekf:
                    sin_theta = np.sin(self.state[2])
                    cos_theta = np.cos(self.state[2])
                    dhx_dtheta = sin_theta * self.state[0] - cos_theta * self.state[1] \
                        - sin_theta * self.state[j*3] + cos_theta * self.state[j*3+1]
                    dhy_dtheta = cos_theta * self.state[0] + sin_theta * self.state[1] \
                        - cos_theta * self.state[j*3] - sin_theta * self.state[j*3+1]
                    H[i*3, 2] = dhx_dtheta
                    H[i*3+1, 2] = dhy_dtheta
            yhat = z.reshape(-1) - H.dot(self.state)
            S = H.dot(self.sigma).dot(H.T)
            dt = time.time() - measurement.header.stamp.sec - measurement.header.stamp.nanosec*1e-9
            R = self.R + dt*self.Rv*(self.twist.reshape(-1, 1))
            for i in range(length):
                S[i*3:i*3+3, i*3:i*3+3] += R
            K = self.sigma.dot(H.T).dot(np.linalg.inv(S))
            self.state += K.dot(yhat)
            self.sigma -= K.dot(H).dot(self.sigma)
        
        if length < len(detections):
            robot_mat_inv = robot_mat.T
            for i, detection in enumerate(detections):
                if detection in self.april_map:
                    continue
                self.april_map[detection] = self.april_counter
                self.april_counter += 1
                lm = np.array([measurement.poses[i].position.z, -measurement.poses[i].position.x, 0])
                rot = Rot.from_quat([measurement.poses[i].orientation.x, measurement.poses[i].orientation.y, measurement.poses[i].orientation.z, measurement.poses[i].orientation.w])
                lm[2] = -rot.as_rotvec()[1]
                lm = robot_mat_inv.dot(lm) + self.state[:3]
                self.state = np.append(self.state, lm)
                self.sigma = np.block([
                    [self.sigma, np.zeros((len(self.state)-3, 3))],
                    [np.zeros((3, len(self.state)-3)), self.covariance]
                ])
        
        self.lock.release()



if __name__ == '__main__':
    rclpy.init()
    
    Q = np.diag([0.1, 0.1, 0.05]) + 1e-6
    Qv = np.diag([0.05, 0.05, 0.025]) + 1e-7
    
    R = np.diag([0.02, 0.02, 0.005]) + 1e-6
    Rv = np.diag([0.01, 0.01, 0.005]) + 1e-7
    
    node = KalmanFilter(Q, R, Qv, Rv)
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()