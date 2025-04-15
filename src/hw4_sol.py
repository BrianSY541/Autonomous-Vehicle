from mpi_control import MegaPiController
import rclpy
from rclpy.node import Node
import numpy as np
import time
from geometry_msgs.msg import PoseArray, Twist
from PIL import Image as im
import matplotlib.pyplot as plt
from pathPlanner import PathPlanner

tag2world_dict = {
    "marker_0": [np.array([
        [  0,  0,  1, 1.17],
        [ -1,  0,  0,    0],
        [  0, -1,  0,  0.2],
        [  0,  0,  0,    1]
    ]),  
    # np.array([
    #     [ -1,  0,  0, 0.31],
    #     [  0,  0, -1,  0.2],
    #     [  0, -1,  0,  0.2],
    #     [  0,  0,  0,    1]
    # ]),
    ],
    "marker_1": [np.array([
        [  0,  0,  1, 1.17],
        [ -1,  0,  0, 0.65],
        [  0, -1,  0,  0.2],
        [  0,  0,  0,    1]
    ]),   np.array([
        [ 0,  0, -1, 0.455],
        [ 1,  0,  0,  0.37],
        [ 0, -1,  0,   0.2],
        [ 0,  0,  0,     1]
    ]),
    ],
    "marker_2": [np.array([
        [ 1,  0,  0,  0.57],
        [ 0,  0,  1, 1.065],
        [ 0, -1,  0,   0.2],
        [ 0,  0,  0,     1]
    ]),   np.array([
        [ -1,  0,  0, 0.31],
        [  0,  0, -1,  0.5],
        [  0, -1,  0,  0.2],
        [  0,  0,  0,    1]
    ]),
    ],
    "marker_3": [np.array([
        [ 1,  0,  0,      0],
        [ 0,  0,  1,  1.065],
        [ 0,  -1, 0,   0.25],
        [ 0,  0,  0,      1]
    ])],
    "marker_4": [np.array([
        [ 0,  0, -1, -0.65],
        [ 1,  0,  0, 0.615],
        [ 0, -1,  0,   0.2],
        [ 0,  0,  0,     1]
    ])],
    
    "marker_5": [np.array([
        [ 0,  0, -1, -0.65],
        [ 1,  0,  0,     0],
        [ 0, -1,  0,   0.2],
        [ 0,  0,  0,     1]
    ])],
    "marker_6": [np.array([
        [ -1,  0,  0,    0],
        [  0,  0, -1, -0.5],
        [  0, -1,  0,  0.2],
        [  0,  0,  0,    1]
    ])],
    "marker_7": [np.array([
        [ -1,  0,  0,  0.54],
        [  0,  0, -1,  -0.5],
        [  0, -1,  0,   0.2],
        [  0,  0,  0,     1]
    ])],
    "marker_8": [np.array([
        [  0,  0,  1, 0.145],
        [ -1,  0,  0,  0.36],
        [  0, -1,  0,   0.2],
        [  0,  0,  0,     1]
    ])],
}

class Navigation(Node):
    """
    This is the class of a navigation unit
    """
    def __init__(self, mpi_controler: MegaPiController, waypoints, direction = 0.0, position = [0.0, 0.0]):
        super().__init__('navigation_node')

        self.mpi_ctrl = mpi_controler
        self.direction = direction # rad
        self.position = position   # meter
        self.waypoints = waypoints
        self.waypoint = waypoints.pop(0)
        self.progress = None
        self.kinematic = self.cal_kinematic()
        self.cam2car = self.cal_camera2car_transform()
        # print(f"cam2car:{self.cam2car}")
        self.image_timer = self.get_clock().now()
        self.pos_subscription = self.create_subscription(PoseArray, '/april_poses', self.landmark_callback, 10)
        # prevent unused variable warning.
        self.create_timer(0.5, self.step)
        self.create_timer(0.5, self.record_pos)
        self.last_update_time = self.get_clock().now()
        self.reach_pos = False
        self.record_poses = []
    
    def draw_recorded_pos(self) -> None:
        data = np.array(self.record_poses)
        fig = plt.figure(figsize=(10, 8))
        plt.title("car's trajectory")
        plt.scatter(data[:, 0], data[:, 1])
        plt.axis('equal')
        plt.xlabel('x  (m)')
        plt.ylabel('y  (m)')
        plt.grid()
        plt.savefig('trajectory.png')

    def record_pos(self) -> None:
        """
        This function will record the current position.
        """
        self.record_poses.append(self.position)

    def landmark_callback(self, msg) -> None:

        possible_poses = []
        possible_directions = []
        if len(msg.poses) < 1:
            return
            
        pose_ids = msg.header.frame_id.split(',')[:-1]
        print(f"detect {len(pose_ids)} landmark.")
        possible_poses = []
        for id, pose in zip(pose_ids, msg.poses):
            tag2camera = self.get_transform_from_april_poses_topic(pose.position, pose.orientation)
            if id != 'marker_0': continue
            for tag2world in tag2world_dict[id]:
                car2world = np.linalg.inv(self.cam2car @ tag2camera @ np.linalg.inv(tag2world))
                possible_poses.append(car2world[0:2, -1])
                direction = (car2world[0:3, 0:3] @ np.array([1, 0, 0]).T)[0:2]
                possible_directions.append(np.arctan2(direction[1], direction[0]))
                print(f"{id}, {car2world[0:3, -1]}, {np.arctan2(direction[1], direction[0])}")
        possible_poses = np.array(possible_poses)
        possible_directions = np.array(possible_directions)
        pos_error = np.linalg.norm(possible_poses - self.position, axis = 1)

        self.position = np.mean(possible_poses[pos_error < 0.5], axis = 0)
        self.direction = np.mean(possible_directions[pos_error < 0.5])
    
    def calc_direction(self, vector: np.ndarray) -> float:
        return (np.arctan2(vector[1], vector[0]))
    
    def get_transform_from_april_poses_topic(self, pos: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
        """
        This function return a transform matrix from the format of april poses topic
        Input: 
            w = quaternion[0]  # w
            x = quaternion[1]  # x
            y = quaternion[2]  # y
            z = quaternion[3]  # z
        """
        if pos.__class__ == np.ndarray and quaternion.__class__ == np.ndarray:
            w = quaternion[0]  # w
            x = quaternion[1]  # x
            y = quaternion[2]  # y
            z = quaternion[3]  # z
        
            # 3x3 rotation matrix
            _R = np.array([
                [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y, pos[0]],
                [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x, pos[1]],
                [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2, pos[2]],
                [0, 0, 0, 1]
            ])
        else:
            w = quaternion.w
            x = quaternion.x
            y = quaternion.y
            z = quaternion.z
            _R = np.array([
                [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y, pos.x],
                [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x, pos.y],
                [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2, pos.z],
                [0, 0, 0, 1]
            ])
                      
        return _R

    def genTwistMsg(desired_twist):
        """
        Convert the twist to twist msg.
        """
        twist_msg = Twist()
        twist_msg.linear.x = desired_twist[0] 
        twist_msg.linear.y = desired_twist[1] 
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = desired_twist[2]
        return twist_msg

    def cal_camera2car_transform(self) -> np.ndarray:
        """
        This function calculate the transformation from camera's frame to car's frame.
        That is, $T^{\text{car}}_{\text{camera}}$
        """
        _tag2car = np.array([
            [  0,  0,  1, 1.17],
            [ -1,  0,  0,    0],
            [  0, -1,  0,  0.2],
            [  0,  0,  0,    1]
        ])

        # Calculate the transform matrix from camara to april tag
        _calibrate_pos = np.array([0.001, 0.5, 1.15])
        _calibrate_quaternion = np.array([0.99666, 0,  -0.0816, 0])
        _tag2camera = self.get_transform_from_april_poses_topic(_calibrate_pos, _calibrate_quaternion)
        print(f"tag2cam: {_tag2camera}")
        _camera2car = _tag2car @ np.linalg.inv(_tag2camera)
        return _camera2car
    
    def cal_kinematic(self) -> np.ndarray:
        """
        This function uses [v_x, v_y, omega_z] and their corresponding motor omega to estimate the kinematic model.
        """
        data = np.array([
            [0.17, 0, 0],
            [0, 0.155, 0],
            #[0.093, 0.08, 0],
            [0, 0, 0.25*np.pi / 1.05],
            # [0, 0, np.pi / 3 / 1.32],
            # [0, 0, 0.25*np.pi / 1.]
        ]).T
        motor_signal = np.array([
            [ 50, 50,  50,  50],
            [-48, 59,  60, -57],
            #[75, 0,  0, 75],
            [-50, 50, -50,  50],
            # [-50, 50, -50,  50],
            # [-50, 50, -50,  50],
        ]).T
        # print(data.shape)
        # print(np.matmul(motor_signal, np.matmul(data.T, np.linalg.inv(np.matmul(data, data.T)))).shape)
        assert data.shape[1] == motor_signal.shape[1], "size don't match."
        if data.shape[1] == 3:
            kinematic_model = np.matmul(motor_signal, np.linalg.inv(data))
        elif data.shape[1] > 3:
            kinematic_model = np.matmul(motor_signal, np.matmul(data.T, np.linalg.inv(np.matmul(data, data.T))))
        else:
            raise("Don't have enough data to calculate the kinematic model")
        # print(kinematic_model)
        return kinematic_model

    def cal_rotate_M(self):
        return np.array([
            [ np.cos(self.direction), np.sin(self.direction)],
            [-np.sin(self.direction), np.cos(self.direction)]
        ])
    
    def world2car(self):
        return np.array([
            [ np.cos(self.direction), np.sin(self.direction), 0],
            [-np.sin(self.direction), np.cos(self.direction), 0],
            [                      0,                      0, 1]
        ])

    def step(self, step_time = 0.3):
        """
        Step a little distance toward the waypoint
        """
        # print(f"next: {self.waypoint}")
        # print(f"pose: {self.position}")
        # print(f"dire: {self.direction / np.pi * 180}")
        # print(f"last update {(self.get_clock().now() - self.last_update_time).nanoseconds /1e9} sec ago.")
        # print(f"error: {np.array(self.waypoint) - np.array([*self.position, self.direction])}")
        # return
        if (self.get_clock().now() - self.last_update_time).nanoseconds  > (5e9):
            # rotate
            # print(f"not update for {(self.get_clock().now() - self.last_update_time).nanoseconds / 1e9}")
            motor_signal = np.matmul(self.kinematic, np.array([0, 0, np.pi / 3]).T)
            # self.mpi_ctrl.setFourMotors(
            #     int(motor_signal[0]),
            #     int(motor_signal[1]),
            #     int(motor_signal[2]),
            #     int(motor_signal[3])
            # )
            time.sleep(1.0)
            # self.mpi_ctrl.carStop()
            self.direction += np.pi / 2.

        else:
            if np.linalg.norm(np.array(self.waypoint[0:2]) - self.position) > 0.15 and not self.reach_pos:
                velocity = 0.18
                # velocity = 0.20
                error = (self.world2car() @ (np.array(self.waypoint) - np.array([*self.position, self.direction])))
                v_step = error[0:2] / np.linalg.norm(error[0:2]) * velocity
                ratio = np.abs(v_step[0] / v_step[1])
                # if ratio >= 2:
                #     v_step[1] = 0
                # elif ratio < (1/2):
                #     v_step[0] = 0
                # else:
                #     v_step = np.array([1 / 2**0.5, 1 / 2**0.5]) * velocity
                # motor_signal = np.matmul(self.kinematic, np.array([*v_step, omega]).T)
                motor_signal = np.matmul(self.kinematic, np.array([*v_step, 0]).T)
                # self.mpi_ctrl.setFourMotors(
                #     int(motor_signal[0]),
                #     int(motor_signal[1]),
                #     int(motor_signal[2]),
                #     int(motor_signal[3])
                # )
                time.sleep(step_time)
                # self.mpi_ctrl.carStop()

                # update position and direction
                self.position += v_step*step_time
            else:
                # print("[reach position], start rotate.")
                self.reach_pos = True
                tolerance_angle = 10
                angle_error = (self.waypoint[2] - self.direction) / np.pi * 180
                if np.abs(angle_error) > tolerance_angle:
                    if angle_error > 0:
                        omega = np.pi / 3
                    else:
                        omega = -np.pi / 3

                    motor_signal = self.kinematic @ np.array([0, 0, omega]).T
                    # self.mpi_ctrl.setFourMotors(
                    #     int(motor_signal[0]),
                    #     int(motor_signal[1]),
                    #     int(motor_signal[2]),
                    #     int(motor_signal[3])
                    # )
                    time.sleep(0.1)
                    self.mpi_ctrl.carStop()
                    self.direction += omega*0.1
                    self.direction %= (2*np.pi)
                    if self.direction > np.pi:
                        self.direction -= 2*np.pi
                    elif self.direction < -np.pi:
                        self.direction += 2*np.pi
                    else:
                        pass

                else:
                    # print('[reach waypoint]' + '='*20)
                    if self.waypoints.__len__() == 0:
                        print('Terminate')
                        self.draw_recorded_pos()
                        self.mpi_ctrl.carStop()
                        self.mpi_ctrl.close()
                        # self.destroy_node()
                        # rclpy.shutdown()
                    else:
                        self.waypoint = self.waypoints.pop(0)
                        self.reach_pos = False
                    
        return

if __name__ == "__main__":

    brushfire = PathPlanner(map_file="/root/cse276a_ws/src/rb5_ros2/rb5_ros2_control/src/map_files/map_file_test.png", map_scale=0.3 / 160)
    # Compute the Brushfire distance grid
    distance_grid = brushfire.get_distance_grid()
    # Plot the result
    # brushfire.plot_grid(distance_grid, title="Brushfire Distance Grid")

    # Find the path
    start = np.array([350, 711])  # (y, x)
    goal = np.array([796, 1100])
    car_size = 0.2 # m
    path = np.array(brushfire.bfs_path(distance_grid>(car_size/brushfire.map_scale), start, goal)) * brushfire.map_scale
    waypoints = (path - path[0])[::5]
    # Plot the path
    # brushfire.plot_path(path, title="Path")
    print("path generated")

    rclpy.init()
    mpi_controler = MegaPiController(port='/dev/ttyUSB0', verbose=True)
    time.sleep(1)
    navigation = Navigation(mpi_controler, waypoints)
    rclpy.spin(navigation)
    # print('start')
    # navigation.start(verbose = True)

    # navigation.mpi_ctrl.carStop()
    # navigation.mpi_ctrl.close()
    # navigation.destroy_node()
    # rclpy.shutdown()