# robot_movement.py

import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Function to move the robot a specified distance using odometry
def move_distance(robot, distance, speed=0.2):
    initial_position = None
    odometry_received = False

    def odom_callback(msg):
        nonlocal initial_position, odometry_received
        if initial_position is None:
            initial_position = msg.pose.pose.position
        else:
            current_position = msg.pose.pose.position
            dx = current_position.x - initial_position.x
            dy = current_position.y - initial_position.y
            traveled_distance = math.sqrt(dx**2 + dy**2)
            if traveled_distance >= abs(distance):
                robot.set_cmd_vel(0.0, 0.0, 0)  # Stop the robot
                odometry_received = True

    odom_subscription = robot.create_subscription(Odometry, '/odom', odom_callback, 10)
    linear_speed = speed if distance > 0 else -speed
    robot.set_cmd_vel(linear_speed, 0.0, 0)

    while not odometry_received:
        rclpy.spin_once(robot, timeout_sec=0.1)

    robot.destroy_subscription(odom_subscription)

# Function to rotate the robot a specified angle using IMU data
def rotate_angle(robot, angle, angular_speed=0.5):
    initial_yaw = None
    imu_received = False

    def imu_callback(msg):
        nonlocal initial_yaw, imu_received
        if initial_yaw is None:
            initial_yaw = robot.euler_from_quaternion(msg.orientation)[2]
        else:
            current_yaw = robot.euler_from_quaternion(msg.orientation)[2]
            angle_traveled = abs(current_yaw - initial_yaw)
            if angle_traveled >= abs(math.radians(angle)):
                robot.set_cmd_vel(0.0, 0.0, 0)  # Stop the robot
                imu_received = True

    imu_subscription = robot.create_subscription(Imu, '/imu', imu_callback, 10)
    angular_velocity = angular_speed if angle > 0 else -angular_speed
    robot.set_cmd_vel(0.0, angular_velocity, 0)

    while not imu_received:
        rclpy.spin_once(robot, timeout_sec=0.1)

    robot.destroy_subscription(imu_subscription)
