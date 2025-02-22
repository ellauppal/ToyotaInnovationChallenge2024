import TMMC_Wrapper
import rclpy
import numpy as np
import math

# RUN THIS FILE FOR PURE KEYBOARD CONTROL 

TMMC_Wrapper.is_SIM = True

# start ros
if not rclpy.ok():
    rclpy.init()

#specify hardware api
TMMC_Wrapper.use_hardware()
if not "robot" in globals():
    robot = TMMC_Wrapper.Robot()

# debug messaging 
print("running main")

# start processes
robot.start_keyboard_control()   # this one is just pure keyboard control

rclpy.spin_once(robot, timeout_sec=0.1)

# run the keyboard control functions
try:
    print("Listening for keyboard events. Press keys to test, Ctrl C to exit")
    while True: 
        rclpy.spin_once(robot, timeout_sec=0.1)
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    # when exiting program, run the kill processes
    robot.stop_keyboard_control()
    robot.destroy_node()
    rclpy.shutdown()


