import TMMC_Wrapper
import rclpy
import numpy as np
import math
import time
import rclpy.timer

TMMC_Wrapper.is_SIM = False

# Start ros with initializing the rclpy object
if not rclpy.ok():
    rclpy.init()

# Specify hardware api
TMMC_Wrapper.use_hardware()
if not "robot" in globals():
    robot = TMMC_Wrapper.Robot()

print("running main")

# robot.start_keyboard_control()  # comment if no keyboard control

# rclpy,spin_once is a function that updates the ros topics once
rclpy.spin_once(robot, timeout_sec=0.1)

# run control functions on loop
try:
    runRobot = True
    print("Entering the robot loop which cycles until the srcipt is stopped")

    if robot.checkBattery() < 10:
        print("Battery is low")
        robot.stop_keyboard_control()
        robot.dock()
        runRobot = False
    
    # track turns made on course
    turnCount = 0
    turn1 = False
    turn2 = False
    turn3 = False
    turn4 = False

    while runRobot:
        # rclpy,spin_once is a function that updates the ros topics once
        rclpy.spin_once(robot, timeout_sec=0.1)

        # ---------------- FUNCTIONAILITY HERE -----------------
        
        # get frame
        robot.checkImageRelease()
        image_for_cv = robot.rosImg_to_cv2()

        # ------------------ Check for stop sign ------------------

        blk_and_white_img = robot.red_filter(image_for_cv)
        contour, max_area, (c_X,c_Y) = robot.add_contour(blk_and_white_img)

        if (c_X, c_Y) != (-1,-1):
            print(max_area)
            # only stop the robot if it hasn't already stopped at this stop sign
            if max_area in range(6000, 9500) and not has_stopped_at_stop_sign:
                # robot.stop_keyboard_control()  # comment if no keyboard control
                print("Stopping for 5 seconds at stop sign")
                time.sleep(5)
                print("Started keyboard control")
                # robot.start_keyboard_control()  # comment if no keyboard control

                # set the flag to true after stopping
                has_stopped_at_stop_sign = True

        # If the robot is not at the stop sign, reset the flag
        elif max_area not in range(7500, 9500):
            print("Resetting flag")
            has_stopped_at_stop_sign = False
        
        # ------------  Stop sign check complete --------------
        
        # --------------- Continue on course ------------------

        # use april tags to determine direction to move
        info = robot.detect_april_tag_from_img(image_for_cv)
        print(info)
        if info != {}:
            # extract april tag info
            getTagNum = list(info.keys())[0]
            getDist = list(info.values())[0][0]

            # check scan
            msg = robot.checkScan()

            # course turn 1
            if getTagNum == 1:
                lidar_dist, lidar_angle = robot.detect_obstacle(msg.ranges, 0.6, 45)
                if (lidar_dist,lidar_angle) != (-1,-1):
                    robot.rotate(90, -1)
                    turn1 = True
            # course turn 2
            if getTagNum == 2 and turn2 == False and turn1 == True:
                lidar_dist, lidar_angle = robot.detect_obstacle(msg.ranges, 0.5, 45)
                if (lidar_dist,lidar_angle) != (-1,-1):
                    robot.rotate(45, -1)
                    turn2 = True

            elif getTagNum == 9 and turn2 == False and turn1 == True:
                lidar_dist, lidar_angle = robot.detect_obstacle(msg.ranges, 0.5, 45)
                if (lidar_dist,lidar_angle) != (-1,-1):
                    robot.rotate(45, -1)
                    turn2 = True
            # course turn 3
            elif getTagNum == 3 and turn3 == False and turn2 == True:
                lidar_dist, lidar_angle = robot.detect_obstacle(msg.ranges, 0.5, 45)
                if (lidar_dist,lidar_angle) != (-1,-1):
                    robot.rotate(45, -1)
                    turn3 = True

            elif getTagNum == 5 and turn3 == False and turn2 == True:
                lidar_dist, lidar_angle = robot.detect_obstacle(msg.ranges, 0.8, 45)
                if (lidar_dist,lidar_angle) != (-1,-1):
                    robot.rotate(120, -1)
                    turn3 = True
            # course turn 4
            elif getTagNum == 4 and turn4 == False and turn3 == True:
                lidar_dist, lidar_angle = robot.detect_obstacle(msg.ranges, 0.6, 45)
                if (lidar_dist,lidar_angle) != (-1,-1):
                    robot.rotate(90, -1)
                    turn4 = True
        
        # general obstacle detection (close range, wide view)
        lidar_dist, lidar_angle = robot.detect_obstacle(msg.ranges, 0.25, 90)
        # rotate by 5 degrees until no obstacle in path
        while(lidar_dist,lidar_angle) != (-1,-1):
                msg = robot.checkScan()
                lidar_dist, lidar_angle = robot.detect_obstacle(msg.ranges, 0.25, 90)
                robot.rotate(5, -1)

        # proceed on course
        robot.move_forward()

except KeyboardInterrupt:
    print("Keyboard interrupt receieved. Stopping...")
finally:
    # when exiting program, run the kill processes
    robot.destroy_node()
    rclpy.shutdown()
