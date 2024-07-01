# Toyota Innovation Challenge 2024
TurtleBot software capable of navigating through a course designed to imitate a manufacturing environment, featuring collision and stop sign detection.

See boiler_plate.py for main code.

## Project Description
-	Obstacle detection leverages a LiDAR sensor to provide real-time 3D mapping of the environment
-	For stop sign detection, the TurtleBot employs an RGB-D camera and OpenCV, applying a red filter and contour detection techniques 
-	Path traversal is facilitated using AprilTags for localization, while IMU sensor data ensures precise movement along the designated course.

![Picture2](https://github.com/ellauppal/ToyotaInnovationChallenge2024/assets/89555654/3b809836-aec0-4d40-b2c5-607195ceda1e)
<img width="345" alt="Picture1" src="https://github.com/ellauppal/ToyotaInnovationChallenge2024/assets/89555654/1f5e74bb-a8ae-457c-b07d-19f0414ec9ea">
