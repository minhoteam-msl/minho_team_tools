##vision_calib

This program allows a developer to calibrate robot's vision system from its computer using ROS.
It allows to calibrate LUT, World Mapping, Camera parameters and other variables.
Also, it is used as a live-stream viewer for the robot, being able to ask for different images from the robot.

 - [x] Calibration of HSV Look Up Table
 - [x] Calibration of image parameters
 - [x] Calibration of world mapping (image-to-world mirror mapping)
 - [ ] Camera parameters
 - [ ] Auto-camera calibration control parameters
 - [x] Real time image live-feed for raw, segmented and informative image
 
Hotkeys:
* T - Turn calibration on or off (binary view of a label) 
* R - Request RAW image
* S - Request SEGMENTED image
* I - Request INTEREST POINTS image (identification of image points)
* M - Request WORLD MODEL image (identification of lines, balls and obstacles)
* G - Grab single image
* F - Live-feed mode
* I - Increase live-feed frequency
* D - Decrease live-feed frequency

Program Usage:   
* Real Robot:
   * rosrun minho\_team\_tools vision_calib -r \<TARGET_ROBOT\> - Runs with ROS
* Simulated Robot:
   * rosrun minho\_team\_tools vision_calib -s \<TARGET_ROBOT\> - Runs with ROS (currently not working)
   
\<TARGET_ROBOT\> ranges from 1 to 5 for other robots. When target is 0, it runs on local machine.

*Developed by MinhoTeam @2016*
