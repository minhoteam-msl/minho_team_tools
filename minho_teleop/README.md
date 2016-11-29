##minho_teleop

This program allows a developer to control a robot in real-time, using ROS (connected to robot's ROS Master).
It subscribes robotInfo of the target robot and publishes controlInfo, in order to control the platform.

 - [x] Simple data visualization using ROS
 - [x] Control the robot using keyboard
 - [x] Reset robot's 0º heading
 - [x] Global localization manual reloc
 
Hotkeys:
* T - Turn teleop on or off
* W,A,S,D - Move robot up,left,down and right 
* Q,E - Rotate robot left and right
* ESC - Quit  
* K - Kick ball (time sensitive key press)
* P - Pass ball (time sensitive key press)

Program Usage:   
* Real Robot:
   * rosrun minho\_team\_tools minho_teleop -r \<TARGET_ROBOT\> - Runs with ROS
* Simulated Robot:
   * rosrun minho\_team\_tools minho_teleop -s \<TARGET_ROBOT\> - Runs with ROS
   
\<TARGET_ROBOT\> ranges from 1 to 5.

*Developed by MinhoTeam @2016*
