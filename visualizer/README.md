##visualizer

This program allows a developer to see (graphically) the data sent by a robot in real-time, both using ROS (connected with Robot's ROS Master) and with Multicast Coms, for real and simulated robots.

When in ROS mode, the program subscribes robotInfo/goalKeeperInfo of the target robot and makes the proper display. When in Multicast Coms mode, it received the shared interAgentInfo.

 - [x] Data reception using ROS
 - [x] Data reception using Multicast Coms
 - [x] Run for real and simulated Robots
 
Hotkeys:
* O - Change to Official field  
* L - Change to Lar field  
* ESC - Quit  
* I - Request Extended Debug Information

Program Usage:   
* Real Robot:
   * rosrun minho\_team\_tools visualizer -r \<TARGET_ROBOT\> - Runs with ROS
   * rosrun minho\_team\_tools visualizer -m \<TARGET_ROBOT\> - Runs with Multicast Coms
* Simulated Robot:
   * rosrun minho\_team\_tools visualizer -s \<TARGET_ROBOT\> - Runs with ROS
   * rosrun minho\_team\_tools visualizer -m \<TARGET_ROBOT\> - Runs with Multicast Coms
   
\<TARGET_ROBOT\> ranges from 1 to 5.

*Developed by MinhoTeam @2016*
