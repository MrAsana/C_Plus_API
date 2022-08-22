These are the C++ examples based on the AMBER UDP protocl, the number in the file name is the command number in UDP protocol.

UDP protocol link：https://github.com/MrAsana/AMBER_B1_ROS2/wiki/SDK-&-API---UDP-Ethernet-Protocol--for-controlling-&-programing 

** A example for NRT(Rapidly Random Tree) motion planning of a robotic arm algorithm for tool in the gui_5_node files.

The part of RRT is as below in the demo code.
RRT----RRT::planning
Obstacle avoidance---RRT::collisionCheck2    
Tool End obstacle avoidance---RRT::collisionCheck


** amber_gui_5_node：

1、Go to /amber_gui_5_node/cmake-build-debug/

2、Open a terminal 

3、Run ./amber_gui_5_node, if there is any issues, try the following process：

  3.1 Delete all the files in /amber_gui_5_node/cmake-build-debug/
  
  3.2 cmake ..
  
  3.3 make
  
  3.4 ./amber_gui_5_node
