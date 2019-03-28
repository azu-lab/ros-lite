# ROS-lite
ROS-lite is a static program placement platform for embedded multi-core multi-cluster systems.
It's goal is to provide the means to automatically convert ROS node to application for embedded systems.
It's current target is POSIX system.
It's target dependent code is separated from core code, so you can easily port it to other systems.

ROS-lite consists of following components:
- a lightweight implementation of ROS-like publish/subscribe functions
- ROS bridge function
- code conversion tools

## Requirements
- ROS kinetic (Ubuntu 16.04)

## Build & Run
1. Clone the repository
   ```
   $ git clone https://github.com/azu-lab/ros-lite.git
   ```
1. Set environment variables
   ```
   $ cd ros-lite
   $ source source/appl/roslite/scripts/roslite_cli.sh
   ```
1. Build
   ```
   $ rosl_build -app
   ```
1. Run

   Start five terminals and enter as follow:
  
   ```
   $ roscore
   ```
   ```
   $ cd [ros-lite directory]
   $ ./cluster-1
   ```
   ```
   $ cd [ros-lite directory]
   $ ./cluster-2
   ```
   ```
   $ cd [ros-lite directory]
   $ ./ros_bridge
   ```
   ```
   $ cd [ros-lite directory]
   $ ./ros_param_bridge
   ```

   After starting these programs, following messages will be printed on cluster-1 terminal.
   ```
   hello world
   hello world
   hello world
   ...
   ```
   These messages mean that "talker" node in cluster-1 publishes "hello world" to /chatter topic.

   Meanwhile, following messages will be printed on cluster-2 terminal.
   ```
   I heard: [hello world]
   I heard: [hello world] in listener2
   I heard: [hello world]
   I heard: [hello world] in listener2
   I heard: [hello world]
   I heard: [hello world] in listener2
   ...
   ```
   These messages mean that "listener" and "listner2" node in cluster-2 receive "hello world" from /chatter topic.

   The source codes of these nodes are found in following locations.
   ```
   source/appl/ros_src/nodes/talker/talker.cpp
   source/appl/ros_src/nodes/listener/listener.cpp
   source/appl/ros_src/nodes/listener/listener2.cpp
   ```
   These source codes are valid for ROS node, but these are now working as ROS-lite applications.

   ros_bridge is an application to connect ROS and ROS-bridge transparently.
   Thanks to this application, any ROS node can receive "hello world" from /chatter topic.
   Also, listener and listener2 can receve any /chatter messages published by any ROS node.

   ros_param_bridge is an application to get ROS paramters. In this case, the function of ros_param_bridge is not working because talker, listener, and listener2 does not use ROS parameter.

## How to port ROS-node to ROS-lite

1. Create directories for target ROS nodes in ```source/appl/ros_src/nodes```, and put source codes of these node to these directory.
1. Create node mapping file and put it to ```source/appl/ros_src/map```.
   rosl_create_map will be helpful.

   usage of rosl_create_map:
   ```
   $ cd [ros-lite directory]
   $ source source/appl/roslite/scripts/roslite_cli.sh
   $ rosl_create_map [comma separated node list] -o [map file name]
   ```
1. Generate source codes which are required to run target nodes on ROS-lite.
   ```
   $ cd [ros-lite directory]
   $ source source/appl/roslite/scripts/roslite_cli.sh
   $ rosl_map_gen [map file name]
   ```
1. Put ROS message files required by target nodes in ```source/appl/ros_src/msg```.
   msg files should be placed in directories which has the name same with message name space. (e.g. std_msgs)
1. Generate message header files.
   ```
   $ rosl_msg_gen
   ```
1. Build and run
