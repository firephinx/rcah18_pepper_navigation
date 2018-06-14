# rcah18_pepper_navigation

## Installation Instructions
1. Download the urg_node, laser_proc, and urg_c ros packages from github, or use the script `26_Hokuyo_LIDAR_ROS_Packages.sh` in the computer_setup scripts repository.
2. Make a bash alias `alias chmodACM0='sudo chmod a+rw /dev/ttyACM0'`
3. Clone the hector_slam ros package in your catkin_ws using the command: `git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git`
4. Copy the mapping.launch file in the rcah18_pepper_navigation/hector_slam/hector_mapping/launch folder to the hector_slam/hector_mapping/launch folder.

## Starting the LIDAR
1. Start roscore in a terminal.
2. Type `chmodACM0` in any terminal.
3. Start the LIDAR with the command `roslaunch urg_node urg_lidar.launch`

## Starting Hector Mapping and Saving a Map
1. Follow the instructions above and start the LIDAR. 
2. Start mapping with the command `roslaunch hector_mapping mapping.launch`
3. Open rviz using `rosrun rviz rviz` to visualize the map hector_mapping is creating by adding a map to the displays and select the topic to be /map.
4. Move the LIDAR and laptop around the arena until you are satisfied by the map displayed in rviz.
5. Run the command `rosrun map_server map_saver` to save the map.
6. Put the map in the maps folder of rcah18_pepper_navigation

## Configuring the launch scripts
1. Configure the `amcl_omni.launch` script to your liking using information located here: `http://wiki.ros.org/amcl`
2. Configure the move base params in the `move_base.launch` file according to your preferences and robot configuration.
3. All the params from the ROS navigation stack are located in the param folder. Google each launch file to learn what each param represents. The instructions for the global navfn parameters are located here: `http://wiki.ros.org/navfn`. The instructions for the dwa local planner parameters are located here: `http://wiki.ros.org/dwa_local_planner`. The instructions for the costmap parameters are located here: `http://wiki.ros.org/costmap_2d`. The tutorial for the ros navigation stack is located here: `http://wiki.ros.org/navigation`. A ROS navigation tuning guide is located here: `http://kaiyuzheng.me/documents/navguide.pdf`.
4. If you are using an xbox 360 controller to tele operate the robot, change the controller type in the `teleop.launch` file to `xbox`. Otherwise the default is to the use a `logitech controller`.

## Running the entire system
1. Ssh onto Pepper and start roscore on Pepper
2. Open a new terminal, type `snao` and enter the command `roslaunch rcah18_pepper_navigation amcl_omni.launch`
3. Open a new terminal, type `snao` and enter the command `roslaunch rcah18_pepper_navigation move_base.launch`
3. Open a new terminal, type `snao` and enter the command `roslaunch rcah18_pepper_navigation remote_machine.launch` with a joystick connected to the desktop.
4. Once rviz finishes loading, localize Pepper inside the map using the 2D Pose Estimate icon at the top of Rviz. Click on its approximate location and drag to show the arrow which represents the orientation.
8. Afterwards, Press the 2 trigger buttons on an XBox Controller or buttons 7 and 8 on a logitech controller to switch from Teleop mode to Planner mode.