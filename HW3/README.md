# Homework 3 - Robot localization
The goal of this homework is to get the estimate position of the robot in the space. The basic assumption we have to make is that we have a complete knowledge of the environment in which the robot will move. 

### Homework 2 needed
This homework is based on the code written in Homework 2.

### To run the exercise
Note: a new terminal tab is required for each point in this list. Always remember to write the command `source devel/setup.bash` (inside the labiagi_ws directory) after opening a new tab.
- Start roscore with `roscore`.
- Run gmapping with `rosrun gmapping slam_gmapping scan:=base_scan`
- Start Stage. `roscd stage_ros`, `cd world` and `rosrun stage_ros stageros willow-erratic.world`. Now, move the robot in the map for a while. It will get the data from the laser scanner and will create a map of the space. The more you go around with the robot, the bigger and more precise the map will be. Note: to move the robot in the map you can use a joystick or the keyboard arrows. In both cases you will need to run the appropriete node in a new terminal tab (ex. `rosrun srrg_joystick_teleop joy_teleop_node`.
- `rosrun map_server map_saver -f [mapName]` to save the map. [mapName] is the filename of the map.
- `rosrun map_server map_server [mapName].yaml` to start a server with the map.
- `rosrun tf static_transform_publisher 0 0 0 0 0 0 /odom /map 10`
- `rosrun srrg_localizer2d_ros srrg_localizer2d_node`
- `rosrun thin_navigation thin_planner_node`

Rviz is now used to visualize the estimate position of the robot. In a new terminal tab, open rviz with the command `rosrun rviz rviz`. In Rviz click on "Add", then on "by topic". Add "map" in "/map", PoseArray in "particlecloud" and "path" in path".
Then click on the "2D Pose Estimate" button and click the point in the map where the robot approximately is (look where the robot is on Stage). Try to be accurate in this step.
Finally, click on "2D Nav Goal" and define a direction that the robot will try to follow. On rviz you will find a cloud of arrow, which represent all the estimate positions of the robot, moving in the defined direction (the exact position will still appear on Stage).
