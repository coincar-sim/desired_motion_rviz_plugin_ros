[![Build Status](https://api.travis-ci.org/coincar-sim/desired_motion_rviz_plugin_ros.svg)](https://travis-ci.org/coincar-sim/desired_motion_rviz_plugin_ros)

# desired_motion_rviz_plugin_ros
Rviz Plugin for simulation_only_msgs/DeltaTrajectoryWithID and automated_driving_msgs/ObjectStateArray messages
* visualizes desired motions of objects by appending the desired delta trajectory (from DeltaTrajectoryWithID) to the current pose of the respective object (from ObjectStateArray)
* the desired trajectory is visualized by colored circles, where every color determines a global timestamp

## Installation
* add the package and its dependencies to your workspace
* build it
* source it

## Usage
* start rviz and add the plugin

## Contributors
Pascal Böhmler, Maximilian Naumann, Beija Nigl

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
