# desired_motion_rviz_plugin_ros
Rviz Plugins for
* simulation_only_msgs/DeltaTrajectoryWithID and automated_driving_msgs/ObjectStateArray messages or
* automated_driving_msgs/DeltaTrajectory and automated_driving_msgs/MotionState messages

It
* visualizes desired motions of objects by appending the desired delta trajectory (from DeltaTrajectory/WithID) to the current pose of the respective object (from MotionState/ObjectStateArray)
* the desired trajectory is visualized by colored circles, where every color determines a global timestamp

## Installation
* add the package and its dependencies to your workspace
* build it
* source it

## Usage
* start rviz and add the plugin
* use `DesiredMotion` for ObjectStateArray and DeltaTrajectoryWithID-messages (mainly in simulation)
* use `DesiredMotionSingleVehicle` for MotionState and DeltaTrajectory-messages

## Contributors
Pascal Böhmler, Maximilian Naumann, Beija Nigl

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
