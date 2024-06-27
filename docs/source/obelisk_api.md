# Obelisk API
The Obelisk API defines a consistent set of interfaces to modularize the development of robotics stacks. At the core of Obelisk is ROS2, which provides a Publish-Subscribe (or Pub-Sub) interface for heterogenous nodes to interact with each other. Obelisk provides unified interfaces to simulators, hardware, state estimators, controllers, and other utilities like visualization and logging.

This abstract interface is achieved by defining a set of common [messages](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html) and [topics](https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html). Standardizing these messages and topics within the lab will allow anyone to interface with a robot that is in the Obelisk ecosystem without hassle. Similarily, this will make is easy to test code on any simulator in the ecosystem. Each simulator and robot will need to be brought into the ecosystem through a Obelisk wrapper that will allow it to interface with the Obelisk API. Once a robot is in the ecosystem anyone can use it easily without needing to re-create the entire stack.

As part of accomplishing this, Obelisk defines a standardized "world" interface. The Obelisk wrappers let the simulators or robots expose the world interface that the rest of the robot stack can then interface with. Beyond the convinece of interfacing with other robots, this design choice should make moving from simulation to hardware seamless and increase the chances that a working simulation implies a working robot in the real world.

Beyond unifying the simulation and hardware interface, Obelisk users should be able to use other modules that are designed to fit into the Obelisk API easily. For example, if person A has written a state estimator for a robot, then person B should be able to write a controller that uses those state esimates easily and without modifying the source code for their controller or person A's state estimator. This will allow for more code sharing and collaboration.

Obelisk has been designed to provide these conviences with minimal overhead.

<!-- TODO (@zolkin): Add in a system diagram -->
<!-- TODO (@zolkin): Break this up into multiple files -->
## Configuring Obelisk
Obelisk attempts to be flexible and abstract to meet everyone's needs. This means that for each specific use case we need to configure Obelisk to maximize our efficiency. This can be done through a few configuration files. The configuration files are read in at the start of run time and are not meant to be updated throughout a run.

Obelisk simulator interfaces accept a configuration file to make the simulation match the hardware as best as possible. The possible configuration paramters are given below.
- Robot model
- List of sensors
- Data rates for each sensor
- Configuration of anything hardware bound (e.g. on board PD controller gains)
- Additional noise for each sensor
- Any other objects in the environement (should also be able to be added programatically later.)

Obelisk hardware interfaces accept a configuration file too. The hardware accepts the below paramteres.
- Configuration of anything hardware bound (e.g. on board PD controller gains)

Details on each of these paramters and how to specify them are given at TBD.
<!-- TODO (@zolkin): Add in more information about this -->

## Messages
Below is a list of messages used by Obelisk
- `obelisk_msg/State`
- `obelisk_msg/EstimatedState`
- `obelisk_msg/EstimatedPosition`
- `obelisk_msg/Joints`
- `obelisk_msg/EstimatedJoints`
- `sensor_msg/quat`
- `sensor_msg/Imu`
- `sensor_msg/Image` (is this the one we want?)
- `sensor_msg/PointCloud`
- `sensor_msg/JointState`
- `obelisk_msg/PDFeedForward`
- `obelisk_msg/Torques`

## Topics
Below is the full list of topics used by Obelisk.

Topics relating to states and sensors:
- `/obelisk/Joints` (msg: `obelisk_msg/Joints`)
- `/obelisk/FullState` (msg: `obelisk_msg/State`)
- `/obelisk/IMU` (msg: `sensor_msg/Imu`)
- `/obelisk/Cameras` (msg: `sensor_msg/Image`)
- `/obelisk/Lidars` (msg: `sensor_msg/PointCloud`)
- `/obelisk/Encoders` (msg: `obelisk_msg/EstimatedJoints`)
- `/obelisk/Mocap` (msg: `obelisk_msg/EstimatedPosition`)
- `/obelisk/EstimatedState` (msg: `obelisk_msg/EstimatedState`)

Topics relating to controllers:
- `/obelisk/Torques` (msg: `obelisk_msg/Torques`)
- `/obelisk/PDFeedForward` (msg: `obelisk_msg/PDFeedForward`)
