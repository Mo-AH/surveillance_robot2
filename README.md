
# Surveillance Robot 2 #

**Second Assignment of Experimental Robotic Laboratory - Robotics Engineering - UniGE**

This repository contains the development of a ROS-based simulation of a robot model (a vehicle equipped with a laserscan sensor and a camera arm on top), designed for indoor surveillance operations.
It is the second release of the [Surveillance Robot](https://github.com/Mo-AH/surveillance_robot) simulation.

The full documentation can be found [here](https://Mo-AH.github.io/surveillance_robot2/).

---

# Scenario #


## Environment ##

In this assignment, we have a `world` file with an indoor environment consisting of four rooms and three corridors, as illustrated in the accompanying figure. The robot begins in an additional room connected to corridor `E`, where it finds seven `ArucoMarkers` that contain information about the map. The assignment's environment has been slightly altered, with the boxes on which the markers are located being painted white. This change was made because the marker detection was not performing well due to low contrast when the boxes were black.
It's worth noting that the software has the ability to adapt to other environments as long as it starts from a location that contains information about the rooms coordinates/connections.

![indoor_map](https://user-images.githubusercontent.com/91679281/213928100-e7ce9ddc-b498-4271-a2cb-17d845c98af9.jpg)

The indoor environment is composed of locations and doors.
 - A room is a location with just one door
 - A corridor is a location with 2 or more doors
 - If two locations have the same door, they are connected and the robot can move from one to the other
 - If a room has not been visited for some time (parameter `urgencyThreshold`), it becomes urgent

For the environment representation, it has been used the [topological_map](https://github.com/buoncubi/topological_map) ontology, which was previously created with Protegé. In particular, the [file](https://github.com/Mo-AH/surveillance_robot2/tree/main/ontologies) used in this software is completely without the Abox.

## Assumptions and Behaviour ##

The robot behaviour can be devided into two phases:

 - Phase 1:
    - The robot spawns in his starting location.
    - The robot moves just the arm (that has a camera attached on the end-effector) searching for markers to retrieve the environment informations.
 
 - Phase 2:
    - The robot moves in a new location and checks it by rotating the arm before to visit another location. (This behavior is repeated in a infinite loop).
    - If the battery get low, it leave the task it was doing to reach the charging location and waits some time to recharge.

Moreover, when robot's battery is not low, it should move among locations with 2 basic rules:
 - It should mainly stay on corridors.
 - If a reachable room has not been visited for some time, it becomes `URGENT` and it should be visited.

There are a lot of ways for achieving a surveillance behaviour and a set of assumptions should be made:
 - The environment does not change in time.
 - The robot can move only to locations connected to the current location.
 - The only location that the robot can always reach is the recharging one.
 - Only the rooms can become urgent.
 - The battery can become low at any time.

After having retrieved the informations to build the map, it starts in a loop the phase 2 of the simulation, which can be modeled in a several ways depending on further specifications (e.g. all the locations should be urgent or maybe even the charging location should be be visited normally).
The behaviour implemented in this repository follows the policy described in this pseudocode:

``` 
    # [1] Surveillance policy:
    if there are urgent rooms reachable:
      move to the most urgent
      check the room
    else:
      move to a corridor

    # [0] Battery checking:
    if battery is low:
      move to charging location
      recharge
      start again from [1]

```
Note that, while performing `[1]` , it is always aware of the battery level. Moreover, if the battery is low, the `[0]` algorithm cancels the task it was doing. 

---

## Robot Model ##  

![image](https://user-images.githubusercontent.com/91679281/213946785-63f2050f-9763-42da-8898-2ccdebaf1f15.png)

The model used consist in a simple vehicle with a laser scan and an arm mounted on top, consisting in one rotating base plus two links connected through revolute joints. On the end-effector is mounted a camera which has the aim of detecting markers. 

The arm's movement is managed through the JointStateController interface and the PID values has been tuned manually.

# Software Architecture #


## Components diagram ##
The connections among the nodes are described in the following image.

![component_diagram](https://user-images.githubusercontent.com/91679281/213926931-18e18775-e4e4-4964-840d-c1d41c77f8f5.png)

We can see how the `smach_robot`, which implements the behaviour of the robot using a state machine, is the central entity of the entire architecture and is better described after, in the states diagram.
Taking apart the `ARMOR Server`, which is used by `smach_robot` to update and reason about the ontology, the other nodes simulate specific components of the robot:

 - `robot_state`: node in charge of managing the battery level. It provides a publisher for the battery level (`Bool.msg`) and a service to recharge the battery (`SetBool.srv`).
 
 - `marker_detector`: node in charge of providing the room informations to build the map. It subscribes to the topic `/robot/camera/image_raw` to get the camera images, detect the markers IDs, asks for the embedded information to the `/room_info` service and publishes the room infos in the `/map/rooms` topic. When it has found all the markers, its job is done and so it shutdown. 
 
 - `marker_server`: node that implements a service: it requires the `id` (marker) detected by the robot and it replies with the information about the corresponding room (name of the room, coordinates of the center, connections with other rooms).
 
 - `arm_controller`: node in charge of controlling the robotic arm. Given a `movement_cmd`, it follows the trajectory poses for that movement. For doing so, it implements an Action Server that uses the custom action `ArmControl.action`.

---

## Sequence diagram ##

![sequence_diagram](https://user-images.githubusercontent.com/91679281/213926905-794d1266-1c1d-4766-98ed-dcc29116d9e5.png)

This diagram represents the sequential flow of the architecture. 

In particular, it shows that the nodes always active are `smach_robot`, expectable as it is the central entity that calls the other components, and the `robot_state`, because it always keeps track of the battery level.

Apart from `marker_detector`, which shutdown once finished the trasmission of rooms informations, the other nodes are active only when the `smach_robot` call them:

 - `marker_server`: when the robot has to scan the initial room to retrieve markers info. After that, in principle it could be shutdown as `marker_detector`, given that it won't be used anymore. 

 - `ARMOR Server`: initially, when we are defining the ontology objects and properties (Abox); later after every location change of the robot and to query about urgent/reachable rooms.

 - `arm_controller`: when the robot has to scan the initial room and when it has to check a room.


---

## States diagram ##
![states_diagram drawio](https://user-images.githubusercontent.com/91679281/213926872-ca0e2ce9-e40f-4638-b895-a699765aa7ea.png)

In this diagram is shown the robot behaviour, implemented through a state machine in the `smach_robot` node. The node relies on the use of `smach_helper` module, which decouples the state machine interface from the computations processes.

After having built the map in the `BUILD MAP` state, it passes to the loop of the Phase 2:
 - `REASONER` : queries the ontology about reachable and urgent locations to decide the next location. If the battery is low, the next location is always the charging one.
 - `MOVE`: it moves the robot to the target location. When it has reached the location, it uploads the ontology. 
  When the motion is finished (i.e. it reached the target location) by the `MOVE` node, there are 3 possible transitions:
    1. `location_urgent_reached`: the location should be checked and so it passes to the `CHECK_LOCATION` state.
    2. `location_not_urgent_reached`: the location shouldn't be checked so it passes to the `REASONER` state to decide next location.
    3. `charging_location_reached`: the robot has reached the charging location so it passes to the `CHARGE` state to recharge the battery.
 - `CHECK_LOCATION`: it checks the room by doing a full rotation of the arm base.
 - `CHARGE`: it simply simulate the battery charging by wasting time. 
  
 Note that, except for the `CHARGE` state, all other states pass to the `REASONER` when the battery is low (`battery_low` transition), leaving their task uncompleted.

---

# Repository Structure #

## Package files ##

This repository contains a ROS package named `surveillance_robot` that, besides this README, includes the following resources.
 - [CMakeList.txt](CMakeList.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.
 - [setup.py](setup.py): File to `import` python modules from the `utilities` folder into the 
   files in the `script` folder. 
 - [launcher/](launcher/): Contains the configurations to launch this package.
    - [armor.launch](launcher/armor.launch): It launches the Armor server.
    - [gazebo_rviz.launch](launcher/gazebo_rviz.launch): It launches the simulation with the robot model both in rviz and in gazebo.
    - [gmapping_movebase.launch](launcher/gmapping_movebase.launch): It launches the SLAM Gmapping and the MoveBase nodes.
    - [assignment.launch](launcher/random_battery.launch): It launches this package with a random-based change of battery state.
 - [msg/](msg/): It contains the custom message exchanged through ROS topics.
    - [Room.msg](msg/Room.msg): It represents the informations about the room. (name, coordinates, connections)
    - [RoomConnection.msg](msg/RoomConnection.msg): It is the message representing a connection among a door and a location.
 - [srv/](srv/): It Contains the definition of each server used by this software.
    - [RoomInformation.srv](srv/RoomInformation.srv): It defines the request and response to get the room information given a marker id. 
 - [action/](action/): It contains the definition of each action server used by this software.
    - [ArmControl.action](action/Control.action): It defines the goal, feedback and results 
      concerning arm motion controlling.
 - [scripts/](scripts/): It contains the implementation of the python nodes.
    - [arm_controller.py](scripts/map_builder.py): It implements the action server to move the arm in some predefined trajectory.
    - [smach_robot.py](scripts/gesture.py): It implements the state machine of the robot.
    - [robot_state.py](scripts/robot_state.py): It implements the battery manager of the robot.
 - [src/](src/): It contains the implementation of the c++ nodes.
    - [marker_detector.cpp](src/marker_detector.cpp): It implements the node to detect markers.
    - [marker_server.cpp](src/marker_server.cpp): It implements the service to get markers informations.
 - [utilities/surveillance_robot2/](utilities/surveillance_robot2/): It contains auxiliary python files, which are exploited by the files in the `scripts` folder.
    - [architecture_name_mapper.py](scripts/architecture_name_mapper.py): It contains the name 
      of some *node*, *topic*, *server*, *actions* and *parameters* used in this architecture.
    - [action_client_helper.py](scripts/action_client_helper.py): It defines a class to simplify the interaction with an action server.
    - [smach_helper.py](scripts/smach_helper.py): It defines a class to simplify state machine computations.
 - [worlds/](world/): It contains the assignment world.
 - [urdf/](urdf/): It contains the robot model.
 - [docs/](docs/): It contains the documentation of the repository.
 - [ontologies/](ontologies/): It contains the ontology used in this software.
 - [config/](config/): It contains the motors and rviz configuration files.
 - [param/](param/): It contains the files to configure MoveBase.

## Further specifications on ROS Custom Messages, Services and Actions ##
For the development of the simulation, some custom `msg`, `srv` and `action` have been created:
 - `Room.msg`: it represents a room, with name, coordinates and connections.
 - `RoomConnection.msg`: it represents a connection between a door and a location and is composed of two `string` objects.
 - `RoomInformations.srv`: a service to get the rooms information embedded in a marker ID.
 - `ArmControl.action`: an action to interact with the `arm_controller` action server.
   - *goal*: the movement command among some predefined.
   - *result*: the final pose reached by the arm.
   - *feedback*: the current pose reached by the arm.

## Dependencies ##

This software is developed with a [ROS Noetic](http://wiki.ros.org/noetic) environment and you need to have a ROS workspace initialized in order to run the simulation. Moreover you should have installed:
  - [roslaunch](http://wiki.ros.org/roslaunch), to launch the package.
  - [rospy](http://wiki.ros.org/rospy), to use python with ROS.
  - [actionlib](http://wiki.ros.org/actionlib/DetailedDescription), to define
[SimpleActionServer](http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a2013e3b4a6a3cb0b77bb31403e26f137) and use [SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html).
  - [ARMOR Server](https://github.com/EmaroLab/armor), a ROS integration to manipulate online OWL ontologies, which can be installed by following the instructions in the README.
  - [armor_py_api](https://github.com/EmaroLab/armor_py_api), a library to simplify the python calls to the ARMOR Server, which can be installed by following the instructions in the README.
  - [xterm](https://wiki.archlinux.org/title/Xterm), a terminal simulator, which can be installed by running from the terminal `sudo apt install -y xterm`.
  - [smach](http://wiki.ros.org/smach), a state machine library to simulate the robot behaviour, which can be installed by running from the terminal `sudo apt-get install ros-noetic-smach-ros`
  - [rviz](http://wiki.ros.org/rviz), a robotic visualization tool.
  - [gazebo](https://gazebosim.org/), a simulation environment.
  - [OpenCV](http://wiki.ros.org/vision_opencv), a library to elaborate images.
  - [move_base](http://wiki.ros.org/move_base), to control the robot motion.
  - [gmapping](http://wiki.ros.org/gmapping), implements the SLAM algorithm to build the map based on the laser scan.


---

# Simulation #

## How to Run ##

Once assured that all dependecies are installed, follow those steps:
  1. In the `src` folder of your ROS workspace, clone this repository by running `git clone https://github.com/Mo-AH/surveillance_robot2`
  2. Move first to `src/surveillance_robot2/scripts` and then to `src/surveillance_robot2/utilities/surveillance_robot2` and run a `chmod +x <script_name>.py` for each Python module in both folders.
  3. Build the ROS workspace by running `catkin_make` in its root folder.
  4. Launch the simulation:
      - In a terminal, launch the ARMOR server by typing `roslaunch surveillance_robot2 armor.launch`.
      - In another terminal, launch all the package components by running `roslaunch surveillance_robot2 assignment.launch`.

***Note: This is due to a problem with Armor, which doesn't work well if launched together with the other components***

---

## Parameters ##

There are some parameters that are setted by default, but they can be changed to meet some specification:

  - `test/random_sense/active`:  It is a boolean value that defines the battery mode: True for randomly change the state, False to change the state manually.
  
  -  `test/random_sense/battery_time`: It indicates the time passed within battery state changes 
   (i.e., low/high). It should be a list of two float numbers, i.e., `[min_time, max_time]` in 
   seconds, and the time passed between changes in battery levels will be a random value within 
   such an interval.
  
  - `test/charging_time`:  It indicates the time required to recharge the battery, should be a float number.

  - `map/total_markers`: It indicates the total markers to be found in the initial room.

Other parameters regarding the ontology, such as starting location, charging location or connections list, can be changed directly in the `architecture_name_mapper.py` module.

---

## Running code ##

[# VIDEO #](https://user-images.githubusercontent.com/91679281/213932898-782070d3-dc29-408f-b0a0-107b98f5afa3.mp4)

In the video, there is the demonstration of the running code. It shows the initial phase in which the robot does a complete scan of the room, detecting markers, for building the map informations. After that, it starts moving following the surveillance policy. On the rviz window, is shown the map building process of the SLAM Gmapping and the MoveBase working to reach a goal.



---

## Possible improvements ##

- Making the robot aware of the urgents rooms not adjacent
- When the battery is low, it may be more appropriate pass directly to the move state with the charging location as target, instead of passing by the reasoner.
- Implement a real battery management, so that it runs out proportionally to the number of motions and to the time passed.
- Implement the possibility of having multiple robots that cooperate together for the surveillance purpose.
- When interrupting a task because of the battery low, memorize and continue it after having recharged the battery.
- Use the moveit package to control the robotic arm, instead of publishing the poses on the `joint_state_controller/command` topic.
- Tune better the PID parameters of the joints controllers


---

***Author***: Mohammad Al Horany

***Email***: s5271212@studenti.unige.it
