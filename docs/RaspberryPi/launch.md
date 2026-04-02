## RaspberryPi

These steps describe how to establish stable ROS 2 communication between a robot (Raspberry Pi) and a Linux desktop computer running Isaac Sim on the PSU IoT network.

### 1. RaspberryPi
Set the ROS domain ID for the robot.
Make sure this matches the ROS domain ID used by Isaac Sim and the desktop ROS terminal.

Specify the IP addresses of the desktop computer and the Raspberry Pi using ROS_STATIC_PEERS. 
In the example below, the IP addresses correspond to the desktop computer and Dumptruck_03.
NOTE: IP address should be registered in the main Linux desktop computer. Please be aware of this if this is a new Raspberry Pi you are trying to use. You will not be able to see ROS topics until you register.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=10
export ROS_AUTOMATIC_DISCOVERY=SUBNET
export ROS_STATIC_PEERS="10.170.32.181;10.170.32.194"
```
Note: export ROS_AUTOMATIC_DISCCOVERY=OFF breaks static peers in our setup


### 2. Desktop computer (ROS terminal)
These commands are for a terminal used to run ROS2 commands (e.g., ros2 topic list, ros2 node list).
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=10
export ROS_AUTOMATIC_DISCOVERY=SUBNET
export ROS_STATIC_PEERS="10.170.32.181;10.170.32.194"
```
If you want to control or monitor a second robot, open a separate terminal and assign a different ROS domain ID.
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=11
export ROS_AUTOMATIC_DISCOVERY=SUBNET
export ROS_STATIC_PEERS="10.170.32.181;10.170.32.194"
```
Each terminal operates in a single ROS domain, so multiple robots require separate terminals.

### 3. Desktop computer (IsaacSIM terminal)
For the Isaac Sim launch terminal:
	•	Do not source ROS
	•	ROS_DOMAIN_ID is assigned inside the .usd file or Isaac Sim configuration
	•	Discovery behavior is controlled using environment variables only

```bash
export ROS_AUTOMATIC_DISCCOVERY=SUBNET
export ROS_STATIC_PEERS="10.170.32.181;10.170.32.194"
/opt/isaacsim/isaac-sim.sh
```


