## Establish ROS Communication with RaspberryPi

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
If you want to control or monitor an additional robot, add it in the peers.
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=10
export ROS_AUTOMATIC_DISCOVERY=SUBNET
export ROS_STATIC_PEERS="10.170.32.181;10.170.32.194;10.170.32.193"
```
You would only need to update ROS_ID if you prefer to have a separate set of network.


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

### 4. ssh 
Secure SHell (SSH) potocol is a cryptographic network protocol used to securely log into and execute commands on remote servers.
You can connect to RaspberryPi from your laptop by below commands. Make sure you are connected to the University VPN.

```bash
ssh dumptruck_02@10.190.32.193
# ssh user_name_of_the_RaspberryPi_you_are_trying_to_login@IPaddress
```
You will be prompt to enter password and you are connected!

To activate ssh on the RaspberryPi you need to run the below commands.
```bash
sudo apt update
sudo apt install openssh-server -y 
sudo systemctl enable ssh
sudo systemctl start ssh
```

### 5. remote ssh 

There is a strong extension in VSCode you can add called "RemoteSSH". This allows you to see, edit, and execute files on the RaspberryPi from your laptop. Once you download remote ssh, open settings on SSH. Then locate to /home/XXX/.ssh/config. Copy below.

```bash
Host dumptruck1
    HostName 10.170.32.192
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host dumptruck2
    HostName 10.170.32.193
    User dumptruck_02
    IdentityFile ~/.ssh/id_ed25519

Host dumptruck3
    HostName 10.170.32.194
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator1
    HostName 10.170.32.182
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator2
    HostName 10.170.32.191
    User besure
    IdentityFile ~/.ssh/id_ed25519
```

You also need to create ssh key so that you don't need to type password everytime.
On your laptop, create ssh key.
```bash
ssh-keygen -t ed25519
```

Then enable ssh key for each RapberryPi by running below commands, again on your laptop.
For RaspberryPi Dumptruck 1:
```bash
ssh-copy-id besure@10.170.32.192
```

For RaspberryPi Dumptruck 2:
```bash
ssh-copy-id dumptruck_02@10.170.32.193
```

For RaspberryPi Dumptruck 3:
```bash
ssh-copy-id besure@10.170.32.194
```

For RaspberryPi Excavator 1:
```bash
ssh-copy-id besure@10.170.32.182
```

For RaspberryPi Excavator 2:
```bash
ssh-copy-id besure@10.170.32.191
```

Once you click the arrow besides the Host name, you are connected through SSH! You can now run commands on RaspberryPi from your laptop.
