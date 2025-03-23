# Mastering ROS Essential

- [Exercise 1: ROS Development Setup](#exercise-1-ros-development-setup)
- [Exercise 2: Simulate Robot in ROS](#exercise-2-simulate-robot-in-ros)
  - [Optional 1: differential drive](#optional-1-differential-drive)
  - [Optional 2: lidar and imu](#optional-2-lidar-and-imu)
- [Exercise 3: Raspberry Pi Turtlebot3 Setup](#exercise-3-raspberry-pi-turtlebot3-setup)
  - [SLAM](#slam)
  - [Navigation](#navigation)
- [Exercise 4: Robot Control](#exercise-4-robot-control)
  - [Autonomous Exploring](#autonomous-exploring)
  - [Send Position data to IoT Cloud](#send-position-data-to-iot-cloud)
  - [Control turtlebot3 using hand gesture](#control-turtlebot3-using-hand-gesture)


## Exercise 1: ROS Development Setup
You need to setup a virtual box and install ROS for your robot development
1. Go to Virtual box website and download the application, install in your laptop.
```
https://www.virtualbox.org/wiki/Downloads
```
![Virtual Box](https://github.com/twming/ros2_master_tutorial/blob/main/img/virtualbox.png)

<img src="https://github.com/twming/ros2_master_tutorial/blob/main/img/virtualbox.png" alt="virtual box" width="600">

2. Setup Ubuntu 22.04 in your virtual box
```
https://ubuntu.com/download/server/thank-you?version=22.04.5&architecture=amd64&lts=true
```
![Ubuntu](https://github.com/twming/ros2_master_tutorial/blob/main/img/ubuntu.png)

3. Install ROS Humble in your Ubuntu
```
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```
![ROS](https://github.com/twming/ros2_master_tutorial/blob/main/img/ros.png)

4. Install ROS packages
```
sudo apt install -y ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-xacro
```

> [!TIP]
> How to verify your ROS installation is working?


## Exercise 2: Simulate Robot in ROS
You learn how to describe robot in URDF, create a URDF for below robots and launch it in ROS simulation.
1. Create "autocar_description" package, create "urdf" folder, then add "autocar.xacro" file to the folder.
2. Define the link of robot, including a box (body) and two cylinders (wheel)
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="autocar">
    <!-- Material Color and Definition -->
    <material name="blue"><color rgba="0.0 0.0 0.8 1.0"/></material>
    <material name="green"><color rgba="0.0 1.0 0.0 1.0"/></material>
    <material name="red"><color rgba="0.8 0.0 0.0 1.0"/></material>

    <!-- Propoerty parameters -->
    <xacro:property name="size_ratio" value="___TODO___" />
    <xacro:property name="base_length" value="___TODO___" />
    <xacro:property name="base_width" value="___TODO___" />
    <xacro:property name="base_height" value="___TODO___" />
    <xacro:property name="wheel_radius" value="___TODO___" />
    <xacro:property name="wheel_length" value="___TODO___" />

    <link name="base_footprint_link" />

    <link name="base_link">
        <visual>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}" />
            </geometry>
            <origin xyz="0 0 ${base_height/2.0}" rpy="0 0 0" />
            <material name="green" />
        </visual>
        <collision>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}" />
            </geometry>
            <origin xyz="0 0 ${base_height/2.0}" rpy="0 0 0" />
        </collision>
    </link>

    <link name="right_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}" />
            </geometry>
            <origin xyz="0 0 0" rpy="${-pi/2.0} 0 0" />
            <material name="blue" />
        </visual>
        <collision>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}" />
            </geometry>
            <origin xyz="0 0 0" rpy="${-pi/2.0} 0 0" />     
        </collision>
    </link>

    <link name="left_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}" />
            </geometry>
            <origin xyz="0 0 0" rpy="${-pi/2.0} 0 0" />
            <material name="blue" />
        </visual>
        <collision>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}" />
            </geometry>
            <origin xyz="0 0 0" rpy="${-pi/2.0} 0 0" />     
        </collision>
    </link>

    <link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="${wheel_radius/2.0}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="red" />
        </visual>
        <collision>
            <geometry>
                <sphere radius="${wheel_radius/2.0}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />   
        </collision>
    </link>

</robot>
```
3. The dimension of body and wheels are:

![Robot_Dimension](https://github.com/twming/ros2_master_tutorial/blob/main/img/autocar_model.png)

```
size_ratio = 0.3
base_length = ${size_ratio*0.6}
base_width = ${size_ratio*0.4}
base_height = ${size_ratio*0.2}
wheel_radius = ${size_ratio*0.1}
wheel_length = ${size_ratio*0.05}
```
4. Define the joints between the wheel and body
```
    <joint name="base_footprint_base_joint" type="fixed">
        <parent link="___TODO___" />
        <child link="___TODO___" />
        <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0" />
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="___TODO___" />
        <child link="___TODO___" />
        <origin xyz="${-base_length/4.0} ${-(base_width+wheel_length)/2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="___TODO___" />
        <child link="___TODO___" />
        <origin xyz="${-base_length/4.0} ${(base_width+wheel_length)/2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="base_caster_wheel_joint" type="fixed">
        <parent link="___TODO___" />
        <child link="___TODO___" />
        <origin xyz="${base_length/3.0}  0 ${-wheel_radius/2.0}" rpy="0 0 0" />
    </joint>
```
5. Define the parent and child links
6. Launch it in ROS RViz using robot_state_publisher and joint_state_publisher_gui

> [!TIP]
> Can you visualize your robot in ROS RViz? TF Tree, Joint_State.

```
cd ~/dev_ws/src/autocar_description/urdf
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro autocar.xacro)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui
ros2 run rviz2 rviz2
```

### Optional 1: differential drive
1. Create common_properties.xacro file, add below for inertial simulation.
```
<?xml version="1.0"?>

<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="box_inertia" params="m l w h xyz rpy">
        <inertial>
            <origin xyz="${xyz}" rpy="${rpy}" />
            <mass value="${m}" />
            <inertia ixx="${(m/12)*(h*h+l*l)}" ixy="0" ixz="0"
                iyy="${(m/12)*(w*w+l*l)}" iyz="0"
                izz="${(m/12)*(w*w+h*h)}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="sphere_inertia" params="m r xyz rpy">
        <inertial>
            <origin xyz="${xyz}" rpy="${rpy}" />
            <mass value="${m}" />
            <inertia ixx="${(2*m/5)*(r*r)}" ixy="0" ixz="0"
                iyy="${(2*m/5)*(r*r)}" iyz="0"
                izz="${(2*m/5)*(r*r)}" /> 
        </inertial>
    </xacro:macro>

    <xacro:macro name="cylinder_inertia" params="m r h xyz rpy">
        <inertial>
            <origin xyz="${xyz}" rpy="${rpy}" />
            <mass value="${m}" />
            <inertia ixx="${(m/12)*(3*r*r+h*h)}" ixy="0" ixz="0"
                iyy="${(m/12)*(3*r*r+h*h)}" iyz="0"
                izz="${(m/1)*(r*r)}" /> 
        </inertial>
    </xacro:macro>

</robot>
```
2. Add below lines to the respective object after "collision" tag.
```
<xacro:box_inertia m="5.0" l="${base_length}" w="${base_width}" h="${base_height}" xyz="0 0 ${base_height/2.0}" rpy="0 0 0" />
...
<xacro:cylinder_inertia m="1.0" r="${wheel_radius}" h="${wheel_length}" xyz="0 0 0" rpy="${-pi/2.0} 0 0" />
...
<xacro:cylinder_inertia m="1.0" r="${wheel_radius}" h="${wheel_length}" xyz="0 0 0" rpy="${-pi/2.0} 0 0" />
...
<xacro:sphere_inertia m="0.5" r="${wheel_radius/2.0}" xyz="0 0 0" rpy="${-pi/2.0} 0 0" />
```
3. Create gazebo.xacro file, add below for Gazebo differential drive simulation.
```
<?xml version="1.0"?>

<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <gazebo reference="base_link">
        <material>Gazebo/Green</material>
    </gazebo>

    <gazebo reference="right_wheel_link">
        <material>Gazebo/Blue</material>
    </gazebo>

    <gazebo reference="left_wheel_link">
        <material>Gazebo/Blue</material>
    </gazebo>

    <gazebo reference="caster_wheel_link">
        <material>Gazebo/Red</material>
        <mu1 value="0.1" />
        <mu2 value="0.1" />
    </gazebo>

    <gazebo>
        <plugin name="diff_drive_control" filename="libgazebo_ros_diff_drive.so">

            <!-- Update rate in Hz -->
            <update_rate>50</update_rate>

            <!-- wheels -->
            <left_joint>base_left_wheel_joint</left_joint>
            <right_joint>base_right_wheel_joint</right_joint>

            <!-- kinematics -->
            <wheel_separation>0.45</wheel_separation>
            <wheel_diameter>0.2</wheel_diameter>

            <!-- output -->
            <publish_odom>true</publish_odom>
            <publish_odom_tf>true</publish_odom_tf>
            <publish_wheel_tf>true</publish_wheel_tf>

            <odometry_topic>odom</odometry_topic>
            <odometry_frame>odom</odometry_frame>
            <robot_base_frame>base_footprint_link</robot_base_frame>

        </plugin>
    </gazebo>
</robot>
```
4. Include below lines in the autocar.xacro file
```
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="autocar">
    <xacro:include filename="common_properties.xacro" />
    ....
    <xacro:include filename="gazebo.xacro" />
</robot>
```
5. Launch Gazebo Simulation
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro autocar.xacro)"
ros2 launch gazebo_ros gazebo_launch.py
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot
```
![Gazebo](https://github.com/twming/ros2_master_tutorial/blob/main/img/gazebo.png)

> [!TIP]
> Control your robot using topic /cmd_vel.
> 
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

### Optional 2: lidar and imu
1. Add laser_link and imu_link to autocar.xacro file.
```
    <link name="laser_link">
        <visual>
            <geometry>
                <cylinder radius="0.02" length="0.01"/>
            </geometry>
            <origin xyz="0 0 0.005" rpy="0 0 0" />
            <material name="red"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.01" length="0.01"/>
            </geometry>
        </collision>
        <xacro:cylinder_inertia m="0.1" r="0.05" h="0.04" xyz="0 0 0" rpy="0 0 0" />
    </link>

    <link name="imu_link">
        <visual>
            <geometry>
                <box size="0.01 0.01 0.005"/>
            </geometry>
            <origin xyz="0 0 0.015" rpy="0 0 0" />
            <material name="red"/>
        </visual>        
    </link>
```
2. Add base_laser_joint and imu_base_imu_joint to autocar.xacro file.
```
    <joint name="base_laser_joint" type="fixed">
        <parent link="base_link"/>
        <child link="laser_link"/>
        <origin xyz="${-base_length/3.0} 0 ${base_height}" rpy="0 0 0"/>
    </joint>
 
    <joint name="base_imu_joint" type="fixed">
        <parent link="base_link"/>
        <child link="imu_link"/>
        <origin xyz="0 0 -0.0025" rpy="0 0 0" />
    </joint>
```
3. Add color for laser and imu link in gazebo.xacro.
```
    <gazebo reference="laser_link">
        <material>Gazebo/Red</material>
    </gazebo>
    
    <gazebo reference="imu_link">
        <material>Gazebo/Red</material>
    </gazebo>
```
4. Add ros plugin for laser and imu link in gazebo.xacro
```
    <gazebo reference="laser_link">
        <sensor name="laser" type="ray">
            <pose> 0 0 0 0 0 0 </pose>
            <visualize>true</visualize>
            <update_rate>10</update_rate>
            <ray> 
                <scan>
                    <horizontal>
                        <samples>360</samples>
                        <min_angle>-3.14</min_angle>
                        <max_angle>3.14</max_angle>
                    </horizontal>
                </scan>
                <range>
                    <min>0.3</min>
                    <max>12</max>
                </range>
            </ray>
            <plugin name="laser" filename="libgazebo_ros_ray_sensor.so">
                <output_type>sensor_msgs/LaserScan</output_type>
                <frame_name>laser_link</frame_name>
            </plugin>
        </sensor>
    </gazebo>

    <gazebo reference="imu_link">
        <sensor name="imu_sensor" type="imu">
            <always_on>true</always_on>
            <update_rate>100</update_rate>
            <visualize>true</visualize>
            <imu>
                <angular_velocity>
                    <x>
                        <noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev><bias_mean>0.0000075</bias_mean><bias_stddev>0.0000008</bias_stddev></noise>
                    </x>
                    <y>
                        <noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev><bias_mean>0.0000075</bias_mean><bias_stddev>0.0000008</bias_stddev></noise>
                    </y>
                    <z>
                        <noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev><bias_mean>0.0000075</bias_mean><bias_stddev>0.0000008</bias_stddev></noise>
                    </z>
                </angular_velocity>
                <linear_acceleration>
                    <x>
                        <noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev><bias_mean>0.1</bias_mean><bias_stddev>0.001</bias_stddev></noise>
                    </x>
                    <y>
                        <noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev><bias_mean>0.1</bias_mean><bias_stddev>0.001</bias_stddev></noise>
                    </y>
                    <z>
                        <noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev><bias_mean>0.1</bias_mean><bias_stddev>0.001</bias_stddev></noise>
                    </z>
                </linear_acceleration>
            </imu>
            <plugin name="imu" filename="libgazebo_ros_imu_sensor.so">
                <initial_orientation_as_reference>false</initial_orientation_as_reference>
            </plugin>
        </sensor>
    </gazebo>
```
5. Launch Gazebo Simulation with turtlebot3_world.world
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro autocar.xacro)"
ros2 launch gazebo_ros gazebo.launch.py world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot -x -2.0 -y -0.5
```
6. Move the robot
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.08
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

## Exercise 3: Raspberry Pi Turtlebot3 Setup

1. Download "Raspberry Pi Imager" and install it.
```
https://www.raspberrypi.com/software/
```
![Raspberry Pi](https://github.com/twming/ros2_master_tutorial/blob/main/img/raspberrypi.png)

2. Use the SD-Card provided, install Ubuntu 22.04 server on the card using "Raspberry Pi Imager"
3. Connect up Raspberry Pi to TV monitor, keyboard, mouse. Boot up the into Ubuntu.
4. Edit the sshd_config file to set the ssh QoS to best effort
```
sudo nano /etc/ssh/sshd_config
```
Input below line at the bottom of the file
```
IPQoS cs0 cs0
```
Edit the sshd_config setting
```
sudo nano /etc/ssh/sshd_config.d/50-cloud-init.conf
```
Change the PasswordAuthentication from 'no' to 'yes'
```
PasswordAuthentication yes
```
5. Follow the ROS Humble Desktop/Base Installation in Ubuntu for Raspberry Pi:-
Enable UTF-8 locale support
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
Enable Universe repository in Ubuntu
```
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
```
Add the ROS2 GPG key with apt
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Add the repository to Ubuntu source list
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Update the repository
```
sudo apt update
```
Then install ros-humble-ros-base and ros-humble-turtlebot3-bringup
```
sudo apt install -y ros-humble-ros-base
sudo apt install -y ros-humble-turtlebot3-bringup
```
6. Input the environment data to the .bashrc
```
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc
```
Source and run the .bashrc 
```
source .bashrc
```
7. Setup the USB and OpenCR communication
```
sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

> [!TIP]
> How do you test your turtlebot3 burger?

#### SLAM
```
ros2 launch turtlebot3_bringup robot.launch.py
```
```
ros2 launch slam_toolbox online_async_launch.py
```
```
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```
#### Navigation
```
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/root/my_map.yaml
```
```
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## Exercise 4: Robot Control

Now you have turtlebot3 burger (with Raspberry Pi) and the Ubuntu VM. You need to control the robot to move forward and stop when Lidar detected obstacle 50 cm in front (0 degree).
#### Autonomous Exploring
1. Install ros-humble-turltebot3 packages
```
sudo apt install ros-humble-turtlebot3
```
2. Create "autocar_control" package, put in below code.
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.move = Twist()

    def scan_callback(self, msg):
        print('s1 [0]:', msg.ranges[0])
        
        if msg.ranges[0] > 0.5:
            self.move.linear.x = 0.3
            self.move.angular.z = 0.0
        else:
            self.move.linear.x = 0.0
            self.move.angular.z = 0.0
        
        self.publisher.publish(self.move)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
3. Build and run the python script.

> [!TIP]
> Does turtlebot3 burger able to detect obstacle?


#### Send Position data to IoT Cloud

1. Create a python file, called "iot.py" 
```
#!/usr/bin/env python3
import rclpy
from urllib.request import urlopen
from rclpy.node import Node
from nav_msgs.msg import Odometry

class IoTNode(Node):
    def __init__(self):
        super().__init__('IoTNode')
        self.x=0.0
        self.y=0.0
        self.timer = self.create_timer(2.0, self.send_position_command)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.record_position_callback,
        10)

    def send_position_command(self):
        response=urlopen('https://api.thingspeak.com/update?api_key='
                         +'____________'
                         +'&field1='+str(self.x)
                         +'&field2='+str(self.y))
        self.get_logger().info("Data Sent Status : %s" % response.msg)

    def record_position_callback(self, odom:Odometry):
        self.x=odom.pose.pose.position.x
        self.y=odom.pose.pose.position.y
        #self.get_logger().info("Pose: ("+str(self.x)+","+str(self.y)+")")

def main(args=None):
    rclpy.init(args=args)
    iotnode = IoTNode()
    rclpy.spin(iotnode)
    iotnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
2. Build and run with navigation map and path planning
 
#### Control turtlebot3 using hand gesture
1. Check your VM camera
```
cd C:\Program Files\Oracle\VirtualBox
VBoxManage list webcams
```
2. Attach webcam to VM
```
VBoxManage controlvm <vm_name> webcam attach .1
```
3. Create a python file, called "gesture_control.py" 
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import cv2,time
import math
import random
import numpy as np

scan0 = 0
scan45 = 0
scan90 = 0
scan135 = 0
scan180 = 0
scan225 = 0
scan270 = 0
scan315 = 0
allowance = 0.3
m=Twist()

# function to collect laser scan data
def callback(msg):
	global scan0, scan45, scan90, scan135, scan180, scan225, scan270, scan315
	scan0 = msg.ranges[0]
	scan45 = msg.ranges[45]
	scan90 = msg.ranges[90]
	scan135 = msg.ranges[135]
	scan180 = msg.ranges[180]
	scan225 = msg.ranges[225]
	scan270 = msg.ranges[270]
	scan315 = msg.ranges[315]

# function to clear TB3 away from obstacles
def clearance():
	if scan0 < allowance or scan45 < allowance or scan315 < allowance:
		m.linear.x = -0.05

	elif scan180 < allowance or scan135 < allowance or scan225 < allowance:
		m.linear.x = 0.05

	elif scan90 < allowance:
		m.angular.z = -0.05
		m.linear.x = 0.025

	elif scan270 < allowance:
		m.angular.z = 0.05
		m.linear.x = 0.0

	#pub.publish(m)

############################################################
node=None
# Open Camera
def main(args=None):
	rclpy.init(args=None)
        # TODO:
        # Step1: create a node call 'hand'
        # Step2: create a publisher, to publish a Twist message to /cmd_vel, with QoS =1
        # Step3: create a subscriber, to subscribe a LaserScan message, "callback" function and QoS=10
        #
        #
        #

	capture = cv2.VideoCapture(0)

	print('ONE : FORWARD')
	print('TWO : COUNTERCLOCKWISE')
	print('THREE : CLOCKWISE')
	print('FOUR : REVERSE')


	while(capture.isOpened()):
		
		# Capture frames from the camera
		ret, frame = capture.read()
		frame = cv2.flip(frame,1)
		
		# Get hand data from the rectangle sub window
		cv2.rectangle(frame, (450,80), (600,250), (0, 255, 0), 0)
		crop_image = frame[80:250,450:600]
		drawing = np.zeros(crop_image.shape,dtype =  np.uint8)

		# Apply Gaussian blur (also known as Gaussian smoothing); to reduce image noise and reduce detail
		blur = cv2.GaussianBlur(crop_image, (5, 5), 0)

		# Change color-space from BGR -> HSV
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

		# Create a binary image with where white will be skin colors and rest is black
		mask2 = cv2.inRange(hsv, np.array([0, 20, 70]), np.array([20, 255, 255]))

		# Define Kernel (Filter Matrix or "Slider") for morphological transformation
		kernel = np.ones((5, 5))

	        ############ Apply morphological transformations to filter out the background noise ##############
		# Filtering Sequence: Dilation, Erosion, Closing
		# Dilation is to gradually enlarge the boundaries of regions of foreground pixels, typically white pixels. As areas of foreground pixels grow in size, holes within those regions become smaller.
		# Erosion is to erode away the boundaries of regions of foreground pixels, typically white pixels. As areas of foreground pixels shrink in size, holes within those areas become larger.
		# Closing is simply as a dilation followed by an erosion using the same structuring element for both operations. The effect of this operator is to preserve background regions that have a similar shape to this structuring element, or that can completely contain the structuring element, while eliminating all other regions of background pixels.
	        ##################################################################################################
		
		dilation = cv2.dilate(mask2, kernel, iterations=1)
		erosion = cv2.erode(dilation, kernel, iterations=1)
		closing = cv2.morphologyEx(erosion, cv2.MORPH_CLOSE, np.ones((5,5)))

		# Apply Gaussian Blur and Threshold (Otsu's thresholding after Gaussian filtering)
		# Thresholding is a technique in OpenCV, which is the assignment of pixel values in relation to the threshold value provided. In thresholding, each pixel value is compared with the threshold value. If the pixel value is smaller than the threshold, it is set to 0, otherwise, it is set to a maximum value (set to 255 in this case)
		filtered = cv2.GaussianBlur(erosion, (5, 5), 0)
		ret, thresh = cv2.threshold(filtered,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

		# Show threshold image
		cv2.imshow("Thresholded", thresh)
		# Find contours
		contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		try:
			# Find contour with maximum area
			contour = max(contours, key=lambda x: cv2.contourArea(x))

			# Create bounding rectangle around the contour
			x, y, w, h = cv2.boundingRect(contour)
			cv2.rectangle(crop_image, (x, y), (x + w, y + h), (0, 0, 255), 0)

			# Find convex hull; the set of pixels included in the smallest convex polygon that surround all input data.
			hull = cv2.convexHull(contour)

			# Draw contour; for our own visualization purposes
			drawing = np.zeros(crop_image.shape,dtype =  np.uint8)
			#drawing = np.zeros(crop_image.shape, np.uint8)
			cv2.drawContours(drawing, [contour], -1, (0, 255, 0), 0)
			cv2.drawContours(drawing, [hull], -1, (0, 0, 255), 0)

			# Find convexity defects; Any deviation of the object from the convex hull can be considered as convexity defect; 
			# returnPoints=True, then it returns the coordinates of the hull points. If returnPoints=False, it returns the indices of contour points corresponding to the hull points
			hull = cv2.convexHull(contour, returnPoints=False)

			# cv2.convexityDefects(contour, hull) returns an array where each row contains these values: [start point, end point, farthest point, approximate distance to farthest point]
			defects = cv2.convexityDefects(contour, hull)
			g = range(defects.shape[0])

			# Use cosine rule to find angle of the far point from the start and end point i.e. the convex points (the finger tips) for all defects
			count_defects = 0
			
			for i in g:
				s, e, f, d = defects[i, 0]
				start = tuple(contour[s][0])
				end = tuple(contour[e][0])
				far = tuple(contour[f][0])

				a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
				b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
				c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
				angle = (math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c)) * 180) / 3.14

				# if angle > 90 draw a circle at the far point
				if angle <= 90:
					count_defects += 1
					cv2.circle(crop_image, far, 1, [0, 0, 255], -1)

				cv2.line(crop_image, start, end, [0, 255, 0], 2)
				#print ('count_defects:')
				print (count_defects)

			# TB3 Control Rules
			if (scan0 < allowance or  scan45<allowance  or  scan90<allowance or  scan135<allowance  or  scan180<allowance  or  scan225<allowance or  scan270<allowance   or scan315 < allowance) and (scan0 !=0 or  scan45 !=0 or scan90 !=0 or scan135 !=0 or scan180 !=0 or scan225 !=0 or scan270 !=0 or scan315 !=0):
				m.linear.x = 0.0
				m.angular.z = 0.0
				print('braking due to obstacles detected')

				clearance()
				pub.publish(m)
				print('Clearing away from obstacles')

			elif count_defects == 0:
				cv2.putText(frame, "ONE : FORWARD", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255),2)
				m.linear.x = 0.05
				m.angular.z = 0.0
				print('forward')
			elif count_defects == 1:
				cv2.putText(frame, "TWO : COUNTERCLOCKWISE", (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255), 2)
				m.linear.x = 0.0
				m.angular.z = 0.05
				print('left')
			elif count_defects == 2:
				cv2.putText(frame, "THREE : CLOCKWISE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255), 2)
				m.linear.x = 0.0
				m.angular.z = -0.05
				print('right')
			elif count_defects == 3:
				cv2.putText(frame, "FOUR : REVERSE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255), 2)
				m.angular.z = 0.0
				m.linear.x = -0.05
				print('reverse')
			elif count_defects == 4:
				cv2.putText(frame, "FIVE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255), 2)
				
			# Show required images
			pub.publish(m)
		except:
			# Brake if no gesture input
			m.linear.x = 0.0
			m.angular.z = 0.0
			print('braking due to no input')

			pub.publish(m)
			pass
		
		cv2.imshow("Gesture", frame)
		all_image = np.hstack((drawing, crop_image))
		cv2.imshow('Contours', all_image)
		if cv2.waitKey(3) == ord('q'):
			print('Shutting Down!!!')
			break
	capture.release()

if __name__=='__main__':
	main()
```
4. Build and run.
