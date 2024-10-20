# Mastering ROS Essential
## Exercise 1: ROS Development Setup
You need to setup a virtual box and install ROS for your robot development
1. Go to Virtual box website and download the application, install in your laptop.
```
https://www.virtualbox.org/wiki/Downloads
```
![Virtual Box](https://github.com/twming/ros2_master_tutorial/blob/main/img/virtualbox.png)

2. Setup Ubuntu 22.04 in your virtual box
```
https://ubuntu.com/download
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
base_length = 0.6
base_width = 0.4
base_height = 0.2
wheel_radius = 0.1
wheel_length = 0.05
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

### Optional:
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
3. Create my_robot_gazebo.xacro file, add below for Gazebo differential drive simulation.
```
<?xml version="1.0"?>

<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <gazebo reference="base_link">
        <material>Gazebo/Blue</material>
    </gazebo>

    <gazebo reference="right_wheel_link">
        <material>Gazebo/Grey</material>
    </gazebo>

    <gazebo reference="left_wheel_link">
        <material>Gazebo/Grey</material>
    </gazebo>

    <gazebo reference="caster_wheel_link">
        <material>Gazebo/Grey</material>
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
    <xacro:include filename="my_robot_gazebo.xacro" />
</robot>
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
5. Follow the ROS Humble Desktop/Base Installation in Ubuntu for Raspberry Pi:-
Enable UTF-8 locale support
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
Enable Universe repository in Ubuntu
```
sudo apt install software-properties-common
sudo add-apt-repository universe
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
sudo apt install ros-humble-ros-base
sudo apt install ros-humble-turtlebot3-bringup
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

```
ros2 launch turtlebot3_bringup robot.launch.py
```

## Exercise 4: Robot Control

Now you have turtlebot3 burger (with Raspberry Pi) and the Ubuntu VM. You need to control the robot to move forward and stop when Lidar detected obstacle 50 cm in front (0 degree).
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

