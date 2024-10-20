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

> [!TIP]
> How to verify your ROS installation is working?

## Exercise 2: Simulate Robot in ROS
You learn how to describe robot in URDF, create a URDF for below robots and launch it in ROS simulation.
1. Create autocar package, with urdf folder.
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
![Ubuntu](https://github.com/twming/ros2_master_tutorial/blob/main/img/autocar_model.png)

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

## Exercise 3: Robot Hardware and Software Installation
## Exercise 4: Robot Control
