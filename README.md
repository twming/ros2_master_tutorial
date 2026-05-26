# Mastering ROS Essential

- [Exercise 1: ROS Development Setup](#exercise-1-ros-development-setup)

- [Exercise 2: Simulate Robot in ROS](#exercise-2-simulate-robot-in-ros-and-gazebo)
  - [2.1: autocar URDF and Simulation in RViz](#21-autocar-urdf-and-simulation-in-rviz)
  - [2.2: RViz Configuration Setup and Saving](#22-rviz-configuration-setup-and-saving)
  - [2.3: Launch file for autocar in RViz](#23-launch-file-for-autocar-in-rviz)
  - [2.4: autocar Differential Drive and Simulation in Gazebo](#24-autocar-differential-drive-and-simulation-in-gazebo)
  - [2.5: autocar Lidar and Imu Simulation in Gazebo](#25-autocar-lidar-and-imu-simulation-in-gazebo)
  - [2.6 Launch file for autocar in Gazebo](#26-launch-file-for-autocar-in-gazebo)

- [Exercise 3: Writing a Hardware Component](#exercise-2-writing-a-hardware-component)
  - [3.1: Create babybot_firmware package](#21-create-babybot-firmware-package)
  - [3.2: Setup the babybot_firmware.hpp header file](#22-setup-the-babybotfirmwarehpp-header-file)
  - [3.3: Implement babybot_interface.cpp source file](#23-implement-babybotinterfacecpp-source-file)
  - [3.4: Prepare for build the package](#24-prepare-for-build-the-package)
  - [3.5: Create export definition for pluginlib](#25-create-export-definition-for-pluginlib)
  - [3.6: Build the package](#26-build-the-package)


# Exercise 1: ROS Development Setup
You need to setup a virtual box and install ROS for your robot development
1. Go to Virtual box website and download the application, install in your laptop.
```
https://www.virtualbox.org/wiki/Downloads
```
<img src="https://github.com/twming/ros2_master_tutorial/blob/main/img/virtualbox.png" alt="Virtual Box" width="600">

2. Setup Ubuntu 22.04 in your virtual box
```
https://releases.ubuntu.com/jammy/
```
<img src="https://github.com/twming/ros2_master_tutorial/blob/main/img/ubuntu.png" alt="Ubuntu" width="600">


3. Install ROS Humble in your Ubuntu
   
> [!IMPORTANT] 
> - Follow the steps in "Set locale", "Setup Sources", "Install ROS 2 packages" in below site.
> - In "Install ROS 2 packages" step, install "ros-humble-desktop" for Ubuntu VM, while install "ros-humble-ros-base" for Raspberry Pi

```
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```
<img src="https://github.com/twming/ros2_master_tutorial/blob/main/img/ros.png" alt="ROS" width="600">

4. Install other dependency ROS packages
```
sudo apt install -y ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-xacro
sudo apt install -y ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
```
5. Setup ROS environment
```
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc
```
6. Source the ROS environment
```
source ~/.bashrc
```

> [!TIP]
> - How to verify your ROS installation is working?
> - What is the command to check ROS topic?

> [!IMPORTANT] 
> - While install ROS, it is important to reply "Y" or "YES" to allow ubuntu continue on the installation!
> - To automatic reply "Y", you may add "-y" after all the "sudo apt install -y ..." and "sudo add-apt-repository -y ..."
> - Create ros_humble_install.sh shell script file, and copy the command into the file and automate in the next installation.
___



# Exercise 2: Simulate Robot in ROS and Gazebo
### 2.1: autocar URDF and Simulation in RViz
You learn how to describe robot in URDF, create a URDF for below robots and launch it in ROS simulation.
1. Create "autocar_description" package (ament_cmake), create "urdf" folder, then add "autocar.xacro" file to the folder.
```
ros2 pkg create --build-type ament_cmake autocar_description
```
2. Define the link of robot, including a box (body) and two cylinders (wheel) in the "autocar.xacro" file
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
3. The dimension of body and wheels are, replace _ _TODO_ _ with below values.
```
size_ratio = 0.3
base_length = ${size_ratio*0.6}
base_width = ${size_ratio*0.4}
base_height = ${size_ratio*0.2}
wheel_radius = ${size_ratio*0.1}
wheel_length = ${size_ratio*0.05}
```
![Robot_Dimension](https://github.com/twming/ros2_master_tutorial/blob/main/img/autocar_model.png)

4. Define the joints between the wheel and body (after all the links in "autocar.xacro" file), replace _ _TODO_ _ with the correct parent and child links
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
6. Update CMakeLists.txt to install "urdf" folder. Add below lines to CMakeLists.txt
```
install(
   DIRECTORY urdf 
   DESTINATION share/${PROJECT_NAME}/
)
```
7. Colcon build the package and source the setup.bash
8. Launch it in ROS RViz using robot_state_publisher and joint_state_publisher_gui

> [!TIP]
> Can you visualize your robot in ROS RViz? TF Tree, Joint_State.

Terminal 1:
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro `ros2 pkg prefix --share autocar_description`/urdf/autocar.xacro)"
```
Terminal 2:
```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
Terminal 3:
```
ros2 run rviz2 rviz2
```
9. Add "RobotModel" and "TF" to RViz. You need to set RobotModel "Fixed Frame" to "base_footprint_link", "Description Topic" to "/robot_description",
- Fixed Frame: base_footprint_link
- Description Topic: /robot_description
   
### 2.2: RViz Configuration Setup and Saving

1. Create "rviz" folder
2. Save the RViz view/config to rviz folder, file name "autocar.rviz".

### 2.3: Launch file for autocar in RViz

> [!IMPORTANT]
> - It is good to run multiple nodes (robot_state_publisher, joint_state_publisher and rviz2) using a launch file.
> - Create a launch folder and autocar_display.launch, then launch it. 

1. Create a "launch" folder, then add "autocar_display.launch" file to the folder.
2. Add the "autocar_display.launch" content as below, save the file.
```
<launch>
    <let name="urdf_path" value="$(find-pkg-share autocar_description)/urdf/autocar.xacro" />
    <let name="rviz_path" value="$(find-pkg-share autocar_description)/rviz/autocar.rviz" />
    
    <node pkg="robot_state_publisher" exec="robot_state_publisher">
        <param name="robot_description" value="$(command 'xacro $(var urdf_path)')" />
    </node>

    <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" />

    <!--node pkg="rviz2" exec="rviz2" output="screen"/-->

    <node pkg="rviz2" exec="rviz2" output="screen"
        args="-d $(var rviz_path)" />
</launch>
```
3. Update "rviz" and "launch" in CMakeLists.txt
```
install (
  DIRECTORY urdf rviz launch
  DESTINATION share/${PROJECT_NAME}/

)
```
4. Colcon build and launch
```
ros2 launch autocar_description autocar_display.launch
```
### 2.4: autocar Differential Drive and Simulation in Gazebo
1. Create "common_properties.xacro" file in urdf folder, add below for inertial simulation.
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
2. Add below lines to the respective links after "collision" tag. These help to simulate the moment of inertia in the real-world.
```
<xacro:box_inertia m="5.0" l="${base_length}" w="${base_width}" h="${base_height}" xyz="0 0 ${base_height/2.0}" rpy="0 0 0" />

<xacro:cylinder_inertia m="1.0" r="${wheel_radius}" h="${wheel_length}" xyz="0 0 0" rpy="${-pi/2.0} 0 0" />

<xacro:cylinder_inertia m="1.0" r="${wheel_radius}" h="${wheel_length}" xyz="0 0 0" rpy="${-pi/2.0} 0 0" />

<xacro:sphere_inertia m="0.5" r="${wheel_radius/2.0}" xyz="0 0 0" rpy="${-pi/2.0} 0 0" />
```
<img src="https://github.com/twming/ros2_master_tutorial/blob/main/img/collision_tag.png" alt="Inertia" width="600">

3. Create "gazebo.xacro" file, add below for Gazebo differential drive simulation. This will simulate two-wheel differential drive.
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
4. Before complete, we need to include both "common_properties.xacro" and "gazebo.xacro" in the "autocar.xacro" file.
```
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="autocar">
    <xacro:include filename="common_properties.xacro" />
    <xacro:include filename="gazebo.xacro" />
</robot>
```
5. Colcon build the package and source the setup.bash
6. Launch Gazebo Simulation

Terminal 1:
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro `ros2 pkg prefix --share autocar_description`/urdf/autocar.xacro)"
```
Terminal 2:
```
ros2 launch gazebo_ros gazebo.launch.py
```
Terminal 3:
```
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot
```
<img src="https://github.com/twming/ros2_master_tutorial/blob/main/img/gazebo.png" alt="Gazebo" width="600">

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

### 2.5: autocar Lidar and Imu Simulation in Gazebo
1. Add laser_link and imu_link to "autocar.xacro" file.
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
2. Add base_laser_joint and imu_base_imu_joint to "autocar.xacro" file.
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
3. Add color for laser and imu link in "gazebo.xacro".
```
    <gazebo reference="laser_link">
        <material>Gazebo/Red</material>
    </gazebo>
    
    <gazebo reference="imu_link">
        <material>Gazebo/Red</material>
    </gazebo>
```
4. Add ros plugin for laser and imu link in "gazebo.xacro"
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


Terminal 1:
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro `ros2 pkg prefix --share autocar_description`/urdf/autocar.xacro)"
```
Terminal 2:
```
ros2 launch gazebo_ros gazebo.launch.py world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
```
Terminal 3:
```
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
### 2.6 Launch file for autocar in Gazebo
> [!IMPORTANT]
> - create a launch file "autocar_gazebo.launch"

```
<launch>
    <let name="urdf_path" value="$(find-pkg-share autocar_description)/urdf/autocar.xacro" />

    <node pkg="robot_state_publisher" exec="robot_state_publisher">
        <param name="robot_description" value="$(command 'xacro $(var urdf_path)')" />
    </node>

    <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py">
        <arg name="world" value="$(find-pkg-share turtlebot3_gazebo)/worlds/turtlebot3_world.world" />
    </include>
    
    <node pkg="gazebo_ros" exec="spawn_entity.py"
        args=" -topic robot_description -entity my_robot -x -2.0 -y -0.5" />

</launch>
```
> [!TIP]
> You have learn how to teleop the turtlebot3, use the turtlebot3_teleop to move it.
___



# Exercise 3: Writing a Hardware Component

You learn how to create hardware interface component by step-by-step guides.
### 3.1: Create babybot_firmware package

1. Create "babybot_firmware" package (ament_cmake), create "babybot_firmware" folder in "include" folder.
```
ros2 pkg create --build-type ament_cmake babybot_firmware
```
### 3.2: Setup the babybot_firmware.hpp header file
2. Create header file, "babybot_firmware.hpp" in the "include/babybot_firmware" folder. The header file include system_interface, node_interfaces, state and SerialPort.

```
#ifndef BABYBOT_INTERFACE_HPP_
#define BABYBOT_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <libserial/SerialPort.h>

#include <vector>
#include <string>

namespace babybot_firmware
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    
    class BabybotInterface : public hardware_interface::SystemInterface 
    {
        public:
        // TODO: define public functions here
            
        private:
        // TODO: define private functions here
    };
}

#endif
```
3. Define the public functions below.

```
            BabybotInterface();
            virtual ~BabybotInterface();

            virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
            virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
```

4. Define the private function below.
```
            LibSerial::SerialPort arduino_;

            std::string port_;
            std::vector<double> velocity_commands_;
            std::vector<double> position_states_;
            std::vector<double> velocity_states_;

            rclcpp::Time last_run_;
```

### 3.3: Implement babybot_interface.cpp source file
5. Create source file, "babybot_interface.cpp" in the "src/babybot_firmware" folder.
```
#include "babybot_firmware/babybot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace babybot_firmware
{
    BabybotInterface::BabybotInterface()
    {

    }

    BabybotInterface::~BabybotInterface()
    {
        /*
        Babybot Destructor - if arduino port is opened, then close it
        */

    }

    CallbackReturn BabybotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
    {
        /*
        Initialize serial port to Arduino
        */

    }

    std::vector<hardware_interface::StateInterface>BabybotInterface::export_state_interfaces()
    {
        /*
        position state and velocity state
        */        
    }
    
    std::vector<hardware_interface::CommandInterface> BabybotInterface::export_command_interfaces()
    {
        /*
        the command_interface 
        */

    }

    CallbackReturn BabybotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        /*
        initialize commands, positions and velocity, set the serial baudrate 
        */        
    }

    CallbackReturn BabybotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        /*
        deactivate serial port 
        */        
    }

    hardware_interface::return_type BabybotInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        /*
        read the wheel encoder reading, store in the velocity, position arrays 
        */        
    }

    hardware_interface::return_type BabybotInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        /*
        write the PMW to serial
        */        
    }
        
}

/*
export the babybot_firmware plugin 
*/


```
6. Implement the BabybotInterface::~BabybotInterface() function.
```
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("BabybotInterface"),"Something wen wrong while closing the connection with port "<< port_);
            }

        }
```
7. Implement BabybotInterface::on_init function.
```
CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }
        try
        {
            port_=info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("BabybotInterface"),"No Serial Port Provided! Aborting");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        last_run_ = rclcpp::Clock().now();

        return CallbackReturn::SUCCESS;
```

8. Implement BabybotInterface::export_state_interfaces() function.
```
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
                hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }
        return state_interfaces;
```
9. Implement BabybotInterface::export_command_interfaces() function.
```
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }
        return command_interfaces;
```
10. Implement BabybotInterface::on_activate function.
```
       RCLCPP_INFO(rclcpp::get_logger("BabybotInterface"), "Starting robot hardware...");
        velocity_commands_ = {0.0, 0.0};
        position_states_ = {0.0, 0.0};
        velocity_states_ = {0.0, 0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("BabybotInterface"),"Something wen wrong while closing the connection with port "<< port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("BabybotInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
```
11. Implement BabybotInterface::on_deactivate function.
```
        RCLCPP_INFO(rclcpp::get_logger("BabybotInterface"), "Stopping robot hardware...");
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("BabyotInterface"),"Something wen wrong while closing the connection with port "<< port_);
                return CallbackReturn::FAILURE;
            }

        }
```
12. Implement BabybotInterface::read function.
```
        if(arduino_.IsDataAvailable())
        {
            auto dt=(rclcpp::Clock().now() - last_run_).seconds();
            std::string message;
            arduino_.ReadLine(message);
            std::stringstream ss(message);
            std::string res;
            int multiplier =1;
            while (std::getline(ss, res, ','))
            {
                multiplier = res.at(1) == 'p' ? 1 : -1;
                if(res.at(0) == 'r')
                {
                    velocity_states_.at(0) = multiplier * std::stod(res.substr(2,res.size()));
                    position_states_.at(0) += velocity_states_.at(0) * dt;
                }
                else if (res.at(0) == 'l')
                {
                    velocity_states_.at(1) = multiplier * std::stod(res.substr(2,res.size()));
                    position_states_.at(1) += velocity_states_.at(1) * dt;    
                }
            }
            last_run_ = rclcpp::Clock().now();
        }
        return hardware_interface::return_type::OK;
```
13. Implement BabybotInterface::write function.
```
        std::stringstream message_stream;
        char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
        char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";

        if ( std::abs(velocity_commands_.at(0)) < 10.0 )
        {
            compensate_zeros_right = "0";
        }
        else
        {
            compensate_zeros_right = "";
        }

        if ( std::abs(velocity_commands_.at(1)) < 10.0 )
        {
            compensate_zeros_left = "0";
        }
        else
        {
            compensate_zeros_left = "";
        }

        message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) <<
            ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << ",";

        try
        {
            arduino_.Write(message_stream.str());
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("BabybotInterface"),"Something went wrong while sending the message" << 
                message_stream.str() << " on the port " << port_ );
            return hardware_interface::return_type::ERROR;
        }
        
        return hardware_interface::return_type::OK;
```
14. Export BabybotInterface as plugin.
```
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(babybot_firmware::BabybotInterface, hardware_interface::SystemInterface)
```

### 3.4: Prepare for build the package
15. Setup CMakeLists.txt for colcon build
```
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


pkg_check_modules(SERIAL libserial)


add_library(${PROJECT_NAME} SHARED
  src/babybot_interface.cpp  
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include 
  "${SERIAL_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME} "${SERIAL_LDFLAGS}")
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface babybot_interface.xml)


# Install include, library, launch
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include
  DESTINATION include
)

#install(
#  DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)

# Export include, library, dependency
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

```
16. Setup the dependency in package.xml
```
  <depend>rclpy</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>libserial-dev</depend>
  <depend>hardware_interface</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>pluginlib</depend>

  <exec_depend>python3-serial</exec_depend>
```

### 3.5: Create export definition for pluginlib
17. Create export definition for pluginlib. Create "babybot_interface.xml" in the package root folder.

```
<library path = "babybot_firmware">
    <class name="babybot_firmware/BabybotInterface" 
           type="babybot_firmware::BabybotInterface"
           base_class_type="hardware_interface::SystemInterface">
        <description>
            Babybot hardware interface
        </description>
    </class>
</library>
```

### 3.6: Build the package
18. Build the package, make sure no error in the built.
```
cd ~/dev_ws
colcon build
```
### 3.7: Launch babybot_firmware
19. Create a "launch" folder in the root package, "babybot_interface.launch.py" in the "launch" folder.
```
import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    babybot_description = get_package_share_directory("babybot_description")

    # publish the urdf in real robot mode (is_sim = False)
    # gazebo run in simulation mode (default is_sim = True)
    robot_description = ParameterValue(Command([
            "xacro ",
            os.path.join(babybot_description,"urdf","babybot.urdf.xacro"),
            " is_sim:=False"
        ]), 
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
            "use_sim_time": False},
            os.path.join(
                get_package_share_directory("babybot_controller"),
                "config",
                "babybot_controllers.yaml"
            )
        ],
        remappings=[
            # ("/robot_description", "/robot_description"),
            ("~/robot_description", "/robot_description"),
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
    ])

```