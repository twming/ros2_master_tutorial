# Mastering ROS Essential

- [Exercise 1: ROS Development Setup](#exercise-1-ros-development-setup)

- [Exercise 2: Simulate Robot in ROS](#exercise-2-simulate-robot-in-ros-and-gazebo)
  - [2.1: autocar URDF and Simulation in RViz](#21-autocar-urdf-and-simulation-in-rviz)
  - [2.2: RViz Configuration Setup and Saving](#22-rviz-configuration-setup-and-saving)
  - [2.3: Launch file for autocar in RViz](#23-launch-file-for-autocar-in-rviz)
  - [2.4: autocar Differential Drive and Simulation in Gazebo](#24-autocar-differential-drive-and-simulation-in-gazebo)
  - [2.5: autocar Lidar and Imu Simulation in Gazebo](#25-autocar-lidar-and-imu-simulation-in-gazebo)
  - [2.6 Launch file for autocar in Gazebo](#26-launch-file-for-autocar-in-gazebo)

- [Exercise 3: Writing a Hardware Component](#exercise-3-writing-a-hardware-component)
  - [3.1: Create babybot_firmware package](#31-create-babybot_firmware-package)
  - [3.2: Setup the babybot_firmware.hpp header file](#32-setup-the-babybot_firmwarehpp-header-file)
  - [3.3: Implement babybot_interface.cpp source file](#33-implement-babybot_interfacecpp-source-file)
  - [3.4: Prepare for build the package](#34-prepare-for-build-the-package)
  - [3.5: Create export definition for pluginlib](#35-create-export-definition-for-pluginlib)
  - [3.6: Build the package](#36-build-the-package)


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
### 2.1: babybot URDF and Simulation in RViz
You learn how to describe robot in URDF, create a URDF for below robots and launch it in ROS simulation.
1. Create "babybot_description" package, create "urdf" folder, then add "babybot.urdf" file to the folder.
```
ros2 pkg create --build-type ament_cmake babybot_description
```
2. Define the link, joint, material, collision and inertia of the robot.
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="autocar">

    <!-- Material Color and Definition -->
    <material name="blue"><color rgba="0.0 0.0 0.8 1.0"/></material>
    <material name="green"><color rgba="0.0 1.0 0.0 1.0"/></material>
    <material name="red"><color rgba="0.8 0.0 0.0 1.0"/></material>

    <!-- Propoerty parameters -->
    <xacro:property name="size_ratio" value="0.3" />
    <xacro:property name="base_length" value="${size_ratio*0.6}" />
    <xacro:property name="base_width" value="${size_ratio*0.4}" />
    <xacro:property name="base_height" value="${size_ratio*0.2}" />
    <xacro:property name="wheel_radius" value="${size_ratio*0.1}" />
    <xacro:property name="wheel_length" value="${size_ratio*0.05}" />

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

    <joint name="base_footprint_base_joint" type="fixed">
        <parent link="base_footprint_link" />
        <child link="base_link" />
        <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0" />
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="right_wheel_link" />
        <origin xyz="${-base_length/4.0} ${-(base_width+wheel_length)/2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="left_wheel_link" />
        <origin xyz="${-base_length/4.0} ${(base_width+wheel_length)/2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="base_caster_wheel_joint" type="fixed">
        <parent link="base_link" />
        <child link="caster_wheel_link" />
        <origin xyz="${base_length/3.0}  0 ${-wheel_radius/2.0}" rpy="0 0 0" />
    </joint>


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
3. Update CMakeLists.txt to install "urdf" folder. Add below lines to CMakeLists.txt
```
install(
   DIRECTORY urdf 
   DESTINATION share/${PROJECT_NAME}/
)
```
4. Colcon build the package and source the setup.bash
```
cd ~/dev_ws
colcon build
```
5. Launch it in ROS RViz using robot_state_publisher and joint_state_publisher_gui

> [!TIP]
> Can you visualize your robot in ROS RViz? TF Tree, Joint_State.

Terminal 1:
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro `ros2 pkg prefix --share babybot_description`/urdf/babybot.urdf)"
```
Terminal 2:
```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
Terminal 3:
```
ros2 run rviz2 rviz2
```


### 2.2: babybot Differential Drive and Simulation in Gazebo
1. Add the Gazebo differential drive in the simulation. This will simulate two-wheel differential drive.
```
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

```
2. Colcon build the package and source the setup.bash
3. Launch Gazebo Simulation

Terminal 1:
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro `ros2 pkg prefix --share babybot_description`/urdf/babybot.urdf)"
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
___



# Exercise 3: Writing a Hardware Component

You learn how to create hardware interface component by step-by-step guides.
### 3.1: Create babybot_firmware package

1. Create "babybot_firmware" package.
```
ros2 pkg create --build-type ament_cmake babybot_firmware
```
### 3.2: Setup the babybot_interface.hpp header file
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
5. Create source file, "babybot_interface.cpp" in the "src" folder.
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
