# Mastering ROS Essential

- [Exercise 1: ROS Development Setup](#exercise-1-ros-development-setup)
- [Exercise 2: Writing a Hardware Component](#exercise-2-writing-a-hardware-component)
  - [2.1: Create babybot_firmware package](#21-create-babybot-firmware-package)
  - [2.2: Setup the babybot_firmware.hpp header file](#22-setup-the-babybot-firmware-hpp-header-file)
  - [2.3: Implement babybot_interface.cpp source file](#23-implement-babybot-interface-cpp-source-file)
  - [2.4: Prepare for build the package](#24-prepare-for-build-the-package)
  - [2.5: Create export definition for pluginlib](#25-create-export-definition-for-pluginlib)
  - [2.6: Build the package](#26-build-the-package)


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


# Exercise 2: Writing a Hardware Component

You learn how to create hardware interface component by step-by-step guides.
### 2.1: Create babybot_firmware package

1. Create "babybot_firmware" package (ament_cmake), create "babybot_firmware" folder in "include" folder.
```
ros2 pkg create --build-type ament_cmake babybot_firmware
```
### 2.2: Setup the babybot_firmware.hpp header file
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

### 2.3: Implement babybot_interface.cpp source file
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

### 2.4: Prepare for build the package
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

### 2.5: Create export definition for pluginlib
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

### 2.6: Build the package
18. Build the package, make sure no error in the built.
```
cd ~/dev_ws
colcon build
```
