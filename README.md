Simulate Robot in ROS and Gazebo
1. Create democar_description package
```
ros2 pkg create --build-type ament_cmake democar_description
```

2. Create "urdf" folder in the package
3. Create democar.urdf file in "urdf" folder

4. base_link (car body)
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="autocar">
    <link name="base_link">
        <visual>
            <geometry><box size="0.6 0.4 0.2" /></geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </visual>
    </link>
</robot>
```
5. Install urdf-tutorial
```
apt update
apt install ros-humble-urdf-tutorial
```
6. Update CMakeLists.txt
```
install(
   DIRECTORY urdf 
   DESTINATION share/${PROJECT_NAME}/
)
```
7. Display URDF
```
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share autocar_description`/urdf/autocar.urdf
```
8. Typically base_footprint_link (imagine this is the ground) has no geometry and dimension, but link to the robot
```
<link name="base_footprint_link" />
<joint name="base_footprint_base_joint" type="fixed">
        <parent link="base_footprint_link" />
        <child link="base_link" />
        <origin xyz="0 0 0.1" rpy="0 0 0" />
</joint>
```
9. right_wheel_link and joint
```
<link name="right_wheel_link">
        <visual>
            <geometry> <cylinder radius="0.1" length="0.05" /></geometry>
            <origin xyz="0 0 0" rpy="-1.57 0 0" />
        </visual>
</link>
<joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="right_wheel_link" />
        <origin xyz="-0.15 -0.225 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
</joint>
```
10. Practice: Add left_wheel_link and joint
```
<link name="left_wheel_link">
        <visual>
            <geometry> <cylinder radius="0.1" length="0.05" /></geometry>
            <origin xyz="0 0 0" rpy="-1.57 0 0" />
        </visual>
</link>
<joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="left_wheel_link" />
        <origin xyz="-0.15 TODO:XXXX 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
</joint>
```
11. caster_wheel_link and joint
```
<link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="0.05" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </visual>
</link>
<joint name="base_caster_wheel_joint" type="fixed">
        <parent link="base_link" />
        <child link="caster_wheel_link" />
        <origin xyz="0.2  0 -0.05" rpy="0 0 0" />
</joint>

```
