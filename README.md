# Simulate Robot in ROS RViz
1. Create democar_description package
```
ros2 pkg create --build-type ament_cmake democar_description
```
2. Create "urdf" folder in the package
3. Create "democar.urdf" file in "urdf" folder
4. base_link (car body in a box shape), the dimension is 0.6 x 0.4 x 0.2
```
<?xml version="1.0"?>
<robot name="autocar">

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
6. Update CMakeLists.txt, so that ROS will copy the "urdf" folder to share
```
install(
   DIRECTORY urdf 
   DESTINATION share/${PROJECT_NAME}/
)
```
7. Colcon build, source the setup.bash, display the robot (How to offset the box above the ground?)
```
colcon build
source ~/dev_ws/install/setup.bash
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share democar_description`/urdf/democar.urdf
```
<img src="https://github.com/twming/ros2_master_tutorial/blob/demo/img/img1.png" alt="img1" width="600">

<img src="https://github.com/twming/ros2_master_tutorial/blob/demo/img/img2.png" alt="img2" width="600">

8. Typically, base_footprint_link (imagine this is the ground) has no geometry and dimension, but link to the robot. base_link is offset above base_footprint_link by 0.1 (caster wheel diameter)
```
<link name="base_footprint_link" />

<joint name="base_footprint_base_joint" type="fixed">
        <parent link="base_footprint_link" />
        <child link="base_link" />
        <origin xyz="0 0 0.1" rpy="0 0 0" />
</joint>
```
<img src="https://github.com/twming/ros2_master_tutorial/blob/demo/img/img3.png" alt="img3" width="600">

<img src="https://github.com/twming/ros2_master_tutorial/blob/demo/img/img3a.png" alt="img3a" width="600">

9. right_wheel_link and joint, joint offset by (x= -0.15, y= -0.225, (base width + wheel length/2) ). right_wheel_link is attached to the joint, but rotate 90 degree. 
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

<img src="https://github.com/twming/ros2_master_tutorial/blob/demo/img/img4.png" alt="img4" width="600">

10. Practice: Add left_wheel_link and joint, fill in the TODO:XXXX (opposite side)
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

<img src="https://github.com/twming/ros2_master_tutorial/blob/demo/img/img4a.png" alt="img4a" width="600">

11. caster_wheel_link and joint (radius = 0.05, diameter = 0.1)
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
<img src="https://github.com/twming/ros2_master_tutorial/blob/demo/img/img5.png" alt="img5" width="600">
