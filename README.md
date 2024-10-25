### Launch the ROSBot XL
source ~/ros2_ws/install/setup.bash <br>
ros2 launch rosbot_xl_gazebo simulation.launch.py

### Run the wheel velocities publisher
source ~/ros2_ws/install/setup.bash <br>
ros2 run wheel_velocities_publisher wheel_velocities_publisher

### Launch the Kinematic Model
source ~/ros2_ws/install/setup.bash <br>
ros2 launch kinematic_model kinematic_model.launch.py

### Launch the eight trajectory
source ~/ros2_ws/install/setup.bash <br> 
ros2 launch eight_trajectory eight_trajectory.launch.py
