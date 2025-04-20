echo "Starting launch_interface.sh"
source /home/hello-robot/ada_pet_capstone/install/setup.sh

echo "Running stretch_system_check.py"
stretch_system_check.py

echo "Homeing the robot"
stretch_robot_home.py

echo "Launching web teleop"
ros2 launch web_teleop teleop.launch.py
