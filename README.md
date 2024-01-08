Steps to compile and run the entire pong game. 
-----------------------------------------------------------------------------------------
This README.md explains how to run he pong game. The two paddles can be controlled by the
keyboard, while the right paddle can be controlled also with the camera. 
-----------------------------------------------------------------------------------------
1. Unzip in an empty directory <rosProject>
2. Adjust SDL library path in src/visualization_pkg/CMakeLists.txt
3. Go to <rosProject>
4. colcon build
5. In a new terminal: 
	cd <rosProject>
	. install/local_setup.sh
	ros2 launch pong_ros_keyboard_input keyboard_input_launch.py
6. In new terminal:
	cd <rosProject>
	. install/setup.bash
	ros2 launch pong_ros_bringup _launch.xml

Until this point the two paddles can be controlled with the keyboard. If you want to 
control the right paddle with flash light, do: 

1. In a new terminal: 
  ros2 run image_tools cam2image history:=keep_last
2. In a new terminal: 
	cd <rosProject>
  ros2 run pong_ros_image_input image