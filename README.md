Steps to compile and run the entire pong game. 
-----------------------------------------------------------------------------------------
This readme.txt file explains the steps needed to properly run each of the nodes
for the entire pong game to work. Make sure you have the webcam connected in the VM
-----------------------------------------------------------------------------------------
1. Unzip in an empty directory <pong_game_ws>
2. Adjust SDL library path in src/visualization_pkg/CMakeLists.txt
3. Go to <pong_game_ws>
4. colcon build
4. In new terminal:
	cd <pong_game_ws>
	. install/setup.bash
	ros2 run pong_core physics
5. In another new terminal:
	cd <pong_game_ws>
	. install/setup.bash
	ros2 run score score
6. In another new terminal:
	cd <pong_game_ws>
	. install/setup.bash
	ros2 run paddle_physics paddle_physics
7. In another new terminal:
	ros2 run image_tools cam2image history:=keep_last
6. In another new terminal:
	cd <pong_game_ws>
	. install/local_setup.sh
	ros2 launch keyboard_input keyboard_input_launch.py
7. in another new terminal:
	cd <pong_game_ws>
	. install/setup.bash
	ros2 launch image_input image
8. In new terminal:
	cd <pong_game_ws>
	. install/setup.bash
	ros2 run visualization_package visualization_package
9. In another new terminal:
	cd <pong_game_ws>
	. install/setup.bash
	ros2 run pong_core logic
10. Check the visualization and use the 'w' (upwards) and 's' (downwards) keys 
to control the left paddle, holding a light in front of the webcam at the top, 
or bottom directs the right paddle respectively. Have fun playing.  

	// Size settings. All in pixels.
        screenWidth = 1000;
        screenHeight = 600;

        wallHeight = 30;
        batWidth = 30;
        batHeight = 100; // Make sure this is even
        ballSize = 30/2; // Make sure this is even
