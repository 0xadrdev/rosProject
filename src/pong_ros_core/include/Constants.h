//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================
#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace pong_ros_constants {

  // Window constants. 

  constexpr char SCREEN_TITLE[] = "Classic ping pong game in ROS2. ";
  constexpr int SCREEN_WIDTH = 1000;
  constexpr int SCREEN_HEIGHT = 600;
  constexpr int FONT_SIZE = 160;
  
  // Pong game constants. 

  constexpr int WALL_HEIGHT = 15; // 30
  constexpr int PADDLE_WIDTH = 5; // 30 
  constexpr int PADDLE_HEIGHT = 100; // Make sure this is even
  constexpr int BALL_SIZE = 30; // Make sure this is even

  // Collisions types. 

  constexpr int NO_COLLISION = 0;
  constexpr int TOP_WALL_COLLISION = 1;
  constexpr int BOTTOM_WALL_COLLISION = 2;
  constexpr int LEFT_PADDLE_COLLISION = 3;
  constexpr int RIGHT_PADDLE_COLLISION = 4;
  constexpr int OUTSIDE_RIGHT_COLLISION = 5;
  constexpr int OUTSIDE_LEFT_COLLISION = 6;

  // You can also define constants for specific purposes like topic names
  const char* const TOPIC_NAVIGATION = "/navigation";
  const char* const TOPIC_DIAGNOSTICS = "/diagnostics";

}

#endif // CONSTANTS_H
