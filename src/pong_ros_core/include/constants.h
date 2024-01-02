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

  constexpr int SCREEN_WIDTH = 1000;
  constexpr int SCREEN_HEIGHT = 600;
  constexpr int fontSize = 160;

  constexpr int WALL_HEIGHT = 15; // 30
  constexpr int PADDLE_WIDTH = 5; // 30 
  constexpr int PADDLE_HEIGHT = 100; // Make sure this is even
  constexpr int BALL_SIZE = 30; // Make sure this is even

  // You can also define constants for specific purposes like topic names
  const char* const TOPIC_NAVIGATION = "/navigation";
  const char* const TOPIC_DIAGNOSTICS = "/diagnostics";

}

#endif // CONSTANTS_H
