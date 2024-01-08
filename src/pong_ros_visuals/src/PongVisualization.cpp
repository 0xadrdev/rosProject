
#include <string>

#include "../include/PongVisualization.h"
#include "../../pong_ros_core/include/Constants.h"

using namespace pong_ros_constants;


/** Set the vertical position of the left bat.
 * Units are up to the project group.
 */
void PongVisualization::setLeftPaddlePosition(double v_) {
  leftPaddlePosition = v_;
}

/** Set the vertical position of the right bat.
 * Units are up to the project group.
 */
void PongVisualization::setRightPaddlePosition(double v_) {
  rightPaddlePosition = v_;
}

/** Set the horizontal and vertical position of the (center of) the ball.
 * Units are up to the project group.
 */
void PongVisualization::setBallPosition(double x_, double y_) {
  ballPositionX = x_;
  ballPositionY = y_;
}

/** Set the text to display in the middle of the field.
 *  You can use this to show the score, "0-4"
 *  or any other text, "Game over".
 *  The text is shown until a new text is set.
 *  To remove the text, send an empty string, "".
 */
void PongVisualization::setFieldText(std::string s_) {
  fieldText = s_;
}

/** Draw the current state of all objects on the screen. Just call this
 * function regularly and you'll be fine...
 */
void PongVisualization::render() {
  int batCenter;

  pong_ros_sdl.clearWindow(); // Clear the draw buffer. This calls pong_ros_sdl::clear.
  pong_ros_sdl.handleWindowEvents();

  // We draw each element in a different color. This helps with debugging.

  // Draw walls. They are static so we hard-code them here.
  pong_ros_sdl.setColor(255, 255, 255, 255);
  pong_ros_sdl.setRectangle(SCREEN_WIDTH / 2 - 5 / 2, 0, SCREEN_WIDTH / 2 + 5, SCREEN_HEIGHT); // Middle wall. 
  pong_ros_sdl.setRectangle(0, 0, SCREEN_WIDTH - 1, WALL_HEIGHT); // Top wall
  pong_ros_sdl.setRectangle(0, SCREEN_HEIGHT - WALL_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT); // Bottom wall

  // Left bat
  pong_ros_sdl.setColor(255, 255, 255, 255);
  // batCenter = (int)(170 * sin(leftPaddlePosition / 100)) + SCREEN_HEIGHT / 2; // Just some stupid formula. Change yourself :-)
  batCenter = (int) (leftPaddlePosition);  // Just some stupid formula. Change yourself :-)
  pong_ros_sdl.setRectangle(0, batCenter - PADDLE_HEIGHT / 2, PADDLE_WIDTH, batCenter + PADDLE_HEIGHT / 2);

  // Right bat
  pong_ros_sdl.setColor(255, 255, 255, 255);
  // batCenter = (int)(150 * cos(rightPaddlePosition / 140)) + SCREEN_HEIGHT / 2; // Just some stupid formula. Change yourself :-)
  batCenter = (int) (rightPaddlePosition);  // Just some stupid formula. Change yourself :-)
  pong_ros_sdl.setRectangle(SCREEN_WIDTH - 1 - PADDLE_WIDTH, batCenter - PADDLE_HEIGHT / 2, SCREEN_WIDTH - 1, batCenter + PADDLE_HEIGHT / 2);

  // Ball
  pong_ros_sdl.setColor(255, 255, 255, 255); // Ball is white
  // int ballCenterX = (int)(150 * cos(ballPositionX / 40)) + SCREEN_WIDTH / 2; // Just some stupid formula. Change yourself :-)
  // int ballCenterY = (int)(40 * cos(ballPositionY / 35)) + SCREEN_HEIGHT / 2; // Just some stupid formula. Change yourself :-)
  int ballCenterX = (int) (ballPositionX);  // Just some stupid formula. Change yourself :-)
  int ballCenterY = (int) (ballPositionY);  // Just some stupid formula. Change yourself :-)
  pong_ros_sdl.setRectangle(ballCenterX - BALL_SIZE / 2, ballCenterY - BALL_SIZE / 2, ballCenterX + BALL_SIZE / 2, ballCenterY + BALL_SIZE / 2);

  // Text
  pong_ros_sdl.setText(fieldText, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 4 - 50);

  // Show it on the screen
  pong_ros_sdl.render(); // This calls pong_ros_sdl::present
}