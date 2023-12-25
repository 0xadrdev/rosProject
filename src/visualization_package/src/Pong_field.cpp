/* Pong_field.cpp */ #include <string>

#include "../include/Pong_field.h"


/** Set the vertical position of the left bat.
 * Units are up to the project group.
 */
void Pong_field::setYBatLeft(double v_) {
  yBatLeft = v_;
}

/** Set the vertical position of the right bat.
 * Units are up to the project group.
 */
void Pong_field::setYBatRight(double v_) {
  yBatRight = v_;
}

/** Set the horizontal and vertical position of the (center of) the ball.
 * Units are up to the project group.
 */
void Pong_field::setXYBall(double x_, double y_) {
  xBall = x_;
  yBall = y_;
}

/** Set the text to display in the middle of the field.
 *  You can use this to show the score, "0-4"
 *  or any other text, "Game over".
 *  The text is shown until a new text is set.
 *  To remove the text, send an empty string, "".
 */
void Pong_field::setFieldText(std::string s_) {
  fieldText = s_;
}

/** Draw the current state of all objects on the screen. Just call this
 * function regularly and you'll be fine...
 */
void Pong_field::DrawField() {
  int batCenter;

  sdl2_ui.clear(); // Clear the draw buffer. This calls SDL2_UI::clear.
  sdl2_ui.processEvents();

  // We draw each element in a different color. This helps with debugging.

  // TODO: Middle Wall.

  // Draw walls. They are static so we hard-code them here.
  sdl2_ui.setDrawColor(255, 255, 255, 255);
  sdl2_ui.drawRectangle(0, 0, SCREEN_WIDTH - 1, WALL_HEIGHT); // Top wall
  sdl2_ui.drawRectangle(0, SCREEN_HEIGHT - WALL_HEIGHT - 1, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1); // Bottom wall

  // Left bat
  sdl2_ui.setDrawColor(255, 255, 255, 255);
  // batCenter = (int)(170 * sin(yBatLeft / 100)) + SCREEN_HEIGHT / 2; // Just some stupid formula. Change yourself :-)
  batCenter = (int) (yBatLeft);  // Just some stupid formula. Change yourself :-)
  sdl2_ui.drawRectangle(0, batCenter - PADDLE_HEIGHT / 2, PADDLE_WIDTH, batCenter + PADDLE_HEIGHT / 2);

  // Right bat
  sdl2_ui.setDrawColor(255, 255, 255, 255);
  // batCenter = (int)(150 * cos(yBatRight / 140)) + SCREEN_HEIGHT / 2; // Just some stupid formula. Change yourself :-)
  batCenter = (int) (yBatRight);  // Just some stupid formula. Change yourself :-)
  sdl2_ui.drawRectangle(SCREEN_WIDTH - 1 - PADDLE_WIDTH, batCenter - PADDLE_HEIGHT / 2, SCREEN_WIDTH - 1, batCenter + PADDLE_HEIGHT / 2);

  // Ball
  sdl2_ui.setDrawColor(255, 255, 255, 255); // Ball is white
  // int ballCenterX = (int)(150 * cos(xBall / 40)) + SCREEN_WIDTH / 2; // Just some stupid formula. Change yourself :-)
  // int ballCenterY = (int)(40 * cos(yBall / 35)) + SCREEN_HEIGHT / 2; // Just some stupid formula. Change yourself :-)
  int ballCenterX = (int) (xBall);  // Just some stupid formula. Change yourself :-)
  int ballCenterY = (int) (yBall);  // Just some stupid formula. Change yourself :-)
  sdl2_ui.drawRectangle(ballCenterX - BALL_SIZE / 2, ballCenterY - BALL_SIZE / 2, ballCenterX + BALL_SIZE / 2, ballCenterY + BALL_SIZE / 2);

  // Text
  sdl2_ui.drawText(fieldText, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 4);

  // Show it on the screen
  sdl2_ui.present(); // This calls SDL2_UI::present
}