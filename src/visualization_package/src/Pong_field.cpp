/* Pong_field.cpp */
#include <string>
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

    // We draw each element in a different color. This helps with debugging.

    // Draw walls. They are static so we hard-code them here.
    sdl2_ui.setDrawColor(128, 128, 128, 255); // Walls are gray
    sdl2_ui.drawRectangle(0, 0, screenWidth - 1, wallHeight); // Top wall
    sdl2_ui.drawRectangle(0, screenHeight - wallHeight - 1, screenWidth - 1 , screenHeight - 1); // Bottom wall

    // Left bat
    sdl2_ui.setDrawColor(64, 64, 255, 255); // Left bat is bLue
    batCenter = (int) (170 * sin(yBatLeft / 100)) + screenHeight / 2;  // Just some stupid formula. Change yourself :-)
    // batCenter = (int) (yBatLeft);  // Just some stupid formula. Change yourself :-)
    sdl2_ui.drawRectangle(0, batCenter-batHeight/2, batWidth,  batCenter+batHeight/2);
    
    // Right bat
    sdl2_ui.setDrawColor(255,64,64, 255); // Right bat is Red
    batCenter = (int) (150 * cos(yBatRight / 140)) + screenHeight / 2;  // Just some stupid formula. Change yourself :-)
    // batCenter = (int) (yBatRight);  // Just some stupid formula. Change yourself :-)
    sdl2_ui.drawRectangle(screenWidth-1-batWidth, batCenter-batHeight/2, screenWidth-1,  batCenter+batHeight/2);
    
    // Ball
    sdl2_ui.setDrawColor(255,255,255, 255); // Ball is white
    int ballCenterX = (int) (150 * cos(xBall / 40)) + screenWidth / 2;  // Just some stupid formula. Change yourself :-)
    int ballCenterY = (int) (40 * cos(yBall / 35)) + screenHeight / 2;  // Just some stupid formula. Change yourself :-)
    // int ballCenterX = (int) (xBall);  // Just some stupid formula. Change yourself :-)
    // int ballCenterY = (int) (yBall);  // Just some stupid formula. Change yourself :-)
    sdl2_ui.drawRectangle(ballCenterX - ballSize / 2, ballCenterY - ballSize / 2, ballCenterX + ballSize / 2, ballCenterY+ballSize/2);
    
    // Text
    sdl2_ui.drawText(fieldText, screenWidth/2, screenHeight/4);

    // Show it on the screen
    sdl2_ui.present(); // This calls SDL2_UI::present
}

