//==============================================================
// Filename : PaddlesController.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description : PaddlesController class to control the paddles physics. 
//==============================================================

#include "PaddlesController.h"

PaddlesController::PaddlesController(double l, double r, double vl, double vr)
	: leftPaddlePosition(l), rightPaddlePosition(r), leftPaddleVelocity(vl), rightPaddleVelocity(vr)
{}

/**
 * @brief Updates the left paddle position to up.
 * 
 * @return nothing 
 */
void PaddlesController::updateLeftPaddleUp() {
  leftPaddlePosition += leftPaddleVelocity;
}

/**
 * @brief Updates the left paddle position to down. 
 * 
 * @return nothing 
 */
void PaddlesController::updateLeftPaddleDown() {
  leftPaddlePosition -= leftPaddleVelocity;
}

/**
 * @brief Updates the right paddle position to up.
 * 
 * @return nothing 
 */
void PaddlesController::updateRightPaddleUp() {
  rightPaddlePosition += rightPaddleVelocity;
}

/**
 * @brief Updates the right paddle position to down. 
 * 
 * @return nothing 
 */
void PaddlesController::updateRightPaddleDown() {
  rightPaddlePosition -= rightPaddleVelocity;
}

double PaddlesController::getLeftPaddlePosition() {
  return leftPaddlePosition;
}

double PaddlesController::getRightPaddlePosition() {
  return rightPaddlePosition;
}

void PaddlesController::setRightPaddleVelocity(double v) { 
  rightPaddleVelocity = v;
}

