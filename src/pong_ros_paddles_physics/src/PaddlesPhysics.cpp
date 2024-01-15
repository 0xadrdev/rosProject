//==============================================================
// Filename : PaddlesPhysics.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description : PaddlesPhysics class to control the paddles physics. 
//==============================================================

#include "PaddlesPhysics.h"

PaddlesPhysics::PaddlesPhysics(double l, double r, double vl, double vr)
	: leftPaddlePosition(l), rightPaddlePosition(r), leftPaddleVelocity(vl), rightPaddleVelocity(vr)
{}

/**
 * @brief Updates the left paddle position to up.
 * 
 * @return nothing 
 */
void PaddlesPhysics::updateLeftPaddleUp() {
  leftPaddlePosition += leftPaddleVelocity;
}

/**
 * @brief Updates the left paddle position to down. 
 * 
 * @return nothing 
 */
void PaddlesPhysics::updateLeftPaddleDown() {
  leftPaddlePosition -= leftPaddleVelocity;
}

/**
 * @brief Updates the right paddle position to up.
 * 
 * @return nothing 
 */
void PaddlesPhysics::updateRightPaddleUp() {
  rightPaddlePosition += rightPaddleVelocity;
}

/**
 * @brief Updates the right paddle position to down. 
 * 
 * @return nothing 
 */
void PaddlesPhysics::updateRightPaddleDown() {
  rightPaddlePosition -= rightPaddleVelocity;
}

double PaddlesPhysics::getLeftPaddlePosition() {
  return leftPaddlePosition;
}

double PaddlesPhysics::getRightPaddlePosition() {
  return rightPaddlePosition;
}

void PaddlesPhysics::setRightPaddleVelocity(double v) { 
  rightPaddleVelocity = v;
}

