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

void PaddlesController::updateLeftPaddleUp() {
  leftPaddlePosition += leftPaddleVelocity;
}

void PaddlesController::updateLeftPaddleDown() {
  leftPaddlePosition -= leftPaddleVelocity;
}

void PaddlesController::updateRightPaddleUp() {
  rightPaddlePosition += rightPaddleVelocity;
}

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

