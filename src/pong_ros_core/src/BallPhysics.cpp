//==============================================================
// Filename : BallPhysics.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "BallPhysics.h"

BallPhysics::BallPhysics(double posX, double posY, double velX, double velY)
	: ballPositionX(posX), ballPositionY(posY), ballVelocityX(velX), ballVelocityY(velY)
{}

// Getters.
double BallPhysics::getBallPositionX() const {
	return ballPositionX;
}
double BallPhysics::getBallPositionY() const {
	return ballPositionY;
}
double BallPhysics::getBallVelocityX() const {
	return ballVelocityX; 
}
double BallPhysics::getBallVelocityY() const {
	return ballVelocityY;
}

// Setters.
void BallPhysics::setBallPosition(double x, double y) {
  ballPositionX = x;
  ballPositionY = y;
}

void BallPhysics::setBallVelocity(double x, double y) {
  ballVelocityX = x;
  ballVelocityY = y;
}