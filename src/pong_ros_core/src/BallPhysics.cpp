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

// Getters for private atributes. 
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

// Setter for private atributes.
void BallPhysics::setBallPosition(double x, double y) {
  ballPositionX = x;
  ballPositionY = y;
}