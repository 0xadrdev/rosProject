//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "BallPhysics.h"

BallPhysics::BallPhysics(double posX, double posY, double velX, double velY)
	: ballPositionX(posX), ballPositionY(posY), ballVelocityX(velX), ballVelocityY(velY)
{}

// Updating the position
void BallPhysics::updateBallPosition(double velX, double velY) {
	// Computing new position
	double new_ball_position_x = ballPositionX + velX;
	double new_ball_position_y = ballPositionY + velY;
	
	// Updating the stored values
	ballPositionX = new_ball_position_x;
	ballPositionY = new_ball_position_y;
}

// Retrieving the class private data
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

// Setting the class private data
void BallPhysics::setBallPositionX(double posX) {
	ballPositionX = posX;
}
void BallPhysics::setBallPositionY(double posY) {
	ballPositionY = posY;
}
void BallPhysics::setBallVelocityX(double velX) {
	ballVelocityX = velX;
}
void BallPhysics::setBallVelocityY(double velY) {
	ballVelocityY = velY;
}
