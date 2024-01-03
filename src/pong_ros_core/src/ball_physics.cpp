//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "ball_physics.h"

ball_physics::ball_physics(double posX, double posY, double velX, double velY)
	: ballPositionX(posX), ballPositionY(posY), ballVelocityX(velX), ballVelocityY(velY)
{}

// Updating the position
void ball_physics::updateBallPosition(double velX, double velY) {
	// Computing new position
	double new_ball_position_x = ballPositionX + velX;
	double new_ball_position_y = ballPositionY + velY;
	
	// Updating the stored values
	ballPositionX = new_ball_position_x;
	ballPositionY = new_ball_position_y;
}

// Retrieving the class private data
double ball_physics::getBallPositionX() const {
	return ballPositionX;
}
double ball_physics::getBallPositionY() const {
	return ballPositionY;
}
double ball_physics::getBallVelocityX() const {
	return ballVelocityX; 
}
double ball_physics::getBallVelocityY() const {
	return ballVelocityY;
}

// Setting the class private data
void ball_physics::setBallPositionX(double posX) {
	ballPositionX = posX;
}
void ball_physics::setBallPositionY(double posY) {
	ballPositionY = posY;
}
void ball_physics::setBallVelocityX(double velX) {
	ballVelocityX = velX;
}
void ball_physics::setBallVelocityY(double velY) {
	ballVelocityY = velY;
}
