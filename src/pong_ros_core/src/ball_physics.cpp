//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "ball_physics.h"

ball_physics::ball_physics(double posX, double posY, double velX, double velY)
	: ballPosX(posX), ballPosY(posY), ballVelX(velX), ballVelY(velY)
{}

// Updating the position
void ball_physics::updatePosition(double velX, double velY) {
	// Computing new position
	double newX = ballPosX + velX;
	double newY = ballPosY + velY;
	
	// Updating the stored values
	//this->setBallPosX(newX);
	//this->setBallPosY(newY); 
	//setBallPosX(newX);
	//setBallPosY(newY); 
	ballPosX = newX;
	ballPosY = newY;
}

// Retrieving the class private data
double ball_physics::getBallPosX() const {
	return ballPosX;
}
double ball_physics::getBallPosY() const {
	return ballPosY;
}
double ball_physics::getBallVelX() const {
	return ballVelX; 
}
double ball_physics::getBallVelY() const {
	return ballVelY;
}

// Setting the class private data
void ball_physics::setBallPosX(double posX) {
	ballPosX = posX;
}
void ball_physics::setBallPosY(double posY) {
	ballPosY = posY;
}
void ball_physics::setBallVelX(double velX) {
	ballVelX = velX;
}
void ball_physics::setBallVelY(double velY) {
	ballVelY = velY;
}
