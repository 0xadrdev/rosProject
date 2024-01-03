//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "logic.h"
#include "../include/constants.h"

#include <cmath>

using namespace pong_ros_constants;

logic::logic(double posX, double posY, double velX, double velY, double padLeft, double padRight, int col)
	: ballPositionX(posX), ballPositionY(posY), ballVelocityX(velX), ballVelocityY(velY), padPosLeft(padLeft), padPosRight(padRight), collisionType(col)
{}

// Checking the collisionType:
void logic::checkCollision() {
	
	// Checking the collisionType type: 
	if (ballPositionX + 15 >= SCREEN_WIDTH) {
		collisionType = 5; // outside right
	} else if (ballPositionX - 15 <= 0) {
		collisionType = 6; // outside left 
	} else if (ballPositionY - BALL_SIZE / 2 <= WALL_HEIGHT) {							        // Top wall
		collisionType = 1; 	// collisionType with top wall indicator
	} else if (ballPositionY + BALL_SIZE / 2 >= SCREEN_HEIGHT - WALL_HEIGHT) {						// Bottom wall
		collisionType = 2;	// collisionType with bottom wall indicator
	} else if ((ballPositionX - BALL_SIZE / 4 <= PADDLE_WIDTH + BALL_SIZE / 4) && (abs(padPosLeft - ballPositionY - BALL_SIZE / 4) <= PADDLE_HEIGHT / 2) ){	// Left bat
		collisionType = 3; 	// collisionType with left bat
	} else if ((ballPositionX + BALL_SIZE / 4 >= SCREEN_WIDTH - PADDLE_WIDTH - BALL_SIZE / 4) && (abs(padPosRight - ballPositionY - BALL_SIZE / 4) <= PADDLE_HEIGHT / 2)) {	// Right bat
		collisionType = 4;	// collisionType with right bat
	} else {
		collisionType = 0; 	// No collisionType takes place
	}

  // if (ballPosX >= screenWidth + 15) {
	// 	collision = 5;
	// } else if (ballPosX <= -15) {
	// 	collision = 6;
	// } else if (ballPosY - ballSize <= wallHeight) {							        // Top wall
	// 	collision = 1; 	// Collision with top wall indicator
	// } else if (ballPosY + ballSize/2 >= screenHeight - wallHeight) {						// Bottom wall
	// 	collision = 2;	// Collision with bottom wall indicator
	// } else if ((ballPosX - ballSize/2 <= batWidth + ballSize/2) && (abs(padPosLeft - ballPosY - ballSize/2) <= batHeight/2) ){	// Left bat
	// 	collision = 3; 	// Collision with left bat
	// } else if ((ballPosX + ballSize/2 >= screenWidth - batWidth - ballSize/2) && (abs(padPosRight - ballPosY - ballSize/2) <= batHeight/2)) {	// Right bat
	// 	collision = 4;	// Collision with right bat
	// } else {
	// 	collision = 0; 	// No collision takes place
	// }
}

void logic::updateBallVelocity() {
	// On the basis of the collisionType type determine the reflection
	if (collisionType == 1) {											// Top wall
		ballVelocityX = ballVelocityX;
		ballVelocityY = -ballVelocityY;
	} else if (collisionType == 2) {										// Bottom wall
		ballVelocityX = ballVelocityX;
		ballVelocityY = -ballVelocityY;
	} else if (collisionType == 3) {										// Left bat
		ballVelocityX = abs(ballVelocityX);
		ballVelocityY = ballVelocityY;
	} else if (collisionType == 4) {										// Right bat
		ballVelocityX = -abs(ballVelocityX);
		ballVelocityY = ballVelocityY;
	} else if (collisionType == 5) {										// Outside right
		ballVelocityX = -abs(ballVelocityX);
		ballVelocityY = ballVelocityY;
	} else if (collisionType == 6) {										// Outside left
		ballVelocityX = abs(ballVelocityX);
		ballVelocityY = ballVelocityY;
	} else {												// No collisionType
		ballVelocityX = ballVelocityX;
		ballVelocityY = ballVelocityY;
	}
}

// Retrieving the class private data
double logic::getBallPositionX() const {
	return ballPositionX;
}
double logic::getBallPositionY() const {
	return ballPositionY;
}
double logic::getBallVelocityX() const {
	return ballVelocityX; 
}
double logic::getBallVelocityY() const {
	return ballVelocityY;
}
double logic::getPadPosLeft() const {
	return padPosLeft;
}
double logic::getPadPosRight() const {
	return padPosRight; 
}

// Setting the class private data
void logic::setBallPositionX(double posX) {
	ballPositionX = posX;
}
void logic::setBallPositionY(double posY) {
	ballPositionY = posY;
}
void logic::setBallVelocityX(double velX) {
	ballVelocityX = velX;
}
void logic::setBallVelocityY(double velY) {
	ballVelocityY = velY;
}
void logic::setPadPosLeft(double padLeft) {
	padPosLeft = padLeft;
}
void logic::setPadPosRight(double padRight) {
	padPosRight = padRight;
}
