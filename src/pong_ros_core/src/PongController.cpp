//==============================================================
// Filename : PongController.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "PongController.h"
#include "../include/Constants.h"

#include <cmath>

using namespace pong_ros_constants;

PongController::PongController(double posX, double posY, double velX, double velY, double padLeft, double padRight, int col)
	: ballPositionX(posX), ballPositionY(posY), ballVelocityX(velX), ballVelocityY(velY), leftPaddlePosition(padLeft), rightPaddlePosition(padRight), collisionType(col)
{}

// Checking the collisionType:
void PongController::checkCollision() {
	
	// Checking the collisionType type: 
	if (ballPositionX + 15 >= SCREEN_WIDTH) {
		collisionType = OUTSIDE_RIGHT_COLLISION; // outside right
	} else if (ballPositionX - 15 <= 0) {
		collisionType = OUTSIDE_LEFT_COLLISION; // outside left 
	} else if (ballPositionY - BALL_SIZE / 2 <= WALL_HEIGHT) {							        // Top wall
		collisionType = TOP_WALL_COLLISION; 	// collisionType with top wall indicator
	} else if (ballPositionY + BALL_SIZE / 2 >= SCREEN_HEIGHT - WALL_HEIGHT) {						// Bottom wall
		collisionType = BOTTOM_WALL_COLLISION;	// collisionType with bottom wall indicator
	} else if ((ballPositionX - BALL_SIZE / 4 <= PADDLE_WIDTH + BALL_SIZE / 4) && (abs(leftPaddlePosition - ballPositionY - BALL_SIZE / 4) <= PADDLE_HEIGHT / 2) ){	// Left bat
		collisionType = LEFT_PADDLE_COLLISION; 	// collisionType with left bat
	} else if ((ballPositionX + BALL_SIZE / 4 >= SCREEN_WIDTH - PADDLE_WIDTH - BALL_SIZE / 4) && (abs(rightPaddlePosition - ballPositionY - BALL_SIZE / 4) <= PADDLE_HEIGHT / 2)) {	// Right bat
		collisionType = RIGHT_PADDLE_COLLISION;	// collisionType with right bat
	} else {
		collisionType = NO_COLLISION; 	// No collisionType takes place
	}
}

void PongController::updateBallVelocity() {
	// On the basis of the collisionType type determine the reflection
	if (collisionType == TOP_WALL_COLLISION) {											// Top wall
		ballVelocityX = ballVelocityX;
		ballVelocityY = -ballVelocityY;
	} else if (collisionType == BOTTOM_WALL_COLLISION) {										// Bottom wall
		ballVelocityX = ballVelocityX;
		ballVelocityY = -ballVelocityY;
	} else if (collisionType == LEFT_PADDLE_COLLISION) {										// Left bat
		ballVelocityX = abs(ballVelocityX);
		ballVelocityY = ballVelocityY;
	} else if (collisionType == RIGHT_PADDLE_COLLISION) {										// Right bat
		ballVelocityX = -abs(ballVelocityX);
		ballVelocityY = ballVelocityY;
	} else if (collisionType == OUTSIDE_RIGHT_COLLISION) {										// Outside right
		ballVelocityX = -abs(ballVelocityX);
		ballVelocityY = ballVelocityY;
	} else if (collisionType == OUTSIDE_LEFT_COLLISION) {										// Outside left
		ballVelocityX = abs(ballVelocityX);
		ballVelocityY = ballVelocityY;
	} else {												// No collisionType
		ballVelocityX = ballVelocityX;
		ballVelocityY = ballVelocityY;
	}
}

// Retrieving the class private data
double PongController::getBallPositionX() const {
	return ballPositionX;
}
double PongController::getBallPositionY() const {
	return ballPositionY;
}
double PongController::getBallVelocityX() const {
	return ballVelocityX; 
}
double PongController::getBallVelocityY() const {
	return ballVelocityY;
}
double PongController::getLeftPaddlePosition() const {
	return leftPaddlePosition;
}
double PongController::getRightPaddlePosition() const {
	return rightPaddlePosition; 
}

// Setting the class private data
void PongController::setBallPositionX(double posX) {
	ballPositionX = posX;
}
void PongController::setBallPositionY(double posY) {
	ballPositionY = posY;
}
void PongController::setBallVelocityX(double velX) {
	ballVelocityX = velX;
}
void PongController::setBallVelocityY(double velY) {
	ballVelocityY = velY;
}
void PongController::setLeftPaddlePosition(double padLeft) {
	leftPaddlePosition = padLeft;
}
void PongController::setRightPaddlePosition(double padRight) {
	rightPaddlePosition = padRight;
}
