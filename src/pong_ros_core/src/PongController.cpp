//==============================================================
// Filename :
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
	: ballPositionX(posX), ballPositionY(posY), ballVelocityX(velX), ballVelocityY(velY), padPosLeft(padLeft), padPosRight(padRight), collisionType(col)
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
	} else if ((ballPositionX - BALL_SIZE / 4 <= PADDLE_WIDTH + BALL_SIZE / 4) && (abs(padPosLeft - ballPositionY - BALL_SIZE / 4) <= PADDLE_HEIGHT / 2) ){	// Left bat
		collisionType = LEFT_PADDLE_COLLISION; 	// collisionType with left bat
	} else if ((ballPositionX + BALL_SIZE / 4 >= SCREEN_WIDTH - PADDLE_WIDTH - BALL_SIZE / 4) && (abs(padPosRight - ballPositionY - BALL_SIZE / 4) <= PADDLE_HEIGHT / 2)) {	// Right bat
		collisionType = RIGHT_PADDLE_COLLISION;	// collisionType with right bat
	} else {
		collisionType = NO_COLLISION; 	// No collisionType takes place
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
double PongController::getPadPosLeft() const {
	return padPosLeft;
}
double PongController::getPadPosRight() const {
	return padPosRight; 
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
	padPosLeft = padLeft;
}
void PongController::setRightPaddlePosition(double padRight) {
	padPosRight = padRight;
}
