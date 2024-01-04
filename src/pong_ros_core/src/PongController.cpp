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
		collisionType = OUTSIDE_RIGHT_COLLISION; 
	} else if (ballPositionX - 15 <= 0) {
		collisionType = OUTSIDE_LEFT_COLLISION;
	} else if (ballPositionY - BALL_SIZE / 2 <= WALL_HEIGHT) {							       
		collisionType = TOP_WALL_COLLISION; 	
	} else if (ballPositionY + BALL_SIZE / 2 >= SCREEN_HEIGHT - WALL_HEIGHT) {						// Bottom wall
		collisionType = BOTTOM_WALL_COLLISION;	
	} else if ((ballPositionX - BALL_SIZE / 4 <= PADDLE_WIDTH + BALL_SIZE / 4) && (abs(leftPaddlePosition - ballPositionY - BALL_SIZE / 4) <= PADDLE_HEIGHT / 2) ){	// Left bat
		collisionType = LEFT_PADDLE_COLLISION; 
	} else if ((ballPositionX + BALL_SIZE / 4 >= SCREEN_WIDTH - PADDLE_WIDTH - BALL_SIZE / 4) && (abs(rightPaddlePosition - ballPositionY - BALL_SIZE / 4) <= PADDLE_HEIGHT / 2)) {	// Right bat
		collisionType = RIGHT_PADDLE_COLLISION;	
	} else {
		collisionType = NO_COLLISION;
	}
}

void PongController::incrementVelocity() {
  if (collisionType == RIGHT_PADDLE_COLLISION || collisionType == LEFT_PADDLE_COLLISION) {
    velocityIncrement += 0.5;
  } else if (collisionType == OUTSIDE_LEFT_COLLISION || collisionType == OUTSIDE_RIGHT_COLLISION) {
    ballVelocityX = ballVelocityX < 0 ? -2 : 2;
    ballVelocityY = ballVelocityY < 0 ? -1 : 1;
    velocityIncrement = 1;
  }
}

void PongController::updateBallVelocity() {
	// On the basis of the collisionType type determine the reflection

	if (collisionType == TOP_WALL_COLLISION) {
    setBallVelocity(ballVelocityX, -ballVelocityY);
		// ballVelocityX = ballVelocityX;
		// ballVelocityY = -ballVelocityY;
	} else if (collisionType == BOTTOM_WALL_COLLISION) {
    setBallVelocity(ballVelocityX, -ballVelocityY);				
		// ballVelocityX = ballVelocityX;
		// ballVelocityY = -ballVelocityY;
	} else if (collisionType == LEFT_PADDLE_COLLISION) {								
    setBallVelocity(abs(ballVelocityX) * velocityIncrement, ballVelocityY * velocityIncrement);
		// ballVelocityX = abs(ballVelocityX) * velocityIncrement;
		// ballVelocityY = ballVelocityY * velocityIncrement;
	} else if (collisionType == RIGHT_PADDLE_COLLISION) {	
    setBallVelocity(-abs(ballVelocityX) * velocityIncrement, ballVelocityY * velocityIncrement);			
		// ballVelocityX = -abs(ballVelocityX) * velocityIncrement;
		// ballVelocityY = ballVelocityY * velocityIncrement;
	} else if (collisionType == OUTSIDE_RIGHT_COLLISION) {
    setBallVelocity(-abs(ballVelocityY), ballVelocityY);
		// ballVelocityX = -abs(ballVelocityX);
		// ballVelocityY = ballVelocityY;
	} else if (collisionType == OUTSIDE_LEFT_COLLISION) {
    setBallVelocity(abs(ballVelocityX), ballVelocityY);
		// ballVelocityX = abs(ballVelocityX);
		// ballVelocityY = ballVelocityY;
	} else { // NO_COLLISION	
    setBallVelocity(ballVelocityX, ballVelocityY);
		// ballVelocityX = ballVelocityX;
		// ballVelocityY = ballVelocityY;
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

void PongController::setBallVelocity(double x, double y) {
  ballVelocityX = x;
  ballVelocityY = y;
}

void PongController::setBallPosition(double x, double y) {
  ballPositionX = x;
  ballPositionY = y;
}

// Setting the class private data
// void PongController::setBallPositionX(double posX) {
// 	ballPositionX = posX;
// }
// void PongController::setBallPositionY(double posY) {
// 	ballPositionY = posY;
// }
// void PongController::setBallVelocityX(double velX) {
// 	ballVelocityX = velX;
// }
// void PongController::setBallVelocityY(double velY) {
// 	ballVelocityY = velY;
// }
void PongController::setLeftPaddlePosition(double padLeft) {
	leftPaddlePosition = padLeft;
}
void PongController::setRightPaddlePosition(double padRight) {
	rightPaddlePosition = padRight;
}
