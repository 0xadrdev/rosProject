//==============================================================
// Filename : PongController.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include <cmath>
#include "PongController.h"
#include "../include/Constants.h"

using namespace pong_ros_constants;

PongController::PongController(double posX, double posY, double velX, double velY, double padLeft, double padRight, int col, double vel)
	: ballPositionX(posX), ballPositionY(posY), ballVelocityX(velX), ballVelocityY(velY), leftPaddlePosition(padLeft), rightPaddlePosition(padRight), collisionType(col), velocityIncrement(vel)
{}

// Depending the position of the ball a collisionType is setted. 
void PongController::determineCollisionType() {
	if (ballPositionX + 15 >= SCREEN_WIDTH && collisionType == NO_COLLISION) {
		collisionType = OUTSIDE_RIGHT_COLLISION; 
	} else if (ballPositionX - 15 <= 0 && collisionType == NO_COLLISION) {
		collisionType = OUTSIDE_LEFT_COLLISION;
	} else if (ballPositionY - BALL_SIZE / 2 <= WALL_HEIGHT && collisionType == NO_COLLISION) {							       
		collisionType = TOP_WALL_COLLISION; 	
	} else if (ballPositionY + BALL_SIZE / 2 >= SCREEN_HEIGHT - WALL_HEIGHT && collisionType == NO_COLLISION) {
		collisionType = BOTTOM_WALL_COLLISION;	
	} else if ((ballPositionX - BALL_SIZE / 4 <= PADDLE_WIDTH + BALL_SIZE / 4) && (abs(leftPaddlePosition - ballPositionY - BALL_SIZE / 4) <= PADDLE_HEIGHT / 2) && collisionType == NO_COLLISION){	
		collisionType = LEFT_PADDLE_COLLISION; 
	} else if ((ballPositionX + BALL_SIZE / 4 >= SCREEN_WIDTH - PADDLE_WIDTH - BALL_SIZE / 4) && (abs(rightPaddlePosition - ballPositionY - BALL_SIZE / 4) <= PADDLE_HEIGHT / 2) && collisionType == NO_COLLISION) {
		collisionType = RIGHT_PADDLE_COLLISION;	
	} else {
		collisionType = NO_COLLISION;
	}
}

// Increments the ball velocity if it bounces the paddles of the players.
// It resets the ball velocity when the ball bounces the outsides. 
void PongController::incrementVelocity() {
  if (collisionType == RIGHT_PADDLE_COLLISION || collisionType == LEFT_PADDLE_COLLISION) {
    velocityIncrement += 0.5;
  } else if (collisionType == OUTSIDE_LEFT_COLLISION || collisionType == OUTSIDE_RIGHT_COLLISION) {
    ballVelocityX = ballVelocityX < 0 ? -2 : 2;
    ballVelocityY = ballVelocityY < 0 ? -1 : 1;
    velocityIncrement = 1;
  }
}

// Depending on the collisionType atrbute updates the ballVelocity. 
void PongController::updateBallVelocity() {
	if (collisionType == TOP_WALL_COLLISION) {
    setBallVelocity(ballVelocityX, -ballVelocityY);
	} else if (collisionType == BOTTOM_WALL_COLLISION) {
    setBallVelocity(ballVelocityX, -ballVelocityY);				
	} else if (collisionType == LEFT_PADDLE_COLLISION) {								
    setBallVelocity(abs(ballVelocityX) * velocityIncrement, ballVelocityY * velocityIncrement);
	} else if (collisionType == RIGHT_PADDLE_COLLISION) {	
    setBallVelocity(-abs(ballVelocityX) * velocityIncrement, ballVelocityY * velocityIncrement);			
	} else if (collisionType == OUTSIDE_RIGHT_COLLISION) {
    setBallVelocity(-abs(ballVelocityY), ballVelocityY);
	} else if (collisionType == OUTSIDE_LEFT_COLLISION) {
    setBallVelocity(abs(ballVelocityX), ballVelocityY);
	} else { // NO_COLLISION	
    setBallVelocity(ballVelocityX, ballVelocityY);
	}
}

// Getters for private attributes. 
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

// Setters. 
void PongController::setBallVelocity(double x, double y) {
  ballVelocityX = x;
  ballVelocityY = y;
}

void PongController::setBallPosition(double x, double y) {
  ballPositionX = x;
  ballPositionY = y;
}

void PongController::setLeftPaddlePosition(double padLeft) {
	leftPaddlePosition = padLeft;
}

void PongController::setRightPaddlePosition(double padRight) {
	rightPaddlePosition = padRight;
}
