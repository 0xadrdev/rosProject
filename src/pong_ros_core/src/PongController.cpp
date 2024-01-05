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

// PongController::PongController(double posX, double posY, double velX, double velY, double padLeft, double padRight, int col, double vel)
// 	: ballPositionX(posX), ballPositionY(posY), ballVelocityX(velX), ballVelocityY(velY), leftPaddlePosition(padLeft), rightPaddlePosition(padRight), collisionType(col), velocityIncrement(vel)
// {}

PongController::PongController(BallPhysics ball, double padLeft, double padRight, int col, double vel)
	: ballPhysics(ball), leftPaddlePosition(padLeft), rightPaddlePosition(padRight), collisionType(col), velocityIncrement(vel)
{}

// Depending the position of the ball a collisionType is setted. 
void PongController::determineCollisionType() {
  double ballPositionX = ballPhysics.getBallPositionX();
  double ballPositionY = ballPhysics.getBallPositionY();

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

int PongController::getCollisionType() const {
  return collisionType;
}

// Increments the ball velocity if it bounces the paddles of the players.
// It resets the ball velocity when the ball bounces the outsides. 
void PongController::incrementVelocity() {

  if (collisionType == RIGHT_PADDLE_COLLISION || collisionType == LEFT_PADDLE_COLLISION) {
    velocityIncrement += 0.5;
  } else if (collisionType == OUTSIDE_LEFT_COLLISION || collisionType == OUTSIDE_RIGHT_COLLISION) {
    // double ballVelocityX = ballPhysics.getBallVelocityX() < 0 ? -2 : 2;
    // double ballVelocityY = ballPhysics.getBallVelocityY() < 0 ? -1 : 1;

    // ballPhysics.setBallVelocity(ballVelocityX, ballVelocityY);
    ballPhysics = BallPhysics();
    velocityIncrement = 1;
  }
}

// Depending on the collisionType atrbute updates the ballVelocity. 
void PongController::updateBallVelocity() {
  double ballVelocityX = ballPhysics.getBallVelocityX();
  double ballVelocityY = ballPhysics.getBallVelocityY();

	if (collisionType == TOP_WALL_COLLISION) {
    ballPhysics.setBallVelocity(ballVelocityX, -ballVelocityY);
	} else if (collisionType == BOTTOM_WALL_COLLISION) {
    ballPhysics.setBallVelocity(ballVelocityX, -ballVelocityY);				
	} else if (collisionType == LEFT_PADDLE_COLLISION) {								
    ballPhysics.setBallVelocity(abs(ballVelocityX) * velocityIncrement, ballVelocityY * velocityIncrement);
	} else if (collisionType == RIGHT_PADDLE_COLLISION) {	
    ballPhysics.setBallVelocity(-abs(ballVelocityX) * velocityIncrement, ballVelocityY * velocityIncrement);			
	} else if (collisionType == OUTSIDE_RIGHT_COLLISION) {
    ballPhysics.setBallVelocity(-abs(ballVelocityY), ballVelocityY);
    ballPhysics.setBallPosition(500, 300);
    // ballPhysics = BallPhysics();
	} else if (collisionType == OUTSIDE_LEFT_COLLISION) {
    ballPhysics.setBallVelocity(abs(ballVelocityX), ballVelocityY);
    // ballPhysics = BallPhysics();
    ballPhysics.setBallPosition(500, 300);
	} else { // NO_COLLISION	
    ballPhysics.setBallVelocity(ballVelocityX, ballVelocityY);
	}

  ballPhysics.setBallPosition(ballPhysics.getBallPositionX() + ballPhysics.getBallVelocityX(), ballPhysics.getBallPositionY() +  ballPhysics.getBallVelocityY());
}

// Getters for private attributes. 
double PongController::getBallPositionX() const {
	return ballPhysics.getBallPositionX();
}

double PongController::getBallPositionY() const {
	return ballPhysics.getBallPositionY();
}

// double PongController::getBallVelocityX() const {
// 	return ballVelocityX; 
// }

// double PongController::getBallVelocityY() const {
// 	return ballVelocityY;
// }

double PongController::getLeftPaddlePosition() const {
	return leftPaddlePosition;
}

double PongController::getRightPaddlePosition() const {
	return rightPaddlePosition; 
}

// Setters. 
// void PongController::setBallVelocity(double x, double y) {
//   ballVelocityX = x;
//   ballVelocityY = y;
// }

// void PongController::setBallPosition(double x, double y) {
//   ballPositionX = x;
//   ballPositionY = y;
// }

void PongController::setLeftPaddlePosition(double padLeft) {
	leftPaddlePosition = padLeft;
}

void PongController::setRightPaddlePosition(double padRight) {
	rightPaddlePosition = padRight;
}
