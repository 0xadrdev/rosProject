//==============================================================
// Filename : PongController.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description : PongController Class to manage the logic of the game. 
//==============================================================

#include <cmath>
#include "PongController.h"
#include "../include/Constants.h"

using namespace pong_ros_constants;

PongController::PongController(BallPhysics ball, double paddleLeft, double paddleRight, int col, double vel)
	: ballPhysics(ball), leftPaddlePosition(paddleLeft), rightPaddlePosition(paddleRight), collisionType(col), velocityIncrement(vel)
{}

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

void PongController::incrementVelocity() {
  if (collisionType == RIGHT_PADDLE_COLLISION || collisionType == LEFT_PADDLE_COLLISION) {
    velocityIncrement += 0.5;
  } else if (collisionType == OUTSIDE_LEFT_COLLISION || collisionType == OUTSIDE_RIGHT_COLLISION) {
    ballPhysics = BallPhysics();
    velocityIncrement = 1;
  }
}

void PongController::updateBallPosition() {
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
	} else if (collisionType == OUTSIDE_LEFT_COLLISION) {
    ballPhysics.setBallVelocity(abs(ballVelocityX), ballVelocityY);
    ballPhysics.setBallPosition(500, 300);
	} else { // NO_COLLISION	
    ballPhysics.setBallVelocity(ballVelocityX, ballVelocityY);
	}

  ballPhysics.setBallPosition(ballPhysics.getBallPositionX() + ballPhysics.getBallVelocityX(), ballPhysics.getBallPositionY() +  ballPhysics.getBallVelocityY());
}

double PongController::getBallPositionX() const {
	return ballPhysics.getBallPositionX();
}

double PongController::getBallPositionY() const {
	return ballPhysics.getBallPositionY();
}

int PongController::getCollisionType() const {
  return collisionType;
}

void PongController::setLeftPaddlePosition(double y) {
	leftPaddlePosition = y;
}

void PongController::setRightPaddlePosition(double y) {
	rightPaddlePosition = y;
}
