//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "logic.h"
#include <cmath>

logic::logic(double posX, double posY, double velX, double velY, double padLeft, double padRight, int col)
	: ballPosX(posX), ballPosY(posY), ballVelX(velX), ballVelY(velY), padPosLeft(padLeft), padPosRight(padRight), collision(col)
{}

// Checking the collision:
void logic::checkCollision() {
	
	// Checking the collision type: 
	if (ballPosX >= SCREEN_WIDTH + 15) {
		collision = 5;
	} else if (ballPosX <= -15) {
		collision = 6;
	} else if (ballPosY - BALL_SIZE <= WALL_HEIGHT) {							        // Top wall
		collision = 1; 	// Collision with top wall indicator
	} else if (ballPosY + BALL_SIZE / 2 >= SCREEN_HEIGHT - WALL_HEIGHT) {						// Bottom wall
		collision = 2;	// Collision with bottom wall indicator
	} else if ((ballPosX - BALL_SIZE / 2 <= PADDLE_WIDTH + BALL_SIZE / 2) && (abs(padPosLeft - ballPosY - BALL_SIZE / 2) <= PADDLE_HEIGHT / 2) ){	// Left bat
		collision = 3; 	// Collision with left bat
	} else if ((ballPosX + BALL_SIZE / 2 >= SCREEN_WIDTH - PADDLE_WIDTH - BALL_SIZE / 2) && (abs(padPosRight - ballPosY - BALL_SIZE / 2) <= PADDLE_HEIGHT / 2)) {	// Right bat
		collision = 4;	// Collision with right bat
	} else {
		collision = 0; 	// No collision takes place
	}
	
	// Updating the stored values, dont think this is needed as the member function can access private members. 
	//this->setCollision(collision); 
}

void logic::updateVelocity() {
	// On the basis of the collision type determine the reflection
	if (collision == 1) {											// Top wall
		ballVelX = ballVelX;
		ballVelY = -ballVelY;
	} else if (collision == 2) {										// Bottom wall
		ballVelX = ballVelX;
		ballVelY = -ballVelY;
	} else if (collision == 3) {										// Left bat
		ballVelX = abs(ballVelX);
		ballVelY = ballVelY + 1;
	} else if (collision == 4) {										// Right bat
		ballVelX = -abs(ballVelX);
		ballVelY = ballVelY + 1;
	} else if (collision == 5) {										// Outside right
		ballVelX = -abs(ballVelX);
		ballVelY = ballVelY;
	} else if (collision == 6) {										// Outside left
		ballVelX = abs(ballVelX);
		ballVelY = ballVelY;
	} else {												// No collision
		ballVelX = ballVelX;
		ballVelY = ballVelY;
	}
}

// Retrieving the class private data
double logic::getBallPosX() const {
	return ballPosX;
}
double logic::getBallPosY() const {
	return ballPosY;
}
double logic::getBallVelX() const {
	return ballVelX; 
}
double logic::getBallVelY() const {
	return ballVelY;
}
double logic::getPadPosLeft() const {
	return padPosLeft;
}
double logic::getPadPosRight() const {
	return padPosRight; 
}

// Setting the class private data
void logic::setBallPosX(double posX) {
	ballPosX = posX;
}
void logic::setBallPosY(double posY) {
	ballPosY = posY;
}
void logic::setBallVelX(double velX) {
	ballVelX = velX;
}
void logic::setBallVelY(double velY) {
	ballVelY = velY;
}
void logic::setPadPosLeft(double padLeft) {
	padPosLeft = padLeft;
}
void logic::setPadPosRight(double padRight) {
	padPosRight = padRight;
}
