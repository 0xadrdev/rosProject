//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#ifndef PONG_CONTROLLER_H
#define PONG_CONTROLLER_H

class PongController {
	
public:
	// Constructor
	PongController(double posX = 500, double posY = 300, double velX = 2, double velY = 1, double padLeft = 300, double padRight = 300, int col = 0); 
	
	void checkCollision();
	
	void updateBallVelocity();
	
	double getBallPositionX() const;
	double getBallPositionY() const;
	double getBallVelocityX() const;
	double getBallVelocityY() const;
	double getPadPosLeft() const;
	double getPadPosRight() const;
	
	void setBallPositionX(double posX);
	void setBallPositionY(double posY);
	void setBallVelocityX(double velX);
	void setBallVelocityY(double velY);
	void setLeftPaddlePosition(double velX);
	void setRightPaddlePosition(double velY);

private:
  // static const int BALL_SIZE = 30 / 2; // Make sure this is even


	// Variables storing ball position
	double ballPositionX;// = 500; // Horizontal position of the center of the ball.
	double ballPositionY;// = 300; // Vertical position of the center of the ball.
	double ballVelocityX; // = 0; // Speed of the ball. 
	double ballVelocityY;// = 0; // Direction of movement of the ball. 
	double padPosLeft;// = 300; // Speed of the ball. 
	double padPosRight;// = 300; // Direction of movement of the ball. 
	int collisionType;// = 0; // Variable storing the type of collision. 
};

#endif // BALL_PHYSICS_H
