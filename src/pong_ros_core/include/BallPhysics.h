//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#ifndef BALL_PHYSICS_H
#define BALL_PHYSICS_H

class BallPhysics {
	
public:
	// Constructor
	BallPhysics(double posX = 500, double posY = 300, double velX = 0, double velY = 0); 
	
	void updateBallPosition(double velX, double velY);
	
	double getBallPositionX() const;
	double getBallPositionY() const;
	double getBallVelocityX() const;
	double getBallVelocityY() const;
	
	void setBallPositionX(double posX);
	void setBallPositionY(double posY);
	void setBallVelocityX(double velX);
	void setBallVelocityY(double velY);

private:
	// Variables storing ball position
	int collisionType = 0;
	double ballPositionX = 500; // Horizontal position of the center of the ball.
	double ballPositionY = 300; // Vertical position of the center of the ball.
	double ballVelocityX = 0; // Speed of the ball. 
	double ballVelocityY = 0; // Direction of movement of the ball. 
};

#endif // BALL_PHYSICS_H
	
	
	
	
	
