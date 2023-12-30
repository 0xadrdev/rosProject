//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#ifndef BALL_PHYSICS_H
#define BALL_PHYSICS_H

class ball_physics {
	
public:
	// Constructor
	ball_physics(double posX = 500, double posY = 300, double velX = 0, double velY = 0); 
	
	void updatePosition(double velX, double velY);
	
	double getBallPosX() const;
	double getBallPosY() const;
	double getBallVelX() const;
	double getBallVelY() const;
	
	void setBallPosX(double posX);
	void setBallPosY(double posY);
	void setBallVelX(double velX);
	void setBallVelY(double velY);

private:
	// Variables storing ball position
	double ballPosX = 500; // Horizontal position of the center of the ball.
	double ballPosY = 300; // Vertical position of the center of the ball.
	double ballVelX = 0; // Speed of the ball. 
	double ballVelY = 0; // Direction of movement of the ball. 
	int collision = 0;
};

#endif // BALL_PHYSICS_H
	
	
	
	
	
