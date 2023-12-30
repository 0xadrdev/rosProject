//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#ifndef LOGIC_H
#define LOGIC_H

class logic {
	
public:
	// Constructor
	logic(double posX = 500, double posY = 300, double velX = 2, double velY = 1, double padLeft = 300, double padRight = 300, int col = 0); 
	
	void checkCollision();
	
	void updateVelocity();
	
	double getBallPosX() const;
	double getBallPosY() const;
	double getBallVelX() const;
	double getBallVelY() const;
	double getPadPosLeft() const;
	double getPadPosRight() const;
	
	void setBallPosX(double posX);
	void setBallPosY(double posY);
	void setBallVelX(double velX);
	void setBallVelY(double velY);
	void setPadPosLeft(double velX);
	void setPadPosRight(double velY);

private:
	// Variables storing ball position
	double ballPosX;// = 500; // Horizontal position of the center of the ball.
	double ballPosY;// = 300; // Vertical position of the center of the ball.
	double ballVelX; // = 0; // Speed of the ball. 
	double ballVelY;// = 0; // Direction of movement of the ball. 
	double padPosLeft;// = 300; // Speed of the ball. 
	double padPosRight;// = 300; // Direction of movement of the ball. 
	int collision;// = 0; // Variable storing the type of collision. 

	static const int SCREEN_WIDTH = 1000;
    static const int SCREEN_HEIGHT = 600;
    static const int fontSize = 160;

    static const int WALL_HEIGHT = 15;
    static const int PADDLE_WIDTH = 5;
    static const int PADDLE_HEIGHT = 100; // Make sure this is even
    static const int BALL_SIZE = 30 / 2; // Make sure this is even
	
	// Size settings. All in pixels.
	// static const int screenWidth = 1000;
	// static const int screenHeight = 600;
	// static const int fontSize = 160;

	// static const int wallHeight = 30;
	// static const int batWidth = 30;
	// static const int batHeight = 100; // Make sure this is even
	// static const int ballSize = 30/2; // Make sure this is even
};

#endif // BALL_PHYSICS_H
