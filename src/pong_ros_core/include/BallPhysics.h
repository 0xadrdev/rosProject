//==============================================================
// Filename : BallPhysics.h 
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#ifndef BALL_PHYSICS_H
#define BALL_PHYSICS_H

class BallPhysics {
  private:
    double ballPositionX = 500; 
    double ballPositionY = 300; 
    double ballVelocityX = 0; 
    double ballVelocityY = 0; 
	
  public:
    // Constructor. 
    BallPhysics(double posX = 500, double posY = 300, double velX = 0, double velY = 0); 
    
    // Getters. 
    double getBallPositionX() const;
    double getBallPositionY() const;
    double getBallVelocityX() const;
    double getBallVelocityY() const;

    // Setters. 
    void setBallPosition(double x, double y);
};

#endif // BALL_PHYSICS_H
	
	
	
	
	
