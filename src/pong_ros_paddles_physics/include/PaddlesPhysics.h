//==============================================================
// Filename : PaddlesPhysics.h
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description : Header of the class PaddlesPhysics. 
//==============================================================

#ifndef PADDLES_PHYSICS_H
#define PADDLES_PHYSICS_H

class PaddlesPhysics {
  public:
    // Constructor
    PaddlesPhysics(double l = 300, double r = 300, double vl = 5, double vr = 50);

    void updateLeftPaddleUp();
    void updateLeftPaddleDown();
    void updateRightPaddleUp();
    void updateRightPaddleDown();

    // Getters 
    double getLeftPaddlePosition();
    double getRightPaddlePosition();

    // Setters 
    void setRightPaddleVelocity(double v);

  private:
    double leftPaddlePosition;
    double rightPaddlePosition;
    double leftPaddleVelocity;
    double rightPaddleVelocity;
};

#endif // PADDLES_PHYSICS_H
	
	
