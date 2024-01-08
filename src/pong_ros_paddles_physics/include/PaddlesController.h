//==============================================================
// Filename : PaddlesController.h
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#ifndef PADDLES_CONTROLLER_H
#define PADDLES_CONTROLLER_H

class PaddlesController {
  public:
    // Constructor
    PaddlesController(double l = 300, double r = 300, double vl = 5, double vr = 50);

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

#endif // PADDLES_CONTROLLER_H
	
	
