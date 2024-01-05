//==============================================================
// Filename : PongController.h
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "BallPhysics.h"

#ifndef PONG_CONTROLLER_H
#define PONG_CONTROLLER_H

class PongController {
  private:
    // double ballPositionX;
    // double ballPositionY;
    // double ballVelocityX; 
    // double ballVelocityY;
    BallPhysics ballPhysics;
    double leftPaddlePosition;
    double rightPaddlePosition;
    int collisionType;
    double velocityIncrement;

  public:
    // Constructor
    PongController(BallPhysics ball = BallPhysics(), double padLeft = 300, double padRight = 300, int col = 0, double vel = 1); 
    // PongController(double posX = 500, double posY = 300, double velX = 2, double velY = 1, double padLeft = 300, double padRight = 300, int col = 0, double vel = 1); 
    
    void determineCollisionType();
    
    void updateBallVelocity();

    void incrementVelocity();
    
    // Getters. 
    double getBallPositionX() const;
    double getBallPositionY() const;
    // double getBallVelocityX() const;
    // double getBallVelocityY() const;
    double getLeftPaddlePosition() const;
    double getRightPaddlePosition() const;
    int getCollisionType() const;

    // Setters. 
    // void setBallPosition(double x, double y);
    // void setBallVelocity(double x, double y); 
    void setLeftPaddlePosition(double velX);
    void setRightPaddlePosition(double velY);
};

#endif // BALL_PHYSICS_H
