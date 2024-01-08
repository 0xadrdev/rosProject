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
    BallPhysics ballPhysics;
    double leftPaddlePosition;
    double rightPaddlePosition;
    int collisionType;
    double velocityIncrement;

  public:
    PongController(BallPhysics ball = BallPhysics(), double paddleLeft = 300, double paddleRight = 300, int col = 0, double vel = 1); 

    /**
     * @brief Sets the collisionType depending on the ball position. 
     *
     * @return nothing  
     */
    void determineCollisionType();

    /**
     * @brief Increments the velocity if the ball bounces a paddle. 
     * 
     * @return nothing
     */
    void updateBallPosition();

    /**
     * @brief Updates the ball position from the collisionType atribute. 
     * 
     * @return nothing 
     */
    void incrementVelocity();
    
    // Getters. 
    int getCollisionType() const;
    double getBallPositionX() const;
    double getBallPositionY() const;

    // Setters. 
    void setLeftPaddlePosition(double y);
    void setRightPaddlePosition(double y);
};

#endif // BALL_PHYSICS_H
