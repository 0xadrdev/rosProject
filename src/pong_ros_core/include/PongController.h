//==============================================================
// Filename : PongController.h
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#ifndef PONG_CONTROLLER_H
#define PONG_CONTROLLER_H

class PongController {
  private:
    double ballPositionX;
    double ballPositionY;
    double ballVelocityX; 
    double ballVelocityY;
    double leftPaddlePosition;
    double rightPaddlePosition;
    int collisionType;
    double velocityIncrement = 1;

  public:
    // Constructor
    PongController(double posX = 500, double posY = 300, double velX = 2, double velY = 1, double padLeft = 300, double padRight = 300, int col = 0); 
    
    void checkCollision();
    
    void updateBallVelocity();

    void incrementVelocity();
    
    double getBallPositionX() const;
    double getBallPositionY() const;
    double getBallVelocityX() const;
    double getBallVelocityY() const;
    double getLeftPaddlePosition() const;
    double getRightPaddlePosition() const;

    void setBallPosition(double x, double y);

    void setBallVelocity(double x, double y); 
    
    // void setBallPositionX(double posX);
    // void setBallPositionY(double posY);
    // void setBallVelocityX(double velX);
    // void setBallVelocityY(double velY);
    void setLeftPaddlePosition(double velX);
    void setRightPaddlePosition(double velY);


};

#endif // BALL_PHYSICS_H
