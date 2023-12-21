/* score.h */

#ifndef SCORE_H
#define Score_H

class score {
	
public:
	// Constructor
	score(double posX = 500, double posY = 300, int first = 0, int second = 0, bool test = false); 
	
	void updateScore();
	
	double getBallPosX() const;
	double getBallPosY() const;
	int getScoreFirst() const;
	int getScoreSecond() const;
	
	void setBallPosX(double posX);
	void setBallPosY(double posY);
	void setScoreFirst(int first);
	void setScoreSecond(int second);

private:
	// Variables storing ball position
	double ballPosX; // Horizontal position of the center of the ball.
	double ballPosY; // Vertical position of the center of the ball.
	int scoreFirst; // Speed of the ball. 
	int scoreSecond; // Direction of movement of the ball. 
	bool test;
	
	// Size settings. All in pixels.
        static const int screenWidth = 1000;
};

#endif // SCORE_H
	
	
