//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#ifndef PLAYERS_SCORES_H
#define PLAYERS_SCORES_H

class PlayersScores {
	
public:
	// Constructor
	PlayersScores(double posX = 500, double posY = 300, int first = 0, int second = 0, bool test = false); 
	
	void updatePlayersScores();
	
	double getBallPositionX() const;
	double getBallPositionY() const;
	int getScoreLeftPlayer() const;
	int getScoreRightPlayer() const;
	
	void setBallPositionX(double posX);
	void setBallPositionY(double posY);
	void setScoreLeftPlayer(int first);
	void setScoreRightPlayer(int second);

private:
	// Variables storing ball position
	double ballPositionX; // Horizontal position of the center of the ball.
	double ballPositionY; // Vertical position of the center of the ball.
	int scoreLeftPlayer; // Speed of the ball. 
	int scoreRightPlayer; // Direction of movement of the ball. 
	bool test;
	
	// Size settings. All in pixels.
  static const int screenWidth = 1000;
};

#endif // PLAYERS_SCORE_H
	
	
