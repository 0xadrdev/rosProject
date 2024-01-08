//==============================================================
// Filename : PlayerScores.h
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

    /**
     * @brief Update the score of the players depending on the ball position. 
     * 
     * @return nothing. 
     */
    bool updatePlayersScores();

    // Getters. 
    double getBallPositionX() const;
    double getBallPositionY() const;
    int getScoreLeftPlayer() const;
    int getScoreRightPlayer() const;

    // Setters. 
    void setBallPositionX(double posX);
    void setBallPositionY(double posY);
    void setScoreLeftPlayer(int first);
    void setScoreRightPlayer(int second);

  private:
    double ballPositionX;
    double ballPositionY; 
    int scoreLeftPlayer; 
    int scoreRightPlayer; 
    bool test;
    
    static const int screenWidth = 1000;
};

#endif // PLAYERS_SCORE_H
	
	
