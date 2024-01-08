//==============================================================
// Filename : PlayerScores.h
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description : Header of the PlayersScores class. 
//==============================================================

#ifndef PLAYERS_SCORES_H
#define PLAYERS_SCORES_H

class PlayersScores {
  public:
    // Constructor
    PlayersScores(double posX = 500, double posY = 300, int first = 0, int second = 0); 

    /**
     * @brief Update the score of the players depending on the ball position setted. 
     * 
     * @return nothing. 
     */
    bool updatePlayersScores();

    // Getters. 
    int getScoreLeftPlayer() const;
    int getScoreRightPlayer() const;

    // Setters. 
    void setBallPosition(double x, double y);

  private:
    double ballPositionX;
    double ballPositionY; 
    int scoreLeftPlayer; 
    int scoreRightPlayer; 
};

#endif // PLAYERS_SCORE_H
	
	
