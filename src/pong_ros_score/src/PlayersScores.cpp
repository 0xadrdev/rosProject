//==============================================================
// Filename : PlayersScores.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description : PlayersScores class to manage the scores. 
//==============================================================

#include "PlayersScores.h"
#include "../../pong_ros_core/include/Constants.h"

using namespace pong_ros_constants; 

PlayersScores::PlayersScores(double posX, double posY, int first, int second)
	: ballPositionX(posX), ballPositionY(posY), scoreLeftPlayer(first), scoreRightPlayer(second)
{}

bool PlayersScores::updatePlayersScores() {
	if (ballPositionX - 15 <= 0) {
		scoreLeftPlayer++;
    return true;
	}
  
  if (ballPositionX + 15 >= SCREEN_WIDTH) {
		scoreRightPlayer++;
    return true;
	} 

  return false;
}

int PlayersScores::getScoreLeftPlayer() const {
	return scoreLeftPlayer; 
}
int PlayersScores::getScoreRightPlayer() const {
	return scoreRightPlayer;
}

void PlayersScores::setBallPosition(double x, double y) {
  ballPositionX = x; 
  ballPositionY = y;
}

