//==============================================================
// Filename : PlayersScores.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "PlayersScores.h"

PlayersScores::PlayersScores(double posX, double posY, int first, int second, bool testInput)
	: ballPositionX(posX), ballPositionY(posY), scoreLeftPlayer(first), scoreRightPlayer(second), test(testInput)
{}

bool PlayersScores::updatePlayersScores() {
	// if (ballPositionX - 15 <= 0 && test == false) {
	// 	scoreLeftPlayer++;
	// 	test = true;
	// } else if (ballPositionX + 15 >= screenWidth && test == false) {
	// 	scoreRightPlayer++;
	// 	test = true;
	// } else if (ballPositionX - 15 <= 0 && test == true) {
	// 	test = true;
	// } else if (ballPositionX + 15 >= screenWidth && test == true ) {
	// 	test = true; 
	// } else {
	// 	test = false;
	// }
	if (ballPositionX - 15 <= 0) {
		scoreLeftPlayer++;
    return true;
	}
  
  if (ballPositionX + 15 >= screenWidth) {
		scoreRightPlayer++;
    return true;
	} 

  return false;
}

double PlayersScores::getBallPositionX() const {
	return ballPositionX;
}
double PlayersScores::getBallPositionY() const {
	return ballPositionY;
}
int PlayersScores::getScoreLeftPlayer() const {
	return scoreLeftPlayer; 
}
int PlayersScores::getScoreRightPlayer() const {
	return scoreRightPlayer;
}

void PlayersScores::setBallPositionX(double posX) {
	ballPositionX = posX;
}
void PlayersScores::setBallPositionY(double posY) {
	ballPositionY = posY;
}
void PlayersScores::setScoreLeftPlayer(int first) {
	scoreLeftPlayer = first;
}
void PlayersScores::setScoreRightPlayer(int second) {
	scoreRightPlayer = second;
}
