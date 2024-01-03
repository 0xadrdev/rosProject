//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "score.h"

score::score(double posX, double posY, int first, int second, bool testInput)
	: ballPositionX(posX), ballPositionY(posY), scoreLeftPlayer(first), scoreRightPlayer(second), test(testInput)
{}

// Updating the position
void score::updatePlayersScores() {

	// Computing whether a score was made
	if (ballPositionX - 15 <= 0 && test == false) {
		scoreLeftPlayer++;
		test = true;
	} else if (ballPositionX + 15 >= screenWidth && test == false) {
		scoreRightPlayer++;
		test = true;
	} else if (ballPositionX <= 0 && test == true) {
		test = true;
	} else if (ballPositionX >= screenWidth && test == true ) {
		test = true; 
	} else {
		test = false;
	}
	
}

// Retrieving the class private data
double score::getBallPositionX() const {
	return ballPositionX;
}
double score::getBallPositionY() const {
	return ballPositionY;
}
int score::getScoreLeftPlayer() const {
	return scoreLeftPlayer; 
}
int score::getScoreRightPlayer() const {
	return scoreRightPlayer;
}

// Setting the class private data
void score::setBallPositionX(double posX) {
	ballPositionX = posX;
}
void score::setBallPositionY(double posY) {
	ballPositionY = posY;
}
void score::setScoreLeftPlayer(int first) {
	scoreLeftPlayer = first;
}
void score::setScoreRightPlayer(int second) {
	scoreRightPlayer = second;
}
