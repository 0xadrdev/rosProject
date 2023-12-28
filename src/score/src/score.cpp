//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include "score.h"

score::score(double posX, double posY, int first, int second, bool testInput)
	: ballPosX(posX), ballPosY(posY), scoreFirst(first), scoreSecond(second), test(testInput)
{}

// Updating the position
void score::updateScore() {

	// Computing whether a score was made
	if (ballPosX <= 0 && test == false) {
		scoreFirst++;
		test = true;
	} else if (ballPosX >= screenWidth && test == false) {
		scoreSecond++;
		test = true;
	} else if (ballPosX <= 0 && test == true) {
		test = true;
	} else if (ballPosX >= screenWidth && test == true ) {
		test = true; 
	} else {
		test = false;
	}
	
}

// Retrieving the class private data
double score::getBallPosX() const {
	return ballPosX;
}
double score::getBallPosY() const {
	return ballPosY;
}
int score::getScoreFirst() const {
	return scoreFirst; 
}
int score::getScoreSecond() const {
	return scoreSecond;
}

// Setting the class private data
void score::setBallPosX(double posX) {
	ballPosX = posX;
}
void score::setBallPosY(double posY) {
	ballPosY = posY;
}
void score::setScoreFirst(int first) {
	scoreFirst = first;
}
void score::setScoreSecond(int second) {
	scoreSecond = second;
}
