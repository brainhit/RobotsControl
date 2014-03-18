#include <vector>
#include <array>
#include "TetrahymenaSwarm.h"

using namespace std;

TetrahymenaSwarm::TetrahymenaSwarm(void)
{
	cellNo = 0;
	cellSpeed = new vector<double>;
	angularChangingRate = new vector<double>;
	curState = new vector<array<double,3>*>;
	targetPoints = new vector<array<double,2>*>;
	stepToTarget = 0;
}