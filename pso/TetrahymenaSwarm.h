#ifndef TETRAHYMENASWARM_H
#define TETRAHYMENASWARM_H

#include <vector>
#include <array>

using namespace std;

// function: define the robot structure

class TetrahymenaSwarm
{
	public:
		// cell number
		int cellNo;
		// cell speed
		vector<double>* cellSpeed;
		// cell angular changing rate
		vector<double>* angularChangingRate;
		// current state of the cells (x,y,theta)
		vector<array<double,3>*>* curState;
		// target point of the cells (x,y)
		vector<array<double,2>*>* targetPoints;
		// step to head to the target (dimension of the cost function)
		int stepToTarget;

	public:
		// constructor
		TetrahymenaSwarm(void);
};

#endif