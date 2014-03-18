#ifndef COSTFUNCTARGETS_H
#define COSTFUNCTARGETS_H

#include <vector>
#include "TetrahymenaSwarm.h"
using namespace std;

// function: define cost function

// define the call back function type
typedef double (*CallbackType)(vector<double>* x, TetrahymenaSwarm swarm);
// typedef double (*CallbackTypeSphereFunction)(vector<double>* x, TetrahymenaSwarm swarm);

class CostFuncTarget
{
	public:
		// define the cost function to be minimized
		static double CellFunction(vector<double>* x, TetrahymenaSwarm swarm);
		static double SphereFunction(vector<double>* x, TetrahymenaSwarm swarm);

		// calculate the norm between the final state of the cells and the target points
		static double CalculateNormValue(vector<array<double,3>*>* cellState, vector<array<double,2>*>* targets);
};

#endif