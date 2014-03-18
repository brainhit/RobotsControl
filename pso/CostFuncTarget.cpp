#include "CostFuncTarget.h"
#include <vector>
#include <math.h>

using namespace std;

double CostFuncTarget::CellFunction(vector<double>* x, TetrahymenaSwarm swarm)
{
	// set cells initial state
	vector<array<double,3>*>* state = new vector<array<double,3>*>;

	for (int i = 0; i < swarm.cellNo; i++)
	{
		array<double,3>* cell = new array<double,3>;
		cell->at(0) = swarm.curState->at(i)->at(0);
		cell->at(1) = swarm.curState->at(i)->at(1);
		cell->at(2) = swarm.curState->at(i)->at(2);
		state->push_back(cell);
	}
	for (int i = 0; i < swarm.stepToTarget; i++)
	{
		for (int j = 0; j < (int)swarm.curState->size(); j++)
		{
			state->at(j)->at(2) = state->at(j)->at(2) + swarm.angularChangingRate->at(j)*sin(x->at(i)-state->at(j)->at(2));
			state->at(j)->at(0) = state->at(j)->at(0)+swarm.cellSpeed->at(j)*cos(state->at(j)->at(2));
			state->at(j)->at(1) = state->at(j)->at(1)+swarm.cellSpeed->at(j)*sin(state->at(j)->at(2));
		}
	}
	
	// get the cost value
	double cost = CostFuncTarget::CalculateNormValue(state, swarm.targetPoints);

	// delete pointer
	delete state;

	return sqrt(cost);
}
double CostFuncTarget::SphereFunction(vector<double>* x, TetrahymenaSwarm swarm)
{
	double result = 0;
	for (int i = 0; i < (int)x->size(); i++)
	{
		result = result + pow(x->at(i) - (double)(i+1),2);
	}
	return result;
}
double CostFuncTarget::CalculateNormValue(vector<array<double,3>*>* cellState, vector<array<double,2>*>* targets)
{
	double norm = 0;

	for (int i = 0; i < targets->size(); i++)
	{
		norm=norm+pow(cellState->at(i)->at(0)-targets->at(i)->at(0),2)+pow(cellState->at(i)->at(1)-targets->at(i)->at(1),2);
	}

	return norm;
}