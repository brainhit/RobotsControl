#include "mex.h"

#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include "CostFuncTarget.h"
#include "pso.h"
#include "TetrahymenaSwarm.h"
#include "Particles.h"
#include <array>

#define PI 3.14159265358

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //// pass the matlab data into the c function
    mxArray *fun_in, *np_in, *lb_in, *ub_in, *inertia_in, *correction_factor_in, *iteration_in, *stopIter_in, *x0_in;
    double *fun_val, *np_val, *lb_val, *ub_val, *inertia_val, *correction_factor_val, *iteration_val, *stopIter_val, *x0_val;
    const int *dims = mxGetDimensions(prhs[2]);
    const int *dimsWarmStart = mxGetDimensions(prhs[8]);
    fun_in = mxDuplicateArray(prhs[0]);
    np_in = mxDuplicateArray(prhs[1]);
    lb_in = mxDuplicateArray(prhs[2]);
    ub_in = mxDuplicateArray(prhs[3]);
    inertia_in = mxDuplicateArray(prhs[4]);
    correction_factor_in = mxDuplicateArray(prhs[5]);
    iteration_in = mxDuplicateArray(prhs[6]);
    stopIter_in = mxDuplicateArray(prhs[7]);
    x0_in = mxDuplicateArray(prhs[8]);
    fun_val = mxGetPr(fun_in);
	np_val = mxGetPr(np_in);
	lb_val = mxGetPr(lb_in);
	ub_val = mxGetPr(ub_in);
	inertia_val = mxGetPr(inertia_in);
	correction_factor_val = mxGetPr(correction_factor_in);
	iteration_val = mxGetPr(iteration_in);
	stopIter_val = mxGetPr(stopIter_in);
	x0_val = mxGetPr(x0_in);
    
	//// robot info
    int cellNum = (int)fun_val[0];
    double cellInfo[500];
    for (int i = 0; i < cellNum*2; i++)
        cellInfo[i] = fun_val[1+i];
    double states[500];
    for (int i = 0; i < cellNum*5; i++)
        states[i] = fun_val[1+cellNum*2+i];
	int stepToTarget = dims[0];
	TetrahymenaSwarm swarm;
	swarm.cellNo = cellNum;
	for (int i = 0; i<swarm.cellNo; i++)
	{
		swarm.cellSpeed->push_back(cellInfo[i]);
		swarm.angularChangingRate->push_back(cellInfo[swarm.cellNo+i]);
		array<double,3>* cell = new array<double,3>;
		for (int j = 0; j < 3; j++)
			cell->at(j) = states[i*3+j];
		swarm.curState->push_back(cell);
		array<double,2>* target = new array<double,2>;
		for (int j = 0; j < 2; j++)
			target->at(j) = states[i*2+j+swarm.cellNo*3];
		swarm.targetPoints->push_back(target);
	}
	swarm.stepToTarget = stepToTarget;

	pso psoObj;
	//// define the pso function input value
	int np = (int)np_val[0];
	vector<double>* lb = new vector<double>;
	vector<double>* ub = new vector<double>;
	for (int i = 0; i < swarm.stepToTarget; i++)
	{
		lb->push_back(lb_val[i]);
		ub->push_back(ub_val[i]);
	}
	double iner = inertia_val[0];
	double c = correction_factor_val[0];
	int iter = (int)iteration_val[0];
	int stopIter = (int)stopIter_val[0];
	vector<double>* warmStartPosition = new vector<double>;
	for (int i = 0; i < (int)dimsWarmStart[0]; i++)
		warmStartPosition->push_back(x0_val[i]);

	//// run the pso function
	srand(time(NULL));
	const clock_t begin_time = clock();
	psoObj.PSOAlgorithm(CostFuncTarget::CellFunction, swarm, np, lb, ub, iner, c, iter, stopIter, warmStartPosition);
	std::cout << "time cost is "<<double( clock () - begin_time ) /  CLOCKS_PER_SEC<<endl;
	// get the best cost and best solution
	double cost = psoObj.GetCostValue();
	// get the best solution to the cost function
	vector<double>* bestSolution = new vector<double>;
	bestSolution = psoObj.GetBestFuncSolution();
    
    //// pass the data out to matlab
    mxArray *xMin_out, *yMin_out;
    double *xMin_val, *yMin_val;
    xMin_out = plhs[0] = mxCreateDoubleMatrix((int)dims[0],1,mxREAL);
    yMin_out = plhs[1] = mxCreateDoubleMatrix(1,1,mxREAL);
    xMin_val = mxGetPr(xMin_out);
    yMin_val = mxGetPr(yMin_out);
    yMin_val[0] = cost;
	for (int i = 0; i < bestSolution->size(); i++)
		xMin_val[i] = bestSolution->at(i);

	//// delete pointer
	delete lb;
	delete ub;
	delete warmStartPosition;
  
	return;
}