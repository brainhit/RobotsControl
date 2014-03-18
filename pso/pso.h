#ifndef PSO_H
#define PSO_H

#include <vector>
#include <array>
#include <algorithm>
#include <stdlib.h>
#include "Particles.h"
#include "TetrahymenaSwarm.h"
#include "CostFuncTarget.h"
#include "mtrand.h"

using namespace std;

// function: particle swarm optimization (PSO) algorithm

class pso: public Particles
{
	private:
		// the stop criteria of the iteration
		double stopValue;
		// inertia value
		double inertia;
		// correlation factor
		double correction_factor;
		// iteration number
		int iteration;
		// number of swarm particles
		int numParticle;
		// lower bound of the variable
		vector<double>* lowerBound;
		// upper bound of the variable
		vector<double>* upperBound;
		// stop criteria iteration
		int stopIteration;
		// warm start position (100000 means that this dimension is undefined)
		vector<double>* warmStartPosition;
		// global best value collection
		vector<double>* globalBestValueCollection;
		// projected particle
		vector<double>* projectParticle;
		// stop the iteration or not
		bool stopOrNot;
		// call back function
		CallbackType costFunction;
		// call back function
		TetrahymenaSwarm swarm;
		// random number generation object by Mersenne twister
		MTRand drand;

	public:
		// constructor
		pso();
		// pso function
		void PSOAlgorithm(CallbackType costFunc, TetrahymenaSwarm tempSwarm, int np, vector<double>* lb, vector<double>* ub, double iner, double c, int iter, int stopIter, vector<double>* x0);
		// initialization
		void Initialization(CallbackType costFunc, TetrahymenaSwarm tempSwarm, int np, vector<double>* lb, vector<double>* ub, double iner, double c, int iter, int stopIter, vector<double>* x0);
		// update curPosition, velocity, localBestPosition, localBestValue with projection method (if the next position is out of boundary)
		void Projection();
		// update globalBestPosition, globalBestValue
		void UpdateGlobalBest();
		// update velocity
		void UpdateVelocity();
		// random number generator
		double GenerateRandomNum(double lowerBound, double upperBound);
		// stop criteria
		bool CheckStopCriteria();
		// find the min value through the vector
		double MinValue(vector<double>* value);
		// find the max value through the vector
		double MaxValue(vector<double>* value);
		// find the position with min value
		vector<double>* MinPosition(vector<vector<double>*>* position, vector<double>* value);
		// vector1 - vector2
		vector<double>* DiffTwoVector(vector<double>* vector1, vector<double>* vector2);
		// vector1 + vector2
		vector<double>* SumTwoVector(vector<double>* vector1, vector<double>* vector2);
		// min(vector1, vector2)
		vector<double>* MinTwoVector(vector<double>* vector1, vector<double>* vector2);
		// max(vector1, vector2)
		vector<double>* MaxTwoVector(vector<double>* vector1, vector<double>* vector2);
		// log of vector
		vector<double>* LogVector(vector<double>* vector1);
		// sum of vector
		double SumVector(vector<double>* vector1);
		// abs of vector
		vector<double>* AbsVector(vector<double>* vector1);
		// subvector of a vector
		vector<double>* SubVector(vector<double>* vector1, int beginPtr, int endPtr);
		// subtract vector
		vector<double>* SubtractZeroFromVector(vector<double>* a);
		// change the 0 in the vector to 1
		vector<double>* ModifyZeroInVector(vector<double>* a);
		// get the best cost
		double GetCostValue();
		// get the best solution to the cost function
		vector<double>* GetBestFuncSolution();
};

#endif