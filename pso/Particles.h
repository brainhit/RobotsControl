#ifndef PARTICLES_H
#define PARTICLES_H

#include <vector>

using namespace std;

// function: define the particles for pso algorithm

class Particles
{
	protected:
		// the current position of the particle swarm
		vector<vector<double>*>* curPosition;

		// the velocity of the particle swarm
		vector<vector<double>*>* velocity;

		// the position of the best result that the specific particle have searched
		vector<vector<double>*>* localBestPosition;

		// the best value with respect to lobalBestPosition
		vector<double>* localBestValue;

		// global best position
		vector<double>* globalBestPosition;

		// global best value
		double globalBestValue;

	public:
		// constructor
		Particles();
};

#endif