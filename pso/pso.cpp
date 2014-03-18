#include "pso.h"
#include <stdlib.h>
#include <algorithm>
#include <time.h>
#include <math.h>
#include <iostream>
#include<fstream>

using namespace std;

pso::pso()
{
	stopValue = 0;
	inertia = 0;
	correction_factor = 0;
	iteration = 0;
	numParticle = 0;
	lowerBound = new vector<double>;
	upperBound = new vector<double>;
	projectParticle = new vector<double>;
	globalBestValueCollection = new vector<double>;
	stopOrNot = false;
}
void pso::PSOAlgorithm(CallbackType costFunc, TetrahymenaSwarm tempSwarm, int np, vector<double>* lb, vector<double>* ub, double iner, double c, int iter, int stopIter, vector<double>* x0)
{
	// initialization
	Initialization(costFunc, tempSwarm, np, lb, ub, iner, c, iter, stopIter, x0);

	// run the iteration to get the best value
	int i = 0;
	for (i = 0; i < iteration; i++)
	{
		// update curPosition, velocity, localBestPosition, localBestValue with projection method (if the next position is out of boundary)
		Projection();
		
		// update globalBestPosition, globalBestValue
		UpdateGlobalBest();
	
		// update velocity
		UpdateVelocity();
		
		// stop criteria
		if (i > stopIteration)
		{
			stopOrNot = CheckStopCriteria();
		}
		if (stopOrNot == true)
		{
			break;
		}
	}
	cout<<"iteration number is "<<i<<endl;
}
void pso::Initialization(CallbackType costFunc, TetrahymenaSwarm tempSwarm, int np, vector<double>* lb, vector<double>* ub, double iner, double c, int iter, int stopIter, vector<double>* x0)
{		
	// initialize the iteration parameter
	stopValue = 100;
	inertia = iner;
	correction_factor = c;
	iteration = iter;
	numParticle = np;
	lowerBound = lb;
	upperBound = ub;
	stopIteration = stopIter;
	warmStartPosition = x0;
	stopOrNot = false;
	costFunction = costFunc;
	swarm = tempSwarm;
	//randomNumBeginPot = (int)GenerateRandomNum(2, 10);
	//randomNumDeltaPot = (int)GenerateRandomNum(2, 10);

	// read the random numbers from the txt file
	//randomNumSet = GetRandNum(randomNumSet);

	// reset the unknown warmStartPosition dimension
	if (warmStartPosition->empty()!=1)
	{
		for (int i = 0; i < lowerBound->size(); i++)
		{
			if (warmStartPosition->at(i) == 100000)
				//warmStartPosition->at(i) = GenerateRandomNumFromFile("../../MPITOPI.txt",2,3);
				warmStartPosition->at(i) = GenerateRandomNum(lowerBound->at(i), upperBound->at(i));
		}
	}

	// initialize the particle info
	for (int i = 0; i < np; i++)
	{
		vector<double>* tempCurPosition = new vector<double>;
		vector<double>* tempVelocity = new vector<double>;
		vector<double>* tempLocalBestPosition = new vector<double>;
		double xIni; // random generated x value
		double vIni; // random generated v value
		// for each particle, randomly allocate the initial value
		for (int j = 0; j < (int)lb->size(); j++)
		{
			//xIni = GenerateRandomNumFromFile("../MPITOPI.txt",randomNumBeginPot,randomNumDeltaPot);
			xIni = GenerateRandomNum(lowerBound->at(j), upperBound->at(j));
			//vIni = GenerateRandomNumFromFile("../M2PITO2PI.txt",randomNumBeginPot,randomNumDeltaPot);
			vIni = GenerateRandomNum(-abs(upperBound->at(j)-lowerBound->at(j)), abs(upperBound->at(j)-lowerBound->at(j)));
			tempCurPosition->push_back(xIni);
			tempVelocity->push_back(vIni);
			//tempVelocity->push_back(GenerateRandomNum(-abs(upperBound->at(j)-lowerBound->at(j)), abs(upperBound->at(j)-lowerBound->at(j))));
			if (warmStartPosition->empty()==1)
				tempLocalBestPosition->push_back(xIni);
			else
				tempLocalBestPosition->push_back(warmStartPosition->at(j));
		}

		// form the entire swarm
		curPosition->push_back(tempCurPosition);
		velocity->push_back(tempVelocity);
		localBestPosition->push_back(tempLocalBestPosition);
		//localBestValue->push_back(10000000);
		localBestValue->push_back(costFunction(tempLocalBestPosition, swarm));
	}

	// get the global best position and value
	if (warmStartPosition->empty()==1)
	{
		globalBestPosition = MinPosition(localBestPosition, localBestValue);
		globalBestValue = MinValue(localBestValue);
	}
	else
	{
		globalBestPosition = warmStartPosition;
		globalBestValue = localBestValue->at(1);
	}

	// update the initial velocity if using the warm start algorithm
	if (warmStartPosition->empty()!=1)
		UpdateVelocity();
}
double pso::GenerateRandomNum(double lowerBound, double upperBound)
{
	// int randNum = rand() % (int)((upperBound-lowerBound)*1000) + (int)(lowerBound*1000);
	// double randNum = ((double(rand()) / double(RAND_MAX)) * (upperBound - lowerBound)) + lowerBound;
	double j = drand();
	double randNum = j * (upperBound - lowerBound) + lowerBound;
	return randNum;
}
double pso::MinValue(vector<double>* value)
{
	vector<double>::const_iterator it;
	it = min_element(value->begin(),value->end());

	return *it;
}
double pso::MaxValue(vector<double>* value)
{
	vector<double>::const_iterator it;
	it = max_element(value->begin(),value->end());

	return *it;
}
vector<double>* pso::MinPosition(vector<vector<double>*>* position, vector<double>* value)
{
	vector<double>* tempPosition = new vector<double>;
	vector<double>::const_iterator it;
	it = min_element(value->begin(),value->end());
	tempPosition = position->at(it-value->begin());

	return tempPosition;
}
void pso::Projection()
{
	for (int i = 0; i < numParticle; i++)
	{
		// project the particle which is outside the boundary to inside the boundary
		projectParticle = SumTwoVector(curPosition->at(i), velocity->at(i));
		projectParticle = MinTwoVector(projectParticle,upperBound);
		projectParticle = MaxTwoVector(projectParticle,lowerBound);

		// reset the velocity of the particles
		velocity->at(i) = DiffTwoVector(projectParticle,curPosition->at(i));

		// get the new particle position
		curPosition->at(i) = projectParticle;

		// get the current best value
		double cost = costFunction(curPosition->at(i), swarm);
		cost = costFunction(curPosition->at(i), swarm);
		cost = costFunction(curPosition->at(i), swarm);
		cost = costFunction(curPosition->at(i), swarm);

		// update the local best value
		if (cost < localBestValue->at(i))
		{
			localBestPosition->at(i) = curPosition->at(i);
			localBestValue->at(i) = cost;
		}
	}
}
void pso::UpdateGlobalBest()
{
	double cost = MinValue(localBestValue);
	if (cost < globalBestValue)
	{
		globalBestValue = cost;
		globalBestPosition = MinPosition(localBestPosition, localBestValue);
	}
	globalBestValueCollection->push_back(globalBestValue);
}
void pso::UpdateVelocity()
{
	for (int i = 0; i < numParticle; i ++)
		for (int j = 0; j < (int)lowerBound->size(); j ++)
		{
			//double r1 = abs(GenerateRandomNumFromFile("../MONETOONE.txt",randomNumBeginPot,randomNumDeltaPot));
			//double r2 = abs(GenerateRandomNumFromFile("../MONETOONE.txt",randomNumBeginPot,randomNumDeltaPot));
			//velocity->at(i)->at(j) = inertia*velocity->at(i)->at(j)+correction_factor*r1*(localBestPosition->at(i)->at(j)-curPosition->at(i)->at(j))+correction_factor*r2*(globalBestPosition->at(j)-curPosition->at(i)->at(j));
			velocity->at(i)->at(j) = inertia*velocity->at(i)->at(j)+correction_factor*GenerateRandomNum(0,1)*(localBestPosition->at(i)->at(j)-curPosition->at(i)->at(j))+correction_factor*GenerateRandomNum(0,1)*(globalBestPosition->at(j)-curPosition->at(i)->at(j));
		}
}
bool pso::CheckStopCriteria()
{	
	// get the delta of the global best value
	vector<double>* deltaGbestValue = new vector<double>;
	vector<double>* deltaGbestValueLog = new vector<double>;
	deltaGbestValue = AbsVector(DiffTwoVector(SubVector(globalBestValueCollection, 2, (int)globalBestValueCollection->size()), SubVector(globalBestValueCollection, 1, (int)globalBestValueCollection->size()-1)));
	deltaGbestValueLog = LogVector(SubtractZeroFromVector(deltaGbestValue));
	
	double sumDelta = 0;
	if ((int)deltaGbestValueLog->size() > 10)
	{
		//  get the stop value based on the current steps of global best value
		stopValue = SumVector(LogVector(SubVector(globalBestValueCollection, (int)globalBestValueCollection->size()-10, (int)globalBestValueCollection->size())));

		// get the delta sum value
		sumDelta = SumVector(SubVector(deltaGbestValueLog, (int)deltaGbestValueLog->size()-10, (int)deltaGbestValueLog->size()));
	}
	else
	{
		stopValue = SumVector(LogVector(globalBestValueCollection));
		sumDelta = SumVector(deltaGbestValueLog);
	}

	// delete pointer
	delete deltaGbestValue;
	delete deltaGbestValueLog;

	// identify whether the iteration need to be stoped or not
	if (sumDelta < stopValue/2)
	{
		return true;
	}
	else
	{
		return false;
	}
}
vector<double>* pso::DiffTwoVector(vector<double>* vector1, vector<double>* vector2)
{
	vector<double>* tempVector = new vector<double>;
	
	for (int i = 0; i < (int)vector1->size(); i ++)
	{
		tempVector->push_back(vector1->at(i)-vector2->at(i));
	}

	return tempVector;
}
vector<double>* pso::SumTwoVector(vector<double>* vector1, vector<double>* vector2)
{
	vector<double>* tempVector = new vector<double>;
	
	for (int i = 0; i < (int)vector1->size(); i ++)
	{
		tempVector->push_back(vector1->at(i)+vector2->at(i));
	}

	return tempVector;
}
vector<double>* pso::MinTwoVector(vector<double>* vector1, vector<double>* vector2)
{
	vector<double>* tempVector = new vector<double>;
	
	for (int i = 0; i < (int)vector1->size(); i ++)
	{
		tempVector->push_back(min(vector1->at(i),vector2->at(i)));
	}

	return tempVector;
}
vector<double>* pso::MaxTwoVector(vector<double>* vector1, vector<double>* vector2)
{
	vector<double>* tempVector = new vector<double>;
	
	for (int i = 0; i < (int)vector1->size(); i ++)
	{
		tempVector->push_back(max(vector1->at(i),vector2->at(i)));
	}

	return tempVector;
}
vector<double>* pso::LogVector(vector<double>* vector1)
{
	vector<double>* tempVector = new vector<double>;
	for (int i = 0; i < (int)vector1->size(); i++)
	{
		tempVector->push_back(log(vector1->at(i)));
	}
	return tempVector;
}
double pso::SumVector(vector<double>* vector1)
{
	double sum = 0;
	for (int i = 0; i < (int)vector1->size(); i ++)
	{
		sum = sum+vector1->at(i);
	}

	return sum;
}
vector<double>* pso::AbsVector(vector<double>* vector1)
{
	vector<double>* tempVector = new vector<double>;
	for (int i = 0; i < (int)vector1->size(); i++)
	{
		tempVector->push_back(abs(vector1->at(i)));
	}
	return tempVector;
}
vector<double>* pso::SubVector(vector<double>* vector1, int beginPtr, int endPtr)
{
	vector<double> temp1;
	temp1 = *vector1;
	vector<double> temp2(temp1.begin()+beginPtr-1,temp1.begin()+endPtr);
	vector<double>* temp3 = new vector<double>;
	for(int i = 0; i < temp2.size(); i++)
		temp3->push_back(temp2[i]);

	return temp3;
}
vector<double>* pso::SubtractZeroFromVector(vector<double>* a)                                                     
{
    vector<double>::iterator       it = a->begin();                                                 
    vector<double>::iterator       end = a->end();                                                      

    while (it != end)                                                                                            
    {
            if (*it == 0)                                                                                     
            {
                it = a->erase(it);                                                                                
                end = a->end();                                                                                     
            }
            else
        ++it;                                                                                        
    }

	return a;
}
vector<double>* pso::ModifyZeroInVector(vector<double>* a)
{
	vector<double>* temp = new vector<double>;
	return temp;
}
double pso::GetCostValue()
{
	return globalBestValue;
}
vector<double>* pso::GetBestFuncSolution()
{
	return globalBestPosition;
}