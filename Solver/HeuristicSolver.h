#ifndef HEURISTCISOLVER_H_
#define HEURISTCISOLVER_H_


#include "../Model/Problem.h"
#include "../Model/Solution.h"
#include "../Model/CplexSolution.h"
#include "../Model/Request.h"
#include "../Config.h"





struct Position
{
	unsigned int _t ;
	unsigned int _i;
	unsigned int _p1;
	unsigned int _j;
	unsigned int _p2;
	double _delta;
	int _nbBikes;


	Position(const unsigned int & t, const unsigned int & i, const unsigned int & p1,
				 const unsigned int & j, const unsigned int & p2, const double & delta, const int & nbBikes)
				 :_t(t),_i(i),_p1(p1),_j(j),_p2(p2),_delta(delta), _nbBikes(nbBikes){}
};

class HeuristicSolver {
private:
	Problem * _problem;
public:

	double evaluateDelta(Solution & sol, unsigned int i, unsigned int p, unsigned int t);

	Position findBestInsertion(Solution & sol, Request  & r );

	void doInsertion(Solution & sol,Request  & r , Position & p  );

	HeuristicSolver(Problem * problem);
	void doBestInsertion(Solution &sol, vector<Request>   requests);
	void localSearch(Solution &sol, vector<Request>  requests);
	void swipIJ(Solution &sol, vector<Request> & requests);
	void reverseIJ(Solution &sol, vector<Request> & requests);


	void test();
};

inline HeuristicSolver::HeuristicSolver(Problem * problem)
: _problem(problem) { }



#endif /*TOPIDCHSOLVER_H_*/
