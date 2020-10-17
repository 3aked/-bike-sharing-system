/*
 * CplexSolver.h
 *
 *  Created on: 1 mai 2019
 *      Author: boubaker
 */

#ifndef CPLEXSOLVER_H_
#define CPLEXSOLVER_H_

#include"../Model/CplexSolution.h"

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN


class CplexSolver {

private :

	Problem * _problem;
	CplexSolution *_solution;


public:
	CplexSolver(Problem * problem,CplexSolution *_solution);
	void resolve();

};

inline CplexSolver::CplexSolver(Problem * problem,CplexSolution *solution):_problem(problem),_solution(solution){}

#endif /* CPLEXSOLVER_H_ */
