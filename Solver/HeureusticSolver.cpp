/*
 * HeuristicSolver.cpp
 *
 *  Created on: 28 juin 2019
 *      Author: boubaker
 */
#include "HeuristicSolver.h"



bool myfunction (Request i,Request j) { return (i.nbB > j.nbB); }

void HeuristicSolver::swipIJ(Solution &sol, vector<Request> & requests){
	unsigned int i,j;
	sort (requests.begin(), requests.end(), myfunction);


	i=rand()%requests.size();
	j=rand()%requests.size();
	cout<< " \t  swap teminé entre \t:"<<endl;

	cout<<i<<endl;
	cout<<j<<endl;
    swap(requests[i], requests[j]);

  //  for(unsigned int i=0;i<requests.size();i++)
   // 	cout<<this->_problem->getNameStation(requests[i].id_d)<<endl;

    sol.clear();
    doBestInsertion( sol,  requests);
}

void HeuristicSolver::reverseIJ(Solution &sol, vector<Request> & requests){
	unsigned int i,j;
	sort (requests.begin(), requests.end(), myfunction);


	i=rand()%requests.size();
	j=rand()%requests.size();
	cout<< " \t  echange teminé entre \t:"<<endl;
//	cout<<i<<endl;
//	cout<<j<<endl;

    reverse(requests.begin()+i,requests.begin()+j);
    sol.clear();
    doBestInsertion( sol,  requests);
}

void HeuristicSolver::localSearch(Solution &sol , vector<Request>  requests){

	unsigned int i=0,k=0;

	doBestInsertion(sol,requests);
	Solution s(sol);
	while( i < requests.size() * requests.size()){
		switch (k) {
		case 0 : reverseIJ(sol, requests); break;
		case 1 : swipIJ(sol,  requests) ; break;}
		if(s.Taux < sol.Taux){
//			cout<<"le taux de la solution test :\t"<<s.Taux<<endl;
	//		cout<<"le taux de la solution courante :\t"<<sol.Taux<<endl;
			cout<<"sollllllllllllll"<<endl;
			sol.print();
			cout<<"ssssssssssssssss"<<endl;
			s.print();
			sol = s;

		}else if(k<1){
			k++;
			cout<<" Method de choix de variable n°: \t"<<k<<endl;

			s=sol;
		}else{
			i++;
			cout<< "number of request"<<requests.size() * requests.size()<<endl;
			cout<<"itération n°: \t"<<i<<endl;
		}
	}
}


void HeuristicSolver::doBestInsertion(Solution & sol, vector<Request>  requests)
{

	for(unsigned int i =0; i< requests.size() ;i++)
	{
			unsigned int t =requests[i].t;
			Position pos = findBestInsertion(sol,requests[i]);

/* si on ne dépasse pas la capacité du camion régulateur*/
/* si on ne dépasse pas le temps d'un interval : 1h ou 1/2 h*/
			if ((requests[i].nbB+ sol._sumNbBikes[t]<this->_problem->getCapacityTruck())
					&& (sol._tlentgh[t]+pos._delta < this->_problem->getDmaxInterval()) ){
					doInsertion(sol,requests[i],pos);
					sol._sumNbBikes[t]+=requests[i].nbB;
			}else{
				sol._unroutedRequst.push_back(requests[i]);
		/*		cout<< " \n \n requete retiré parce que \t "<<requests[i].t
						<< " \n contrainte  de capacity de la véhicule \n"
						<<requests[i].nbB+ sol._sumNbBikes[t]<<"\t > \t"<<this->_problem->getCapacityTruck()<<"\n ou bien \n"
						<<"contrainte temporelle \n"
						<<sol._tlentgh[t]<<"\t"<<sol._tlentgh[t]+pos._delta<<"\t > \t"<<this->_problem->getDmaxInterval()<<endl; */

			}
		}
	}



double HeuristicSolver::evaluateDelta(Solution & sol, unsigned int c, unsigned int p, unsigned int t)
{

	double tic,tcj,tij;
	unsigned int nextt;
	unsigned int previous;
	unsigned int preced;


	if (p == 0)
	{
			if(t==0){
			tic = this->_problem->getDistance(0,c);
			}else
			{
				previous=t-1;
				while(previous > 0 && sol._tour[previous].size()==0) previous--;
				if(previous==0 && sol._tour[previous].size()==0){
					tic = this->_problem->getDistance(0,c);
				}else{
					preced = sol._tour[previous].size()-1;
					tic = this->_problem->getDistance(sol._tour[previous][preced],c);
				}
			}
	}else{

		tic = this->_problem->getDistance(sol._tour[t][p-1],c);
	}


	if (p==sol._tour[t].size())
	{
		if(t==sol._tour.size()-1){
		 tcj = this->_problem->getDistance(c,0);

		}
		else
		{
			nextt = t+1;
			while(nextt < sol._tour.size() && sol._tour[nextt].size()==0) nextt ++;
			if(nextt==sol._tour.size() && sol._tour[sol._tour.size()-1].size()==0 ){
				 tcj = this->_problem->getDistance(c,0 );
			}else{
				tcj = this->_problem->getDistance(c,sol._tour[nextt][0] );

			}
		}
	}
	else{ tcj = this->_problem->getDistance(c, sol._tour[t][p]);
	}

	nextt = 0;
	if (p == 0)
		{
				if(t==0)
				tij = this->_problem->getDistance(0,sol._tour[0][0]);
				else
				{
					previous=t-1;
					while(previous > 0 && sol._tour[previous].size()==0) previous--;

					if(previous==0 && sol._tour[previous].size()==0)
					{
						tij = this->_problem->getDistance(0,sol._tour[t][p]);

					}else{
						preced = sol._tour[previous].size()-1;
						tij = this->_problem->getDistance(sol._tour[previous][preced],sol._tour[t][p]);
					}
				}
		}else{
			if (p==sol._tour[t].size())
			{
				if(t==sol._tour.size()-1){
				 tij = this->_problem->getDistance(sol._tour[t][p],0);
				}else{
					nextt = t+1;
					while(nextt < sol._tour.size() && sol._tour[nextt].size()==0) nextt ++;

					if(nextt==sol._tour.size() && sol._tour[nextt-1].size()==0 )
						 tij = this->_problem->getDistance(sol._tour[t][p-1],0 );
					else
						tij = this->_problem->getDistance(sol._tour[t][p-1],sol._tour[nextt][0] );
				}

			}else{
				tij = this->_problem->getDistance(sol._tour[t][p-1],sol._tour[t][p] );
			}

		}


	return tic+tcj-tij;
}

Position HeuristicSolver::findBestInsertion(Solution & sol, Request & r )
{
		double deltai, deltaj;


		unsigned i = r.id_s;
		unsigned j = r.id_d;
		unsigned t = r.t;


		Position best(t,i,0,j,1,Config::_infDouble,0);



	    if(sol._tour[t].size() == 0){

	    	Position pos(t,i,0,j,1,0,r.nbB);
	    	return( pos);
	    }else{
	    	for (unsigned int p1=0; p1<sol._tour[t].size()+1; p1++) { // for each position p1 for i

			deltai = evaluateDelta(sol, i, p1, t);
			sol._tour[r.t].insert(sol._tour[r.t].begin()+p1,r.id_s);

			for (unsigned int p2=p1+1; p2<sol._tour[t].size()+1; p2++) { // for each position p2 of j

			deltaj = evaluateDelta(sol, j, p2, t );
			sol.doEvaluation();


			Position pos(t,i,p1,j,p2, deltai+deltaj,r.nbB);



					if ((pos._nbBikes/pos._delta) >  (best._nbBikes/best._delta) ){
						best =pos;
		//				cout<< " delta calcule à temps \t"<<r.t<<"\t à la position P \t"<<pos._p1<<"\t de sommet \t "<<pos._i
		//						<<"\t de position 2 \t"<<pos._p2<<"\t de sommet 2 \t"<<pos._j<<"\t de somme égale à \t"<<pos._delta<<endl;
					}




		}
			sol._tour[r.t].erase(sol._tour[r.t].begin()+p1);

		}
			return best;
	    }
		}











void HeuristicSolver::doInsertion(Solution & sol,Request  & r , Position & p  )
{

	sol._tour[r.t].insert(sol._tour[r.t].begin()+p._p1,r.id_s);
//	cout<<"\t\t station \t"<<r.id_s<<"\t es bien inserer \t"<<p._p1<<endl;
	sol._tour[r.t].insert(sol._tour[r.t].begin()+p._p2,r.id_d);
//	cout<<"\t\t station \t"<<r.id_d<<"\t es bien inserer \t"<<p._p2<<endl;

// count #bike

	sol.nbv += r.nbB ;
}

