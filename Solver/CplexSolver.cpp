/*
 * CplexSolver.cpp
 *
 *  Created on: 1 mai 2018
 *      Author: lisic
 */
#include <string.h>
#include "CplexSolver.h"
#include"../Utility.h"


void CplexSolver::resolve()
{
	IloEnv env ;
	try{

		IloModel model (env , "BSS") ;
		typedef IloArray<IloNumArray> NumMatrix;
		typedef IloArray<NumMatrix>   Num3Matrix;

		typedef IloArray<IloNumVarArray> NumVarMatrix;
		typedef IloArray<NumVarMatrix>   Num3VarMatrix;

		/* déclarations */
		IloInt Tmin, Tmax ; // la fenêtre de temps de travail de l'agent régulateur [6h,13h] ou bien  [13h,20] pour Calais
		Tmin=_problem->getTmin(); // correspond à 6h
		Tmax=_problem->getTmax(); // correspond à 13h (créneau d'une demi heure)
		IloInt V; // Nombre de vélos dans le système
		IloInt Tv; // temps de manipulation d'un vélo
		IloInt Ta ; // temps de manipulation d'une batterie
		IloInt N; // Nombre de stations (y compris le depot)
		IloInt Q ; // la capacité du camion de régulation
		Num3Matrix Flow(env,Tmax-Tmin);
		IloNumArray capacityStation(env,N) ;
		IloNumArray bikeSafety(env,N) ;
		IloNumArray rackSafety(env,N) ;
		IloNumArray M(env,_problem->getBatteries().size()) ;
		IloArray<IloNumArray> durations(env , N) ;



		/* initialisation */
		N=_problem->getNbStations();
		V =_problem->getNbBikes() ;
		Q=_problem->getCapacityTruck();
		Tv = _problem->getTm();
		//Ta = 1;


		// matrice de distance
		for(int i=0; i<N; i++ ){
			durations[i] = IloNumArray(env , N) ;
		}

		for(int i=0;i<N;i++){
			for(int j=0; j<N ; j++){
				durations[i][j]= _problem->getDistance(i,j);
			}
		}

		//  matrice des flows
		for(int i=0; i< Tmax-Tmin; i++) {
			Flow[i] = NumMatrix(env, N );
			for(int j=0; j< N; j++) {
				 Flow[i][j] = IloNumArray(env, N);
			}
		}

		for(int i=0;i<Tmax-Tmin;i++){
			for(int j=0; j<N ; j++){
				for(int k=0; k<N ; k++){
					Flow[i][j][k]=_problem->getFlow(i+Tmin,j,k);
				}
			}
		}

		// capacité des stations
		for(int i = 0 ; i<N ; i++){
			capacityStation[i]=_problem->getStation(i).getCapacity();
		}

		// nombre de vélos de sécurité
		for(int i = 0; i<N ; i++){
		    bikeSafety[i]=4;//_problem->getStation(i).getBikeSafety();
		}

		// nombre de porte vélos de sécurité
		for(int i = 0 ; i<N ; i++){
			rackSafety[i]=0;//_problem->getStation(i).getRackSafety();
		}

		// variables de décision


		// xijt = 1 si transfert de vélos de la station i à la station j à la période t
		Num3VarMatrix x(env,Tmax-Tmin);
		for(int t=0; t< Tmax-Tmin; t++) {
			x[t] = NumVarMatrix(env, N);
			for(int i=0; i< N; i++) {
				x[t][i] = IloNumVarArray(env, N);
				for(int j=0; j<N; j++) {
					x[t][i][j] = IloNumVar(env,0.0,1.0,ILOBOOL);
				 }
			}
		}


		// r_ijt : nombre de vélos qui doivent être transférés de la station i à la satation j à la période t
		Num3VarMatrix r(env,Tmax-Tmin);
		for(int t=0; t< Tmax-Tmin; t++) {
			  r[t] = NumVarMatrix(env, N);
			  for(int i=0; i< N; i++) {
				  r[t][i] = IloNumVarArray(env, N);
				  for(int j=0; j<N; j++) {
					  r[t][i][j] = IloNumVar(env,0.0,Q, ILOINT);
			   }
			  }
			 }



		//  v_it : nombre de vélos à chaque station
		IloArray< IloNumVarArray> v(env ,Tmax-Tmin) ;
		for(int t = 0 ; t< Tmax-Tmin ; t++){
			v[t] =  IloNumVarArray(env ,N);
			for( int i= 0 ; i< N ; i++){
				v[t][i] = IloNumVar(env,0,capacityStation[i],ILOINT);
			}
		}
		for(unsigned int i=0; i<_problem->getBatteries().size(); i++ ){
			M[i] = _problem->getBatteries(i);
		}

		//decision batteries
		IloArray< IloNumVarArray> b(env ,Tmax-Tmin) ;
		for(int t = 0 ; t< Tmax-Tmin ; t++){
			b[t] =  IloNumVarArray(env ,N);
			for( int i= 0 ; i< N ; i++){
				b[t][i] = IloNumVar(env,0.0,1.0,ILOBOOL);
			}
		}

		// fonction objective

		IloExpr fo (env) ;
		for(unsigned int  t = 0; t<Tmax-Tmin ; t++){
			for(unsigned int i=0 ; i<N ; i++){
				for(unsigned int j=0 ; j<N ; j++){
					fo +=  durations[i][j]*x[t][i][j]+ r[t][i][j] ;
				}
			}
		}
		model.add(IloMinimize(env,fo));

		// contraintes

		// contraintes de routage
/*		IloExpr expresion1(env);
		IloExpr expresion2(env);
		for(unsigned int  t = 1; t<Tmax-Tmin-1 ; t++){
			for(unsigned int i=1 ; i<N ; i++){
				for(unsigned int j=1 ; j<N ; j++){
					expresion1 += x[t][i][j] ;
					expresion2 += x[t][j][i] ;
				}
			model.add( expresion1== expresion2);
			expresion1.end();
			expresion2.end() ;
			expresion1 = IloExpr(env) ;
			expresion2 = IloExpr(env) ;
			}
		}
*/
/*		unsigned int k ;
		IloExpr expr (env);
		for(unsigned int j=0 ; j<_problem->getBatteries().size()  ; j++){// constraint batterie 1
			for(unsigned int t =0 ; t<Tmax-Tmin ; t++){
			k = M[j];
			expr += b[t][k] ;
			}
			model.add( expr == 1) ;
			expr.end();
			expr=IloExpr(env);
		}

		for(unsigned int t =0 ; t<Tmax-Tmin-1 ; t++){// constraint batterie 1
				for(unsigned int j=0 ; j< N ; j++){
				if (exist(j,_problem->getBatteries())== 0){
					model.add( b[t][j]==0) ;
				}
				}
			}


		IloExpr exprr(env) ;
		for(int t =0 ; t<Tmax-Tmin-1 ; t++){// constraint batterie 1
			for(int i=0 ; i<N ; i++){
				for(int j=0 ; j<N ; j++){
					model.add( b[t][j] <= x[t][i][j]) ;
				}
			}
		}

*/

       // contrainte sur la charge du camion
		for(int t =0 ; t<Tmax-Tmin ; t++){
			for(int i=1 ; i<N ; i++){
				for(int j=1 ; j<N ; j++){
					model.add(r[t][i][j] <= Q * x[t][i][j]) ;
				}
			}
		}


    // calcul de nombre de v_it+1 en fonction de v_it
	IloExpr expr1 (env) ;
	for(int t =0 ; t<Tmax-Tmin-1 ; t++){
		for(int i=1 ; i<N ; i++){
			for(int j=1 ; j<N ; j++){
				expr1 += -Flow[t][i][j]+Flow[t][j][i]-r[t][i][j]+r[t][j][i] ;
			}
			model.add( v[t+1][i] ==  v[t][i] + expr1 );
			expr1.end();
			expr1 = IloExpr(env) ;

		}
	}

	// contraintes sur les bike safety
	IloExpr expr2 (env) ;
	for(int t =0 ; t<Tmax-Tmin ; t++){
		for(int i=1 ; i<N ; i++){
			for(int j=1 ; j<N ; j++){
				expr2 += Flow[t][j][i]-Flow[t][i][j]-r[t][i][j];
			}
			model.add(v[t][i]+ expr2 >= bikeSafety[i]);
			expr2.end();
			expr2 = IloExpr(env) ;
		}
	}

	// contraintes sur les rack safety
	IloExpr expr3 (env) ;
	for(int t =0 ; t<Tmax-Tmin ; t++){
		for(int i=1 ; i<N ; i++){
			for(int j=1 ; j<N ; j++){
				expr3 += Flow[t][i][j]-Flow[t][j][i]-r[t][j][i] ;
			}
			model.add(capacityStation[i]-v[t][i]+ expr3 >= rackSafety[i]);
			expr3.end();
			expr3 = IloExpr(env) ;
		}
	}



	// le nombre de vélos dans le système est toujours égale à V
	IloExpr expr5(env) ;
	for(unsigned int t=0;t<Tmax-Tmin ; t++){
	for(int i=1 ; i<N ; i++){
		expr5+=v[t][i];
	}
	model.add(expr5 == V);
	expr5.end();
	expr5=IloExpr(env) ;
	}

	// aucune charge lorsqu'on part du dépot
	for(unsigned int j = 0 ; j<N ;j++ ){
		model.add(r[Tmax-Tmin-1][j][0] == 0) ;
	}

	for(unsigned int j = 0 ; j<N ;j++ ){
		model.add( r[0][0][j] == 0) ;
	}


	for(unsigned int t =0 ; t<Tmax-Tmin;t++){
		for(unsigned int j = 0 ; j<N ;j++ ){
		 model.add(x[t][j][j] == 0 );
		}
	}

/*	IloExpr midi(env);
	for(unsigned int j = 0 ; j<N ;j++ ){
		midi += x[26][j][0] ;
	}
		model.add(midi==1);*/

// affichage du model
	      IloCplex cplex ( model );
//	 env.out()<<cplex.getConflict(tab)<<endl;

//Paramètres du solver

	cplex.setParam(IloCplex::Param::TimeLimit,10); // Temps de Résolution
	cplex.exportModel("newM.Lp");
	cplex.solve();
    cplex.getCplexStatus();
    int a ;
		for(unsigned int t=0 ; t<Tmax-Tmin;t++){
				for(unsigned int i=0; i<N;i++){
					for(unsigned int j=0 ; j<N;j++){
						if(cplex.getValue(r[t][i][j])>= 0.5){
							a=cplex.getValue(r[t][i][j]);
							 _solution->setRequest(i,j,a,t);
							 a=0;
						}
					}
				}
		}


/*	for(unsigned int i=0 ; i<Tmax-Tmin;i++){
		for(unsigned int j=0; j<N;j++){
				env.out()<<"station : "<<_problem->getStation(j).getName()<<"--->"<<cplex.getValue(v[i][j])<<"  att time  "<<i+Tmin<<"\n";

		}
		cout<<endl;
	} */

	}
	catch (IloException& e)
	{
		cerr<<"erreur : exeption = " << e << endl ;
	}
	catch (...){
		cerr<<"error Cpp"<<endl;
	}
	env.end();
}

