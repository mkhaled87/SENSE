/*
 * fifo.cc
 *
 *  created on: 16.08.2016
 *      author: M. Khaled
 */


#include <array>
#include <iostream>

#include "cuddObj.hh"

#include "SymbolicSet.hh"
#include "SymbolicModelGrowthBound.hh"
#include "SymbolicModel.hh"

#include "TicToc.hh"
#include "RungeKutta4.hh"
#include "FixedPoint.hh"

using namespace std;
using namespace scots;

#define SS_DIM 1
#define IS_DIM 1

typedef array<double,1> state_type;
typedef array<double,1> input_type;

bool cbFunc_add_ts(double* x);
bool cbFunc_add_obs(double* x);
void sys_post(state_type &x, input_type &u);
void r_post(state_type &r, input_type &u);

int main() {

  system("rm *.bdd");

  /* to measure time */
  TicToc tt;
  
  /* there is one unique manager to organize the bdd variables */
  Cudd mgr;

  double sslb[1]={1};
  double ssub[1]={13};
  double sseta[1]={1};
  
  double islb[1]={1};
  double isub[1]={2};
  double iseta[1]={1};
  
  /****************************************************************************/
  /* construct SymbolicSet for the state space and input */
  /****************************************************************************/
  SymbolicSet ss(mgr,SS_DIM,sslb,ssub,sseta);
  ss.addGridPoints();
  ss.writeToFile("state13_ss.bdd");
  
  SymbolicSet is(mgr,IS_DIM,islb,isub,iseta);
  is.addGridPoints();
  is.writeToFile("state13_is.bdd");

  SymbolicSet sspost(mgr,SS_DIM,sslb,ssub,sseta);
  sspost.addGridPoints();
  sspost.writeToFile("state13_sspost.bdd");

  /****************************************************************************/
  /* construct SymbolicSet for the obstacles */
  /****************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  SymbolicSet obs(ss);
  obs.addByFunction(cbFunc_add_obs);
  obs.writeToFile("state13_obst.bdd");

  /****************************************************************************/
  /* we define the target set */
  /****************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  SymbolicSet ts(ss);
  ts.addByFunction(cbFunc_add_ts);
  ts.writeToFile("state13_ts.bdd");


  /****************************************************************************/
  /* we define t fhe abstraction */
  /****************************************************************************/
  SymbolicModelGrowthBound<state_type, input_type> abstraction(&ss,&is,&sspost);
  abstraction.computeTransitionRelation(sys_post, r_post);

  abstraction.getTransitionRelation().writeToFile("state13_rel.bdd");


  /****************************************************************************/
  /* we continue with the controller synthesis */
  /****************************************************************************/
  /* we setup a fixed point object to compute reachabilty controller */
  FixedPoint fp(&abstraction);
  /* the fixed point algorithm operates on the BDD directly */
  BDD T = ts.getSymbolicSet();
  //BDD O = obs.getSymbolicSet();
  
  /* compute controller */
  //BDD C=fp.reachAvoid(T,O,1);
  BDD C=fp.reach(T,1);

  /****************************************************************************/
  /* last we store the controller as a SymbolicSet 
   * the underlying uniform grid is given by the Cartesian product of 
   * the uniform gird of the space and uniform gird of the input space */
  /****************************************************************************/
  scots::SymbolicSet controller(ss,is);
  controller.setSymbolicSet(C);
  controller.writeToFile("state13_controller.bdd");

  return 1;
}


bool cbFunc_add_ts(double* x)
{
	if(x[0] == 12)
		return true;
	else
		return false;
}

bool cbFunc_add_obs(double* x)
{
	if(x[0] == 11 || x[0] == 8)
			return true;
	else
			return false;
}


void sys_post(state_type &x, input_type &u)
{
	if(x[0] == 1 && u[0] == 1) { x[0] = 2; return; }
	if(x[0] == 1 && u[0] == 2) { x[0] = 1; return; }

	if(x[0] == 2 && u[0] == 1) { x[0] = 1; return; }
	if(x[0] == 2 && u[0] == 2) { x[0] = 3; return; }
	
	if(x[0] == 3 && u[0] == 1) { x[0] = 3; return; }
	if(x[0] == 3 && u[0] == 2) { x[0] = 4; return; }

	if(x[0] == 4 && u[0] == 1) { x[0] = 6; return; }
	if(x[0] == 4 && u[0] == 2) { x[0] = 5; return; }

	if(x[0] == 5 && u[0] == 1) { x[0] = 4; return; }
	if(x[0] == 5 && u[0] == 2) { x[0] = 5; return; }

	if(x[0] == 6 && u[0] == 1) { x[0] = 7; return; }
	if(x[0] == 6 && u[0] == 2) { x[0] = 5; return; }
	
	if(x[0] == 7 && u[0] == 1) { x[0] = 13; return; }
	if(x[0] == 7 && u[0] == 2) { x[0] = 8; return; }

	if(x[0] == 8 && u[0] == 1) { x[0] = 11; return; }
	if(x[0] == 8 && u[0] == 2) { x[0] = 9; return; }

	if(x[0] == 9 && u[0] == 1) { x[0] = 10; return; }
	if(x[0] == 9 && u[0] == 2) { x[0] = 3; return; }

	if(x[0] == 10 && u[0] == 1) { x[0] = 9; return; }
	if(x[0] == 10 && u[0] == 2) { x[0] = 11; return; }
	
	if(x[0] == 11 && u[0] == 1) { x[0] = 10; return; }
	if(x[0] == 11 && u[0] == 2) { x[0] = 13; return; }

	if(x[0] == 12 && u[0] == 1) { x[0] = 11; return; }
	if(x[0] == 12 && u[0] == 2) { x[0] = 12; return; }

	if(x[0] == 13 && u[0] == 1) { x[0] = 11; return; }
	if(x[0] == 13 && u[0] == 2) { x[0] = 12; return; }	
}

/* computation of the growth bound (the result is stored in r)  */
void r_post (state_type &r, input_type &u)
{
	r[0] = 0;
}
