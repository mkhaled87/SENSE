/*
 * vehicle.cc
 *
 *  created on: 27.12.2016
 *      author: M.Khaled
 */

#include <array>
#include <iostream>

#include "cuddObj.hh"

#include "SymbolicSet.hh"
#include "SymbolicModelGrowthBound.hh"

#include "TicToc.hh"
#include "RungeKutta4.hh"
#include "FixedPoint.hh"


/* state space dim */
#define sDIM 2
#define iDIM 1

/* data types for the ode solver */
typedef std::array<double,sDIM> state_type;
typedef std::array<double,iDIM> input_type;

/* sampling time */
const double tau = 0.3;
/* number of intermediate steps in the ode solver */
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);

double omega_squared = 1;
/* we integrate the vehicle ode by 0.3 sec (the result is stored in x)  */
auto  sys_post = [](state_type &x, input_type &u) -> void {

  /* the ode describing the vehicle */
  auto rhs =[](state_type& xx,  const state_type &x, input_type &u) {
      xx[0] = -x[1] -1.5*x[0]*x[0] -0.5*x[0]*x[0]*x[0];
      xx[1] = (1/omega_squared)*(x[0] - u[0]);
  };
  ode_solver(rhs,x,u);
};

/* computation of the growth bound (the result is stored in r)  */
auto sys_growth_bound = [](state_type &r, input_type &u) {

	double a = -3.0*r[0]-1.5*r[0]*r[0];
	double b = 1/omega_squared;

	double eLtau_3 = a*a + 4.0*b;;
	double eLtau_1 = std::exp(a/2 - std::sqrt(eLtau_3)/2.0);
	double eLtau_2 = std::exp(a/2 - std::sqrt(eLtau_3)/2.0);

	double eLtau[2][2];
	eLtau[0][0] = tau*((eLtau_1*std::sqrt(eLtau_3) + eLtau_2*std::sqrt(eLtau_3) - a*eLtau_1 + a*eLtau_2)/(2.0*std::sqrt(eLtau_3)));
	eLtau[0][1] = tau*((a*a*eLtau_1 - a*a*eLtau_2 - eLtau_1*eLtau_3 + eLtau_2*eLtau_3)/(4.0*std::sqrt(eLtau_3)));
	eLtau[1][0] = tau*((eLtau_1-eLtau_2)/(std::sqrt(eLtau_3)));
	eLtau[1][1] = tau*((eLtau_1*std::sqrt(eLtau_3) + eLtau_2*std::sqrt(eLtau_3)+a*(eLtau_1+eLtau_2))/(2.0*std::sqrt(eLtau_3)));

	r[0] = r[0]*eLtau[0][0] + r[1]*eLtau[0][1];
	r[1] = r[0]*eLtau[1][0] + r[1]*eLtau[1][1];
};


scots::SymbolicSet sysCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={-1.9,-1.9};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={+1.9,+1.9}; 
  /* grid node distance diameter */
  double eta[sDIM]={0.025,0.025};   


  scots::SymbolicSet ss(mgr,sDIM,lb,ub,eta);

  /* add the grid points to the SymbolicSet ss */
  ss.addGridPoints();

 return ss;
}

scots::SymbolicSet sysCreateInputSpace(Cudd &mgr) {

  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={-2.0};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={+2.0}; 
  /* grid node distance diameter */
  double eta[sDIM]={0.1};   

  scots::SymbolicSet is(mgr,iDIM,lb,ub,eta);
  is.addGridPoints();

  return is;
}

int main() {
  /* to measure time */
  TicToc tt;
  /* there is one unique manager to organize the bdd variables */
  Cudd mgr;

  /****************************************************************************/
  /* construct SymbolicSet for the state space */
  /****************************************************************************/
  std::cout << "Constructing state-space ... ";
  scots::SymbolicSet ss=sysCreateStateSpace(mgr);
  ss.printInfo();
  ss.writeToFile("sys_ss.bdd");

  /****************************************************************************/
  /* construct SymbolicSet for the input space */
  /****************************************************************************/
  std::cout << "Constructing input-space ... ";
  scots::SymbolicSet is=sysCreateInputSpace(mgr);
  is.printInfo();
  is.writeToFile("sys_is.bdd");

  /****************************************************************************/
  /* setup class for symbolic model computation */
  /****************************************************************************/
  std::cout << "Constructing the symbolic-model ... " << std::endl;
  /* first create SymbolicSet of post variables 
   * by copying the SymbolicSet of the state space and assigning new BDD IDs */
  scots::SymbolicSet sspost(ss,1);
  /* instantiate the SymbolicModel */
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction(&ss, &is, &sspost);
  /* compute the transition relation */
  tt.tic();
  abstraction.computeTransitionRelation(sys_post, sys_growth_bound);
  std::cout << std::endl;
  tt.toc();
  /* get the number of elements in the transition relation */
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction.getSize() << std::endl;
  abstraction.getTransitionRelation().writeToFile("sys_rel.bdd");



  return 1;
}

