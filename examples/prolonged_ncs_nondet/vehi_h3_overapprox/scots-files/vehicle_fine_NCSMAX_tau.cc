/*
 * vehicle_exp2.cc
 *
 *  created on: 30.09.2015
 *      author: rungger
 */

/*
 * information about this example is given in the readme file
 *
 */

#include <array>
#include <iostream>

#include "cuddObj.hh"

#include "SymbolicSet.hh"
#include "SymbolicModelGrowthBound.hh"

#include "TicToc.hh"
#include "RungeKutta4.hh"
#include "FixedPoint.hh"

#define TAU_MULTIPLIER 2
#define SS_ETA {.05,.05,.05}


/* state space dim */
#define sDIM 3
#define iDIM 2

/* data types for the ode solver */
typedef std::array<double,3> state_type;
typedef std::array<double,2> input_type;

/* sampling time */
const double tau = TAU_MULTIPLIER*0.3;
/* number of intermediate steps in the ode solver */
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);


/* we integrate the vehicle ode by 0.3 sec (the result is stored in x)  */
auto  vehicle_post = [](state_type &x, input_type &u) -> void {

  /* the ode describing the vehicle */
  auto rhs =[](state_type& xx,  const state_type &x, input_type &u) -> void {
      double alpha=std::atan(std::tan(u[1])/2.0);
      xx[0] = u[0]*std::cos(alpha+x[2])/std::cos(alpha);
      xx[1] = u[0]*std::sin(alpha+x[2])/std::cos(alpha);
      xx[2] = u[0]*std::tan(u[1]);
  };
  ode_solver(rhs,x,u);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_type &r, input_type &u) -> void {
    double c = std::abs(u[0]*std::sqrt(std::tan(u[1])*std::tan(u[1])/4.0+1));
    r[0] = r[0]+c*r[2]*tau;
    r[1] = r[1]+c*r[2]*tau;
	
	static bool done = false;
	
	static double r0 = r[0];
	static double r1 = r[1];
	
	if(!done){
		done = true;
		std::cout << std::endl << "Initial Grouth bound: " << r0 << ", " << r1 << ", "  << r[2] << std::endl;
	}
	
	if(r[0] > r0 || r[1] > r1){
		
		r0 = r[0];
		r1 = r[1];
		
		std::cout << "New bigger grouth bound: " << r0 << r1 << r[2] << std::endl;
	}
	
};


/* forward declaration of the functions to setup the state space 
 * and input space of the vehicle example */
scots::SymbolicSet vehicleCreateStateSpace(Cudd &mgr);
scots::SymbolicSet vehicleCreateInputSpace(Cudd &mgr);


int main() {
  /* to measure time */
  TicToc tt;
  /* there is one unique manager to organize the bdd variables */
  Cudd mgr;

  /****************************************************************************/
  /* construct SymbolicSet for the state space */
  /****************************************************************************/
  scots::SymbolicSet ss=vehicleCreateStateSpace(mgr);
  ss.writeToFile("vehicle_ss_fine_2_tau.bdd");
  
  
  std::cout << "Unfiorm grid details:" << std::endl;
  ss.printInfo(1);

  
  /****************************************************************************/
  /* construct SymbolicSet for the input space */
  /****************************************************************************/
  scots::SymbolicSet is=vehicleCreateInputSpace(mgr);
  std::cout << std::endl << "Input space details:" << std::endl;
  is.printInfo(0);

  /****************************************************************************/
  /* setup class for symbolic model computation */
  /****************************************************************************/
  /* first create SymbolicSet of post variables 
   * by copying the SymbolicSet of the state space and assigning new BDD IDs */
  scots::SymbolicSet sspost(ss,1);
  /* instantiate the SymbolicModel */
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction(&ss, &is, &sspost);
  /* compute the transition relation */
  tt.tic();
  abstraction.computeTransitionRelation(vehicle_post, radius_post);
  std::cout << std::endl;
  tt.toc();

  abstraction.getTransitionRelation().writeToFile("vehicle_rel_fine_2_tau.bdd");

  /* get the number of elements in the transition relation */
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction.getSize() << std::endl;

  return 1;
}

scots::SymbolicSet vehicleCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={0,0,-M_PI-0.4};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={6,5,M_PI+0.4}; 
  /* grid node distance diameter */
  double eta[sDIM]=SS_ETA;   


  scots::SymbolicSet ss(mgr,sDIM,lb,ub,eta);

  /* add the grid points to the SymbolicSet ss */
  ss.addGridPoints();

 return ss;
}

scots::SymbolicSet vehicleCreateInputSpace(Cudd &mgr) {

  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={-1,-1};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={1,1}; 
  /* grid node distance diameter */
  double eta[sDIM]={.3,.3};   

  scots::SymbolicSet is(mgr,iDIM,lb,ub,eta);
  is.addGridPoints();

  return is;
}
