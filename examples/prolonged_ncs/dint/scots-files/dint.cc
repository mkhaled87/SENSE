/*
 * dcdc.cc
 *
 *  created on: 31.08.2015
 *      author: m.khaled
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
typedef std::array<double,2> state_type;
typedef std::array<double,1> input_type;

/* sampling time */
const double tau = 0.30;
/* number of intermediate steps in the ode solver */
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);



auto  dint_post = [](state_type &x, input_type &u) -> void {

  auto  system_ode = [](state_type &dxdt, const state_type &x, const input_type &u) -> void {

    const double b[2]    =  {0, 1};
    const double a[2][2] = {{0, 1},
							{0, 0}};
							
    dxdt[0] = a[0][0]*x[0]+a[0][1]*x[1] + b[0]*u[0];
    dxdt[1] = a[1][0]*x[0]+a[1][1]*x[1] + b[1]*u[0];
  };
  ode_solver(system_ode,x,u);
};


auto radius_post = [](state_type &r, input_type &u) -> void {

	/*
	auto growth_bound_ode = [](state_type &drdt,  const state_type &r, const input_type &u) {

    const double a[2][2] = {{0, 1},
							{0, 0}};
							
    drdt[0] = a[0][0]*r[0]+a[0][1]*r[1];
    drdt[1] = a[1][0]*r[0]+a[1][1]*r[1];
  };
  ode_solver(growth_bound_ode,r,u);
  */
   u[0]=u[0];
  r[0]=0;
  r[1]=0;
};


int main() {
  
  TicToc tt; 
  Cudd mgr;

  /****************************************************************************/
  /* construct SymbolicSet for the state space */
  /****************************************************************************/ 
  double xlb[sDIM]	={0.60,0.30};  
  double xub[sDIM]	={3.40,3.30}; 
  double xeta[sDIM]	={0.20,0.30};
  
  scots::SymbolicSet ss(mgr,sDIM,xlb,xub,xeta);
  ss.addGridPoints();
  ss.writeToFile("dint_ss.bdd");

  /****************************************************************************/
  /* construct SymbolicSet for the input space */
  /****************************************************************************/
  double ulb[iDIM]	 ={0.00};  
  double uub[iDIM]	 ={1.00}; 
  double ueta[iDIM]  	 ={0.3};   
  
  scots::SymbolicSet is(mgr,iDIM,ulb,uub,ueta);
  is.addGridPoints();
  is.writeToFile("dint_is.bdd");

  /****************************************************************************/
  /* setup class for symbolic model computation */
  /****************************************************************************/
 
  scots::SymbolicSet sspost(ss,1);
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction(&ss, &is, &sspost);

  tt.tic();
  abstraction.computeTransitionRelation(dint_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction.getSize() << std::endl;
  abstraction.getTransitionRelation().writeToFile("dint_rel.bdd");

  /****************************************************************************/
  /* we continue with the specification:: target set to reach */
  /****************************************************************************/
  
  double H[4*sDIM]={-1, 0,
                     1, 0,
                     0,-1,
                     0, 1};  
  double h[4] = {-2.4,3.4,-1.5, 3.3};
//  double h[4] = {-2.85,3.3,-2.5, 3.1};

  scots::SymbolicSet ts(ss);
  ts.addPolytope(4,H,h, scots::INNER);
  ts.writeToFile("dint_ts.bdd");

  /****************************************************************************/
  /* we continue with the controller synthesis */
  /****************************************************************************/
  scots::FixedPoint fp(&abstraction);
  /* the fixed point algorithm operates on the BDD directly */
  BDD T = ts.getSymbolicSet();
  tt.tic();
  BDD C=fp.reach(T, 1);
  tt.toc();  

  /****************************************************************************/
  /* last we store the controller as a SymbolicSet 
   * the underlying uniform grid is given by the Cartesian product of 
   * the uniform gird of the space and uniform gird of the input space */
  /****************************************************************************/
  scots::SymbolicSet cont(ss,is);
  cont.setSymbolicSet(C);
  cont.writeToFile("dint_cont.bdd");
  std::cout << "Controller size: " << cont.getSize() << std::endl;

  return 1;
}
