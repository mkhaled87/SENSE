/*
 * pip.cc
 *
 *  created on: 08.12.2016
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
#define sDIM 4
#define iDIM 1

/* data types for the ode solver */
typedef std::array<double,sDIM> state_type;
typedef std::array<double,iDIM> input_type;

/* sampling time */
const double tau = 0.1;

/* number of intermediate steps in the ode solver */
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);


/* we integrate the vehicle ode by 0.3 sec (the result is stored in x)  */
auto  sys_post = [](state_type &s, input_type &u) -> void {

  /* the ode describing the vehicle */
  auto rhs =[](state_type& sd,  const state_type &s, input_type &u) -> void {
    double th  = s[0];
    double thd = s[1];

    double xd  = s[3];    
    double F   = u[0];

    double Mc = 0.25;
    double Mp = 0.5;
    double L  = 0.2;
    double g  = 9.807;
   
    sd[0] = thd;
    sd[1] = ( (F*cos(th)) -(Mp*L*(thd*thd)*sin(th)*cos(th)) + ((Mc+Mp)*g*sin(th)) ) / ( (-Mp*L*(cos(th)*cos(th))) + ((Mc+Mp)*L) );
    sd[2] = xd;
    sd[3] = (L*sd[1]- g*sin(th))/cos(th);
  };

  ode_solver(rhs,s,u);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_type &r, input_type &u) -> void {
    u[0]=u[0];
    r[0] = 0.1;
    r[1] = 0.4;
    r[2] = 0.6;
    r[3] = 0.6;
};


/* forward declaration of the functions to setup the state space 
 * and input space of the vehicle example */
scots::SymbolicSet sysCreateStateSpace(Cudd &mgr);
scots::SymbolicSet sysCreateInputSpace(Cudd &mgr);


int main() {
  /* to measure time */
  TicToc tt;
  /* there is one unique manager to organize the bdd variables */
  Cudd mgr;

  /****************************************************************************/
  /* construct SymbolicSet for the state space */
  /****************************************************************************/
  scots::SymbolicSet ss=sysCreateStateSpace(mgr);
  ss.writeToFile("pip_ss.bdd");

  /****************************************************************************/
  /* we define the safe/target set */
  /****************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  scots::SymbolicSet ts = ss;
  /* define the target set as a symbolic set */
  double H[8*sDIM]={-1, 0, 0, 0,
                     1, 0, 0, 0,
		     0,-1, 0, 0,
                     0, 1, 0, 0,
		     0, 0,-1, 0,
                     0, 0, 1, 0,
		     0, 0, 0,-1,
                     0, 0, 0, 1};
  /* compute inner approximation of P={ x | H x<= h1 }  */
  double h[2] = {0.15,0.15};
  ts.addPolytope(2,H,h, scots::INNER);
  ts.writeToFile("pip_ts.bdd");

  /****************************************************************************/
  /* construct SymbolicSet for the input space */
  /****************************************************************************/
  scots::SymbolicSet is=sysCreateInputSpace(mgr);

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
  abstraction.computeTransitionRelation(sys_post, radius_post);
  std::cout << std::endl;
  tt.toc();

  abstraction.getTransitionRelation().writeToFile("pip_rel.bdd");

  /* get the number of elements in the transition relation */
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction.getSize() << std::endl;

  return 1;
}

scots::SymbolicSet sysCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={-M_PI/4, -M_PI, -4.5, -4.5};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={ M_PI/4,  M_PI,  4.5,  4.5}; 
  /* grid node distance diameter */
  double eta[sDIM]={0.05,0.2,0.3,0.3};   


  /* eta is added to the bound so as to ensure that the whole
   * [0,10]x[0,10]x[-pi-eta,pi+eta] is covered by the cells */

  scots::SymbolicSet ss(mgr,sDIM,lb,ub,eta);
  ss.addGridPoints();

  std::cout << std::endl << "State space details:" << std::endl;
  ss.printInfo(0);

 return ss;
}

scots::SymbolicSet sysCreateInputSpace(Cudd &mgr) {

  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={-1.5};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={+1.5}; 
  /* grid node distance diameter */
  double eta[sDIM]={0.1};   

  scots::SymbolicSet is(mgr,iDIM,lb,ub,eta);
  is.addGridPoints();

  std::cout << std::endl << "Input space details:" << std::endl;
  is.printInfo(0);

  return is;
}
