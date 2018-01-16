/*
 * robot.cc
 *
 *  created on: 15.11.2016
 *      author: M. Khaled
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


/* state space dim */
#define sDIM 2
#define iDIM 2

/* data types for the ode solver */
typedef std::array<double,2> state_type;
typedef std::array<double,2> input_type;

/* sampling time */
const double tau = 1;
/* number of intermediate steps in the ode solver */
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);


/* we integrate the robot ode by 0.3 sec (the result is stored in x)  */
auto  robot_post = [](state_type &x, input_type &u) -> void {

  /* the ode describing the robot */
  auto rhs =[](state_type& xx,  const state_type &x, input_type &u) -> void {     
      double x0=x[0];
      x0=x0-1;
      xx[0] = u[0];
      xx[1] = u[1];
  };
  ode_solver(rhs,x,u);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_type &r, input_type &u) -> void {
    u[0]=u[0];
    r[0] = 0;
    r[1] = 0;
};


/* forward declaration of the functions to setup the state space 
 * and input space of the robot example */
scots::SymbolicSet robotCreateStateSpace(Cudd &mgr);
scots::SymbolicSet robotCreateInputSpace(Cudd &mgr);


int main() {
  TicToc tt;
  Cudd mgr;

  /****************************************************************************/
  /* construct SymbolicSet for the state space */
  /****************************************************************************/
  scots::SymbolicSet ss=robotCreateStateSpace(mgr);
  ss.writeToFile("robot_ss.bdd");

  /* write SymbolicSet of obstacles to robot_obst.bdd */
  ss.complement();
  ss.writeToFile("robot_obst.bdd");
  ss.complement();

  std::cout << "Unfiorm grid details:" << std::endl;
  ss.printInfo(1);

  /****************************************************************************/
  /* we define the target set */
  /****************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  scots::SymbolicSet ts = ss;

  /* define the target set as a symbolic set */
  double H[4*sDIM]={-1, 0,
                    1, 0,
                    0,-1,
                    0, 1};
  /* compute inner approximation of P={ x | H x<= h1 }  */
  double h1[4] = {-12,13,-12,13};
  ts.addPolytope(4,H,h1, scots::OUTER);

  double h2[4] = {-12,13,-2,3};
  ts.addPolytope(4,H,h2, scots::OUTER);

  ts.writeToFile("robot_ts.bdd");


  /****************************************************************************/
  /* construct SymbolicSet for the input space */
  /****************************************************************************/
  scots::SymbolicSet is=robotCreateInputSpace(mgr);
  is.writeToFile("robot_is.bdd");
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
  abstraction.computeTransitionRelation(robot_post, radius_post);
  std::cout << std::endl;
  tt.toc();

  abstraction.getTransitionRelation().writeToFile("robot_rel.bdd");

  /* get the number of elements in the transition relation */
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction.getSize() << std::endl;

  /****************************************************************************/
  /* we continue with the controller synthesis */
  /****************************************************************************/
  /* we setup a fixed point object to compute reachabilty controller */
  scots::FixedPoint fp(&abstraction);
  /* the fixed point algorithm operates on the BDD directly */
  BDD T = ts.getSymbolicSet();
  /* the output of the fixed point computation are two bdd's */
  tt.tic();

  /* we implement the nested fixed point algorithm
   *
   * mu X. nu Y. ( pre(Y) & T ) | pre(X)
   *
   */
  //BDD C = fp.reach(T, 1);
  size_t i,j;
  /* outer fp*/
  BDD X=mgr.bddOne();
  BDD XX=mgr.bddZero();
  /* inner fp*/
  BDD Y=mgr.bddZero();
  BDD YY=mgr.bddOne();
  /* the controller */
  BDD C=mgr.bddZero();
  BDD U=is.getCube();
  /* as long as not converged */
  for(i=1; XX != X; i++) {
    X=XX;
    BDD preX=fp.pre(X);
    /* init inner fp */
    YY = mgr.bddOne();
    for(j=1; YY != Y; j++) {
      Y=YY;
      YY= ( fp.pre(Y) & T ) | preX;
    }
    XX=YY;
    std::cout << "Iterations inner: " << j << std::endl;
    /* remove all (state/input) pairs that have been added
     * to the controller already in the previous iteration * */
    BDD N = XX & (!(C.ExistAbstract(U)));
    /* add the remaining pairs to the controller */
    C=C | N;
    //std::cout << C.CountMinterm(17) << std::endl;
  }
  std::cout << "Iterations outer: " << i << std::endl;
  tt.toc();

  /****************************************************************************/
  /* last we store the controller as a SymbolicSet 
   * the underlying uniform grid is given by the Cartesian product of 
   * the uniform gird of the space and uniform gird of the input space */
  /****************************************************************************/
  scots::SymbolicSet controller(ss,is);
  controller.setSymbolicSet(C);
  controller.writeToFile("robot_controller.bdd");

  return 1;
}

scots::SymbolicSet robotCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={0,0};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={15,15}; 
  /* grid node distance diameter */
  double eta[sDIM]={1,1};   


  /* eta is added to the bound so as to ensure that the whole
   * [0,10]x[0,10]x[-pi-eta,pi+eta] is covered by the cells */

  scots::SymbolicSet ss(mgr,sDIM,lb,ub,eta);

  /* add the grid points to the SymbolicSet ss */
  ss.addGridPoints();


  /* remove the obstacles from the state space */
  /* the obstacles are defined as polytopes */
  /* define H* x <= h */

  double H[4*sDIM]={-1, 0,
                    1, 0,
                    0,-1,
                    0, 1,};


  double h1[4] = {-3,6,-3, 6};
  ss.remPolytope(4,H,h1, scots::OUTER);


 return ss;
}

scots::SymbolicSet robotCreateInputSpace(Cudd &mgr) {

  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={-1,-1};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={1,1}; 
  /* grid node distance diameter */
  double eta[sDIM]={1,1};   

  scots::SymbolicSet is(mgr,iDIM,lb,ub,eta);
  is.addGridPoints();

  return is;
}
