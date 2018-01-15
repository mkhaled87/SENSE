/*
 * robot_infoft.cc
 *
 *  created on: 11.01.2017
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
      xx[0] = (x[0]*0) + u[0];
      xx[1] = (x[1]*0) + u[1];
  };
  ode_solver(rhs,x,u);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_type &r, input_type &u) -> void {
    r[0] = u[0]*0;
    r[1] = u[1]*0;
};

/* construct state space */
scots::SymbolicSet robotCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={0,0};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={63,63}; 
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


  double h2[4] = {-5,15,-20, 22};
  ss.remPolytope(4,H,h2, scots::OUTER);

  double h3[4] = {-15,17,-5, 22};
  ss.remPolytope(4,H,h3, scots::OUTER);

  double h4[4] = {-48,50,-45, 60};
  ss.remPolytope(4,H,h4, scots::OUTER);

  double h5[4] = {-51, 58,-45, 47};
  ss.remPolytope(4,H,h5, scots::OUTER);

  double h1[4] = {-27,36,-20, 45};
  ss.remPolytope(4,H,h1, scots::OUTER);

  double h6[4] = {-44, 49,-27, 36};
  ss.remPolytope(4,H,h6, scots::OUTER);

  double h7[4] = {-27, 36,-52, 57};
  ss.remPolytope(4,H,h7, scots::OUTER);

  double h8[4] = {-27,36,-5, 10};
  ss.remPolytope(4,H,h8, scots::OUTER);

  double h9[4] = {-14, 19,-27, 36};
  ss.remPolytope(4,H,h9, scots::OUTER);

 return ss;
}

/* construct input space */
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
  scots::SymbolicSet ts  = ss;
  scots::SymbolicSet ts1 = ss;
  scots::SymbolicSet ts2 = ss;

  /* define the target set as a symbolic set */
  double H[4*sDIM]={-1, 0,
                    1, 0,
                    0,-1,
                    0, 1};
  /* compute inner approximation of P={ x | H x<= h1 }  */
  double h1[4] = {-5,15,-45,55};
  ts1.addPolytope(4,H,h1, scots::OUTER);
  ts.addPolytope(4,H,h1, scots::OUTER);

  double h2[4] = {-45,55,-5,15};
  ts2.addPolytope(4,H,h2, scots::OUTER);
  ts.addPolytope(4,H,h2, scots::OUTER);

  ts1.writeToFile("robot_ts1.bdd");
  ts2.writeToFile("robot_ts2.bdd");
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
  BDD T1 = ts1.getSymbolicSet();
  BDD T2 = ts2.getSymbolicSet();

  /* the output of the fixed point computation are two bdd's */
  tt.tic();

  /* we implement the nested fixed point algorithm
   *
   * mu X. nu Y. ( pre(Y) & T ) | pre(X)
   *
   */
  tt.tic();
  size_t i,j;
  /* outer fp*/
  BDD X=mgr.bddZero();
  BDD XX=mgr.bddOne();

  /* inner fp*/
  BDD Y1=mgr.bddOne();
  BDD YY1=mgr.bddZero();
  BDD Y2=mgr.bddOne();
  BDD YY2=mgr.bddZero();

  /* the controller */
  BDD C1=mgr.bddZero();
  BDD C2=mgr.bddZero();
  BDD U=is.getCube();

  for(i=1; XX != X; i++) {
    X=XX;
    BDD preX=fp.pre(X);


    /* init inner fp */
    YY1 = mgr.bddZero();
    for(j=1; YY1 != Y1; j++) {
      Y1=YY1;
      YY1= (( preX & T1 )) | fp.pre(Y1);
      BDD N1 = YY1 & (!(C1.ExistAbstract(U)));
      C1 = C1 | N1;
    }
    std::cout << "Iterations inner1: " << j << std::endl;

    /* init inner fp */
    YY2 = mgr.bddZero();
    for(j=1; YY2 != Y2; j++) {
      Y2=YY2;
      YY2= (( preX & T2 )) | fp.pre(Y2);
      BDD N2 = YY2 & (!(C2.ExistAbstract(U)));
      C2 = C2 | N2;
    }
    std::cout << "Iterations inner2: " << j << std::endl;

    XX=YY1 & YY2;



    /* remove all (state/input) pairs that have been added
     * to the controller already in the previous iteration * */
//    BDD N2 = YY2 & (!(C2.ExistAbstract(U)));
//    BDD N2 = YY2 & (!(C2.ExistAbstract(U)));

    /* add the remaining pairs to the controller */
//    if(i == 1){
//    C1=C1 | N1;
//    C2=C2 | N1;
//    }
  }
  std::cout << "Iterations outer: " << i << std::endl;


  tt.toc();


  scots::SymbolicSet cont1(ss,is);
  cont1.setSymbolicSet(C1);
  cont1.writeToFile("robot_cont1.bdd");

  scots::SymbolicSet cont2(ss,is);
  cont2.setSymbolicSet(C2);
  cont2.writeToFile("robot_cont2.bdd");

  return 1;
}


