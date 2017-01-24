/*
 * robot.cc
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
#define sDIM 4
#define iDIM 4

/* data types for the ode solver */
typedef std::array<double,sDIM> state_type;
typedef std::array<double,iDIM> input_type;

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
      xx[2] = (x[2]*0) + u[2];
      xx[3] = (x[3]*0) + u[3];
  };
  ode_solver(rhs,x,u);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_type &r, input_type &u) -> void {
    r[0] = u[0]*0;
    r[1] = u[1]*0;
    r[2] = u[2]*0;
    r[3] = u[3]*0;
};

/* State Space Params */
#define ARENA_WIDTH 16
#define ARENA_HIGHT 16

#define SS_X1_LB 0
#define SS_X2_LB 0
#define SS_X3_LB SS_X1_LB
#define SS_X4_LB SS_X2_LB
#define SS_X1_UB (ARENA_WIDTH-1)
#define SS_X2_UB (ARENA_HIGHT-1)
#define SS_X3_UB SS_X1_UB
#define SS_X4_UB SS_X2_UB
#define SS_X1_MU 1
#define SS_X2_MU 1
#define SS_X3_MU SS_X1_MU
#define SS_X4_MU SS_X2_MU

/* Input Space Params */
#define IS_U1_LB -1
#define IS_U2_LB -1
#define IS_U3_LB -1
#define IS_U4_LB -1
#define IS_U1_UB 1
#define IS_U2_UB 1
#define IS_U3_UB 1
#define IS_U4_UB 1
#define IS_U1_MU 1
#define IS_U2_MU 1
#define IS_U3_MU 1
#define IS_U4_MU 1

/* Target Set Params*/
#define T_CUBE_WIDTH 1
#define T_MARGIN 2

#define T1_X1_LB (SS_X1_LB+T_MARGIN)
#define T1_X2_LB (SS_X2_LB+T_MARGIN)
#define T1_X1_UB (T1_X1_LB+T_CUBE_WIDTH)
#define T1_X2_UB (T1_X2_LB+T_CUBE_WIDTH)

#define T2_X1_LB (SS_X1_UB-T_MARGIN-T_CUBE_WIDTH)
#define T2_X2_LB (SS_X2_UB-T_MARGIN-T_CUBE_WIDTH)
#define T2_X1_UB (T2_X1_LB+T_CUBE_WIDTH)
#define T2_X2_UB (T2_X2_LB+T_CUBE_WIDTH)

#define T3_X3_LB (SS_X3_LB+T_MARGIN)
#define T3_X4_LB (SS_X4_UB-T_MARGIN-T_CUBE_WIDTH)
#define T3_X3_UB (T3_X3_LB+T_CUBE_WIDTH)
#define T3_X4_UB (T3_X4_LB+T_CUBE_WIDTH)

#define T4_X3_LB (SS_X3_UB-T_MARGIN-T_CUBE_WIDTH)
#define T4_X4_LB (SS_X4_LB+T_MARGIN)
#define T4_X3_UB (T4_X3_LB+T_CUBE_WIDTH)
#define T4_X4_UB (T4_X4_LB+T_CUBE_WIDTH)

/* Target Set Params*/
// Center box
#define O1_BOX_W 3
#define O1_X1_LB ((SS_X1_UB/2) - O1_BOX_W + 2)
#define O1_X2_LB ((SS_X2_UB/2) - O1_BOX_W + 2)
#define O1_X1_UB (O1_X1_LB + O1_BOX_W)
#define O1_X2_UB (O1_X2_LB + O1_BOX_W)

// Right Rectangle
#define O2_REC_W 3
#define O2_REC_H 1
#define O2_X1_LB (SS_X1_UB - O2_REC_W - 1)
#define O2_X2_LB ((SS_X2_UB/2) - O2_REC_H + 1)
#define O2_X1_UB (O2_X1_LB + O2_REC_W)
#define O2_X2_UB (O2_X2_LB + O2_REC_H)


// Left Rectangle
#define O3_REC_W 3
#define O3_REC_H 1
#define O3_X1_LB (SS_X1_LB + 1)
#define O3_X2_LB ((SS_X2_UB/2) - O3_REC_H + 1)
#define O3_X1_UB (O3_X1_LB + O3_REC_W)
#define O3_X2_UB (O3_X2_LB + O3_REC_H)

// Bottom Rectangle
#define O4_REC_W 1
#define O4_REC_H 3
#define O4_X1_LB ((SS_X1_UB/2) - O4_REC_W + 1)
#define O4_X2_LB (SS_X2_LB + 1)
#define O4_X1_UB (O4_X1_LB + O4_REC_W)
#define O4_X2_UB (O4_X2_LB + O4_REC_H)


// Top Rectangle
#define O5_REC_W 1
#define O5_REC_H 3
#define O5_X1_LB ((SS_X1_UB/2) - O5_REC_W + 1)
#define O5_X2_LB (SS_X2_UB - O5_REC_H - 1)
#define O5_X1_UB (O5_X1_LB + O5_REC_W)
#define O5_X2_UB (O5_X2_LB + O5_REC_H)

/* construct state space */
scots::SymbolicSet robotCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[sDIM] = {SS_X1_LB,SS_X2_LB,SS_X3_LB,SS_X4_LB};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM] = {SS_X1_UB,SS_X2_UB,SS_X3_UB,SS_X4_UB}; 
  /* grid node distance diameter */
  double mu[sDIM] = {SS_X1_MU,SS_X2_MU,SS_X3_MU,SS_X4_MU};   


  /* eta is added to the bound so as to ensure that the whole
   * [0,10]x[0,10]x[-pi-eta,pi+eta] is covered by the cells */

  scots::SymbolicSet ss(mgr,sDIM,lb,ub,mu);

  /* add the grid points to the SymbolicSet ss */
  ss.addGridPoints();

    
  double HC[8*sDIM]={-1, 0,  0,  0,
                      1, 0,  0,  0,
                      0,-1,  0,  0,
                      0, 1,  0,  0,
                      0, 0, -1,  0,
                      0, 0,  1,  0,
                      0, 0,  0, -1,
                      0, 0,  0,  1};

  /* Constrain: two robots should not exist at same position !*/
  double nu1=SS_X1_MU*0.1;
  double nu2=SS_X2_MU*0.1;
  for(double i=SS_X1_LB; i<=SS_X1_UB; i+=SS_X1_MU){
    for(double j=SS_X2_LB; j<=SS_X2_UB; j+=SS_X2_MU){
       double h[8] = {-((i)-nu1), (i)+nu1,-((j)-nu2), (j)+nu2,
		      -((i)-nu1), (i)+nu1,-((j)-nu2), (j)+nu2};

       ss.remPolytope(8,HC,h, scots::OUTER);
    }
  }

  /* obstacles in the state space */
  scots::SymbolicSet ss_obst = ss;
  // Center Box:
  double h11[8] = {-O1_X1_LB, O1_X1_UB, -O1_X2_LB, O1_X2_UB,
		   -SS_X3_LB, SS_X3_UB, -SS_X4_LB, SS_X4_UB};
  double h12[8] = {-SS_X1_LB, SS_X1_UB, -SS_X2_LB, SS_X2_UB,
		   -O1_X1_LB, O1_X1_UB, -O1_X2_LB, O1_X2_UB};
  ss.remPolytope(8,HC,h11, scots::OUTER);
  ss.remPolytope(8,HC,h12, scots::OUTER);
  ss_obst.addPolytope(8,HC,h11, scots::OUTER);


  // Right Rectangle:
  double h21[8] = {-O2_X1_LB, O2_X1_UB, -O2_X2_LB, O2_X2_UB,
		   -SS_X3_LB, SS_X3_UB, -SS_X4_LB, SS_X4_UB};
  double h22[8] = {-SS_X1_LB, SS_X1_UB, -SS_X2_LB, SS_X2_UB,
		   -O2_X1_LB, O2_X1_UB, -O2_X2_LB, O2_X2_UB};
  ss.remPolytope(8,HC,h21, scots::OUTER);
  ss.remPolytope(8,HC,h22, scots::OUTER);
  ss_obst.addPolytope(8,HC,h21, scots::OUTER);


  // Left Rectangle:
  double h31[8] = {-O3_X1_LB, O3_X1_UB, -O3_X2_LB, O3_X2_UB,
		   -SS_X3_LB, SS_X3_UB, -SS_X4_LB, SS_X4_UB};
  double h32[8] = {-SS_X1_LB, SS_X1_UB, -SS_X2_LB, SS_X2_UB,
		   -O3_X1_LB, O3_X1_UB, -O3_X2_LB, O3_X2_UB};
  ss.remPolytope(8,HC,h31, scots::OUTER);
  ss.remPolytope(8,HC,h32, scots::OUTER);
  ss_obst.addPolytope(8,HC,h31, scots::OUTER);


  // Bottom Rectangle:
  double h41[8] = {-O4_X1_LB, O4_X1_UB, -O4_X2_LB, O4_X2_UB,
		   -SS_X3_LB, SS_X3_UB, -SS_X4_LB, SS_X4_UB};
  double h42[8] = {-SS_X1_LB, SS_X1_UB, -SS_X2_LB, SS_X2_UB,
		   -O4_X1_LB, O4_X1_UB, -O4_X2_LB, O4_X2_UB};
  ss.remPolytope(8,HC,h41, scots::OUTER);
  ss.remPolytope(8,HC,h42, scots::OUTER);
  ss_obst.addPolytope(8,HC,h41, scots::OUTER);


  // Top Rectangle:
  double h51[8] = {-O5_X1_LB, O5_X1_UB, -O5_X2_LB, O5_X2_UB,
		   -SS_X3_LB, SS_X3_UB, -SS_X4_LB, SS_X4_UB};
  double h52[8] = {-SS_X1_LB, SS_X1_UB, -SS_X2_LB, SS_X2_UB,
		   -O5_X1_LB, O5_X1_UB, -O5_X2_LB, O5_X2_UB};
  ss.remPolytope(8,HC,h51, scots::OUTER);
  ss.remPolytope(8,HC,h52, scots::OUTER);
  ss_obst.addPolytope(8,HC,h51, scots::OUTER);


  ss_obst.writeToFile("tworobots_obst_matlab.bdd");
  return ss;
}

/* construct input space */
scots::SymbolicSet robotCreateInputSpace(Cudd &mgr) {

  /* lower bounds of the hyper rectangle */
  double lb[sDIM] = {IS_U1_LB,IS_U2_LB,IS_U3_LB,IS_U4_LB};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM] = {IS_U1_UB,IS_U2_UB,IS_U3_UB,IS_U4_UB}; 
  /* grid node distance diameter */
  double mu[sDIM] = {IS_U1_MU,IS_U2_MU,IS_U3_MU,IS_U4_MU};   

  scots::SymbolicSet is(mgr,iDIM,lb,ub,mu);
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
  ss.writeToFile("tworobots_ss.bdd");

  /* write SymbolicSet of obstacles to robot_obst.bdd */
  ss.complement();
  ss.writeToFile("tworobots_obst.bdd");
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
  scots::SymbolicSet ts3 = ss;
  scots::SymbolicSet ts4 = ss;

  scots::SymbolicSet ts13 = ss;
  scots::SymbolicSet ts14 = ss;
  scots::SymbolicSet ts23 = ss;
  scots::SymbolicSet ts24 = ss;

  /* define the target set as a symbolic set */
  double HC[8*sDIM]={-1, 0,  0,  0,
                      1, 0,  0,  0,
                      0,-1,  0,  0,
                      0, 1,  0,  0,
                      0, 0, -1,  0,
                      0, 0,  1,  0,
                      0, 0,  0, -1,
                      0, 0,  0,  1};

  /* compute inner approximation of P={ x | H x<= h1 }  */
  double h1[8] = {-T1_X1_LB, T1_X1_UB, -T1_X2_LB, T1_X2_UB,
		  -SS_X3_LB, SS_X3_UB, -SS_X4_LB, SS_X4_UB};
  ts1.addPolytope(8,HC,h1, scots::OUTER);
  ts.addPolytope(8,HC,h1, scots::OUTER);

  double h2[8] = {-T2_X1_LB, T2_X1_UB, -T2_X2_LB, T2_X2_UB,
		  -SS_X3_LB, SS_X3_UB, -SS_X4_LB, SS_X4_UB};
  ts2.addPolytope(8,HC,h2, scots::OUTER);
  ts.addPolytope(8,HC,h2, scots::OUTER);

  double h3[8] = {-SS_X1_LB, SS_X1_UB, -SS_X2_LB, SS_X2_UB,
		  -T3_X3_LB, T3_X3_UB, -T3_X4_LB, T3_X4_UB};
  ts3.addPolytope(8,HC,h3, scots::OUTER);
  ts.addPolytope(8,HC,h3, scots::OUTER);

  std::cout << "h= " << -1*h3[0] << " , " << h3[1] << " , " << -1*h3[2] << " , " << h3[3] << " , ";
  std::cout <<          -1*h3[4] << " , " << h3[5] << " , " << -1*h3[6] << " , " << h3[7] << std::endl;

  double h4[8] = {-SS_X1_LB, SS_X1_UB, -SS_X2_LB, SS_X2_UB,
		  -T4_X3_LB, T4_X3_UB, -T4_X4_LB, T4_X4_UB};
  ts4.addPolytope(8,HC,h4, scots::OUTER);
  ts.addPolytope(8,HC,h4, scots::OUTER);

  ts1.writeToFile("tworobots_ts1.bdd");
  ts2.writeToFile("tworobots_ts2.bdd");
  ts3.writeToFile("tworobots_ts3.bdd");
  ts4.writeToFile("tworobots_ts4.bdd");
  ts.writeToFile("tworobots_ts.bdd");

  ts13.setSymbolicSet(ts1.getSymbolicSet() & ts3.getSymbolicSet());
  ts14.setSymbolicSet(ts1.getSymbolicSet() & ts4.getSymbolicSet());
  ts23.setSymbolicSet(ts2.getSymbolicSet() & ts3.getSymbolicSet());
  ts24.setSymbolicSet(ts2.getSymbolicSet() & ts4.getSymbolicSet());

  ts13.writeToFile("tworobots_ts13.bdd");
  ts14.writeToFile("tworobots_ts14.bdd");
  ts23.writeToFile("tworobots_ts23.bdd");
  ts24.writeToFile("tworobots_ts24.bdd");

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

  abstraction.getTransitionRelation().writeToFile("tworobots_rel.bdd");

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
  BDD T3 = ts3.getSymbolicSet();
  BDD T4 = ts4.getSymbolicSet();

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
  BDD Y3=mgr.bddOne();
  BDD YY3=mgr.bddZero();
  BDD Y4=mgr.bddOne();
  BDD YY4=mgr.bddZero();

  /* the controller */
  BDD C1=mgr.bddZero();  // assigned to T1, T3
  BDD C2=mgr.bddZero();	 // assigned to T1, T4
  BDD C3=mgr.bddZero();	 // assigned to T2, T3
  BDD C4=mgr.bddZero();	 // assigned to T2, T4
  BDD U=is.getCube();

  for(i=1; XX != X; i++) {
    X=XX;
    BDD preX=fp.pre(X);

    /* init inner fp */
    YY1 = mgr.bddZero();
    for(j=1; YY1 != Y1; j++) {
      Y1=YY1;
      YY1= (( preX & (T1&T3) )) | fp.pre(Y1);
      BDD N1 = YY1 & (!(C1.ExistAbstract(U)));
      C1 = C1 | N1;
    }
    std::cout << "Iterations inner1: " << j << std::endl;

    /* init inner fp */
    YY2 = mgr.bddZero();
    for(j=1; YY2 != Y2; j++) {
      Y2=YY2;
      YY2= (( preX & (T1&T4) )) | fp.pre(Y2);
      BDD N2 = YY2 & (!(C2.ExistAbstract(U)));
      C2 = C2 | N2;
    }
    std::cout << "Iterations inner2: " << j << std::endl;

    /* init inner fp */
    YY3 = mgr.bddZero();
    for(j=1; YY3 != Y3; j++) {
      Y3=YY3;
      YY3= (( preX & (T2&T3) )) | fp.pre(Y3);
      BDD N3 = YY3 & (!(C3.ExistAbstract(U)));
      C3 = C3 | N3;
    }
    std::cout << "Iterations inner3: " << j << std::endl;

    /* init inner fp */
    YY4 = mgr.bddZero();
    for(j=1; YY4 != Y4; j++) {
      Y4=YY4;
      YY4= (( preX & (T2&T4) )) | fp.pre(Y4);
      BDD N4 = YY4 & (!(C4.ExistAbstract(U)));
      C4 = C4 | N4;
    }
    std::cout << "Iterations inner4: " << j << std::endl;

    XX=YY1 & YY2 & YY3 & YY4;
  }
  std::cout << "Iterations outer: " << i << std::endl;


  tt.toc();


  scots::SymbolicSet cont1(ss,is);
  cont1.setSymbolicSet(C1);
  cont1.writeToFile("tworobots_cont1.bdd");

  scots::SymbolicSet cont2(ss,is);
  cont2.setSymbolicSet(C2);
  cont2.writeToFile("tworobots_cont2.bdd");

  scots::SymbolicSet cont3(ss,is);
  cont3.setSymbolicSet(C3);
  cont3.writeToFile("tworobots_cont3.bdd");

  scots::SymbolicSet cont4(ss,is);
  cont4.setSymbolicSet(C4);
  cont4.writeToFile("tworobots_cont4.bdd");

  return 1;
}


