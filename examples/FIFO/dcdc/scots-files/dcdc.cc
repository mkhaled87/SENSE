/*
 * dcdc.cc
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

/* state space dim */
#define sDIM 2
#define iDIM 1

/* data types for the ode solver */
typedef std::array<double,sDIM> state_type;
typedef std::array<double,sDIM> input_type;

/* sampling time */
const double tau = 0.5;
/* number of intermediate steps in the ode solver */
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);

/* we integrate the dcdc ode by 0.3 sec (the result is stored in x)  */
auto  dcdc_post = [](state_type &x, input_type &u) -> void {

  /* the ode describing the system */
  auto  system_ode = [](state_type &dxdt, const state_type &x, const input_type &u) -> void {

    const double r0=1.0 ; 
    const double vs = 1.0 ;
    const double rl = 0.05 ;
    const double rc = rl / 10 ;
    const double xl = 3.0 ;
    const double xc = 70.0 ;

    const double b[2]={vs/xl, 0};

    double a[2][2];
    if(u[0]==1) {
      a[0][0] = -rl / xl;
      a[0][1] = 0;
      a[1][0] = 0;
      a[1][1] = (-1 / xc) * (1 / (r0 + rc));
    } else {
      a[0][0] = (-1 / xl) * (rl + ((r0 * rc) / (r0 + rc))) ;
      a[0][1] =  ((-1 / xl) * (r0 / (r0 + rc))) / 5 ;
      a[1][0] = 5 * (r0 / (r0 + rc)) * (1 / xc);
      a[1][1] =(-1 / xc) * (1 / (r0 + rc)) ;
    }
    dxdt[0] = a[0][0]*x[0]+a[0][1]*x[1] + b[0];
    dxdt[1] = a[1][0]*x[0]+a[1][1]*x[1] + b[1];
  };
  ode_solver(system_ode,x,u);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_type &r, input_type &u) -> void {

  /* the ode to determine the radius of the cell which over-approximates the
   * attainable set see: http://arxiv.org/abs/1503.03715v1 */
  auto growth_bound_ode = [](state_type &drdt,  const state_type &r, const input_type &u) {
    /* for the dcdc boost converter the growth bound is simply given by the metzler matrix of the system matrices */ 
    const double r0=1.0 ; 
    const double rl = 0.05 ;
    const double rc = rl / 10 ;
    const double xl = 3.0 ;
    const double xc = 70.0 ;

    double a[2][2];
    if(u[0]==1) {
      a[0][0] = -rl / xl;
      a[0][1] = 0;
      a[1][0] = 0;
      a[1][1] = (-1 / xc) * (1 / (r0 + rc));
    } else {
      a[0][0] = (-1 / xl) * (rl + ((r0 * rc) / (r0 + rc))) ;
      a[0][1] =  ((1 / xl) * (r0 / (r0 + rc))) / 5 ;
      a[1][0] = 5 * (r0 / (r0 + rc)) * (1 / xc);
      a[1][1] =(-1 / xc) * (1 / (r0 + rc)) ;
    }
    drdt[0] = a[0][0]*r[0]+a[0][1]*r[1];
    drdt[1] = a[1][0]*r[0]+a[1][1]*r[1];
  };
  ode_solver(growth_bound_ode,r,u);
};





int main() {
  /* to measure time */
  TicToc tt;
  /* there is one unique manager to organize the bdd variables */
  Cudd mgr;

  /****************************************************************************/
  /* construct SymbolicSet for the state space */
  /****************************************************************************/
  std::cout << "Constructing state-space ... " << std::endl;
  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={1.15,5.45};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={1.55,5.85}; 
  /* grid node distance diameter */
  double eta[sDIM]={2/4e3,2/4e3};   
  scots::SymbolicSet ss(mgr,sDIM,lb,ub,eta);
  ss.addGridPoints();
  ss.writeToFile("sys_ss.bdd");

  /****************************************************************************/
  /* construct SymbolicSet for the input space */
  /****************************************************************************/
  std::cout << "Constructing input-space ... " << std::endl;
  double ilb[iDIM]={1};  
  double iub[iDIM]={2}; 
  double ieta[iDIM]={1};   
  scots::SymbolicSet is(mgr,iDIM,ilb,iub,ieta);
  is.addGridPoints();
  is.writeToFile("sys_is.bdd");

  /****************************************************************************/
  /* setup class for symbolic model computation */
  /****************************************************************************/
  /* first create SymbolicSet of post variables 
   * by copying the SymbolicSet of the state space and assigning new BDD IDs */
  scots::SymbolicSet sss(ss,1);
  /* instantiate the SymbolicModel */
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction(&ss, &is, &sss);
  /* compute the transition relation */
  tt.tic();
  abstraction.computeTransitionRelation(dcdc_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  /* get the number of elements in the transition relation */
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction.getSize() << std::endl;
  abstraction.getTransitionRelation().writeToFile("sys_rel.bdd");
  return 1;
}
