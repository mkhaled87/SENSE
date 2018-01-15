/*
 * robot_ncs.cc
 *
 *  created on: 31.08.2017
 *      author: m.khaled
 */

#include <array>
#include <iostream>
#include "../../../interface/common/fifosim.hh"

/* state-space / input-space dimensions and sampling period*/
#define ssDIM 2
#define isDIM 2

/* NCS Delay bounds */
#define NSCMAX 2
#define NCAMAX 2

/* files needed for simulation */
#define FILE_BDD_TS1 "scots-files/robot_ts1.bdd"
#define FILE_BDD_TS2 "scots-files/robot_ts2.bdd"
#define FILE_NBDD_REL "robot_rel.nbdd"
#define FILE_NBDD_CONTR1 "robot_contr1.nbdd"
#define FILE_NBDD_CONTR2 "robot_contr2.nbdd"


const double tau = 1.0;

/* robot simulation class */
class RobotPlant : public Plant{
public:
    RobotPlant(){}
    ~RobotPlant(){}

    void
    initOde(double tau){
        odeTau = tau;
        base_initialize(ssDIM, isDIM);
    }

    void
    oderhs(vector<double>& xnew, vector<double> x, vector<double> u){
	x[0]=0;
        xnew[0] = u[0];
        xnew[1] = u[1];
    }
};


#define SIM_LOOPS 100
int main() {
  vector<double> u0(isDIM);
  vector<double> x0(ssDIM);
  x0[0]=10;
  x0[1]=10;
  u0[0]=1;
  u0[1]=1;

  Cudd cuddManager;
  RobotPlant robot;
  robot.initOde(tau);
  ncsFIFOEstimator T(NSCMAX, NCAMAX, u0 ,tau , &robot);
  FIFOChannel channel_SC(NSCMAX);
  FIFOChannel channel_CA(NCAMAX, u0);

  vector<string> conts = {FILE_NBDD_CONTR1, FILE_NBDD_CONTR2};
  vector<string> targets={FILE_BDD_TS1, FILE_BDD_TS2};
  ncsFIFOController controller(cuddManager, conts, targets, NSCMAX);

  vector<double> x = x0;
  vector<double> xc;
  vector<double> u;
  vector<double> uc=u0;

  // The simulation loop
  for(size_t i=0; i<SIM_LOOPS; i++){

	  // The plant
	  u = channel_CA.get();
	  x = robot.Compute(x, u);

	  // The SC channel
	  channel_SC.put(x);
	  xc = channel_SC.get();

	  // The controller
	  if(xc.size() > 0){
		  T.Refresh(xc, uc);
		  uc = controller.getInputs(T)[0];
	  }else{
		  uc = u0;
	  }

	  cout << "LOOP #" << setw(3) << i+1 << "\t|\t";
	  cout << "X:" << setw(2) <<  (int)x[0] << "," << setw(2) <<  (int)x[1] << "\t\t";
	  cout << "U:" << setw(2) << (int)uc[0] << "," << setw(2) << (int)uc[1] << endl;

	  // The SC channel
	  channel_CA.put(uc);
  }

  cout << "Simulation Completed !!" << endl;

  return 1;
}
