/*
 * robot_ncs.cc
 *
 *  created on: 31.08.2015
 *      author: m.khaled
 */

#include <array>
#include <iostream>
#include "../../../interface/common/fifosim.hh"

/* state-space / input-space dimensions and sampling period*/
#define ssDIM 4
#define isDIM 4

/* NCS Delay bounds */
#define NSCMAX 2
#define NCAMAX 2

/* files needed for simulation */
#define FILE_BDD_TS1 "scots-files/tworobots_ts1.bdd"
#define FILE_BDD_TS2 "scots-files/tworobots_ts2.bdd"
#define FILE_BDD_TS3 "scots-files/tworobots_ts3.bdd"
#define FILE_BDD_TS4 "scots-files/tworobots_ts4.bdd"


#define FILE_NBDD_REL "robot_rel.nbdd"
#define FILE_NBDD_CONTR1 "tworobots_contr1.nbdd"
#define FILE_NBDD_CONTR2 "tworobots_contr2.nbdd"
#define FILE_NBDD_CONTR3 "tworobots_contr3.nbdd"
#define FILE_NBDD_CONTR4 "tworobots_contr4.nbdd"


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
        xnew[2] = u[2];
        xnew[3] = u[3];
    }
};

int direction_to_mode(int robot1_dir, int robot2_dir){
	int m;
       if(robot1_dir == 1 && robot2_dir == 3)
      m=1;
   else if(robot1_dir == 1 && robot2_dir == 4)
      m=2;
   else if(robot1_dir == 2 && robot2_dir == 3)
      m=3;
   else if(robot1_dir == 2 && robot2_dir == 4)
      m=4;
   else
      m=0;

   return m;
}

#define SIM_LOOPS 100
int main() {
  vector<double> u0(isDIM);
  vector<double> x0(ssDIM);
  x0[0]=7;
  x0[1]=15;
  x0[2]=15;
  x0[3]=7;

  u0[0]=1;
  u0[1]=0;
  u0[2]=0;
  u0[3]=1;

  Cudd cuddManager;
  RobotPlant robot;
  robot.initOde(tau);
  ncsFIFOEstimator T(NSCMAX, NCAMAX, u0 ,tau , &robot);
  FIFOChannel channel_SC(NSCMAX);
  FIFOChannel channel_CA(NCAMAX, u0);

  vector<string> conts = {FILE_NBDD_CONTR1, FILE_NBDD_CONTR2, FILE_NBDD_CONTR3, FILE_NBDD_CONTR4};
  vector<string> targets={FILE_BDD_TS1, FILE_BDD_TS2, FILE_BDD_TS3, FILE_BDD_TS4};

  cout << "Loading the controllers ... " << endl;
  ncsFIFOController controller(cuddManager, conts, targets, NSCMAX);


  vector<double> x = x0;
  vector<double> xc;
  vector<double> u;
  vector<double> uc=u0;
  
  int robot1_to = 1;
  int robot2_to = 3;
  // The simulation loop
  cout << "Simulating ... " << endl;
  for(size_t i=0; i<SIM_LOOPS; i++){

	  // The plant
	  u = channel_CA.get();
	  x = robot.Compute(x, u);

	  // The SC channel
	  channel_SC.put(x);
	  xc = channel_SC.get();

            if(x[0] >= 2 && x[0] <= 4 && x[1] >= 2 && x[1] <= 4)
                robot1_to = 2;
            else if(x[0] >= 12 && x[0] <= 14 && x[1] >= 12 && x[1] <= 14)
                robot1_to = 1;

            if(x[2] >= 2 && x[2] <= 4 && x[3] >= 12 && x[3] <= 14)
                robot2_to = 4;
            else if(x[2] >= 12 && x[2] <= 14 && x[3] >= 2 && x[3] <= 4)
                robot2_to = 3;

	  // The controller
	  if(xc.size() > 0){
		  T.Refresh(xc, uc);
		  uc = controller.getInputs(T, direction_to_mode(robot1_to, robot2_to))[0];
	  }else{
		  uc = u0;
	  }

	  cout << "LOOP #" << setw(3) << i+1 << "\t|\t";
	  cout << "X:" << setw(2) <<  (int)x[0] << "," << setw(2) <<  (int)x[1] << "," << setw(2) <<  (int)x[2] << "," << setw(2) <<  (int)x[3] << "\t\t";
	  cout << "U:" << setw(2) << (int)uc[0] << "," << setw(2) << (int)uc[1] << "," << setw(2) << (int)uc[2] << "," << setw(2) << (int)uc[3] << endl;

	  // The SC channel
	  channel_CA.put(uc);
  }

  cout << "Simulation Completed !!" << endl;

  return 1;
}
