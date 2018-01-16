/*
 * vehicle.cc
 *
 *  created on: 15.01.2018
 *      author: M. Khaled
 */

#include <array>
#include <iostream>

#define VERBOSEBASIC

#include "cuddObj.hh"
#include "SENSE.hh"

/* state-space / input-space dimensions */
#define ssDIM 2
#define isDIM 2

/* NCS Delay bounds */
#define NSCMAX 2
#define NCAMAX 2

#define FILE_BDD_REL "scots-files/robot_rel.bdd"
#define FILE_BDD_TS "scots-files/robot_ts.bdd"
#define FILE_NBDD_REL "robot_ncs_rel.nbdd"
#define FILE_NBDD_CONTR "robot_ncs_controller.bdd"

int main() {
  Cudd cuddManager;

  cout << "Initiating the NCS Transition relation from the original relation ... " << endl;
  ncsFIFOTransitionRelation ncsRobot(cuddManager, FILE_BDD_REL, ssDIM, isDIM, NSCMAX, NCAMAX);
  cout << "NCS relation intialized !" << endl;

  cuddManager.AutodynEnable();
  cout << "Expanding transition relation ... " << endl;
  ncsRobot.ExpandBDD();
  cout << "NCS relation expanded in " << ncsRobot.getExpandTime() << " seconds !" << endl;

  cout << "Computing a reach controller ... " << endl;
  ncsController ncsContr = ncsRobot.ComputeReachController(FILE_BDD_TS);
  cout << "Controller computed in " << ncsContr.getControllerComputeTile() << " seconds !" << endl;
  ncsContr.WriteToSCOTS2File(FILE_NBDD_CONTR);
  cout << "Synthesized controller is saved to the file: " << FILE_NBDD_CONTR << endl;
  

  return 1;
}
