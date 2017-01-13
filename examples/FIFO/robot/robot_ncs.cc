/*
 * robot_ncs.cc
 *
 *  created on: 31.08.2015
 *      author: m.khaled
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

#define FILE_NBDD_REL "robot_rel.nbdd"
#define FILE_NBDD_CONTR "robot_contr.nbdd"

int main() {
  Cudd cuddManager;

  cout << "Initiating the NCS Transition relation from the original relation ... " << endl;
  ncsFIFOTransitionRelation ncsState13(cuddManager, FILE_BDD_REL, ssDIM, isDIM, NSCMAX, NCAMAX);
  cout << "NCS relation intialized !" << endl;

  cuddManager.AutodynEnable();
  cout << "Expanding transition relation ... " << endl;
  ncsState13.ExpandBDD();
  cout << "NCS relation expanded in " << ncsState13.getExpandTime() << " seconds !" << endl;
  //ncsState13.WriteToFile(FILE_NBDD_REL);
  //cout << "New expanded transition relation is saved to the file: " << FILE_NBDD_REL << endl;

  
  cout << "Computing a reach controller ... " << endl;
  ncsController ncsContr = ncsState13.ComputeReachController(FILE_BDD_TS);
  cout << "Controller computed in " << ncsContr.getControllerComputeTile() << " seconds !" << endl;
  ncsContr.WriteToFile(FILE_NBDD_CONTR);
  cout << "Synthesized controller is saved to the file: " << FILE_NBDD_CONTR << endl;

  return 1;
}
