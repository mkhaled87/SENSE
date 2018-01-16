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
#define ssDIM 1
#define isDIM 1

/* NCS Delay bounds */
#define NSCMAX 2
#define NCAMAX 2

#define FILE_BDD_REL "scots-files/state13_rel.bdd"
#define FILE_BDD_TS "scots-files/state13_ts.bdd"
#define FILE_NBDD_REL "state13_ncs_rel.nbdd"
#define FILE_NBDD_CONTR "state13_ncs_controller.bdd"

int main() {
  Cudd cuddManager;

  cout << "Initiating the NCS Transition relation from the original relation ... " << endl;
  ncsFIFOTransitionRelation ncsState13(cuddManager, FILE_BDD_REL, ssDIM, isDIM, NSCMAX, NCAMAX);
  cout << "NCS relation intialized !" << endl;

  cuddManager.AutodynEnable();
  cout << "Expanding transition relation ... " << endl;
  ncsState13.ExpandBDD();
  cout << "NCS relation expanded in " << ncsState13.getExpandTime() << " seconds !" << endl;

  cout << "Computing a reach controller ... " << endl;
  ncsController ncsContr = ncsState13.ComputeReachController(FILE_BDD_TS);
  cout << "Controller computed in " << ncsContr.getControllerComputeTile() << " seconds !" << endl;
  ncsContr.WriteToSCOTS2File(FILE_NBDD_CONTR);
  cout << "Synthesized controller is saved to the file: " << FILE_NBDD_CONTR << endl;
  

  return 1;
}
