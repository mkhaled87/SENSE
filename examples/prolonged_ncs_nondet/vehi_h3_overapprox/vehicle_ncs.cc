/*
 * vehicle.cc
 *
 *  created on: 29.11.2016
 *      author: M. Khaled
 */

#include <array>
#include <iostream>

#define VERBOSEBASIC

#include "cuddObj.hh"
#include "SENSE.hh"

/* state-space / input-space dimensions */
#define ssDIM 3
#define isDIM 2

/* NCS Delay bounds */
#define NSCMAX 2
#define NCAMAX 2

#define FILE_BDD_REL "scots-files/vehicle_rel.bdd"
#define FILE_BDD_TS "scots-files/vehicle_ts.bdd"

#define FILE_NBDD_REL "vehicle_ncs_rel.nbdd"
#define FILE_NBDD_CONTR "vehicle_ncs_controller.nbdd"

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
