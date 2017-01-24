/*
 * vehicle.cc
 *
 *  created on: 31.08.2015
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
#define FILE_NBDD_REL "vehicle_rel.nbdd"

int main() {
  Cudd cuddManager;

  cout << "Initiating the NCS Transition relation from the original relation ... " << endl;
  ncsFIFOTransitionRelation ncsState13(cuddManager, FILE_BDD_REL, ssDIM, isDIM, NSCMAX, NCAMAX);
  cout << "NCS relation intialized !" << endl;

  cuddManager.AutodynEnable();
  cout << "Expanding transition relation ... " << endl;
  ncsState13.ExpandBDD();
  cout << "NCS relation expanded in " << ncsState13.getExpandTime() << " seconds !" << endl;
  ncsState13.WriteToFile(FILE_NBDD_REL);
  cout << "New expanded transition relation is saved to the file: " << FILE_NBDD_REL << endl;

  return 1;
}