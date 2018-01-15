/*
 * dcdc.cc
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
#define ssDIM 4
#define isDIM 4

/* NCS Delay bounds */
#define NSCMAX 2
#define NCAMAX 2

#define FILE_BDD_REL "scots-files/tworobots_rel.bdd"
#define FILE_BDD_TS13 "scots-files/tworobots_ts13.bdd"
#define FILE_BDD_TS14 "scots-files/tworobots_ts14.bdd"
#define FILE_BDD_TS23 "scots-files/tworobots_ts24.bdd"
#define FILE_BDD_TS24 "scots-files/tworobots_ts24.bdd"

#define FILE_NBDD_REL "tworobots_rel.nbdd"
#define FILE_NBDD_CONTR1 "tworobots_contr1.nbdd"
#define FILE_NBDD_CONTR2 "tworobots_contr2.nbdd"
#define FILE_NBDD_CONTR3 "tworobots_contr3.nbdd"
#define FILE_NBDD_CONTR4 "tworobots_contr4.nbdd"

int main() {
  Cudd cuddManager;

  cout << "Initiating the NCS Transition relation from the original relation ... " << endl;
  ncsFIFOTransitionRelation ncsRobot(cuddManager, FILE_BDD_REL, ssDIM, isDIM, NSCMAX, NCAMAX);
  cout << "NCS relation intialized !" << endl;

  cuddManager.AutodynEnable();
  cout << "Expanding transition relation ... " << endl;
  ncsRobot.ExpandBDD();
  cout << "NCS relation expanded in " << ncsRobot.getExpandTime() << " seconds !" << endl;
  //ncsState13.WriteToFile(FILE_NBDD_REL);
  //cout << "New expanded transition relation is saved to the file: " << FILE_NBDD_REL << endl;
 
  cout << "Computing recurrence controllers ... " << endl;
  vector<const char*> bdd_target_files;
  bdd_target_files.push_back(FILE_BDD_TS13);
  bdd_target_files.push_back(FILE_BDD_TS14);
  bdd_target_files.push_back(FILE_BDD_TS23);
  bdd_target_files.push_back(FILE_BDD_TS24);
  vector<ncsController> ncsContr = ncsRobot.ComputeRecurrenceConjController(bdd_target_files);

  cout << "Controllers computed in " << ncsContr[0].getControllerComputeTile() << " seconds !" << endl;
  ncsContr[0].WriteToFile(FILE_NBDD_CONTR1);
  ncsContr[1].WriteToFile(FILE_NBDD_CONTR2);
  ncsContr[2].WriteToFile(FILE_NBDD_CONTR3);
  ncsContr[3].WriteToFile(FILE_NBDD_CONTR4);

  cout << "Synthesized controller-1 is saved to the file: " << FILE_NBDD_CONTR1 << endl;
  cout << "Synthesized controller-2 is saved to the file: " << FILE_NBDD_CONTR2 << endl;
  cout << "Synthesized controller-3 is saved to the file: " << FILE_NBDD_CONTR3 << endl;
  cout << "Synthesized controller-4 is saved to the file: " << FILE_NBDD_CONTR4 << endl;

  return 1;
}
