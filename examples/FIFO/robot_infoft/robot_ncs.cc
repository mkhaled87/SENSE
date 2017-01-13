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
#define FILE_BDD_TS1 "scots-files/robot_ts1.bdd"
#define FILE_BDD_TS2 "scots-files/robot_ts2.bdd"

#define FILE_NBDD_REL "robot_rel.nbdd"
#define FILE_NBDD_CONTR1 "robot_contr1.nbdd"
#define FILE_NBDD_CONTR2 "robot_contr2.nbdd"

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

  
  cout << "Computing two reach controllers ... " << endl;
  vector<const char*> bdd_target_files;
  bdd_target_files.push_back(FILE_BDD_TS1);
  bdd_target_files.push_back(FILE_BDD_TS2);
  vector<ncsController> ncsContr = ncsRobot.ComputeRecurrenceConjController(bdd_target_files);

  cout << "Controllers computed in " << ncsContr[0].getControllerComputeTile() << " seconds !" << endl;
  ncsContr[0].WriteToFile(FILE_NBDD_CONTR1);
  ncsContr[1].WriteToFile(FILE_NBDD_CONTR2);
  cout << "Synthesized controller-1 is saved to the file: " << FILE_NBDD_CONTR1 << endl;
  cout << "Synthesized controller-2 is saved to the file: " << FILE_NBDD_CONTR2 << endl;

  return 1;
}
