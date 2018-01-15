/*
 * testAllSameU.cc
 *
 *  created on: 31.08.2015
 *      author: m.khaled
 */

#include <array>
#include <vector>
#include <iostream>

#include "cuddObj.hh"
#include "SENSE.hh"
#include "psmController.hh"

#define FILE_PLANT_BDD  "../../examples/FIFO/vehicle_half3/scots-files/vehicle_rel.bdd"
#define FILE_CONTR_NBDD "../../examples/FIFO/vehicle_half3/vehicle_contr.nbdd"

#define ssDIM 3
#define isDIM 2
#define NSC 2
#define NCA 2

#define X0 0.5,0.5,0.6
#define U0 0.6,0.0

using namespace std;


Cudd cuddManager;
int main() {

  // the concrete state at the controller
  vector<double> x0{X0};

  // the inputs in the CA channel
  vector<vector<double>> uCA{{U0},{U0}};

  psmController C(cuddManager, FILE_PLANT_BDD, FILE_CONTR_NBDD, ssDIM, isDIM, NSC, NCA);
  vector<vector<double>> us_suggested = C.suggestControlInputs(x0, uCA);

  cout << us_suggested.size() << " inputs were suggested: ";
  for(size_t i=0; i<us_suggested.size(); i++){
	  cout << "\t(";
	  for(size_t j=0; j<us_suggested[i].size(); j++){
		  cout << us_suggested[i][j] << " ";
	  }
	  cout << ")";
  }


  return 1;
}

