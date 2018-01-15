/*
 * contCoverage.cc
 *
 *  created on: 01.11.2017
 *      author: m.khaled
 *
 *********************************************************
 * This tool shows on terminal the coverage of 2-D symbolic controllers.
 */

#include <array>
#include <iostream>

#include "cuddObj.hh"
#include "ncsACT.hh"

#define NBDD_FILE_CONTR "../../examples/prolonged_ncs/vehicle_h3/vehicle_contr.nbdd"

int main() {
  Cudd cuddManager;
  ncsController contr(cuddManager, NBDD_FILE_CONTR);

  vector<double> ssLb  = contr.getSourceState()->getSsLb();
  vector<double> ssUb  = contr.getSourceState()->getSsUb();
  vector<double> ssEta = contr.getSourceState()->getSsEta();
  vector<size_t> ssNNBddVars = contr.getSourceState()->getSsVarsCount();

  vector<vector<size_t>> x1xn_vars = contr.getSourceState()->getX1xnBddVars();

  vector<BDD> x1x2_bdds;
  for(size_t k=0; k<ssNNBddVars[0]+ssNNBddVars[1];k++)
	  x1x2_bdds.push_back(cuddManager.bddVar(x1xn_vars[0][k]));


  BDD contrBdd = contr.getBDD();

  double x1, x2;

  for(x2= ssUb[1]; x2>=ssLb[1]; x2-=ssEta[1]){
	  for(x1= ssLb[0]; x1<=ssUb[0]; x1+=ssEta[0]){
		  //cout << x1 << "," << x2 << "," << x3 << " : ";

		  vector<int> x1_bin = Quantizer::encode(x1, ssEta[0], ssLb[0], ssUb[0], ssNNBddVars[0]);
		  vector<int> x2_bin = Quantizer::encode(x2, ssEta[1], ssLb[1], ssUb[1], ssNNBddVars[1]);

		  vector<int> x1x2_bin;
		  VectorManager::AppendVector(x1x2_bin, x1_bin);
		  VectorManager::AppendVector(x1x2_bin, x2_bin);

		  BDD x1x2_cube = cuddManager.bddComputeCube(x1x2_bdds.data(), x1x2_bin.data(), x1x2_bdds.size());
		  //BDDUtils::PrintBDD("x1x2_cube",x1x2_cube);

		  BDD x1x2_in_C = contrBdd*x1x2_cube;
		  //BDDUtils::PrintBDD("x1x2_in_C",x1x2_in_C);

		  if(x1x2_in_C == cuddManager.bddOne() || x1x2_in_C == cuddManager.bddZero())
			  cout << ".";
		  else
			  cout << "o";

		  /*
		  VectorManager::PrintVector(x1_bin,'X',false);
		  VectorManager::PrintVector(x2_bin,'X',false);
		  VectorManager::PrintVector(x3_bin,'X',false);
		  cout << "," << endl;
		  */
	  }
	  cout << endl;
  }
  
  cout << "done !";

  return 1;
}

