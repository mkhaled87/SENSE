/*
 * sysExplorer.cc
 *
 *  created on: 31.08.2017
 *      author: m.khaled
 *
 *********************************************************
 * This tool is an interactive terminal tool to navigate through
 * a transition relation or a controller.
 */

#include <array>
#include <iostream>

#include "cuddObj.hh"
#include "SENSE.hh"

#define NBDD_FILE_REL 	"../../examples/prolonged_ncs/vehicle2_h3/vehicle_rel.nbdd"
#define NBDD_FILE_CONTR "../../examples/prolonged_ncs/vehicle2_h3/vehicle_contr.nbdd"

int main() {
  Cudd cuddManager;

  cout << "Loading the relation/controller files ... " << endl;
  ncsFIFOTransitionRelation ncsRel(cuddManager, NBDD_FILE_REL);
  ncsController ncsContr(cuddManager, NBDD_FILE_CONTR);
  cout << "Loading done !!" << endl;

  cuddManager.AutodynEnable();

  BDD bddRel = ncsRel.getTransitionRelation();
  BDD bddContr = ncsContr.getBDD();




  cout << "The relation was designed for delays(NcsMax, NcaMax): " << ncsRel.getNscMax() << "," << ncsRel.getNscMax() << endl;
  cout << "Number of Bdd vars for the state per dim: "; VectorManager::PrintVector(ncsRel.getSsVarsCount());
  cout << "Number of Bdd vars for the input per dim: "; VectorManager::PrintVector(ncsRel.getIsVarsCount());
  cout << "Your source states should be in this form: ";
  cout << ncsRel.getSourceStateTemplate()->getStateTemplateText() << endl;


  vector<size_t> stateVars = ncsRel.getSourceStateTemplate()->getQXUVarsOrganized();

  while(true){
	  cout << endl;
	  cout << "use (0,1,-) in the template and provide the requested state (CTRL+C to exit): ";
	  string strState;
	  cin >> strState;

	  if(strState.length() != stateVars.size()){
		  cout << "invalid state length! [Try Again]" << endl;
		  continue;
	  }

	  vector<size_t> selectedVars;
	  vector<BDD> stateBDDs;
	  vector<int> statePhases;

	  for(size_t i=0; i<stateVars.size(); i++){
		  switch(strState[i]){
		  	  case '0':
		  		  stateBDDs.push_back(cuddManager.bddVar(stateVars[i]));
		  		  statePhases.push_back(0);
		  		  selectedVars.push_back(stateVars[i]);
		  		  break;
		  	  case '1':
		  		  stateBDDs.push_back(cuddManager.bddVar(stateVars[i]));
		  		  statePhases.push_back(1);
		  		  selectedVars.push_back(stateVars[i]);
		  		  break;
		  	  default:
		  		  break;
		  }
	  }

	  BDD stateCube = cuddManager.bddComputeCube(stateBDDs.data(), statePhases.data(), stateBDDs.size());

	  cout << "selected vars  : "; VectorManager::PrintVector(selectedVars);
	  cout << "selected phases: "; VectorManager::PrintVector(statePhases);
	  BDDUtils::PrintBDD("stateCube", stateCube);

	  cout << "-------------------------------------------------------------------------------" << endl;
	  cout << "finiding the posts/inputs in the relation ... ";
	  BDD bddPosts = stateCube*bddRel;
	  //BDDUtils::PrintBDD("Relation's in/Posts", bddPosts);
	  cout << "found " << (int)bddPosts.CountMinterm(ncsRel.getVarsCount()) << " members" << endl;

	  cout << "finiding the inputs in the controller ... ";
	  BDD bddInputs = stateCube*bddContr;
	  //BDDUtils::PrintBDD("Controller's inputs", bddInputs);
	  cout << "found " << (int)bddInputs.CountMinterm(ncsContr.getVarsCount()) << " members"  << endl;
	  cout << "-------------------------------------------------------------------------------" << endl;
  }




  
  return 1;
}

