#ifndef BddDeterminizer_HH_
#define BddDeterminizer_HH_

#include "cuddObj.hh"
#include "BddReader.hh"
#include "CuddMintermIterator.hh"
#include "utils.hh"

enum DETERMINIZE_METHOD{
	FIRST,
	LAST,
	MIN_VAL,
	MAX_VAL,
	RANDOM,
	USER_FUNC
};

class BddDeterminizer{
	Cudd* cuddManager;
	size_t STATES_BDDVARSCOUNT;
	size_t ACTION_BDDVARSCOUNT;

	bool useLocalReader;
	BDDReader *reader;
	BDD passedBDD;

public:
	BddDeterminizer(Cudd& _mgr, const char* filename, BDD_FILE_TYPE type, size_t state_bddVar_count, size_t action_bddVar_count){

		STATES_BDDVARSCOUNT = state_bddVar_count;
		ACTION_BDDVARSCOUNT = action_bddVar_count;
		cuddManager = &_mgr;

		reader = new BDDReader(*cuddManager, filename, type);
		passedBDD = cuddManager->bddZero();
		useLocalReader = true;
	}

	BddDeterminizer(Cudd& _mgr, const BDD& givenBdd, size_t state_bddVar_count, size_t action_bddVar_count){

		STATES_BDDVARSCOUNT = state_bddVar_count;
		ACTION_BDDVARSCOUNT = action_bddVar_count;
		cuddManager = &_mgr;

		passedBDD = givenBdd;
		useLocalReader = false;
	}

	BDD determinize(DETERMINIZE_METHOD method, bool check=false, int verbosity=0){
		BDD mainBdd;
		BDD determinizedBdd = cuddManager->bddZero();
		
		if(useLocalReader)
			mainBdd = reader->ReadBdd();
		else
			mainBdd = passedBDD;

		if (mainBdd == cuddManager->bddZero()){
		      ostringstream os;
		      os << "Error: BddDeterminizer::determinize: invalid BDD ! BDD is zero function.";
		      throw invalid_argument(os.str().c_str());
		}

		if(method != DETERMINIZE_METHOD::RANDOM)
			throw "Error: BddDeterminizer::determinize: currently, only RANDOM determinization is implemented !";



		vector<size_t> projVarIdx;
		vector<BDD> projVarBdd;
		vector<BDD> AllBddVars;
		for(size_t i=0; i<STATES_BDDVARSCOUNT; i++){
			projVarIdx.push_back(i);
			projVarBdd.push_back(cuddManager->bddVar(i));
			AllBddVars.push_back(cuddManager->bddVar(i));
		}
		for(size_t i=STATES_BDDVARSCOUNT; i<STATES_BDDVARSCOUNT+ACTION_BDDVARSCOUNT; i++){
			AllBddVars.push_back(cuddManager->bddVar(i));
		}

		// project mainBdd to get the BDD of states only
		BDD bddStates= BDDUtils::ProjectBDD(*cuddManager, mainBdd, projVarIdx);

		if(verbosity>0)
			std::cout << "Determinizing the BDD: ";

		// iterate over all states
		int minterm[STATES_BDDVARSCOUNT];
		CuddMintermIterator  it(bddStates,projVarIdx,STATES_BDDVARSCOUNT);
		for(; !it.done(); ++it) {
			if(verbosity>1){
				std::cout << "minterm: ";
				it.printMinterm();
			}

			it.copyMinterm(minterm);
			BDD stateCube = cuddManager->bddComputeCube(projVarBdd.data(), minterm, STATES_BDDVARSCOUNT);

			// all outputs of the current state
			BDD bddStateOutputs = mainBdd&stateCube;

			// pick random one
			BDD selected = bddStateOutputs.PickOneMinterm(AllBddVars);
			if(verbosity>1){
				std::cout << "selcted: ";
				selected.PrintMinterm();
			}
			determinizedBdd |= selected;

			if(verbosity>0)
				it.printProgress();
		}
		if(verbosity>0)
			cout << std::endl;

		// check if it is included
		if(check){
			if((mainBdd&determinizedBdd) != determinizedBdd){
				ostringstream os;
				os << "Error: Error: BddDeterminizer::determinize: BDD was not correctly determinized.";
				throw invalid_argument(os.str().c_str());
			}
			else{
				std::cout << "BDD determinized and passed the check !" << std::endl;
			}
		}
		
		return determinizedBdd;
	}

	~BddDeterminizer(){
		if(useLocalReader)
			delete reader;
	}

};


#endif
