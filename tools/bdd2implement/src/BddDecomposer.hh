#ifndef BddDecomposer_HH_
#define BddDecomposer_HH_

#include "cuddObj.hh"
#include "BddReader.hh"
#include "utils.hh"

/*
 * a class to decompose a BDD to set of BDDs for some specific output bits
 */
class BddOutputDecomposer{
	Cudd *pCuddManager;
	BDD bddObject_;
	size_t ACTION_BDDVARSCOUNT;
	size_t STATES_BDDVARSCOUNT;
public:
	BddOutputDecomposer(Cudd& cuddManager, BDD& bddObject, size_t state_bddVar_count, size_t action_bddVar_count){
		pCuddManager = &cuddManager;
		bddObject_ = bddObject;
		STATES_BDDVARSCOUNT = state_bddVar_count;
		ACTION_BDDVARSCOUNT = action_bddVar_count;
	}

	vector<BDD> Decompose(int verbosity=0){

		vector<BDD> actionBdds(ACTION_BDDVARSCOUNT);
		vector<size_t> projVarIdx;

		for(size_t i=0; i<STATES_BDDVARSCOUNT+1; i++)
			projVarIdx.push_back(i);

		vector<size_t> projVarIdxReduced = projVarIdx;
		projVarIdxReduced.erase(projVarIdxReduced.end()-1);

		for(size_t i=0; i<ACTION_BDDVARSCOUNT; i++){
			//vector<int> map = BDDUtils::GetReadyPermuteMap(*pCuddManager);
			//map[STATES_BDDVARSCOUNT] = STATES_BDDVARSCOUNT+i;
			//BDD tmp = bddObject_.Permute(map.data());
			//actionBdds[i] = BDDUtils::ProjectBDD(*pCuddManager, tmp, projVarIdx);

			if(verbosity >1){
				std::cout << "-----------------------------------------------" << std::endl;
				std::cout << "sub-bdd #" << i << " :" <<std::endl;
				std::cout << "-----------------------------------------------" << std::endl;
				//BDDUtils::PrintBDD("before selector", actionBdds[i]);
			}

			actionBdds[i] = bddObject_;
			BDD selector = pCuddManager->bddVar(STATES_BDDVARSCOUNT+i);
			actionBdds[i]&=selector;

			if(verbosity >1)
				BDDUtils::PrintBDD("after selector", actionBdds[i],STATES_BDDVARSCOUNT);

			BDD bddBeforeProjection = actionBdds[i];
			actionBdds[i] = BDDUtils::ProjectBDD(*pCuddManager, actionBdds[i], projVarIdxReduced);

			if(verbosity >1){
				BDDUtils::PrintBDD("after projection", actionBdds[i], STATES_BDDVARSCOUNT);
				std::cout << "-----------------------------------------------" << std::endl;
			}

			if(actionBdds[i]*bddBeforeProjection != bddBeforeProjection)
			    throw invalid_argument("Error: BddToVhdlBooleanFunction::Decompose: Self check failed !! Please report to developer !!.");
		}
		
		return actionBdds;
	}

	BDD Compose(vector<BDD> actionBdds){

		BDD composed = pCuddManager->bddOne();
		
		for(size_t i=0; i<ACTION_BDDVARSCOUNT; i++){
			BDD selector = pCuddManager->bddVar(STATES_BDDVARSCOUNT+i);
			actionBdds[i] = selector.Ite(actionBdds[i], pCuddManager->bddOne());
		}

		for(size_t i=0; i<ACTION_BDDVARSCOUNT; i++){
			composed &= actionBdds[i];
		}
		
		return composed;
	}

};

#endif
