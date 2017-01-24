/*
 * ncsTransitionRelation.hh
 *
 *  created on: 07.11.2016
 *      author: M.Khaled
 */

#ifndef NCSTRANSITIONRELATION_HH_
#define NCSTRANSITIONRELATION_HH_

#include <vector>

#include "cuddObj.hh"
#include "SymbolicSetInterface.hh"
#include "ncsState.hh"
#include "ncsFixPoint.hh"
#include "ncsController.hh"
#include "Misc.hh"

#define NCS_TR_REMARKS_TEXT "NCS Transition Relation"

using namespace std;

class ncsTransitionRelation{
protected:
	Cudd* pCuddManager;														// CUDD Manager

	size_t SS_DIM;															// State space dimension, got from the user in command line
	size_t IS_DIM;															// Input space dimension, got from the user in command line
	
	size_t NSCMIN;
	size_t NSCMAX;															// Max delay in the Sensor-Controller Channel, got from the user in command line
	size_t NCAMIN;
	size_t NCAMAX;															// Max delay in the Controller-Actuator Channel, got from the user in command line

	/*
	 * BDD, details and bddVars of the original transition relation
	 */
	BDD f_org_rel;
	vector<size_t> org_x_bddVars, org_u_bddVars, org_xd_bddVars;
	vector<size_t> ssVarsCount, isVarsCount;
	vector<double> ssEta, ssLb, ssUb, isEta, isLb, isUb;


	/* 1] BddVars for the new transition relation.
	 * Note the the new system transition relation will have:
	 * 		(x1, x2 ... xn, u1, u2 ... um, u, xd1, xd2 ... xdn, ud1, ud2 ... udm)
	 *
	 * 	- where n = NscMax and m= NcaMax
	 * 	- x1, um and xd1 are directly from the old transition relation (org_x, org_u, org_xd)
	 * 	- xd2 to xdn are synchronized with x1 to xn-1 (FIFO style shifting)
	 *  - xn is dont-care (can be any allowed value as it will be kicked out of the channel)
	 *  - u  is dont-care (can be any allowed value as it will the input to the system)
	 *  - ud1 is synchronized with u
	 *  - ud2 to udm are synchronized with u1 to um-1
	 *
	 *  x1  = org_x
	 *  xd1 = org_xd
	 *  um  = org_u
	 *
	 *  synchronization means having the same value whatever the combination of other vars. Please also
	 *  notice that all of these values might require more than one bddVar (in case the number of
	 *  possibilities of the values are more than 2).
	 */

	vector<size_t> q1qn, qd1qdn, ui;
	vector<vector<size_t>> x1xn, xd1xdn, u1um, ud1udm;

	/* same vars but as X_U_Xd*/
	vector<size_t> preVars_noq , postVars_noq;
	vector<size_t> qBddVars;
	
	// BDD variables for the new reation and the IC mask
	BDD f_new_rel;
	BDD normalTransRules,  noqTransitions;
	BDD qTransRules, qTransitions;
	string Remarks;
	bool isExpanded = false;
	double expandTime = 0;
	double ComputeContrTime = 0;


	/* Asks the Cudd BDD-Manager to give bddVars for the new elements in the expanded relation
	 * returns the index of last created bddVar */
	size_t RequestNewBddVars(){

		if(org_x_bddVars.size() != org_xd_bddVars.size())
			throw runtime_error("Error::ReArrangeBDDVars:: Size mismatch between the BddVars of X and Xd!");

		if(org_x_bddVars[0] != 0)
			throw runtime_error("Error::ExpandBDD:: First BddVar of the original relation should be BddVar of index 0.");

		if((pCuddManager->ReadSize()-1) != (int)org_xd_bddVars[org_xd_bddVars.size()-1])
			throw runtime_error("Error::ExpandBDD:: The manager should only have BddVars from the original relation.");

		// Counting current and new vars that will be requested
		size_t nBddVarsX = org_x_bddVars.size();
		size_t nBddVarsU = org_u_bddVars.size();

		size_t nBddVars = 2*NSCMAX*nBddVarsX + // for x1xn and xd1xdn
						  2*NCAMAX*nBddVarsU + // for u1um and ud1udm
						  nBddVarsU + 		   // for the single ui
						  2*NSCMAX;			   // for q1qn and qd1qdm

		// exclude already existing BddVars from the original relation
		size_t nBddVarsToRquest = nBddVars-(2*nBddVarsX + nBddVarsU);

		// requesting for extra BddVars
		for(size_t i=0; i<nBddVarsToRquest; i++)
			pCuddManager->bddVar();

		// arranging/assigning the current/new BddVars
		int c=0;

		// q1|x1, q2|x2 .... qn|xn
		for(size_t i=0; i<NSCMAX; i++){
			q1qn.push_back(c++);

			vector<size_t> tmp;
			for(size_t j=0; j<nBddVarsX; j++)
				tmp.push_back(c++);
			x1xn.push_back(tmp);
		}

		// u1, u2 .. um
		for(size_t i=0; i<NCAMAX; i++){
			vector<size_t> tmp;
			for(size_t j=0; j<nBddVarsU; j++)
				tmp.push_back(c++);
			u1um.push_back(tmp);
		}

		for(size_t j=0; j<nBddVarsU; j++)
			ui.push_back(c++);

		// qd1|xd1, qd2|xd2 .... qdn|xdn
		for(size_t i=0; i<NSCMAX; i++){
			qd1qdn.push_back(c++);
			vector<size_t> tmp;
			for(size_t j=0; j<nBddVarsX; j++)
				tmp.push_back(c++);
			xd1xdn.push_back(tmp);
		}

		// ud1, ud2 .. udm
		for(size_t i=0; i<NCAMAX; i++){
			vector<size_t> tmp;
			for(size_t j=0; j<nBddVarsU; j++)
				tmp.push_back(c++);
			ud1udm.push_back(tmp);
		}

		if(c != pCuddManager->ReadSize())
			throw runtime_error("Error::ReArrangeBddVars:: Some error happened, the assigned BddVars seems to miss some.");

	#ifdef VERBOSEBASIC
		cout << "ReArranged BddVars:" << endl;
		cout << "  q1qn: "; VectorManager::PrintVector(q1qn);
		cout << "  x1xn: "; for(size_t i=0; i<NSCMAX;i++) {VectorManager::PrintVector(x1xn[i], ' ', false); cout<<", ";} cout<<endl;
		cout << "  u1um: "; for(size_t i=0; i<NCAMAX;i++) {VectorManager::PrintVector(u1um[i], ' ', false); cout<<", ";} cout<<endl;
		cout << "    ui: "; VectorManager::PrintVector(ui, ' ');
		cout << "qd1qdn: "; VectorManager::PrintVector(qd1qdn);
		cout << "xd1xdn: "; for(size_t i=0; i<NSCMAX;i++) {VectorManager::PrintVector(xd1xdn[i], ' ', false); cout<<", ";} cout<<endl;
		cout << "ud1udm: "; for(size_t i=0; i<NCAMAX;i++) {VectorManager::PrintVector(ud1udm[i], ' ', false); cout<<", ";} cout<<endl;
		cout << endl;
	#endif

		return (c-1);
	}

	/* Permutes a given BDD from original transition relation to the new order of transition relation*/
	BDD PermutreBddToNewVars(BDD& f_to_permute){
		size_t nBddVarsX = org_x_bddVars.size();
		size_t nBddVarsU = org_u_bddVars.size();

		// Permute f_org_rel to reflect the new BddVars
		vector<int> permute = BDDUtils::GetReadyPermuteMap(*pCuddManager);

		for(size_t i=0; i<nBddVarsX; i++){
			permute[org_x_bddVars[i]] = x1xn[0][i];
			permute[org_xd_bddVars[i]] = xd1xdn[0][i];
		}

		for(size_t i=0; i<nBddVarsU; i++){
			permute[org_u_bddVars[i]] = u1um[NCAMAX-1][i];
		}

		BDD f_permuted  = f_to_permute.Permute(permute.data());
		return f_permuted;
	}
	BDD ExpandStateBasedBDD(BDD& f_org_ts){

		BDD f_org_ts_permuted  = PermutreBddToNewVars(f_org_ts);

	#ifdef VERBOSE
		BDDUtils::PrintBDD("f_org_ts before permutation", f_org_ts);
		BDDUtils::PrintBDD("f_org_ts after permutation", f_org_ts_permuted);
	#endif

		BDD f_new_ts  = f_org_ts_permuted*noqTransitions;

	#ifdef VERBOSE
		BDDUtils::PrintBDD("original ts", f_org_ts);
		cout << "support vars of permuted ts  (before projection): "; VectorManager::PrintVector(f_new_ts.SupportIndices());
		BDDUtils::PrintBDD("new permuted ts (before projection)", f_new_ts);
	#endif

		vector<size_t> sourceStateVars;
		VectorManager::AppendVector(sourceStateVars, q1qn);
		VectorManager::AppendVectors(sourceStateVars, x1xn);
		VectorManager::AppendVectors(sourceStateVars, u1um);
		f_new_ts = BDDUtils::ProjectBDD(*pCuddManager, f_new_ts, sourceStateVars);

	#ifdef VERBOSE
		cout << "support vars of ts: "; VectorManager::PrintVector(f_new_ts.SupportIndices());
		BDDUtils::PrintBDD("new ts", f_new_ts);
	#endif

		return f_new_ts;
	}

	/* Virtual unctions needed by any NCS-TR type  */
	virtual ncsState* getSourceStateTemplate() = 0;
	virtual BDD ConstructQTransitionRules() = 0;
	virtual BDD ConstructNormalTransRules(const BDD& f_org_permuted, const BDD& f_org_projected) = 0;


public:
	// Constructors
	/* reads an NBDD File holding a transition relation */
	ncsTransitionRelation(Cudd& cuddManager_, const char* NBDD_INPUT_FILE){
		pCuddManager = &cuddManager_;
		string Remarks;

		BDDUtils::readFromFile(NBDD_INPUT_FILE, cuddManager_,
					  f_new_rel,
					  SS_DIM, IS_DIM,
					  ssVarsCount, isVarsCount,
					  ssEta, ssLb, ssUb, isEta, isLb, isUb,
					  NSCMIN, NSCMAX, NCAMIN, NCAMAX,
					  preVars_noq, ui, postVars_noq,
					  qBddVars, Remarks);

		size_t good_remarks = Remarks.find(NCS_TR_REMARKS_TEXT);
		if (good_remarks==std::string::npos)
			throw(runtime_error("Error::ncsTransitionRelation:: This is not a valid NCS Transition Relation File !!"));

		string time = "Time:";
		size_t time_found = Remarks.find(time);
		if (time_found!=std::string::npos){
			stringstream ss;
			ss << Remarks.substr(time_found+time.length());
			ss >> expandTime;
		}

		isExpanded = true;

		// TO DO :: fill the other original BDD and BddVars
		size_t nBddVarsX = 0, nBddVarsU = 0;

		for(size_t i=0, k=0; i<SS_DIM; i++)
			for(size_t j=0; j<ssVarsCount[i]; j++, k++){
				org_x_bddVars.push_back(preVars_noq[k]);
				org_xd_bddVars.push_back(postVars_noq[k]);
			 	nBddVarsX++;
			}

		for(size_t i=0; i<IS_DIM; i++)
			for(size_t j=0; j<isVarsCount[i]; j++)
				nBddVarsU++;


		for(size_t i=0, k=0; i<IS_DIM; i++)
			for(size_t j=0; j<isVarsCount[i]; j++, k++)
				org_u_bddVars.push_back(preVars_noq[NSCMAX*nBddVarsX + (NCAMAX-1)*nBddVarsU + k]);

		size_t c=0;
		for(size_t i=0; i<NSCMAX; i++){
			vector<size_t> tmp1,tmp2;
			for(size_t j=0; j<SS_DIM; j++){
				for(size_t k=0; k<ssVarsCount[j]; k++){
					tmp1.push_back(preVars_noq[c]);
					tmp2.push_back(postVars_noq[c]);
					c++;
				}
			}
			x1xn.push_back(tmp1);
			xd1xdn.push_back(tmp2);
		}

		for(size_t i=0; i<NCAMAX; i++){
			vector<size_t> tmp1,tmp2;
			for(size_t j=0; j<IS_DIM; j++){
				for(size_t k=0; k<isVarsCount[j]; k++){
					tmp1.push_back(preVars_noq[c]);
					tmp2.push_back(postVars_noq[c]);
					c++;
				}
			}
			u1um.push_back(tmp1);
			ud1udm.push_back(tmp2);
		}

		size_t nQsOverTwo = qBddVars.size()/2;
		for(size_t i=0; i<nQsOverTwo; i++){
			q1qn.push_back(qBddVars[i]);
			qd1qdn.push_back(qBddVars[nQsOverTwo+i]);
		}


		BDD NoQsCube = BDDUtils::ConstructZerosAnd(*pCuddManager, qBddVars);
		f_org_rel = f_new_rel*NoQsCube;

		vector<size_t> projVars;
		VectorManager::AppendVector(projVars, org_x_bddVars);
		VectorManager::AppendVector(projVars, org_u_bddVars);
		VectorManager::AppendVector(projVars, org_xd_bddVars);
		f_org_rel = BDDUtils::ProjectBDD(*pCuddManager, f_org_rel, projVars);

	}
	/* Reads a BDD file and expnds the relation*/
	ncsTransitionRelation(Cudd& cuddManager_, 
						  const char* BDD_INPUT_FILE,
						  const size_t SS_DIM_, const size_t IS_DIM_, 
						  const size_t NSCMIN_, const size_t NSCMAX_, const size_t NCAMIN_, const size_t NCAMAX_){
					
		pCuddManager = &cuddManager_;

		SS_DIM = SS_DIM_;
		IS_DIM = IS_DIM_;

		NSCMIN = NSCMIN_;
		NSCMAX = NSCMAX_;
		NCAMIN = NCAMIN_;
		NCAMAX = NCAMAX_;

		// Reading original transition relation and target/avoid states from the file as a symbolic set
		if(!IOUtils::file_exists(BDD_INPUT_FILE)){
			throw runtime_error("Error::ncsTransitionRelation:: The input file ( trans_rel ) does not exist.");
		}

		SymbolicSet ss_org;
		ss_org.LoadFromFile(*pCuddManager, BDD_INPUT_FILE);
		
		f_org_rel = ss_org.getSymbolicSet();

		// Checking dimensions in the file with the provided ones
		if(ss_org.getDimension() != (2*SS_DIM + IS_DIM)){
			throw runtime_error("Error::ncsTransitionRelation:: input provided dimensions in input-file do not match the given dimensions");
		}

		ssVarsCount = VectorManager::ArrayToVector(ss_org.getNofBddVars(),SS_DIM);
		isVarsCount = VectorManager::ArrayToVector(ss_org.getNofBddVars()+SS_DIM, IS_DIM);
		ssEta 		= VectorManager::ArrayToVector(ss_org.getEta(),SS_DIM);
		ssLb 		= VectorManager::ArrayToVector(ss_org.getFirstGridPoint(),SS_DIM);
		ssUb 		= VectorManager::ArrayToVector(ss_org.getLastGridPoint(),SS_DIM);
		isEta 		= VectorManager::ArrayToVector(ss_org.getEta()+SS_DIM, IS_DIM);
		isLb 		= VectorManager::ArrayToVector(ss_org.getFirstGridPoint()+SS_DIM, IS_DIM);
		isUb 		= VectorManager::ArrayToVector(ss_org.getLastGridPoint()+SS_DIM, IS_DIM);




		SymbolicSetInterface::ExtractBddVars(ss_org, org_x_bddVars, org_u_bddVars, org_xd_bddVars, SS_DIM, IS_DIM);

		/* 3] Requesting extra bddVars for the expanded system and re arrange BddVars
		 */
		size_t last_var = RequestNewBddVars();

		if((last_var-1)>(sizeof(size_t)*8)){
			cout << "Warning::ncsTransitionRelation:: Your bddVars exceeds the bound of (size_t). You will have to do more job if you like to iterate on the transitions !" << endl;
		}


		VectorManager::AppendVectors(preVars_noq, x1xn);
		VectorManager::AppendVectors(preVars_noq, u1um);

		VectorManager::AppendVectors(postVars_noq, xd1xdn);
		VectorManager::AppendVectors(postVars_noq, ud1udm);

		VectorManager::AppendVector(qBddVars, q1qn);
		VectorManager::AppendVector(qBddVars, qd1qdn);
	}
	virtual ~ncsTransitionRelation(){
	}
	
	void ExpandBDD(){
		if(isExpanded)
			throw runtime_error("Error::Expand:: You are trying to expand already expanded NCS transition relation !");

		StopWatch::Start();
		// 0] Permute and Project the original relation
		BDD f_org_rel_permuted, f_org_rel_projected;
		f_org_rel_permuted  = PermutreBddToNewVars(f_org_rel);

		vector<size_t> projVars;
		VectorManager::AppendVector(projVars, x1xn[0]);
		VectorManager::AppendVector(projVars, xd1xdn[0]);
		f_org_rel_projected = BDDUtils::ProjectBDD(*pCuddManager, f_org_rel_permuted, projVars);

	#ifdef VERBOSE
		BDDUtils::PrintBDD("f_org_rel before permutation", f_org_rel);
		BDDUtils::PrintBDD("f_org_rel after permutation", f_org_rel_permuted);
		BDDUtils::PrintBDD("f_org_rel after projection", f_org_rel_projected);
	#endif

		// 1] Normal Transitions
		// But first, All Qs are zeros
		vector<size_t> AllQsBddVars;
		VectorManager::AppendVector(AllQsBddVars, q1qn);
		VectorManager::AppendVector(AllQsBddVars, qd1qdn);
		BDD f_AllQsAreZeros = BDDUtils::ConstructZerosAnd(*pCuddManager, AllQsBddVars);

	#ifdef VERBOSE
			BDDUtils::PrintBDD("f_AllQsAreZeros", f_AllQsAreZeros);
	#endif

		normalTransRules = ConstructNormalTransRules(f_org_rel_permuted, f_org_rel_projected);
		noqTransitions = normalTransRules*f_AllQsAreZeros;

	#ifdef VERBOSE
			BDDUtils::PrintBDD("noqTransitions", noqTransitions);
	#endif

		// 2] Q-based Transitions
		qTransRules = ConstructQTransitionRules();
		qTransitions = qTransRules*/*f_org_rel_permuted*/normalTransRules/*f_org_rel_projected*/;

		#ifdef VERBOSE
			BDDUtils::PrintBDD("qTransitions", qTransitions);
		#endif

		// 3] The new expanded relation
		f_new_rel = qTransitions+noqTransitions;

		expandTime = StopWatch::Stop(false);

	#ifdef VERBOSE
		BDDUtils::PrintBDD("new rel", f_new_rel);
	#endif

	#ifdef VERBOSEBASIC
		cout << "The size of the new expanded relation is: " << f_new_rel.CountMinterm(pCuddManager->ReadSize()) << endl;
	#endif

	}

	size_t getStateVarsCount(){
		return getSourceStateTemplate()->getVarsCount();
	}
	size_t getInpVarsCount(){
		return ui.size();
	}
	size_t getVarsCount(){
		return 2*getStateVarsCount() + getInpVarsCount();
	}

	vector<size_t> getSsVarsCount(){
		return ssVarsCount;
	}
	vector<size_t> getIsVarsCount(){
		return isVarsCount;
	}

	size_t getNscMax(){
		return NSCMAX;
	}

	size_t getNcaMax(){
		return NCAMAX;
	}

	BDD getOriginalRelation(){
		return f_org_rel;
	}
	double getExpandTime(){
		return expandTime;
	}
	double getComputeContrTime(){
		return ComputeContrTime;
	}

	vector<size_t> getPreVars(){
		vector<size_t> preVars;
		for(size_t i=0; i<NSCMAX; i++){
			preVars.push_back(q1qn[i]);
			VectorManager::AppendVector(preVars, x1xn[i]);
		}
		VectorManager::AppendVectors(preVars, u1um);
		return preVars;
	}
	vector<size_t> getPostVars(){
		vector<size_t> postVars;
		for(size_t i=0; i<NSCMAX; i++){
			postVars.push_back(qd1qdn[i]);
			VectorManager::AppendVector(postVars, xd1xdn[i]);
		}
		VectorManager::AppendVectors(postVars, ud1udm);
		return postVars;
	}
	vector<size_t> getOrginalVars(){
		vector<size_t> orgVars;
		VectorManager::AppendVector(orgVars, org_x_bddVars);
		VectorManager::AppendVector(orgVars, org_u_bddVars);
		VectorManager::AppendVector(orgVars, org_xd_bddVars);
		return orgVars;
	}
	BDD getTransitionRelation(){
		return f_new_rel;
	}
	BDD getXICMask(){
		throw(runtime_error("Not Implemented !!"));
	}


	ncsController ComputeReachController(const char* BDD_FILE_TARGET){
		SymbolicSet ss_ts;
		ss_ts.LoadFromFile(*pCuddManager, BDD_FILE_TARGET);

		return ComputeReachController(ss_ts.getSymbolicSet());
	}
	ncsController ComputeReachController(BDD f_org_ts){

		vector<size_t> preVars = getPreVars();
		vector<size_t> postVars = getPostVars();

		ncsFixPoint fp(pCuddManager, f_new_rel, preVars, ui, postVars);

		BDD f_ts = ExpandStateBasedBDD(f_org_ts);

		StopWatch::Start();
		BDD C = fp.reach(f_ts,1);
		ComputeContrTime = StopWatch::Stop();

	#ifdef VERBOSE
		BDDUtils::PrintBDD("The Controller:", C);
	#endif


		ncsController contr(*pCuddManager, C, getSourceStateTemplate(), ui, ComputeContrTime);

		return contr;

	}


	ncsController ComputeSafetyController(const char* BDD_FILE_SAFESET){
		SymbolicSet ss_safe;
		ss_safe.LoadFromFile(*pCuddManager, BDD_FILE_SAFESET);

		return ComputeSafetyController(ss_safe.getSymbolicSet());
	}
	ncsController ComputeSafetyController(BDD f_org_safe){

		vector<size_t> preVars = getPreVars();
		vector<size_t> postVars = getPostVars();

		ncsFixPoint fp(pCuddManager, f_new_rel, preVars, ui, postVars);

		BDD f_safe = ExpandStateBasedBDD(f_org_safe);

		StopWatch::Start();
		BDD C = fp.safe(f_safe, 1);
		ComputeContrTime = StopWatch::Stop();

	#ifdef VERBOSE
		BDDUtils::PrintBDD("The Controller:", C);
	#endif


		ncsController contr(*pCuddManager, C, getSourceStateTemplate(), ui, ComputeContrTime);

		return contr;

	}


	ncsController ComputePersistanceController(const char* BDD_FILE_TARGET){
		SymbolicSet ss_ts;
		ss_ts.LoadFromFile(*pCuddManager, BDD_FILE_TARGET);

		return ComputePersistanceController(ss_ts.getSymbolicSet());
	}
	ncsController ComputePersistanceController(BDD f_org_ts){

		vector<size_t> preVars = getPreVars();
		vector<size_t> postVars = getPostVars();

		ncsFixPoint fp(pCuddManager, f_new_rel, preVars, ui, postVars);

		BDD f_ts = ExpandStateBasedBDD(f_org_ts);

		StopWatch::Start();
		BDD C = fp.persistance(f_ts,1);
		ComputeContrTime = StopWatch::Stop();

	#ifdef VERBOSE
		BDDUtils::PrintBDD("The Controller:", C);
	#endif


		ncsController contr(*pCuddManager, C, getSourceStateTemplate(), ui, ComputeContrTime);

		return contr;

	}


	ncsController ComputeRecurrenceController(const char* BDD_FILE_TARGET){
		SymbolicSet ss_ts;
		ss_ts.LoadFromFile(*pCuddManager, BDD_FILE_TARGET);

		return ComputeRecurrenceController(ss_ts.getSymbolicSet());
	}
	ncsController ComputeRecurrenceController(BDD f_org_ts){

		vector<size_t> preVars = getPreVars();
		vector<size_t> postVars = getPostVars();

		ncsFixPoint fp(pCuddManager, f_new_rel, preVars, ui, postVars);

		BDD f_ts = ExpandStateBasedBDD(f_org_ts);

		StopWatch::Start();
		BDD C = fp.recurrence(f_ts,1);
		ComputeContrTime = StopWatch::Stop();

	#ifdef VERBOSE
		BDDUtils::PrintBDD("The Controller:", C);
	#endif


		ncsController contr(*pCuddManager, C, getSourceStateTemplate(), ui, ComputeContrTime);

		return contr;

	}


	vector<ncsController> ComputeRecurrenceConjController(vector<const char*> vBDD_FILE_TARGET){
		vector<SymbolicSet> ss_targets(vBDD_FILE_TARGET.size());
		vector<BDD> targetBDDs(vBDD_FILE_TARGET.size());

		for(size_t i=0; i<vBDD_FILE_TARGET.size(); i++){
			ss_targets[i].LoadFromFile(*pCuddManager, vBDD_FILE_TARGET[i]);
			targetBDDs[i] = ss_targets[i].getSymbolicSet();
		}

		return ComputeRecurrenceConjController(targetBDDs);
	}
	vector<ncsController> ComputeRecurrenceConjController(vector<BDD> f_org_targets){

		vector<size_t> preVars = getPreVars();
		vector<size_t> postVars = getPostVars();
		vector<BDD> f_targets(f_org_targets.size());
		ncsFixPoint fp(pCuddManager, f_new_rel, preVars, ui, postVars);

		for(size_t i=0; i<f_org_targets.size(); i++)
			f_targets[i] = ExpandStateBasedBDD(f_org_targets[i]);

		StopWatch::Start();
		vector<BDD> vC = fp.recurrence_conj(f_targets,1);
		ComputeContrTime = StopWatch::Stop();

		vector<ncsController> vContr;
		for(size_t i=0; i<f_org_targets.size(); i++){
			ncsController contr(*pCuddManager, vC[i], getSourceStateTemplate(), ui, ComputeContrTime);
			vContr.push_back(contr);

		}

		return vContr;

	}



	void WriteToFile(string outRelationFile){

		stringstream ss;
		ss << NCS_TR_REMARKS_TEXT << " | Time:" << expandTime;

		BDDUtils::writeToFile(
					f_new_rel,
					SS_DIM, IS_DIM,
					ssVarsCount.data(), isVarsCount.data(),
					ssEta.data(), ssLb.data() , ssUb.data(), isEta.data(), isLb.data(), isUb.data(),
					NSCMAX, NSCMAX, NCAMAX, NCAMAX,
					preVars_noq, ui, postVars_noq,
					qBddVars,
					ss.str().c_str(),
					outRelationFile.c_str());
	}
}; /* class ncsModel */


class ncsFIFOTransitionRelation : public ncsTransitionRelation {
private:
	ncsState* sourceState;

	void Init(){
		sourceState = new ncsFifoState(NSCMAX, NCAMAX,
										SS_DIM, IS_DIM,
										ssEta, ssLb, ssUb, isEta, isLb, isUb,
										ssVarsCount, isVarsCount,
										x1xn, u1um, q1qn);
	}

public:
	// Constructors
	ncsFIFOTransitionRelation(Cudd& cuddManager_, const char* NBDD_INPUT_FILE) :
			ncsTransitionRelation(cuddManager_, NBDD_INPUT_FILE){
		Init();
	}
	ncsFIFOTransitionRelation(Cudd& cuddManager_, 
			  const char* BDD_INPUT_FILE,
			  const size_t SS_DIM_, const size_t IS_DIM_, 
			  const size_t NSCMAX_, const size_t NCAMAX_) :
		ncsTransitionRelation(cuddManager_, BDD_INPUT_FILE, SS_DIM_, IS_DIM_, NSCMAX_, NSCMAX_, NCAMAX_, NCAMAX_){
		Init();
	}
	~ncsFIFOTransitionRelation(){
		delete sourceState;
	}

	ncsState* getSourceStateTemplate(){
		return sourceState;
	}
	
	// Interface methods enforced by the abstract class
	/* Construct rules of the Q-based Transitions as a BDD  */
	BDD ConstructQTransitionRules(){

		BDD qTransRules = pCuddManager->bddOne();

		// 1] qd1 is always zero
		BDD R1_qd1_zero = !pCuddManager->bddVar(qd1qdn[0]);

	#ifdef VERBOSE
		BDDUtils::PrintBDD("R1_qd1_zero", R1_qd1_zero);
	#endif

		// 2] any q=1 ==> its x=0
		BDD R2_qOne_implies_xZero = pCuddManager->bddOne();
		for(size_t i=0; i<NSCMAX; i++){
			BDD qEqOne;
			BDD xEqZeros;
			qEqOne   = pCuddManager->bddVar(q1qn[i]);
			xEqZeros = BDDUtils::ConstructZerosAnd(*pCuddManager, x1xn[i]);
			R2_qOne_implies_xZero *= qEqOne.Ite(xEqZeros, pCuddManager->bddOne());

			qEqOne   = pCuddManager->bddVar(qd1qdn[i]);
			xEqZeros = BDDUtils::ConstructZerosAnd(*pCuddManager, xd1xdn[i]);
			R2_qOne_implies_xZero *= qEqOne.Ite(xEqZeros, pCuddManager->bddOne());
		}

	#ifdef VERBOSE
		BDDUtils::PrintBDD("R2_qOne_implies_xZero", R2_qOne_implies_xZero);
	#endif


		// 3] Allowed configurations of Qs (Pyramid-Upsidedown)
		BDD R3_Allowed_Q_Configs = pCuddManager->bddZero();
		size_t skip_q =0, skip_qd = 1;				// this allows for the (all-q-in-source case)
		for(size_t i=0; i<NSCMAX; i++){
			BDD subR3 = pCuddManager->bddOne();

			for(size_t j=0; j<q1qn.size(); j++){
				if(j >= skip_q)
					subR3 *= pCuddManager->bddVar(q1qn[j]);
				else
					subR3 *= !pCuddManager->bddVar(q1qn[j]);
			}

			for(size_t j=0; j<qd1qdn.size(); j++){
				if(j >= skip_qd)
					subR3 *= pCuddManager->bddVar(qd1qdn[j]);
				else
					subR3 *= !pCuddManager->bddVar(qd1qdn[j]);
			}

			skip_q++;
			skip_qd++;
			R3_Allowed_Q_Configs += subR3;
		}

	#ifdef VERBOSE
		BDDUtils::PrintBDD("R3_Allowed_Q_Configs", R3_Allowed_Q_Configs);
	#endif

		// 4] All us are the same
		BDD R4_AllUsTheSame = pCuddManager->bddOne();
		for(size_t i=0; i<NCAMAX; i++){
			R4_AllUsTheSame *= BDDUtils::BddSyncValues(*pCuddManager, ui, u1um[i]);
			R4_AllUsTheSame *= BDDUtils::BddSyncValues(*pCuddManager, ui, ud1udm[i]);
		}

	#ifdef VERBOSE
		BDDUtils::PrintBDD("R4_AllUsTheSame", R4_AllUsTheSame);
	#endif

		// Collect Rules
		qTransRules = R1_qd1_zero * R2_qOne_implies_xZero * R3_Allowed_Q_Configs*R4_AllUsTheSame;

	#ifdef VERBOSE
		BDDUtils::PrintBDD("qTransRules", qTransRules);
	#endif

		return qTransRules;

	}
	/* Construct rules of the Non-Q Transitions as a BDD */
	BDD ConstructNormalTransRules(const BDD& f_org_permuted, const BDD& f_org_projected){

		size_t nBddVarsX = x1xn[0].size();

		// 1] R1: follow the original transition
		BDD R1_OrgTrans_Permuted = f_org_permuted;

		// 2] R2: X-Coherency (xs come from Xs not arbitrary)
		BDD R2_XCoherency = pCuddManager->bddOne();
		for(size_t i=0; i<NSCMAX-1; i++){
			// x[i+1] --> x[i]
			vector<int> permute = BDDUtils::GetReadyPermuteMap(*pCuddManager);

			for(size_t j=0; j<nBddVarsX; j++){
				permute[x1xn[0][j]]   = x1xn[i+1][j];	// the pre
				permute[xd1xdn[0][j]] = x1xn[i][j];	// the post
			}

			R2_XCoherency *= f_org_projected.Permute(permute.data());

	#ifdef VERBOSE
			BDDUtils::PrintBDD("R2_XCoherency", R2_XCoherency);
	#endif
		}

		// 3] R3: Xs are shifted from pre to post (synchronization)
		BDD R3_XShifting = pCuddManager->bddOne();
		for(size_t i=0; i<NSCMAX-1; i++){
			R3_XShifting *= BDDUtils::BddSyncValues(*pCuddManager, x1xn[i], xd1xdn[i+1]);
		}

	#ifdef VERBOSE
			BDDUtils::PrintBDD("R3_XShifting", R3_XShifting);
	#endif

		// 4] R4: Input ui goes to first Input in post (syncronization)
		BDD R4_ui_ud1_sync = BDDUtils::BddSyncValues(*pCuddManager, ui, ud1udm[0]);

	#ifdef VERBOSE
			BDDUtils::PrintBDD("R4_ui_ud1_sync", R4_ui_ud1_sync);
	#endif

		// 5] R5 Us are shifted from pre to post
		BDD R5_UShifting = pCuddManager->bddOne();
		for(size_t i=0; i<NCAMAX-1; i++){
			R5_UShifting *= BDDUtils::BddSyncValues(*pCuddManager, u1um[i], ud1udm[i+1]);
		}

	#ifdef VERBOSE
			BDDUtils::PrintBDD("R5_UShifting", R5_UShifting);
	#endif

		// Combine all
		BDD normalTransRules = R1_OrgTrans_Permuted*R2_XCoherency*R3_XShifting*R4_ui_ud1_sync*R5_UShifting;

	#ifdef VERBOSE
			BDDUtils::PrintBDD("normalTransRules", normalTransRules);
	#endif

		return normalTransRules;
	}

};

#endif /* NCSTRANSITIONRELATION_HH_ */
