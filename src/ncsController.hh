/*
 * ncsController.hh
 *
 *  created on: 17.11.2016
 *      author: M.Khaled
 */

#ifndef NCSCONTROLLER_HH_
#define NCSCONTROLLER_HH_

#include <vector>
#include "cuddObj.hh"

#include "CuddMintermIterator.hh"
#include "Misc.hh"
#include "ncsState.hh"

#define NCS_CONTR_REMARKS_TEXT "NCS Synthesized Controller"

using namespace std;

class ncsController {
protected:
	
	/* vars: pCuddManager
	 * pointer to the BDD manager*/
	Cudd* pCuddManager;

	/* vars: sourceState
	 * an ncsState to hold contining bddVars infos and methods for source state*/
	ncsState* sourceState;
	bool sourceStateLocallyCreated = false;
	
	/* vars: preVars
	 * a vector to hold the bddVars of the input to the BigX state from the expanded transition relation*/
	vector<size_t> inpVars;
	
	/*var: bddContr 
	 * a BDD that contains the controller */
	BDD bddContr;
	
	double contrTime = 0;

	
	void init(vector<size_t> inpVars_){
		if(sourceState->getBddVars().size() == 0)
			throw runtime_error("Error::init:: source state has no valid state info !");
		
		if(VectorManager::isVectorIntersects(inpVars_, sourceState->getBddVars()))
			throw runtime_error("Error::init:: inputs vars should be different from state vars!");
		
		inpVars = inpVars_;
		
		size_t isVarsCount = 0;
		vector<size_t> isVarsCountPerDimension = sourceState->getIsVarsCount();
		for(size_t i=0; i<isVarsCountPerDimension.size(); i++)
			isVarsCount += isVarsCountPerDimension[i];
		
		if(isVarsCount != inpVars.size())
			throw runtime_error("Error::init:: size mismatch in input vars !");
	}

public:
	ncsController(){
	}

	ncsController(Cudd& cuddManager_, BDD bddContr_, ncsState* sourceState_, vector<size_t> inpVars_, double contrTime_){

		pCuddManager = &cuddManager_;
		bddContr = bddContr_;
		contrTime = contrTime_;
		sourceState = sourceState_;

		init(inpVars_);
	}
	

	ncsController(Cudd& cuddManager_, const char* INPUT_FILE){

		pCuddManager = &cuddManager_;

		size_t ssdim, isdim, nscmin, nscmax, ncamin, ncamax;
		vector<size_t> ssVarsCount, isVarsCount, preVars, postVars, qVars, inpVars_;
		vector<double> ssEta, ssLb, ssUb, isEta, isLb, isUb;
		string Remarks;

		BDDUtils::readFromFile(INPUT_FILE, cuddManager_, bddContr, ssdim, isdim,
						ssVarsCount, isVarsCount,
						ssEta, ssLb, ssUb, isEta, isLb, isUb,
						nscmin, nscmax, ncamin, ncamax,
						preVars, inpVars_, postVars, qVars,
						Remarks);

		size_t good_remarks = Remarks.find(NCS_CONTR_REMARKS_TEXT);
		if (good_remarks==std::string::npos)
			throw(runtime_error("Error::ncsController:: This is not a valid NCS Controller File !!"));

		string time = "Time:";
		size_t time_found = Remarks.find(time);
		if (time_found!=std::string::npos){
			stringstream ss;
			ss << Remarks.substr(time_found+time.length());
			ss >> contrTime;
		}

		if(qVars.size() != nscmax)
			throw(runtime_error("Error::ncsController:: Stored Q-Vars are not valid !!"));

		vector<vector<size_t>> x1xn, u1um;
		size_t c=0;
		for(size_t i=0; i<nscmax; i++){
			vector<size_t> tmp1;
			for(size_t j=0; j<ssdim; j++){
				for(size_t k=0; k<ssVarsCount[j]; k++){
					tmp1.push_back(preVars[c]);
					c++;
				}
			}
			x1xn.push_back(tmp1);
		}

		for(size_t i=0; i<ncamax; i++){
			vector<size_t> tmp1;
			for(size_t j=0; j<isdim; j++){
				for(size_t k=0; k<isVarsCount[j]; k++){
					tmp1.push_back(preVars[c]);
					c++;
				}
			}
			u1um.push_back(tmp1);
		}

		sourceState = new ncsFifoState(nscmax, ncamax, ssdim, isdim, ssEta, ssLb, ssUb,isEta, isLb, isUb, ssVarsCount,isVarsCount, x1xn, u1um, qVars);
		sourceStateLocallyCreated = true;

		init(inpVars_);
	}

	double getControllerComputeTile(){
		return contrTime;
	}
	BDD getBDD(){
		return bddContr;
	}

	void WriteToFile(const char* OUT_FILE_CONTR){

		stringstream ss;
		ss << NCS_CONTR_REMARKS_TEXT << " | Time:" << contrTime;

		string outFile = OUT_FILE_CONTR;

		vector<size_t> preVars_noq, postVars_noq;
		VectorManager::AppendVectors(preVars_noq, sourceState->getX1xnBddVars());
		VectorManager::AppendVectors(preVars_noq, sourceState->getU1umBddVars());

		BDDUtils::writeToFile(
					bddContr,
					sourceState->getSsdim(),
					sourceState->getIsdim(),
					sourceState->getSsVarsCount().data(),
					sourceState->getIsVarsCount().data(),
					sourceState->getSsEta().data(),
					sourceState->getSsLb().data(),
					sourceState->getSsUb().data(),
					sourceState->getIsEta().data(),
					sourceState->getIsLb().data(),
					sourceState->getIsUb().data(),
					sourceState->getNscmin(),
					sourceState->getNscmax(),
					sourceState->getNcamin(),
					sourceState->getNcamax(),
					preVars_noq,
					inpVars,
					postVars_noq,
					sourceState->getqBddVars(),
					ss.str().c_str(),
					outFile.c_str());
	}

	vector<vector<double>> getInputs(vector<double>& XU_values, vector<int>& q_values){
		
		vector<vector<double>> Inputs;		
	
		BDD proj = sourceState->getCube(*pCuddManager, XU_values, q_values) * bddContr;
		size_t isdim = sourceState->getIsdim();

	#ifdef VERBOSE
		cout << "The controller recieved the state (no q): ";
		VectorManager::PrintVector(XU_values, ',', false);
		cout << endl;
	#endif

		vector<size_t> PreInpVars;

		VectorManager::AppendVector(PreInpVars, sourceState->getBddVars());
		VectorManager::AppendVector(PreInpVars, inpVars);

		CuddMintermIterator  it(proj, PreInpVars, PreInpVars.size());
		vector<double> isEta = sourceState->getIsEta();
		vector<double> isLb = sourceState->getIsLb();
		vector<double> isUb = sourceState->getIsUb();

		vector<size_t> isVarsCount = sourceState->getIsVarsCount();

		for(; !it.done(); ++it) {
			const int* xu_bin = it.currentMinterm();
			vector<double> sub_u_values;
			try{
				for(size_t i=0, k=0; i<isdim; i++){
					vector<int> sub_u_bin;

					for(size_t j=0; j<isVarsCount[i]; j++, k++)
						sub_u_bin.push_back(xu_bin[inpVars[k]]);

					sub_u_values.push_back(Quantizer::decode(sub_u_bin, isEta[i], isLb[i], isUb[i]));
				}
				Inputs.push_back(sub_u_values);
			}catch(const std::exception& exp){}
		}		

	#ifdef VERBOSE
		if(Inputs.size() != 0){
			cout << "The following control-inputs were found: ";
			for(size_t i=0; i<Inputs.size(); i++) {VectorManager::PrintVector(Inputs[i], ',', false); cout << " || ";}
			cout << endl;
		}
		else{
			cout << "No control inputs were found !!" << endl;
		}
		cout << "---------------------------------------------------------------------------------------------------------------------------" << endl;
	#endif

		return Inputs;
	}

	ncsState* getSourceState(){
		return sourceState;
	}

	size_t getVarsCount(){
		return sourceState->getVarsCount() + inpVars.size();
	}

	~ncsController(){
		if(sourceStateLocallyCreated)
			delete sourceState;
	}
	
};


#endif /* NCSCONTROLLER_HH_ */
