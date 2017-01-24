#ifndef FIFOSIM_HH_
#define FIFOSIM_HH_

#include <vector>
#include <queue>
#include "cuddObj.hh"
#include "SENSE.hh"
#include "RungeKutta4.h"

class Plant{
protected:
    int ssdim=0, isdim=0;
    double odeTau=-1;
    OdeSolver* solver = nullptr;
    bool initialized = false;

public:
    Plant(){
    }

    int getSsDim(){return ssdim;}
    int getIsDim(){return isdim;}

    void base_initialize(int ssdim_, int isdim_){
        ssdim = ssdim_;
        isdim = isdim_;

        solver = new OdeSolver(ssdim, 5, odeTau);

        initialized = true;
    }
    virtual ~Plant(){
        delete solver;
    }
    std::vector<double> Compute(const std::vector<double>& x, const std::vector<double>& u){

        std::vector<double> ret = x;

	    void (Plant::*func)(std::vector<double>&, std::vector<double>, std::vector<double>);
	    func = &Plant::oderhs;
	    solver->solve(*this, func, ret, u);
	    return ret;
    }

    virtual void initOde(double tau)=0;
    virtual void oderhs(std::vector<double>& xnew, std::vector<double> x, std::vector<double> u)=0;
};

class ncsFIFOEstimator{
	private:
	size_t NSCMAX;
	size_t NCAMAX;

	std::vector<std::vector<double>> ncsFifoStateValues_X;   // order: (x0, x1, x2 ...... )
	std::vector<std::vector<double>> ncsFifoStateValues_U;   // order: (un, un-1, .... u2, u1)

	std::vector<std::vector<double>> uoutBuffer;

	std::vector<double> u0;

	Plant* plant;

	public:
	ncsFIFOEstimator(size_t NSC, size_t NCA, std::vector<double> u0_,  double tau, Plant* plant_){
	    NSCMAX = NSC;
	    NCAMAX = NCA;
	    
	    u0 = u0_;

	    ncsFifoStateValues_X.resize(NSCMAX);
	    ncsFifoStateValues_U.resize(NCAMAX);

	    for(size_t i=0; i<NCAMAX; i++)
		uoutBuffer.push_back(u0);


	    // plant model identification
	    plant = plant_;
	    plant->initOde(tau);
	}
	~ncsFIFOEstimator(){
	}

	Plant* getPlant(){
	    return plant;
	}

	void Refresh(const std::vector<double>& new_x, const std::vector<double>& old_u){

	    // estimating the state values not yet arrived to onstruct the NCS state
	    std::vector<double> tmp = old_u;
	    uoutBuffer.push_back(tmp);
	    
	    ncsFifoStateValues_X[NSCMAX-1]=new_x;
	    for(size_t i=1; i<NSCMAX; i++)
		ncsFifoStateValues_X[NSCMAX-i-1] = plant->Compute(ncsFifoStateValues_X[NSCMAX-i],uoutBuffer[i-1]);

	    uoutBuffer.erase(uoutBuffer.begin());


	    // A copy ready for the call of getNCSFifoStateValues

	    size_t copy_idx = uoutBuffer.size() - NCAMAX;
	    for(size_t i=0; i<NCAMAX; i++)
		ncsFifoStateValues_U[i] = uoutBuffer[copy_idx + i];

	}

	std::vector<std::vector<double>> getNCSStateValues(){
	    std::vector<std::vector<double>> ncs_state;
	    
	    for(size_t i=0; i< ncsFifoStateValues_X.size(); i++)
		ncs_state.push_back(ncsFifoStateValues_X[i]);

	    for(size_t i=0; i< ncsFifoStateValues_U.size(); i++)
		ncs_state.push_back(ncsFifoStateValues_U[ncsFifoStateValues_U.size()-1 -i]);
	    
	    return  ncs_state;
	}

	std::vector<double> getCurrentSystemState(){
	    return ncsFifoStateValues_X[NSCMAX-1];
	}

};

class Packet{
private:
	bool isEmpty_;
	vector<double> data_;
public:
	Packet(){
		isEmpty_ = true;
	}
	Packet(const vector<double>& data){
		isEmpty_ = false;
		data_ = data;
	}

	bool isEmpty(){
		return isEmpty_;
	}
	vector<double> get(){
		return data_;
	}
};

class FIFOChannel{
private:
	queue<Packet> channel;
	size_t size_;
public:
	FIFOChannel(size_t size){
		size_ = size;
		for(size_t i=0; i<size; i++){
			Packet tmp;
			channel.push(tmp);
		}
	}

	FIFOChannel(size_t size, vector<double> initial){
		size_ = size;
		for(size_t i=0; i<size; i++){
			Packet tmp(initial);
			channel.push(tmp);
		}
	}

	void put(const vector<double> in){
		Packet tmp(in);
		channel.push(tmp);
	}

	vector<double> get(){
		Packet tmp = channel.front();
		channel.pop();
		return tmp.get();
	}
};

class ncsFIFOController{
private:
	Cudd* pCuddManager;
	size_t mode = 0;
	vector<ncsController*> controllers;
	vector<SymbolicSet*> ssTargets;
	size_t nscmax;
public:
	ncsFIFOController(Cudd& pMgr, vector<string>& conts, vector<string>& targets, size_t nscmax_){
		pCuddManager = &pMgr;
		nscmax = nscmax_;

        for(size_t i=0; i<targets.size(); i++){
        	SymbolicSet* tmp = new SymbolicSet();
        	ssTargets.push_back(tmp);
        	ssTargets[i]->LoadFromFile(*pCuddManager, targets[i].c_str() ,0);
        }

		for(size_t i=0; i<conts.size(); i++){
			ncsController* tmp = new ncsController(*pCuddManager, conts[i].c_str());
			controllers.push_back(tmp);
		}
	}

	~ncsFIFOController(){
		for(size_t i=0; i<ssTargets.size(); i++)
			delete ssTargets[i];

		for(size_t i=0; i<controllers.size(); i++)
			delete controllers[i];
	}

    std::vector<std::vector<double>> getControlInput(std::vector<double> XU_values){
        std::vector<std::vector<double>> inputs;
        std::vector<int> q_values;

        for(size_t i=0; i<nscmax; i++)
            q_values.push_back(0);

        inputs = controllers[mode]->getInputs(XU_values, q_values);

        return inputs;

    }

	vector<vector<double>> getInputs(ncsFIFOEstimator& T, int newmode=-1){
		if(newmode == -1)
		if(ssTargets[mode]->isElement(T.getCurrentSystemState())){
			cout << "------------------------------------------------------------------" << endl;
			cout << "INFO: Target #" << mode << " is achieved .. moved to next target !" << endl;
			cout << "------------------------------------------------------------------" << endl;

			mode = (mode+1)%ssTargets.size();
		}

		if(newmode != -1)
			mode = newmode;

		vector<vector<double>> NCS_state_values;
	    	vector<double> NCS_state;
		NCS_state_values = T.getNCSStateValues();
		NCS_state = VectorManager::UnrollVectors(NCS_state_values);

		return getControlInput(NCS_state);
	}
};

#endif
