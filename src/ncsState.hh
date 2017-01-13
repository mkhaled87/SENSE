/*
 * ncsState.hh
 *
 *  created on: 17.11.2016
 *      author: M.Khaled
 */

#ifndef NCSSTATE_HH_
#define NCSSTATE_HH_

#include <vector>
#include "cuddObj.hh"
#include "Misc.hh"

using namespace std;

class ncsState {
protected:
	/* vars: nscmax, ncamax, nscmin, ncamin
	 * bddVar max delays of both channels */
	size_t nscmax, ncamax;
	size_t nscmin, ncamin;

	/* vars: ssdim, isdim
	 * dimensions of X and U */
	size_t ssdim, isdim;
	
	/* vars:
	 * quantization params of X and U */
	vector<double> ssEta, ssLb, ssUb;
	vector<double> isEta, isLb, isUb;

	/* vars: ssVarsCount, isVarsCount
	 * no of bddVar for X and U for each dimension */
	vector<size_t> ssVarsCount, isVarsCount;

	/* var: x1xN_bddVars
	 * bddVar indicies for x1 to xn */
	vector<vector<size_t>> x1xn_bddVars;
		
	/* var: u1xM_bddVars
	 * bddVar indicies for u1 to um */
	vector<vector<size_t>> u1um_bddVars;
	
	/* var: nsc1nscN_bddVars
	 * bddVar indicies for Nsc1 to NscN */
	vector<vector<size_t>> nsc1nscn_bddVars;

	/* var: nca1ncaM_bddVars
	 * bddVar indicies for Nca1 to NcaM */
	vector<vector<size_t>> nca1ncam_bddVars;

	/* var: q_bddVars 
	 * bddVar indicies for q1 to qn (one bddVar for each q)*/
	vector<size_t> q_bddVars;

	size_t bddVarsCount;

public:
	ncsState(){
		nscmax=0;
		ncamax=0;
		nscmin=0;
		ncamin=0;

		ssdim = 0;
		isdim = 0;
	}
	virtual ~ncsState(){

	}


	virtual BDD getCube(Cudd& cuddManager, vector<double>& XU_values, vector<int>& q_values)=0;


	size_t getBddVarsCount() const {
		return bddVarsCount;
	}

	void setBddVarsCount(size_t bddVarsCount) {
		this->bddVarsCount = bddVarsCount;
	}

	size_t getIsdim() const {
		return isdim;
	}

	void setIsdim(size_t isdim) {
		this->isdim = isdim;
	}

	const vector<double>& getIsEta() const {
		return isEta;
	}

	void setIsEta(const vector<double>& isEta) {
		this->isEta = isEta;
	}

	const vector<double>& getIsLb() const {
		return isLb;
	}

	void setIsLb(const vector<double>& isLb) {
		this->isLb = isLb;
	}

	const vector<double>& getIsUb() const {
		return isUb;
	}

	void setIsUb(const vector<double>& isUb) {
		this->isUb = isUb;
	}

	const vector<size_t>& getIsVarsCount() const {
		return isVarsCount;
	}

	void setIsVarsCount(const vector<size_t>& isVarsCount) {
		this->isVarsCount = isVarsCount;
	}

	const vector<vector<size_t> >& getNca1ncamBddVars() const {
		return nca1ncam_bddVars;
	}

	void setNca1ncamBddVars(const vector<vector<size_t> >& nca1ncamBddVars) {
		nca1ncam_bddVars = nca1ncamBddVars;
	}

	size_t getNcamax() const {
		return ncamax;
	}

	void setNcamax(size_t ncamax) {
		this->ncamax = ncamax;
	}

	size_t getNcamin() const {
		return ncamin;
	}

	void setNcamin(size_t ncamin) {
		this->ncamin = ncamin;
	}

	const vector<vector<size_t> >& getNsc1nscnBddVars() const {
		return nsc1nscn_bddVars;
	}

	void setNsc1nscnBddVars(const vector<vector<size_t> >& nsc1nscnBddVars) {
		nsc1nscn_bddVars = nsc1nscnBddVars;
	}

	size_t getNscmax() const {		
		return nscmax;
	}

	void setNscmax(size_t nscmax) {
		this->nscmax = nscmax;
	}

	size_t getNscmin() const {
		return nscmin;
	}

	void setNscmin(size_t nscmin) {
		this->nscmin = nscmin;
	}

	const vector<size_t>& getqBddVars() const {
		return q_bddVars;
	}

	void setBddVars(const vector<size_t>& bddVars) {
		q_bddVars = bddVars;
	}

	size_t getSsdim() const {
		return ssdim;
	}

	void setSsdim(size_t ssdim) {
		this->ssdim = ssdim;
	}

	const vector<double>& getSsEta() const {
		return ssEta;
	}

	void setSsEta(const vector<double>& ssEta) {
		this->ssEta = ssEta;
	}

	const vector<double>& getSsLb() const {
		return ssLb;
	}

	void setSsLb(const vector<double>& ssLb) {
		this->ssLb = ssLb;
	}

	const vector<double>& getSsUb() const {
		return ssUb;
	}

	void setSsUb(const vector<double>& ssUb) {
		this->ssUb = ssUb;
	}

	const vector<size_t>& getSsVarsCount() const {
		return ssVarsCount;
	}

	void setSsVarsCount(const vector<size_t>& ssVarsCount) {
		this->ssVarsCount = ssVarsCount;
	}

	const vector<vector<size_t> >& getU1umBddVars() const {
		return u1um_bddVars;
	}

	void setU1umBddVars(const vector<vector<size_t> >& u1umBddVars) {
		u1um_bddVars = u1umBddVars;
	}

	const vector<vector<size_t> >& getX1xnBddVars() const {
		return x1xn_bddVars;
	}

	void setX1xnBddVars(const vector<vector<size_t> >& x1xnBddVars) {
		x1xn_bddVars = x1xnBddVars;
	}

	size_t getVarsCount(){
		return
		VectorManager::UnrollVectors(x1xn_bddVars).size() +
		VectorManager::UnrollVectors(u1um_bddVars).size() +
		q_bddVars.size();
	}

	virtual vector<size_t> getBddVars()=0;
	virtual vector<size_t> getQXUVarsOrganized()=0;
	virtual string getStateTemplateText()=0;
};

class ncsFifoState : public ncsState {
public:
	ncsFifoState(){
	}
	~ncsFifoState(){
	}
	ncsFifoState(size_t nscmax_, size_t ncamax_,
			size_t ssdim_, size_t isdim_,
			vector<double>& ssEta_, vector<double>& ssLb_, vector<double>& ssUb_,
			vector<double>& isEta_, vector<double>& isLb_, vector<double>& isUb_,
			vector<size_t>& ssVarsCount_, vector<size_t>& isVarsCount_,
			vector<vector<size_t>>& x1xn, vector<vector<size_t>>& u1um, vector<size_t>& q_bddVars_){

		if(ssdim_ != ssEta_.size() || ssdim_ != ssLb_.size() || ssdim_ != ssUb_.size())
			throw runtime_error("Error::ncsFifo:: Size mismatch for X quantization params! ");

		if(isdim_ != isEta_.size() || isdim_ != isLb_.size() || isdim_ != isUb_.size())
			throw runtime_error("Error::ncsFifo:: Size mismatch for X quantization params! ");

		if(ssdim_ != ssVarsCount_.size())
			throw runtime_error("Error::ncsFifo:: Size mismatch for ssVarsCount_! ");

		if(isdim_ != isVarsCount_.size())
			throw runtime_error("Error::ncsFifo:: Size mismatch for isVarsCount_! ");

		bddVarsCount = 0;
		for(size_t i=0; i<nscmax_; i++)
			for(size_t j=0; j<ssdim_; j++)
				for(size_t k=0; k<ssVarsCount_[j]; k++)
					bddVarsCount++;

		for(size_t i=0; i<ncamax_; i++)
			for(size_t j=0; j<isdim_; j++)
				for(size_t k=0; k<isVarsCount_[j]; k++)
					bddVarsCount++;



		if((VectorManager::UnrollVectors(x1xn).size() + VectorManager::UnrollVectors(u1um).size()) != bddVarsCount)
			throw runtime_error("Error::ncsFifo::getBinary:: Size mismatch for XU_bddVars! ");

		if(q_bddVars_.size() != nscmax_)
			throw runtime_error("Error::ncsFifo::getBinary:: Size mismatch for q_bddVars_! ");

		nscmax = nscmax_; 
		ncamax = ncamax_;

		nscmin=nscmax_;
		ncamin=ncamax_;

		ssdim = ssdim_;
		isdim = isdim_;
		ssVarsCount = ssVarsCount_;
		isVarsCount = isVarsCount_;
		
		ssEta = ssEta_;
		ssLb  = ssLb_;
		ssUb  = ssUb_;
		isEta = isEta_;
		isLb  = isLb_;
		isUb  = isUb_;


		x1xn_bddVars = x1xn;
		u1um_bddVars = u1um;
		
		q_bddVars = q_bddVars_;
		bddVarsCount += q_bddVars_.size();
	}

	vector<int> getBinaryXUQ(vector<double>& XU_values, vector<int>& q_values){

		if(XU_values.size() != (nscmax*ssdim + ncamax*isdim))
			throw runtime_error("Error::ncsFifo::getBinaryXUQ:: Size mismatch for XU_values! ");

		if(q_values.size() != nscmax)
			throw runtime_error("Error::ncsFifo::getBinaryXUQ:: Size mismatch for q_values! ");

		vector<int> binary;
		size_t c=0;

		// for Xs
		for(size_t i=0; i<nscmax; i++){
			for(size_t j=0; j<ssdim; j++){

				if(q_values[i] == 0){
					double x_value = XU_values[c];
					vector<int> x_encoded = Quantizer::encode(x_value, ssEta[j], ssLb[j], ssUb[j], ssVarsCount[j]);

					if(x_encoded.size() != ssVarsCount[j])
						throw runtime_error("Error::ncsFifo::getBinaryXUQ:: Error while encoding x value ! ");

					for(size_t k=0; k<x_encoded.size(); k++)
						binary.push_back(x_encoded[k]);
				}
				else{
					for(size_t k=0; k<ssVarsCount[j]; k++)
						binary.push_back(0);
				}
				c++;
			}
		}


		// for Us
		for(size_t i=0; i<ncamax; i++){
			for(size_t j=0; j<isdim; j++){

				double u_value = XU_values[c];
				vector<int> u_encoded = Quantizer::encode(u_value, isEta[j], isLb[j], isUb[j], isVarsCount[j]);

				if(u_encoded.size() != isVarsCount[j])
					throw runtime_error("Error::ncsFifo::getBinaryXUQ:: Error while encoding x value ! ");

				for(size_t k=0; k<u_encoded.size(); k++)
					binary.push_back(u_encoded[k]);

				c++;
			}
		}

		for(size_t i=0; i<q_values.size(); i++)
			binary.push_back(q_values[i]);

		return binary;		
	}
	
	BDD getCube(Cudd& cuddManager, vector<double>& XU_values, vector<int>& q_values){

		vector<int> XUQ_binary = getBinaryXUQ(XU_values, q_values);
		vector<size_t> XUQ_bddVars;

		VectorManager::AppendVectors(XUQ_bddVars, x1xn_bddVars);
		VectorManager::AppendVectors(XUQ_bddVars, u1um_bddVars);
		VectorManager::AppendVector(XUQ_bddVars, q_bddVars);

		vector<BDD> XUQ_BDDs;
		for(size_t i=0; i<XUQ_bddVars.size(); i++)
			XUQ_BDDs.push_back(cuddManager.bddVar(XUQ_bddVars[i]));

		BDD state = cuddManager.bddComputeCube(XUQ_BDDs.data(),XUQ_binary.data(),XUQ_bddVars.size());
		return state;
	}

	vector<double> getValueXUQ(vector<int> XUQ_binary){
		if(XUQ_binary.size() != bddVarsCount)
			throw runtime_error("Error::ncsFifo::getValueXUQ:: Size mismatch for XUQ_binary! ");

		// TODO :: prepare for decoding of each x and each u taking care of qs !
		// TODO :: decore and fill the value vecor
		throw runtime_error("Error::ncsFifo::getValueXUQ:: Not Implemented ! ");

		vector<double> value;
		return value;
	}

	vector<double> getValueXUQ(int* XUQ_binary_array, size_t size){
		vector<int> XUQ_binary(size);

		for(size_t i=0; i<size; i++)
			XUQ_binary[i] = XUQ_binary_array[i];

		return getValueXUQ(XUQ_binary);
	}


	vector<size_t> getBddVars(){
		vector<size_t> bddVars;

		VectorManager::AppendVectors(bddVars, x1xn_bddVars);
		VectorManager::AppendVectors(bddVars, u1um_bddVars);
		VectorManager::AppendVector(bddVars, q_bddVars);

		return bddVars;
	}

	vector<size_t> getQXUVarsOrganized(){
		vector<size_t> vars;
		for(size_t i=0; i<nscmax; i++){
			vars.push_back(q_bddVars[i]);
			for(size_t j=0; j<x1xn_bddVars[i].size(); j++)
				vars.push_back(x1xn_bddVars[i][j]);
		}
		for(size_t i=0; i<ncamax; i++)
			for(size_t j=0; j<u1um_bddVars[i].size(); j++)
				vars.push_back(u1um_bddVars[i][j]);

		return vars;
	}

	string getStateTemplateText(){
		string t="";
		for(size_t i=0; i<nscmax; i++){
			t += "q";
			for(size_t j=0; j<x1xn_bddVars[i].size(); j++)
				t += "x";
		}
		for(size_t i=0; i<ncamax; i++){
			for(size_t j=0; j<u1um_bddVars[i].size(); j++)
				t += "u";
		}
		return t;
	}
			
	
}; /* class ncsState */

#endif /* NCSSTATE_HH_ */
