/*
 * Misc.hh
 *
 *  Created on: Sep 8, 2016
 *      Author: mk
 *
 *  Yet another usefull routines !
 */
#ifndef UTILS_HH_
#define UTILS_HH_

#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <sys/stat.h>
#include "cuddObj.hh"
#include "dddmp.h"
#include "CuddMintermIterator.hh"


using namespace std;

/*********************************************************
 * File Operations !
 *********************************************************/
class IOUtils{
	
public:
	
	/* checks for the existance of a file*/
	static
	inline
	bool file_exists (const std::string& name) {
	  struct stat buffer;
	  return (stat (name.c_str(), &buffer) == 0);
	}
	
	static
	string GetFileName(const string str) {
	  size_t found = str.find_last_of("/\\");
	  if(string::npos != found)
		return str.substr(found+1);
	  else
		return str;
	}
};

/*********************************************************
 * Vector and Arrays Operations !
 *********************************************************/
class VectorManager{
	
public:
	/* constructs a one vector from a set of vectors by appending */
	template<typename T>
	static
	inline
	vector<T> UnrollVectors(const vector<vector<T>>& a){
		vector<T> v;
		for(size_t i=0; i<a.size(); i++)
			VectorManager::AppendVector(v,a[i]);
		return v;
	}

	/* removes the values of one vector from other  one*/
	template<typename T>
	static	
	inline
	void SubtractVector(vector<T>& a, const vector<T>& b){
		for(size_t i=0; i<b.size(); i++)
			a.erase(remove(a.begin(), a.end(), b[i]), a.end());
	}
	
	/* Appends vector's elements to another vector*/
	template<typename T>
	static	
	inline
	void AppendVector(vector<T>& a, const vector<T>& b){
		a.reserve(a.size() + b.size());
		a.insert(a.end(), b.begin(), b.end());
	}
	
	template<typename T>
	static	
	inline
	void AppendVectors(vector<T>& a, const vector<vector<T>>& b){
		for(size_t i=0; i<b.size(); i++)
			AppendVector(a, b[i]);
	}
	

	/* removes the elements in vector (a) not exsiting in the vector (b) */
	template<typename T>
	static
	inline
	void IntersectVector(vector<T>& a, const vector<T>& b){
		vector<size_t> marked4Removal;
		for(size_t i=0; i<a.size(); i++)
			if(!isVectorElement(b, a[i]))
				marked4Removal.push_back(i);

		for(int i=marked4Removal.size()-1; i>=0; i--)
			a.erase(a.begin()+marked4Removal[i]);
	}

	template<typename T>
	static	
	inline
	vector<T> ArrayToVector(const T* A, size_t size){
		vector<T> v(size);
		for (size_t i=0; i<size; i++)
			v[i]=A[i];
		return v;
	}
	
	/* Prints a vector to std-out or other ostream*/
	template<typename T>
	static	
	inline
	void PrintVector(const vector<T>& v, char separator = ',', bool do_endl=true, ostream& strm=cout){
		size_t size = v.size();
		for(size_t i=0; i<size; i++){
			strm << v[i];
			if(i < size-1 && separator != 'X') strm << separator;
		}
		if(do_endl)
			strm << endl;
	}
	
	template<typename T>
	static	
	inline
	void PrintArray(const T* v, size_t size, char separator = ',', bool do_endl=true, ostream& strm=cout){
		for (size_t i=0; i<size; i++)
		{
			strm << v[i];
			if(i<size-1 && separator != 'X') strm << separator;
		}
		if(do_endl)
			strm << endl;
	}
	
	/* checks whether an element is in the vector*/
	template<typename T>
	static	
	inline
	bool isVectorElement(const vector<T>& a, const T& e){
		if ((find(a.begin(),a.end(),e) != a.end()))
			return true;
		else
			return false;
	}
	
	/* checks whether a vector is included in another vector*/
	template<typename T>
	static	
	inline
	bool isVectorIncluded(const vector<T>& a, const vector<T>& b){
		for(size_t i=0; i<a.size(); i++)
			if (!isVectorElement(b,a[i]))
				return false;
	
		return true;
	}
	
	/* checks whether the vector intersects with another vector*/
	template<typename T>
	static	
	inline
	bool isVectorIntersects(const vector<T>& a, const vector<T>& b){
		for(size_t i=0; i<a.size(); i++)
			if (isVectorElement(b,a[i]))
				return true;
	
		return false;
	}
	
	/* checks whether a vector has has same elements of another vector*/
	template<typename T>
	static	
	inline
	bool isVectorEquivilant(const vector<T>& a, const vector<T>& b){
	
		if(isVectorIncluded(a,b) && isVectorIncluded(b,a))
			return true;
		else
			return false;
	}
};

/*********************************************************
 * Quantization/Dequantization
 *********************************************************/
class Quantizer{

public:	
	/*
	 * function: quantize
	 * quantizes a value to the (nearest) quantization level ! */
	static
	inline
	size_t quantize(const double value, const double q, const double zero_level, const double max_level){
	
		if(value < zero_level || value > max_level)
				throw runtime_error("Error::quantize:: Value below zero quantization level or exceeds maximum quantization level !");
	
		size_t q_idx=llround((value-zero_level)/q);
		return q_idx;
	}
	
	static
	inline
	double dequantize(const size_t q_idx, const double q, const double zero_level, const double max_level){
		double value = zero_level + (q*(double)q_idx);
	
		if(value > max_level){
			stringstream ss;
			ss << "Error::dequantize:: quantization level " << q_idx << " with q=" << q << ", when dequantized, exceeds the max level " << max_level; 
			throw runtime_error(ss.str().c_str());
		}
	
		return value;
	}
	
	/*
	 * function: decode
	 * Given a vector of binary values (lsb at index 0) and the quantization param q and zero_level,
	 * the value is computed. max_level is only used for checking !*/
	static
	inline
	double decode(const vector<int>& binary, const double q, const double zero_level, const double max_level){
		size_t q_idx=0;
	
		for(size_t mul=1, i=0; i<binary.size(); mul*=2, i++)
			q_idx+=binary[i]*mul;
	
		return dequantize(q_idx, q, zero_level, max_level);
	}
	
	/*
	 * function: encode
	 * Given a value and the quantization param q and zero_level/max_level,
	 * the binary vector (lsb at index 0) that symbolizes the value is computed.
	 * max_level is only used for checking !*/
	static
	inline
	vector<int> encode(const double value, const double q, const double zero_level, const double max_level, const size_t nbits){
	
		size_t q_symbol=quantize(value, q, zero_level, max_level);
	
		vector<int> binary;
		for (; q_symbol; q_symbol/=2)
			binary.push_back(0+q_symbol%2);
	
		size_t to_add = nbits-binary.size();
		for(size_t i=0; i<to_add; i++)
			binary.push_back(0);
	
		return binary;
	}
};


/*********************************************************
 * BDD Operations !
 *********************************************************/
class BDDUtils{

public:	

	/* returns an empty permutation map ready for manipulation */
	static
	vector<int> GetReadyPermuteMap(const Cudd& cuddManager){
		size_t nAllBddVars = cuddManager.ReadSize();
		vector<int> permute(nAllBddVars);
		for(size_t i=0; i<nAllBddVars; i++) permute[i]=i;
		return permute;
	}

	/* returns a BDD that is a projection of the supplied BDD to the supplied BddVars,
	 * all other BDD vars are treated as dont-cares*/
	static
	BDD ProjectBDD(const Cudd& cuddManager, const BDD& srcBDD, vector<size_t> projVars){
		size_t nAllBddVars = cuddManager.ReadSize();
		vector<BDD> otherBDDs;

		for(size_t i=0; i<nAllBddVars; i++)
			if (!(find(projVars.begin(),projVars.end(),i) != projVars.end()))
				otherBDDs.push_back(cuddManager.bddVar(i));

		BDD otherBdDDsCube = cuddManager.bddComputeCube(otherBDDs.data(), NULL, otherBDDs.size());
		return srcBDD.ExistAbstract(otherBdDDsCube);
	}

	static
	BDD ConstructOnesAnd(const Cudd& cuddManager, vector<size_t> bddVars){
		BDD ret = cuddManager.bddOne();

		for(size_t i=0; i<bddVars.size(); i++)
			ret *= cuddManager.bddVar(bddVars[i]);

		return ret;
	}

	static
	BDD ConstructZerosAnd(const Cudd& cuddManager, vector<size_t> bddVars){
		BDD ret = cuddManager.bddOne();

		for(size_t i=0; i<bddVars.size(); i++)
			ret *= !cuddManager.bddVar(bddVars[i]);

		return ret;
	}



	/* Prints the information within a BDD object to std-out*/
	static
	void PrintBDD(string NameIt, const BDD& bddObj){
		cout << "Info for BDD: " << NameIt << endl;
		//cout << "\t    f : (" << bddObj << ")" << endl;
		cout << "       cover: " << endl;
		bddObj.PrintCover();
		cout << endl;
	}
	
	/* function: BddAsMap
	 * treats the BDD (func) as a map asking for the output y by the input x where,
	 * x belongs to the domain and y belongs to the codomain. Input x is supplied
	 * as vector of 0s and 1s representing the phase of x. The output(s) is(are) saved in
	 * yPhases as a group of vectors containing the outputs y whose input is x.
	 *
	 * User should supply:
	 * 1- The Cudd manager in (cuddManager) and the Bdd function in (unc)
	 * 2- All bddVars of the func (this includes bddVars of the domain and codomain) in (funcVars)
	 * 3- All domain bddVars  in (xVars).
	 * 4- the binarvalue of x in (xPhase).
	 *
	 * Note:
	 * The codomain bddVars are those found by subtracting the set xVars from funcVars.
	 *
	 * The function then uses the BDD object to construct a projection based on the value of x
	 * by ANDING the x with the func. Now the projection has a fixed part which corresponds to
	 * the domain. We then iterate over all cubes of the projection and extracts the values
	 * of outputs.
	 * */
	static
	vector<vector<int>>
	BddAsMap(const Cudd& cuddManager,const BDD& func, const vector<size_t>& funcVars,
									 const vector<size_t>& xVars, vector<int>& xPhase){
	
		if(xVars.size() != xPhase.size())
			throw runtime_error("Error::BddAsMap:: Mismatch in sizes of vectors passed !.");
	
		vector<vector<int>> yPhases;
		vector<BDD> xBdds;
	
		// Constructing the codomain, yVars
		vector<size_t> yVars;
		for(size_t i=0; i<funcVars.size(); i++)
			if (!(find(xVars.begin(),xVars.end(),funcVars[i]) != xVars.end()))
				yVars.push_back(funcVars[i]);
	
	
		// Constructing the projection
		for(size_t i=0; i< xVars.size(); i++)
			xBdds.push_back(cuddManager.bddVar(xVars[i]));
	
		BDD x_cube = cuddManager.bddComputeCube(xBdds.data(),xPhase.data(),xPhase.size());
		BDD func_proj = x_cube*func;
	
		// for each cube in proj
		CuddMintermIterator  it(func_proj,funcVars,funcVars.size());
		for(; !it.done(); ++it) {
			const int* minterm_array = it.currentMinterm();
			vector<int> y_cube;
	
			for(size_t i=0; i<yVars.size(); i++)
				y_cube.push_back(minterm_array[yVars[i]]);
	
			yPhases.push_back(y_cube);
		}
	
		return yPhases;
	}
	
	
	/*
	 * This function returns a BDD that represents a logical function for which two values are
	 * synchronized. Synchronization means that they will have always same values regardless of
	 * other bddVars in the manager. The synchronization of two bddVars is simply the function:
	 *
	 * 		f_sync : (x1 & x2 | !x1 & !x2)
	 *
	 * 	, where we have a manager with bddVars 0,1,2, ..... and we sync the bddVars 1 and 2.
	 * 	The cover will then look like:
	 * 				-11---------- (1)
	 *				-00---------- (1)
	 *
	 *	The function does this for all bddVars in the two values (vectors) v1 and v2.
	 * */
	static
	BDD BddSyncValues(const Cudd& cuddManager, const vector<size_t>& v1, const vector<size_t>& v2){
	
		if((v1.size() != v2.size()) || v1.empty() || v2.empty() )
			throw runtime_error("Error::BddSyncValues:: Vectors of bddVars are empty or have different sizes !!");
	
	
		BDD ret = cuddManager.bddOne();
		for(size_t i=0; i<v1.size(); i++){
			BDD tmp = cuddManager.bddVar(v1[i])&cuddManager.bddVar(v2[i]);
			tmp|= (!cuddManager.bddVar(v1[i]))&(!cuddManager.bddVar(v2[i]));
	
			ret&=tmp;
		}
	
		return ret;
	
	}
	
	/*
	 * This function synchronizes a group of pairs of values scattered over two vectors.
	 * */
	static
	BDD BddSyncMultiValues(Cudd& cuddManager, const vector<vector<size_t>>& vv1, const vector<vector<size_t>>& vv2){
	
		if((vv1.size() != vv2.size()) || vv1.empty() || vv2.empty() )
				throw runtime_error("Error::BddSyncMultiValues:: vectors of vectors of bddVars are empty or have different sizes !!");
	
		BDD ret = cuddManager.bddOne();
		for(size_t i=0; i<vv1.size(); i++){
			BDD tmp = BddSyncValues(cuddManager, vv1[i],vv2[i]);
	
			ret *= tmp;
		}
	
		return ret;
	
	}
	
	/* function:  writeToFile
	   * writes a bdd with some information to a file*/
	static
	void writeToFile(BDD& func,
						const size_t ssdim, const size_t isdim,
						const size_t* ssVarsCount, const size_t* isVarsCount,
						const double* ssEta, const double* ssLb, const double* ssUb,
						const double* isEta, const double* isLb, const double* isUb,
						const size_t nscmin, const size_t nscmax, const size_t ncamin, const size_t ncamax,
						const vector<size_t>& preVars, const vector<size_t>& inpVars, const vector<size_t>& postVars,
						const vector<size_t>& qVars,
						const char* Remarks,
						const char* fname) {
	
		/*
		 * 1] Meta-Data for NCS toolbox
		 */
	
		/* stringstream to manager the meta-data */
		std::stringstream metadata;
		char sep = ' ';
	
		metadata << "#NCS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
		metadata << "#NCS NCS Model Construction and Controller Synthesis Toolbox %%" << endl;
		metadata << "#NCS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
		metadata << "#NCS ssDim:" << ssdim << endl;
		metadata << "#NCS ssEta:"; VectorManager::PrintArray(ssEta, ssdim, sep, true, metadata);
		metadata << "#NCS ssLb:"; VectorManager::PrintArray(ssLb, ssdim, sep, true, metadata);
		metadata << "#NCS ssUb:"; VectorManager::PrintArray(ssUb, ssdim, sep, true, metadata);
		metadata << "#NCS ssVarsCount:"; VectorManager::PrintArray(ssVarsCount, ssdim, sep, true, metadata);
	
		metadata << "#NCS isDim:" << isdim << endl;
		metadata << "#NCS isEta:"; VectorManager::PrintArray(isEta, isdim, sep, true, metadata);
		metadata << "#NCS isLb:"; VectorManager::PrintArray(isLb, isdim, sep, true, metadata);
		metadata << "#NCS isUb:"; VectorManager::PrintArray(isUb, isdim, sep, true, metadata);
		metadata << "#NCS isVarsCount:"; VectorManager::PrintArray(isVarsCount, isdim, sep, true, metadata);
	
		metadata << "#NCS nscMin:" << nscmin << endl;
		metadata << "#NCS nscMax:" << nscmax << endl;
		metadata << "#NCS ncaMin:" << ncamin << endl;
		metadata << "#NCS ncaMax:" << ncamax << endl;
	
		metadata << "#NCS preVars:"; VectorManager::PrintVector(preVars, sep, true, metadata);
		metadata << "#NCS inpVars:"; VectorManager::PrintVector(inpVars, sep, true, metadata);
		metadata << "#NCS postVars:"; VectorManager::PrintVector(postVars, sep, true, metadata);
		metadata << "#NCS qVars:"; VectorManager::PrintVector(qVars, sep, true, metadata);
	
		metadata << "#NCS Remarks:" << Remarks << endl;


		FILE *file = fopen (fname,"w");
		if (file == NULL){
			ostringstream os;
			os << "Error::writeToFile:: Unable to open the file: " << fname << "'.";
			throw runtime_error(os.str().c_str());
		}
		if (metadata.str().size() != fwrite(metadata.str().c_str(),1,metadata.str().size(),file)) {
			throw "Error::writeToFile:: Failed to save meta-data to the file.";
		}
	
		/*
		 * 2] BDD Object
		 */
		/* transfer the BDD function to another manager and save */
		//Cudd newManager;
		//BDD newFunc = func.Transfer(newManager);
		//PrintBDD("newFunc", newFunc);
		int storeReturnValue = Dddmp_cuddBddStore(
		  //newManager.getManager(),
		  func.manager(),
		  NULL,
		  //newFunc.getNode(),
		  func.getNode(),
		  NULL,
		  NULL,
		  DDDMP_MODE_BINARY,
		  DDDMP_VARIDS,
		  NULL,
		  file
		);
	
		fclose(file);
		if (storeReturnValue != DDDMP_SUCCESS)
		  throw "Error:: Failed to write the BDD object to the file.";
	  }
	
	static
	void readFromFile(const char* fname, Cudd& cuddManager,
					BDD& func,
					size_t& ssdim, size_t& isdim,
					vector<size_t>& ssVarsCount, vector<size_t>& isVarsCount,
					vector<double>& ssEta, vector<double>& ssLb, vector<double>& ssUb,
					vector<double>& isEta, vector<double>& isLb, vector<double>& isUb,
					size_t& nscmin, size_t& nscmax, size_t& ncamin, size_t& ncamax,
					vector<size_t>& preVars, vector<size_t>& inpVars, vector<size_t>& postVars,
					vector<size_t>& qVars,
					string& Remarks){
	
		if(!IOUtils::file_exists(fname))
			throw runtime_error("Error::readNbdd:: The requested file does not exist.");
	
		ifstream nbddfile(fname);
		if (!nbddfile.good()){
			ostringstream os;
			os << "Error::readNbdd:: Unable to open the file: " << fname << "'.";
			throw runtime_error(os.str().c_str());
		}
	
		/*
		 * 1] Read meta-data
		 */
		string line;
		bool meta_data_done = false;
		while(!meta_data_done) {
		  getline(nbddfile,line);
		  if (line.substr(0,4)=="#NCS") {
	
			istringstream ssValue(line.substr(line.find(":")+1));
			string sItem = line.substr(5, line.find(":")-5);
			size_t size_t_tmp;
			double double_tmp;
	
			if(sItem.size() == 0)
				continue;
	
			if(sItem == "ssDim"){
				ssValue >> ssdim;
			}
			else if(sItem == "ssEta"){
				for(size_t i=0; i<ssdim; i++){
					ssValue >> double_tmp;
					ssEta.push_back(double_tmp);
				}
			}
			else if(sItem == "ssLb"){
				for(size_t i=0; i<ssdim; i++){
					ssValue >> double_tmp;
					ssLb.push_back(double_tmp);
				}
			}
			else if(sItem == "ssUb"){
				for(size_t i=0; i<ssdim; i++){
					ssValue >> double_tmp;
					ssUb.push_back(double_tmp);
				}
			}
			else if(sItem == "ssVarsCount"){
				for(size_t i=0; i<ssdim; i++){
					ssValue >> size_t_tmp;
					ssVarsCount.push_back(size_t_tmp);
				}
			}
	
	
			else if(sItem == "isDim"){
				ssValue >> isdim;
			}
			else if(sItem == "isEta"){
				for(size_t i=0; i<isdim; i++){
					ssValue >> double_tmp;
					isEta.push_back(double_tmp);
				}
			}
			else if(sItem == "isLb"){
				for(size_t i=0; i<isdim; i++){
					ssValue >> double_tmp;
					isLb.push_back(double_tmp);
				}
			}
			else if(sItem == "isUb"){
				for(size_t i=0; i<isdim; i++){
					ssValue >> double_tmp;
					isUb.push_back(double_tmp);
				}
			}
			else if(sItem == "isVarsCount"){
				for(size_t i=0; i<isdim; i++){
					ssValue >> size_t_tmp;
					isVarsCount.push_back(size_t_tmp);
				}
			}
	
	
			else if(sItem == "nscMin"){
				ssValue >> nscmin;
			}
			else if(sItem == "nscMax"){
				ssValue >> nscmax;
			}
			else if(sItem == "ncaMin"){
				ssValue >> ncamin;
			}
			else if(sItem == "ncaMax"){
				ssValue >> ncamax;
			}
	
	
			else if(sItem == "preVars"){
				while(!ssValue.eof()) {
					ssValue >> size_t_tmp;
					preVars.push_back(size_t_tmp);
				}
			}
			else if(sItem == "inpVars"){
				while(!ssValue.eof()) {
					ssValue >> size_t_tmp;
					inpVars.push_back(size_t_tmp);
				}
			}
			else if(sItem == "postVars"){
				while(!ssValue.eof()) {
					ssValue >> size_t_tmp;
					postVars.push_back(size_t_tmp);
				}
			}
			else if(sItem == "qVars"){
				while(!ssValue.eof()) {
					ssValue >> size_t_tmp;
					qVars.push_back(size_t_tmp);
				}
			}
	
			else if(sItem == "Remarks"){
	
				Remarks = ssValue.str();
	
				/***************************************************************************************************/
				/**** THIS IS THE LAST METADATA ENTRY AND WE SHOULD NOT CONTINUE AS THERE WILL COMBE THE BDD OBJ***/
				/***************************************************************************************************/
				meta_data_done = true;
			}
	
	
		  }
		  else
		  {
			throw std::runtime_error("Error::readNbdd:: Seems to have a corrupt file !");
		  }
		}
	
		nbddfile.close();
	
		/*
		 * 2] Read BDD object
		 */
		FILE *file = fopen (fname,"r");
		if (file == NULL) {
			ostringstream os;
			os << "Error: Unable to open file:" << fname << "'.";
			throw std::runtime_error(os.str().c_str());
		}
	
		DdNode *bdd= Dddmp_cuddBddLoad( cuddManager.getManager(),\
										DDDMP_VAR_MATCHIDS,\
										NULL,\
										NULL,\
										NULL,\
										DDDMP_MODE_BINARY,\
										NULL,\
										file);
		fclose(file);
		BDD tmp(cuddManager, bdd);
		func=tmp;
	
	}	
};

/*********************************************************
 * BDD Operations !
 *********************************************************/

class StopWatch{
private:
    static chrono::high_resolution_clock::time_point start;
    static chrono::high_resolution_clock::time_point stop;
public:

    static
    inline void Start(void) {
      start=chrono::high_resolution_clock::now();
    }

    static
    inline double Stop(bool printTime = true) {
      stop=chrono::high_resolution_clock::now();
      chrono::duration<double> dt;
      dt=chrono::duration_cast<std::chrono::duration<double > >(stop-start);

      double duration = dt.count();
      if(printTime)
    	  std::cout << "Elapsed time is " << duration << " seconds." << std::endl;

      return duration;
    }

};

chrono::high_resolution_clock::time_point StopWatch::start;
chrono::high_resolution_clock::time_point StopWatch::stop;


#endif /* UTILS_HH_ */

