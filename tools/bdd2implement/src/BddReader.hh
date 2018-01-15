#ifndef BDDReader_HH_
#define BDDReader_HH_

#include <vector>
#include <algorithm>
#include <sstream>
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdexcept>

#include "dddmp.h"
#include "cuddObj.hh"

using namespace std;

enum BDD_FILE_TYPE{
	SCOTS_BDD,
	SENSE_NBDD
};


class SCOTSBDDReader {
private:
  /* var: ddmgr_  
   * the binary decision diagram manager */
	Cudd *ddmgr_;
  /* var: dim_ 
   * dimension of the real space */
  size_t dim_;
  /* var: eta_
   * dim_-dimensional vector containing the grid node distances */
  double* eta_;
  /* var: z_
   * dim_-dimensional vector containing the measurement error bound */
  double* z_;
  /* var: firstGridPoint_
   * dim_-dimensinal vector containing the real values of the first grid point */
  double* firstGridPoint_;
  /* var: firstGridPoint_
   * dim_-dimensinal vector containing the real values of the last grid point */
  double* lastGridPoint_;
  /* var: nofGridPoints_
   * integer array[dim_] containing the grid points in each dimension */
  size_t* nofGridPoints_;

  /* var: nofBddVars_ 
   * integer array[dim_] containing the number of bdd variables in each dimension */
  size_t *nofBddVars_;
  /* var: indBddVars_ 
   * 2D integer array[dim_][nofBddVars_] containing the indices (=IDs) of the bdd variables */
  size_t **indBddVars_;
  /* var: nvars_ 
   * total number of bdd variables representing the support of the set */
  size_t nvars_;
  /* var: bddFunction 
   * the bdd representing the set of points */
  BDD bddFunction;
public:
  /* constructor: SCOTSBDDReader :: default, added by MK 01.09/.2016
   */
  SCOTSBDDReader(){
  }


  void LoadFromFile(Cudd &ddmgr, const char *filename, int newID=0) {
    ddmgr_=&ddmgr;
    /* read the BDD info members from file */
    readMembersFromFile(filename);
    int* composeids=NULL;
    Dddmp_VarMatchType match=DDDMP_VAR_MATCHIDS;
    /* do we need to create new variables ? */
    if(newID) {
      /* we have to create new variable id's and load the bdd with those new ids */
      match=DDDMP_VAR_COMPOSEIDS;
      /* allocate memory for comopsids */
      size_t maxoldid=0;
      for(size_t i=0; i<dim_; i++) 
        for(size_t j=0; j<nofBddVars_[i]; j++) 
          maxoldid=( (maxoldid < indBddVars_[i][j]) ? indBddVars_[i][j] :  maxoldid );
      composeids = new int[maxoldid+1];
      /* match old id's (read from file) with newly created ones */
      for(size_t i=0; i<dim_; i++) {
        for(size_t j=0; j<nofBddVars_[i]; j++) {
          BDD bdd=ddmgr.bddVar();
          composeids[indBddVars_[i][j]]=bdd.NodeReadIndex();
          indBddVars_[i][j]=bdd.NodeReadIndex();
        }
      }
    /* number of total variables */
    }
    /* load bdd */
    FILE *file = fopen (filename,"r");
    if (file == NULL) {
        ostringstream os;
        os << "Error: Unable to open file:" << filename << "'.";
        throw runtime_error(os.str().c_str());
    }
    DdNode *bdd= Dddmp_cuddBddLoad( ddmgr.getManager(),\
                                    match,\
                                    NULL,\
                                    NULL,\
                                    composeids,\
                                    DDDMP_MODE_BINARY,\
                                    NULL,\
                                    file);
    fclose(file);
    BDD tmp(ddmgr, bdd);
    bddFunction=tmp;

    delete[] composeids;
  }


  /* function: mintermToElement
   * compute the element of the symbolic set associated with the minterm */
  inline void mintermToElement(const int* minterm, double* element) const {
    for(size_t i=0; i<dim_; i++) {
      size_t idx=0;
      for(size_t c=1, j=0; j<nofBddVars_[i]; c*=2, j++)
        idx+=minterm[indBddVars_[i][j]]*c;
      element[i]=eta_[i]*idx+firstGridPoint_[i];
    }
  }

  /* function:  getDimension 
   *
   * get the dimension of the real space in which the points that are represented by
   * the symbolic set live in */
  inline size_t getDimension(void) const {
    return dim_;
  }

  /* function:  getBDD
   *
   * get the BDD that stores the function
   */
  inline BDD getBDD(void) const {
    return bddFunction;
  }  

  /* function:  getCube
   *
   * returns a BDD with ones in the domain of the SymbolicSet
   */
  inline BDD getCube(void) const {
    /* create cube */
    BDD* vars = new BDD[nvars_];
    for (size_t k=0, i=0; i<dim_; i++) {
      for (size_t j=0; j<nofBddVars_[i]; k++,j++) 
        vars[k]=ddmgr_->bddVar(indBddVars_[i][j]);
    }
    BDD cube=ddmgr_->bddComputeCube(vars,NULL,nvars_);
    delete[] vars;

    return cube; 
  }  

   /* function: setBDD
   *
   * set the Bdd set to symset
   */
  inline void setBDD(const BDD new_bdd) {
    bddFunction=new_bdd;
  }

  /* function:  getnofbddvars 
   *
   * get the pointer to the size_t array that contains the number of bdd
   * variables 
   */
  inline size_t* getNofBddVars(void) const {
    return nofBddVars_;
  }
  /* function:  getIndBddVars 
   *
   * get the pointer to the size_t array of indices of the bdd
   * variables 
   */
  inline size_t** getIndBddVars(void) const {
    return indBddVars_;
  }
  /* function:  setIndBddVars 
   *
   * set the indices of the bdd variables to the indices provided in 
   * size_t* newIndBddVars
   * NOTE: the size of pointer newIndBddVars needs to be dim_*nofBddVars_!
   */
  void setIndBddVars(size_t* newIndBddVars) {
    for(size_t i=0,k=0; i<dim_; i++) 
      for(size_t j=0; j<nofBddVars_[i]; k++,j++)
        indBddVars_[i][j]=newIndBddVars[k];
  }
  /* function:  getSize
   * get the number of grid points in the symbolic set */
  inline double getSize(void) {
    return bddFunction.CountMinterm(nvars_);
  }
  /* function:  getSize
   *
   * project the BDD on the dimensions specified in projectDimension and then
   * count the number of elements
   */
  inline double getSize(vector<size_t> projectDimension) {
    size_t n=projectDimension.size();
    if(n>dim_) {
      ostringstream os;
      os << "Error: SCOTSBDDReader::getSize(projectDimension): number of dimensions onto which the BDD should be projected exceeds dimension.";
      throw invalid_argument(os.str().c_str());
    }
    for(size_t i=0; i<n; i++) {
      if(projectDimension[i]>=dim_) {
        ostringstream os;
        os << "Error: SCOTSBDDReader::getSize(projectDimension): dimensions onto which the BDD should be projected exceeds dimension.";
        throw invalid_argument(os.str().c_str());
      }
    }
    /* compute complement dim */
    vector<size_t> codim;
    for(size_t i=0; i< dim_; i++) {
      if (!(find(projectDimension.begin(),projectDimension.end(),i) != projectDimension.end())) 
        codim.push_back(i);
    } 
    /* create cube to be used in the ExistAbstract command */
    size_t t=0;
    for (size_t i=0; i<codim.size(); i++) 
      t+=nofBddVars_[codim[i]];
    BDD* vars = new BDD[t];
    for (size_t k=0, i=0; i<codim.size(); i++) {
      for (size_t j=0; j<nofBddVars_[codim[i]]; k++,j++) 
        vars[k]=ddmgr_->bddVar(indBddVars_[codim[i]][j]);
    }
    BDD cube = ddmgr_->bddComputeCube(vars,NULL,t);
    BDD bdd = bddFunction.ExistAbstract(cube);
    delete[] vars;

    return bdd.CountMinterm(nvars_-t);
  }
  /* function: clear
   * empty the symbolic set*/
  inline void clear() {
    bddFunction = ddmgr_->bddZero();
  }

  /* function:  writeToFile
   * store the bdd with all information to file*/
  void writeToFile(const char *filename) {
    /* produce string with parameters to be written into file */
    stringstream dim;
    stringstream z;
    stringstream eta;
    stringstream first;
    stringstream last;
    stringstream nofGridPoints;
    stringstream indBddVars;

    dim <<"#scots dimension: " << dim_ << endl;
    z << "#scots measurement error bound: ";
    eta << "#scots grid parameter eta: ";
    first << "#scots coordinate of first grid point: ";
    last << "#scots coordinate of last grid point: ";
    nofGridPoints << "#scots number of grid points (per dim): ";
    indBddVars << "#scots index of bdd variables: ";

    for(size_t i=0; i<dim_; i++) {
      z << z_[i] << " ";
      eta << eta_[i] << " ";
      first << firstGridPoint_[i] << " ";
      last << lastGridPoint_[i] << " ";
      nofGridPoints << nofGridPoints_[i] << " ";
      for(size_t j=0; j<nofBddVars_[i]; j++) 
        indBddVars << indBddVars_[i][j] << " ";
    }
    z << endl;
    eta << endl;
    first << endl;
    last << endl;
    nofGridPoints << endl;
    indBddVars << endl;

    stringstream paramss;
    paramss << "################################################################################" << endl
            << "########### symbolic controller synthesis toolbox information added ############" << endl
            << "################################################################################" << endl
            << dim.str() \
            << eta.str() \
            << z.str() \
            << first.str() \
            << last.str() \
            << nofGridPoints.str() \
            << indBddVars.str();
    string param=paramss.str();


    FILE *file = fopen (filename,"w");
    if (file == NULL){
        ostringstream os;
        os << "Error: Unable to open file for writing." << filename << "'.";
        throw runtime_error(os.str().c_str());
    }
    if (param.size() != fwrite(param.c_str(),1,param.size(),file)) {
        throw "Error: Unable to write set information to file.";
    }

    /* before we save the BDD to file, we save it to another manager,
     * because the current manager is classified as ADD manager */
    Cudd mdest;
    BDD tosave = bddFunction.Transfer(mdest);

    int storeReturnValue = Dddmp_cuddBddStore(
      mdest.getManager(),
      NULL,
      tosave.getNode(),
      //(char**)varnameschar, // char ** varnames, IN: array of variable names (or NULL)
      NULL, // char ** varnames, IN: array of variable names (or NULL)
      NULL,
      DDDMP_MODE_BINARY,
      // DDDMP_VARNAMES,
      DDDMP_VARIDS,
      NULL,
      file
    );

    fclose(file);
    if (storeReturnValue!=DDDMP_SUCCESS) 
      throw "Error: Unable to write BDD to file.";
    else
      cout << "BDD saved to file: "<< filename << endl;
  } 
  /* function:  getZ
   * return constant pointer to the double array z_*/
  inline const double* getZ() const {
    return z_;
  }
  /* function:  getEta
   * return constant pointer to the double array eta_*/
  inline const double* getEta() const {
    return eta_;
  }
  /* function: complement
   * compute the of the symbolic set with respect to the uniform grid*/
  void complement() {
    /* generate ADD variables and ADD for the grid points in each dimension */
    ADD constant;
    ADD* aVar = new ADD[dim_]; 
    ADD** addVars = new ADD*[dim_];
    for(size_t i=0; i<dim_; i++)  {
      aVar[i] = ddmgr_->addZero();
      addVars[i] = new ADD[nofBddVars_[i]];
      CUDD_VALUE_TYPE c=1;
      for(size_t j=0; j<nofBddVars_[i]; j++) {
        constant=ddmgr_->constant(c);
        addVars[i][j] = ddmgr_->addVar(indBddVars_[i][j]);
        addVars[i][j] *= constant;
        aVar[i]+=addVars[i][j];
        c*=2;
      }
    }
    /* set grid points outside of the uniform grid to infinity */
    BDD outside=ddmgr_->bddZero();
    for(size_t i=0; i<dim_; i++) {
      BDD bdd=aVar[i].BddThreshold(nofGridPoints_[i]);
      outside+=bdd;;
    }
    for(size_t i=0; i<dim_; i++) 
      delete[] addVars[i];
    delete[] aVar;

    BDD inside=!outside;
    inside&=!bddFunction;
    bddFunction=inside;
  }
  /* function:  getFirstGridPoint 
   * the first grid point is saved to a double array of size dim_*/
  inline const double* getFirstGridPoint() const {
    return firstGridPoint_;
  }
  /* function:  getLastGridPoint 
   * the last grid point is saved to a double array of size dim_*/
  inline const double* getLastGridPoint() const {
    return lastGridPoint_;
  } 
  /* function:  getNofGridPoints
   * return pointer to size_t array containing the number of grid points*/
  inline const size_t* getNofGridPoints() const {
      return nofGridPoints_;
  }  
  /* function:  copyZ
   * z_ is written to the double array z of size dim_*/
  inline void copyZ(double z[]) const {
    for(size_t i=0; i<dim_; i++)
      z[i]=z_[i];
  }  
  /* function:  copyEta
   * eta_ is written to the double array eta of size dim_*/
  inline void copyEta(double eta[]) const {
    for(size_t i=0; i<dim_; i++)
      eta[i]=eta_[i];
  }
  /* function:  copyFirstGridPoint 
   * the first grid point is saved to a double array of size dim_*/
  inline void copyFirstGridPoint(double firstGridPoint[]) const {
    for(size_t i=0; i<dim_; i++)
      firstGridPoint[i]=firstGridPoint_[i];
  }
  /* function:  copyLastGridPoint 
   * the last grid point is saved to a double array of size dim_*/
  inline void copyLastGridPoint(double lastGridPoint[]) const {
    for(size_t i=0; i<dim_; i++)
      lastGridPoint[i]=lastGridPoint_[i];
  }
  /* function:  copyNofGridPoints
   * copy the size_t array containing the number of grid points to nofGridPoints*/
  inline void getNofGridPoints(size_t nofGridPoints[]) const {
    for(size_t i=0; i<dim_; i++)
      nofGridPoints[i]=nofGridPoints_[i];
  } 
 
  /* function: isElement
   *
   * check if an element x is an element of the SymolicSet
   *
   *  x -- x is an element of the domain
   *
   */
  bool isElement(vector<double> x) {
    if(!(x.size()==dim_)) {
      ostringstream os;
      os << "Error: SCOTSBDDReader::isElement(x): x must be of size dim_.";
      throw invalid_argument(os.str().c_str());
    }
    /* check if x is inside domain */
    for(size_t i=0; i<dim_; i++) 
      if (x[i]>(lastGridPoint_[i]+eta_[i]/2) || x[i]<(firstGridPoint_[i]-eta_[i]/2)) 
        return false;

    /* combute BDD representation of x */
    BDD cube=ddmgr_->bddOne();
    for(size_t i=0; i<dim_; i++) {
      int *phase = new int[nofBddVars_[i]];
      BDD *vars = new BDD[nofBddVars_[i]];
      for(size_t j=0;j<nofBddVars_[i];j++) {
        phase[j]=0;
        vars[j]=ddmgr_->bddVar(indBddVars_[i][j]);
      }
      int *p=phase;
      int idx=lround((x[i]-firstGridPoint_[i])/eta_[i]);
      for (; idx; idx/=2) *(p++)=0+idx%2;
      cube &= ddmgr_->bddComputeCube(vars,phase,nofBddVars_[i]);
      delete[] phase;
      delete[] vars;
    }
    /* check if cube is element of bddFunction */
    if( (cube & bddFunction)!=ddmgr_->bddZero() )
      return true;
    else
      return false;
  } 

private:
  void readMembersFromFile(const char *filename) {
    /* open file */
    ifstream bddfile(filename);
    if(!bddfile.good()) {
      ostringstream os;
      os << "Error: Unable to open file:" << filename << "'.";
      throw runtime_error(os.str().c_str());
    }
    /* read dimension from file */
    string line;
    while(!bddfile.eof()) {
      getline(bddfile,line); 
      if (line.substr(0,6)=="#scots") {
        if(line.find("dimension")!=string::npos) {
          istringstream sline(line.substr(line.find(":")+1));
          sline >> dim_;
        }
      }
    }
    if(dim_==0) {
      ostringstream os;
      os << "Error: Could not read dimension from file: " << filename << ". ";
      os << "Was " << filename << " created with SCOTSBDDReader::writeToFile?";
      throw runtime_error(os.str().c_str());
    }
    z_ = new double[dim_];
    eta_ = new double[dim_];
    lastGridPoint_ = new double[dim_];
    firstGridPoint_ = new double[dim_];
    nofGridPoints_ = new size_t[dim_];
    nofBddVars_ = new size_t[dim_];
    /* read eta/first/last/no of grid points/no of bdd vars */
    bddfile.clear();
    bddfile.seekg(0, ios::beg);
    int check=0;
    while(!bddfile.eof()) {
      getline(bddfile,line); 
      if (line.substr(0,6)=="#scots") {
        /* read eta */
        if(line.find("eta")!=string::npos) {
          check++;
          istringstream sline(line.substr(line.find(":")+1));
          for(size_t i=0; i<dim_; i++)
            sline >> eta_[i];
        }
       /* read z */
        if(line.find("measurement")!=string::npos) {
          check++;
          istringstream sline(line.substr(line.find(":")+1));
          for(size_t i=0; i<dim_; i++)
            sline >> z_[i];
        }
        /* read first grid point*/
        if(line.find("first")!=string::npos) {
          check++;
          istringstream sline(line.substr(line.find(":")+1));
          for(size_t i=0; i<dim_; i++)
            sline >> firstGridPoint_[i];
        }
        /* read last grid point*/
        if(line.find("last")!=string::npos) {
          check++;
          istringstream sline(line.substr(line.find(":")+1));
          for(size_t i=0; i<dim_; i++)
            sline >> lastGridPoint_[i];
        }
        /* read no of grid points */
        if(line.find("number")!=string::npos) {
          check++;
          istringstream sline(line.substr(line.find(":")+1));
          for(size_t i=0; i<dim_; i++) {
            sline >> nofGridPoints_[i];
            if(nofGridPoints_[i]==1)
              nofBddVars_[i]=1;
            else
              nofBddVars_[i]=(size_t)ceil(log2(nofGridPoints_[i]));
          }
        }
      }
      if(check==5)
        break;
    }
    if(check<5) {
      ostringstream os;
      os << "Error: Could not read all parameters from file: " << filename << ". ";
      os << "Was " << filename << " created with SCOTSBDDReader::writeToFile?";
      throw runtime_error(os.str().c_str());
    }
    /* read index of bdd vars  */
    indBddVars_ = new size_t*[dim_];
    bddfile.clear();
    bddfile.seekg(0, ios::beg);
    check=0;
    while(!bddfile.eof()) {
      getline(bddfile,line); 
      if (line.substr(0,6)=="#scots") {
        if(line.find("index")!=string::npos) {
          check++;
          istringstream sline(line.substr(line.find(":")+1));
          for(size_t i=0; i<dim_; i++) {
            indBddVars_[i]=new size_t[nofBddVars_[i]];
            for (size_t j=0; j<nofBddVars_[i]; j++) 
              sline >> indBddVars_[i][j];
          }
        }
      }
    }
    if(check!=1) {
      ostringstream os;
      os << "Error: Could not read bdd indices from file: " << filename << ".";
      os << "Was " << filename << " created with SCOTSBDDReader::writeToFile?";
      throw runtime_error(os.str().c_str());
    }  /* close file */
    bddfile.close();
    /* number of total variables */
    nvars_=0;
    for(size_t i=0; i<dim_; i++)
      for(size_t j=0; j<nofBddVars_[i]; j++)
        nvars_++;

  } /* end readMembersFromFile */

}; /* close class def */



#define NCS_CONTR_REMARKS_TEXT "NCS Synthesized Controller"

class SENSENDDReader {
private:
  /* var: ddmgr_  
   * the binary decision diagram manager */
  Cudd *ddmgr_;
	
  size_t ssdim, isdim;
  vector<size_t> ssVarsCount, isVarsCount;
  vector<double> ssEta, ssLb, ssUb, isEta, isLb, isUb;
  size_t nscmin, nscmax, ncamin, ncamax;
  vector<size_t> preVars, inpVars_, postVars, qVars;
  string Remarks;
  
  vector<size_t> nofBddVars_;
		
  /* var: bddFunction 
   * the bdd representing the set of points */
  BDD bddFunction;
public:
  /* constructor: SENSENDDReader :: default, added by MK 01.09/.2016
   */
  SENSENDDReader(){
  }


  void LoadFromFile(Cudd &ddmgr, const char *filename) {
	  
		ddmgr_ = &ddmgr;
			  
		readFromFile(filename, *ddmgr_, bddFunction, ssdim, isdim,
						ssVarsCount, isVarsCount,
						ssEta, ssLb, ssUb, isEta, isLb, isUb,
						nscmin, nscmax, ncamin, ncamax,
						preVars, inpVars_, postVars, qVars,
						Remarks);

		size_t good_remarks = Remarks.find(NCS_CONTR_REMARKS_TEXT);
		if (good_remarks==std::string::npos)
			throw(runtime_error("Error::ncsController:: This is not a valid NCS Controller File !!"));
		
		// for pre-states + q
		for(size_t i=0; i<nscmax; i++){
			for(size_t i=0; i<ssdim; i++){
				if(i==0)
					nofBddVars_.push_back(ssVarsCount[i]+1);
				else
					nofBddVars_.push_back(ssVarsCount[i]);
			}				
		}
		for(size_t i=0; i<ncamax; i++){
			for(size_t i=0; i<isdim; i++){
				nofBddVars_.push_back(isVarsCount[i]);
			}				
		}		
		
		// for inputs
		for(size_t i=0; i<isdim; i++)			
			nofBddVars_.push_back(isVarsCount[i]);		
		
		// for post-states in case it is a transition relation
		// => no need since we assume it is a controller and read it like this and check the remarks text
		
  }

  /* function:  getDimension 
   *
   * get the dimension of the real space in which the points that are represented by
   * the symbolic set live in */
  inline size_t getDimension(void) const {
    return (nscmax*ssdim + ncamax*isdim) + isdim;
  }

  /* function:  getBDD
   *
   * get the BDD that stores the function
   */
  inline BDD getBDD(void) const {
    return bddFunction;
  }  

  /* function:  getnofbddvars 
   *
   * get the pointer to the size_t array that contains the number of bdd
   * variables 
   */
  inline const size_t* getNofBddVars(void) const {
    return nofBddVars_.data();
  }
  
private:
	
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
		FILE *file = fopen(fname,"r");
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

}; /* close class def */


class BDDReader{
	Cudd *ddmgr_;
	string filename_;
	BDD_FILE_TYPE type_;
	SCOTSBDDReader scots_reader;
	SENSENDDReader sense_reader;
	
public:
	BDDReader(Cudd& ddmgr, const char *filename, BDD_FILE_TYPE type){
		ddmgr_ = &ddmgr;
		filename_ = filename;
		type_ = type;
	}

	BDD ReadBdd(){
		BDD zeroBdd = ddmgr_->bddZero();

		if(type_ == BDD_FILE_TYPE::SCOTS_BDD){
			scots_reader.LoadFromFile(*ddmgr_, filename_.c_str());
			return scots_reader.getBDD();
		}

		if(type_ == BDD_FILE_TYPE::SENSE_NBDD){
			sense_reader.LoadFromFile(*ddmgr_, filename_.c_str());
			return sense_reader.getBDD();
		}

		return zeroBdd;
	}


	size_t getNumBddVars(size_t dim_from, size_t dim_count){
		size_t count=0;
		if(type_ == BDD_FILE_TYPE::SCOTS_BDD){
			if(dim_from >= scots_reader.getDimension() || dim_count == 0){
				ostringstream os;
				os << "Error: BDDReader[SCOTS]::gettNumBddVars: invalid dimension index/count !";
				throw invalid_argument(os.str().c_str());
			}

			for(size_t i=dim_from; i< dim_from+dim_count; i++){
				count+= scots_reader.getNofBddVars()[i];
			}

			return count;
		}

		if(type_ == BDD_FILE_TYPE::SENSE_NBDD){
			if(dim_from >= sense_reader.getDimension() || dim_count == 0){
				ostringstream os;
				os << "Error: BDDReader[SENSE]::gettNumBddVars: invalid dimension index/count !";
				throw invalid_argument(os.str().c_str());
			}

			for(size_t i=dim_from; i< dim_from+dim_count; i++){
				count+= sense_reader.getNofBddVars()[i];
			}

			return count;
		}

		return 0;
	}


	SCOTSBDDReader getScotsReader(){
		if(type_ != BDD_FILE_TYPE::SCOTS_BDD){
			ostringstream os;
			os << "Error: BDDReader::getScotsReader: this is not a SCOTS bdd!";
			throw invalid_argument(os.str().c_str());
		}

		return scots_reader;
	}
	
	SENSENDDReader getSenseReader(){
		if(type_ != BDD_FILE_TYPE::SENSE_NBDD){
			ostringstream os;
			os << "Error: BDDReader::getSenseReader: this is not a SENSE nbdd!";
			throw invalid_argument(os.str().c_str());
		}

		return sense_reader;
	}	

};


#endif /* BDDReader_HH_ */
