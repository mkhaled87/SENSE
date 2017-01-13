/*
 * mexNcsController.cc
 *
 *  created on: 25.11.2016
 *      author: M.Khaled
 */



#include <iostream>
#include <vector>
#include <math.h>
#include <cstring>

#include "mex.h"
#include "ncsController.hh"


using namespace std;

vector<Cudd*> pCuddManager;
vector<ncsController*> pContr;

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  /* Check for proper number of arguments */
  if (nrhs < 1) 
    mexErrMsgIdAndTxt("MATLAB:mexNcsController:","mexNcsController requires at least two input arguments.");

  /* input must be a string */
  if ( mxIsChar(prhs[0]) != 1)
    mexErrMsgIdAndTxt("MATLAB:mexNcsController:", "First input must a string out of {open,getInput,close}.");


  /* get pointer to the paramter */
  char* param = mxArrayToString(prhs[0]);

  bool recognized=false;

  /*****************************************************************/
  /* command: open						   */
  /*****************************************************************/
  if (!std::strcmp(param,"open")) {
    recognized=true;

    if(nrhs != 2)
	mexErrMsgIdAndTxt("MATLAB:mexNcsController","You must provide one parametes with the 'open' which is the file name .");
    
    if ( mxIsChar(prhs[1]) != 1)
      mexErrMsgIdAndTxt("MATLAB:mexNcsController", "Second input must be the filename (i.e. a string).");

    char* filename=mxArrayToString(prhs[1]);

    try{
      pCuddManager.push_back(new Cudd());
      size_t id = pCuddManager.size()-1;
      pContr.push_back(new ncsController(*pCuddManager[id], filename));
      plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);
      double *out = mxGetPr(plhs[0]);
      out[0]=id;
    }
    catch (const std::exception& e) {
      mexErrMsgIdAndTxt("MATLAB:mexNcsController",e.what());
    }
  }

  /*****************************************************************/
  /* command: close						   */
  /*****************************************************************/
  if (!std::strcmp(param,"close")) {
    recognized=true;

    for(size_t i=0; i<pCuddManager.size(); i++){
    	delete pContr[i];
    	delete pCuddManager[i];
    }
  }

  /*****************************************************************/
  /* command: getInput						   */
  /*****************************************************************/
  if (!std::strcmp(param,"getInput")) {
    recognized=true;

    if(nrhs != 4)
	mexErrMsgIdAndTxt("MATLAB:mexNcsController","You must provide two parametes with the 'getInput' which are: q_values and x_values.");


    mwSize IdSize = mxGetN(prhs[1]);
    if(IdSize != 1)
	mexErrMsgIdAndTxt("MATLAB:mexNcsController", "First parameter should be the id of the controller (i.e. single value).");

    size_t contId = (size_t)(mxGetPr(prhs[1])[0]);

    mwSize nqValues = mxGetN(prhs[2]);
    double* qValues = mxGetPr(prhs[2]);

    mwSize nxValues = mxGetN(prhs[3]);
    double* xValues = mxGetPr(prhs[3]);

    ncsState* stateX = pContr[contId]->getSourceState();

    size_t ssDim = stateX->getSsdim();
    size_t isDim = stateX->getIsdim();

    size_t nscmax = stateX->getNscmax();

    if(nqValues != nscmax)
       mexErrMsgIdAndTxt("MATLAB:mexNcsController","number of q values is not correct. It should be euql to NSC_MAX.");
    
    if(nxValues != (nscmax*ssDim + nscmax*isDim))
       mexErrMsgIdAndTxt("MATLAB:mexNcsController","number of x values is not correct. It should be euql to (NSC_MAX*SS_DIM + NSC_MAX*IS_DIM).");

    vector<int> q_values;
    vector<double> XU_values;

    for(size_t i=0; i<nqValues; i++)
	q_values.push_back(qValues[i]);

    for(size_t i=0; i<nxValues; i++)
	XU_values.push_back(xValues[i]);

    vector<vector<double>> inputs;
    try{
      inputs = pContr[contId]->getInputs(XU_values, q_values);
    }
    catch (const std::exception& e) {
      mexErrMsgIdAndTxt("MATLAB:mexNcsController",e.what());
    }

    size_t length = inputs.size();
    if(length) {
      size_t idim =  inputs[0].size();
      plhs[0]=mxCreateDoubleMatrix(length,idim,mxREAL);
      double *u = mxGetPr(plhs[0]);
      for(size_t i=0; i<idim; i++) {
        for(size_t j=0; j<length; j++)
          u[i*length+j] = inputs[j][i];
      }
    } else {
      plhs[0]= mxCreateDoubleMatrix( 0, 0, mxREAL );
    }

  }

  /*****************************************************************/
  /* command: getInitialInputs						   */
  /*****************************************************************/
  /*
  if (!std::strcmp(param,"getInitialInputs")) {
    recognized=true;

    if(nrhs != 2)
	mexErrMsgIdAndTxt("MATLAB:mexNcsController","You must provide one parametes with the 'getInitialInputs' which is: x0, the initial state.");

    if(!loaded)
      mexErrMsgIdAndTxt("MATLAB:mexNcsController", "No controller is loaded.");

    mwSize nx0 = mxGetN(prhs[1]);
    double* x0Values = mxGetPr(prhs[1]);

    ncsState* stateX = pContr->getSourceState();

    size_t ssDim = stateX->getSsdim();
    size_t isDim = stateX->getIsdim();

    size_t nscmax = stateX->getNscmax();

    if(nx0 != ssDim)
       mexErrMsgIdAndTxt("MATLAB:mexNcsController","number of x0 elements is not correct.");

    vector<double> x0_vector;
	for(size_t i=0; i<nx0; i++)
		x0_vector.push_back(x0Values[i]);
   
    vector<vector<double>> inputs;
    try{
      inputs = pContr->getInitialInputs(x0_vector);
    }
    catch (const std::exception& e) {
      mexErrMsgIdAndTxt("MATLAB:mexNcsController",e.what());
    }

    size_t length = inputs.size();
    if(length) {
      size_t idim =  inputs[0].size();
      plhs[0]=mxCreateDoubleMatrix(length,idim,mxREAL);
      double *u = mxGetPr(plhs[0]);
      for(size_t i=0; i<idim; i++) {
        for(size_t j=0; j<length; j++)
          u[i*length+j] = inputs[j][i];
      }
    } else {
      plhs[0]= mxCreateDoubleMatrix( 0, 0, mxREAL );
    }
  }
  */
  

  if(!recognized)
    mexErrMsgIdAndTxt("MATLAB:mexNcsController","Parameter is not recognized or you didnt load the controller first.");

  return;
}

