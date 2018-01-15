/*
 * psmController.h
 *
 *  Created on: Nov 14, 2016
 *      Author: M.Khaled
 */

#ifndef PSMCONTROLLER_HH_
#define PSMCONTROLLER_HH_

#include <array>
#include <vector>
#include <iostream>

#include "cuddObj.hh"
#include "SENSE.hh"

using namespace std;

class psmController{
	size_t ssDIM;
	size_t isDIM;
	size_t NSC;
	size_t NCA;

	SymbolicSet* symSetPlant;
	ncsController* ncsCont;

	vector<size_t> xu_dimensions;
public:
	psmController(Cudd& cuddManager, const char* plantSMFile, const char* controllerFile, size_t ssDIM_, size_t isDIM_, size_t NSC_, size_t NCA_, bool doPlantInDifferentBDDVars = false){
		symSetPlant = new SymbolicSet();
		
		if(doPlantInDifferentBDDVars)
			symSetPlant->LoadFromFile(cuddManager, plantSMFile, 1);
		else
			symSetPlant->LoadFromFile(cuddManager, plantSMFile);
		
		ncsCont = new ncsController(cuddManager, controllerFile);

		NSC = NSC_;
		NCA = NCA_;

		ssDIM = ssDIM_;
		isDIM = isDIM_;

		for(size_t i=0; i<ssDIM + isDIM; i++)
			xu_dimensions.push_back(i);
	}

	~psmController() {
		// TODO Auto-generated destructor stub
		delete symSetPlant;
		delete ncsCont;
	}


	vector<vector<double>> sys_posts(const vector<double>& x, const vector<double>& u){




		vector<double> xu_concrete;
		VectorManager::AppendVector(xu_concrete, x);
		VectorManager::AppendVector(xu_concrete, u);

		vector<vector<double>> xdash = symSetPlant->setValuedMap(xu_concrete, xu_dimensions);
		return xdash;
	}

	// NCS: This function recursively constructs the normal ncs-state given any initial x0 and u0
	void constructPossibleNCSStates(size_t depth,const vector<double>& x0, const vector<vector<double>>& uCA, vector<vector<double>>& currentPath, vector<vector<vector<double>>>& pathsPool){

		vector<double> x_current;
		(depth == 0)?x_current = x0:x_current=currentPath[depth-1];
		vector<vector<double>> post_states = sys_posts(x_current,uCA[depth]);

		for(size_t i=0; i<post_states.size(); i++){
			vector<double> xdash = post_states[i];
			currentPath[depth] = xdash;
			if(depth == NSC-1){
				vector<vector<double>> tmp = currentPath;
				pathsPool.push_back(tmp);
			}
			else
				constructPossibleNCSStates(depth+1,x0,uCA,currentPath, pathsPool);
		}
	}


	vector<vector<double>> suggestControlInputs(const vector<double>& x0, const vector<vector<double>>& uCA){

		vector<vector<double>> ret;

		vector<vector<double>> x2 = sys_posts(x0, uCA[0]);

		// constructing all possible tuples
		vector<vector<double>> currentPath(NSC);
		vector<vector<vector<double>>> pathsPool;

		currentPath[0] = x0;
		constructPossibleNCSStates(1, x0, uCA, currentPath, pathsPool);

		/*
		cout << "we have " << pathsPool.size() << " posible NCS states !" << endl;
		cout << "Corresponding inputs: " << endl;
		*/

		// getting the control input for each one
		for(size_t i=0; i<pathsPool.size(); i++){
			vector<int> q_values;
			vector<double> XU_values;

			for(size_t j=0; j<NSC; j++)
				q_values.push_back(0);

			for(int j=NSC-1; j>=0; j--)
				for(size_t k=0; k<ssDIM; k++)
					XU_values.push_back(pathsPool[i][j][k]);

			for(size_t j=0; j<NCA; j++)
				for(size_t k=0; k<isDIM; k++)
					XU_values.push_back(uCA[j][k]);

			vector<vector<double>> cInputs;
			cInputs = ncsCont->getInputs(XU_values, q_values);

			/*
			cout << "NCS-state #" << setw(2) << i << ":";
			for(size_t j=0; j<cInputs.size(); j++){
				cout << "\t(";
				for(size_t k=0; k<cInputs[j].size(); k++)
					cout << cInputs[j][k] << " ";
				cout << ")";
			}
			*/

			if(i == 0)
				ret = cInputs;
			else
				VectorManager::IntersectVector(ret,cInputs);

			/*cout << endl;*/
		}

		return ret;
	}
};


#endif /* PSMCONTROLLER_HH_ */
