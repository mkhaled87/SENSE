/*
 * abstract.cc
 *
 *  created on: 27.12.2017
 *      author: M. Khaled
 *
 *********************************************************
 * This tool helps doing the abstraction directly from the command line.
 * Users dont need to craft/build their own examples to do symbolic 
 * abstractions of NCS. Users provide their symbolic abstractions of 
 * the plants and the tool constructs the abstraction of the NCS.
 */

#include <array>
#include <iostream>

#define VERBOSEBASIC

#include "cuddObj.hh"
#include "SENSE.hh"

class InputParser{
    public:
        InputParser (int &argc, char **argv){
            for (int i=1; i < argc; ++i)
                this->tokens.push_back(std::string(argv[i]));
        }

        std::string getCmdOptionStr(const std::string &option){
            std::vector<std::string>::const_iterator itr;
            itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
            if (itr != this->tokens.end() && ++itr != this->tokens.end()){
                return *itr;
            }
            return string("");
        }

        size_t getCmdOptionInt(const std::string &option){
	    string opt = getCmdOptionStr(option);
	    if(opt == "") return 0;
	    std::stringstream ss(opt);
	    size_t num;
	    ss >> num;
	    return num;
        }

        bool cmdOptionExists(const std::string &option) const{
            return std::find(this->tokens.begin(), this->tokens.end(), option)
                   != this->tokens.end();
        }
    private:
        std::vector <std::string> tokens;
};

int main(int argc, char **argv) {
  InputParser input(argc, argv);  
  Cudd cuddManager;
  size_t ssDIM, isDIM, NSCMAX, NCAMAX;
  stringstream ss;
  string FILE_BDD_REL, FILE_NBDD_REL;


  if(   !input.cmdOptionExists("-nscmax") || 
	!input.cmdOptionExists("-nscmax") || 
	!input.cmdOptionExists("-ssdim") || 
	!input.cmdOptionExists("-isdim")  || 
	!input.cmdOptionExists("-bddrel")){
	cout << "Missing essential inputs !! " << endl;
	return 0;
  }

  NSCMAX = input.getCmdOptionInt("-nscmax");
  NCAMAX = input.getCmdOptionInt("-ncamax");
  ssDIM  = input.getCmdOptionInt("-ssdim");
  isDIM  = input.getCmdOptionInt("-isdim");
  FILE_BDD_REL = input.getCmdOptionStr("-bddrel");

  if(input.cmdOptionExists("-o")){
    FILE_NBDD_REL = input.getCmdOptionStr("-o");
  }else{
    FILE_NBDD_REL = FILE_BDD_REL + ".nbdd";
  }


  cout << "Parsed params:" << endl;
  cout << " ssDIM: " << ssDIM << endl;
  cout << " isDIM: " << isDIM << endl;
  cout << "NSCMAX: " << NSCMAX << endl;
  cout << "NCAMAX: " << NCAMAX << endl;
  cout << "   REL: " << FILE_BDD_REL << endl;


  cout << "Initiating the NCS Transition relation from the original relation ... " << endl;
  ncsFIFOTransitionRelation ncsState13(cuddManager, FILE_BDD_REL.c_str(), ssDIM, isDIM, NSCMAX, NCAMAX);
  cout << "NCS relation intialized !" << endl;

  cuddManager.AutodynEnable();
  cout << "Expanding transition relation ... " << endl;
  ncsState13.ExpandBDD();
  cout << "NCS relation expanded in " << ncsState13.getExpandTime() << " seconds !" << endl;
  ncsState13.WriteToFile(FILE_NBDD_REL.c_str());
  cout << "New expanded transition relation is saved to the file: " << FILE_NBDD_REL << endl;

  return 1;
}
