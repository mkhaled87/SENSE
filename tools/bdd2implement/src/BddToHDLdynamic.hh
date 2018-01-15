/*YUE LI
BddtoVHDL tools for dynamic controller*/


#ifndef BDDTOHDLdynamic_HH_
#define BDDTOHDLdynamic_HH_

#include "cuddObj.hh"
#include "BddReader.hh"
#include "BddToString.hh"
#include "BddDecomposer.hh"
#include "utils.hh"

/*
 * a class to construct HDL codes from the BDD
 */
class BddToHDLdynamic{
	Cudd* cuddManager;
	/*new 2 variables for controller states
	 combine controller current state with input system states as dynamic state input
	 combine controller next state with output action state as dynamic action output*/
	size_t STATES_BDDVARSCOUNT;
	size_t CONTR_CURRENT_STATES_BDDVARSCOUNT;
	size_t CONTR_NEXT_STATES_BDDVARSCOUNT;
	size_t ACTION_BDDVARSCOUNT;
	size_t STATES_BDDVARSCOUNT_DYNAMIC;
	size_t ACTION_BDDVARSCOUNT_DYNAMIC;
	bool useLocalReader;
	BDDReader *reader;
	BDD passedBDD;

public:
	BddToHDLdynamic(Cudd& _mgr, const char* filename, BDD_FILE_TYPE type, size_t state_bddVar_count,size_t contr_current_state_bddVar_count,size_t contr_next_state_bddVar_count, size_t action_bddVar_count){


		STATES_BDDVARSCOUNT = state_bddVar_count;
		CONTR_CURRENT_STATES_BDDVARSCOUNT =contr_current_state_bddVar_count;
		CONTR_NEXT_STATES_BDDVARSCOUNT=contr_next_state_bddVar_count;
		ACTION_BDDVARSCOUNT = action_bddVar_count;

		STATES_BDDVARSCOUNT_DYNAMIC=state_bddVar_count+contr_current_state_bddVar_count;
		ACTION_BDDVARSCOUNT_DYNAMIC=contr_next_state_bddVar_count+action_bddVar_count;

		cuddManager = &_mgr;

		reader = new BDDReader(*cuddManager, filename, type);
		passedBDD = cuddManager->bddZero();
		useLocalReader = true;
	}

	BddToHDLdynamic(Cudd& _mgr, const BDD& givenBdd, size_t state_bddVar_count,size_t contr_current_state_bddVar_count,size_t contr_next_state_bddVar_count,size_t action_bddVar_count){
		STATES_BDDVARSCOUNT = state_bddVar_count;
		CONTR_CURRENT_STATES_BDDVARSCOUNT =contr_current_state_bddVar_count;
		 CONTR_NEXT_STATES_BDDVARSCOUNT=contr_next_state_bddVar_count;
		ACTION_BDDVARSCOUNT = action_bddVar_count;

		STATES_BDDVARSCOUNT_DYNAMIC=state_bddVar_count+contr_current_state_bddVar_count;
		ACTION_BDDVARSCOUNT_DYNAMIC=contr_next_state_bddVar_count+action_bddVar_count;

		cuddManager = &_mgr;

		passedBDD = givenBdd;
		useLocalReader = false;
	}

	vector<BDD> getDecomposedBdds(bool check, int verbosity=0){
		BDD mainBdd;
		if(useLocalReader)
			mainBdd = reader->ReadBdd();
		else
			mainBdd = passedBDD;

		if (mainBdd == cuddManager->bddZero()){
		      ostringstream os;
		      os << "Error: BddToVhdlBooleanFunction::GenerateVHDL: invalid BDD ! BDD is zero function.";
		      throw invalid_argument(os.str().c_str());
		}

		if(verbosity > 0)
			std::cout << "Decomposing the BDD to multi-functions ... ";

		BddOutputDecomposer decomposer(*cuddManager, mainBdd, STATES_BDDVARSCOUNT_DYNAMIC,ACTION_BDDVARSCOUNT_DYNAMIC);
		vector<BDD> subBdds = decomposer.Decompose(verbosity);

		if(verbosity > 0)
			std::cout << "decomposed to " <<  subBdds.size() << " functions." << std::endl;


		if(check){
			BDD composed_back = decomposer.Compose(subBdds);

			if(verbosity > 1){
				BDDUtils::PrintBDD("mainBdd", mainBdd);
				BDDUtils::PrintBDD("composed_back", composed_back);
				BDDUtils::PrintBDD("mainBdd*composed_back", mainBdd*composed_back);
			}

			if(mainBdd == mainBdd*composed_back)
				std::cout << "Checking decomposed BDD succceeded !" << std::endl;
			else
				std::cout << "Checking decomposed BDD failed !" << std::endl;
		}

		return subBdds;
	}

	void GenerateRawVHDL(string outFilename, bool check=false, int verbosity=0){

		/*use new template for dynamic controller*/
		const string template_file = "../../templates/DynamicBooleanFunction.vhdl";


		vector<BDD> subBdds = getDecomposedBdds(check, verbosity);


		BddToStringConverter converter(*cuddManager, "and", "or", "not");


		if(verbosity > 0)
			std::cout << "Converting to VHDL code ... ";


		/*add State function for the controller state*/
		stringstream inPorts, outPorts, outFuncs, StateFuncs;

		for(size_t i=0; i<STATES_BDDVARSCOUNT; i++){
			 inPorts << "x" << i << ": in std_logic;" << endl;
		}


		for(size_t i=0; i<ACTION_BDDVARSCOUNT; i++){
			outPorts << "f" << i << ": out std_logic";

			if(i != ACTION_BDDVARSCOUNT-1)
				outPorts << ";" << endl;
		}

		for(size_t i=0; i<CONTR_CURRENT_STATES_BDDVARSCOUNT; i++){
				StateFuncs<<i;
		}
		for(size_t i=0; i<subBdds.size(); i++){
			string bddAsString = converter.asShannonForm(subBdds[i]);
			outFuncs << "f" << i << " <= " << bddAsString << ";" << endl;
		}


		string templateText = IOUtils::ReadAllFileText(template_file);
			/*add string placement for the controller state count*/
		string OutText = StringManipulator::ReplaceString(templateText,   "#$ENTITY_INPUT_PORTS$#",  inPorts.str());
		       OutText = StringManipulator::ReplaceString(OutText,       "#$ENTITY_OUTPUT_PORTS$#", outPorts.str());
			   OutText = StringManipulator::ReplaceString(OutText,       "#$noBddvarState$#", StateFuncs.str());
		       OutText = StringManipulator::ReplaceString(OutText, "#$BEHAVIORAL_OUTPUT_FUNCTIONS$#", outFuncs.str());
		       OutText = StringManipulator::ReplaceString(OutText, "#$DATES$#", StopWatch::GetCurrentDateTime());

		IOUtils::FileWriteAllText(outFilename, OutText);

		if(verbosity > 0)
			std::cout << "saved to the file: " <<  outFilename << std::endl;




	}


	~BddToHDLdynamic(){
		if(useLocalReader)
			delete reader;
	}

};


#endif
