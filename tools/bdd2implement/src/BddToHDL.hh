#ifndef BDDTOHDL_HH_
#define BDDTOHDL_HH_

#include "cuddObj.hh"
#include "BddReader.hh"
#include "BddToString.hh"
#include "BddDecomposer.hh"
#include "utils.hh"

/*
 * a class to construct HDL codes from the BDD
 */
class BddToHDL{
	Cudd* cuddManager;
	size_t STATES_BDDVARSCOUNT;
	size_t ACTION_BDDVARSCOUNT;

	bool useLocalReader;
	BDDReader *reader;
	BDD passedBDD;

public:
	BddToHDL(Cudd& _mgr, const char* filename, BDD_FILE_TYPE type, size_t state_bddVar_count, size_t action_bddVar_count){
		STATES_BDDVARSCOUNT = state_bddVar_count;
		ACTION_BDDVARSCOUNT = action_bddVar_count;
		cuddManager = &_mgr;

		reader = new BDDReader(*cuddManager, filename, type);
		passedBDD = cuddManager->bddZero();
		useLocalReader = true;
	}

	BddToHDL(Cudd& _mgr, const BDD& givenBdd, size_t state_bddVar_count, size_t action_bddVar_count){
		STATES_BDDVARSCOUNT = state_bddVar_count;
		ACTION_BDDVARSCOUNT = action_bddVar_count;
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
			std::cout << "Decomposing the BDD to multi-functions ... " << std::endl;

		BddOutputDecomposer decomposer(*cuddManager, mainBdd, STATES_BDDVARSCOUNT, ACTION_BDDVARSCOUNT);
		vector<BDD> subBdds = decomposer.Decompose(verbosity);

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
		const string template_file = "../../templates/BooleanFunctions.vhdl";

		vector<BDD> subBdds = getDecomposedBdds(check, verbosity);

		BddToStringConverter converter(*cuddManager, "and", "or", "not");

		if(verbosity > 0)
			std::cout << "Converting to VHDL code ... ";

		stringstream inPorts, outPorts, outFuncs;
		for(size_t i=0; i<STATES_BDDVARSCOUNT; i++){
			 inPorts << "x" << i << ": in std_logic;" << endl;
		}
		for(size_t i=0; i<ACTION_BDDVARSCOUNT; i++){
			outPorts << "f" << i << ": out std_logic";

			if(i != ACTION_BDDVARSCOUNT-1)
				outPorts << ";" << endl;
		}
		for(size_t i=0; i<subBdds.size(); i++){
			string bddAsString = converter.asShannonForm(subBdds[i]);
			outFuncs << "f" << i << " <= " << bddAsString << ";" << endl;
		}

		string templateText = IOUtils::ReadAllFileText(template_file);
		string OutText = StringManipulator::ReplaceString(templateText,   "#$ENTITY_INPUT_PORTS$#",  inPorts.str());
		       OutText = StringManipulator::ReplaceString(OutText,       "#$ENTITY_OUTPUT_PORTS$#", outPorts.str());
		       OutText = StringManipulator::ReplaceString(OutText, "#$BEHAVIORAL_OUTPUT_FUNCTIONS$#", outFuncs.str());
		       OutText = StringManipulator::ReplaceString(OutText, "#$DATES$#", StopWatch::GetCurrentDateTime());

		IOUtils::FileWriteAllText(outFilename, OutText);

		if(verbosity > 0)
			std::cout << "saved to the file: " <<  outFilename << std::endl;
	}

	void GenerateRawVerilog(string outFilename, bool check=false, int verbosity=0){
		const string template_file = "../../templates/BooleanFunctions.v";
		vector<BDD> subBdds = getDecomposedBdds(check, verbosity);

		BddToStringConverter converter(*cuddManager, " & ", " | ", "~");

		if(verbosity > 0)
			std::cout << "Converting to Verilog code ... ";

		stringstream inPorts, outPorts, outFuncs;
		for(size_t i=0; i<STATES_BDDVARSCOUNT; i++){
			 inPorts << "input x" << i << "," << endl;
		}
		for(size_t i=0; i<ACTION_BDDVARSCOUNT; i++){
			outPorts << "output f" << i;

			if(i != ACTION_BDDVARSCOUNT-1)
				outPorts << "," << endl;
		}
		for(size_t i=0; i<subBdds.size(); i++){
			string bddAsString = converter.asShannonForm(subBdds[i]);
			outFuncs << "assign f" << i << " = " << bddAsString << ";" << endl;
		}

		string templateText = IOUtils::ReadAllFileText(template_file);
		string OutText = StringManipulator::ReplaceString(templateText,   "#$MODULE_INPUT_PORTS$#",  inPorts.str());
		       OutText = StringManipulator::ReplaceString(OutText,       "#$MODULE_OUTPUT_PORTS$#", outPorts.str());
		       OutText = StringManipulator::ReplaceString(OutText, "#$ASSIGN_OUTPUT_FUNCTIONS$#", outFuncs.str());
		       OutText = StringManipulator::ReplaceString(OutText, "#$DATES$#", StopWatch::GetCurrentDateTime());

		IOUtils::FileWriteAllText(outFilename, OutText);

		if(verbosity > 0)
			std::cout << "saved to the file: " <<  outFilename << std::endl;
	}

	~BddToHDL(){
		if(useLocalReader)
			delete reader;
	}

};


#endif
