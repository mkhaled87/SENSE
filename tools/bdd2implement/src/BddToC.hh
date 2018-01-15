#ifndef BDDTOC_HH_
#define BDDTOC_HH_

#include "cuddObj.hh"
#include "BddReader.hh"
#include "BddToString.hh"
#include "BddDecomposer.hh"
#include "utils.hh"

/*
 * a class to construct HDL codes from the BDD
 */
class BddToC{
	Cudd* cuddManager;
	size_t STATES_BDDVARSCOUNT;
	size_t ACTION_BDDVARSCOUNT;

	bool useLocalReader;
	BDDReader *reader;
	BDD passedBDD;

public:
	BddToC(Cudd& _mgr, const char* filename, BDD_FILE_TYPE type, size_t state_bddVar_count, size_t action_bddVar_count){
		STATES_BDDVARSCOUNT = state_bddVar_count;
		ACTION_BDDVARSCOUNT = action_bddVar_count;
		cuddManager = &_mgr;

		reader = new BDDReader(*cuddManager, filename, type);
		passedBDD = cuddManager->bddZero();
		useLocalReader = true;
	}

	BddToC(Cudd& _mgr, const BDD& givenBdd, size_t state_bddVar_count, size_t action_bddVar_count){
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

	void GenerateBooleanFunctions(string outFilename, bool check=false, int verbosity=0){
		const string template_file = "../../templates/BooleanFunctions.h";
		vector<BDD> subBdds = getDecomposedBdds(check, verbosity);

		BddToStringConverter converter(*cuddManager, " && ", " || ", "!");

		if(verbosity > 0)
			std::cout << "Converting to C/C++ code ... ";


		stringstream bfuncTemplate;
		bfuncTemplate << "bool f#$idx$#(#$INPUT_VARS$#){" << endl;
		bfuncTemplate << "\t return (#$BOOLEAN_EXPR$#);" << endl;
		bfuncTemplate << "}" << endl;


		stringstream inVars, outFuncs, boolFuncArray, passArgs;
		for(size_t i=0; i<STATES_BDDVARSCOUNT; i++){
			inVars << "bool x" << i;
			passArgs << "inputValues[" << i << "]";

			if(i != STATES_BDDVARSCOUNT-1){
				inVars << ", ";
				passArgs << ", ";
			}
		}
		for(size_t i=0; i<ACTION_BDDVARSCOUNT; i++){
			string bddAsString = converter.asShannonForm(subBdds[i]);
			stringstream idx;
			idx << i;
			string tmp_bfunc = bfuncTemplate.str();
				   tmp_bfunc = StringManipulator::ReplaceString(tmp_bfunc, "#$INPUT_VARS$#", inVars.str());
				   tmp_bfunc = StringManipulator::ReplaceString(tmp_bfunc, "#$BOOLEAN_EXPR$#", bddAsString);
				   tmp_bfunc = StringManipulator::ReplaceString(tmp_bfunc, "#$idx$#", idx.str());

		   outFuncs << tmp_bfunc << endl << endl;

		   boolFuncArray << "bddBoolFunctions[" << i << "] = f" << i << ";" << endl;
		}

		stringstream inp_cnt, out_cnt;
		inp_cnt << STATES_BDDVARSCOUNT;
		out_cnt << ACTION_BDDVARSCOUNT;

		string templateText = IOUtils::ReadAllFileText(template_file);
		string OutText = StringManipulator::ReplaceString(templateText,   "#$OUTPUT_FUNCTIONS$#",  outFuncs.str());
			   OutText = StringManipulator::ReplaceString(OutText,   "#$INPUT_COUNT$#",  inp_cnt.str());
			   OutText = StringManipulator::ReplaceString(OutText,   "#$OUTPUT_COUNT$#",  out_cnt.str());
			   OutText = StringManipulator::ReplaceString(OutText,   "#$INPUT_VARS$#",  inVars.str());
			   OutText = StringManipulator::ReplaceString(OutText,   "#$BOOL_FUNC_ARRAY$#",  boolFuncArray.str());
			   OutText = StringManipulator::ReplaceString(OutText,   "#$PASS_ARGS$#",  passArgs.str());

		IOUtils::FileWriteAllText(outFilename, OutText);

		if(verbosity > 0)
			std::cout << "saved to the file: " <<  outFilename << std::endl;
	}

	~BddToC(){
		if(useLocalReader)
			delete reader;
	}

};


#endif

