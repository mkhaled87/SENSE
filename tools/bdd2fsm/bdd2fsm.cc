/*
 * bdd2fsm.cc
 *
 *  created on: 02.12.2016
 *      author: M. Khaled
 *
 *********************************************************
 * This tool generates a .fsm file for any SCOTS-based bdd.
 * Usually, the bdd should represent a transition relation.
 * Users can use the generated file to visualize the relation
 * as a graph using a tool like Giphy.
 */


#include <array>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "cuddObj.hh"
#include "SymbolicSetInterface.hh"


using namespace std;

#define VERBOSE

string INPUT_FILE;
size_t SS_DIM;
size_t IS_DIM;


Cudd cuddManager;
ofstream out_file_fsm;
ofstream out_file_cs_edges;
ofstream out_file_cs_nodes;
ofstream out_file_execluded;

template<typename T>
void vector_print(vector<T>& v)
{
	cout << "(";
    for (size_t i=0; i<v.size(); i++)
    {
        std::cout << v[i];
		if(i<v.size()-1)
			cout << ',';
    }
	cout << ")";
}

template<typename T>
inline size_t vector_find(vector<T>& v, T item){
	return (find(v.begin(), v.end(), item) - v.begin());
}

/*
* return 0: indicate error - one of the following
		1] sizes mismatch
		2] c is not in the grid points
*/
size_t CoordinatesToIndex(vector<double>& c, vector<vector<double>>& GridPoints){
	if(c.size() != GridPoints.size()){
		cout << "Error::CoordinatesToIndex::Dimension mismatch !" << endl;
		return 0;
	}
	
	
		
	size_t ret = 0;
	for(size_t i=0; i<c.size()-1; i++){
		
		size_t mul = vector_find(GridPoints[i],c[i]);
		if(mul >= GridPoints[i].size())
			return 0;
		
		for(size_t j=i+1; j<c.size(); j++)
			mul*= GridPoints[j].size();
		
		ret+=mul;
	}
	size_t last_elem = vector_find(GridPoints[c.size()-1],c[c.size()-1]);
	if(last_elem >= GridPoints[c.size()-1].size())
		return 0;
	
	ret += last_elem;	

	/*
	bool verb = (c[0]==13 && c[1]==13 && c[2]==13)?true:false;
	if(verb || ret == 17590){
		cout << "I am asked about this vector: ";
		vector_print(c);		
		cout << " and my result is: " << ret << endl;
	}
	*/
	
	return ret+1;
}


bool ParseInputArgs(int argc, char* argv[]){
	
	size_t arg_count = 0;
	
	for(int i=0; i<argc; i++){	
		size_t pos;
		string arg(argv[i]);			

		pos = arg.find("-ssdim=");
		if( pos != string::npos){
			SS_DIM = atoi(arg.substr(pos+7).c_str());
			arg_count++;
		}

		pos = arg.find("-isdim=");
		if( pos != string::npos){
			IS_DIM = atoi(arg.substr(pos+7).c_str());
			arg_count++;
		}			
		
		pos = arg.find(".bdd");
		if( pos != string::npos){
			INPUT_FILE = arg;
			arg_count++;
		}			
	}
	
	
#ifdef VERBOSE
	system("clear");
	cout << "Input arguments: " << endl;
	cout << "\tInput file :" << INPUT_FILE << endl;
	cout << "\tssDim      :" << SS_DIM << endl;
	cout << "\tisDim      :" << IS_DIM << endl;

	cout << endl;
#endif	
	
	if(arg_count == 3)
		return true;
	else
		return false;	
}


void PrintGridPoints(vector<vector<double>>& GridPoints, size_t depth, vector<double>& current_point, size_t& global_cnt){
	if(GridPoints.size() <= depth){
		// now i got a complete tuple, print it !
		out_file_fsm << endl;				
		out_file_cs_nodes << global_cnt << ",(";
		
		
		for(size_t i=0; i<current_point.size(); i++){
			out_file_fsm << current_point[i] << " ";
			out_file_cs_nodes << current_point[i] << " ";;
		}
		
		out_file_cs_nodes << ")";
		out_file_cs_nodes << endl;			
								
		global_cnt++;
		return;
	}
		
	for(size_t i=0; i< GridPoints[depth].size(); i++){
		current_point.push_back(GridPoints[depth][i]);
		PrintGridPoints(GridPoints, depth+1, current_point, global_cnt);
		current_point.pop_back();		
	}
}

size_t BinArrayToInteger(int* A, size_t size){
	size_t val =0;
	for(size_t i=0; i< size; i++){
		val += A[i]*(1<<i);
	}
	return val;
}

void PrintArrayToString(double* A, size_t start, size_t size, string& str){
	stringstream sstr;
	sstr << "(";
	for(size_t i=0; i<size; i++){
		sstr << A[start+i];		
		if(i != size-1) 
			sstr << ',';
	}
	sstr << ")";	
	str = sstr.str();
}

void PrintTransitions(SymbolicSet &ss, vector<vector<double>> X_GridPoints)
{
	const size_t dim = ss.getDimension();
	vector<size_t> ivars;
	size_t nvars = 0;
	size_t nssvars = 0;
	size_t nisvars = 0;
	
	
	for(size_t i=0; i<dim; i++)
	{
		if(i<SS_DIM)
			nssvars += ss.getNofBddVars()[i];		
		
		if(i>=SS_DIM && i<(SS_DIM + IS_DIM))
			nisvars += ss.getNofBddVars()[i];
			
		nvars += ss.getNofBddVars()[i];
		
		for (size_t j=0; j<ss.getNofBddVars()[i]; j++)
			ivars.push_back(ss.getIndBddVars()[i][j]);
		
	}

	/*
	cout << "nvars: " << nvars << endl;
	for (size_t j=0; j<ivars.size(); j++)
			cout << ivars[j] << " ";
	cout << endl;
	*/
	
	CuddMintermIterator cuddIterator(ss.getSymbolicSet(), ivars, nvars);


	int* mterm = new int[nvars];
	double* element = new double[dim];
	size_t cnt=0, cnt_exclude=0;
	while(!cuddIterator.done())
	{		
		cuddIterator.copyMinterm(mterm);
		ss.mintermToElement(mterm, element);
		
		
		vector<double> x_vec, xd_vec, u_vec;
		for(size_t i=0; i<SS_DIM; i++){
			x_vec.push_back(element[i]);
			xd_vec.push_back(element[SS_DIM+IS_DIM+i]);
		}		
		
		size_t x_idx = CoordinatesToIndex(x_vec, X_GridPoints);
		size_t xd_idx = CoordinatesToIndex(xd_vec, X_GridPoints);
		
		
		
		string input_label="";
		PrintArrayToString(element,SS_DIM, IS_DIM, input_label);
		
		//size_t xsource = BinArrayToInteger(mterm , nssvars);
		//size_t xtarget = BinArrayToInteger(mterm + nssvars + nisvars, nssvars);
		
		if(x_idx ==0 || xd_idx == 0){
			
			string full_state;
			PrintArrayToString(element, 0, dim, full_state);
			out_file_execluded << full_state << endl;
			
			cnt_exclude++;
			++cuddIterator;
			continue;
		}

		out_file_fsm << endl;
		out_file_fsm << x_idx << " " << xd_idx << " " << "\"" << input_label.c_str() << "\"";
		
		out_file_cs_edges << x_idx-1 << ","  << xd_idx-1 << "," << input_label << endl;
				
		
		++cuddIterator;
		cnt++;
	}
	
	cout << "Number of transitions: " << cnt << endl;
	cout << "Number of excluded transitions: " << cnt_exclude << endl;

	delete mterm;
	delete element;
}

int main(int argc, char* argv[]) {
		
	vector<vector<double>> X_GridPoints;
	bool parsed = ParseInputArgs(argc, argv);
	if(!parsed){
		cout << "Error: Invalid input arguements !" << endl;
		return 0;
	}
	
	// reading the bdd file as SymbolicSet
	SymbolicSet symXUX;
	symXUX.LoadFromFile(cuddManager, INPUT_FILE.c_str(), 1);
	
	// Checking dimensions in the file with the provided ones
	if(symXUX.getDimension() != (2*SS_DIM + IS_DIM)){
		cout << "Error:: input provided dimensions in input-file do not match the given dimensions";
		return 0;
	}

	// Opening output file (same_name.fsm)
	
	out_file_fsm.open (INPUT_FILE.replace(INPUT_FILE.end()-3, INPUT_FILE.end(), "fsm"));
	out_file_cs_nodes.open (INPUT_FILE.replace(INPUT_FILE.end()-3, INPUT_FILE.end(), "nodes.csv"));
	out_file_cs_edges.open (INPUT_FILE.replace(INPUT_FILE.end()-3, INPUT_FILE.end(), "edges.csv"));
	out_file_execluded.open (INPUT_FILE.replace(INPUT_FILE.end()-3, INPUT_FILE.end(), "xcludes.csv"));
	
	// Writing Parameters
	for(size_t i=0; i< SS_DIM; i++){
		
		out_file_fsm << "x" << i << "(" << symXUX.getNofGridPoints()[i] <<") Nat ";
		double first_point = symXUX.getFirstGridPoint()[i];
		double last_point = symXUX.getLastGridPoint()[i];
		double current_point;
		double eta = symXUX.getEta()[i];;
		size_t j=0;
		vector<double> dimGridPoints;
		do{			
			current_point = first_point + j*eta;			
			dimGridPoints.push_back(current_point);
			out_file_fsm << "\"" << current_point << "\" ";
			j++;
		}while(current_point != last_point);
		X_GridPoints.push_back(dimGridPoints);
		
		if(i != SS_DIM-1)
			out_file_fsm << endl;
	}
	
	// Parameters Separator
	out_file_fsm << endl << "---";
	
	// States
	vector<double> empty_vector;
	size_t global_cnt = 0;
	
	out_file_cs_nodes << "id,label" << endl;
	
	PrintGridPoints(X_GridPoints, 0, empty_vector, global_cnt);
	cout << "State-space size: " << global_cnt << endl;
	
	// States Separator
	out_file_fsm << endl << "---";
	
	// Transitions
	out_file_cs_edges << "Source,Target,label" << endl;
	out_file_execluded << "Source,Target,label" << endl;
	
	PrintTransitions(symXUX, X_GridPoints);
	out_file_fsm << endl;
	
	cout << "Done !" << endl;
	

	
	return 1;
}
