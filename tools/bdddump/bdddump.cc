/*
 * bdddump.cc
 *
 *  created on: 16.08.2016
 *      author: M. Khaled
 *
 *********************************************************
 * This tool dumps the information in any SCOTS-based bdd file.
 */



#include <array>
#include <iostream>
#include <iomanip>

#include "cuddObj.hh"
#include "SymbolicSetInterface.hh"

using namespace std;

#define MAX_POINTS 20


void DumpBDDFile(const char* file, signed int maxpoints);
void DumpBDDObject(SymbolicSet &ss, signed int maxpoints);
void PrintBDD(string NameIt, BDD& bddObj);

int main(int argc, char* argv[]) {

  if (argc < 2){
    cout << "Error: No input arguements are found !" << endl;
    return 1;
  }

  size_t maxp;

  //system("clear");

  if(argc == 2)
  {
    maxp = MAX_POINTS;
    cout << "No maximum number of points specified, we use default of " << maxp << endl;
  }

  if(argc == 3)
  {
    maxp = atoi(argv[2]);
    cout << "Use maximum number of points of " << maxp << endl;
  }

  cout << "Dumping " << argv[1] << " ... " << endl;
  DumpBDDFile(argv[1], maxp);
  return 0;
}

void PrintBDD(string NameIt, BDD& bddObj){
	std::cout << "Info for BDD: " << NameIt << std::endl;
	std::cout << "\t    f : (" << bddObj << ")" << std::endl;
  	std::cout << "\tcover : " << std::endl;
  	bddObj.PrintCover();
  	std::cout << std::endl;
}

template<typename T>
void PrintArray(T* v, size_t size)
{
    for (size_t i=0; i<size; i++)
    {
        std::cout << setw(6) << v[i];
	if(i<size-1)
	    cout << ',';
    }
}

void DumpBDDFile(const char* file, signed int maxpoints)
{
	Cudd cuddManager;
	SymbolicSet ss;
	ss.LoadFromFile(cuddManager, file, 1);

	const size_t  dim = ss.getDimension();
	const double* Eta = ss.getEta();
	const double* FirstPoint  = ss.getFirstGridPoint();
	const double* LastPoint   = ss.getLastGridPoint();
	const size_t* nGridpoints = ss.getNofGridPoints();
	double ssSize = ss.getSize();
	const size_t* nBddVars    = ss.getNofBddVars();
	size_t** indBddVars = ss.getIndBddVars();
    vector<unsigned int> support_indicies = ss.getSymbolicSet().SupportIndices();

	cout << "-----------------------------------------------------------------------" << endl;
	cout << "Information from the file ( " << file << " ) with new BDD variable ids:" << endl;
	cout << "-----------------------------------------------------------------------" << endl;

 	cout << "  Dimenstion : " << dim << endl;

	cout << "         Eta : {";
	PrintArray(Eta,dim);
	cout << "}" << endl;

	cout << "  First/Last : (";
	PrintArray(FirstPoint,dim);
	cout << ") / (";
	PrintArray(LastPoint,dim);
	cout << ")" << endl;

	cout << " Grid Points : {";
	PrintArray(nGridpoints,dim);
	cout << "}" << endl;

	cout << "    BDD Vars : {";
	PrintArray(nBddVars,dim);
	cout << "}" << endl;

	cout << "Support Vars : {";
	PrintArray(support_indicies.data(),support_indicies.size());
	cout << "}" << endl;

	cout << "BDD Vars Ind : ";
		for(size_t i=0; i<dim; i++){
			cout << " dim." << i << " -> {";
			PrintArray(indBddVars[i],nBddVars[i]);
			cout << "}\t";
		}
	cout << endl;

	cout << "        Size : " << ssSize << " points in the grid" << endl;

	cout << "    BDD Dump : ";
		DumpBDDObject(ss, maxpoints);


	cout << "BDD PrintInf : ";
	BDD bddObj = ss.getSymbolicSet();
	PrintBDD("SymbolicSet_",bddObj);
	cout << "-----------------------------------------------------------------------" << endl;
}


void DumpBDDObject(SymbolicSet &ss, signed int maxpoints)
{
	const size_t dim = ss.getDimension();
	vector<size_t> ivars;
	size_t nvars = 0;
	bool monitor_max = (maxpoints == -1)?false:true;

	cout << " for BDD-vars ";


	for(size_t i=0; i<dim; i++)
	{
		nvars += ss.getNofBddVars()[i];
		for (size_t j=0; j<ss.getNofBddVars()[i]; j++)
		{
			ivars.push_back(ss.getIndBddVars()[i][j]);
			cout << ss.getIndBddVars()[i][j] << " ";
		}
	}
	cout << "[" << nvars << " vars]"<< endl;


	CuddMintermIterator cuddIterator(ss.getSymbolicSet(), ivars, nvars);
	//CuddMintermIterator cuddIterator(ss.getSymbolicSet());


	int* mterm = new int[nvars];
	double* element = new double[dim];

	size_t maxp = (size_t)maxpoints;
	size_t i=0;

	while(!cuddIterator.done())
	{
		if(monitor_max)
			if(i>=maxp)
				break;
			
		cuddIterator.copyMinterm(mterm);
		ss.mintermToElement(mterm, element);

		cout << "\t\t (";
		PrintArray(element,dim);
		cout << ") <-- ";
		cuddIterator.printMinterm();		

		++cuddIterator;
		++i;
	}

	delete mterm;
	delete element;
}




