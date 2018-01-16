/*
 * ex4.cc
 *
 *  created on: 11.01.2016
 *      author: M.Khaled
 *
 *	An example to demonstrate the code generation from the SENSE controller.
 *  You must first run the example ROBOT from the examples of SENSE and generate the controller file.
 */

#include <vector>
#include <iostream>
#include "BDD2Implement.hh"

#define NSC_MAX 2
#define NCA_MAX 2

#define isDIM 2
#define ssDIM 2
#define ncs_ssDIM ((ssDIM*NSC_MAX)+(isDIM*NSC_MAX))


#define NBDD_FILE "../../../../examples/prolonged_ncs/robot/robot_contr.nbdd"
#define DCONTR_FILE "robot_controller_determinized.bdd"
#define OUT_VHDL_FILE "robot_controller.vhdl"

int main() {
	Cudd mgr;

	BDDReader reader(mgr, NBDD_FILE, BDD_FILE_TYPE::SENSE_NBDD);
	BDD contr = reader.ReadBdd();
	
	size_t nBddvars_states  = reader.getNumBddVars(0, ncs_ssDIM);
	size_t nBddvars_actions = reader.getNumBddVars(ncs_ssDIM, isDIM);

	cout << "nBddvars_states : " << nBddvars_states << endl;
	cout << "nBddvars_actions: " << nBddvars_actions << endl;

	BddDeterminizer determinizer(mgr, contr, nBddvars_states, nBddvars_actions);
	contr = determinizer.determinize(DETERMINIZE_METHOD::RANDOM, true, 1);


	BddToHDL converter(mgr, contr, nBddvars_states, nBddvars_actions);
	converter.GenerateRawVHDL(OUT_VHDL_FILE, true, 1);

	cout << "done !";
	return 1;
}
