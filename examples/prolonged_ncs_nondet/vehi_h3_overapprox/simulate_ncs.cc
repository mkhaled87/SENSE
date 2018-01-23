/* state space dim */
#define sDIM 3
#define iDIM 2


#include <array>
#include <deque>
#include <iostream>
#include <iomanip>
#include <string>

#include "cuddObj.hh"
Cudd mgr;

// To be able to simulate the NCS controller
#include "SENSE.hh"

// To be able to simulate the system by solving the ODE
#include "RungeKutta4.hh"

/* state space dim */
#define sDIM 3
#define iDIM 2

#define concrete_type double
#define SEPARATOR "*************************************************************************************************************************"
#define LONG_TAB  "\t\t\t\t\t"

/* data types for the ode solver */
typedef std::array<concrete_type,sDIM> x_type;
typedef std::array<concrete_type,iDIM> u_type;
double Inf = std::numeric_limits<double>::infinity();


// the samnpling time
const concrete_type tau = 0.3;

// the ODE solver
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);

// system and NCS dynamics
#define STATE_ETA_FINE {0.1, 0.1, 0.1}
#define INITIAL_STATE {0.5,4.2,-0.6}
#define INITIAL_INPUT {0.3, 0.0}

#define NSC_MAX 2
#define NCA_MAX 2
auto  vehicle_post = [](x_type &xpost, const x_type &x, const u_type &u) -> void {

  /* the ode describing the vehicle */
  auto rhs =[](x_type& xx,  const x_type &x, const u_type &u) -> void {
      concrete_type alpha=std::atan(std::tan(u[1])/2.0);
      xx[0] = u[0]*std::cos(alpha+x[2])/std::cos(alpha);
      xx[1] = u[0]*std::sin(alpha+x[2])/std::cos(alpha);
      xx[2] = u[0]*std::tan(u[1]);
  };
  
  xpost[0] = x[0];
  xpost[1] = x[1];
  xpost[2] = x[2];
  
  ode_solver(rhs,xpost,u);
};
auto radius_post = [](x_type &rpost, const x_type &r, const u_type &u) -> void {
    concrete_type c = std::abs(u[0]*std::sqrt(std::tan(u[1])*std::tan(u[1])/4.0+1));
    rpost[0] = r[0]+c*r[2]*0.3;
    rpost[1] = r[1]+c*r[2]*0.3;
	rpost[2] = r[2];
};

// some utility functions
void print_vector(const std::vector<concrete_type>& v){
	for(size_t i=0; i<v.size(); i++)
		std::cout << std::setw(-6) << v[i] << " ";
}
std::vector<concrete_type> concatenate_vectors(const std::vector<concrete_type>& v1, const std::vector<concrete_type>& v2){
	std::vector<concrete_type> ret;
	
	for(size_t i=0; i<v1.size(); i++)
		ret.push_back(v1[i]);
	
	for(size_t i=0; i<v2.size(); i++)
		ret.push_back(v2[i]);	
	
	return ret;
}
std::vector<concrete_type> state2vector(const x_type& state){
	std::vector<concrete_type> ret;
	
	for(size_t i=0; i<sDIM; i++)
		ret.push_back(state[i]);
	
	return ret;
}
std::vector<concrete_type> input2vector(const u_type& input){
	std::vector<concrete_type> ret;
	
	for(size_t i=0; i<iDIM; i++)
		ret.push_back(input[i]);
	
	return ret;
}
void print_state_time(const x_type& x, long double simtime ){
	std::cout << "At [" << std::setw(3) << simtime << " sec.], the state of system is ";
	print_vector(state2vector(x));
}
bool state_is_inf(const x_type& x){
  for(size_t i=0; i<sDIM; i++)
	  if(x[i] != Inf)
		return false;
		
  return true;
}

// ------------------------------------------------------------------
// the NCS controller used inside the refined controller
// ------------------------------------------------------------------
vector<vector<double>> get_NCS_ControlActions(std::vector<concrete_type> xu){
	
	static bool doneLoading = false;
	
	if(!doneLoading){
		std::cout << std::endl << LONG_TAB << "Loading the NCS Controller from the file ... ";
		std::flush(std::cout);
	}
	
	
	static Cudd mgr;
	static ncsController ncs_controller(mgr, "vehicle_ncs_controller.nbdd");
	doneLoading = true;
	
	std::vector<int> q_values;
	for(size_t i=0;i<NSC_MAX; i++)
		q_values.push_back(0);
	
	return ncs_controller.getInputs(xu, q_values);
}


// ------------------------------------------------------------------
// the system
// ------------------------------------------------------------------
void sim_system(x_type& out_state, const u_type& in_action){

  static bool dataLoaded = false;
  static SymbolicSet transitions;
  static SymbolicSet transitions_fine_1_tau;
  static SymbolicSet target;
  static SymbolicSet obstacles;
  static x_type x  = INITIAL_STATE;
  static x_type r  = STATE_ETA_FINE;
  static std::vector<size_t> xu_idx;  
  x_type xpost;
  x_type rpost;
  
  std::cout << std::endl << LONG_TAB << "-----------------------------------------";  
  std::cout << std::endl << LONG_TAB << "System:";  
  
  if(!dataLoaded){
	  std::cout << std::endl << LONG_TAB << "Loading data files for the system ... ";
	  std::flush(std::cout);
	  dataLoaded = true;
	  transitions.LoadFromFile(mgr, "scots-files/vehicle_rel.bdd");
	  transitions_fine_1_tau.LoadFromFile(mgr, "scots-files/vehicle_rel_fine_1_tau.bdd", 1);
      target.LoadFromFile(mgr, "scots-files/vehicle_ts.bdd");
      obstacles.LoadFromFile(mgr, "scots-files/vehicle_obst.bdd");	  
	  
	  for(size_t i=0; i<sDIM+iDIM; i++)
		xu_idx.push_back(i);	  
  }       
  
  // compute concrete post
  vehicle_post(xpost, x, in_action);
  radius_post(rpost, r, in_action);
  std::cout << std::endl << LONG_TAB << "concrete post state is ";
  print_vector(state2vector(xpost));
  std::cout << std::endl << LONG_TAB << "concrete rad  post  is ";
  print_vector(state2vector(rpost));
  
  // compute symbolic posts
  std::vector<std::vector<concrete_type>> symposts = transitions.setValuedMap(concatenate_vectors(state2vector(x), input2vector(in_action)),xu_idx);
  std::cout << std::endl << LONG_TAB << "symbolic posts (" << symposts.size() << ") are: ";
  for(size_t i=0;i<symposts.size(); i++){
	std::cout << std::endl << LONG_TAB << "\t";
	print_vector(symposts[i]);
  }	
  
  std::vector<std::vector<concrete_type>> symposts_fine = transitions_fine_1_tau.setValuedMap(concatenate_vectors(state2vector(x), input2vector(in_action)),xu_idx);
  std::cout << std::endl << LONG_TAB << "symbolic posts after 1*tau in fine abstraction (" << symposts_fine.size() << ") are: ";
  for(size_t i=0;i<symposts_fine.size(); i++){
	std::cout << std::endl << LONG_TAB << "\t";
	print_vector(symposts_fine[i]);
  }	  
  
  for(size_t i=0; i<sDIM; i++){
	out_state[i] = xpost[i];
	x[i] = xpost[i];
  }
  
  if(target.isElement(state2vector(out_state)))
	  throw "Simulation finished: reached the target !";
  
  if(obstacles.isElement(state2vector(out_state)))
	  throw "Simulation finished: reached an obstacle !";  
  
}

// ------------------------------------------------------------------
// the Quantizer
// ------------------------------------------------------------------
void sim_quantizer(x_type& out_state, const x_type& in_state){	
	static bool dataLoaded = false;  
	static SymbolicSet stateset;
	
	std::cout << std::endl << LONG_TAB << "-----------------------------------------";  
	std::cout << std::endl << LONG_TAB << "Quantizer:"; 	
	
	if(!dataLoaded){
	  dataLoaded = true;
	  std::cout << std::endl << LONG_TAB << "Loading data files for the quantizer ... ";
	  std::flush(std::cout);
	  stateset.LoadFromFile(mgr, "scots-files/vehicle_ss_fine_1_tau.bdd");
	}
	
	std::cout << std::endl << LONG_TAB << "Input concrete state: ";
	print_vector(state2vector(in_state));
	
	vector<int> minterm;
	double quantized[sDIM];
	stateset.elementToMinterm(minterm, state2vector(in_state));
	stateset.mintermToElement(minterm.data(), quantized);
	
	for(size_t i=0; i<sDIM; i++){
		out_state[i] = quantized[i];
	}	
	
	std::cout << std::endl << LONG_TAB << "Output quantized state: ";
	print_vector(state2vector(out_state));
}

// ------------------------------------------------------------------
// the channel SC
// ------------------------------------------------------------------
void sim_channel_sc(x_type& out_state, const x_type& in_state){	  
  static std::deque<x_type> SC_CHANNEL;
  static x_type q  = {Inf, Inf, Inf};
  
  // fill the NCS SC channel
  if(SC_CHANNEL.size() == 0)
	for(size_t i=0; i<NSC_MAX; i++)
	  SC_CHANNEL.push_back(q);
  
  std::cout << std::endl << LONG_TAB << "-----------------------------------------";  
  std::cout << std::endl << LONG_TAB << "SC Channel:";     
  
  std::cout << std::endl << LONG_TAB << "           => ["; print_vector(state2vector(in_state)); std::cout << "]";
  std::cout << std::endl;  
  
  for(size_t i=0; i<SC_CHANNEL.size(); i++){
	std::cout << std::endl << LONG_TAB << "               \t";  
	print_vector(state2vector(SC_CHANNEL[i]));	
  }
  
  // get last element
  for(size_t i=0; i<sDIM; i++)
	out_state[i] = SC_CHANNEL[SC_CHANNEL.size()-1][i];

  std::cout << std::endl;
  std::cout << std::endl << LONG_TAB << "           \t\t=> ["; print_vector(state2vector(out_state)); std::cout << "]";
  
  // push new element
  SC_CHANNEL.pop_back();
  SC_CHANNEL.push_front(in_state);  
}

// ------------------------------------------------------------------
// the controller
// ------------------------------------------------------------------
void sim_controller(u_type& out_action, const x_type& in_state){	
    
  static bool dataLoaded = false;
  static std::vector<size_t> x_idx;
  static std::vector<size_t> xu_idx; 
  static SymbolicSet controller;
  static SymbolicSet transitions_fine_1_tau;
  static u_type u0 = INITIAL_INPUT;
  static std::deque<u_type> inputMemory;
  
  std::cout << std::endl << LONG_TAB << "-----------------------------------------";  
  std::cout << std::endl << LONG_TAB << "Controller:";
  
  if(!dataLoaded){
	  std::cout << std::endl << LONG_TAB << "Loading data files for the controller ... ";
	  std::flush(std::cout);
	  dataLoaded = true;  	  
	  controller.LoadFromFile(mgr, "scots-files/vehicle_controller.bdd");	  
	  transitions_fine_1_tau.LoadFromFile(mgr, "scots-files/vehicle_rel_fine_1_tau.bdd", 1);
	  
	
	  for(size_t i=0; i<sDIM; i++)
		x_idx.push_back(i); 
		  
	  for(size_t i=0; i<sDIM+iDIM; i++)
		xu_idx.push_back(i);	  	
  
	  for(size_t i=0; i<NCA_MAX; i++)
		inputMemory.push_back(u0);
  }
  

  std::cout << std::endl << LONG_TAB << "Input memory is [";
  for(size_t i=0; i<inputMemory.size();i ++){
	print_vector(input2vector(inputMemory[i]));
	
	if(i<inputMemory.size()-1)
		std::cout << " | ";
  }	    
  std::cout << "]";
  
  if(!state_is_inf(in_state)){	  
	  std::cout << std::endl << LONG_TAB << "Controller recieved non-q state! ";
  	  
	  x_type x_sys, x_tmp;
	  x_tmp = in_state;
	  
	  // Simulate with the concrete model
	  std::cout << std::endl << LONG_TAB << "Detected Concrete states: ";
	  for(size_t i=0; i<NSC_MAX; i++){						/* state detector */
		u_type u =  inputMemory[inputMemory.size()-1-i];
		vehicle_post(x_sys, x_tmp, u);
		x_tmp = x_sys;		
		
		print_vector(state2vector(x_sys));		
		if(i<NSC_MAX-1)
			std::cout << " ==> ";
	  }
	  	  	  
	  
	  std::cout << std::endl << LONG_TAB << "Expected Concrete states: ";
	  for(size_t i=NSC_MAX; i<NSC_MAX+NCA_MAX; i++){		/* state predector */
		u_type u =  inputMemory[inputMemory.size()-1-i];
		vehicle_post(x_sys, x_tmp, u);
		x_tmp = x_sys;		
		
		print_vector(state2vector(x_sys));
		if(i<NSC_MAX+NCA_MAX-1)
			std::cout << " ==> ";
	  }
	  
	  // Simulate with the fine symbolic model
	  std::vector<x_type> ncs_states;
	  ncs_states.push_back(in_state);
	  std::cout << std::endl << LONG_TAB << "Detected symbolic states: ";
	  for(size_t i=0; i<NSC_MAX; i++){						/* state detector */		

		std::vector<std::vector<concrete_type>> symposts;
		symposts = transitions_fine_1_tau.setValuedMap(concatenate_vectors(state2vector(ncs_states[ncs_states.size()-1]), input2vector(inputMemory[inputMemory.size()-1-i])),xu_idx);
				
		if(symposts.size() == 0){
			throw "Simulation failed: Fine symbolic model has no post states !";  
		}
				
		for(size_t j=0; j<symposts.size(); j++){		
			std::cout << std::endl << LONG_TAB << " =(" << j + 1 << "/" << symposts.size() << ")=> ";		
			print_vector(symposts[j]);		
			
			if(j == 0){
				std::cout << " [chosen]";
				for(size_t k=0; k<sDIM; k++)
					x_tmp[k] = symposts[j][k];

				ncs_states.push_back(x_tmp);
			}
		}
	  }
	  	  
	  
	  std::vector<concrete_type> xu_vals;
	  for(size_t i=0; i<NSC_MAX; i++)
		  for(size_t j=0; j<sDIM; j++)
			xu_vals.push_back(ncs_states[NSC_MAX-i-1][j]);
	  
	  
	  for(size_t i=0; i<NCA_MAX; i++)
		  for(size_t j=0; j<iDIM; j++)
			xu_vals.push_back(inputMemory[i][j]);
	  
	  std::cout << std::endl << LONG_TAB << "NCS State [XU]: ";
	  print_vector(xu_vals);
	  
	  
		
	  // For original controller get all possible inputs + select one input	  
	  std::vector<std::vector<concrete_type>> inputs = controller.setValuedMap(state2vector(x_sys),x_idx);	  
	  if(inputs.size() == 0)
		  throw "Simulation failed: reached a state non-controllable in the original controller !";

	  std::cout << std::endl << LONG_TAB << "From the original controller, " << inputs.size() << " inputs are found and we select the input: ";
	  print_vector(inputs[0]);
	  
	  
	  // For NCS controller get all possible inputs + select one input	  
	  vector<vector<double>> ncs_inputs  = get_NCS_ControlActions(xu_vals);
	  if(ncs_inputs.size() == 0)
		  throw "Simulation failed: reached a state non-controllable in the NCS controller !";	  
	  
	  std::cout << std::endl << LONG_TAB << "From the NCS controller, " << ncs_inputs.size() << " inputs are found and we select the input: ";
	  print_vector(ncs_inputs[0]);	
	  
	  
	  // take one of the input
	  for(size_t i=0; i<iDIM; i++)
		  out_action[i] = inputs[0][i];	  
	  
	  // prepare for next
	  inputMemory.pop_back();
	  
  }else{
	  std::cout << std::endl << LONG_TAB << "Controller recieved a q state! ";
	  std::cout << "and selected the initial input: ";
	  print_vector(input2vector(u0));
	  
	  for(size_t i=0; i<iDIM; i++)
		  out_action[i] = u0[i];
	  
	  std::cout << std::endl;
  }
  
  // prepare for nexr
  inputMemory.push_front(out_action);
}

// ------------------------------------------------------------------
// the channel CA
// ------------------------------------------------------------------
void sim_channel_ca(u_type& out_action, const u_type& in_action){	
  
  static std::deque<u_type> CA_CHANNEL;
  static u_type u0 = INITIAL_INPUT;
  
  // fill the NCS CA channel
  if(CA_CHANNEL.size() == 0)
    for(size_t i=0; i<NCA_MAX; i++)
	  CA_CHANNEL.push_back(u0);
  
  
  std::cout << std::endl << LONG_TAB << "-----------------------------------------";  
  std::cout << std::endl << LONG_TAB << "CA Channel:";     
  std::cout << std::endl << LONG_TAB << "           => ["; print_vector(input2vector(in_action)); std::cout << "]";
  std::cout << std::endl;
  
  for(size_t i=0; i<CA_CHANNEL.size(); i++){
	std::cout << std::endl << LONG_TAB << "               \t";  
	print_vector(input2vector(CA_CHANNEL[i]));	
  }

  // read last element
  for(size_t i=0; i<iDIM; i++)
	out_action[i] = CA_CHANNEL[CA_CHANNEL.size()-1][i];    

  std::cout << std::endl;
  std::cout << std::endl << LONG_TAB << "           \t\t=> ["; print_vector(input2vector(out_action)); std::cout << "]";

  // push new element
  CA_CHANNEL.pop_back();
  CA_CHANNEL.push_front(in_action);  
 
}


int main() {    
  long double simtime = 0.0;

  // prepare for sim
  x_type x_system = INITIAL_STATE;
  x_type x_quantizer;
  x_type x_channel_sc;
  u_type u_controller;
  u_type u_channel_ca;
   
  // simulation loop
  std::cout << SEPARATOR << std::endl;
  while(true){
	try{
	  // ------------------------------------------------------------------
	  // current state of the system
	  // ------------------------------------------------------------------  
	  print_state_time(x_system, simtime);
	  	 
	  // ------------------------------------------------------------------
	  // Quantizer
	  // ------------------------------------------------------------------  
	  sim_quantizer(x_quantizer, x_system);
	  
	  // ------------------------------------------------------------------
	  // SC Channel
	  // ------------------------------------------------------------------  
	  sim_channel_sc(x_channel_sc, x_quantizer);
	  	  
	  // ------------------------------------------------------------------
	  // Controller
	  // ------------------------------------------------------------------  
	  sim_controller(u_controller, x_channel_sc);
	  
	  // ------------------------------------------------------------------
	  // CA Channel
	  // ------------------------------------------------------------------  
	  sim_channel_ca(u_channel_ca, u_controller);	  
	  	  
	  // ------------------------------------------------------------------
	  // The system
	  // ------------------------------------------------------------------  
	  sim_system(x_system, u_channel_ca);

	  // ------------------------------------------------------------------
	  // prepare for next loop
	  // ------------------------------------------------------------------	  
	  simtime += tau;	  
	  std::cout << std::endl;	 
	  std::cout << SEPARATOR << std::endl;
	}
	catch(const char* msg){
		std::cout << std::endl;
		std::cout << msg << std::endl;
		return 0;
	}
  }  
  return 0;
}





