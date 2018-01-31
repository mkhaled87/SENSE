/*
 * SCOTS2Interface.hh
 *
 *  created on: 30.10.2016
 *      author: M.Khaled
 *
 * Classes to interfe SCOTS-based BDD to be used within SENSE.
 */


#ifndef SCOTS2INTERFACE_HH_
#define SCOTS2INTERFACE_HH_

#include <vector>
#include <algorithm>
#include <sstream>
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <iomanip>
#include <memory>

#include "dddmp.h"
#include "cuddObj.hh"
#include "CuddMintermIterator.hh"

using namespace std;
using abs_type=std::uint64_t;

/* general definitions */
#define SCOTS_FH_VERSION    "v0.2"
#define SCOTS_FH_SYMBOL     "#"
#define SCOTS_FH_SEPERATOR  ";"
#define SCOTS_FH_EXTENSION  ".scs"
#define SCOTS_FH_BDD_EXTENSION  ".bdd"

#define SCOTS_FH_KEY        "#SCOTS:"
#define SCOTS_FH_TYPE       "#TYPE:"
#define SCOTS_FH_MEMBER     "#MEMBER:"
#define SCOTS_FH_VECTOR     "#VECTOR:"
#define SCOTS_FH_ARRAY      "#ARRAY:"
#define SCOTS_FH_MATRIX     "#MATRIX:"
#define SCOTS_FH_BEGIN      "#BEGIN:"
#define SCOTS_FH_END        "#END"

/* StaticController definitions */
#define  SCOTS_SC_TYPE          "STATICCONTROLLER"

/* WinningDomain definitions */
#define  SCOTS_WD_TYPE          "WINNINGDOMAIN"
#define  SCOTS_WD_DATA          "DATA"

/* UniformGrid definitions */
#define SCOTS_UG_TYPE         "UNIFORMGRID"
#define SCOTS_UG_DIM          "DIM"
#define SCOTS_UG_ETA          "ETA"
#define SCOTS_UG_LOWER_LEFT   "LOWER_LEFT"
#define SCOTS_UG_UPPER_RIGHT  "UPPER_RIGHT"

/* SCOTS2_SymbolicSet definitions */
#define SCOTS_SS_TYPE         "SYMBOLICSET"
#define SCOTS_SS_BDD_VAR_ID   "BDD_VAR_ID_IN_DIM_"

/* TransitionFunction definitions */
#define SCOTS_TF_TYPE         "TRANSITIONFUNCTION"
#define SCOTS_TF_NO_STATES    "NO_STATES"
#define SCOTS_TF_NO_INPUTS    "NO_INPUTS"
#define SCOTS_TF_NO_TRANS     "NO_TRANS"
#define SCOTS_TF_NO_PRE       "NO_PRE"
#define SCOTS_TF_NO_POST      "NO_POST"
#define SCOTS_TF_PRE_PTR      "PRE_PTR"
#define SCOTS_TF_PRE          "PRE"

/* Grid points definitions */
#define  SCOTS_GP_TYPE        "SET_OF_GRIDPOINTS"
#define  SCOTS_GP_DATA        "GRIDPOINTS"

/**
 * @class UniformGrid 
 *
 * @brief Holds the information of a uniform grid confined by the hyper-interval
 * [lb, ub]
 *
 *
 * Grid point alignment: 
 * - the grid points are distributed uniformly in each dimension 
 * - the origin is a grid point (not necessarily contained in the set) 
 * - the distance of the grid points in each dimension i is defined by eta[i] 
 *
 * Each grid point is associated with an ID of type abs_type (default
 * abs_type=uint32_t). The abs_type need to be an integral type and implicitly determines an upper bound on the number of
 * grid points that are representable by the UniformGrid.
 *
 *
 * A UniformGrid is often used to encode the state alphabet (or real quantizer
 * symbols) of an abstraction. In that context, we interpret the grid points as
 * centers of cells and the grid point IDs as cell IDs. The radius of the cells
 * is given by \f$\eta/2\f$. 
 *
 * The member functions itox and xtoi are used to map between the IDs and the
 * grid points/center of the cells.
 *
 * See 
 * - the manual in <a href="./../../manual/manual.pdf">manual</a>
 * - http://arxiv.org/abs/1503.03715 for theoretical background 
 **/
class UniformGrid {
protected:
  /** @brief dimension of the Eucleadian space **/
  int m_dim;                
  /** @brief m_dim-dimensional vector containing the grid node distances **/
  std::unique_ptr<double[]> m_eta;                      
  /** @brief m_dim-dimensional vector containing the real values of the first grid point **/
  std::unique_ptr<double[]> m_first;        
  /** @brief scots::abs_type array[m_dim] containing the number of grid points in each dimension **/
  std::unique_ptr<abs_type[]> m_no_grid_points;        
  /** @brief array recursively defined by: m_NN[0]=1; m_NN[i]=m_NN[i-1}*no_grid_points[i-1]; **/
  std::unique_ptr<abs_type[]> m_NN;                       

public:
  /* @cond  EXCLUDE from doxygen */
  /* default constructor */
  UniformGrid() : m_dim(0),
                  m_eta(nullptr), 
                  m_first(nullptr),
                  m_no_grid_points(nullptr),
                  m_NN(nullptr) { }
  /* destructor */
  virtual
  ~UniformGrid() = default;
  /* copy constructor */
  UniformGrid(const UniformGrid& other) : UniformGrid() {
    *this=other;
  }
  /* move constructor */
  UniformGrid(UniformGrid&& other) : UniformGrid() {
    *this=std::move(other);
  }
  /* copy assignment operator */
  UniformGrid& operator=(const UniformGrid &other) {
    if(this==&other)
      return *this;
    m_dim=other.m_dim;
    if(m_dim != 0) {
      m_eta.reset(new double[m_dim]);
      m_first.reset(new double[m_dim]);
      m_no_grid_points.reset(new abs_type[m_dim]);
      m_NN.reset(new abs_type[m_dim]);
      for(int i=0; i<m_dim; i++) {
        m_eta[i] = other.m_eta[i];
        m_first[i]  = other.m_first[i];
        m_no_grid_points[i]  = other.m_no_grid_points[i];
        m_NN[i]  = other.m_NN[i];
      }
    } 
    return *this;
  }
  /* create UniformGrid from other by projection on the dimension specified in dim */
  UniformGrid(const UniformGrid &other, const std::vector<int>& dim) : UniformGrid() {
    m_dim=dim.size();
    if(m_dim != 0) {
      m_eta.reset(new double[m_dim]);
      m_first.reset(new double[m_dim]);
      m_no_grid_points.reset(new abs_type[m_dim]);
      m_NN.reset(new abs_type[m_dim]);
      for(int i=0; i<m_dim; i++) {
        m_eta[i] = other.m_eta[dim[i]];
        m_first[i]  = other.m_first[dim[i]];
        m_no_grid_points[i]  = other.m_no_grid_points[dim[i]];
      }
      calc_nn();
    } 
  }
  /* move assignment operator */
  UniformGrid& operator=(UniformGrid&& other) {
    m_dim=std::move(other.m_dim);
    m_eta=std::move(other.m_eta);
    m_first=std::move(other.m_first);
    m_no_grid_points=std::move(other.m_no_grid_points);
    m_NN=std::move(other.m_NN);
    return *this;
  } 
  /* @endcond */

  /**
   * @brief provide uniform grid parameters and domain defining hyper-interval 
   * 
   * @param dim   - dimension of the real space
   * @param lb    - lower-left corner of the hyper-interval confining the uniform grid
   * @param ub    - upper-right corner of the hyper-interval confining the uniform grid
   * @param eta   - grid point distances
   **/
  template<class grid_point_t>
  UniformGrid(const int dim,
              const grid_point_t& lb,
              const grid_point_t& ub,
              const grid_point_t& eta) : UniformGrid() {
    m_dim = dim;
    if(m_dim != 0) {
      /* check inut arguments */
      for(int index=0; index<dim; index++) {
        if(eta[index] <= 0) 
          throw std::runtime_error("\nscots::UniformGrid: eta must have positive entries.");
        if(lb[index] > ub[index]) 
          throw std::runtime_error("\nscots::UniformGrid: lower-left bound must be less than or equal to upper-right bound.");
      }
      m_eta.reset(new double[m_dim]);
      m_first.reset(new double[m_dim]);
      m_no_grid_points.reset(new abs_type[m_dim]);
      m_NN.reset(new abs_type[m_dim]);

      /* determine number grid points in each dimension */
      std::size_t no_l, no_u;
      int sign_l, sign_u;
      for(int index=0; index<m_dim; index++) {
        m_eta[index] = eta[index];
        /* rounding heuristic */
        double delta = m_eta[index]/1E10;
        try {
          /* get sign */
          sign_l = (lb[index] > 0) ? 1 : ((lb[index] < 0) ? -1 : 0);
          /* compute number of grid points from zero to lower bound */
          no_l=std::llround(std::abs(lb[index])/eta[index]+sign_l*(0.5-delta));
        } catch (...) {
          std::ostringstream os;
          os << "\nscots::UniformGrid: something wrong in the division of " << lb[index] << " by " << eta[index] ;
          throw std::runtime_error(os.str().c_str());
        }
        try {
          /* get sign */
          sign_u = (ub[index] > 0) ? 1 : ((ub[index] < 0) ? -1 : 0);
          /* compute number of grid points from zero to upper bound */
          no_u=std::llround(std::abs(ub[index])/eta[index]-sign_u*(0.5-delta));
        } catch (...) {
          std::ostringstream os;
          os << "\nscots::UniformGrid: something wrong in the division of " << ub[index] << " by " << eta[index] ;
          throw std::runtime_error(os.str().c_str());
        }
        /* check if number of grid points in dimension index does not exceed max representable by abs_type  */
        if((sign_u*no_u-sign_l*no_l+1) > std::numeric_limits<abs_type>::max()) {
          std::ostringstream os;
          throw std::runtime_error("\nscots::UniformGrid: number of grid points exceeds maximum value of abs_type.");
        }
        m_no_grid_points[index] = sign_u*no_u-sign_l*no_l+1;
        /* first grid point coordinates */
        m_first[index]= (double)sign_l*(double)no_l*eta[index];
      }
      calc_nn();
    } else {
      throw std::runtime_error("\nscots::UniformGrid: grid dimension has to be greater than zero (using non-default constructor)");
    }
  }

  /** @brief compute the index associated with a grid point **/
  template<class grid_point_t>
  abs_type xtoi(const grid_point_t& x) const {
    abs_type id = 0;
    double d_id;
    double eta_h;

    for(int k=0; k<m_dim; k++) {
      d_id = x[k]-m_first[k];
      eta_h = m_eta[k]/2.0;

      if ( d_id <= -eta_h || d_id >= m_no_grid_points[k]*m_eta[k]+eta_h ) {
        std::ostringstream os;
        os << "\nscots::UniformGrid: state ";
        for(int i=0; i<m_dim; i++) {
          os << x[i] << " ";
        }
        os << "is outside uniform grid.";
        throw std::runtime_error(os.str().c_str());
      }
      id += static_cast<abs_type>((d_id+eta_h )/m_eta[k])*m_NN[k];
    }
    return id;
  }

  /** @brief compute the grid point associated with a index **/
  template<class grid_point_t>
  void itox(abs_type id, grid_point_t& x) const {
    /* map index id to grid point */
    abs_type num;
    for(int k = m_dim-1; k > 0; k--) {
      num=id/m_NN[k];
      id=id%m_NN[k];
      x[k]=m_first[k]+num*m_eta[k];
    }
    num=id;
    x[0]=m_first[0]+num*m_eta[0];
  }

  /** @brief compute the grid point associated with a index **/
  void itox(abs_type id, std::vector<double>& x) const {
    x.resize(m_dim);
    /* map index id to grid point */
    abs_type num;
    for(int k = m_dim-1; k > 0; k--) {
      num=id/m_NN[k];
      id=id%m_NN[k];
      x[k]=m_first[k]+num*m_eta[k];
    }
    num=id;
    x[0]=m_first[0]+num*m_eta[0];
  }

  /** @brief do a index to state conversion for vectors **/
  template<class grid_point_t>
  std::vector<grid_point_t> ItoX(std::vector<abs_type>& Ivector) const{

        std::vector<grid_point_t> Xvector;
        grid_point_t x;

        for(abs_type k=0; k<Ivector.size(); k++) {
          itox<grid_point_t>(Ivector[k],x);
          Xvector.push_back(x);
        }
        return Xvector;
  }

  /** @brief do a state to index conversion for vectors **/
  template<class grid_point_t>
  std::vector<abs_type> XtoI(std::vector<grid_point_t>& Xvector) const{

        std::vector<abs_type> Ivector;

        for(abs_type k=0; k<Xvector.size(); k++) {
          Ivector.push_back(xtoi<grid_point_t>(Xvector[k]));
        }
        return Ivector;
  }

  /** @brief creates console output with grid information **/
  void print_info() const {
    std::cout << "Distance of grid points (eta): ";
    for(int i=0; i<m_dim; i++) {
      std::cout << m_eta[i] << " ";
    }
    std::cout << "\nLower-left grid point: ";
    for(int i=0; i<m_dim; i++) {
      std::cout << m_first[i] << " ";
    }
    std::cout << "\nUpper-right grid point: ";
    for(int i=0; i<m_dim; i++) {
      std::cout << m_first[i]+m_eta[i]*(m_no_grid_points[i]-1) << " ";
    }
    std::cout << "\nNumber of grid points in each dimension: ";
    for(int i=0; i<m_dim; i++) {
        std::cout << m_no_grid_points[i] << " ";
    }
    std::cout << "\nNumber of grid points: "<< total_no_grid_points() << std::endl;
  }

  /** @name get functions **/
  //@{
  int get_dim() const {
    return m_dim;
  }
  /* total number of grid points */
  abs_type size() const {
    return total_no_grid_points();
  }
  std::vector<double> get_eta() const {
    std::vector<double> eta;
    for(int i=0; i<m_dim; i++) {
      eta.push_back(m_eta[i]);
    }
    return eta;
  }
  std::vector<double> get_lower_left() const {
    std::vector<double> lower_left;
    for(int i=0; i<m_dim; i++) {
      lower_left.push_back(m_first[i]);
    }
    return lower_left;
  }
  std::vector<double> get_upper_right() const {
    std::vector<double> upper_right;
    for(int i=0; i<m_dim; i++) {
      upper_right.push_back(m_first[i]+m_eta[i]*(m_no_grid_points[i]-1));
    }
    return upper_right;
  }
  std::vector<abs_type> get_no_gp_per_dim() const {
    std::vector<abs_type> no_grid_points;
    for(int i=0; i<m_dim; i++) {
      no_grid_points.push_back(m_no_grid_points[i]);
    }
    return no_grid_points;
  }
  std::vector<abs_type> get_nn() const {
    std::vector<abs_type> NN;
    for(int i=0; i<m_dim; i++) {
      NN.push_back(m_NN[i]);
    }
    return NN;
  }
  //@}

protected:
  void calc_nn() {
    /* compute m_NN */
    abs_type max = std::numeric_limits<abs_type>::max();
    abs_type total=1;
    for(int i=0; i<m_dim; i++) {
      m_NN[i] = total;
      /* check overflow */
      if(total > (max/m_no_grid_points[i])) {
        throw std::runtime_error("\nscots::UniformGrid: number of grid points exceeds maximum value of abs_type (defined in UniformGrid.hh).");
      }
      total *= m_no_grid_points[i];
    }
  }

private:
  /** @brief helper function to calculate the overall number of grid points **/
  abs_type total_no_grid_points() const {
    abs_type total=1;
    for(int i=0; i<m_dim; i++) {
      total *= m_no_grid_points[i];
    }
    return total;
  }
};


/*The IntegerInterval class*/
/** @class IntegerInterval 
 *
 *  @brief A class to represent the integers in a closed integer interval [lb; ub] as BDDs. \n
 *  The least significant bit is the BDD variable with the highest ID number.\n
 *  The most significant bit is the BDD variable with the lowest ID number.
 * 
 **/
template<class int_type >
class IntegerInterval {
private:
  /* lower bound of interval */
  int_type m_lb;
  /* upper bound of interval */
  int_type m_ub;
  /* number of integers in the interval */
  int_type m_size;
  /* m_bdd_vars contains the BDD variables */
  std::vector<BDD> m_bdd_vars;
  /* m_bdd_var_id contains the BDD variable IDs */
  std::vector<unsigned int> m_bdd_var_id;
  /* an array[m_size] containing the BDD representation of each element */
  std::vector<BDD> m_int_to_bdd;
  /* helper function */
  void add_one(int* phase, unsigned int num) {
    int carry = 1;
    for(int i=num-1; carry && (i>=0) ;i--) {
      if(phase[i]) {
        phase[i]=0;
        carry=1;
      } else {
        phase[i]=1;
        carry=0;
      }
    }
  }

public:
  /* @cond  EXCLUDE from doxygen */
  /* default constructor */
  IntegerInterval() : m_lb(0),  
                      m_ub(0), 
                      m_bdd_vars{},
                      m_bdd_var_id{},
                      m_int_to_bdd{} { }
  /* @endcond */
    
  /** @brief Instantiate the IntegerInterval with the integer interval [lb;ub]\n 
   *  optionally, provide the bdd variable ids to reprsent the integer interval **/
  IntegerInterval(const Cudd& manager,
                  int_type lb, int_type ub,
                  const std::vector<unsigned int>& bdd_var_id = {}) :
                  m_lb(lb), m_ub(ub), m_bdd_var_id(bdd_var_id) {
    /* compute necessar no of BDD variables to represent the integer in the interval  */ 
    m_size = ub-lb+1;
    unsigned int no_bdd_var = m_bdd_var_id.size();
    if(!no_bdd_var) {
      int_type x = ub-lb;
      while(x) {
        x>>=1u;
        no_bdd_var++;
      }
      if(!no_bdd_var) {
        no_bdd_var=1;
      }
    }
    /* get new BDD variable IDs */
    m_bdd_vars.resize(no_bdd_var);
    m_bdd_var_id.resize(no_bdd_var);
    std::unique_ptr<BDD[]> vars{ new BDD[no_bdd_var] }; 
    for(unsigned int j=0; j<no_bdd_var; j++) {
      /* copy ids from bdd_var_id or create new ones */
      if(!bdd_var_id.size()) {
        /* get new IDs */
        vars[j] = manager.bddVar();
        m_bdd_var_id[j]=vars[j].NodeReadIndex();
      } else {
        /* copy IDs from bdd_var_id */
        vars[j] = manager.bddVar(bdd_var_id[j]);
      }
      m_bdd_vars[j]=vars[j];
    }
    /* create BDDs to represent the integers in the interval */
    std::unique_ptr<int[]> phase{ new int[no_bdd_var] () }; 
    m_int_to_bdd.resize(m_size);
    for(int_type i=0; i<m_size; i++) {
      m_int_to_bdd[i] = manager.bddComputeCube(vars.get(),phase.get(),no_bdd_var);
      add_one(phase.get(),no_bdd_var);
      //m_int_to_bdd[i].PrintMinterm();
    }
  }

  /** @brief maps an integer in the interval to its BDD representation **/
  BDD int_to_bdd(int_type i) const {
    return m_int_to_bdd[i-m_lb];
  }

  /** @brief maps a (sub)interval in [lb; ub]  to its BDD representation **/
  BDD interval_to_bdd(const Cudd& manager, int_type lb, int_type ub) const {
    return manager.Interval(m_bdd_vars, static_cast<unsigned int>(lb)-m_lb, static_cast<unsigned int>(ub)-m_lb);
  }

  /** @brief prints the BDD variable IDs used to represent the integer interval */
  void print_bdd_IDs() const {
    std::cout << "BDD variable IDs: ";
    for(size_t j=0; j<m_bdd_var_id.size(); j++)  {
      std::cout << m_bdd_var_id[j] << " ";
    }
    std::cout << "\n";
  }
 
  /** @brief get a vector with the BDD variables used to represent the integer interval **/
  std::vector<BDD> get_bdd_vars() const {
    return m_bdd_vars;
  }

  /** @brief get a vector with the BDD variable IDs used to represent the integer interval **/
  std::vector<unsigned int> get_bdd_var_ids() const {
    return m_bdd_var_id;
  }

  /** @brief get number of BDD variables used to represent the integer interval **/
  int_type get_no_bdd_vars() const {
    return m_bdd_vars.size();
  }

  /** @brief get a BDD that encodes all elements of the integer interval **/
  BDD get_all_elements() const {
    BDD elements = m_int_to_bdd[0];
    for(const auto& bdd : m_int_to_bdd)
      elements = elements | bdd;
    return elements;
  }

  /** @brief get a vector with strings for BDD variable names\n as used in slugs for integer intervals 
   *
   *  For example, the integer interval variable x:0...4 declared in slugs is encoded by 3 BDD variables
   *  with names (from LSB to MSB) x@0.0.4, x@1, x@2
   *  
   **/
  std::vector<std::string> get_slugs_var_names() const {
    auto no_bdd_var=m_bdd_vars.size();
    std::vector<std::string> var_names(no_bdd_var);
    var_names[no_bdd_var-1]=std::string{"@0."}+std::to_string(m_lb)+std::string{"."}+std::to_string(m_ub);
    for(size_t i=0; i<(no_bdd_var-1); i++) {
      var_names[i]=std::string{"@"}+std::to_string((no_bdd_var-i-1));
    }
    return var_names;
  }

}; /* close class def */




/** 
 *  @class SCOTS2_SymbolicSet 
 *
 *  @brief The UniformGrid class with the capability to represent sets of grid
 *  points as BDDs. See UniformGrid for details.
 *
 **/
class SCOTS2_SymbolicSet : public UniformGrid {
private:
  /* a vector of IntegerIntervals - one for each dimension */
  std::vector<IntegerInterval<abs_type>> m_bdd_interval;
  /* a vector containing the slugs variable names */
  std::vector<std::string> m_slugs_var_names;
public:
  /** @brief construct SCOTS2_SymbolicSet with a Cudd manager **/
  SCOTS2_SymbolicSet() : UniformGrid(), m_bdd_interval{}, m_slugs_var_names{} { }

  /** @brief create a SCOTS2_SymbolicSet from other by projecting it onto the dimension in dim **/
  SCOTS2_SymbolicSet(const SCOTS2_SymbolicSet& other, std::vector<int> dim) :
              UniformGrid(other,dim), m_bdd_interval{}, m_slugs_var_names{} {
    for(const auto& i : dim) {
      m_bdd_interval.push_back(other.m_bdd_interval[i]);
      if(other.m_slugs_var_names.size()>static_cast<size_t>(i))
        m_slugs_var_names.push_back(other.m_slugs_var_names[i]);
    }
  }
  
  /** @brief create a SCOTS2_SymbolicSet and initialize m_bdd_interval with intervals **/
  SCOTS2_SymbolicSet(const UniformGrid& grid,
              const std::vector<IntegerInterval<abs_type>>& intervals) :
              UniformGrid(grid), m_bdd_interval(intervals), m_slugs_var_names{} { 
  }

  /**
   * @brief provide BDD variable manager and  UniformGrid parameters 
   * 
   * @param manager  - BDD variable manager 
   * @param dim      - dimension of the real space
   * @param lb       - lower-left corner of the hyper-interval confining the uniform grid
   * @param ub       - upper-right corner of the hyper-interval confining the uniform grid
   * @param eta      - grid point distances
   * OPTIONAL
   * @param names    - vector of size dim with names of integer variables used to interface with slugs
   **/
  template<class grid_point_t>
  SCOTS2_SymbolicSet(const Cudd& manager,
              const int dim,
              const grid_point_t& lb,
              const grid_point_t& ub,
              const grid_point_t& eta,
              const std::vector<std::string> names = {}) :
              UniformGrid(dim,lb,ub,eta), m_bdd_interval{}, m_slugs_var_names(names) {
    for(int i=0; i<m_dim; i++) 
      m_bdd_interval.emplace_back(manager,abs_type{0},m_no_grid_points[i]-1);
  }

  /**
   * @brief provide BDD variable manager and UniformGrid 
   * 
   * @param manager  - BDD variable manager 
   * @param grid     - UnfiormGrid
   * OPTIONAL
   * @param names    - vector of size dim with names of integer BDD variables used to interface with slugs
   **/
  SCOTS2_SymbolicSet(const Cudd& manager,
              const UniformGrid& grid,
              const std::vector<std::string> names = {}) :
              UniformGrid(grid), m_bdd_interval{}, m_slugs_var_names(names) {
    for(int i=0; i<m_dim; i++) 
      m_bdd_interval.emplace_back(manager,abs_type{0},m_no_grid_points[i]-1);
  }
  /**
   * @brief construct product of two SymbolicSets
   * 
   * The instantiated SCOTS2_SymbolicSet represents the Cartesian product of the
   * SCOTS2_SymbolicSet set1 and the SCOTS2_SymbolicSet set2
   * 
   * @param set1  - SCOTS2_SymbolicSet
   * @param set2  - SCOTS2_SymbolicSet
   **/
  SCOTS2_SymbolicSet(const SCOTS2_SymbolicSet& set1, const SCOTS2_SymbolicSet& set2) {
    m_dim = set1.m_dim + set2.m_dim;
    m_eta.reset(new double[m_dim]);
    m_first.reset(new double[m_dim]);
    m_no_grid_points.reset(new abs_type[m_dim]);
    m_NN.reset(new abs_type[m_dim]);
    for(int i=0; i<set1.m_dim; i++) {
      m_eta[i] = set1.m_eta[i];
      m_first[i]  = set1.m_first[i];
      m_no_grid_points[i]  = set1.m_no_grid_points[i];
      m_bdd_interval.push_back(set1.m_bdd_interval[i]);
      }
    for(int j=set1.m_dim, i=0; i<set2.m_dim; i++) {
      m_eta[j+i] = set2.m_eta[i];
      m_first[j+i]  = set2.m_first[i];
      m_no_grid_points[j+i]  = set2.m_no_grid_points[i];
      m_bdd_interval.push_back(set2.m_bdd_interval[i]);
    }
    calc_nn();
    /* copy slugs names */
    if(set1.m_slugs_var_names.size() || set2.m_slugs_var_names.size()) {
      if(set1.m_slugs_var_names.size())
        m_slugs_var_names=set1.m_slugs_var_names;
      else
        m_slugs_var_names.resize(set1.m_dim, std::string{'d'});
      if(set2.m_slugs_var_names.size())
        m_slugs_var_names.insert(m_slugs_var_names.end(), set2.m_slugs_var_names.begin(), set2.m_slugs_var_names.end());
      else
        m_slugs_var_names.resize(set1.m_dim+set2.m_dim, std::string{'d'});
    }

  }

  /** @brief print information about the symbolic set **/
  void print_info(int verbose=0) const {
    UniformGrid::print_info();
    std::cout << "Number of BDD variables per dimension ";
    for(int i=0; i<m_dim; i++) {
      std::cout << m_bdd_interval[i].get_no_bdd_vars() << " ";
    }
    if(verbose) {
      std::vector<int> no_bdd_var{0};
      std::cout << "\n";
      for(int i=0; i<m_dim; i++) {
        std::cout << "Dim " << i+1 << ": ";
        m_bdd_interval[i].print_bdd_IDs();
        no_bdd_var.push_back(no_bdd_var.back()+m_bdd_interval[i].get_no_bdd_vars());
      }
      if(m_slugs_var_names.size()) {
        auto slugs_names = get_slugs_var_names();
        std::cout << "Bdd variable names (to interface with slugs)\n";
        for(int i=0; i<m_dim; i++) {
          std::cout << "Integer var in dim " << i+1 << ": ";
          for(int j=no_bdd_var[i]; j<no_bdd_var[i+1]; j++) {
              std::cout << slugs_names[j] << " ";
          }
          std::cout << std::endl;
        }
      }
    }
    std::cout << "\n";
  }

  /** @brief function to obtain a BDD representation of the grid point id **/
  BDD id_to_bdd(abs_type id) const {
    abs_type num;
    int k=m_dim-1;
    /* k= m_dim -1 */
    num=id/m_NN[k];
    id=id%m_NN[k];
    BDD bdd = m_bdd_interval[k].int_to_bdd(num);
    for(k=m_dim-2; k >= 0; k--) {
      num=id/m_NN[k];
      id=id%m_NN[k];
      bdd = bdd & m_bdd_interval[k].int_to_bdd(num);
    }
    return bdd;
  }

  /** @brief get a BDD representation of the grid points  whose IDs are 
   *  an element of the integer hyper-interval [lb; ub] **/
  BDD interval_to_bdd(const Cudd& manager,
                      const std::vector<abs_type>& lb,   
                      const std::vector<abs_type>& ub) const {
    BDD bdd = m_bdd_interval[0].interval_to_bdd(manager,lb[0],ub[0]);
    for(int i=1; i<m_dim; i++) 
      bdd = bdd & m_bdd_interval[i].interval_to_bdd(manager,lb[i],ub[i]);
    return bdd;
  }

  /**
   * @brief obtain a BDD representation of the grid points whose grid point IDs
   * evaluate to true in the lambda expression
   * \verbatim [](const abs_type& i) -> bool \endverbatim
   **/
  template<class F>
  BDD ap_to_bdd(const Cudd& manager, const F& atomic_prop) const {
    BDD result = manager.bddZero();; 
    for(abs_type i=0; i<size(); i++) {
      if(atomic_prop(i)) 
        result = result | id_to_bdd(i);
    }
    return result;
  }


  /** @brief get a vector of grid points that are encoded in the BDD
   *
   *  The return vector is of size (number of grid points) x n where n is the  dimension.\n 
   *  The grid points are stacked on top of each other, i.e., the first n
   *  entries of the return vector represent the first grid point.
   **/
  std::vector<double> bdd_to_grid_points(const Cudd& manager, BDD bdd) {
    if((!get_no_bdd_vars()) || bdd==manager.bddZero())
      return {};
    /* disable reordering (if enabled) */
    Cudd_ReorderingType *method=nullptr;
    if(manager.ReorderingStatus(method))
      manager.AutodynDisable();
    /* get variable ids */
    auto var_id = get_bdd_var_ids();
    /* find the variables in the support of the BDD but outside the SCOTS2_SymbolicSet */
    auto support_id = bdd.SupportIndices();
    std::vector<BDD> out{}; 
    for(const auto& id : support_id) {
        if(std::find(std::begin(var_id), std::end(var_id), id)==std::end(var_id))
          out.emplace_back(manager.bddVar(id));
    }
    /* remove those variables from the bdd */
    if(out.size()) 
      bdd = bdd.ExistAbstract(manager.computeCube(out));
    /* limit the grid points in the grid */
    for(const auto& interval : m_bdd_interval) 
      bdd = bdd & interval.get_all_elements();
    /* init the vector of grid points to be returned */
    abs_type no_gp = get_size(manager,bdd);
    std::vector<double> gp(no_gp*m_dim);
    for(abs_type i=0; i<no_gp; i++) {
      for(int j=0; j<m_dim; j++)
        gp[i*m_dim+j]=m_first[j];
    }
    /* set up iteration to iterate over BDD cubes */
    DdManager* dd = manager.getManager();
    int *cube;
    CUDD_VALUE_TYPE value;
    DdGen *gen;
    abs_type counter=0;
    /* iterate over BDD cubes */
    Cudd_ForeachCube(dd,bdd.getNode(),gen,cube,value) {
      abs_type offset=1;
      for(int i=0; i<m_dim; i++) {
        unsigned int no_vars = m_bdd_interval[i].get_no_bdd_vars();
        for (unsigned int j=0; j<no_vars; j++) {
          unsigned int id = m_bdd_interval[i].get_bdd_var_ids()[j];
          if(cube[id]==1) {
            for(abs_type k=0; k<offset; k++) {
              gp[(counter+k)*m_dim+i]+=(abs_type{1}<<(no_vars-1-j))*m_eta[i];
            }
          }
          /* take care of don't care */
          if(cube[id]==2) {
            for(abs_type k=0; k<offset; k++) {
              for(int l=0; l<=i; l++) {
                gp[(counter+k+offset)*m_dim+l]=gp[(counter+k)*m_dim+l];
              }
            }
            for(abs_type k=0; k<offset; k++) {
              gp[(counter+k+offset)*m_dim+i]+=(abs_type{1}<<(no_vars-1-j))*m_eta[i];
            }
            offset=(offset<<1);
          }
        }
      }
      counter+=offset;
    }
    /* reactivate reordering if it was enabled */
    if(method!=nullptr)
      manager.AutodynEnable(*method);
    return gp;
  }

  /** @brief projection of the set of grid points onto the dimension in dim+1,
   * e.g., to project onto the 1st and 2nd coordinate, set dim = {0,1}  **/
  std::vector<double> projection(const Cudd& manager, const BDD& bdd, std::vector<int> dim) const {
    if(!dim.size())
      return {};
    return SCOTS2_SymbolicSet(*this,dim).bdd_to_grid_points(manager,bdd);
  }

  /**
   * @brief restriction of the set of grid points (represented as the BDD bdd) to x
   *
   * Consider the set \f$S\f$ represented by the BDD bdd, being a subset of the
   * n-dimensional hyper-interval 
   * \f$
   *  S\subseteq ( [lb_0,ub_0] \times \cdots [lb_{n-1},ub_{n-1}] ) \cap \eta  \mathbb Z^n
   * \f$.
   * Let \f$x=(x_{j_0},\ldots, x_{j_{m-1}})\f$ with \f$m<n\f$ and \f$p=n-m\f$. In
   * the default setting, the indices \f$j_i\f$ are set to \f$j_i=i\f$. Then the
   * function returns the grid points in the p-dimensional hyper-interval\n\n
   * \f$
   *  S(x):= \{ y\mid (x_{j_0},\ldots, x_{j_m},y_{0},\ldots,y_{p})\in S \}.
   * \f$
   * \n\n
   * Note that \f$S(x) \subseteq [lb_{m},ub_{m}]\times\cdots\times
   * [lb_{n-1},ub_{n-1}] ) \cap \eta  \mathbb Z^n\f$. Optionally, provide the
   * m-dimensional vector domain defining the indices
   * \f$j_0,\ldots,j_{m-1}\f$.
   * For example, if \f$n=4\f$, \f$m=2\f$ and \f$j_0=1,j_1=3\f$, then the
   * function returns the grid points in \n\n
   * \f$
   *  S(x):= \{ (y_0,y_1)\mid (y_0,x_1,y_1,x_3)\in S \}.
   * \f$
   * 
   * @param bdd      the BDD that is used to map 
   * @param x        the element to which the relation is restricted \n
   *                 if domain is not provided, then grid_point_t needs to have grid_point_t::size() method
   * @param domain   Optinally, provide the indices over which the elements of
   *                   x are interpreted
   * @result          vector of grid points  \f$ S(x) \f$ 
   **/
  template<class grid_point_t>
  std::vector<double> restriction(const Cudd& manager,
                                  const BDD& bdd, 
                                  const grid_point_t& x,  
                                  std::vector<int> domain = {}) const {
    /* compute indices in domain/codomain */
    std::vector<int> codomain {};
    /* fill in default values */
    if(domain.size()==0) {
      domain.resize(x.size());
      std::iota(std::begin(domain),std::end(domain),0);
      codomain.resize(m_dim-x.size());
      std::iota(std::begin(codomain),std::end(codomain),x.size());
    } else {
      for(int i=0; i<m_dim; i++) {
        if(std::find(std::begin(domain), std::end(domain), i) == std::end(domain))
          codomain.push_back(i);
      }
    }
    /* create SCOTS2_SymbolicSet of the domain and codmain */
    SCOTS2_SymbolicSet set_dom(*this,domain);
    SCOTS2_SymbolicSet set_codom(*this,codomain);
    /* extract bdd with restriction */
    abs_type id = set_dom.xtoi(x);
    BDD restricted = bdd & set_dom.id_to_bdd(id);
    restricted = restricted.ExistAbstract(set_dom.get_cube(manager));
    /* map restricted BDD to grid points */
    return set_codom.bdd_to_grid_points(manager,restricted);
  }

  /** @brief get number of BDD variables used to represent the SCOTS2_SymbolicSet **/
  abs_type get_no_bdd_vars() const {
    abs_type num = 0;
    for(const auto& interval : m_bdd_interval) 
      num+=interval.get_no_bdd_vars();
    return num;
  }

  /** @brief get number of BDD variables used to represent the SCOTS2_SymbolicSet **/
  std::vector<BDD> get_bdd_vars() const {
    std::vector<BDD> vars {};
    for(const auto& interval : m_bdd_interval) {
      std::vector<BDD> var = interval.get_bdd_vars();
      for(const auto& p : var) {
        vars.push_back(p);
      }
    }
    return vars;
  }

  /** @brief get number of BDD variables used to represent the SCOTS2_SymbolicSet **/
  std::vector<unsigned int> get_bdd_var_ids() const {
    std::vector<unsigned int> var_id {};
    for(const auto& interval : m_bdd_interval) {
      for(const auto& id : interval.get_bdd_var_ids()) 
        var_id.push_back(id);
    }
    return var_id;
  }

  /** @brief get a vector of IDs that corresponding to the grid points encoded by the BDD **/
  std::vector<abs_type> bdd_to_id(const Cudd& manager, BDD bdd) {
    if((!get_no_bdd_vars()) || bdd==manager.bddZero())
      return {};
    /* disable reordering (if enabled) */
    Cudd_ReorderingType *method=nullptr;
    if(manager.ReorderingStatus(method))
      manager.AutodynDisable();
    /* get variable ids */
    auto var_id = get_bdd_var_ids();
    /* find the variables in the support of the BDD but outside the SCOTS2_SymbolicSet */
    auto support_id = bdd.SupportIndices();
    std::vector<BDD> out{}; 
    for(const auto& id : support_id) {
      if(std::find(std::begin(var_id), std::end(var_id), id)==std::end(var_id))
        out.emplace_back(manager.bddVar(id));
    }
    /* remove those variables from the bdd */
    if(out.size()) 
      bdd = bdd.ExistAbstract(manager.computeCube(out));
    /* limit the grid points in the grid */
    for(const auto& interval : m_bdd_interval) 
      bdd = bdd & interval.get_all_elements();
    /* init the vector of grid points to be returned */
    abs_type no_id = get_size(manager,bdd);
    std::vector<abs_type> IDs(no_id,0);

    /* set up iteration to iterate over BDD cubes */
    DdManager* dd = manager.getManager();
    int *cube;
    CUDD_VALUE_TYPE value;
    DdGen *gen;
    abs_type counter=0;
    /* iterate over BDD cubes */
    Cudd_ForeachCube(dd,bdd.getNode(),gen,cube,value) {
      abs_type offset=1;
      for(int i=0; i<m_dim; i++) {
        unsigned int no_vars = m_bdd_interval[i].get_no_bdd_vars();
        for (unsigned int j=0; j<no_vars; j++) {
          unsigned int id = m_bdd_interval[i].get_bdd_var_ids()[j];
          if(cube[id]==1) {
            for(abs_type k=0; k<offset; k++) {
              IDs[counter+k]+=(abs_type{1}<<(no_vars-1-j))*m_NN[i];
            }
          }
          /* take care of don't care */
          if(cube[id]==2) {
            for(abs_type k=0; k<offset; k++) {
              for(int l=0; l<=i; l++) {
                IDs[counter+k+offset]=IDs[counter+k];
              }
            }
            for(abs_type k=0; k<offset; k++) {
              IDs[counter+k+offset]+=(abs_type{1}<<(no_vars-1-j))*m_NN[i];
            }
            offset=(offset<<1);
          }
        }
      }
      counter+=offset;
    }

    /* reactivate reordering if it was enabled */
    if(method!=nullptr)
      manager.AutodynEnable(*method);
    return IDs;
  }


  /** @brief get number grid points represented by the BDD  **/
  abs_type get_size(const Cudd& manager, BDD bdd) const {
    /* find the variables in the support of the BDD but outside the SCOTS2_SymbolicSet */
    auto var_id = get_bdd_var_ids();
    auto support_id = bdd.SupportIndices();
    std::vector<BDD> out{}; 
    for(const auto& id : support_id) {
        if(std::find(std::begin(var_id), std::end(var_id), id)==std::end(var_id))
          out.emplace_back(manager.bddVar(id));
    }
    /* remove those variables from the bdd */
    if(out.size()) 
      bdd = bdd.ExistAbstract(manager.computeCube(out));
    /* limit the grid points in the grid */
    for(const auto& interval : m_bdd_interval) 
      bdd = bdd & interval.get_all_elements();
    return static_cast<abs_type>(bdd.CountMinterm(get_no_bdd_vars()));
  } 

  /** @brief limit the support of the bdd to the variables in the SCOTS2_SymbolicSet  **/
  void clean(const Cudd& manager, BDD &bdd) const {
    /* find the variables in the support of the BDD but outside the SCOTS2_SymbolicSet */
    auto var_id = get_bdd_var_ids();
    auto support_id = bdd.SupportIndices();
    std::vector<BDD> out{}; 
    for(const auto& id : support_id) {
        if(std::find(std::begin(var_id), std::end(var_id), id)==std::end(var_id))
          out.emplace_back(manager.bddVar(id));
    }
    /* remove those variables from the bdd */
    if(out.size()) 
      bdd = bdd.ExistAbstract(manager.computeCube(out));
    /* limit the grid points in the grid */
    for(const auto& interval : m_bdd_interval) 
      bdd = bdd & interval.get_all_elements();
  } 

  /** @brief get IntegerInterval  **/
  std::vector<IntegerInterval<abs_type>> get_bdd_intervals() const {
    return m_bdd_interval;
  }
  /** @brief get cube BDD with the BDD variables of the SCOTS2_SymbolicSet **/
  BDD get_cube(const Cudd& manager) const {
    return manager.computeCube(get_bdd_vars());
  }

  /** @brief set bdd variable names (used to interface with slugs) **/
  void set_slugs_var_names(const std::vector<std::string>& var_names) { 
    m_slugs_var_names = var_names;
  }

  /** @brief get bdd variable names (used to interface with slugs) **/
  std::vector<std::string> get_slugs_var_names() const { 
    std::vector<std::string> var_names {};
    if(m_slugs_var_names.size()) {
      for(size_t i=0; i<m_bdd_interval.size(); i++) {
        for(const auto& bdd_names : m_bdd_interval[i].get_slugs_var_names()) {
          if(m_slugs_var_names[i].back()=='\'') {
            std::string tmp = m_slugs_var_names[i];
            /* remove prime from name  */
            tmp.pop_back();
            var_names.push_back(tmp+bdd_names);
            /* add prime at the end of name  */
            var_names.back().push_back('\'');
          }
          else 
            var_names.push_back(m_slugs_var_names[i]+bdd_names);
        }
      }
    } 
    return var_names;
  }

}; /* close class def */



/* The SCOTS2_FileHandler class stores the filename */
class SCOTS2_FileHandler {
protected:
  std::string m_filename{};
public:
  SCOTS2_FileHandler(const std::string& filename) : m_filename(filename) {};

  std::string get_filename() {
    return m_filename;
  }
};

/* The SCOTS2_FileWriter class is used to write information into files */
class SCOTS2_FileWriter : public SCOTS2_FileHandler {
private:
  std::ofstream m_file;
public:
  SCOTS2_FileWriter(const std::string& filename) : SCOTS2_FileHandler(filename) {};
  bool create() {
    m_file.close();
    m_file.open(m_filename.append(SCOTS_FH_EXTENSION),std::fstream::out);
    return m_file.is_open();
  }
  bool open() {
    m_file.close();
    m_file.open(m_filename.append(SCOTS_FH_EXTENSION),std::fstream::app);
    return m_file.is_open();
  }
  void close() {
    m_file.close();
  }
  bool add_TEXT(const std::string& text) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_KEY << text <<"\n";
      return true;
    }
    return false;
  }
  bool add_VERSION(void) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_KEY << SCOTS_FH_VERSION <<"\n";
      return true;
    }
    return false;
  }
  bool add_TYPE(const std::string& type) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_TYPE << type << "\n";
      return true;
    }
    return false;
  }
  template<class T>
  bool add_MEMBER(const std::string& name,T &&member) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_MEMBER << name << "\n";
      m_file << member << "\n";
      return true;
    }
    return false;
  }
  template<class T>
  bool add_VECTOR(const std::string& name, const std::vector<T>& vector) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_VECTOR << name << "\n";
      m_file << SCOTS_FH_BEGIN << vector.size() << "\n";
      m_file <<  std::setprecision(std::numeric_limits<T>::max_digits10);
      for(size_t index = 0; index < vector.size(); index++) {
        m_file << vector[index] << "\n";
      }
      m_file << SCOTS_FH_END << "\n";
      return true;
    }
    return false;
  }
  template<class T>
  bool add_ARRAY(const std::string& name, T&& array, size_t array_size) {
    if(m_file.is_open()) {
      m_file << SCOTS_FH_ARRAY << name << "\n";
      m_file << SCOTS_FH_BEGIN << array_size << "\n";
      for(size_t index = 0; index < array_size; index++) {
        m_file << array[index] << "\n";
      }
      m_file << SCOTS_FH_END << "\n";
      return true;
    }
    return false;
  }
  template<class T>
  bool add_WINNINGDOMAIN(const std::string& name, 
                         const std::vector<T>& vector,
                         const std::vector<bool>& matrix,
                         size_t row, size_t col) {
    if(vector.size()!=row) {
      return false;
    }
    if(m_file.is_open()) {
      m_file << SCOTS_FH_MATRIX << name << "\n";
      if(matrix.size()==row*col) {
        m_file << SCOTS_FH_BEGIN << row << " " << col << "\n";
        for(size_t i=0; i<row; i++) {
          if(vector[i]!=std::numeric_limits<T>::max()) {
            m_file << i << " ";
            for(size_t j=0; j<col; j++) {
              if(matrix[i*col+j]) {
                m_file << j << " ";
              }
            }
            m_file << "\n";
          }
        }
      } else {
        m_file << SCOTS_FH_BEGIN << row << " " << 1 << "\n";
        for(size_t i=0; i<row; i++) {
          if(vector[i]!=std::numeric_limits<T>::max()) {
            m_file << i << " " << vector[i] << "\n";
          }
        }
      }
      m_file << SCOTS_FH_END << "\n";
      return true;
    }
    return false;
  }
  /* functions are only availabe if BDD support is activated */  
  bool add_BDD(const Cudd& manager, const BDD& bdd, char** varnames, char mode='B') {
    /* disable reordering (if enabled) */
    Cudd_ReorderingType *method=nullptr;
    if(manager.ReorderingStatus(method))
      manager.AutodynDisable();
    /* open filename */
    std::string filename = m_filename.append(SCOTS_FH_BDD_EXTENSION);
    FILE *file = fopen (filename.c_str(),"w");
    if(!file) 
      return false;
    int store = Dddmp_cuddBddStore(bdd.manager(),NULL,
                                   bdd.getNode(),varnames,NULL,
                                   (int)mode,DDDMP_VARIDS,NULL,file);
    if(fclose(file))
      return false;
    if (store!=DDDMP_SUCCESS)  
      return false;
    /* reactivate reordering if it was enabled */
    if(method!=nullptr)
      manager.AutodynEnable(*method);

    return true;
  }
};

/* The SCOTS2_FileReader class is used to read information from files */
class SCOTS2_FileReader : public SCOTS2_FileHandler {
private:
  std::ifstream       m_file;
  std::string         m_line;

  bool skip_offset(size_t& offset) {
    for(size_t index = 0; index<offset; index++) {
      if(!(m_file.ignore(std::numeric_limits<std::streamsize>::max(),'\n'))) {
        return false;
      }
    }
    return true;
  }
  void back_to_first_line() {
    m_file.clear();
    m_file.seekg(0,std::ios::beg);
  }

public:
  SCOTS2_FileReader(const std::string& filename) : SCOTS2_FileHandler(filename) {};
  bool open() {
    m_file.open(m_filename.append(SCOTS_FH_EXTENSION));
    return m_file.good();
  }
  void close() {
    m_file.close();
  }
  size_t get_VERSION(double& version, size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t counter=0;
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line.find(SCOTS_FH_KEY)!=std::string::npos) {
          std::istringstream stream(m_line.substr(m_line.find(":")+1));
          stream >> version;
          return counter;
        }
      }
      return 0;
    }
    return 0;
  }
  size_t get_TYPE(std::string& string, size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      string.clear();
      size_t counter=0;
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line.find(SCOTS_FH_TYPE)!=std::string::npos) {
          std::istringstream stream(m_line.substr(m_line.find(":")+1));
          stream >> string;
          return counter;
        }
      }
    }
    return 0;
  }
  size_t find_TEXTPOS(const std::string& text, size_t offset=0) {
    std::string match = SCOTS_FH_KEY + text;
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t counter=0;
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line==match) {
          return counter;
        }
      }
    }
    return 0;
  }
  template<class T>
  size_t get_MEMBER(const std::string& member_name, T& member, size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t counter=0;
      std::string match = SCOTS_FH_MEMBER + member_name;
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line.find(match)!=std::string::npos) {
          if(std::getline(m_file,m_line)) {
            std::istringstream stream(m_line);
            stream >> member;
            counter++;
            return counter;
          }
        }
      }
    }
    return 0;
  }
  template<class T>
  size_t get_VECTOR(const std::string& vector_name, std::vector<T>& vector, size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t size=0;
      size_t counter=0;
      std::string match = SCOTS_FH_VECTOR + vector_name;
      vector.clear();
      /* first get the size of the vector */
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line.find(match)!=std::string::npos) {
          if(std::getline(m_file,m_line)) {
            if(m_line.find(SCOTS_FH_BEGIN)!=std::string::npos) {
              std::istringstream stream(m_line.substr(m_line.find(":")+1));
              stream >> size;
              counter++;
              break;
            }
          }
        }
      }
      if(m_file.eof() || size==0) {
        return 0;
      }
      /* now fill the vector */
      vector.resize(size);
      for(size_t index = 0; index < size; index++) {
        if(std::getline(m_file,m_line)) {
          if(m_line.find(SCOTS_FH_SYMBOL)!=std::string::npos) {
            vector.clear();
            return 0;
          }
          counter++;
          std::istringstream stream(m_line);
          stream >> vector[index];
        } else {
          vector.clear();
          return 0;
        }
      }
      if(std::getline(m_file,m_line)) {
        if(m_line.find(SCOTS_FH_END)!=std::string::npos) {
          counter++;
          return counter;
        }
      }
    }
    return 0;
  }
  template<class T>
  size_t get_ARRAY(const std::string array_name, T& array, size_t array_size, size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t size=0;
      size_t counter=0;
      std::string match = SCOTS_FH_ARRAY + array_name;
      /* check size */
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line.find(match)!=std::string::npos) {
          if(std::getline(m_file,m_line)) {
            if(m_line.find(SCOTS_FH_BEGIN)!=std::string::npos) {
              std::istringstream stream(m_line.substr(m_line.find(":")+1));
              stream >> size;
              counter++;
              break;
            } 
          }
        }
      }
      if(m_file.eof() || size==0 || size!=array_size) {
        return 0;
      }
      /* fill array */
      for(size_t index = 0; index < size; index++) {
        if(std::getline(m_file,m_line)) {
          counter++;
          std::istringstream stream(m_line);
          stream >> array[index];
        } else {
          return 0;
        }
      }
      if(std::getline(m_file,m_line)) {
        if(m_line.find(SCOTS_FH_END)!=std::string::npos) {
          counter++;
          return counter+offset;
        }
      }
    }
    return 0;
  }
  template<class T>
  bool get_WINNINGDOMAIN(const std::string& name, 
                         std::vector<T>& vector,
                         std::vector<bool>& matrix,
                         T& row, 
                         T& col,
                         size_t offset=0) {
    back_to_first_line();
    if(skip_offset(offset)) {
      size_t counter=0;
      std::string match = SCOTS_FH_MATRIX + name;
      /* get size */
      while(std::getline(m_file,m_line)) {
        counter++;
        if(m_line==match) {
          if(std::getline(m_file,m_line)) {
            if(m_line.find(SCOTS_FH_BEGIN)!=std::string::npos) {
              std::istringstream stream(m_line.substr(m_line.find(":")+1));
              stream >> row >> col;
              counter++;
              break;
            }
          }
        }
      }
      /* nothing found */
      if(m_file.eof()) {
        return 0;
      }
      /* if col > 1 the boolean matrix[row][col] is necessary to represent the winning domain */
      if(col>1) {
        matrix.clear();
        matrix.resize(row*col,false);
      }
      vector.clear();
      vector.resize(row,std::numeric_limits<T>::max());
      T i,j;
      /* fill vector and matrix */
      while(std::getline(m_file,m_line)) {
        counter++;
        /* check if we reached end */
        if(m_line==SCOTS_FH_END) {
          return counter+offset;
        }
        std::istringstream stream(m_line);
        stream >> i;
        if(col>1) {
          while( stream >> j ) {
            matrix[i*col+j]=true;
            vector[i]=0;
          }
        } else {
          stream >> j;
          vector[i]=j;
        }
      }
    }
    return 0;
  }
  /* functions are only availabe if BDD support is activated */  
  bool get_BDD(const Cudd& manager, BDD& bdd, char mode='B') {
    /* disable reordering (if enabled) */
    Cudd_ReorderingType *method=nullptr;
    if(manager.ReorderingStatus(method))
      manager.AutodynDisable();

    /* open file1name */
    std::string filename = m_filename.append(SCOTS_FH_BDD_EXTENSION);
    FILE *file = fopen(filename.c_str(),"r");
    if(!file) 
      return false;


    DdNode *node =
    Dddmp_cuddBddLoad(manager.getManager(),
                      DDDMP_VAR_MATCHIDS,NULL,NULL,
                      NULL,(int)mode,NULL,file);
    fclose(file);
    if(!node) 
      return false;
    bdd=BDD(manager,node);
    /* reactivate reordering if it was enabled */
    if(method!=nullptr)
      manager.AutodynEnable(*method);

    return true;
  }
};


class SCOTS2_IO{
public:

static bool write_to_file(const SCOTS2_SymbolicSet& set, const std::string& filename) {
    SCOTS2_FileWriter writer(filename);
    if(writer.create()) {

        writer.add_VERSION();
        writer.add_TYPE(SCOTS_SS_TYPE);
        writer.add_MEMBER(SCOTS_UG_DIM,set.get_dim());
        writer.add_VECTOR(SCOTS_UG_ETA,set.get_eta());
        writer.add_VECTOR(SCOTS_UG_LOWER_LEFT,set.get_lower_left());
        writer.add_VECTOR(SCOTS_UG_UPPER_RIGHT,set.get_upper_right());
        auto intervals = set.get_bdd_intervals();
        for(int i=0;i<set.get_dim(); i++) {
            std::string s = SCOTS_SS_BDD_VAR_ID;
            writer.add_VECTOR(s.append(std::to_string(i+1)),intervals[i].get_bdd_var_ids());
        }

        writer.close();
        return true;
    }
    return false;
}

static bool write_to_file(const Cudd& manager, const SCOTS2_SymbolicSet& set, const BDD& bdd, const std::string& filename, char mode='B') {
    SCOTS2_FileWriter writer(filename);
    if(!write_to_file(set,filename)) {
        return false;
    }
   
   /* check if we should write slugs variable names */
   if(set.get_slugs_var_names().size()) {
    /* create data structure for variable names of the BDD variables */
    char** varnames = new char*[manager.ReadSize()];
    for(int i=0; i<manager.ReadSize(); i++) 
      varnames[i]=nullptr;
    /* create marker to keep track which variable names are allocated here */
    std::unique_ptr<bool[]> marker(new bool[manager.ReadSize()]());
     /* fill varnames with slugs variable names */
    auto slugs_names = set.get_slugs_var_names();
    auto bdd_ids = set.get_bdd_var_ids();
    for(size_t i=0; i<slugs_names.size(); i++) 
      varnames[bdd_ids[i]]=&slugs_names[i][0];
    for(int i=0; i<manager.ReadSize(); i++) {
      if(varnames[i]==nullptr) {
        varnames[i] = new char;
        varnames[i][0]='d';
        marker[i]=true;
      }
    }

    /* write BDD to file */
    if(!writer.add_BDD(manager,bdd,varnames,mode)) {
        return false;
    }

    /* clean variable names */
    for(int i=0; i<manager.ReadSize(); i++) {
      if(marker[i])
        delete varnames[i];
    }
    delete[] varnames;
   } else {
    /* write BDD to file */
    if(!writer.add_BDD(manager,bdd,nullptr,mode)) {
        return false;
    }
  }

  return true;
}



/** @brief read UniformGrid from a file via a FileReader **/
static bool read_from_file(UniformGrid& grid, const std::string& filename, size_t offset = 0) {
    SCOTS2_FileReader reader(filename);
    if(!reader.open()) {
        return false;
    }
    int dim;
    if(!reader.get_MEMBER(SCOTS_UG_DIM,dim,offset)) {
        return false;
    }
    std::vector<double> eta;
    if(!reader.get_VECTOR(SCOTS_UG_ETA,eta,offset)) {
        return false;
    }
    std::vector<double> lb;
    if(!reader.get_VECTOR(SCOTS_UG_LOWER_LEFT,lb,offset)) {
        return false;
    }
    std::vector<double> ub;
    if(!reader.get_VECTOR(SCOTS_UG_UPPER_RIGHT,ub,offset)) {
        return false;
    }
    reader.close();
    /* make sure that rounding in the UniformGrid constructor works correctly */
    for(int i=0; i<dim; i++) {
        lb[i]-=eta[i]/4.0;
        ub[i]+=eta[i]/4.0;
    }
    grid = UniformGrid(dim,lb,ub,eta);
    return true;
}



static bool read_from_file(const Cudd& manager, SCOTS2_SymbolicSet& set, const std::string& filename) {
    /* read UniformGrid from file */
    UniformGrid grid;
    if(!read_from_file(grid,filename)){
        return false;
    }
    /* read the BDD variable IDs and create IntegerInterval*/
    std::vector<IntegerInterval<abs_type>> bdd_interval{};
    SCOTS2_FileReader reader(filename);
    if(reader.open()) {
        size_t offset = 0;
        for(int i=0;i<grid.get_dim(); i++) {
            std::vector<unsigned int> var_id {};
            std::string s = SCOTS_SS_BDD_VAR_ID;
            offset=reader.get_VECTOR(s.append(std::to_string(i+1)),var_id,offset);
            if(!offset)
                return false;
            bdd_interval.emplace_back(manager,abs_type{0},grid.get_no_gp_per_dim()[i]-abs_type{1},var_id);
        }
        reader.close();
    } else {
        return false;
    }
    /* initialize SCOTS2_SymbolicSet  */
    set = SCOTS2_SymbolicSet(grid,bdd_interval);
    return true;
}

static bool read_from_file(const Cudd& manager, SCOTS2_SymbolicSet& set, BDD& bdd, const std::string& filename, char mode = 'B') {
    if(!read_from_file(manager,set,filename))
        return false;
    SCOTS2_FileReader reader(filename);
    if(!reader.get_BDD(manager,bdd,mode)) {
        return false;
    }
    set.clean(manager,bdd);
    return true;
}

};

#endif
