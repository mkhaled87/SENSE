/*
 * ncsFixedPoint.hh
 *
 *  created on: 11.11.2016
 *      author: M.Khaled
 */

#ifndef ncsFixPoint_HH_
#define ncsFixPoint_HH_

#include <array>
#include <iostream>
#include <stdexcept>

#include "cuddObj.hh"


/*
 * class: ncsFixPoint
 *
 * it provides fixed point computations to
 * synthesize controllers for NCS Models provided as BDD objects
 *
 */

class ncsFixPoint {
protected:
  /* var: ddmgr_ */
  Cudd* ddmgr_;

  /* var: permute 
   * stores the permutation array used to swap pre with post variables */
  int* permute_;

  /*vars: nisVars_, nssVars_*/
  /* store number of vars in the ss and is */
  size_t nisVars_, nssVars_;

  /*vars: postVars_, preVars_, inpVars_*/
  /* store bddVars for pre, post and input*/
  std::vector<size_t> postVars_;
  std::vector<size_t> preVars_;
  std::vector<size_t> inpVars_;

  /* helper BDDs */
  /* transition relation */
  BDD R_;

  /* transition relation with cubePost_ abstracted */
  BDD RR_;  

   /* cubes with input and post variables; used in the existential abstraction  */
  BDD cubePost_;
  BDD cubeInput_;
  
public:

  /* function: ncsFixPoint
   *
   * initialize the ncsFixPoint object with a <SymbolicModel> containing the
   * transition relation
   */
  ncsFixPoint(Cudd* cuddManager, BDD& transRelation, std::vector<size_t>& preVars, std::vector<size_t>& inpVars, std::vector<size_t>& postVars) {

	  if(postVars.size() != preVars.size()){
		  std::ostringstream os;
		  os << "Error: ncsFixPoint: preVars and postVars should have same number of bddVars.";
		  throw std::invalid_argument(os.str().c_str());
	  }


    ddmgr_ = cuddManager;
    nssVars_ = preVars.size();
    nisVars_ = inpVars.size();

    postVars_ = postVars;
    preVars_ = preVars;
    inpVars_ = inpVars;

     /* the permutation array */
    size_t n=ddmgr_->ReadSize();
    permute_ = new int[n];
    for(size_t i=0; i<n; i++)
      permute_[i]=i;
    for(size_t i=0; i<nssVars_; i++)
      permute_[preVars_[i]]=postVars_[i];
    /* create a cube with the input Vars */
    BDD* vars = new BDD[nisVars_];
    for (size_t i=0; i<nisVars_; i++)
      vars[i]=ddmgr_->bddVar(inpVars_[i]);
    cubeInput_ = ddmgr_->bddComputeCube(vars,NULL,nisVars_);
    delete[] vars;
    /* create a cube with the post Vars */
    vars = new BDD[nssVars_];
    for (size_t i=0; i<nssVars_; i++)
      vars[i]=ddmgr_->bddVar(postVars_[i]);
    cubePost_ = ddmgr_->bddComputeCube(vars,NULL,nssVars_);
    delete[] vars;

    /* copy the transition relation */
    R_ = transRelation;
    RR_ = R_.ExistAbstract(cubePost_);
  }

  ~ncsFixPoint() {
    delete[] permute_;
  }

  /* function: pre 
   *
   * computes the enforcable predecessor 
   *  
   * pre(Z) = { (x,u) | exists x': (x,u,x') in transitionRelation 
   *                    and (x,u,x') in transitionRelation  => x' in Z } 
   *
   */
  BDD pre(BDD Z)  {
    /* project onto state alphabet */
	BDD cbInpPost = cubePost_*cubeInput_;

    Z=Z.ExistAbstract(cbInpPost);

    /* swap variables */
    Z=Z.Permute(permute_);

    /* find the (state, inputs) pairs with a post outside the safe set */
    BDD nZ = !Z;

    BDD F = R_.AndAbstract(nZ,cubePost_);

    /* the remaining (state, input) pairs make up the pre */
    BDD nF = !F;

    BDD preZ= RR_.AndAbstract(nF,cubePost_);

    return preZ;
  }

  
  /* function: reach 
   *  
   * computation of the minimal fixed point mu Z.pre(Z) | T
   *
   */
  BDD reach(const BDD &T, int verbose)  {
    /* check if target is a subset of the state space */
    std::vector<unsigned int> sup = T.SupportIndices();
    for(size_t i=0; i<sup.size();i++) {
      int marker=0;
      for(size_t j=0; j<nssVars_; j++) {
        if (sup[i]==preVars_[j])
          marker=1;
      }
      if(!marker) {
        std::ostringstream os;
        os << "Error: reach: the target set depends on variables outside of the state space.";
        throw std::invalid_argument(os.str().c_str());
      }
    }
    if(verbose) 
      std::cout << "Iterations: ";

    BDD Z = ddmgr_->bddOne();
    BDD ZZ = ddmgr_->bddZero();
    /* the controller */
    BDD C = ddmgr_->bddZero();
    /* as long as not converged */
    size_t i;
    for(i=1; ZZ != Z; i++ ) {

      Z=ZZ;
      BDD the_pre = pre(Z);
      ZZ= the_pre | T;


      /* new (state/input) pairs */
      BDD N = ZZ & (!(C.ExistAbstract(cubeInput_)));
      /* add new (state/input) pairs to the controller */
      C=C | N;

      /* print progress */
      if(verbose) {
        std::cout << ".";
        std::flush(std::cout);
        if(!(i%80))
          std::cout << std::endl;
      }
    }
    if(verbose) 
      std::cout << " number: " << i << std::endl;
    return C;
  }


  /* function: safe
     *
     * computation of the maximal fixed point nu Z.pre(Z) & S
     *
     */
    BDD safe(BDD S, int verbose=0)  {
      /* check if safe is a subset of the state space */
      std::vector<unsigned int> sup = S.SupportIndices();
      for(size_t i=0; i<sup.size();i++) {
        int marker=0;
        for(size_t j=0; j<nssVars_; j++) {
          if (sup[i]==preVars_[j])
            marker=1;
        }
        if(!marker) {
            std::ostringstream os;
            os << "Error: safe: the inital set depends on variables  outside of the state space.";
            throw std::invalid_argument(os.str().c_str());
        }
      }
      if(verbose)
        std::cout << "Iterations: ";

      BDD Z = ddmgr_->bddZero();
      BDD ZZ = ddmgr_->bddOne();
      /* as long as not converged */
      size_t i;
      for(i=1; ZZ != Z; i++ ) {
        Z=ZZ;
        ZZ=pre(Z) & S;

        /* print progress */
        if(verbose) {
          std::cout << ".";
          std::flush(std::cout);
          if(!(i%80))
            std::cout << std::endl;
        }
      }
      if(verbose)
        std::cout << " number: " << i << std::endl;
      return Z;
    }


	/* function: persistance
	*
	* computation of the fixed point: mu X. nu Y. ( pre(Y) & T ) | pre(X)
	*
	*/
	BDD persistance(BDD T, int verbose=0)  {
		/* check if safe is a subset of the state space */
		std::vector<unsigned int> sup = T.SupportIndices();
		for(size_t i=0; i<sup.size();i++) {
		  int marker=0;
		  for(size_t j=0; j<nssVars_; j++) {
			if (sup[i]==preVars_[j])
			  marker=1;
		  }
		  if(!marker) {
			  std::ostringstream os;
			  os << "Error: safe: the inital set depends on variables  outside of the state space.";
			  throw std::invalid_argument(os.str().c_str());
		  }
		}
		if(verbose)
		  std::cout << "Iterations: ";

		size_t i,j;

		/* outer fp*/
		BDD X=ddmgr_->bddOne();
		BDD XX=ddmgr_->bddZero();

		/* inner fp*/
		BDD Y=ddmgr_->bddZero();
		BDD YY=ddmgr_->bddOne();

		/* the controller */
		BDD C=ddmgr_->bddZero();

		/* as long as not converged */
		for(i=1; XX != X; i++) {
		  X=XX;
		  BDD preX= pre(X);

		  /* init inner fp */
		  YY = ddmgr_->bddOne();
		  for(j=1; YY != Y; j++) {
			Y=YY;
			YY= ( pre(Y) & T ) | preX;
		  }
		  XX=YY;

		  /* remove all (state/input) pairs that have been added
		   * to the controller already in the previous iteration * */
		  BDD N = XX & (!(C.ExistAbstract(cubeInput_)));

		  /* add the remaining pairs to the controller */
		  C=C | N;

		    /* print progress */
			if(verbose) {
			  std::cout << ".";
			  std::flush(std::cout);
			  if(!(i%80))
				std::cout << std::endl;
			}
		}

		return C;
	}

	/* function: recurrence
	*
	* computation of the fixed point: nu X. mu Y. ( pre(X) & T ) | pre(Y)
	*
	*/
	BDD recurrence(BDD T, int verbose=0)  {
		/* check if safe is a subset of the state space */
		std::vector<unsigned int> sup = T.SupportIndices();
		for(size_t i=0; i<sup.size();i++) {
		  int marker=0;
		  for(size_t j=0; j<nssVars_; j++) {
			if (sup[i]==preVars_[j])
			  marker=1;
		  }
		  if(!marker) {
			  std::ostringstream os;
			  os << "Error: safe: the inital set depends on variables  outside of the state space.";
			  throw std::invalid_argument(os.str().c_str());
		  }
		}
		if(verbose)
		  std::cout << "Iterations: ";

		size_t i,j;

		/* outer fp*/
		BDD X=ddmgr_->bddZero();
		BDD XX=ddmgr_->bddOne();

		/* inner fp*/
		BDD Y=ddmgr_->bddOne();
		BDD YY=ddmgr_->bddZero();

		/* the controller */
		BDD C=ddmgr_->bddZero();

		/* as long as not converged */
		for(i=1; XX != X; i++) {
		  X=XX;
		  BDD preX=pre(X);

		  /* init inner fp */
		  YY = ddmgr_->bddZero();
		  for(j=1; YY != Y; j++) {
			Y=YY;
			YY= ( preX & T ) | pre(Y);
		  }
		  XX=YY;

		  /* remove all (state/input) pairs that have been added
		   * to the controller already in the previous iteration * */
		  BDD N = XX & (!(C.ExistAbstract(cubeInput_)));


		  /* add the remaining pairs to the controller */
		  C=C | N;

		    /* print progress */
			if(verbose) {
			  std::cout << ".";
			  std::flush(std::cout);
			  if(!(i%80))
				std::cout << std::endl;
			}
		}

		return C;
	}



	/* function: recurrence_conj
	*
	* computation of the fixed point: nu X.( Conj_{i=1,2,3....}  mu Y_i. ( pre(X) & T_i ) | pre(Y_i) )
	*
	*/
	std::vector<BDD> recurrence_conj(std::vector<BDD> vT, int verbose=0)  {
		/* check if safe is a subset of the state space */
		for(size_t k=0; k<vT.size(); k++){
			BDD T = vT[k];
			std::vector<unsigned int> sup = T.SupportIndices();
			for(size_t i=0; i<sup.size();i++) {
			  int marker=0;
			  for(size_t j=0; j<nssVars_; j++) {
				if (sup[i]==preVars_[j])
				  marker=1;
			  }
			  if(!marker) {
				  std::ostringstream os;
				  os << "Error: safe: the inital set depends on variables  outside of the state space.";
				  throw std::invalid_argument(os.str().c_str());
			  }
			}
		}
		if(verbose)
		  std::cout << "Iterations: ";

		size_t i,j;
		/* outer fp*/
		BDD X=ddmgr_->bddZero();
		BDD XX=ddmgr_->bddOne();

		/* inner fp*/
		std::vector<BDD>  vY(vT.size());
		std::vector<BDD> vYY(vT.size());

		/* the controller */
		std::vector<BDD>  vC(vT.size());

		/* init */
		for(size_t k=0; k<vT.size(); k++){
			vY[k] = ddmgr_->bddOne();
			vYY[k] = ddmgr_->bddZero();
			vC[k] = ddmgr_->bddZero();
		}

		/* outer FP : nu */
		for(i=1; XX != X; i++) {
			X=XX;
			BDD preX=pre(X);

			/* init inner fp: mu */
			for(size_t k=0; k<vT.size(); k++){
				vYY[k] = ddmgr_->bddZero();
				for(j=1; vYY[k] != vY[k]; j++) {
				  vY[k]  = vYY[k];
				  vYY[k] = (( preX & vT[k] )) | pre(vY[k]);
				  BDD N = vYY[k] & (!(vC[k].ExistAbstract(cubeInput_)));
				  vC[k] = vC[k] | N;
				}

			}

			XX = ddmgr_->bddOne();
			for(size_t k=0; k<vT.size(); k++)
				XX &= vYY[k];
		}

		return vC;
	}




}; /* close class def */

#endif /* ncsFixPoint_HH_ */
