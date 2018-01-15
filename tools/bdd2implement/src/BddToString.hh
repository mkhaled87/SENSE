#ifndef BDDTOSTRING_HH_
#define BDDTOSTRING_HH_

#include <iostream>
#include <string>
#include "cuddObj.hh"


using namespace std;

#if SIZEOF_VOID_P == 8 && SIZEOF_INT == 4
typedef uint32_t DdHalfWord;
#else
typedef uint16_t DdHalfWord;
#endif
/**
 * @brief Decision diagram node.
 */
struct DdNode {
    DdHalfWord index;		/**< variable index */
};
typedef struct DdNode DdNode;

//#define cuddT(node) ((node)->type.kids.T)
//#define cuddE(node) ((node)->type.kids.E)

/*
 * a class to generate the string of the boolran function from BDD to set of BDDs for some specific output bits
 */
class BddToStringConverter{
	Cudd* pCuddManager;
	DdNode* DD_ZERO;
	DdNode* DD_ONE;

	const string LOGIC_ONE_STR = "1";
	const string LOGIC_ZERO_STR = "0";

	string AND_STR, OR_STR, NOT_STR;

public:
	BddToStringConverter(Cudd& cuddManager, string AND_SYMBOL_, string OR_SYMBOL_, string NOT_SYMBOL_){
		pCuddManager = &cuddManager;
		DD_ZERO = pCuddManager->bddZero().getNode();
		DD_ONE = pCuddManager->bddOne().getNode();
		AND_STR = AND_SYMBOL_;
		OR_STR = OR_SYMBOL_;
		NOT_STR = NOT_SYMBOL_;
	}

	string asShannonForm(BDD& bddFunc)
	{
		stringstream ss;
		int retval;

		if (bddFunc == pCuddManager->bddOne()) {
			ss << LOGIC_ONE_STR;
		} else if (bddFunc == !pCuddManager->bddOne() || bddFunc == pCuddManager->bddZero()) {
			ss << LOGIC_ZERO_STR;
		} else {
			ss << (Cudd_IsComplement(bddFunc.getNode()) ? (Cudd_bddIsVar(pCuddManager->getManager(), bddFunc.getRegularNode()) ? (NOT_STR + "(") : (NOT_STR + "(")) : "(");
			retval = recursiveShannonString(bddFunc.getRegularNode(),ss);
			if (retval == 0) {
				throw "Error while extracteing factored form string";
			}
			ss << (Cudd_IsComplement(bddFunc.getNode()) && !Cudd_bddIsVar(pCuddManager->getManager(), bddFunc.getRegularNode()) ? ")" : ")");
		}
		return(ss.str());
	}

private:


	int recursiveShannonString(DdNode* f, stringstream& ss)
	{
	    DdNode	*T, *E;
	    int		retval;
	    bool or_open = false;
	    bool p_open = false;

	    if (f == NULL)
	    	return(0);

	    T = Cudd_T(f);
	    E = Cudd_E(f);

	    if (T != DD_ZERO) {

			if (E != DD_ONE) {

				ss << "(";
				p_open = true;

				ss << 'x';
				ss << (unsigned) f->index;
			}
			if (T != DD_ONE) {
				ss << (E != DD_ONE ? (" " + AND_STR + " ") : "");
				ss << "(";

				retval = recursiveShannonString(T,ss);
				if (retval != 1)
					return(retval);

				ss << ")";
			}

			if(p_open)
				ss << ")";

			if (E == Cudd_Not(DD_ONE) || E == DD_ZERO)
				return(1);

			ss << " " << OR_STR << " (";
			or_open = true;
	    }

	    E = Cudd_Regular(E);
	    if (T != DD_ONE) {
				ss << NOT_STR << "(x";
				ss << ((unsigned) f->index);
				ss << ")";
		}
		if (E != DD_ONE) {
			ss << (T != DD_ONE ? (" "+AND_STR+" ") : "");
			ss << (E != Cudd_E(f) ? NOT_STR : "");
			ss << "(";
			retval = recursiveShannonString(E,ss);
			ss << ")";
	    }

		if(or_open)
			ss << ")";

	    return(1);
	}
};


#endif
