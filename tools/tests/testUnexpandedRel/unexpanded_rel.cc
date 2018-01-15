/*
 * unexpanden_rel.cc
 *
 *  created on: 31.08.2015
 *      author: m.khaled
 */

#include <array>
#include <iostream>

#include "cuddObj.hh"
#include "SENSE.hh"

#define FILE_BDD_REL  "../../examples/dint_ncs/dint_rel.bdd"
#define FILE_NBDD_REL "../../examples/dint_ncs/dint_rel.nbdd"

int main() {
  Cudd cuddManager;
  
  SymbolicSet ss_org;
  ss_org.LoadFromFile(cuddManager, FILE_BDD_REL);

  ncsFIFOTransitionRelation ncs_rel(cuddManager, FILE_NBDD_REL);
  BDD bdd1 = ss_org.getSymbolicSet();
  BDD bdd2 = ncs_rel.getOriginalRelation();

  vector<int> permute = BDDUtils::GetReadyPermuteMap(cuddManager);
  vector<size_t> orgVars = ncs_rel.getOrginalVars();
  for(size_t i=0; i<orgVars.size(); i++)
	  permute[orgVars[i]]=i;

  bdd2 = bdd2.Permute(permute.data());

  string ans;
  cout << "print relations ? [y=yes]: ";
  cin >> ans;
  if(ans == "y"){
	  BDDUtils::PrintBDD("ss_org.getSymbolicSet()", bdd1);
	  BDDUtils::PrintBDD("ncs_rel.getOriginalRelation()", bdd2);
  }

  if(bdd1 == bdd2)
	cout << "they are the same !";
  else{
	cout << "they are not the same, there seems to be a problem !!";
	BDD intersect = bdd2.Intersect(bdd1);
	BDDUtils::PrintBDD("intersect",intersect);
  }

  return 1;
}

