#
# matlab root 
#
MATLABROOT	= /Applications/MATLAB_R2016b.app/

#
# compiler
#
#CC        = g++
CC       	  = $(MATLABROOT)/bin/mex
CXXFLAGS 		= -g -Wall -Wextra -std=c++11 -O3 

#
# scots
#
SCOTSROOT   = ../..
SCOTSINC		= -I$(SCOTSROOT)/bdd

#
# cudd 
#
CUDDPATH		=  /opt/local/
CUDDINC 		= -I$(CUDDPATH)/include
CUDDLIBS		= -lcudd 
CUDDLPATH   = -L$(CUDDPATH)/lib


TARGET 			= mexSymbolicSet

all: $(TARGET)

$(TARGET): $(TARGET).cc
	$(CC) COMPFLAGS='$(COMPFLAGS) $(CXXFLAGS)' -largeArrayDims $(SCOTSINC) $(CUDDINC) $(CUDDLPATH) $(CUDDLIBS) $(TARGET).cc

