#
# matlab root 
#
MATLABROOT		= /usr/local/MATLAB/R2016a

#
# compiler
#
CC       	  	= $(MATLABROOT)/bin/mex
CXXFLAGS 		= -Wall -Wextra -std=c++11 -O3

#
# ncsACT
#
NCSACTROOT   		= ../..
NCSACTINC		= -I$(NCSACTROOT)/src -I$(NCSACTROOT)/utils

#
# cudd 
#
CUDDPATH		=  /opt/local/
CUDDINC 		= -I$(CUDDPATH)/include
CUDDLIBS		= -lcudd 
CUDDLPATH   	= -L$(CUDDPATH)/lib


TARGET1 			= mexNcsController
TARGET2 			= mexSymbolicSet

all: $(TARGET1) $(TARGET2)

$(TARGET1): $(TARGET1).cc
	$(CC) COMPFLAGS='$(COMPFLAGS) $(CXXFLAGS)' -largeArrayDims $(NCSACTINC) $(CUDDINC) $(CUDDLPATH) $(CUDDLIBS) $(TARGET1).cc


$(TARGET2): $(TARGET2).cc
	$(CC) COMPFLAGS='$(COMPFLAGS) $(CXXFLAGS)' -largeArrayDims $(NCSACTINC) $(CUDDINC) $(CUDDLPATH) $(CUDDLIBS) $(TARGET2).cc
