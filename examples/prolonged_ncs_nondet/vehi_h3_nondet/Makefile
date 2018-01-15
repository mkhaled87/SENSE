#
# compiler
#
CC        		= g++
CXXFLAGS 		= -Wall -Wextra -std=c++11 -g -O0

#
# ncsAct
#
NCSACTROOT		= ../../..
NCSACTINC		= -I$(NCSACTROOT)/src -I$(NCSACTROOT)/utils 

#
# cudd 
#
CUDDPATH		=  /opt/local
CUDDINC 		= -I$(CUDDPATH)/include
CUDDLIBS		= -lcudd 
CUDDLPATH   		= -L$(CUDDPATH)/lib

TARGET1 = vehicle_ncs
TARGET2 = simulate_ncs

all: $(TARGET1) $(TARGET2)

%.o:%.cc
	$(CC) -c $(CXXFLAGS) $(CUDDINC) $(NCSACTINC) $< -o $@

$(TARGET1): $(TARGET1).o
	$(CC) $(CXXFLAGS) -o $(TARGET1) $(TARGET1).o $(CUDDLPATH) $(CUDDLIBS)

$(TARGET2): $(TARGET2).o
	$(CC) $(CXXFLAGS) -o $(TARGET2) $(TARGET2).o $(CUDDLPATH) $(CUDDLIBS)	

clean:
	rm  ./$(TARGET1)  ./$(TARGET1).o
	rm  ./$(TARGET2)  ./$(TARGET2).o	
