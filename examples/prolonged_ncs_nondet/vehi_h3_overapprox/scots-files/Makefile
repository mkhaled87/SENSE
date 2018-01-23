#
# compiler
#
CC        		= g++
CXXFLAGS 		= -Wall -Wextra -std=c++11 -O3 -DNDEBUG

#
# scots 
#
SCOTSROOT		= ../../../../lib/scots
SCOTSINC		= -I$(SCOTSROOT)/bdd -I$(SCOTSROOT)/utils 

#
# cudd 
#
CUDDPATH		=  /opt/local
CUDDINC 		= -I$(CUDDPATH)/include
CUDDLIBS		= -lcudd 
CUDDLPATH   = -L$(CUDDPATH)/lib

TARGET1 = vehicle
TARGET2 = vehicle_fine_1_tau
TARGET3 = vehicle_fine_NCSMAX_tau

all: $(TARGET1) $(TARGET2) $(TARGET3)

%.o:%.cc
	$(CC) -c $(CXXFLAGS) $(CUDDINC) $(SCOTSINC) $< -o $@

$(TARGET1): $(TARGET1).o
	$(CC) $(CXXFLAGS) -o $(TARGET1) $(TARGET1).o $(CUDDLPATH) $(CUDDLIBS)
	
$(TARGET2): $(TARGET2).o
	$(CC) $(CXXFLAGS) -o $(TARGET2) $(TARGET2).o $(CUDDLPATH) $(CUDDLIBS)	
	
$(TARGET3): $(TARGET3).o
	$(CC) $(CXXFLAGS) -o $(TARGET3) $(TARGET3).o $(CUDDLPATH) $(CUDDLIBS)		

clean:
	rm  ./$(TARGET1)  ./$(TARGET1).o
	rm  ./$(TARGET2)  ./$(TARGET2).o
	rm  ./$(TARGET3)  ./$(TARGET3).o	
