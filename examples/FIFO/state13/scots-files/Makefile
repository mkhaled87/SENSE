#
# compiler
#
CC       	  	= clang++
CXXFLAGS 		= -Wall -Wextra -std=c++11 -DNDEBUG -g

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

TARGET = state13

all: $(TARGET)

%.o:%.cc
	$(CC) -c $(CXXFLAGS) $(CUDDINC) $(SCOTSINC) $< -o $@

$(TARGET): $(TARGET).o
	$(CC) $(CXXFLAGS) -o $(TARGET) $(TARGET).o $(CUDDLPATH) $(CUDDLIBS)


clean:
	rm  ./$(TARGET)  ./$(TARGET).o ./*.bdd
