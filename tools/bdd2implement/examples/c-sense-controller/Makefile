#
# compiler
#
CC        		= g++
CXXFLAGS 		= -Wall -Wextra -std=c++11 -g -O0

#
# local source
#
MYROOT			= ../..
MYINC			= -I$(MYROOT)/src

#
# cudd 
#
CUDDPATH		=  /opt/local
CUDDINC 		= -I$(CUDDPATH)/include
CUDDLIBS		= -lcudd 
CUDDLPATH   	= -L$(CUDDPATH)/lib

TARGET = example

all: $(TARGET)

%.o:%.cc
	$(CC) -c $(CXXFLAGS) $(CUDDINC) $(MYINC) $< -o $@

$(TARGET): $(TARGET).o
	$(CC) $(CXXFLAGS) -o $(TARGET) $(TARGET).o $(CUDDLPATH) $(CUDDLIBS)


clean:
	rm  ./$(TARGET)  ./$(TARGET).o ./*.h
