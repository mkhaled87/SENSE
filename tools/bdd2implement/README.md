BDD2Implement - A code generation tool for BDD-based symbolic controllers
=========================================================================
BDD2Implement is a C++ tool to generate hardware/software implementations of BDD-based symbolic controllers.
Having the tools [SCOTS](http://www.hcs.ei.tum.de/en/software/scots/) and [SENSE](http://www.hcs.ei.tum.de/en/software/sense/) that generate BDD-based sysbolic controllers of (networked) general nonlinear dynamical systems, BDD2Implement completes missing ring in the automatic synthesis technique.

BDD2Implement accepts static or dynamic determinized symbolic controllers in the form of BDD-files.
The BDD files encodes the controller dynamics as boolean functions.
If the provided controller is not determinized, BDD2Implement provides a determinization of the controller.

Due to the technquie used in BDD2Implement, the generated implementations are formal.
This guarantees the generated codes are exactly achieveing the behaviour in the provided controllers.
Consequently, the whole development cycle SCOTS/SENSE/BDD2Implement is now formal.
Theoritically (i.e. not implemented in the tool), the generated codes can be converted back to their exact controllers which implies their formality.

BDD2Implement can gerate codes in the following formats:
- HARDWARE:
	- Verilog/VHDL modules
- SOFTWARE:
	- C/C++ boolean-valued functions

BDD2Implement expects existing BDD-based symbolic controllers from SCOTS or SENSE. 

It starts by converting the multi-output boolean functions inside BDDs to multi singel-output functions.
If the provided controller is not determinized, BDD2Implement provides a determinization of the controller using several posssible determinization methods.
For VHDL/Verilog, the boolean functions are dumped to the VHDL module contains the boolean functions as maps from input-port to output-port.
For dynamic controllers, the HW module contains additional memory for the state of the cotroller.
For C/C++, the boolean functions are dumped as C++ codes.
The C/C++ compiler at implementer-side takes care of converting such boolean functions to machine codes.

It also expects infotmation about the delay bounds in the NCS. 
Then, it operates within the symbolic abstraction of the plants to construct a symbolic abstraction for NCS. Plats symbolic models can be easly constructed using a tool like [SCOTS](https://www.hcs.ei.tum.de/en/software/scots/).

BDD2Implement depends on the CUDD-3.0.0 library for manipulating BDDs, written by Fabio Somenzi [here](http://vlsi.colorado.edu/~fabio/). 
The _dddmp_ library is also used for reading and writing BDDs which already comes with CUDD.






Internal Structure of BDD2Implement
===================================

File structure:

- examples/ - The pre-crafted examples.
- src/ - The classes of BDD2Implement.
- templates/ - The templates used for code generation.

BDD2Implement has a couple of classes in the 'src' folder:

- [BDD2Implement.hh](src/BDD2Implement.hh) - The main header file that need to be included by all programs using BDD2Implement.
- [BddDecomposer.hh](src/BddDecomposer.hh) - A C++ class to decomponse multi-output boolean functions in BDD to multi single-output boolean functions in BDD.
- [BddDeterminizer.hh](src/BddDeterminizer.hh) - A C++ class to determinize the symbolic controller.
- [BddReader.hh](src/BddReader.hh) - A C++ class to read BDD files from SCOTS or SENSE.
- [BddToC.hh](src/BddToC.hh) - A C++ class to to generat C/C++ codes.
- [BddToHDL.hh](src/BddToHDL.hh) - A C++ class to to generat VHDL/Verilog codes representing a static controller.
- [BddToHDLdynamic.hh](src/BddToHDLdynamic.hh) - A C++ class to to generat VHDL/Verilog codes representing a dynamic controller.
- [BddToString.hh](src/BddToString.hh) - A C++ class to to dump boolean functions from BDD files.
- [CuddMintermIterator.hh](src/CuddMintermIterator.hh) - A C++ class representing an C++ iterator for iterating over BDD minterms.


There is also a some utility classes in the 'src/utils.hh' file:

- ncsFixPoint - For fixed point operations with BDD objects.
- SymbolicSetInterface - For interfacing BDD files from SCOTS.


Installation
============

Requirements
------------
- A C++ and C compiler installed and accesible in a Unix-like environment. Linux and MacOS should be fine. Windows with Ubuntu bash or MSYS-2 is also OK.
- An installation of the CUDD-3.0.0 library.
- The SCOTS/SENSE tools are needed to construct symbolic controllers in BDD-based from.

Using BDD2Implement
-------------------
BDD2Implement is built by examples. 
This means each example code that includes [BDD2Implement.hh](src/BDD2Implement.hh) will compile with BDD2Implement code included. 
You generally need to kraft your example or use one of the provided examples.

For instance, to build the example (ex1-vhdl-raw) corresponding to a VHDL implementation of a vehicle controller:

> cd examples/ex1-vhdl-raw

> make

> ./ex1

You are first advised to check that the accompanying folder 'bdd' has the BDD files for the vehicle controller generated. 
If not, you need to build it and run it first. You will then need the tool SCOTS/SENSE for this.

For this example, BDD2Implement will load the BDD-controller provided in the folder 'bdd'. 
It'll then start operating on the BDD and constructs a determinized controller and then generate the VHDL code.


