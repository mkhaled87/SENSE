SENSE - Symbolic controlEr for Networked SystEms
================================================
SENSE is an experimental framework for the correct-by-construction controller synthesis for networked control systems (NCS).
It facilitates constructing finite abstraction and synthesizing symbolic controller using the plants of NCS. 
It uses binary decision diagrams (BDDs) as the primary data structure for efficient symbolic abstraction construction. 

SENSE expects existing finite abstracts (a.k.a. symbolic models) of the plants inside NCS as input. 
It also expects information about the delay bounds in the NCS and the specification to be enforced on the plants of NCS. 
It uses the abstraction of the plant to construct an abstraction for NCS. 
Plants' symbolic models can be easily constructed using a tool like [SCOTS](https://www.hcs.ei.tum.de/en/software/scots/), which is provided with SENSE.
Then, the tool constructs provably-correct controllers that satisfy the given specification.

SENSE depends on the CUDD-3.0.0 library for manipulating BDDs, written by Fabio Somenzi [here](http://vlsi.colorado.edu/~fabio/). 
The _dddmp_ library is also used for reading and writing BDDs which already comes with CUDD.

The tool has an experimental nature.
It serves as a host for several classes of NCS and facilitates the following:

- Constructing symbolic models of NCS.
- Synthesizing controllers.
- Testing the constructed abstractions and the synthesized controllers.
- Simulating and visualizing the closed-loop of the NCS/controller.
- Automatic code generation for final real-world implementation.


Internal Structure of SENSE
===========================

File structure:

- doc/ - The manual of SENSE.
- examples/ - The pre krafted examples.
- interface/ - MATLAB and OMNeT++ interface classes.
- lib/ - The libraries used SENSE and its examples.
- src/ - The main C++ classes of SENSE.
- tools/ - A sety of helper tools for SENSE.
- utils/ - Utility C++ classes for SENSE.


SENSE has a couple of classes in the 'src' folder:

- ncsState - The class representing a NCS state consisting of some plants' states, inputs and delay information.
- ncsTransitionRelation - The class representing the transition relation in a NCS.
- ncsController - The class encapsulating the NCS synthesized controller.
- ncsFixPoint - The class contains source code of the Fixedpoint operations on BDDs to synthesize the symbolic controllers.

There is also a some utility classes/files helping SENSE to work, most of them are in the 'utils' folder:

- ncsFixPoint - For fixed point operations with BDD objects.
- SymbolicSetInterface - For interfacing BDD files from SCOTS.


SENSE comes also with some helper tools in the 'tools' folder:

- bdd2implement - A tool to automatically generate C++/VHDL/Verilog codes from the synthesized BDD-based controllers.
- abstractNcs - A tool to construct the BDD-based symbolic model of the NCS from the BDD-symbolic model of the plant in the NCS.
- bdd2fsm - A tool to generate a .fsm file from the BDD representing a transition relation. Used to visualize the system using Giphy.
- bdddump - A tool that dumps the information in any BDD file.
- contCoverage - A tool that visualizes in terminal the coverage of 2-D symbolic controllers generated from SENSE.



Installation
============

SENSE comes in the form of header files. 
Users compile any of the provided examples or compose their own examples and then run them.
Examples generate symbolic controller files that can be simulated using one of the provided interfaces.
Also, an implementation C++/VHDL/Verilog code can be generated automatically from the synthesized NCS controllers using the provided helper tool 'tools/bdd2implement'.

You may also watch the follwoing video showing the basic installation of SENSE and running one example:

[![SENSE: installation and example](https://img.youtube.com/vi/YBsFgyymR9s/0.jpg)](https://www.youtube.com/watch?v=YBsFgyymR9s)

Requirements
------------
- A C++ and C compiler installed and accesible in a Unix-like environment. Linux and MacOS should be fine. Windows with Ubuntu bash or MSYS-2 is also OK.
- An installation of the CUDD-3.0.0 library.
- The tool SCOTS is needed to construct original symbolic models of the plants inside NCS. It is contained in SENSE in the /lib folder.
- MATLAB: needed to do a closed-loop simulation of the synthesized controller with the NCS.
- OMNeT++: needed to do a closed-loop simulation/visualization of the synthesized controller with the NCS.

Using SENSE
-----------
SENSE is built by examples. This means each example code that includes SENSE will compile with SENSE code included. You generally need to kraft your example or use one of the provided examples.

For instance, to build the example (vehicle_half3) corresponding to a vehicle dynamics within an arena with three obstacles:

> cd examples/prolonged_ncs/vehicle_half3

> make

> ./vehicle

You are first advised to check that the accompanying folder 'scots-files' has the BDD files for the plant inside NCS generated. If not, you need to build it and run it first. You will then need the tool SCOTS for this.

For this example, SENSE will load the original symbolic models provided in the folder 'scots-files'. It'll then start operating on the original symbolic model and constructs the new one and synthesize a controller for it.

You can then


