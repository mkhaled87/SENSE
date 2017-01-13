SENSE - Symbolic controlEr for Networked SystEms
================================================
SENSE is an abstraction and controller synthesis tool for networked control systems (NCS). It uses binary decision diagrams (BDDs) as the primary data structure for efficient symbolic abstraction construction. 

SENSE expects existing abstracts of the plants inside NCS as input. It also expects infotmation about the delay bounds in the NCS. Then, it operates within the symbolic abstraction of the plants to construct a symbolic abstraction for NCS. Plats symbolic models can be easly constructed using a tool like [SCOTS](https://www.hcs.ei.tum.de/en/software/scots/).

If you want to cite SENSE in a scientific paper, please cite its tool paper (currently archived version):

- Mahmoud Khaled and Majid Zamani: _SENSE: Symbolic Modeling and Control for Networked Control Systems. [here](http://motesy.cs.uni-bremen.de/pdfs/cav2016.pdf)

SENSE depends on the CUDD-3.0.0 library for manipulating BDDs, written by Fabio Somenzi [here](http://vlsi.colorado.edu/~fabio/). The _dddmp_ library is also used for reading and writing BDDs which already comes with CUDD.


Internal Structure of SENSE
===========================

File structure:

- examples/ - The pre krafted examples.
- interface/ - MATLAB and OMNeT++ interface classes.
- src/ - The classes of SENSE.
- utils/ - Utility classes for SENSE.

SENSE has a couple of classes in the 'src' folder:

- ncsState - The class representing a NCS state consisting of some plants' states, inputs and delay information.
- ncsTransitionRelation - The class representing the transition relation in a NCS.
- ncsController - The class encapsulating the NCS synthesized controller.

There is also a some utility classes/files helping SENSE to work, most of them are in the 'utils' folder:

- ncsFixPoint - For fixed point operations with BDD objects.
- SymbolicSetInterface - For interfacing BDD files from SCOTS.


Installation
============

Requirements
------------
- A C++ and C compiler installed and accesible in a Unix-like environment. Linux and MacOS should be fine. Windows with Ubuntu bash or MSYS-2 is also OK.
- An installation of the CUDD-3.0.0 library.
- The tool is needed to construct original symbolic models of the plants inside NCS.
- MATLAB: you need to do a closed-loop simulation of the synthesized controller with the NCS.
- OMNeT++: you need to do a closed-loop simulation/visualization of the synthesized controller with the NCS.

Using SENSE
-----------
SENSE is built by examples. This means each example code that includes SENSE will compile with SENSE code included. You generally need to kraft your example or use one of the provided examples.

For instance, to build the example (vehicle_half3) corresponding to a vehicle dynamics within an arena with three obstacles:

> cd examples/FIFO/vehicle_half3

> make

> ./vehicle

You are first advised to check that the accompanying folder 'scots-files' has the BDD files for the plant inside NCS generated. If not, you need to build it and run it first. You will then need the tool SCOTS for this.

For this example, SENSE will load the original symbolic models provided in the folder 'scots-files'. It'll then start operating on the original symbolic model and constructs the new one and synthesize a controller for it.

You can then


