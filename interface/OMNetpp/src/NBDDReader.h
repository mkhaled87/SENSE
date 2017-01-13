/*
 * NBDDReader.h
 *
 *  Created on: Oct 20, 2016
 *      Author: mk
 */

#ifndef NBDDREADER_H_
#define NBDDREADER_H_

#include <omnetpp.h>
#include "SENSE.hh"

using namespace omnetpp;

class NBDDReader{
private:
    bool opened = false;
    Cudd* pCuddManager;
    ncsController* pContr;
    std::string nbdd_file;

    size_t ssdim = 0;
    size_t isdim = 0;

    size_t nscmax = 0;
    size_t ncamax = 0;

public:
    /*****************************************************************/
    /* Constructors /Destructors                                     */
    /*****************************************************************/
    NBDDReader(const char* nbdd_file_){
        pCuddManager = new Cudd();
        nbdd_file = nbdd_file_;
    }
    ~NBDDReader(){
        if(opened)
            close();

        delete pCuddManager;
    }

    /*****************************************************************/
    /* getters/setters                                               */
    /*****************************************************************/
    size_t getNscMax(){return nscmax;}
    size_t getNcaMax(){return ncamax;}


    /*****************************************************************/
    /* member function: open                         */
    /*****************************************************************/
    void open(){
        if(opened)
            EV_ERROR << "NBDDReader::open: NBDD file already open !";

        try{
            pContr = new ncsController(*pCuddManager, nbdd_file.c_str());

            ncsState* stateX = pContr->getSourceState();

            ssdim = stateX->getSsdim();
            isdim = stateX->getIsdim();

            nscmax = stateX->getNscmax();
            ncamax = stateX->getNcamax();

            opened = true;
        }
        catch (const std::exception& e) {
            EV_ERROR << "NBDDReader::open: " << e.what();
        }
    }

    /*****************************************************************/
    /* member function: open                         */
    /*****************************************************************/
    void close(){
        if(!opened)
            EV_ERROR << "NBDDReader::open: NBDD is not open !";

        delete pContr;
    }


    /*****************************************************************/
    /* command: getInput                                             */
    /*      assumes the channel is already full and no qs            */
    /*****************************************************************/
    std::vector<std::vector<double>>
    getInput(std::vector<double> XU_values){
        if(!opened)
            EV_ERROR << "NBDDReader::open: NBDD is not open !";

        if(XU_values.size() != (nscmax*ssdim + nscmax*isdim))
            EV_ERROR << "NBDDReader::open: number of x values is not correct. It should be euql to (NSC_MAX*SS_DIM + NSC_MAX*IS_DIM).";


        std::vector<std::vector<double>> inputs;
        std::vector<int> q_values;

        for(size_t i=0; i<nscmax; i++)
            q_values.push_back(0);

        try{
          inputs = pContr->getInputs(XU_values, q_values);
        }
        catch (const std::exception& e) {
            EV_ERROR << "NBDDReader::getInput: " << e.what();
        }

        return inputs;

    }

};




#endif /* NBDDREADER_H_ */
