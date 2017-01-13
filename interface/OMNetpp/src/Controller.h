//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __NCS_CONTROLLER_H
#define __NCS_CONTROLLER_H

#include <omnetpp.h>
#include <string>
#include <vector>
#include <queue>
#include "ncsPacket_m.h"
#include "ncsmisc.h"
#include "NBDDReader.h"
#include "ncsFIFOEstimator.h"
#include "psmController.hh"

using namespace omnetpp;

namespace ncs {

    /**
     * Implements the Controller simple module.
     * See the NED file for more information.
     */
    class Controller : public cSimpleModule
    {
      protected:
        double tau;

        bool isEventDriven;

        // for target-mode dynamic controllers:
        bool isTargetModes;
        vector<string> targetBDDs;
        int controllerMode;
        int numTargetModes;
        Cudd targetsCuddManager;
        vector<SymbolicSet*> ssTargets;
        //-------------------------------


        cMessage* tauEventController;

        std::string nbdd_file;
        size_t nsc;
        size_t nca;

        std::vector<double> u0;
        std::vector<double> last_uout;

        vector<NBDDReader*> nbddReader;     // one per mode
        ncsFIFOEstimator* estimator;

        std::vector<double> tdScheduledControlInput;
        std::vector<double> tdLastArrivedState;

        void HandleStateInput_EventDriven(NcsPacket* pkt);
        vector<double> suggestControlInput(vector<double> x_arrived);

        virtual void initialize();
        virtual void handleMessage(cMessage *msg);

      public:
        Controller();
        ~Controller();
    };

}; // namespace

#endif
