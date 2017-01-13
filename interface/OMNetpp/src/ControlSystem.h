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

#ifndef __NCS_CONTROLSYSTEM_H
#define __NCS_CONTROLSYSTEM_H

#include <omnetpp.h>
#include <string>
#include <vector>
#include "ncsmisc.h"
#include "ncsPacket_m.h"
#include "vehicleAnimatorPkt_m.h"
#include "Plant.h"

using namespace omnetpp;

namespace ncs {

    /**
     * Implements the Controller simple module.
     * See the NED file for more information.
     */
    class ControlSystem : public cSimpleModule
    {
      protected:
        double tau;
        std::string plantmodel;
        std::vector<double> current_input;
        std::vector<double> current_state;

        Plant* plant;

        cMessage* tauEventPlant;         // used to handle tau-events (sampler)
        NcsPacket* lastSentPacket;  // stores the pointer of last sent packet

        virtual void initialize();
        virtual void handleMessage(cMessage *msg);

      public:
        ControlSystem();
        ~ControlSystem();
    };

}; // namespace

#endif
