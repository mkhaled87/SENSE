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

#include "ControlSystem.h"


namespace ncs {

    Define_Module(ControlSystem);

    void ControlSystem::initialize()
    {
        tau = par("tau");
        std::string tmp = par("plantmodel");
        plantmodel = tmp;
        current_input = ncsMisc::CssToVectorDouble(par("initial_input"));
        current_state = ncsMisc::CssToVectorDouble(par("initial_state"));


        // plant model identification
        plant = PLANT_MANAGER::NewPlantModel(plantmodel);
        plant->initOde(tau);

        tauEventPlant = new cMessage("tauEventPlant");
        scheduleAt(0, tauEventPlant);
    }

    void ControlSystem::handleMessage(cMessage *msg)
    {
        if(msg == tauEventPlant){


            if(simTime() != 0){
                current_state = plant->Compute(current_state, current_input);
                EV_INFO << "ControlSystem generated/sent a new state ! x=" << ncsMisc::VectorDoubleToCss(current_state);
            }
            else{
                EV_INFO << "ControlSystem sent initial state ! x=" << ncsMisc::VectorDoubleToCss(current_state);
            }

            lastSentPacket = new NcsPacket();
            lastSentPacket->setValues(current_state);
            send(lastSentPacket, "out");

            // for the animator
            vehcileAnimatorPkt* animatorPkt = new vehcileAnimatorPkt();
            animatorPkt->setX(current_state[0]);
            animatorPkt->setY(current_state[1]);
            animatorPkt->setTheta(0);
            send(animatorPkt,"outAnimator");

            scheduleAt(simTime()+tau, tauEventPlant);
        }
        else{
            if(msg->isPacket()){
                current_input = ((NcsPacket*)msg)->getValues();
                EV_INFO << "ControlSystem recieved a new control-input ! u=" << ncsMisc::VectorDoubleToCss(current_input);
                cancelAndDelete(msg);
            }
            else{
                EV_ERROR << "invalid msg/packet !";
            }
        }
    }

    ControlSystem::ControlSystem(){
        tauEventPlant = NULL;
    }

    ControlSystem::~ControlSystem(){
        cancelAndDelete(tauEventPlant);
        delete plant;
    }


}; // namespace
