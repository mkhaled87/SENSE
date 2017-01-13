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

#include "Prolonger.h"

namespace ncs {

    Define_Module(Prolonger);

    void Prolonger::initialize(){
        isActive = par("isActive");
        max_delay = par("max_delay");
    }

    void Prolonger::handleMessage(cMessage *msg){
        if(strcmp(msg->getName(),"tosendEvent")==0){
            SendPackets(msg);
        }
        else{
            if(msg->isPacket()){

                double moredelay = max_delay - ((NcsPacket*)msg)->getChannelDelay() - (0.01*max_delay);

                if(!isActive || moredelay==0){
                    send(msg, "out");
                    return;
                }

                if(moredelay<0){
                    EV_ERROR << "invalid delay at prolonger: chack whether max_delay in prolonger is less than the one in the channel!";
                    return;
                }

                packetsUnderControl.push_back(((NcsPacket*)msg));

                cMessage* tosendEvent = new cMessage("tosendEvent");
                tosendEvents.push_back(tosendEvent);

                scheduleAt(simTime()+moredelay, tosendEvent);
            }
            else{
                EV_ERROR << "invalid msg/packet !";
                return;
            }
        }
    }

    void Prolonger::SendPackets(cMessage* tosendEvent){
        for(size_t i=0; i<tosendEvents.size(); i++){
            if(tosendEvents[i] == tosendEvent){
                NcsPacket* pktToSend = packetsUnderControl[i];
                packetsUnderControl.erase(packetsUnderControl.begin()+i);
                tosendEvents.erase(tosendEvents.begin()+i);
                cancelAndDelete(tosendEvent);
                //pktToSend->setSchedulingPriority(-1);
                send(pktToSend, "out");
                return;
            }
        }
        EV_ERROR << "CommunicationChannel::SendPackets: found nothing to send !";
    }

} //namespace
