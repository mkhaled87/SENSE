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

#include "CommunicationChannel.h"

namespace ncs {

    Define_Module(CommunicationChannel);

    void CommunicationChannel::initialize(){
        min_delay = par("min_delay");
        max_delay = par("max_delay");
        AllowDropouts = par("AllowDropouts");
        DropoutProbability = par("DropoutProbability");
    }

    void CommunicationChannel::handleMessage(cMessage *msg)
    {
        if(strcmp(msg->getName(),"tosendEvent")==0){
            SendPackets(msg);
        }
        else{
            if(msg->isPacket()){

                if(isToDrop()){
                    ((NcsPacket*)msg)->setIsDropped(true);
                    cancelAndDelete(msg);
                    return;
                }

                double delay = GetRandomDelay();
                ((NcsPacket*)msg)->setChannelDelay(delay);
                ((NcsPacket*)msg)->setIsDropped(false);

                packetsUnderControl.push_back(((NcsPacket*)msg));

                cMessage* tosendEvent = new cMessage("tosendEvent");
                tosendEvents.push_back(tosendEvent);

                scheduleAt(simTime()+delay, tosendEvent);
            }
            else{
                EV_ERROR << "invalid msg/packet !";
            }
        }
    }

    void CommunicationChannel::SendPackets(cMessage* tosendEvent){
        for(size_t i=0; i<tosendEvents.size(); i++){
            if(tosendEvents[i] == tosendEvent){
                NcsPacket* pktToSend = packetsUnderControl[i];
                packetsUnderControl.erase(packetsUnderControl.begin()+i);
                tosendEvents.erase(tosendEvents.begin()+i);
                cancelAndDelete(tosendEvent);
                send(pktToSend, "out");
                return;
            }
        }
        EV_ERROR << "CommunicationChannel::SendPackets: found nothing to send !";
    }

    double CommunicationChannel::GetRandomDelay(){
        return uniform(min_delay, max_delay);
    }

    bool CommunicationChannel::isToDrop(){
        if(!AllowDropouts)
            return false;

        if(uniform(0,1) > DropoutProbability)
            return true;
        else
            return false;
    }

    CommunicationChannel::CommunicationChannel(){
    }

    CommunicationChannel::~CommunicationChannel(){
    }

} //namespace
