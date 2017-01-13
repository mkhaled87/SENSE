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

#ifndef __NCS_COMMUNICATIONCHANNEL_H_
#define __NCS_COMMUNICATIONCHANNEL_H_

#include <omnetpp.h>
#include <vector>
#include "ncsPacket_m.h"

using namespace omnetpp;

namespace ncs {

/**
 * TODO - Generated class
 */
class CommunicationChannel : public cSimpleModule
{
  protected:
    double min_delay;
    double max_delay;
    bool AllowDropouts;
    double DropoutProbability;

    std::vector<cMessage*> tosendEvents;
    std::vector<NcsPacket*> packetsUnderControl;

    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

    double GetRandomDelay();
    bool isToDrop();
    void SendPackets(cMessage* tosendEvent);

  public:
    CommunicationChannel();
    ~CommunicationChannel();
};

} //namespace

#endif
