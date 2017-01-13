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

package ncs.simulations;

import ncs.Controller;
import ncs.SmartController;
import ncs.ControlSystem;
import ncs.Prolonger;
import ncs.CommunicationChannel;
import ncs.VehicleAnimator;



network NCS_Topology
{
    @display("bgb=899.48,674.61");

    //@figure[grid](type=group; transform=translate(500,25));
    //@figure[grid.bg](type=rectangle; pos=0,0; size=500,500; anchor=n; cornerRadius=5; fillColor=#fafdb9; fillOpacity=0.5; lineColor=#000000; lineOpacity=0.7);

    //@figure[obsticle1](type=group; transform=translate(500,25));
    //@figure[obsticle1.bg](type=rectangle; pos=-12.5,200; size=25,300; anchor=n; cornerRadius=5; fillColor=#000000; fillOpacity=0.8; lineColor=#000000; lineOpacity=0.7);

    //@figure[target](type=group; transform=translate(500,25));
    //@figure[target.bg](type=rectangle; pos=125,250; size=250,250; anchor=n; cornerRadius=5; fillColor=#00ff00; fillOpacity=0.8; lineColor=#000000; lineOpacity=0.7);

    @figure[trail](type=polyline; lineWidth=3; lineOpacity=0.6; lineColor=orange);
    @figure[car](type=image; image="car-top-view"; pos=0,0; anchor=c);

    //@figure[txt1](type=text; transform="((1.5 0) (0 1.5) (0 0))"; text="ZOH"; pos=16,23);
    //@figure[txt2](type=text; transform="((1.5 0) (0 1.5) (0 0))"; text="Plant"; pos=73,23);
    //@figure[txt3](type=text; transform="((1.5 0) (0 1.5) (0 0))"; text="Channel\nSC"; pos=60,60);
    //@figure[txt4](type=text; transform="((1.5 0) (0 1.5) (0 0))"; text="Channel\nCA"; pos=350,650);
    //@figure[txt5](type=text; transform="((1.5 0) (0 1.5) (0 0))"; text="Symbolic\nController"; pos=750,450);

    submodules:
        Animator: VehicleAnimator {
            @display("p=220.34999,75.71;i=abstract/opticalswitch");
        }
        Channel_SC: CommunicationChannel {
            @display("p=210.18,180.8;i=abstract/router");
        }

        Symbolic_Controller: SmartController {
            @display("p=127.689995,249.73;i=block/network2");
        }

        Channel_CA: CommunicationChannel {
            @display("p=45.2,180.8;i=abstract/router");
        }
        Smart_ZOH: Prolonger {
            @display("p=43.211998,76.451996;i=block/boundedqueue");
        }
        Plant: ControlSystem {
            @display("p=127.689995,75.71;i=block/cogwheel");
        }

    connections:
        Plant.outAnimator --> Animator.in;
        Plant.out --> Channel_SC.in;
        Channel_SC.out --> Symbolic_Controller.in;
        Channel_CA.in <-- Symbolic_Controller.out;
        Smart_ZOH.in <-- Channel_CA.out;
        Plant.in <-- Smart_ZOH.out;
}
