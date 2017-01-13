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

package ncs;

module SmartController
{
    gates:
        input in;		// At this gate, we recieve sampled-decoded packets from the control systems over the network
        output out;		// At this gate, we send control actions to the control system over the networl

    submodules:
        prolonger: Prolonger;
        controller: Controller;

    connections:
        in --> prolonger.in;
        prolonger.out --> controller.in;
        out <-- controller.out;
}
