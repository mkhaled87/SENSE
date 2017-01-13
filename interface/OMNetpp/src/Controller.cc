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
#define VERBOSE
#include "Controller.h"

namespace ncs {

    Define_Module(Controller);

    void Controller::initialize(){
        isEventDriven = par("isEventDriven");
        isTargetModes = par("isTargetModes");
        controllerMode = par("startMode");

        vector<string> bddFiles = ncsMisc::CssToVectorStrings(par("nbdd_file"));
        if(bddFiles.size() == 0){
            EV_ERROR << "You didn't provide any controller NBDD files !";
        }

        targetBDDs = ncsMisc::CssToVectorStrings(par("targetBDDs"));
        numTargetModes = targetBDDs.size();

        if(isTargetModes && (numTargetModes == 0)){
            EV_ERROR << "You are using the target modes while you didn't provide any targets !";
        }

        if(controllerMode > (numTargetModes-1)){
            EV_ERROR << "Invalid start mode !";
        }


        if(!isTargetModes) numTargetModes=0;


        tau = par("tau");
        nsc = par("nsc");
        nca = par("nca");
        u0 = ncsMisc::CssToVectorDouble(par("initial_input"));
        last_uout = u0;

        // the estimator
        std::string plantmodel = par("plantmodel");
        estimator = new ncsFIFOEstimator(nsc, nca, u0, tau, plantmodel);

        // opening the nbddFile
        if(!isTargetModes){
            nbddReader.push_back(new NBDDReader(bddFiles[0].c_str()));
            nbddReader[0]->open();
        }
        else{
            for(size_t i=0; i<numTargetModes; i++){
                nbddReader.push_back(new NBDDReader(bddFiles[i].c_str()));
                nbddReader[i]->open();
            }
        }

        if(!isEventDriven){
            tdScheduledControlInput = u0;
            tauEventController = new cMessage("tauEventController");
            scheduleAt(0, tauEventController);
        }


        if(isTargetModes){
            for(size_t i=0; i<numTargetModes; i++){
                ssTargets.push_back(new SymbolicSet());
                ssTargets[i]->LoadFromFile(targetsCuddManager, targetBDDs[i].c_str() ,0);
            }
        }
    }

    Controller::Controller(){
        tauEventController = NULL;
    }

    Controller::~Controller(){
        for(size_t i=0; i<numTargetModes; i++){
            nbddReader[i]->close();
            delete nbddReader[i];
        }

        if(isTargetModes){
            for(size_t i=0; i<numTargetModes; i++){
                delete ssTargets[i];
            }
        }
        delete estimator;

        if(!isEventDriven){
            cancelAndDelete(tauEventController);
        }
    }

    void Controller::handleMessage(cMessage *msg){
        if(msg->isPacket()){
            if(isEventDriven){
                HandleStateInput_EventDriven((NcsPacket*)msg);
            }
            else{
                std::vector<double> tmp = ((NcsPacket*)msg)->getValues();
                tdLastArrivedState = tmp;
            }
            cancelAndDelete(msg);
        }
        else if(msg == tauEventController){

            tdScheduledControlInput = suggestControlInput(tdLastArrivedState);

            // send a new packet with the control input
            NcsPacket* newControlPckt = new NcsPacket();
            newControlPckt->setValues(tdScheduledControlInput);
            send(newControlPckt,"out");

            scheduleAt(simTime()+tau, tauEventController);
        }
        else{
            EV_ERROR << "invalid msg/packet !";
        }
    }

    vector<double> Controller::suggestControlInput(vector<double> x_arrived){

        // -------------------------------------------------------------------
        // TEST CODE:: Controller based on symbolic model of the plant
        // -------------------------------------------------------------------
        /*
        static Cudd cuddManager;
        static vector<vector<double>> uCA{{0.3,0},{0.3,0}};
        char* FILE_CONTR_NBDD = "../../../examples/FIFO/vehicle_half3/vehicle_contr.nbdd";
        char* FILE_PLANT_BDD = "../../../examples/FIFO/vehicle_half3/scots-files/vehicle_rel.bdd";
        static psmController C(cuddManager, FILE_PLANT_BDD, FILE_CONTR_NBDD, 3, 2, 2, 2);
        vector<vector<double>> us_suggested = C.suggestControlInputs(x_arrived, uCA);

        if(us_suggested.size() == 0)
          EV_ERROR << "Hopeless !!";
          */
        // -------------------------------------------------------------------
        // END TEST CODE
        // -------------------------------------------------------------------


        //1] ask the estimator to estimate the NCS state
        estimator->Refresh(x_arrived, last_uout);
        std::vector<std::vector<double>> NCSFifoStateValues = estimator->getNCSFifoStateValues();
        std::vector<double> ncsState = ncsMisc::UnrollVectors(NCSFifoStateValues);


        //1] extra :: if we are mode-based, decide which mode is to use
        if(isTargetModes){
            if(ssTargets[controllerMode]->isElement(estimator->getCurrentSystemState())){
                controllerMode = (controllerMode+1)%numTargetModes;
            }
        }


        //2] ask the BDD about the control-input
        std::vector<std::vector<double>> control_inputs = nbddReader[controllerMode]->getInput(ncsState);

        if(control_inputs.size() == 0){
            EV_ERROR << "Controller::HandleStateInput: NBDD file has no such a state !";
        }

        if(control_inputs.size() > 1)
            EV_INFO << "Controller::HandleStateInput: NBDD file offered many inputs, we selected randomly !";

        std::vector<double> control_input;
        int rand_idx = intrand(control_inputs.size());
        control_input = control_inputs[rand_idx];




        // -------------------------------------------------------------------
        // TEST CODE
        // -------------------------------------------------------------------
        /*
        static std::vector<double> last_last_uout = last_uout;
        static int lastU_test=0;
        static bool testReady = false;
        static std::vector<double> lastEstimated;
        if(testReady && (lastEstimated != x_arrived)){
            EV_ERROR << "Bas Previous Estimateion !!";
        }

        if(lastU_test>=2 && (
                (last_last_uout != NCSFifoStateValues[3]) ||
                (last_uout != NCSFifoStateValues[2])
                )){
            EV_ERROR << "Bad constructed input buffer !!";
        }

        lastEstimated = NCSFifoStateValues[0];
        last_last_uout = last_uout;
        testReady = true;
        lastU_test++;
        */

        /*
        uCA[0]=uCA[1];
        uCA[1]=control_input;
        */

        // -------------------------------------------------------------------
        // END TEST CODE
        // -------------------------------------------------------------------

        return control_input;

    }

    void Controller::HandleStateInput_EventDriven(NcsPacket* pkt){

        std::vector<double> x_arrived = pkt->getValues();
        std::vector<double> control_input = suggestControlInput(x_arrived);

        //4] send a new packet with the control input
        NcsPacket* newControlPckt = new NcsPacket();
        newControlPckt->setValues(control_input);
        send(newControlPckt,"out");



        //*] extra
        last_uout = control_input;
    }


}; // namespace
