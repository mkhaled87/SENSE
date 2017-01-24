#ifndef __NCSFIFOESTIMATOR_H
#define __NCSFIFOESTIMATOR_H

#include <vector>
#include "Plant.h"

    /**
     * Implements the NCS-Fifo-State (x ...., u ....) estimator.
     */
    class ncsFIFOEstimator{
    private:
        size_t NSCMAX;
        size_t NCAMAX;
        
        std::vector<std::vector<double>> ncsFifoStateValues_X;   // order: (x0, x1, x2 ...... )
        std::vector<std::vector<double>> ncsFifoStateValues_U;   // order: (un, un-1, .... u2, u1)
        
        std::vector<std::vector<double>> uoutBuffer;
        
        std::vector<double> u0;

        Plant* plant;

    public:
        ncsFIFOEstimator(size_t NSC, size_t NCA, std::vector<double> u0_,  double tau, const std::string& plantmodel){
            NSCMAX = NSC;
            NCAMAX = NCA;
            
            u0 = u0_;

            ncsFifoStateValues_X.resize(NSCMAX);
            ncsFifoStateValues_U.resize(NCAMAX);

            for(size_t i=0; i<NCAMAX; i++)
                uoutBuffer.push_back(u0);


            // plant model identification
            plant = PLANT_MANAGER::NewPlantModel(plantmodel);
            plant->initOde(tau);
        }
        ~ncsFIFOEstimator(){
        }
        
        Plant* getPlant(){
            return plant;
        }

        void Refresh(const std::vector<double>& new_x, const std::vector<double>& old_u){

            // estimating the state values not yet arrived to onstruct the NCS state
            std::vector<double> tmp = old_u;
            uoutBuffer.push_back(tmp);
            
            ncsFifoStateValues_X[NSCMAX-1]=new_x;
            for(size_t i=1; i<NSCMAX; i++)
                ncsFifoStateValues_X[NSCMAX-i-1] = plant->Compute(ncsFifoStateValues_X[NSCMAX-i],uoutBuffer[i-1]);

            uoutBuffer.erase(uoutBuffer.begin());


            // A copy ready for the call of getNCSFifoStateValues

            size_t copy_idx = uoutBuffer.size() - NCAMAX;
            for(size_t i=0; i<NCAMAX; i++)
                ncsFifoStateValues_U[i] = uoutBuffer[copy_idx + i];

        }
        
        std::vector<std::vector<double>> getNCSFifoStateValues(){
            std::vector<std::vector<double>> ncs_state;
            
            for(size_t i=0; i< ncsFifoStateValues_X.size(); i++)
                ncs_state.push_back(ncsFifoStateValues_X[i]);

            for(size_t i=0; i< ncsFifoStateValues_U.size(); i++)
                ncs_state.push_back(ncsFifoStateValues_U[ncsFifoStateValues_U.size()-1 -i]);
            
            return  ncs_state;
        }
        
        std::vector<double> getCurrentSystemState(){
            return ncsFifoStateValues_X[NSCMAX-1];
        }

    };


#endif
