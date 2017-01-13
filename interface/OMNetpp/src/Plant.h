/*
 * Plant.h
 *
 *  Created on: Oct 20, 2016
 *      Author: mk
 */

#ifndef PLANT_H_
#define PLANT_H_

#include <omnetpp.h>
#include <vector>
#include "RungeKutta4.h"

using namespace omnetpp;

#define RUNGKUTTASTEPS 5

// Plant types
enum PLANT_TYPE {
    ODE
};

class Plant{
protected:
    int ssdim=0, isdim=0;
    double odeTau=-1;
    OdeSolver* solver = nullptr;
    PLANT_TYPE plantType;
    bool initialized = false;

public:
    Plant(){
    }

    int getSsDim(){return ssdim;}
    int getIsDim(){return isdim;}

    void base_initialize(int ssdim_, int isdim_, PLANT_TYPE plantType_){
        ssdim = ssdim_;
        isdim = isdim_;
        plantType = plantType_;

        if(solver != nullptr){
            EV_ERROR << "Plant::base_initialize: you can't reinitialize the plant !";
            return;
        }

        if (plantType == PLANT_TYPE::ODE)
            solver = new OdeSolver(ssdim, RUNGKUTTASTEPS, odeTau);
        else{
            EV_ERROR << "Plant::base_initialize: unidentified plant type of computation !";
            return;
        }

        initialized = true;
    }
    virtual ~Plant(){
        delete solver;
    }
    std::vector<double> Compute(const std::vector<double>& x, const std::vector<double>& u){

        std::vector<double> ret = x;

        if(!initialized){
            EV_ERROR << "Plant::Compute: plant is not initialized. Call any initialization function first !";
            return ret;
        }

        if(x.size() != ssdim || u.size() != isdim){
            EV_ERROR << "Plant::Compute: invalid dimensions !";
            return ret;
        }

       if(plantType == PLANT_TYPE::ODE){
            void (Plant::*func)(std::vector<double>&, std::vector<double>, std::vector<double>);
            func = &Plant::oderhs;
            solver->solve(*this, func, ret, u);
            return ret;
        }
        else{
            EV_ERROR << "Plant::Compute: unidentified plant type of computation !";
            return ret;
        }
    }

    virtual void initOde(double tau)=0;
    virtual void oderhs(std::vector<double>& xnew, std::vector<double> x, std::vector<double> u)=0;
};

class PLANT_MANAGER{
private:
    static const std::string PLANT_MODEL_VEHICLE;
    static const std::string PLANT_MODEL_ROBOT;

public:
    static Plant* NewPlantModel(const std::string& plantmodel);
};


#endif /* PLANT_H_ */
