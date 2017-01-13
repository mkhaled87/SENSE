#include "Plant.h"

class RobotPlant : public Plant{

public:
    RobotPlant(){}
    ~RobotPlant(){}

    void
    initOde(double tau){
        odeTau = tau;
        base_initialize(2, 2, PLANT_TYPE::ODE);
    }

    void
    oderhs(std::vector<double>& xnew, std::vector<double> x, std::vector<double> u){
        xnew[0] = u[0];
        xnew[1] = u[1];
    }
};

class VeciclePlant : public Plant{

public:
    VeciclePlant(){}
    ~VeciclePlant(){}

    void
    initOde(double tau){
        odeTau = tau;
        base_initialize(3, 2, PLANT_TYPE::ODE);
    }

    void
    oderhs(std::vector<double>& xnew, std::vector<double> x, std::vector<double> u){
        double alpha=std::atan(std::tan(u[1]/2.0));
        xnew[0] = u[0]*std::cos(alpha+x[2])/std::cos(alpha);
        xnew[1] = u[0]*std::sin(alpha+x[2])/std::cos(alpha);
        xnew[2] = u[0]*std::tan(u[1]);
    }
};


// define a string for each derived class that will be used from inside the omnetpp.ini file
const std::string PLANT_MANAGER::PLANT_MODEL_VEHICLE = "vehicle";
const std::string PLANT_MANAGER::PLANT_MODEL_ROBOT = "robot";

Plant* PLANT_MANAGER::NewPlantModel(const std::string& plantmodel){
    Plant* plant = nullptr;

    if(plantmodel.compare(PLANT_MODEL_VEHICLE) == 0)
        plant = new VeciclePlant();
    else if(plantmodel.compare(PLANT_MODEL_ROBOT) == 0)
        plant = new RobotPlant();
    else
        EV_ERROR << "unidentified plant model !";

    return plant;
}

