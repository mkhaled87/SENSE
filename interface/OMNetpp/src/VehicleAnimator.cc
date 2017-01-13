//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2015 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#include <omnetpp.h>
#include "vehicleAnimatorPkt_m.h"
#include "ncsmisc.h"

#define LEFT_SHIFT 250
#define TOP_SHIFT 25

using namespace omnetpp;

namespace ncs {

    class VehicleAnimator : public cSimpleModule
    {
        private:
            simtime_t timeStep;
            cFigure::Point loc;
            double heading;
            cImageFigure *car;
            cPolylineFigure *trail;

            double UnitScale;
            double SsXLb, SsXUb, SsYLb, SsYUb;
            int numTargets, numObsticles;
            std::vector<double> TargetX1, TargetX2, TargetY1, TargetY2;
            std::vector<double> ObsticleX1, ObsticleX2, ObsticleY1, ObsticleY2;
            std::vector<cRectangleFigure> figTargets;
            std::vector<cRectangleFigure> figObsticles;
            cRectangleFigure grid;

        protected:
            void DrawArena();
            virtual void initialize() override;
            virtual void handleMessage(cMessage *msg) override;
            void refresh();
            ~VehicleAnimator();
    };

    Define_Module(VehicleAnimator);

    void VehicleAnimator::DrawArena(){
        cCanvas *canvas = getParentModule()->getCanvas();

        omnetpp::cAbstractImageFigure::Rectangle r;

        cFigure::Color obst_color = cFigure::Color(0x4e, 0xaa, 0xce);
        cFigure::Color target_color = cFigure::Color(0xdb, 0x6e, 0x4c);

        // The Grid
        r.x=LEFT_SHIFT;
        r.y=TOP_SHIFT;
        r.width=(SsXUb-SsXLb)*UnitScale;
        r.height=(SsYUb-SsYLb)*UnitScale;

        // the trail
        trail->setLineColor(cFigure::BLACK);

        // the car
        car->setImageName(par("Icon"));

        //grid = new cRectangleFigure();
        grid.setBounds(r);
        grid.setCornerRadius(5);
        grid.setFilled(true);
        grid.setLineColor(cFigure::BLACK);
        grid.setFillColor(cFigure::WHITE);
        grid.setFillOpacity(0.9);
        grid.setName("grid");
        canvas->addFigureBelow(&grid, trail);

        // The Obsticles
        figObsticles.resize(numObsticles);
        for(size_t i=0; i<numObsticles; i++){
            r.width  = (ObsticleX2[i]-ObsticleX1[i])*UnitScale;
            r.height = (ObsticleY2[i]-ObsticleY1[i])*UnitScale;
            r.x = LEFT_SHIFT + ObsticleX1[i]*UnitScale;
            r.y = TOP_SHIFT + grid.getBounds().height - r.height - ObsticleY1[i]*UnitScale;
            figObsticles[i].setBounds(r);
            figObsticles[i].setCornerRadius(2);
            figObsticles[i].setFilled(true);
            figObsticles[i].setLineColor(cFigure::BLACK);
            figObsticles[i].setFillColor(obst_color);
            std::stringstream ss;
            ss << "obsticle-" << i+1;
            figObsticles[i].setName(ss.str().c_str());
            canvas->addFigureBelow(&figObsticles[i], trail);
        }

        // The Targets
        figTargets.resize(numTargets);
        for(size_t i=0; i<numTargets; i++){
            cRectangleFigure figTmp;
            r.width  = (TargetX2[i]-TargetX1[i])*UnitScale;
            r.height = (TargetY2[i]-TargetY1[i])*UnitScale;
            r.x = LEFT_SHIFT + TargetX1[i]*UnitScale;
            r.y = TOP_SHIFT + grid.getBounds().height - r.height - TargetY1[i]*UnitScale;
            figTargets[i].setBounds(r);
            figTargets[i].setCornerRadius(2);
            figTargets[i].setFilled(true);
            figTargets[i].setLineColor(cFigure::BLACK);
            figTargets[i].setFillColor(target_color);
            std::stringstream ss;
            ss << "target-" << i+1;
            figTargets[i].setName(ss.str().c_str());
            canvas->addFigureBelow(&figTargets[i], trail);
        }
    }

    VehicleAnimator::~VehicleAnimator(){
        cCanvas *canvas = getParentModule()->getCanvas();
        canvas->removeFigure(&grid);
        for(size_t i=0; i<numObsticles; i++)
            canvas->removeFigure(&figObsticles[i]);
        for(size_t i=0; i<numTargets; i++)
            canvas->removeFigure(&figTargets[i]);
    }
    void VehicleAnimator::initialize()
    {
        UnitScale = par("UnitScale");

        SsXLb = par("SsXLb");
        SsXUb = par("SsXUb");
        SsYLb = par("SsYLb");
        SsYUb = par("SsYUb");

        numTargets = par("numTargets");
        numObsticles = par("numObsticles");

        TargetX1 = ncsMisc::CssToVectorDouble(par("TargetX1"));
        TargetX2 = ncsMisc::CssToVectorDouble(par("TargetX2"));
        TargetY1 = ncsMisc::CssToVectorDouble(par("TargetY1"));
        TargetY2 = ncsMisc::CssToVectorDouble(par("TargetY2"));

        ObsticleX1 = ncsMisc::CssToVectorDouble(par("ObsticleX1"));
        ObsticleX2 = ncsMisc::CssToVectorDouble(par("ObsticleX2"));
        ObsticleY1 = ncsMisc::CssToVectorDouble(par("ObsticleY1"));
        ObsticleY2 = ncsMisc::CssToVectorDouble(par("ObsticleY2"));

        timeStep = 0.1;

        //heading = M_PI/4;
        //loc.x = 0;
        //loc.y = 0;

        cCanvas *canvas = getParentModule()->getCanvas();

        trail = check_and_cast<cPolylineFigure *>(canvas->getFigure("trail"));
        car = check_and_cast<cImageFigure *>(canvas->getFigure("car"));

        DrawArena();

        WATCH(timeStep);
        WATCH(loc.x);
        WATCH(loc.y);
        WATCH(heading);

        //refresh();
        //scheduleAt(simTime(), new cMessage());
    }

    void VehicleAnimator::refresh()
    {
        double iconScale = par("IconScale");

        cFigure::Transform t;
        t.rotate(-heading);
        t.scale(iconScale);
        t.translate(LEFT_SHIFT+loc.x, TOP_SHIFT+grid.getBounds().height-loc.y);
        car->setTransform(t);

        cFigure::Point trail_loc(LEFT_SHIFT+loc.x, TOP_SHIFT+grid.getBounds().height-loc.y);
        trail->addPoint(trail_loc);
        //if (trail->getNumPoints() > 500)
        //    trail->removePoint(0);
     }

    void VehicleAnimator::handleMessage(cMessage *msg)
    {
        if(msg->isPacket()){
            vehcileAnimatorPkt* animatorPkt = (vehcileAnimatorPkt*)msg;
            // move
            heading = animatorPkt->getTheta();
            loc.x   = animatorPkt->getX()*UnitScale;
            loc.y   = animatorPkt->getY()*UnitScale;

            refresh();
            cancelAndDelete(msg);
            //scheduleAt(simTime() + timeStep, msg);
        }
    }

}
