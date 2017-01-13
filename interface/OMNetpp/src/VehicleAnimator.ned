//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2015 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

package ncs;

simple VehicleAnimator
{   
    parameters:
        double UnitScale;
                       
        double SsXLb;
        double SsXUb;
        double SsYLb;
        double SsYUb;
        
        int numTargets;
        string TargetX1;
        string TargetX2;
        string TargetY1;
        string TargetY2;
        
        int numObsticles;
        string ObsticleX1;
        string ObsticleX2;
        string ObsticleY1;
        string ObsticleY2;
        
        string Icon;
        double IconScale;
        
    gates:
         input in;
}

