classdef FIFOStateReconstructor < handle 
% some matlab commands to load points from a file stored by
% SCS:SymbolicSet::writeToFile() 
% 
% Please have a look at ./manual/manual.pdf for usage details
%
  properties (SetAccess=private)
    NscMax
    NcaMax
    u0
    system_ode
    qValues  % array of the current q values
    xValues  % array of the current x values
    uValues
    tau
    
    isGotStat
    isNCSStateReady
  end
  methods 
    function obj=FIFOStateReconstructor(NscMax, NcaMax, u0, system_ode, tau)
        % the constructor iniate the vvariables
        obj.tau = tau;
        obj.NscMax = NscMax;
        obj.NcaMax = NcaMax;
        obj.u0 = u0;
        obj.system_ode = system_ode;
        obj.qValues = ones(1,NcaMax);
        obj.xValues = cell(1,NcaMax);
        obj.uValues = cell(1,NscMax+NcaMax);
        obj.isGotStat = 0;
        obj.isNCSStateReady = 0;
        
        for i=1:NscMax+NcaMax
            obj.uValues{i}=u0;
        end
        
    end
        
    function new = shift_and_fill(obj, old, val)
        old(end) = [];
        old = [val old];
        new = old;
    end
    
    
    function pushQ(obj, u)
        if(obj.isGotStat == 1)
            error('In FIFO, you cant pass q after getting a state !');
        end
        obj.uValues = obj.shift_and_fill(obj.uValues, u);
    end
    
    function pushState(obj, xnew, unew)
        obj.isGotStat = 1;
        
        % 0- u
        obj.uValues = obj.shift_and_fill(obj.uValues, unew);
        
        % 1- no more qs
        obj.qValues = zeros(1,obj.NcaMax);
        
        % 2- compute the ncs state by the simulating the ode with the
        % stored input
        x = xnew;
        for i=1:obj.NscMax-1            
            obj.xValues{obj.NscMax-i+1} = x;   
            [t xfull]=ode45(@obj.system_ode,[0 obj.tau], x, odeset('abstol',1e-4,'reltol',1e-4),obj.uValues{obj.NcaMax + i});
            x = xfull(end,:);
        end        
        obj.xValues{1} = x;
        
    end
    
    function [qVal, xVal] = getState(obj)
        qVal = obj.qValues; % this is already an array
        
        xVal = [];
        for i=1:obj.NscMax
            xVal = [xVal obj.xValues{i}];
        end        
        for i=1:obj.NcaMax
            xVal = [xVal obj.uValues{i}];
        end                
    end
   
    
  end
end
