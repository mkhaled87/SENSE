%
% dintncs.m
%
% created on: 04.10.2016
%     author: M. Khaled
%
% you need to run ./dint_ncs binary first 
%
% so that the file: dint_cont.nbdd is created
%

function dint
clear set
close all
clear all
clc


% initial state of the original system (non NCS)
x0 = [0.6 0.6];
u0 = 0.9;

% NCS
Nscmax = 3;
Ncamax = 3;

%% 1] load the symbolic sets
contr = ncsFIFOController('dint_contr.nbdd');

    
%% 2] plotting !
% colors
colors=get(groot,'DefaultAxesColorOrder');

% load the symbolic set containig the abstract state space
set=SymbolicSet('scots-files/dint_ss.bdd');
plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on

% load the symbolic set containig target set
set=SymbolicSet('scots-files/dint_ts.bdd');
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

box on
axis([0.3 3.7 0 3.7])


%% 3] Simulation
for i=1:20
buffer_sc = cell(1,Nscmax);
buffer_ca = cell(1,Ncamax); 

    for k=1:length(buffer_ca)
        buffer_ca{k}=u0;
    end

y=x0;
v=[];
tau = 0.3;
SIM_STEPS=100; 
isError = 0; 

  for t=1:SIM_STEPS/tau
      
    % Chech end point (When TS is reached !)
    if(y(end,1) >= 2.4 && y(end,2) >= 1.5)
        break;
    end

    % NCS filling SC channel:    
    buffer_sc(end) = [];
    buffer_sc = [y(end,:) buffer_sc];
    
    % are we ready to construct the xBig ?
    if(isempty(buffer_sc{end}))
        u = u0;
    else
        qValues = [0 0 0];
        xValues = [ buffer_sc{1}(1) buffer_sc{1}(2) ...
                    buffer_sc{2}(1) buffer_sc{2}(2) ...
                    buffer_sc{3}(1) buffer_sc{3}(2) ...
                    buffer_ca{1}(1)...
                    buffer_ca{2}(1)...
                    buffer_ca{3}(1)];
        try
	    u=contr.getInputs(qValues, xValues);
            
            if(isempty(u))
                isError = 1;
                break;
            end
            
        catch mexp
            isError = 1;
            break;
        end
    end
    
    % selecting one u from the offered inputs
    u_selected = u(ceil(rand*length(u)));
    
    u_at_sys = buffer_ca{end};
    
    % NCS: filling ca channel
    buffer_ca(end) = [];
    buffer_ca = [u_selected buffer_ca];

    %simulate and update
    [t x]=ode45(@dint_ode,[0 tau], y(end,:), odeset('abstol',1e-4,'reltol',1e-4),u_at_sys);
    y=[y; x(end,:)];

  end

 % initial state  and trajectory:
plot(y(:,1),y(:,2),'.-','color',colors(1,:))
plot(y(1,1),y(1,2),'.','color',colors(5,:),'markersize',20)
hold on 

if(isError==1)
    plot(y(end-1,1),y(end-1,2),'o','color','r')
    plot(y(end,1),y(end,2),'o','color','g')
end    
end
 
end

function dxdt = dint_ode(t,x,u)
    B = [0; 1];
    A = [0 1; 0 0];
    dxdt= A*x+B*u;
end

