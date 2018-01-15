%
% vehiclencs.m
%
% created on: 07.10.2016
%     author: M. Khaled
%
% you need to run ./vehicle_ncs binary first 
%
% so that the file: vehicle_cont.nbdd is created
%

function vehicle_ncs
clear set
close all
clear all
clc


% initial state of the original system (non NCS)
x0 = [0.5; 0.5; 0.6];
u0 = [0.3; 0];

% Plant and Network
ssDim = 3;
isDim = 2;
tau = 0.3;
Nscmax = 2;
Ncamax = 2;

%simulation
SIM_STEPS = 100; 
SIM_REPEATS = 1;



%% 1] load the controller
disp('loading the controller NBD file ... ');
contr = ncsFIFOController('vehicle_contr.nbdd');
disp('done !');


    
%% 2] plotting !
% colors
colors=get(groot,'DefaultAxesColorOrder');

% load the symbolic set containig obstacles
set=SymbolicSet('scots-files/vehicle_obst.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)
hold on

% load the symbolic set containig the abstract state space
set=SymbolicSet('scots-files/vehicle_ss.bdd','projection',[1 2]);
plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on

% plot the real obstacles and target set
% plot_domain

% load the symbolic set containig target set
set=SymbolicSet('scots-files/vehicle_ts.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

box on
axis([-.5 5.5 -.5 5.5])


%% 3] Simulation
for i=1:SIM_REPEATS
buffer_sc = cell(ssDim,Nscmax);
buffer_ca = cell(isDim,Ncamax); 

    for k=1:length(buffer_ca)
        buffer_ca(:,k)=num2cell(u0);
    end

y=x0';
v=[];
isError = 0; 

  for t=1:SIM_STEPS/tau
      
    % Chech end point (When TS is reached !)
    if(isTargetReached(y(end,:)))
        disp('Target reached !!');
        break;
    end

    % NCS filling SC channel:    
    buffer_sc(:, end) = [];
    buffer_sc = [num2cell(y(end,:))' buffer_sc];
    
    % are we ready to construct the xBig ?
    if(isempty(buffer_sc{end, end}))
        u = u0;
    else
        qValues = [0 0];
        xValues = [ buffer_sc{1,1} buffer_sc{2,1} buffer_sc{3,1}...
                    buffer_sc{1,2} buffer_sc{2,2} buffer_sc{3,2} ...
                    buffer_ca{1,1} buffer_ca{2,1} ...
                    buffer_ca{1,2} buffer_ca{2,2}];
        try
            u=contr.getInputs(qValues, xValues);
            
            if(isempty(u))
                disp(['Error: no inputs for the state: ' num2str(xValues)]);
                isError = 1;
                break;
            end
            
        catch mexp
            disp(['Error during state:' num2str(xValues)]);
            disp(getReport(mexp));
            isError = 1;
            break;
        end
    end
    
    % selecting one u from the offered inputs
    %u_selected = u(:,ceil(rand*size(u,2)));
    u_selected = u(:,1);
    
    disp('selected input: ');
    disp(u_selected');
    disp('---------------------------------------------------------------------------------');
    
    
    u_at_sys = cell2mat(buffer_ca(:, end))';
    
    % NCS: filling ca channel
    buffer_ca(:, end) = [];
    buffer_ca = [num2cell(u_selected) buffer_ca];

    %simulate and update
    [t x]=ode45(@vehicle_ode,[0 tau], y(end,:), odeset('abstol',1e-4,'reltol',1e-4),u_at_sys);
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
mexNcsController('close');
end


function yesno = isTargetReached(y)
    yesno =  y(1) >= 4.1 && ...
             y(1) <= 4.5 && ...
             y(2) >= 4.1 && ...
             y(2) <= 4.9;
end

function dxdt = vehicle_ode(t,x,u)

  dxdt = zeros(3,1);
  c=atan(tan(u(2)/2));

  dxdt(1)=u(1)*cos(c+x(3))/cos(c);
  dxdt(2)=u(1)*sin(c+x(3))/cos(c);
  dxdt(3)=u(1)*tan(u(2));

end


