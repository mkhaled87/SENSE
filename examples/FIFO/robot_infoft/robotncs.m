%
% robotncs.m
%
% created on: 15.11.2016
%     author: M. khaled
%
% you need to run ./robot_ncs binary first 
%
% so that the file: robot_cont.nbdd is created
%

function vehicle_ncs
clear set
close all
clear all
clc

hold on;

ARENA_XMAX = [-1 64];
ARENA_YMAX = [-1 64];

% initial state of the original system (non NCS)
x0 = [10; 10];
u0 = [ 1;  1];

% Plant and Network
ssDim  = 2;
isDim  = 2;
tau    = 1;
Nscmax = 2;
Ncamax = 2;

%simulation
SIM_STEPS_MAX   = 1000; 
SIM_REPEATS = 1;

% the NCS state reconstructor
StateReconstructor =  FIFOStateReconstructor(Nscmax, Ncamax, u0', @robot_ode, tau);



%% 1] load the controller
disp('loading the controllers NBD files ... ');
contr1 = ncsFIFOController('robot_contr1.nbdd');
contr2 = ncsFIFOController('robot_contr2.nbdd');
disp('done !');

target1=SymbolicSet('scots-files/robot_ts1.bdd');
target2=SymbolicSet('scots-files/robot_ts2.bdd');


    
%% 2] plotting !
% colors
colors=get(groot,'DefaultAxesColorOrder');

% load the symbolic set containig obstacles
set=SymbolicSet('scots-files/robot_obst.bdd');
plotCells(set,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)
hold on

% load the symbolic set containig the abstract state space
set=SymbolicSet('scots-files/robot_ss.bdd');
plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on

% plot the real obstacles and target set
% plot_domain

% load the symbolic set containig target set
set=SymbolicSet('scots-files/robot_ts1.bdd');
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)
set=SymbolicSet('scots-files/robot_ts2.bdd');
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

box on
axis([ARENA_XMAX ARENA_YMAX])

%% 3] Simulation
for i=1:SIM_REPEATS
    
    mode = 1;
    tmp = 0;
    
buffer_sc = cell(ssDim,Nscmax);
buffer_ca = cell(isDim,Ncamax); 

    for k=1:length(buffer_ca)
        buffer_ca(:,k)=num2cell(u0);
    end

y=x0';
v=[];
isError = 0; 

  for t=1:(SIM_STEPS_MAX/tau)
    % NCS filling SC channel:    
    buffer_sc(:, end) = [];
    buffer_sc = [num2cell(y(end,:))' buffer_sc];
    
    % are we ready to construct the xBig ?
    if(isempty(buffer_sc{end, end}))
        StateReconstructor.pushQ(u0');
        u = u0;        
    else
        StateReconstructor.pushState([buffer_sc{1,end} buffer_sc{2,end}], [buffer_ca{1,1} buffer_ca{2,1}]);
        [new_qValues, new_xValues] = StateReconstructor.getState();
        
        qValues = [0 0];
        xValues = [ buffer_sc{1,1} buffer_sc{2,1} ...
                    buffer_sc{1,2} buffer_sc{2,2} ...
                    buffer_ca{1,1} buffer_ca{2,1} ...
                    buffer_ca{1,2} buffer_ca{2,2}];
        
        if(~isequal(qValues, new_qValues) || ~isequal(xValues, new_xValues))
            error('state reconstruction is not correct !');
        end
        
        try                
            sys_state = [buffer_sc{1,1} buffer_sc{2,1}];
            if(target1.isElement(sys_state) && mode == 1)
              mode = 2;
              tmp = tmp + 1;
              if tmp == 2
                break;
              end
            elseif (target2.isElement(sys_state) && mode == 2)
              mode = 1;
            end
           
            if(mode == 1)
              u=contr1.getInputs(qValues, xValues);
            elseif(mode == 2)
              u=contr2.getInputs(qValues, xValues);
            end
                        
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
    u_selected = u(:,ceil(rand*size(u,2)));
    %u_selected = u(:,1);
    
    u_at_sys = cell2mat(buffer_ca(:, end))';
    
    % NCS: filling ca channel
    buffer_ca(:, end) = [];
    buffer_ca = [num2cell(u_selected) buffer_ca];

    %simulate and update
    [t x]=ode45(@robot_ode,[0 tau], y(end,:), odeset('abstol',1e-4,'reltol',1e-4),u_at_sys);
    y=[y; x(end,:)];

  end

  % initial state  and trajectory:
  plot(y(:,1),y(:,2),'.-','color',colors(2,:))
  plot(y(1,1),y(1,2),'.','color',colors(5,:),'markersize',20)
  hold on 

  if(isError==1)
      %plot(y(end-1,1),y(end-1,2),'o','color','r')
      plot(y(end,1),y(end,2),'o','color','g')
  end    

end
mexNcsController('close');
end

function dxdt = robot_ode(t,x,u)
  dxdt = zeros(2,1);
  dxdt(1)=u(1);
  dxdt(2)=u(2);
end

