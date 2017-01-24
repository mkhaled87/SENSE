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

function tworobotsncs
clear set
close all
clear all
clc

hold on;

ARENA_WIDTH = 16;
ARENA_HIGHT = 16;
ARENA_XMAX = [-1 (ARENA_WIDTH)];
ARENA_YMAX = [-1 (ARENA_HIGHT)];

% simulation time
SIM_STEPS   = 150;

% initial state
x0=[7 15 15 7];
u0=[1 0 0 1];
tau = 1;

% NCS
ssDim  = 4;
isDim  = 4;
Nscmax = 2;
Ncamax = 2;

% the NCS state reconstructor
StateReconstructor =  FIFOStateReconstructor(Nscmax, Ncamax, u0', @robot_ode, tau);

disp('loading the controllers NBD files ... ');
contr1=ncsFIFOController('tworobots_contr1.nbdd');
contr2=ncsFIFOController('tworobots_contr3.nbdd');
contr3=ncsFIFOController('tworobots_contr3.nbdd');
contr4=ncsFIFOController('tworobots_contr4.nbdd');
disp('done !');

target1=SymbolicSet('scots-files/tworobots_ts1.bdd','projection',[1 2]);
target2=SymbolicSet('scots-files/tworobots_ts2.bdd','projection',[1 2]);
target3=SymbolicSet('scots-files/tworobots_ts3.bdd','projection',[3 4]);
target4=SymbolicSet('scots-files/tworobots_ts4.bdd','projection',[3 4]);

    
%% 2] plotting !
% colors
colors=get(groot,'DefaultAxesColorOrder');

% load the symbolic set containig obstacles
set=SymbolicSet('scots-files/tworobots_obst_matlab.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)
hold on

% load the symbolic set containig the abstract state space
set=SymbolicSet('scots-files/tworobots_ss.bdd','projection',[1 2]);
plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on

% plot the real obstacles and target set
% plot_domain

% plot target sets
plotCells(target1,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)
plotCells(target2,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)
plotCells(target3,'facecolor',colors(3,:)*0.5+0.5,'edgec',colors(3,:),'linew',.1)
plotCells(target4,'facecolor',colors(3,:)*0.5+0.5,'edgec',colors(3,:),'linew',.1)

box on
axis([ARENA_XMAX ARENA_YMAX])

%% 3] Simulation
robot1_to = 1;
robot2_to = 3;
y=x0;
isError = 0; 
tmp = 0;
plot(y(1,1),y(1,2),'.','color',colors(4,:),'markersize',20)
plot(y(1,3),y(1,4),'.','color',colors(5,:),'markersize',20)
drawnow
buffer_sc = cell(ssDim,Nscmax);
buffer_ca = cell(isDim,Ncamax); 

  for k=1:length(buffer_ca)
    buffer_ca(:,k)=num2cell(u0);
  end

  for t=1:SIM_STEPS
    % NCS filling SC channel:    
    buffer_sc(:, end) = [];
    buffer_sc = [num2cell(y(end,:))' buffer_sc];
    
    % are we ready to construct the xBig ?
    if(isempty(buffer_sc{end, end}))
        StateReconstructor.pushQ(u0);
        u = u0;        
    else
        StateReconstructor.pushState([buffer_sc{1,end} buffer_sc{2,end} buffer_sc{3,end} buffer_sc{4,end}], ...
            [buffer_ca{1,1} buffer_ca{2,1} buffer_ca{3,1} buffer_ca{4,1}]);
        [new_qValues, new_xValues] = StateReconstructor.getState();
        
        qValues = [0 0];
        xValues = [ buffer_sc{1,1} buffer_sc{2,1} buffer_sc{3,1} buffer_sc{4,1}...
                    buffer_sc{1,2} buffer_sc{2,2} buffer_sc{3,2} buffer_sc{4,2}...
                    buffer_ca{1,1} buffer_ca{2,1} buffer_ca{3,1} buffer_ca{4,1}...
                    buffer_ca{1,2} buffer_ca{2,2} buffer_ca{3,2} buffer_ca{4,2}];
        
        if(~isequal(qValues, new_qValues) || ~isequal(xValues, new_xValues))
            error('state reconstruction is not correct !');
        end
        
        try                
            sys_state = [buffer_sc{1,1} buffer_sc{2,1} buffer_sc{3,1} buffer_sc{4,1}];
            y_robot1 = round(sys_state(1:2));
            y_robot2 = round(sys_state(3:4));
  
            if(y_robot1(1)==y_robot2(1) && y_robot1(2)==y_robot2(2))
                error('The robots hit eachothers !');
            end
  
            if(y_robot1(1)<0 || y_robot1(2)>15 || y_robot2(1)<0 || y_robot2(2)>15)
                error('Robots is out of range !!');
            end
  
            if(target1.isElement([y_robot1 0 0]))
                %disp('Robot 1 reached target 1 and will be oriented to target 2 !');
                robot1_to = 2;
            elseif(target2.isElement([y_robot1 0 0]))
                %disp('Robot 1 reached target 2 and will be oriented to target 1 !');
                robot1_to = 1;
            end

            if(target3.isElement([0 0 y_robot2]))
                %disp('Robot 2 reached target 3 and will be oriented to target 4 !');
                robot2_to = 4;
            elseif(target4.isElement([0 0 y_robot2]))
                %disp('Robot 2 reached target 4 and will be oriented to target 3 !');
                robot2_to = 3;
            end

            mode = direction_to_mode(robot1_to, robot2_to);
            if(mode == 1)
                u=contr1.getInputs(qValues, xValues);
            elseif(mode == 2)
                u=contr2.getInputs(qValues, xValues);
            elseif(mode == 3)
                u=contr3.getInputs(qValues, xValues);
            elseif(mode == 4)
                u=contr4.getInputs(qValues, xValues);
            elseif(mode == 0)
                error('Invalid Mode !');
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
    %u_selected = u(:,ceil(rand*size(u,2)));
    u_selected = u(1,:)';    
    u_at_sys = cell2mat(buffer_ca(:, end))';
    
    % NCS: filling ca channel
    buffer_ca(:, end) = [];
    buffer_ca = [num2cell(u_selected) buffer_ca];

    %simulate and update
    [t x]=ode45(@robot_ode,[0 tau], y(end,:), odeset('abstol',1e-4,'reltol',1e-4),u_at_sys);
    y=[y; x(end,:)];
    
    plot(y(end-1:end,1),y(end-1:end,2),'k.-','color',colors(4,:))
    plot(y(end-1:end,3),y(end-1:end,4),'k.-','color',colors(5,:))
    drawnow
  end

  if(isError==1)
      %plot(y(end-1,1),y(end-1,2),'o','color','r')
      plot(y(end,1),y(end,2),'o','color','g')
  end   

  xlabel('X-coordinate');
  ylabel('Y-coordinate');

  figure
  plot(0:SIM_STEPS, y(:,1)', 'color',colors(4,:));
  hold on
  plot(0:SIM_STEPS, y(:,3)','color',colors(5,:));
  grid on
  xlabel('Time');
  ylabel('X-coordinate');

  figure
  plot(0:SIM_STEPS, y(:,2)', 'color',colors(4,:));
  hold on
  plot(0:SIM_STEPS, y(:,4)','color',colors(5,:));
  grid on
  xlabel('Time');
  ylabel('Y-coordinate');

  assignin('base', 'y', y);  

end

function m = direction_to_mode(robot1_dir, robot2_dir)
       if(robot1_dir == 1 && robot2_dir == 3)
      m=1;
   elseif(robot1_dir == 1 && robot2_dir == 4)
      m=2;
   elseif(robot1_dir == 2 && robot2_dir == 3)
      m=3;
   elseif(robot1_dir == 2 && robot2_dir == 4)
      m=4;
   else
      m=0;
   end
end

function dxdt = robot_ode(t,x,u)
  dxdt = zeros(4,1);
  dxdt(1)=u(1);
  dxdt(2)=u(2);
  dxdt(3)=u(3);
  dxdt(4)=u(4);
end

