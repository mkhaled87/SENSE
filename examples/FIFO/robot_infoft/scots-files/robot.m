%
% robot.m
%
% created on: 15.11.2016
%     author: M. Khaled
%
% see readme file for more information on the vehicle example
%
% you need to run ./robot binary first 
%
% so that the files: robot_ss.bdd 
%                    robot_obst.bdd
%                    robot_target.bdd
%                    robot_controller.bdd 
% are created
%

function robot
clear set
close all



%% Initialization
ARENA_XMAX = [-1 64];
ARENA_YMAX = [-1 64];

% simulation time
SIM_STEPS   = 500;
SIM_REPEATS = 1;

% initial state
x0=[55 55];
tau = 1;

cont1=SymbolicSet('robot_cont1.bdd','projection',[1 2]);
cont2=SymbolicSet('robot_cont2.bdd','projection',[1 2]);

target1=SymbolicSet('robot_ts1.bdd');
target2=SymbolicSet('robot_ts2.bdd');

%% plot the vehicle domain
% colors
colors=get(groot,'DefaultAxesColorOrder');

% load the symbolic set containig the controller coverage
%set=SymbolicSet('robot_cont1.bdd','projection',[1 2]);
%plotCells(set,'facecolor',[255/255,255/255,153/255]*0.5+0.5)

%set=SymbolicSet('robot_cont2.bdd','projection',[1 2]);
%plotCells(set,'facecolor',[255/255,255/255,153/255]*0.5+0.5)

% load the symbolic set containig the abstract state space
% set=SymbolicSet('robot_ss.bdd');
% plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on

% load the symbolic set containig obstacles
set=SymbolicSet('robot_obst.bdd');
plotCells(set,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)

% load the symbolic set containig target set
set=SymbolicSet('robot_ts.bdd');
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

hold on

%% Simulation
for s=1:SIM_REPEATS
    y=x0;
    i=0;
    mode = 2;
    tmp = 0;
    while(i<SIM_STEPS)
      
       if(target1.isElement(y(end,:)))
           mode = 2;
           tmp = tmp + 1;
           if tmp == 2
               break;
           end
       elseif (target2.isElement(y(end,:)))
           mode = 1;
       end
        
      if(mode == 1)
        u=cont1.getInputs(y(end,:));
      elseif(mode == 2)
        u=cont2.getInputs(y(end,:));
      end
      
      u_selected = u(ceil(rand*size(u,1)),:);      

      [t x]=ode45(@robot_ode,[0 tau], y(end,:),[],u_selected);

      y=[y; x(end,:)];
      i = i+1;
    end


    %% plot initial state  and trajectory
    plot(y(:,1),y(:,2),'k.-')
    plot(y(1,1),y(1,2),'.','color',colors(5,:),'markersize',20)
end

box on
axis([ARENA_XMAX ARENA_YMAX])

end

function dxdt = robot_ode(t,x,u)
  dxdt = zeros(2,1);
  dxdt(1)=u(1);
  dxdt(2)=u(2);
end
