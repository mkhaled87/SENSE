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
ARENA_WIDTH = 16;
ARENA_HIGHT = 16;
ARENA_XMAX = [-1 (ARENA_WIDTH)];
ARENA_YMAX = [-1 (ARENA_HIGHT)];

% simulation time
SIM_STEPS   = 150;

% initial state
x0=[7 15 15 7];
tau = 1;

cont1=SymbolicSet('tworobots_cont1.bdd','projection',[1 2 3 4]);
cont2=SymbolicSet('tworobots_cont2.bdd','projection',[1 2 3 4]);
cont3=SymbolicSet('tworobots_cont3.bdd','projection',[1 2 3 4]);
cont4=SymbolicSet('tworobots_cont4.bdd','projection',[1 2 3 4]);

target1=SymbolicSet('tworobots_ts1.bdd','projection',[1 2]);
target2=SymbolicSet('tworobots_ts2.bdd','projection',[1 2]);
target3=SymbolicSet('tworobots_ts3.bdd','projection',[3 4]);
target4=SymbolicSet('tworobots_ts4.bdd','projection',[3 4]);

%% plot the vehicle domain
% colors
colors=get(groot,'DefaultAxesColorOrder');

% load the symbolic set containig the abstract state space
% set=SymbolicSet('robot_ss.bdd');
% plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on
box on
axis([ARENA_XMAX ARENA_YMAX])

% load the symbolic set containig obstacles
obst=SymbolicSet('tworobots_obst_matlab.bdd','projection',[1 2]);
plotCells(obst,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)

% plot targets
plotCells(target1,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)
plotCells(target2,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)
plotCells(target3,'facecolor',colors(3,:)*0.5+0.5,'edgec',colors(3,:),'linew',.1)
plotCells(target4,'facecolor',colors(3,:)*0.5+0.5,'edgec',colors(3,:),'linew',.1)

%% Simulation
robot1_to = 1;
robot2_to = 3;
y=x0;
i=0;
tmp = 0;
plot(y(1,1),y(1,2),'.','color',colors(4,:),'markersize',20)
plot(y(1,3),y(1,4),'.','color',colors(5,:),'markersize',20)
drawnow
while(i<SIM_STEPS)  
  y_robot1 = round(y(end,1:2));
  y_robot2 = round(y(end,3:4));
  
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
  %disp(['Using controller of mode: ' num2str(mode)]);
  if(mode == 1)
    u=cont1.getInputs(y(end,:));
  elseif(mode == 2)
    u=cont2.getInputs(y(end,:));
  elseif(mode == 3)
    u=cont3.getInputs(y(end,:));
  elseif(mode == 4)
    u=cont4.getInputs(y(end,:));
  elseif(mode == 0)
    error('Invalid Mode !');
  end
  
  u_selected = u(1,:);
  %u_selected = u(ceil(rand*size(u,1)),:);      

  [t x]=ode45(@robot_ode,[0 tau], y(end,:),[],u_selected);

  y=[y; x(end,:)];
  i = i+1;
  
  plot(y(end-1:end,1),y(end-1:end,2),'k.-','color',colors(4,:))
  plot(y(end-1:end,3),y(end-1:end,4),'k.-','color',colors(5,:))
  drawnow
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

%% plot initial state  and trajectory



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
