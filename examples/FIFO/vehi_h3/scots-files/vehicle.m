%
% vehicle.m
%
% created on: 09.10.2015
%     author: rungger
%
% see readme file for more information on the vehicle example
%
% you need to run ./vehicle binary first 
%
% so that the files: vehicle_ss.bdd 
%                    vehicle_obst.bdd
%                    vehicle_target.bdd
%                    vehicle_controller.bdd 
% are created
%

function vehicle
clear set
close all

%% simulation

% target set
lb=[4 0];
ub=lb+0.5;

% initial state
x0=[0.4 4.4 0];

controller=SymbolicSet('vehicle_controller.bdd','projection',[1 2 3]);
target=SymbolicSet('vehicle_target.bdd');

y=x0;
v=[];
while(1)

  u=controller.getInputs(y(end,:));
 
  r = 1+floor(size(u,1)*rand);
  
  u_rand = u(r,:);
  
  v=[v; u_rand];
  if (target.isElement(y(end,:)))
    break;
  end 
  [t x]=ode45(@vehicle_ode,[0 .3], y(end,:),[],u_rand);

  y=[y; x(end,:)];
end

%% plot the vehicle domain
% colors
colors=get(groot,'DefaultAxesColorOrder');

% load the symbolic set containig obstacles
set=SymbolicSet('vehicle_obst.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)
hold on

% load the symbolic set containig the abstract state space
set=SymbolicSet('vehicle_ss.bdd','projection',[1 2]);
plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on

% plot the real obstacles and target set
% plot_domain

% load the symbolic set containig target set
set=SymbolicSet('vehicle_target.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

% plot initial state  and trajectory
plot(y(:,1),y(:,2),'k.-')
plot(y(1,1),y(1,2),'.','color',colors(5,:),'markersize',20)

box on
axis([-.5 5.5 -.5 5.5])

end


function dxdt = vehicle_ode(t,x,u)

  dxdt = zeros(3,1);
  c=atan(tan(u(2)/2));

  dxdt(1)=u(1)*cos(c+x(3))/cos(c);
  dxdt(2)=u(1)*sin(c+x(3))/cos(c);
  dxdt(3)=u(1)*tan(u(2));


end

