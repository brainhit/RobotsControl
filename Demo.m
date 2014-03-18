%% function: using pso algorithm for the multi-robot control
% author: Yan Ou
% date: 20130418

function Demo
%% add path
addpath(genpath('..\RobotsControl'));

%% clear all
warning off; close all;

%% set initial value
PLOT =  true;
constantValue.globalMinTolerance = 5; % set the tolerance for the global minimum to stop the optimization process
constantValue.initialState = [0,20,0;0,0,0]; % set the initial state (x,y,theta) for the cells
constantValue.goalState = [50,120;200,-160]; % set the goal state (x,y) for the cells
constantValue.alpha = [0.7;0.4]; % set the angular changing rate of cell
constantValue.speed = [30;50]; % set the speed of cell
constantValue.step = 15; % how many steps we need to make cell head to the target
constantValue.gridDimension = 4; % equally separate the one dimension into gridDimension+1 parts
constantValue.A = [eye(constantValue.step);-eye(constantValue.step)];
constantValue.b = pi*ones(2*constantValue.step,1); % build the inequality matrix for the fmincon function
constantValue.lb = -pi*ones(constantValue.step,1); % lower bound
constantValue.ub = pi*ones(constantValue.step,1); % upper bound
constantValue.randomStartNum = 20; % choose the number of the start point to find the local minima
constantValue.PSOiterations = 20;
constantValue.PSOinertia = 0.1;
constantValue.PSOcorrection_factor = 2.0;
constantValue.PSOswarm_size = 200; % PSO paramters
constantValue.cellNo = 2; % number of cells

%% solve the cost function and record the computational time
iter = 10;
f = 0;
tic
for i = 1:iter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
thetaM = [];
funPsoTP = PSOTPFormat(constantValue); % form the parameter for the psoTP
[thetaM,fval] = psoTP(funPsoTP,200,constantValue.lb,constantValue.ub,0.1,2,40,10,thetaM);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f = f+fval;
end
disp(['The average time consumption of the PSO algorithm in this example is: ',num2str(toc/iter),' second']);
disp(['The average error is: ',num2str(f/iter),' px']);

%% plot the path that generated from the plant inputs
if PLOT == true
figure(1);
title('Cell Motion');
xlabel('x axis (px)');
ylabel('y axis (px)');
color = ['r','g','k','y','b'];
width = 3;
height = 10;
r = 5;
for i = 1:constantValue.cellNo
    rectangle('Position',[constantValue.initialState(i,1)-width/2,constantValue.initialState(i,2)-height/2,width,height],'FaceColor',color(i));
    rectangle('Position',[constantValue.initialState(i,1)-height/2,constantValue.initialState(i,2)-width/2,height,width],'FaceColor',color(i));
    hold on
    filledCircle([constantValue.goalState(i,1),constantValue.goalState(i,2)],r,1000,color(i));
end
axis equal
cellNo = constantValue.cellNo;
speed = constantValue.speed(1:cellNo); % cell speed
alpha = constantValue.alpha(1:cellNo); % cell angular changing rate
xCell = zeros(constantValue.step+1,constantValue.cellNo);
yCell = zeros(constantValue.step+1,constantValue.cellNo);
thetaCell = xCell;
xCell(1,:) = constantValue.initialState(1:cellNo,1); % cell starting point
yCell(1,:) = constantValue.initialState(1:cellNo,2);
thetaCell(1,:) = constantValue.initialState(1:cellNo,3);
for i = 1:constantValue.step
    thetaCell(i+1,:) = thetaCell(i,:) + alpha'.*sin(thetaM(i)-thetaCell(i,:));
    xCell(i+1,:) = xCell(i,:) + speed'.*cos(thetaCell(i+1,:));
    yCell(i+1,:) = yCell(i,:) + speed'.*sin(thetaCell(i+1,:));
end
for j = 1:constantValue.cellNo
    for i = 1:(constantValue.step)
        cell = plot([xCell(i,j) xCell(i+1,j)], [yCell(i,j) yCell(i+1,j)],['--',color(j),'*']);
    end
end
for j = 1:constantValue.cellNo
    for i = 1:(constantValue.step+1)
        drawCell(xCell(i,j),yCell(i,j),thetaCell(i,j),8,3);
    end
end
end

end