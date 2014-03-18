%% function: form the cost function for the cells head to diff targets
% author: Yan Ou
% date: 20130418

function f = CostFuncTargets(x,constantValue)
% set the constant values
cellNo = constantValue.cellNo; % cell number
speed = constantValue.speed(1:cellNo); % cell speed
alpha = constantValue.alpha(1:cellNo); % cell angular changing rate
xIni = constantValue.initialState(1:cellNo,1); % cell starting point
yIni = constantValue.initialState(1:cellNo,2);
thetaIni = constantValue.initialState(1:cellNo,3);
xSetPoint = constantValue.goalState(1:cellNo,1); % cell ending point
ySetPoint = constantValue.goalState(1:cellNo,2);
step = constantValue.step; % how many step to head to the target

% form the cost function
xCell = xIni;
yCell = yIni;
thetaCell = thetaIni; % set the initial cell position
for i = 1:step
    thetaCell = thetaCell + alpha.*sin(x(i)-thetaCell);
    xCell = xCell + speed.*cos(thetaCell);
    yCell = yCell + speed.*sin(thetaCell);
end % update the cell position based on the plant input
f = (norm(xCell - xSetPoint)^2 + norm(yCell - ySetPoint)^2)^0.5;
end