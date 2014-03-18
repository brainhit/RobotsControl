%% function: form the c++ format psoTP function input
% editor: Yan Ou
% date: 2013/12/23

function [funPsoTP] = PSOTPFormat(constantValue)
cellNo = constantValue.cellNo;
cellInfo = [constantValue.speed',constantValue.alpha'];
stateInfo = [reshape(constantValue.initialState',1,numel(constantValue.initialState)),reshape(constantValue.goalState',1,numel(constantValue.goalState))];
funPsoTP = [cellNo,cellInfo,stateInfo];
end