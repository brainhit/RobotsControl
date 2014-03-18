%% function: draw cell

function hCell = drawCell(xr,yr,tr,numMag,len)
% DRAWMAGNET @brief: draws a puriformis cell at (xr,yr) orientation tr
% with numMag particles and of length len
%
% @author: Aaron Becker
% @since: Jan 4, 2013
hCell.htetra = patch(0,0,'g');
hCell.hNucleus = patch(0,0,[.4,.5,.4]);
for j = 1:max(1,numMag)
    hCell.hMag(j) = patch(0,0,'k');
end
hCell.numMag = numMag;
hCell.len = len;
hCell = updateCell(hCell,xr,yr,tr);
end