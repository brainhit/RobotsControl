function hCell = updateCell(hCell,xr,yr,tr)
% UPDATECELL @brief: draws a puriformis cell at (xr,yr) orientation tr
% with numMag particles and of length len
% 
% @author: Aaron Becker
% @since: Jan 4, 2013

a = hCell.len; %major axis
b = 0.6*a; %minor axis
%draw green oval (cell body)
o=a/8;
t = linspace(0,2*pi,50);
xt = xr + reshape([a*cos(t)*cos(tr)-b*sin(t)*sin(tr);(a+o)*cos(t)*cos(tr)-(b+o)*sin(t)*sin(tr);a*cos(t)*cos(tr)-b*sin(t)*sin(tr)],numel(t)*3,1);
yt = yr + reshape([a*cos(t)*sin(tr)+b*sin(t)*cos(tr);(a+o)*cos(t)*sin(tr)+(b+o)*sin(t)*cos(tr);a*cos(t)*sin(tr)+b*sin(t)*cos(tr)],numel(t)*3,1);
%plot(xt,yt,'k');
%hCell.htetra = patch(xt,yt,'g');
set(hCell.htetra,'XData',xt,'YData',yt);
%draw the nucleus
nr = a/4;
xn = xr+nr*cos(t)-nr*cos(tr);
yn = yr+nr*sin(t)-nr*sin(tr);
%hCell.hNucleus = patch(xn,yn,[.4,.5,.4]);
set(hCell.hNucleus,'XData',xn,'YData',yn);
%draw magnetic particles.
nrn = a/20;
numMag = hCell.numMag;
if numMag<1
    nrn = nrn*numMag;
    numMag = 1;
end
for j = 1:numMag
    sc = pi/8;
    as = a*.8;
    bs = b*.8;
    trn = -(numMag+1)*sc/2+(j*sc); %angle of position of center
    xn = xr+nrn*cos(t)+as*cos(trn)*cos(tr)-bs*sin(trn)*sin(tr);
    yn = yr+nrn*sin(t)+as*cos(trn)*sin(tr)+bs*sin(trn)*cos(tr);
   % xn = xr+nrn*cos(t)+nr*cos(trn);
   % yn = xr+nrn*sin(t)+nr*sin(trn);    
   % hCell.hMag(j) = patch(xn,yn,'k');
    set(hCell.hMag(j),'XData',xn,'YData',yn);
end
end