
%clear all;
% Copyright (c) 2012, Los Alamos National Security, LLC
% All rights reserved.
% 
% Copyright 2012. Los Alamos National Security, LLC. This software was produced under U.S.
% Government contract DE-AC52-06NA25396 for Los Alamos National Laboratory (LANL), which is
% operated by Los Alamos National Security, LLC for the U.S. Department of Energy. The U.S.
% Government has rights to use, reproduce, and distribute this software.  NEITHER THE
% GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC MAKES ANY WARRANTY, EXPRESS OR IMPLIED,
% OR ASSUMES ANY LIABILITY FOR THE USE OF THIS SOFTWARE.  If software is modified to produce
% derivative works, such modified software should be clearly marked, so as not to confuse it
% with the version available from LANL.
% 
% Additionally, redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% ·         Redistributions of source code must retain the above copyright notice, this list
% 		  of conditions and the following disclaimer.
% ·         Redistributions in binary form must reproduce the above copyright notice, this
% 	      list of conditions and the following disclaimer in the documentation and/or
% 	      other materials provided with the distribution.
% ·         Neither the name of Los Alamos National Security, LLC, Los Alamos National
% 	      Laboratory, LANL, the U.S. Government, nor the names of its contributors may be
% 	      used to endorse or promote products derived from this software without specific
% 	      prior written permission.
% THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
% EVENT SHALL LOS ALAMOS NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
% INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
% LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
% OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
% WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
close all;
%figure;


turningRays = 1;
reverse = 0;
phi=30;
theta=15;
resolution=200;
h_ele=0;
fullray=0;  % 1 BP, 0 all points


global ray;
clear global ray;
global ray;
plotflag_V=0;
max_range=2000;
lat_ini=3;
lon_ini=274;
plot(lon_ini,lat_ini,'sk','MarkerFaceColor','k');
hold on;
%%
k1=1;
taupRD('G2SGCS_2003061000.bin',lat_ini,lon_ini,resolution,h_ele,phi,theta,max_range,plotflag_V,1,reverse,fullray,turningRays);

if(length(ray)>0 )
        plot(ray.x,ray.y,'o')
end
%%
%taup('G2SGCS_2003061000.bin',0,0,resolution,h_ele,phi,theta,max_range,plotflag_V,1,reverse,fullray,turningRays);
extract_MET_safe ('G2SGCS_2003061000.bin',lat_ini,lon_ini,resolution);
k1=k1+1;
taup(['G2SGCS_2003061000_' num2str(lat_ini) '_' num2str(lon_ini) '.met'],resolution,h_ele,phi,theta,max_range,plotflag_V,k1,reverse,fullray,turningRays);
if(isempty(ray(k1).x)==0)
RotateXY(k1);
plot(km2deg(ray(k1).x)+lon_ini,km2deg(ray(k1).y)+lat_ini,'r+');
axis equal;
else
   disp('No return');
end

legend('Source', 'Seudo RD','RI')

