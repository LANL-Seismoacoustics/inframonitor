function taupRD (binfile,lat_s,lon_s,dz_G,h,phi,theta,max_range,plotflag,I,reverse,fullray,turningRays )
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

global rayT
clear global rayT;
global rayT;

global ray

[ a b c]=fileparts(binfile);
if(strcmp(c,'.bin')==0)
    disp('BIN file should be provided to create seudo-range dependent CRH');
    return
end

if(length(dir(binfile))==0)
    disp([binfile ' file was not found in current directory']);
    return
end

FID = fopen(binfile);
if(FID==-1)
   disp('error opennign bin file');
   return
else
   disp('file open OK');
end


g2sclient('load',binfile);
%disp('file load OK');
nalt = 601;
dz = dz_G/1000;
%g2sclient('setgrid',nalt,dz);
trunc = 121;
%disp('setgrid OK');
tot_range=0;
k1=1;
lat=lat_s;
lon=lon_s;
phi_D=phi;

ray(I).x=[];
ray(I).y=[];
ray(I).v_g=[];
ray(I).phi=[];

ray(I).xFR=[];
ray(I).yFR=[];
ray(I).zFR=[];


while tot_range < max_range
    global rayT;
    [zg,ug,vg,tg,dg,pg] = g2sclient('extract',lat,lon,trunc);
    %disp(['extract' num2str(k1) ' OK']);
    A=[zg;tg;ug;vg;dg;pg];
    fileID = fopen('tmpMATLAB.met','w');
    fprintf(fileID,'%f %f %f %f %f %f \n',A);
    fclose(fileID);
    taup_onebounce('tmpMATLAB.met',dz_G,h,phi_D,theta,max_range,plotflag,1 ,reverse,fullray,turningRays);
    if(isempty(rayT(1).x)==1 || ~isreal(rayT(1).x) || ~isreal(rayT(1).y))
        disp('No turn')
       break;
    end
    AzDev_RAW(1);
    tot_range=tot_range+sqrt(rayT(1).x^2+rayT(1).y^2);
    rr=km2deg(sqrt(rayT(1).x^2+rayT(1).y^2));
    lon_ST=lon;
    lat_ST=lat;
    [lat lon]=reckon(lat,lon,rr,rayT(1).az_dev);
    rayT(1).lat=lat;
    rayT(1).lon=lon;
    phi_D=rayT(1).az_dev;
    Rotate_FR(1);
    ray(I).x=[ray(I).x lon];
    ray(I).y=[ray(I).y lat];
    ray(I).v_g=[ray(I).v_g rayT(1).v_g];
    ray(I).phi=[ray(I).phi rayT(1).az_dev];

    ray(I).xFR=[ray(I).xFR rayT(1).xFR];
    ray(I).yFR=[ray(I).yFR rayT(1).yFR];
    ray(I).zFR=[ray(I).zFR rayT(1).zFR];


        k1=k1+1;
        clear global rayT;
    if k1>20
        return
    end

end
g2sclient('shutdown');

end

