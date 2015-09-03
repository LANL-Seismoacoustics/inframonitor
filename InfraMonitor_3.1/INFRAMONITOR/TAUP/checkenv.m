%  Name: readramenv.m
%
%  Description: A script to read an G2S enviromental record
%     output by the G2S extractgcp command line appliance
%     into memory.
%
%  Author: Douglas P. Drob
%          NRL Code 7643
%
%  Date: Updated - January 19, 2007
%
% ======================================================================
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

[file_name, mach_path]=uigetfile('*.env', 'File Selector');
filename=strcat(mach_path,file_name);

fid = fopen(filename,'r','ieee-le');

nr = fread(fid,1,'integer*4');      %[pnt] # of range point in profiles
nz = fread(fid,1,'integer*4');      %[pnt] # altitude points in profile

lat = fread(fid,nr,'real*8');       %[deg] latitude profile grid
lon = fread(fid,nr,'real*8');       %[deg] longitude profile grid
baz = fread(fid,nr,'real*8');       %[deg] longitude profile grid
rng = fread(fid,nr,'real*8');       %[m] range from source of grid
alt = fread(fid,nr,'real*8');       %[m] height of air/ground/sea
ai = fread(fid,nz,'real*8');        %[km] Altitudes for 2d grid

ti = fread(fid,[nz,nr],'real*8');   %[k] Temperature
di = fread(fid,[nz,nr],'real*8');   %[g/cm3] density
pr = fread(fid,[nz,nr],'real*8');   %[hPa] pressure
ui = fread(fid,[nz,nr],'real*8');   %[m/s] Along track wind
vi = fread(fid,[nz,nr],'real*8');   %[m/s] Cross track wind
wi = fread(fid,[nz,nr],'real*8');   %[m/s] vertical wind

tj = fread(fid,[nz,nr],'real*8');   %[k] Temperature (parallel path)
dj = fread(fid,[nz,nr],'real*8');   %[g/cm3] density (parallel path)
pj = fread(fid,[nz,nr],'real*8');   %[hPa] pressure (parallel path)
uj = fread(fid,[nz,nr],'real*8');   %[m/s] Along track wind (parallel path)
vj = fread(fid,[nz,nr],'real*8');   %[m/s] Cross track wind (parallel path)
wj = fread(fid,[nz,nr],'real*8');   %[m/s] vertical wind (parallel path)

% ===================================================
% calculate some quantities from the specifications
% ===================================================

si = sqrt( 1.4 * 0.1 * pr ./ di);
sj = sqrt( 1.4 * 0.1 * pj ./ dj);

hgt = ai(:);

figure;
plot(ui(:,1),hgt,'r',ui(:,nr),hgt,'g')
xlabel('Velocity (m/s)');
ylabel('Altitude (km)');
title('Along Track Wind');
axis([-70 70 0 130]);

figure;
plot(vi(:,1),hgt,vi(:,nr),hgt)
xlabel('Velocity (m/s)');
ylabel('Altitude (km)');
title('Cross Track Wind');
axis([-50 50 0 130]);

figure;
plot(si(:,1),hgt,':',si(:,nr),hgt,':')
hold on
plot(si(:,1)+ui(:,1),hgt,si(:,nr)+ui(:,nr),hgt)
axis([250 400 0 130]);
hold off;
xlabel('Velocity (m/s)');
ylabel('Altitude (km)');
title('Sound Velocity');

fclose(fid);
