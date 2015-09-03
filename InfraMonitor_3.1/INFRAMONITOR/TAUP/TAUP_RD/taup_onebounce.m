function taup_onebounce(metfile,dz,h,phi,theta,max_range,plotflag,I,reverse,fullrayT,turningrayTs)
% taup - Computes a single rayTpath using the Tau-P method
% USES NUMERICAL METHODS OF DROB ET AL. (2010)
%
% Usage:
% taup(metfile,dz,phi,theta,max_range,plotflag,I)
%
% Inputs:
% - metfile: An ASCII file containing a vertical profile of%   temperatures
%   [format: elevation (km), temperature (K), zonal wind (m/s), meridional
%   wind (m/s)]
%   *** or, a matfile generated via G2S system and readenv.m (only one azimuth) ***
% - dz: Vertical sampling distance for meteorological profiles (m)
% - h: Ground elevation (km)
% - phi: Launch azimuth (degrees)
% - theta: Launch angle from horizontal (degrees)
% - max_range: Maximum straight line distance from source (km)
% - plotflag: 1 for plotting meteorological data, 0 otherwise
% - I: rayT number (set I=1 if processing a single rayT)
% - reverse: Forward propagation from source (=0) or reverse propagation
%   from receiver (=1)
% - fullrayT: Flag to indicate whether or not to compute the full rayT
%   coordinates (0) or just the bounce-points (1)
%
% Example (Creates a global variable called 'rayT'):
% taup('METDATA/RealModel.met',2000,90,10,2000,0,1,0)
%
% Stephen Arrowsmith (arrows@lanl.gov)
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

if (numel(regexp(metfile,'met')) > 0)
    met = load(metfile,'-ascii');

    % Restructuring met for known ground elevation:
    [m1,m2] = min(abs(met(:,1)-h));
    met = met(m2:size(met,1),:);
    met(:,1) = (0:(dz/1000):size(met,1)*(dz/1000)-(dz/1000));

    rayT(I).phi = phi; rayT(I).theta = theta;

    theta = 90-theta;     % Launch angle (deg)

    % Obtaining isotropic sound speed:

    % ********************
    % Comment first line and uncomment following three lines for adding
    % noise:
    c = sqrt(402.8*met(:,2));
%     c = sqrt(402.8*(met(:,2) + 10*randn(numel(met(:,2)),1)));    % Adding noise
%     met(:,3) = met(:,3)+50*randn(numel(met(:,3)),1);               % Adding noise
%     met(:,4) = met(:,4)+50*randn(numel(met(:,4)),1);               % Adding noise
    % ********************

    s = 1./c;

    % Computing the wind components relative to the direction of propagation:
    [u,v] = AlongPathWind(phi,met(:,3),met(:,4));
    z = met(:,1);
    s = s';
else
    load(metfile)
    s = 1./c';
    theta = 90-theta;     % Launch angle (deg)
end

if (reverse == 1)
    u = -u; v = -v;
end

v_eff = c + u;                     % Effective sound speed

if (plotflag == 1)
    global MET
    MET.c = c; MET.v_eff = v_eff; MET.u = u; MET.v = v; MET.z = met(:,1);
    %plotmet(c,v_eff,u,v,z)
end

% Computing the rayT parameter at the source:
p = (sind(theta)/c(1))/(1 + (u(1)*sind(theta))/c(1));

% Finding maximum altitude of rayT:
uu = spline(z,u);
vv = spline(z,v);
ss = spline(z,s);
f = @(z)ppval(ss,z).^2 - p^2./(1 - p*ppval(uu,z)).^2;
zmax = findroot(z,p,s,u,f);
if (zmax == -999)
    if (turningrayTs == 1)
        rayT(I).x=[];
        return
    else
        zmax = max(z);
        exit_type = 1;
    end
else
    exit_type = 0;
end

% Calculating rayT parameters at bounce point:
R = @(z)(f(z).^-0.5).*(p./(1-ppval(uu,z).*p) + ppval(uu,z).*ppval(ss,z).^2);
T = @(z)(f(z).^-0.5).*ppval(ss,z).^2;
Q = @(z)(f(z).^-0.5).*ppval(vv,z).*ppval(ss,z).^2;

if (exit_type == 0)
    r_max = 2*quadgk(R,0,zmax);
    t_max = 2*quadgk(T,0,zmax)*1000;
    q_max = 2*quadgk(Q,0,zmax);
else
    r_max = quadgk(R,0,zmax);
    t_max = quadgk(T,0,zmax)*1000;
    q_max = quadgk(Q,0,zmax);
end

    rayT(I).x = r_max; rayT(I).y = q_max;
    rayT(I).v_g = sqrt(r_max^2 + q_max^2)/t_max;
    rayT(I).zmax = zmax;
    range = sqrt((rayT(I).x)^2 + (rayT(I).y)^2);
if (fullrayT == 1)

    return;
end

%% -------------------------------------
% Calculating full upgoing rayT-path:
rayT(I).zFR = linspace(0,zmax,100);        % Evaluating the function at 100 points
for i = 1:numel(rayT(I).zFR)
    try
        rayT(I).xFR(i) = quadgk(R,rayT(I).zFR(i-1),rayT(I).zFR(i)) + rayT(I).xFR(i-1);
        rayT(I).yFR(i) = quadgk(Q,rayT(I).zFR(i-1),rayT(I).zFR(i)) + rayT(I).yFR(i-1);
    catch
        rayT(I).xFR(i) = quadgk(R,0,rayT(I).zFR(i));
        rayT(I).yFR(i) = quadgk(Q,0,rayT(I).zFR(i));
    end
end
%% -------------------------------------

if (exit_type == 0)
    % Computing downgoing rayT-path:
    dX = diff(rayT(I).xFR);
    dX = dX(numel(dX):-1:1);
    for i = 1:numel(dX)
        if (i == 1)
            X2(i) = rayT(I).xFR(numel(rayT(I).xFR)) + dX(i);
        else
            X2(i) = X2(i-1) + dX(i);
        end
    end
    rayT(I).xFR = cat(2,rayT(I).xFR,X2);

    dY = diff(rayT(I).yFR);
    dY = dY(numel(dY):-1:1);
    for i = 1:numel(dY)
        if (i == 1)
            Y2(i) = rayT(I).yFR(numel(rayT(I).yFR)) + dY(i);
        else
            Y2(i) = Y2(i-1) + dY(i);
        end
    end
    rayT(I).yFR = cat(2,rayT(I).yFR,Y2);

    rayT(I).zFR = cat(2,rayT(I).zFR,rayT(I).zFR(numel(rayT(I).zFR)-1:-1:1));
end



% Computing group velocity:
rayT(I).v_g = sqrt(r_max^2 + q_max^2)/t_max;
