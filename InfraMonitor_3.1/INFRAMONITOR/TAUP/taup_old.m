function taup_old(metfile,dz,h,phi,theta,max_range,plotflag,I,reverse,fullray,turningRays)
% taup - Computes a single raypath using the Tau-P method
% DOES NOT USE NUMERICAL METHODS OF DROB ET AL. (2010)
%
% Usage:
% taup_old(metfile,dz,phi,theta,max_range,plotflag,I)
%
% Inputs:
% - metfile: An ASCII file containing a vertical profile of winds and
%   temperatures
%   [format: elevation (km), temperature (K), zonal wind (m/s), meridional
%   wind (m/s)]
% - dz: Vertical sampling distance for meteorological profiles (m)
% - h: Ground elevation (km)
% - phi: Launch azimuth (degrees)
% - theta: Launch angle from horizontal (degrees)
% - max_range: Maximum straight line distance from source (km)
% - plotflag: 1 for plotting meteorological data, 0 otherwise
% - I: Ray number (set I=1 if processing a single ray)
% - reverse: Forward propagation from source (=0) or reverse propagation
%   from receiver (=1)
%
% Example (Creates a global variable called 'ray'):
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

global ray

met = load(metfile,'-ascii');

% Restructuring met for known ground elevation:
[m1,m2] = min(abs(met(:,1)-h));
met = met(m2:size(met,1),:);
met(:,1) = (0:(dz/1000):size(met,1)*(dz/1000)-(dz/1000));

ray(I).phi = phi; ray(I).theta = theta;

theta = 90-theta;     % Launch angle (deg)

% Obtaining isotropic sound speed:

% ********************
% Comment first line and uncomment following three lines for adding
% noise:
c = sqrt(402.8*met(:,2));
% c = sqrt(402.8*(met(:,2) + 10*randn(numel(met(:,2)),1)));    % Adding noise
% met(:,3) = met(:,3)+20*randn(numel(met(:,3)),1);               % Adding noise
% met(:,4) = met(:,4)+20*randn(numel(met(:,4)),1);               % Adding noise
% ********************

s = 1./c;

% Computing the wind components relative to the direction of propagation:
[u,v] = AlongPathWind(phi,met(:,3),met(:,4));

%---
% u = zeros(numel(u),1);
% v = -20*ones(numel(v),1);
%---

if (reverse == 1)
    u = -u; v = -v;
end

% %--------------------------------
% % Creating synthetic wind profiles:
% u = linspace(0,200,numel(u));
% %u = zeros(1,numel(u));
% v = linspace(0,50,numel(v));
% %--------------------------------

v_eff = c + u;                     % Effective sound speed

if (plotflag == 1)
    global MET
    MET.c = c; MET.v_eff = v_eff; MET.u = u; MET.v = v; MET.z = met(:,1);
    %plotmet(c,v_eff,u,v,met(:,1))
end

% Computing the ray parameter at the source:
p = (sind(theta)/c(1))/(1 + (u(1)*sind(theta))/c(1));

% Computing the ray path to the turning point:
T = 0; ray(I).x(1) = 0; ray(I).y(1) = 0; ray(I).z(1) = 0;

i = 1;
while 1

    i = i + 1;

    tau(i) = (1 - p*u(i)) * (s(i)^2 - p^2/(1 - p*u(i))^2)^(0.5) * dz;

    % Exiting loop at ray turning point:
    if (real(tau(i)) < 0.0001)
        exit_type = 0;
        break
    end

    T = T + s(i)^2 * (s(i)^2 - (p^2/(1 - p*u(i))^2))^(-0.5) * dz;
    ray(I).x(i) = ray(I).x(i-1) + (p/(1 - p*u(i)) + s(i)^2*u(i)) * (s(i)^2 - p^2/(1 - p*u(i))^2)^(-0.5) * dz;
    ray(I).y(i) = ray(I).y(i-1) + (s(i)^2*v(i)) * (s(i)^2 - p^2/(1 - p*u(i))^2)^(-0.5) * dz;
    ray(I).z(i) = ray(I).z(i-1) + dz/1000;

    % Exiting loop if ray reaches maximum altitude:
    if (abs(ray(I).z(i) - max(met(:,1))) < dz/1000)
        exit_type = 1;
        break
    end

end

if (turningRays == 1)
    if (exit_type == 1)
        return
    end
end

% Added 01/21/09 to correct for rays that do not get off the ground:
if (i==2)
    return
end

% Computing the downward path (reflection of upward path):
if (exit_type == 0)
    dX = diff(ray(I).x);
    dX = dX(numel(dX):-1:1);
    for i = 1:numel(dX)
        if (i == 1)
            X2(i) = ray(I).x(numel(ray(I).x)) + dX(i);
        else
            X2(i) = X2(i-1) + dX(i);
        end
    end
    ray(I).x = cat(2,ray(I).x,X2)./1000;

    dY = diff(ray(I).y);
    dY = dY(numel(dY):-1:1);
    for i = 1:numel(dY)
        if (i == 1)
            Y2(i) = ray(I).y(numel(ray(I).y)) + dY(i);
        else
            Y2(i) = Y2(i-1) + dY(i);
        end
    end
    ray(I).y = cat(2,ray(I).y,Y2)./1000;

    Z2 = ray(I).z(numel(ray(I).z):-1:1);
    ray(I).z = cat(2,ray(I).z,Z2(2:numel(Z2)));

    T = T*2;
else
    ray(I).x = ray(I).x./1000;
    ray(I).y = ray(I).y./1000;
end

% Computing group velocity:
ray(I).v_g = sqrt(ray(I).x(numel(ray(I).x))^2 + ray(I).y(numel(ray(I).y))^2)/T;

if (fullray == 1)
    if (numel(find(ray(I).z==0)) > 1)
        ray(I).x = ray(I).x(numel(ray(I).x)); ray(I).y = ray(I).y(numel(ray(I).y));

        % *** For single bounce only, comment out this section ***
        njumps = floor(max_range/sqrt(ray(I).x^2 + ray(I).y^2));
        if (njumps > 1)
            for njumps = 2:njumps
                ray(I).x = cat(1,ray(I).x,ray(I).x(1)*njumps);
                ray(I).y = cat(1,ray(I).y,ray(I).y(1)*njumps);
            end
        end
        % ********************************************************

        ray(I).zmax = max(ray(I).z); ray(I).z = [];
    else
        ray(I).x = []; ray(I).y = []; ray(I).z = []; ray(I).v_g = [];
    end
    return
end

% Adding additional bounces as required:
if (ray(I).z(numel(ray(I).z)) == 0)
    range = sqrt(ray(I).x(numel(ray(I).x))^2 + ray(I).y(numel(ray(I).y))^2);
    nbounces = ceil(max_range/range);
    if (nbounces ~= 1)
        for i = 1:nbounces-1
            if (i == 1)
                X2 = ray(I).x(2:numel(ray(I).x))+ray(I).x(numel(ray(I).x));
                Y2 = ray(I).y(2:numel(ray(I).y))+ray(I).y(numel(ray(I).y));
                Z2 = ray(I).z(2:numel(ray(I).z));
            else
                X2 = cat(2,X2,ray(I).x(2:numel(ray(I).x))+X2(numel(X2)));
                Y2 = cat(2,Y2,ray(I).y(2:numel(ray(I).y))+Y2(numel(Y2)));
                Z2 = cat(2,Z2,ray(I).z(2:numel(ray(I).z)));
            end
        end
        ray(I).x = cat(2,ray(I).x,X2);
        ray(I).y = cat(2,ray(I).y,Y2);
        ray(I).z = cat(2,ray(I).z,Z2);
    end
end
