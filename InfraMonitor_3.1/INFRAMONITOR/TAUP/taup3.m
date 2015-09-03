function taup3(metfile,dz,h,phi,theta,max_range,plotflag,I,reverse,fullray,turningRays,varargin)
% taup - Computes a single raypath using the Tau-P method
% USES NUMERICAL METHODS OF DROB ET AL. (2010)
%
% Usage:
% taup(metfile,dz,phi,theta,max_range,plotflag,I)
%
% Inputs:
% - metfile: An ASCII file containing a vertical profile of winds and
%   temperatures
%   [format: elevation (km), temperature (K), zonal wind (m/s), meridional
%   wind (m/s)]
%   *** or, a matfile generated via G2S system and readenv.m (only one azimuth) ***
% - dz: Vertical sampling distance for meteorological profiles (m)
% - h: Ground elevation (km)
% - phi: Launch azimuth (degrees)
% - theta: Launch angle from horizontal (degrees)
% - max_range: Maximum straight line distance from source (km)
% - plotflag: 1 for plotting meteorological data, 0 otherwise
% - I: Ray number (set I=1 if processing a single ray)
% - reverse: Forward propagation from source (=0) or reverse propagation
%   from receiver (=1)
% - fullray: Flag to indicate whether or not to compute the full ray
%   coordinates (0) or just the bounce-points (1)
%
% Example (Creates a global variable called 'ray'):
% taup('METDATA/RealModel.met',2000,90,10,2000,0,1,0)
%
% Stephen Arrowsmith (arrows@lanl.gov)
%
%OM, Oct. 2012
%taup3 adds one variable to the arguments of taup, if the variable exist and
%varargin(1)=1 the code calculates just one bounce point, this is an
%efficient way for the calculation of the range dependence.
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

if (numel(regexp(metfile,'met')) > 0)
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

% Computing the ray parameter at the source:
p = (sind(theta)/c(1))/(1 + (u(1)*sind(theta))/c(1));

% Finding maximum altitude of ray:
uu = spline(z,u);
vv = spline(z,v);
ss = spline(z,s);
f = @(z)ppval(ss,z).^2 - p^2./(1 - p*ppval(uu,z)).^2;
zmax = findroot(z,p,s,u,f);
if (zmax == -999)
    if (turningRays == 1)
        disp('NO turn')
        return
    else
        zmax = max(z);
        exit_type = 1;
    end
else
    exit_type = 0;
end

% Calculating ray parameters at bounce point:
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

if (fullray == 1)
    ray(I).x = r_max; ray(I).y = q_max;
    ray(I).v_g = sqrt(r_max^2 + q_max^2)/t_max;
    ray(I).zmax = zmax;

    range = sqrt((ray(I).x)^2 + (ray(I).y)^2);
    if nargin > 11
       nbounces=0;
    else
       nbounces = floor(max_range/range);
    end

    for i = 1:nbounces
        ray(I).x = cat(2,ray(I).x,ray(I).x(numel(ray(I).x))+r_max);
        ray(I).y = cat(2,ray(I).y,ray(I).y(numel(ray(I).y))+q_max);
    end
    return;

end






%% -------------------------------------
% Calculating full upgoing ray-path:
ray(I).z = linspace(0,zmax,100);        % Evaluating the function at 100 points
for i = 1:numel(ray(I).z)
    try
        ray(I).x(i) = quadgk(R,ray(I).z(i-1),ray(I).z(i)) + ray(I).x(i-1);
        ray(I).y(i) = quadgk(Q,ray(I).z(i-1),ray(I).z(i)) + ray(I).y(i-1);
    catch
        ray(I).x(i) = quadgk(R,0,ray(I).z(i));
        ray(I).y(i) = quadgk(Q,0,ray(I).z(i));
    end
end
%% -------------------------------------

if (exit_type == 0)
    % Computing downgoing ray-path:
    dX = diff(ray(I).x);
    dX = dX(numel(dX):-1:1);
    for i = 1:numel(dX)
        if (i == 1)
            X2(i) = ray(I).x(numel(ray(I).x)) + dX(i);
        else
            X2(i) = X2(i-1) + dX(i);
        end
    end
    ray(I).x = cat(2,ray(I).x,X2);

    dY = diff(ray(I).y);
    dY = dY(numel(dY):-1:1);
    for i = 1:numel(dY)
        if (i == 1)
            Y2(i) = ray(I).y(numel(ray(I).y)) + dY(i);
        else
            Y2(i) = Y2(i-1) + dY(i);
        end
    end
    ray(I).y = cat(2,ray(I).y,Y2);

    ray(I).z = cat(2,ray(I).z,ray(I).z(numel(ray(I).z)-1:-1:1));
end
% Adding additional bounces as required:

if(nargin == 11)
if (ray(I).z(numel(ray(I).z)) == 0 )
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
end
% Computing group velocity:
ray(I).v_g = sqrt(r_max^2 + q_max^2)/t_max;
