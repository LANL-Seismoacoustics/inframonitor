function beam = tdelay ( data, samprate, x, y, azimuth, slowness, nthroot )
%TDELAY Time-delayer
%	BEAM = TDELAY(DATA,SAMPRATE,X,Y,AZIMUTH,SLOWNESS,NTHROOT) returns
%	the time-delay-and-sum beam of DATA to the direction defined by
%	AZIMUTH and SLOWNESS.
%	TDBEAM will beam to one direction with each call.
%
%	DATA is an N-column matrix of data vectors.
%	SAMPRATE is the scalor sample rate of all data vectors.
%
%       X and Y are the offsets in degs of the elements of the array
%       relative to a reference point
%
%	AZIMUTH is the scalor beam azimuth clockwise from north (in degrees).
%	SLOWNESS is the scalor beam slowness (in S/deg).
%
%	NTHROOT is an optional parameter that specifies the root factor
%	for Nth root robust stacking (Muirhead & Datt, 1976, GJRAS #47).
%
%	BEAM is a column vector of the data beamed to AZIMUTH and SLOWNESS.
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

%	Adapted from Matseis 1.6 by Stephen Arrowsmith


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check arguments.
%
if nargin < 6
  error ( 'Not enough arguments.' );
end
if size(data,2) < 2
  error ( 'DATA must be an N-column matrix.' );
end
samprate = samprate(1);         %problems if different sample rates!!!!
if size(x,1) ~= size(data,2) & size(y,1) ~= size(data,2)
  error ( 'rows of LOC must match columns of DATA.' );
end
azimuth = azimuth(1);
slowness = slowness(1);
if nargin < 7
  nthroot = 0;
else
  nthroot = nthroot(1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup beam matrix.
%
data = data';
[M,N] = size(data);
beam = zeros(M,N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate time delays (integer number of samples to shift).
%
dist = sqrt(x.^2 + y.^2);
az = 90 - (180/pi)*(atan2(y,x));      %includes conversion to seismo az
%az = 90 - rad2deg(atan2(y,x));
delay = round ( ( dist*slowness ) .* cos ( (az-azimuth)*pi/180 ) .* samprate );
delay1 = max(delay,zeros(M,1));
delay2 = min(delay,zeros(M,1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate beam index.
%
b1 = delay1 + 1;
bN = delay2 + N;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate data vector index.
%
d1 = - delay2 + 1;
dN = - delay1 + N;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check Nth root.
%
if nthroot
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Shift data.
  %
  for m = 1:M
    wdata = data(m,d1(m):dN(m));
    wdata = ( sign(wdata) ) .* ( abs(wdata) .^ (1/nthroot) );
    beam(m,b1(m):bN(m)) = wdata;
  end
else
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Shift data.
  %
  for m = 1:M
    beam(m,b1(m):bN(m)) = data(m,d1(m):dN(m));
  end
end

beam = beam';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check Nth root.
%
if nthroot
  beam = ( sign(beam) ) .* ( abs(beam) .^ nthroot );
end
