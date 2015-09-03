function [D,A,B] = dist_az ( X1, X2 )
%DIST_AZ	Angular distance and direction between two locations.
%	[D,A,B] = DIST_AZ ( X1, X2 ) calculates the angular distance between
%	the locations X1 and X2 in degrees. X1 and X2 are given as 2-column
%	matrices of geographic [latitude,longitude] pairs. Either X1 or X2
%	may be a single location, in which case the output is calculated
%	between this point and each of the other locations.
%
%	D is a column vector of distances between X1 and X2.
%	A is a column vector of azimuths from X1 to X2.
%	B is a column vector of back azimuths from X2 to X1.
%
%	The GRS80 reference ellipsoid is used:
%	    equatorial radius = 6378137.0 meters
%	    polar radius      = 6356752.0 meters
%
%	Adapted from Matseis 1.6 by Stephen Arrowsmith
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check arguments.
%
if nargin < 2
  return;
end
if size(X1,2) < 2 | size(X2,2) < 2
  return;
end
if size(X1,1) ~= size(X2,1) & size(X1,1) ~= 1 & size(X2,1) ~= 1
  return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert to radians.
%
lat1 = X1(:,1) * pi/180;
lat2 = X2(:,1) * pi/180;
lon_dist = ( X1(:,2) - X2(:,2) ) * pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate geocentric latitude.
% Use GRS80 reference ellipsoid.
%
% a = 6378137.0 meters (equatorial radius)
% b = 6356752.0 meters (polar radius)
% f = (b/a)^2 = 0.99330552180201
%
lat1 = atan2 ( cos(lat1), 0.99330552180201 * sin(lat1) );
lat2 = atan2 ( cos(lat2), 0.99330552180201 * sin(lat2) );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate trig.
%
s1 = sin(lat1);
s2 = sin(lat2);
c1 = cos(lat1);
c2 = cos(lat2);
sd = sin(lon_dist);
cd = cos(lon_dist);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate distance.
%
D = real ( acos ( c1.*c2 + s1.*s2.*cd ) ) * 180/pi;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate azimuth.
%
if nargout > 1
  A = atan2 ( -s2.*sd, s1.*c2 - c1.*s2.*cd ) * 180/pi;
  A = A + 360*(A<0);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate back azimuth.
%
if nargout > 2
  B = atan2 ( s1.*sd, c1.*s2 - s1.*c2.*cd ) * 180/pi;
  B = B + 360*(B<0);
end
