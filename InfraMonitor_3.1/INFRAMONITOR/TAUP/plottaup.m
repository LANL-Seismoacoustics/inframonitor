function plottaup(ray,evloc,stloc,miss)
% plottaup - Plots all/selected raypaths in 3D and source location (evloc).
% Optionally also plots the receiver location and search ellipse (for
% eigenray solutions)
%
% Examples:
% plottaup(ray,[35 -120],[],[])         - Plots all rays in the ray structure array
% plottaup(ray(10),[35 -120],[],[])     - Plots the 10th ray
% plottaup(ray([1 10]),[35 -120],[],[]) - Plots the 1st and 10th rays
% plottaup(ray([25 115 204]),[35 -120],[35 -122],100) - Also plots receiver
%                                                       location and search ellipse
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

figure; hold on; view([45 45])
for i = 1:numel(ray)

    try
        % Computing closest bounce-point to avoid plotting unnecessary skips:
        ray(i).lon = evloc(2) + km2degsc(ray(i).x,evloc(1));
        ray(i).lat = evloc(1) + km2deg(ray(i).y);
        j = find(ray(i).z == 0);
        [u,v] = min(deg2km(distance(stloc(1),stloc(2),ray(i).lat(j),ray(i).lon(j))));
        plot3(evloc(2) + km2degsc(ray(i).x(1:j(v)),evloc(1)),evloc(1) + km2deg(ray(i).y(1:j(v))),ray(i).z(1:j(v)))
    catch
        plot3(evloc(2) + km2degsc(ray(i).x,evloc(1)),evloc(1) + km2deg(ray(i).y),ray(i).z)
    end
end

grid on
xlabel('lon')
ylabel('lat')
zlabel('z (km)')
plot3(evloc(2),evloc(1),0,'kp','MarkerFaceColor','k','MarkerSize',10)

if (isempty(stloc) ~= 1)
    plot3(stloc(2),stloc(1),0,'kp','MarkerFaceColor','r','MarkerSize',10)
    [latc,lonc] = scircle1(stloc(1),stloc(2),km2deg(miss));
    plot3(lonc,latc,zeros(numel(latc),1),'r:','LineWidth',1.5)
end
