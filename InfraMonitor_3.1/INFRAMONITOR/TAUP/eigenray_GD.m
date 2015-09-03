function [ray,eigenrays] = eigenray_GD(evloc,stloc,miss,metfile,dz,h,fullray)
% eigenray_GD - Computes eigenrays between given event and receiver locations,
% and a specified miss distance using the Tau-P method for long distances
% using one atmospheric specification.
%
% Where:
% - evloc is the event location
% - stloc is the receiver location
% - miss is the allowed miss distance (km)
% - metfile contains the meteorological profile
% - dz is the vertical sampling in metfile
% - h is the ground elevation above sea level (km)
% - fullray: Flag to indicate whether or not to compute the full ray
%   coordinates (0) or just the bounce-points (1)
%
% Example:
% [ray,eigenrays] = eigenray([35 -120],[35 -122],10,'METDATA/RealModel.met',2000)
%
% Stephen Arrowsmith (arrows@lanl.gov)
% OM I changed this so the distance can be measured properly using reckon;
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

% Computing (x,y) coordinates (in km) of station relative to event:
 x = deg2km(distance([evloc(1) stloc(2)],[evloc(1) evloc(2)]));
%x = m_idist(stloc(2),stloc(1),evloc(2),stloc(1))/1000;
if (stloc(2) < evloc(2))
    x = -x;
end
 y = deg2km(distance(stloc,[evloc(1) stloc(2)]));
%y = m_idist(stloc(2),stloc(1),stloc(2),evloc(1))/1000;
if (stloc(1) < evloc(1))
    y = -y;
end

% Computing range and azimuth of station relative to the event:
% range = deg2km(distance(evloc,stloc));
[range,phi,throwAway] = m_idist(evloc(2),evloc(1),stloc(2),stloc(1));
% remember the distances is in meters for the m_idist
range = range/1000;
%phi = azimuth(evloc,stloc);

% Computing ray-paths with starting azimuths within 5 deg of known azimuth:
phis = (round(phi-5):1:round(phi+5));
i = find(phis < 0); phis(i) = phis(i)+360;
i = find(phis >= 360); phis(i) = phis(i)-360;
thet=1:2:50;
ray = runtaup_GD(metfile,phis,thet,range+miss,dz,h,0,fullray);
% Finding eigenrays (rays within allowed miss distance):
% x and y are in km
k = 0; eigenrays = [];
k1=1;
num_NR=1;
for i = 1:numel(ray)
    l_x=length(ray(i).x);
    ray(i).latO(1:l_x)=nan;
    ray(i).lonO(1:l_x)=nan;
    for j = 1:l_x
        if(isreal(ray(i).y(j))==1 && isreal(ray(i).x(j))==1 )
            xd=sqrt(ray(i).x(j)^2+ray(i).y(j)^2);
            try
                ang_d=rad2deg(atan2(ray(i).y(j),ray(i).x(j)));
            catch TRE
                ang_d=nan;
                disp('problem');
                disp(TRE);
            end
            %this is the section that makes the code to see longer
            %distances. I use reckon instead of a pitagorean calculation,
            %this also make the calculation o fthe distance latitude
            %dependent.
            if(abs(xd-range)<5*miss)
            [latout longout]=reckon(evloc(1),evloc(2),km2deg(xd),ray(i).phi-ang_d);
            else
               latout=nan;
               longout=nan;
            end
%             disp(longout);
%             pause
            %[latout longout]=reckon(evloc(1),evloc(2),xd,ray(i).phi);
            %[latout longout]=reckon(evloc(1),evloc(2),20,ray(i).phi);
        else
          num_NR=num_NR+1;
          latout=nan;
          longout=nan;
        end
        k1=k1+1;
        ray(i).latO(j)=latout;
        ray(i).lonO(j)=longout;
        %ray(i).ANG(j)=ang_d;
        % here maybe it does not need to use distance again as they are
        % suppose to be very close to each aother and
        d_SE=deg2km(sqrt((stloc(1)-latout)^2+(stloc(2)-longout)^2));
        if ((d_SE) <= miss)
             k = k + 1;
             eigenrays(k) = i;
             break;  % this break looking
        end
    end
end
disp(num_NR)
