function SyntheticTest(loc,o_time_s,v_g,sensors_s,sensor_file,det_files_s,append)
% SyntheticTest: Generates synthetic detection bulletins for testing the
% InfraMonitor association routine
%
% e.g., SyntheticTest([41 -111],'2007-08-01 00:00:00',[0.3 0.3 0.3
% 0.3],'BGU1,EPU1,NOQ3,FSU1','../INPUTS/sta.inp','BGUt.det,EPUt.det,NOQt.de
% t,FSUt.det',0)
%
% v1.1: append flag = 0 for new detection file, 1 to append to an existing
% detection file
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

lat = loc(1);
lon = loc(2);
o_time = datenum(o_time_s,'yyyy-mm-dd HH:MM:SS');
sensors = list2cell(sensors_s);
det_files = list2cell(det_files_s);

% Creating bulletin files:

fid = fopen(sensor_file,'r');
sensor_locs = textscan(fid,'%s %f %f');
fclose(fid);

for i = 1:numel(sensors)

    if (append == 0)
        fid = fopen(det_files{i},'w');
    else
        fid = fopen(det_files{i},'a');
    end

    % Finding sensor location:
    for j = 1:numel(sensor_locs{1})
        strtest1 = sensor_locs{1}{j};
        strtest2 = sensors{i};
        if (strcmp(strtest1,strtest2) == 1)

            slat = sensor_locs{2}(j);
            slon = sensor_locs{3}(j);
            break
        end
    end

    % Computing backazimuth (b) and distance (d):
    [d,a,b] = dist_az([lat lon],[slat slon]);
    d = 111.1949*(d);

    % Computing arrival time:
    t = d/v_g(i);
    a_time1 = o_time + t/86400;
    a_time2 = o_time + (t+60)/86400;

    % Writing bulletin file:
    fprintf(fid,'%s %s %5.2f %5.2f %1.0f\n',...
        datestr(a_time1,31),datestr(a_time2,31),...
        b,0.8,1);

    fclose(fid);

end
