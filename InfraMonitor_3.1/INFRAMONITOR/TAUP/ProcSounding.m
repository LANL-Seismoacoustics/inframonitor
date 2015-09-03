function ProcSounding(SoundingFile,OutputFile)
% ProcSounding - Processes atmospheric sounding data obtained from the
% University of Wyoming (http://weather.uwyo.edu/upperair/sounding.html)
%
% e.g.,
% ProcSounding('SLC_022108_12Z.orig','Wells.met')
%
% Note: Requires ProcSounding.py in the system path
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

[status, result] = system(['ProcSounding.py ' SoundingFile]);
sounding = str2num(result);

fid = fopen(OutputFile,'w');

for i = 1:size(sounding,1)

    % Correcting wind direction:
    if (sounding(i,3) >= 180)
        sounding(i,3) = sounding(i,3) - 180;
    else
        sounding(i,3) = sounding(i,3) + 180;
    end

    % Calculating zonal and meridional wind components:
    if (sounding(i,3) <= 90)
        z = sounding(i,4)*sind(sounding(i,3));
        m = sounding(i,4)*cosd(sounding(i,3));
    elseif (sounding(i,3) > 90 && sounding(i,3) <= 180)
        z = sounding(i,4)*sind(180 - sounding(i,3));
        m = -sounding(i,4)*cosd(180 - sounding(i,3));
    elseif (sounding(i,3) > 180 && sounding(i,3) <= 270)
        z = -sounding(i,4)*sind(sounding(i,3) - 180);
        m = -sounding(i,4)*cosd(sounding(i,3) - 180);
    else
        z = -sounding(i,4)*sind(360 - sounding(i,3));
        m = sounding(i,4)*cosd(360 - sounding(i,3));
    end

    fprintf(fid,'%9.7e   %9.7e   %9.7e   %9.7e\n',sounding(i,1),sounding(i,2),...
        z,m);
end

fclose(fid);
