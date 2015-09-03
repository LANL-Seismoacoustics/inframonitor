function detections = makebul(time,spout,epout,az,slow,c,f,dtype)
% makebul: Assembles detection parameters
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

j = 0;
spout_test = exist('spout');
if (spout_test ~= 0)
    for i = 1:numel(spout)
        if (spout == 0)
            detections = -999;
            break
        elseif (spout == -1)
            detections = -999;
            break
        end

        aztmp = az((spout(i)+1):(epout(i)-1));                % Taking azimuths during detection
        slowtmp = slow((spout(i)+1):(epout(i)-1));

        % Added 07/07/10 (Implements Grubbs' test for outliers):
        try
            [aztmp,idx] = deleteoutliers(aztmp);
        catch
            continue
        end
        slowtmp = setdiff((1:numel(slowtmp)),slowtmp(idx));

        j = j + 1;

        detections(j,1) = time(spout(i));                                   % Start time of detection
        detections(j,2) = time(epout(i));                                   % End time of detection
        [a,b] = max(f((spout(i)):(epout(i))));
        a = az(spout(i):epout(i));
        detections(j,3) = a(b);                                             % Azimuth at maximum F-stat
        detections(j,4) = max(c((spout(i)):(epout(i))));                    % Maximum correlation
        detections(j,5) = dtype;                                            % Detector type
        detections(j,6) = std(aztmp);                                       % Azimuth standard deviation
        s = slow(spout(i):epout(i));
        detections(j,7) = (1./s(b))*111.1949;                               % Slowness at maximum F-stat
        detections(j,8) = std((1./slowtmp)*111.1949);                          % Slowness standard deviation
        clear aztmp azstt azels
    end
else
    detections = -999;
end

if (exist('detections','var') == 0)
    detections = -999;
end
