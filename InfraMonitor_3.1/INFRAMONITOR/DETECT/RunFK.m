function F_K = RunFK(InfraUser,stime,etime,t_start,AdWinNo,AdWinNum)
% RunFK - Applies F-K analysis to array data
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

C = progress('init',['InfraMonitor2: Processing Adaptive Window '...
    num2str(AdWinNo) ' of ' num2str(AdWinNum)]');
global InfraConfig

% Initializing Parameters:
start_time = stime;
NTrace = numel(InfraConfig.trace.select);
s_f = InfraConfig.trace.s_f{InfraConfig.trace.select(1)};
shift = InfraUser.twin*(1-(InfraUser.overlap/100));
no_windows = ((etime - start_time)*86400)/shift;
[X,Y] = GetXY(InfraConfig);
if (X == -999)
    F_K = -999;
    return
end

% Preallocating vectors:
slofk = zeros(1,ceil(no_windows));
fstat = zeros(1,ceil(no_windows));
avg_corr1 = zeros(1,ceil(no_windows));
time = zeros(1,ceil(no_windows));
az = zeros(1,ceil(no_windows));
pow = zeros(1,ceil(no_windows));    % ***
t = cell(1,NTrace);
s = cell(1,NTrace);

i = 0;

mask = makeMask();

while ((start_time*86400)+InfraUser.twin <= etime*86400)

    i = i + 1;

    C = progress(C,i/no_windows);

%     if (numel(find(III == i)) > 0)
%         update_waitbar(handles,i/no_windows);
%     end

    % Extracting time window:
    x_fw = cell(1,NTrace);
    for j = 1:NTrace
        k = InfraConfig.trace.select(j);

        try
            x_fw{j} = InfraConfig.trace.data{k}(floor(t_start):...
                floor(t_start+(InfraUser.twin*s_f)));
        catch
            break
        end

        if (j == 1)
            DATA = x_fw{j};
        else
            DATA = cat(2,DATA,x_fw{j});
        end
    end

    % Checking for errors in reading data:
    if (exist('DATA','var') == 0)
        detections = -999; plotparam = -999;
        close(waithandle)
        return
    end

    % Applying FK analysis to obtain backazimuth and phase velocity:
    try
        [FK,Fstat1] = fk(DATA,s_f,X,Y,InfraUser.slow,InfraConfig.db.f_band');
    catch
        break
    end
    Fstat1 = Fstat1.*mask;    % Added 06/27/12 to mask out unrealistically high phase velocities
    [maxFK1,r] = max(Fstat1);
    [maxFK1,c] = max(maxFK1);
    [az(i),slofk(i)] = cart2pol( InfraUser.slow(r(c)), InfraUser.slow(c) );
    az(i) = az(i)*180/pi;
    if az(i)<0
      az(i) = az(i) + 360;
    end
    slofk2 = slofk(i);
    slofk(i) = (1/slofk(i))*111.1949;

    % Computing F-statistic and correlation:
    beam = tdelay(DATA,s_f,X,Y,az(i),slofk2);
    tmp1 = (sum(beam,2)./4);    % ***
    pow(i) = sum(tmp1.^2)/numel(tmp1);  % ***
    fstat(i) = bfstat(beam);
    avg_corr1(i) = corrp(beam);

%     if (isnan(avg_corr1(i)) == 1)
%         keyboard
%     end

    % Updating time:
    for j = 1:NTrace
        t{j}(i) = (t_start/s_f) + (InfraUser.twin/2);  % Time after trace start time (s)
        s{j}(i) = t{j}(i) * s_f;                 % Sample corresponding to measurements
    end

    time(i) = InfraConfig.db.stime + t{1}(i)/86400;  % Measurement time

    t_start = t_start + (shift * s_f); % Updating processing starting sample
    start_time = time(i);              % Updating processing start time

end

% close(waithandle);

[F_K.time,i] = setdiff(time,0);
F_K.slofk = slofk(i);
F_K.az = az(i);
F_K.pow = pow(i);   % ***
F_K.fstat = fstat(i);
F_K.corr = avg_corr1(i);
