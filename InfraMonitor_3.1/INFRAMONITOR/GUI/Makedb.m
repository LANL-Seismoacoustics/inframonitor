function ok = Makedb(filename,pathname)
% Makedb - Generates the db structure array
%
% New Functionality for InfraMonitor 2.0
%
% Stephen Arrowsmith (08/22/08)
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

global InfraConfig

fid = fopen([pathname filename]);
InfraConfig.db.dir = pathname;
InfraConfig.db.name = regexprep(filename,'.wfdisc','');

% Extracting all columns from wfdisc file:

% Checking format of wfdisc file (Does not work on Windows):
try
    [A,B] = system(['cat ' filename ' | awk ''{print NF}''']);
    B = str2num(B);

    if (B(1) == 20)
        InfraConfig.db.wfdisc = textscan(fid,...
            '%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s');
        fclose(fid);
    elseif (B(1) == 21)
        InfraConfig.db.wfdisc = textscan(fid,...
            '%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s');
        fclose(fid);
    end

catch
    [A,B] = system(['cat ' filename ' | awk ''{print NF}''']);
    B = str2num(B(30:31));
    if (B == 20)
        InfraConfig.db.wfdisc = textscan(fid,...
            '%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s');
        fclose(fid);
    elseif (B == 21)
        InfraConfig.db.wfdisc = textscan(fid,...
            '%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s');
        fclose(fid);
    end
    %disp('Warning: wfdisc file verification is not available for Windows systems')
end

% Extracting first 5 columns from site file:

if (exist([pathname InfraConfig.db.name '.site'],'file') == 0)
    errordlg('No valid site file!')
    ok = -1;
    return
end

fid = fopen([pathname InfraConfig.db.name '.site']);
i = 0;
while 1
    i = i + 1;

    tline = fgetl(fid);

    if ~ischar(tline),   break,   end
    tline = regexp(tline,'\S*','match');
    if (i == 1)
        for j = 1:5
            InfraConfig.db.site{j} = tline(j);
        end
    else
        for j = 1:5
            try
                InfraConfig.db.site{j} = cat(1,InfraConfig.db.site{j},tline{j});
            catch
                keyboard
            end
        end
    end
end
fclose(fid);

ok = 0;
