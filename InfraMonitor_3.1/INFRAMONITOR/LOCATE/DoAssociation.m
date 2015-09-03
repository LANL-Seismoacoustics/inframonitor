function DoAssociation(min_arrays,NArrays)
%
% Performs associations for all combinations of NArrays, min_arrays at a
% time
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
% Redistributions of source code must retain the above copyright notice, this list
% 		  of conditions and the following disclaimer.
% Redistributions in binary form must reproduce the above copyright notice, this
% 	      list of conditions and the following disclaimer in the documentation and/or
% 	      other materials provided with the distribution.
% Neither the name of Los Alamos National Security, LLC, Los Alamos National
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

global InfraConfig InfraConfigBackup assoc toassoc AssocIn

save Assoc.mat

NRuns = nchoosek(1:NArrays,min_arrays);

k = 0; nassoc = 0;
for i = 1:size(NRuns,1)
    
    % Configuring InfraConfig.array and InfraConfig.grid for each run:
    InfraConfig.array = [];
    InfraConfig.grid.D_STN = []; InfraConfig.grid.BA_STN = [];
    Arrays = NRuns(i,:);
    for j = 1:numel(Arrays)
        InfraConfig.array.arr{j} = InfraConfigBackup.array.arr{Arrays(j)};
%         InfraConfig.array.trc{j} = InfraConfigBackup.array.trc{Arrays(j)};
        InfraConfig.array.loc{j} = InfraConfigBackup.array.loc{Arrays(j)};
        InfraConfig.grid.D_STN{j} = InfraConfigBackup.grid.D_STN{Arrays(j)};
        InfraConfig.grid.BA_STN{j} = InfraConfigBackup.grid.BA_STN{Arrays(j)};
    end
    
    % Running the MSEI association routines:
    Arrivals = FirstArrivals();
    try
        [Y,Arids] = associate(Arrivals,k);
    catch
        continue
    end
    
    %---
    Arids = RemoveDuplicateArids(Arids);
    %---
    
    % Producing an input association structure (associn):
    for j1 = 1:size(Arids,1)
        
        nassoc = nassoc+1;
        associn(nassoc).arrays = Arrays;
        aridCorr = 0;
        for j = 1:size(Arids,2)
            arrivs = datenum(strcat(Y{j}{1:2}),'yyyy-mm-ddHH:MM:SS');
            baz(j) = Y{j}{3}(Arids(j1,j)-aridCorr);
            atimes(j) = arrivs(Arids(j1,j)-aridCorr);
            aridCorr = aridCorr+numel(Y{j}{1});
        end
        associn(nassoc).atimes = atimes;
        associn(nassoc).baz = baz;
    
    end
    
end

% Cleaning input association structure and adding to global structure:
try
    [assoc,toassoc] = CleanAssoc(associn);
    AssocIn = associn;
catch
    assoc = AssocIn; toassoc = 0;
end
