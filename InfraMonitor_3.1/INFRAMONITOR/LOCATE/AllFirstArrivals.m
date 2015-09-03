function Y = AllFirstArrivals()
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

global InfraConfig

InfraUser = open('InfraUser.mat');

% Defining maximum backazimuth and duration constraints:

db = InfraUser.az_dev;

for i = 1:numel(InfraConfig.grid.D_STN)
    max_d(i) = max(max(InfraConfig.grid.D_STN{i}));
end
dt = (max(max_d)/InfraUser.vmin) - (max(max_d)/InfraUser.vmax);

% Extracting first arrivals:

ARID = 0;

for i = 1:numel(InfraConfig.array.arr)
    
    % Selecting arrival data for i'th array:
    u = find(InfraConfig.array.arr{i}(:,5) == 2);
    T = InfraConfig.array.arr{i}(u,1);
    A = InfraConfig.array.arr{i}(u,3);
    
    k = 0;                              % Counts number of first arrivals at i'th array
    
    for j = 1:numel(T)
        
        if (numel(find(InfraConfig.array.select{i} == j)) == 0)
            continue
        end
        
        ARID = ARID + 1;                % Assigning each adaptive arrival a unique ID
        
        k = k + 1;
        X{1}{k} = datestr(T(j),'yyyy-mm-dd');
        X{2}{k} = datestr(T(j),'HH:MM:SS');
        X{3}(k) = A(j);
        X{4}(k) = ARID;
        X{5}(k) = 1;

    end
    
    Y{i} = X;
    clear X;
    
end
