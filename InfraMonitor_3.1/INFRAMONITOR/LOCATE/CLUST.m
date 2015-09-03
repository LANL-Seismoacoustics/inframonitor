%----------------------------------------------------------------------------------------------------------
function [ndet] = CLUST(B,stn_combs)
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

% h = waitbar(0,'Please Wait');

% 1) FINDING EVENT NUMBERS (FOR EACH STATION SEPARATELY) THAT ARE REPEATED IN DIFFERENT STATION PAIRS

stn_numbers = unique(stn_combs);    % Individual station numbers

% waitbar(0,h,'CLUST: Finding repeated event #s');
for i = 1:max(stn_numbers)          % Looping over each individual station
    
%     waitbar(i/max(stn_numbers))
    
    stn = stn_numbers(i);           % Station number
    [a,b] = find(stn_combs == stn); % a = Rows in stn_combs containing station; b = Columns
    
    for j = 1:numel(a)              % Looping over each station pair that station is in
        
        x{j} = B{a(j)}(:,b(j));
        
        if (j > 1)
            
            in{j} = intersect(x{j},x_sav);      % Storing event numbers that are repeated in different station-pairs
            x_sav = cat(1,x_sav,x{j});
        
        else
            
            x_sav = x{j};
            
        end
        
    end
    
    % Reorganizing repeated event numbers...
    EVID{i} = unique(cat(1,in{:}));     % For each station, stores event numbers that are repeated in different station-pairs
    
end

% 2) FINDING EVENTS AT DIFFERENT STATIONS THAT ARE PAIRED WITH REPEATED EVENTS...

% waitbar(0,h,'CLUST: Finding paired events');
for i = 1:numel(EVID)           % Taking each station separately
    
%     waitbar(i/numel(EVID))
    
    stn = stn_numbers(i);       % Station number
    
    for j = 1:numel(EVID{i});   % Taking each event number separately
        
        evt = EVID{i}(j);               % Event number
        
        % a) Finding lists of event numbers at other stations that are
        % pairs with the input event number at the input station...
        
        [a,b] = find(stn_combs == stn); % a = Rows in stn_combs containing station; b = Columns
        
        for j = 1:numel(a)              % Looping over each relevant station pair
        
            % Finding rows in station pair that contain event number (at
            % correct station)...
            [u,v] = find(B{a(j)} == evt);   % Finding event number (u=rows, v=columns)
            u = u(find(v == b(j)));         % Taking only rows for input station
            
            % Finding paired events at other stations...
            if (b(j) == 1)
                evts_other_station = B{a(j)}(u,2);      % Note: vector
                other_station = stn_combs(a(j),2);      % Note: integer
            else
                evts_other_station = B{a(j)}(u,1);
                other_station = stn_combs(a(j),1);
            end
            
            % Storing paired events (where column is the station number and
            % the line represents the station pairing number)...
            pairs{j,other_station} = evts_other_station;
            pairs{j,i} = [];
            
        end
        
        % PAIRS is a cell array containing event numbers that pair with a particular event (evt) at
        % a particular station (stn). Each cell is a separate cell array of event numbers where the
        % column number denotes the station number associated with the event number...
        PAIRS{evt,stn} = pairs;
        clear pairs;
        
    end
    
end

isnstat = exist('PAIRS','var');      % Check to see if n-station detections exist
if (isnstat == 0)
    ndet = 0;
    return
end

% 3) REARRANGING PAIRS INTO A MATRIX OF n-STATION DETECTIONS...

k = 0;

[i_max,j_max] = size(PAIRS);

% waitbar(0,h,'CLUST: Rearranging pairs into n-station detections');

for j = 1:j_max                 % Looping over columns in PAIRS (i.e., station numbers)
    
%     waitbar(j/j_max)
    
    for i = 1:i_max                     % Looping over rows in PAIRS
        
        if (numel(PAIRS{i,j})==0)
            
            continue
            
        end
        
        k = k + 1;
        
        mat_out = SORT_PAIRS(PAIRS{i,j},i,j);
        
        if (k == 1)
            ndet = mat_out;
        else
            ndet = cat(1,ndet,mat_out);
        end
        
    end
    
end

% save CLUST.mat ndet

% close(h)

