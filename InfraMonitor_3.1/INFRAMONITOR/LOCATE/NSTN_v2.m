function [EVENTS] = NSTN_v2(stns,stn_combs,b_offset,gv_min,gv_max,T,...
    A,ndet,ARIDS,D_STN,BA_STN,LAT,LON,p_err)
% ========================================================================
% NSTN_v2.m
% ---------
% Grid search algorithm to associate groups of N first-arrivals at N
% stations into events.
%
% Stephen Arrowsmith (Last modified: 03/2007)
%
% Inputs:
% -------
% - stns: Vector of station ID numbers. (e.g., stns = [1 2 3])
% - stn_combs: All combinations of pairs of stations (e.g., stn_combs = [1 2;1 3;2 3])
% - b_offset: Allowed backazimuth deviation in degrees (e.g., b_offset = 3)
% - T: Cell array containing arrival times of each detection at each station (in days)
%      (Note: Each cell corresponds to a unique station)
% - A: Cell array containing backazimuths of each detection at each station
%      (Note: Same format as "T")
% - ndet: Groups of N detections at N stations that have been grouped
%         together (refer to CONSIS_V2.m)
%         (Note: "ndet" does not contain ARID's but pointers to the ARID's)
% - ARIDS: Cell array containing ARID's (Arrival ID's) of each detection at
%          each station
%          (Note: Same format as "T")
%
% Outputs:
% --------
% - EVENTS: Cell array containing three columns: (1) lat, (2) lon, (3) ARID's
%           (Note: each line in (3) is a group of events that have been
%           associated with the grid node at (lat,lon))
%
% Global Parameters:
% ------------------
cel_max = gv_max; % Maximum infrasonic celerity
cel_min = gv_min; % Minimum infrasonic celerity
% ========================================================================
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

p_err = p_err*2;    % Pick error in s

% (1) Re-organizing detection data:

NCOMB = size(stn_combs,1);  % NCOMB = number of station combinations
NSTN = numel(T);            % NSTN = number of stations

% For each station, obtaining ARID's (Arrival ID's) of detections / Converting
% times of detections to seconds:

ba_in = cell(1,NSTN);
for i = 1:NSTN
    [tf,locs] = ismember(ndet(:,i),ARIDS{i});
    T{i} = T{i}(locs)*86400;  % Converting arrival times from days to seconds
    arids(:,i) = ARIDS{i}(locs);   % Taking ARID's associated with line numbers
    ba_in{i} = A{i}(locs);
end

% For each station combination, computing delay-times:
dt_in = cell(1,NCOMB);
for i = 1:NCOMB
    dt_in{i} = T{stn_combs(i,1)} - T{stn_combs(i,2)};
end

% ========================================================================

% (2) Loading great-circle backazimuths (in deg.) and distances (in km)
%     computed for a set of grid node locations / re-organizing data:

% Computing maximum and minimum delay-times between each pair of stations:
[mindt,maxdt] = GetGridLimits(NCOMB,stn_combs,D_STN,cel_min,cel_max,p_err);

% dtmax = cell(1,NCOMB); dtmin = cell(1,NCOMB);
% mindt2 = cell(1,NCOMB); maxdt2 = cell(1,NCOMB);
% for i = 1:NCOMB
%     dtmax{i} = ((D_STN{stn_combs(i,1)} - D_STN{stn_combs(i,2)})./0.28);
%     dtmin{i} = ((D_STN{stn_combs(i,1)} - D_STN{stn_combs(i,2)})./0.34);
%     mindt{i} = min(dtmin{i},dtmax{i}) - p_err;
%     maxdt{i} = max(dtmin{i},dtmax{i}) + p_err;
% end
% 
% ========================================================================

% (3) Grid search: FOR-loop takes either (a) each grid node separately or (b) each
% N-station detection separately (depending on whether there are more grid nodes or
% N-station detections. In each case, the program finds
% detections with (i) delay-times that lie within the maximum and minimum
% delay-times for the given grid node, and (ii) backazimuths within +/-
% b_offset of the great-circle backazimuth for the given grid node)

if (numel(LAT) < numel(dt_in{1}))
	
	l = 0;
	
	for i = 1:numel(LAT)        % Taking each grid node separately
               
        % Defining the initial set of detection variables for each grid node
        % (this set is reduced using the constraints in (a) and (b) below:
        arids1 = arids;
        dt = cell(1,NCOMB);
        
        for j = 1:NCOMB
            dt{j} = dt_in{j};
        end

        ba = cell(1,NCOMB);
        for j = 1:NSTN
            ba{j} = ba_in{j};
        end
        
        % (a) Finding detections within maximum and minimum delay-times (at
        % each station) for grid node:
        for j = 1:NCOMB
            u = find(mindt{j}(i) <= dt{j} & dt{j} <= maxdt{j}(i));
        	for k = 1:NCOMB
        		dt{k} = dt{k}(u);
        	end
        	for k = 1:NSTN
        		ba{k} = ba{k}(u);
        	end
            arids1 = arids1(u,:);
		end
        
        % (b) Finding detections within +/- b_offset degrees of grid node
        % backazimuth at each station:
        for j = 1:NSTN
        	if (BA_STN{j}(i) > (360-b_offset))
                u = find((BA_STN{j}(i)-b_offset) < ba{j} | ba{j} < ((BA_STN{j}(i)+b_offset)-360));
            elseif (BA_STN{j}(i) < b_offset)
                u = find(((BA_STN{j}(i)-b_offset)+360) < ba{j} | ba{j} < (BA_STN{j}(i)+b_offset));
            else
                u = find((BA_STN{j}(i)-b_offset) < ba{j} & ba{j} < (BA_STN{j}(i)+b_offset));
            end
            for k = 1:NCOMB
        		dt{k} = dt{k}(u);
        	end
        	for k = 1:NSTN
        		ba{k} = ba{k}(u);
        	end
            arids1 = arids1(u,:);
        end
            
        % Storing N-station events:
        if (numel(dt{1}) > 0)
            
            l = l + 1;
	
            EVENTS_TMP{l,1} = LAT(i);
            EVENTS_TMP{l,2} = LON(i);
            EVENTS_TMP{l,3} = arids1;
	
        end
            
    end
	
	clear arids
	
    if (exist('EVENTS_TMP','var')==1)
        [u,v] = size(EVENTS_TMP);
        for j = 1:u
            arids = cat(1,EVENTS_TMP{j,3});
            [u2,v2] = size(arids);
            EVENTS_TMP2 = cat(2,repmat(EVENTS_TMP{j,1},u2,1),repmat(EVENTS_TMP{j,2},u2,1),arids);
            if (j == 1)
                EVENTS = EVENTS_TMP2;
            else
                EVENTS = cat(1,EVENTS,EVENTS_TMP2);
            end
        end
    end
	
	if (exist('EVENTS','var') == 0)
        EVENTS = 0;
    end
    
else
    
    l = 0;
	
	for i = 1:numel(dt_in{1})       % Looping over each N-station detection
        
        LAT1 = LAT;
        LON1 = LON;
        mindt1 = cell(1,NCOMB);
        maxdt1 = cell(1,NCOMB);
        for j = 1:NCOMB
            mindt1{j} = mindt{j};
            maxdt1{j} = maxdt{j};
        end
        
        BA_STN1 = cell(1,NSTN);
        for j = 1:NSTN
            BA_STN1{j} = BA_STN{j};
        end
        
        % (a) Time constraints:
        for j = 1:NCOMB
            u = find(maxdt1{j} >= dt_in{j}(i));
            
            for k = 1:NCOMB
        		mindt1{k} = mindt1{k}(u);
                maxdt1{k} = maxdt1{k}(u);
            end

            for k = 1:NSTN
        		BA_STN1{k} = BA_STN1{k}(u);
            end

            LAT1 = LAT1(u);
            LON1 = LON1(u);
            
        end
        for j = 1:NCOMB
            u = find(mindt1{j} <= dt_in{j}(i));
            
            for k = 1:NCOMB
        		mindt1{k} = mindt1{k}(u);
                maxdt1{k} = maxdt1{k}(u);
            end

            for k = 1:NSTN
        		BA_STN1{k} = BA_STN1{k}(u);
            end

            LAT1 = LAT1(u);
            LON1 = LON1(u);
        end
        
        % (b) Backazimuth constraints:
        for j = 1:NSTN
        	
            if (ba_in{j}(i) > (360-b_offset))
                u = find((ba_in{j}(i)-b_offset) < BA_STN1{j} | BA_STN1{j} < ((ba_in{j}(i)+b_offset)-360));
            elseif (ba_in{j}(i) < b_offset)
                u = find(((ba_in{j}(i)-b_offset)+360) < BA_STN1{j} | BA_STN1{j} < (ba_in{j}(i)+b_offset));
            else
                u = find((ba_in{j}(i)-b_offset) < BA_STN1{j} & BA_STN1{j} < (ba_in{j}(i)+b_offset));
            end
            
            for k = 1:NCOMB
        		mindt1{k} = mindt1{k}(u);
                maxdt1{k} = maxdt1{k}(u);
        	end
        	for k = 1:NSTN
        		BA_STN1{k} = BA_STN1{k}(u);
        	end
            LAT1 = LAT1(u);
            LON1 = LON1(u);
            
        end
        
        % Storing N-station events:
        if (numel(LAT1) > 0)
            
            l = l + 1;
	
            EVENTS_TMP{l,1} = LAT1;
            EVENTS_TMP{l,2} = LON1;
            EVENTS_TMP{l,3} = arids(i,:);
	
        end
        
    end
	
	if (exist('EVENTS_TMP','var')==1)
        [u,v] = size(EVENTS_TMP);
        for j = 1:u
            lats = cat(1,EVENTS_TMP{j,1})';
            lons = cat(1,EVENTS_TMP{j,2})';
            [u2,v2] = size(lats);
            EVENTS_TMP2 = cat(2,lats,lons,repmat(EVENTS_TMP{j,3},u2,1));
            if (j == 1)
                EVENTS = EVENTS_TMP2;
            else
                EVENTS = cat(1,EVENTS,EVENTS_TMP2);
            end
        end
    end
	
	if (exist('EVENTS','var') == 0)
        EVENTS = 0;
	end
	
end
