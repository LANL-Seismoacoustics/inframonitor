function PlotPolyfile(polyfile,region,arraylats,arraylons,area,minArrays)
%
% PlotPolyfile - Plots polygons in an IMpoly file on a map projection
%
% Updated 11/13/10 to utilize a high-resolution coastline file, coast.dat, if it exists in the working directory
%
% e.g., PlotPolyfile('utah.IMpoly',[39 43 -115 -109],[40.9207 39.4731 39.7203 41.60715 40.653 40.07925],[-113.03005 -110.740125 -113.389475 -111.565225 -112.119475 -111.830075],10000)
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

figure

m_proj('mercator','longitudes',region(3:4),'latitudes',region(1:2));

if (exist('coast.dat','file') ~= 0)
	load coast.dat; m_line(coast(:,1),coast(:,2),'Color','k'); m_grid; hold on
else
	m_coast; m_grid; hold on
end

a = str2num(PolyClean(polyfile,area,region));

x = load(polyfile,'-ascii');



dbName = regexp(polyfile,'\.','Split');
f = fopen([dbName{1} '.IMassoc']);
line = fgetl(f);
k = 0;
while ischar(line)
    k = k + 1;
    splitline = regexp(line,'\S*','match');
    assocData(k,1) = datenum([splitline{1} ' ' splitline{2}],'yyyy-mm-dd HH:MM:SS');
    assocData(k,2) = str2double(splitline{3}); assocData(k,3) = str2double(splitline{4});
    line = fgetl(f);
end
fclose(f);




evids = intersect(a,unique(x(:,1)));
k = 0;
for i = 1:numel(evids)
    numArrays = numel(find(assocData(:,2) == evids(i)));
    if (numArrays >= minArrays)
        k = k + 1;
        u = find(x(:,1) == evids(i));
        m_line(x(u,4),x(u,3))
        m_text(x(u(1),4),x(u(1),3),num2str(evids(i)))
        EVIDS(k) = evids(i);
    end
end

for i = 1:numel(arraylats)
    m_plot(arraylons(i),arraylats(i),'^r')
end

disp(['Number of events: ' num2str(k)])
