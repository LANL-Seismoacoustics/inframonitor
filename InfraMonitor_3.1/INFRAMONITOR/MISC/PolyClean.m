% function a = PolyClean(filename,area,region)
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
function a = PolyClean(filename,area)
%
% Identifies events (in .IMpoly file) with uncertainties < area that have
% fully enclosed polygons within the specified region
%
% Stephen Arrowsmith (arrows@lanl.gov)

p = load(filename,'-ascii');
NEvents = p(size(p,1),1);

% a_e = almanac('earth','surfarea','km');     % Area of Earth in km^2
a_e = 510064471.909788;

for i = 1:NEvents
    try
        j = find(p(:,1) == i);    % Find polypoints for i'th polygon

%         if (numel(find(p(j,3)==region(1) | p(j,3)==region(2))) > 0)
%             a(i) = 99999999;
%         elseif (numel(find(p(j,4)==region(3) | p(j,4)==region(4))) > 0)
%             a(i) = 99999999;
%         else
            try
                a(i) = areaint(p(j,3),p(j,4))*a_e;
            catch
                % Mapping toolbox not found...approximating area in
                % Cartesian coordinates:
                x = (p(j,4)-min(p(j,4)))*111.1949;
                y = (p(j,3)-min(p(j,3)))*111.1949;
                a(i) = polyarea(x,y);
            end
%         end
    catch
        a(i) = 99999999;
    end
end

a = num2str(find(a < area));
