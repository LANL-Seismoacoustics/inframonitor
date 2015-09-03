function diff = distfit(fstats,dofnum,dofden,ncp,central)
% Computes the fit between an input distribution (fstats) and a noncentral
% distribution with non-centrality parameter (ncp)
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

x = (0.01:0.1:50.01);
if (central == 1)
    p1 = fpdf(x,dofnum,dofden);
    u = find(isnan(p1) == 1); p1(u) = 0;
    fstats = fstats/ncp;
else
    p1 = ncfpdf(x,dofnum,dofden,ncp);
    u = find(isnan(p1) == 1); p1(u) = 0;
end
[u,v] = max(p1);

% Smoothing observed distribution:
smoothWindow = 7;
dist = hist(fstats,x);
% dist = filter(ones(1,smoothWindow)/smoothWindow,1,dist);

% [u,v1] = max(dist);
% diff = abs(v-v1);

diff1 = corrcoef(dist,p1);
diff = diff1(1,2);
