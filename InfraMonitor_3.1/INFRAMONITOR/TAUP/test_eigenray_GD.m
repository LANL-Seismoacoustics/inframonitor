
%clear all;
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
close all;

%[ray1,eigenrays1] = eigenray([30.0 34.81],[50.409 58.034],200,'G2SGCSE2010010100_3_274.met',25,0,1);
%%
[ray2,eigenrays2] = eigenray_GD([30.0 34.81],[50.409 58.034],200,'G2SGCSE2010010100_3_274.met',125,0,1);

% %%
% %remember the axis are inverted
%
% k1=1;
% for i1=1:numel(eigenrays1)
%     %for j1=1:length(ray1(eigenrays1(i1)).x)
%     x=ray1(eigenrays1(i1)).x;
%     y=ray1(eigenrays1(i1)).y;
%     ph=ray1(eigenrays1(i1)).phi;
%
%     latEI1(k1)=y(end);
%     longEI1(k1)=x(end);
%     k1=k1+1;
% end
%%

k1=1;
for i1=1:numel(eigenrays2)
    %for j1=1:length(ray1(eigenrays1(i1)).x)
    x=ray2(eigenrays2(i1)).x;
    y=ray2(eigenrays2(i1)).y;
    ph=ray2(eigenrays2(i1)).phi;

    for t1=1:length(ray2(eigenrays2(i1)).latO)
        latEI2(k1)=ray2(eigenrays2(i1)).latO(t1);
        longEI2(k1)=ray2(eigenrays2(i1)).lonO(t1);
        k1=k1+1;
    end
end

%%
% %%
figure
      worldmap([25 60],[25 60])
      load coast
      plotm(lat, long,'k');

      plotm(34.81,30,'ro');
      plotm(58.034,50.409,'ro');
    hold on
%    plotm(km2deg(latEI1)+30.0,km2deg(longEI1)+34.81,'ks','MarkerSize',4,'MarkerFaceColor','b')
    plotm(latEI2,longEI2,'ys','MarkerSize',4,'MarkerFaceColor','b')
