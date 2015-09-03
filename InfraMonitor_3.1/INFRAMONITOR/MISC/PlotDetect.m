function PlotDetect(detect_file)
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

f = fopen(detect_file,'r');
C = textscan(f,'%s %s %s %s %s %f %f %f');

Arrays = unique(C{1});

for j = 1:numel(Arrays)
    Array = Arrays{j};

    I = find(strcmp(C{1},Array) == 1);

    for i = 1:numel(I)
        t(i) = datenum([C{2}{I(i)} ' ' C{3}{I(i)}],'yyyy-mm-dd HH:MM:SS');
        b(i) = C{6}(I(i));
        c(i) = C{8}(I(i));
        v_p(i) = C{7}(I(i));
    end

    figure; subplot('Position',[0.1 0.1 0.7 0.8])
    scatter(t,b,5,c,'filled'); datetick('x',6); ylim([0 360]);
    colorbar('location','WestOutside');
    ylabel('Backazimuth')
    xlabel('Time')
    title(Array(1:3))

    subplot('Position',[0.85 0.1 0.1 0.8])
    [h,xout] = hist(b,40);
    barh(xout,h,1); ylim([0 360]);
    set(gca,'XTick',[]); set(gca,'YTick',[])

%     figure
%     u = find(0.22 <= v_p & v_p <= 0.50);
%     PolPts = cat(2,b(u)',v_p(u)');
%     PolPtsU = unique(PolPts,'rows');
%     for j = 1:size(PolPtsU,1)
%         N(j) = numel(findrow(PolPts,PolPtsU(j,:)));
%     end
%     C = colormap(jet(max(N)));
%     polargeo(b(u),v_p(u),'.')
%
%     figure; scatter(PolPtsU(:,2),PolPtsU(:,1),5,log10(N),'filled'); xlim([0.2 0.5]); ylim([0 360]);
%     ylabel('Az (deg.)')
%     xlabel('Vel (km/s)')
%
    clear t b c v_p
end
