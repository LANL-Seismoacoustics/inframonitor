% plot detection result for eacy array with azimuth
% by Junghyun Park
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

clear all
f = fopen('08_2011.detect','r');        % input file for detection plot obtained by running InfraMonitor
C = textscan(f,'%s %s %s %s %s %f %f %f');

Arrays = unique(C{1});

for j = 1:numel(Arrays)

    Array = Arrays{j};

    I = find(strcmp(C{1},Array) == 1);

    starting=datenum(['2011-08-01' ' ' '00:00:00'],'yyyy-mm-dd HH:MM:SS');
    ending=datenum(['2011-08-31' ' ' '00:00:00'],'yyyy-mm-dd HH:MM:SS');

    for i = 1:numel(I)
        t(i) = datenum([C{2}{I(i)} ' ' C{3}{I(i)}],'yyyy-mm-dd HH:MM:SS');
        b(i) = C{6}(I(i));    % Azimuth
        c(i) = C{8}(I(i));    % Correlation
        v_p(i) = C{7}(I(i));  % Phase Velocity
    end
    set_num=0.8-(0.14*(j-1));
    subplot('Position',[0.1 set_num 0.7 0.1])

    scatter(t,b,5,c,'filled');
    datetick('x',6);
    xlim([starting ending]);
    ylim([0 360]);
    ylabel('Backazimuth')
    colorbar('location','WestOutside');
    title(Array(1:3),'FontSize',12,'Fontweight','bold')

   subplot('Position',[0.8 set_num 0.1 0.1])
   rose(b*(pi/180),30)
   set(gca,'View',[-90 90],'YDir','reverse')

   if j==1
   title('Number of Detections','color','b','FontSize',12,'Fontweight','bold')
   end

   box on
     clear t b c v_p
end

