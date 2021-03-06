close all;
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
clear all;

hold on;

turningRays = 1;
reverse = 0;
phi=-45;
theta=25;
resolution=200;
h_ele=0;

fullray=0;  % 0 fullray, 1 BP
global ray;
plotflag_V=0;
max_range=1000;
%test taup3
i1=1;
for i1=1:10
taup3('G2SGCSE2010010100_3_274.met',resolution,h_ele,phi,i1,max_range,plotflag_V,i1 ,reverse,fullray,turningRays,1);
RotateXY(i1);
BouncePoints(i1);
plot(sqrt(ray(i1).x.^2+ray(i1).y.^2),ray(i1).z);
hold on;
%RotateXY_BouncePoints(i1);
%disp(ray(i1));
end
figure
ini=i1;

for i1=ini+1:ini+10
taup3('G2SGCSE2010010100_3_274.met',resolution,h_ele,phi,i1-ini,max_range,plotflag_V,i1 ,reverse,fullray,turningRays);
RotateXY(i1);
BouncePoints(i1);
plot(sqrt(ray(i1).x.^2+ray(i1).y.^2),ray(i1).z);
hold on;
%RotateXY_BouncePoints(i1);
%disp(ray(i1));
end




%
% i1=i1+1;
% taup3('UtahProf00002.met',resolution,h_ele,phi,theta,max_range,plotflag_V,i1 ,reverse,fullray,turningRays);
% %disp(ray);
% % remember I need to rotate the full wave and the bounce points
% RotateXY(i1);
% RotateXY_BouncePoints(i1);
% disp(ray(i1).xb');
% plot(ray.x,ray.z)
%
% clear global ray;
% global ray;
% taup3('UtahProf00002.met',resolution,h_ele,phi,theta,max_range,plotflag_V,i1 ,reverse,fullray,turningRays);
% disp(ray);
% figure
% plot(ray.x,ray.z)
%
% clear global ray;
% global ray;
% fullray=1;
% taup3('UtahProf00002.met',resolution,h_ele,phi,theta,max_range,plotflag_V,i1 ,reverse,fullray,turningRays);
% disp(ray);

%
% range=nan(length(ray),num_max_bounce);
% ang_dev=nan(length(ray),num_max_bounce);
%
% for h1=1:length(ray)
%   if(isempty(ray(h1).v_g)==1)
%       vg(h1)=nan;
%       zm(h1)=nan;
%       range(h1)=nan;
%   else
%       vg(h1)=ray(h1).v_g;
%       zm(h1)=ray(h1).zmax;
%       xa=ray(h1).xb;
%       ya=ray(h1).yb;
%       if(xa~=0)
%           range(h1,1:length(xa))=sqrt(xa.^2+ya.^2);
%           ang_dev(h1,1:length(xa))=atan2(ya,xa);
%       else
%           range(h1,1:length(xa))=nan;
%           ang_dev(h1,1:length(xa))=nan;
%       end
%       end
% end
% %%
%
% %num of bounces 10;
% traidDATA=nan(num_max_bounce*length(vg),3);
% for j1=1:num_max_bounce
%     start_p=1+length(vg)*(j1-1);
%     end_p  =start_p+length(vg)-1;
%     traidDATA(start_p:end_p ,:)=[vg' range(:,j1) ang_dev(:,j1)];
% end
%
%
% veloc_inter=[edg{1}];
% range_inter=[edg{2}];
% [N edges mid loc] = histcn(traidDATA(:,1:2),veloc_inter,range_inter);
%
%
% locNAN=loc;
% loc_no_NAN=loc;
% a1=find(loc(:,1)==0);
% a2=find(loc(:,2)==0);
% aUNION=union(a1,a2);
%
% locNAN(aUNION,:)=nan;
% loc_no_NAN(aUNION,:)=[];
%
% [aN bN]=size(N);
% ang_dev_M=nan(aN,bN);
% ang_dev_M_div=ones(aN,bN);
% for t1=1:length(traidDATA)
%     if(loc(t1,1)~=0 && loc(t1,2)~=0)
%     ang_dev_M(locNAN(t1,1),locNAN(t1,2))=nansum([ang_dev_M(locNAN(t1,1),locNAN(t1,2)) abs(traidDATA(t1,3))]);
%     ang_dev_M_div(locNAN(t1,1),locNAN(t1,2))=ang_dev_M_div(locNAN(t1,1),locNAN(t1,2))+1;
%     end
% end
%
% ang_dev_Md=rad2deg(ang_dev_M./ang_dev_M_div);
%
%
%
% figure;
% av(1)=subplot(121);
% h(1)=imagesc(rangePROP,v_step,N(:,:,ceil(end/2)));
% set(get(h(1),'Parent'),'YDir','normal',...
% 'XGrid','on','YGrid','on',...
%     'XTick',rangePROP(1:10:end),'YTick',v_step,...
%     'XMinorTick','on','XMinorGrid','on');
% xlabel(av(1),'Range (km)');
% ylabel(av(1),'Celerity (km/s)');
% title('celerity-range histogram','FontWeight','b','FontSize',12)
% colorbar;
% av(2)=subplot(122);
%
%
% h(2)=imagesc(rangePROP,v_step,ang_dev_Md);
% set(get(h(2),'Parent'),'YDir','normal',...
% 'XGrid','on','YGrid','on',...
%     'XTick',rangePROP(1:10:end),'YTick',v_step,...
%     'XMinorTick','on','XMinorGrid','on');
% xlabel(av(2),'Range (km)');
% ylabel(av(2),'Celerity (km/s)');
% title('celerity-range azimuth deviation','FontWeight','b','FontSize',12)
% hc(2)=colorbar;
% ylabel(hc(2),'degrees');
% set(h(2),'alphadata',~isnan(ang_dev_Md))
%
% print -deps2c CR_CRDEV_Event_UTAH_03000
%
%
% %%
% % av=subplot(122);
% % colorL=cool(16);
% % vel_range=linspace(0.20,0.35,length(colorL));
% %
% % for h1=1:length(ray)
% %   if(isempty(ray(h1).x)==0)
% %       if(mod(h1,4)==0)
% %           plot(sqrt(ray(h1).x.^2+ray(h1).y.^2),ray(h1).z,'Color',colorL(find(vel_range<=ray(h1).v_g,1,'last'),:));
% %       end
% %       hold on;
% %   end
% % end
% %
% % for h1=1:length(colorL)
% %      text(1400,120-h1*5,num2str(ray(h1).v_g,'V %1.3f'),'Color',colorL(find(vel_range<=ray(h1).v_g,1,'last'),:),'FontWeight','b');
% end
% %%
%
% print -deps2c celerity_dist_Event_UTAH_03000


