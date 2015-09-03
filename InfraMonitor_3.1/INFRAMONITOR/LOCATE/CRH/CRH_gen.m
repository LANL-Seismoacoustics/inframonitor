function [ CRH_tot CRH_par ] = CRH_gen( CRH_par,atmos_file,range_dependent)
%This function creates a CRH a bin file or a met file
%   Detailed explanation goes here
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

global ray;
dz=200;
h=0;
[ a b c]=fileparts(atmos_file);
if range_dependent==1
    if(strcmp(c,'.bin')==0)
        disp('BIN file should be provided to create seudo-range dependent CRH');
        return
    end
else
    if(strcmp(c,'.bin')==0 & strcmp(c,'.met')==0)
        disp('BIN or MET files should be provided to create ray range independent CRH');
        return
    end    
end
CRH_par.rangePROP=0:CRH_par.step_range:CRH_par.max_range;
edg={CRH_par.celerityrange,CRH_par.rangePROP};
%edg=edg;
    j1=1;
    
    for k1=1:length(CRH_par.phiV)
        for m1=1:length(CRH_par.thetaV)
            if(range_dependent==0)
              taup(atmos_file,dz,h,CRH_par.phiV(k1),CRH_par.thetaV(m1),CRH_par.max_range,0,j1,0,1,1);            
            else
            taupRD(atmos_file,CRH_par.lat,CRH_par.lon,dz,h,CRH_par.phiV(k1),CRH_par.thetaV(m1),CRH_par.max_range,0,j1,0,1,1);                
            end
            j1=j1+1;
        end    
    end

num_ele=0;
for h1=1:length(ray)
    num_ele=num_ele+length(ray(h1).x); 
end

linearDATA=nan(num_ele,5);
start_p=1;
end_p  =0;
for h1=1:length(ray)    
    if(isempty(ray(h1).x)==1)
        continue;
    end    
    x=ray(h1).x;
    y=ray(h1).y;
    rang=sqrt(x.^2+y.^2);
    if(range_dependent==0)
        rang=sqrt(x.^2+y.^2);
        vg=ray(h1).v_g+rang*0;
        phi=ray(h1).phi+rang*0;
    else
        %rang=sqrt(x.^2+y.^2);
        rang=deg2km(distance(y,x,CRH_par.lat,CRH_par.lon));
        vg=ray(h1).v_g;
        phi=ray(h1).phi;
    end
    start_p=(end_p+1);
    end_p  =start_p+length(vg)-1;
    vg(vg~=real(vg))=nan;
    rang(vg~=real(vg))=nan;
    linearDATA(start_p:end_p,1:5)=[vg' rang' x' y' phi'];
    
end 

keyboard
[N edges mid loc] = histcn(linearDATA(:,1:2),CRH_par.celerityrange,CRH_par.rangePROP);
CRH_tot=N(:,:,ceil(end/2));
% figure;
% imagesc(CRH_par.v_step,CRH_par.rangePROP,CR_HIST);
%     
%   if(isempty(ray(h1).v_g)==1)
%       vg(h1)=nan;
%       zm(h1)=nan;
%       range(h1)=nan;
%   else
%       vg(h1)=ray(h1).v_g;
% 
%       if(x~=0 & isreal(x)==1 & isreal(y)==1) 
%           range(h1,1:length(x))=sqrt(x.^2+y.^2);
%           ang_dev(h1,1:length(x))=atan2(y,x);
%       else
%           range(h1,1:length(xa))=nan;
%           ang_dev(h1,1:length(xa))=nan;
%       end
%   end        
% end
% 
% 
% 
% traidDATA=nan(length(vg),3);
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
% CR_HIST=N(:,:,ceil(end/2));


end
% 
% for r1=5542:11000
% global ray;
% [a b , ~, b1]=first_bounce2(['Prof_folder/UtahProf' num2str(r1,'%05d') '.met'],200,0,phiV,thetaV,2000,1);
% range=nan(length(ray),num_max_bounce);
% ang_dev=nan(length(ray),num_max_bounce);
% vg=nan(1,length(ray));
% zm=nan(1,length(ray));
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
%       if(xa~=0 & isreal(xa)==1 & isreal(ya)==1) 
%           range(h1,1:length(xa))=sqrt(xa.^2+ya.^2);
%           ang_dev(h1,1:length(xa))=atan2(ya,xa);
%       else
%           range(h1,1:length(xa))=nan;
%           ang_dev(h1,1:length(xa))=nan;
%       end
%   end        
% end
% 
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
% CR_HIST=N(:,:,ceil(end/2));
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
% file_info_proc='test_save1';
% save(['Proccesed_taup_prof/UtahProccesed' num2str(r1,'%05d') '.mat'],'ray','rangePROP','v_step','ang_dev_Md','CR_HIST');
% clear global ray;
% fclose all;
% end
