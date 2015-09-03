function ray = runtaup_GD(metfile,phi,theta,max_range,dz,h,reverse,fullray)
% runtaup - Computes multiple raypaths using the Tau-P method (shooting
% from source = [0,0])
% runtaup_GD - Computes multiple raypaths using the Tau-P method (shooting
% from source and using the reckon code the properly assess the influence
% of distance at global distances.
%
% Refer to taup.m for a description of the parameters
%
% Examples:
% ray = runtaup('METDATA/RealModel.met',(90:1:135),(1:1:90),2000,2000,1)
% ray = runtaup('METDATA/RealModel.met',90,(1:1:90),2000,2000,1)
%
% Stephen Arrowsmith (arrows@lanl.gov)
% Modified by Omar removed Rotation XY or bounce points.
% I removed the RotateXY(k) as it better use reckon to account for the
% cahnges in latitude!
% the output of this function is ray.x and ray.y stil range and offset. the
% following function requires to properly change this to x and y longitude
% and latitude respetively.
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




clear global ray; warning off all
global ray

C = progress('init','runtaup - Computing ray paths');

k = 0;
for i = 1:numel(phi)
    %disp(' inside phi');
    C = progress(C,i/numel(phi));
    for j = 1:numel(theta)
        %disp(' inside theta');
        k = k + 1;
        try
            %disp(' inside loop');
            taup(metfile,dz,h,phi(i),theta(j),max_range,0,k,reverse,fullray,1);
            %RotateXY(k);
            %AzDev(k);
            %BouncePoints(k);
            ZMax(k);
        catch EE
            disp('problem with taup_GD')
        end
    end
end
