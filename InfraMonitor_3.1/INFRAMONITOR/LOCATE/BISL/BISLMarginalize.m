function BISLMarginalize()
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

global InfraConfig
NEvents = numel(InfraConfig.BISL.PDF4);
InfraConfig.BISL.PDF3 = cell(NEvents,1);
InfraConfig.BISL.PDF2 = cell(NEvents,1);
Credibilities = cell(NEvents,1);

for p=1:NEvents
    
    Z_4 = InfraConfig.BISL.PDF4{p};
    len = InfraConfig.BISL.len{p};

  %COMPUTE MARGINAL POSTERIOR PDF (REMOVE v, t0 BY INTEGRATION)
    if (ndims(Z_4) == 4)
        Z_3 = summarginal(Z_4,len); InfraConfig.BISL.PDF3{p} = Z_3;
        Z_2 = summarginal(Z_3,len); InfraConfig.BISL.PDF2{p} = Z_2;
    else
        Z_2 = Z_4; InfraConfig.BISL.PDF2{p} = Z_2;
    end
    
  %COMPUTE CREDIBILITY VALUES
    z_max = max(max(Z_2));
    LevelMax = z_max + 10^(floor(log10(z_max))-1);
  %Construct vector containing z-slice values
    ZList = LevelMax:-LevelMax/5000:0;
    NLevels = length(ZList);
    CList = zeros(NLevels-1,1);
    Credibilities{p} = zeros(size(Z_2));
  %Loop over z-slices
    for i=2:NLevels
        temp = Z_2;
        temp_ind = find( (temp < ZList(i)) );
      %Zero points that lie below z-slice
        temp(temp_ind) = 0;
      %Integrate volume bounded by z-slice
        CList(i-1) = sumdouble(temp,len(1:2));
      
      %Identify points between adjacent z-slices
        Credibilities_ind = find( (ZList(i) <= Z_2) & (Z_2 < ZList(i-1)) );
      %Assign credibility values 
        Credibilities{p}(Credibilities_ind) = CList(i-1);
    end
    
end

InfraConfig.BISL.Credibilities = Credibilities;
