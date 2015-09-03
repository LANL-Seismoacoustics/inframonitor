function [t,p] = canonicalProfile(z)
%
% Provides the temperature and pressure for a given altitude using the
% canonical profile of Sutherland and Bass
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

if (z > 90)
    P0=-1.09914e+2;
    P1=4.7143;
    P2=-8.2123e-2;
    P3=6.622e-4;
    P4=-2.5593e-6;
    P5=3.84825e-9;
    T0=-6.8452e+4;
    T1=2.82794e+3;
    T2=-4.57421+1;
    T3=3.62065e-1;
    T4=-1.39987e-3;
    T5=2.121e-6;
elseif (z > 83)
    P0=-1.1279e+1;
    P1=3.1982e-1;
    P2=-5.8093e-3;
    P3=2.2331e-5;
    P4=0.;
    P5=0.;
    T0=1.7574e+2;
    T1=0;
    T2=0;
    T3=0;
    T4=0;
    T5=0;
elseif (z > 50)
    P0=0.76338;
    P1=-2.58298e-1;
    P2=3.76139e-3;
    P3=-4.20887e-5;
    P4=1.602e-7;
    P5=-1.92509e-10;
    T0=-5.0398e+2;
    T1=3.9214e+1;
    T2=-4.9518e-1;
    T3=-3.2622e-3;
    T4=9.6665e-05;
    T5=-4.788e-7;
elseif (z > 28)
    P0=2.18198;
    P1=-4.11497e-1;
    P2=1.33665e-2;
    P3=-3.59519e-4;
    P4=5.10097e-6;
    P5=-2.89056e-8;
    T0=-9.0104e+2;
    T1=1.5875e+2;
    T2=-8.9255e+0;
    T3=2.4617e-1;
    T4=-3.2807e-3;
    T5=1.6883e-5;
elseif (z > 18)
    P0=0.984143;
    P1=-0.269769;
    P2=8.52275e-3;
    P3=-3.96203e-4;
    P4=1.01465e-5;
    P5=-1.02643e-7;
    T0=2.1212e+3;
    T1=-4.1918e+2;
    T2=3.6474e+1;
    T3=-1.573e+0;
    T4=3.3667e-2;
    T5=-2.8579e-4;
elseif (z > 11)
    P0=-7.99108e-2;
    P1=-8.10464e-2;
    P2=-5.55224e-3;
    P3=3.1117e-4;
    P4=-1.66878e-5;
    P5=3.832e-7;
    T0=1.2535e+3;
    T1=-3.2827e+2;
    T2=4.1898e+1;
    T3=-2.6724e+0;
    T4=8.4485e-2;
    T5=-1.0526e-3;
else
    P0=1.68716e-2;
    P1=-1.14252e-1;
    P2=-1.36123e-3;
    P3=7.36241e-5;
    P4=-1.08003e-5;
    P5=3.30464e-7;
    T0=2.85e+2;
    T1=-5.071619e+0;
    T2=1.9778e-1;
    T3=-5.6132e-2;
    T4=1.4636e-3;
    T5=1.42e-4;
end

B=(P0 + P1*z + P2*z*z + P3*z*z*z + P4*z*z*z*z + P5*z*z*z*z*z);

p=(exp(B))*10^5;

t=T0 + T1*z + T2*z*z + T3*z*z*z + T4*z*z*z*z + T5*z*z*z*z*z;
