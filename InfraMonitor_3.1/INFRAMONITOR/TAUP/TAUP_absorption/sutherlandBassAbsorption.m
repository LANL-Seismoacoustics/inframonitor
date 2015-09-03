function alpha = sutherlandBassAbsorption(freq,z,P_o,T_o,P_z,T_z,rho_z)
%
% Calculates the absorption (Np/km) for a given frequency and altitude
% provided with necessary atmospheric parameters (temperature, pressure and
% density at a given altitude and reference temperature and pressure)
%
% Inputs:
% freq = Frequency (Hz)
% z = Altitude (km)
% P_o = Pressure (Pa) at sea-level
% T_o = Temperature (K) at sea-level
% P_z = Pressure (Pa) at altitude z
% T_z = Temperature (K) at altitude z
% rho_z = Density (kg/m^3) at altitude z
%
% e.g.,
% alpha = sutherlandBassAbsorption(1,9.8,101081.26,304.0586,29695.0134,242.3767,0.4269)
%
% Based on NCPA C++ code
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

% Atmospheric composition constants:
mu_o  = 18.192E-6;             % Reference viscosity [kg/(m*s)]
S     = 117;   	   			   % Sutherland constant [K]
gamma = 1.4;                   % Specific heat ratio
Cv_R(1) = 5/2;                 % Heat capacity|volume (O2)
Cv_R(2) = 5/2;                 % Heat capacity|volume (N2)
Cv_R(3) = 3;                   % Heat capacity|volume (CO2)
Cv_R(4) = 3;                   % Heat capacity|volume (O3)
Cp_R(1) = 7/2;                 % Heat capacity|pressure (O2)
Cp_R(2) = 7/2;                 % Heat capacity|pressure (N2)
Cp_R(3) = 4;                   % Heat capacity|pressure (CO2)
Cp_R(4) = 4;                   % Heat capacity|pressure (O3)
theta(1)= 2239.1;              % Charact. temperature (O2)
theta(2)= 3352;                % Charact. temperature (N2)
theta(3)= 915;                 % Charact. temperature (CO2)
theta(4)= 1037;                % Charact. temperature (O3)

% *************************************************************************

c_snd_z = sqrt(gamma*P_z/rho_z); % in m/s
mu      = mu_o*sqrt(T_z/T_o)*((1+S/T_o)/(1+S/T_z)); % Viscosity [kg/(m*s)]
nu      = (8*pi*freq*mu)/(3*P_z);                   % Nondimensional frequency


% -------- Gas fraction polynomial fits -----------------------------------
if (z > 90.)                                         % O2 profile
    X(1) = power(10,49.296-(1.5524*z)+(1.8714E-2*power(z,2))-(1.1069E-4*power(z,3))+(3.199E-7*power(z,4))-(3.6211E-10*power(z,5)));
else
    X(1) = power(10,-0.67887);
end

if (z > 76.)                                         % N2 profile
    X(2) = power(10,(1.3972E-1)-(5.6269E-3*z)+(3.9407E-5*power(z,2))-(1.0737E-7*power(z,3)));
else
    X(2) = power(10,-0.10744);
end

X(3)  = power(10,-3.3979);                             % CO2 profile

if (z > 80. )                                        % O3 profile
    X(4) = power(10,-4.234-(3.0975E-2*z));
else
    X(4) = power(10,-19.027+(1.3093*z)-(4.6496E-2*power(z,2))+(7.8543E-4*power(z,3))-(6.5169E-6*power(z,4))+(2.1343E-8*power(z,5)));
end

if (z > 95. )                                        % O profile
    X(5) = power(10,-3.2456+(4.6642E-2*z)-(2.6894E-4*power(z,2))+(5.264E-7*power(z,3)));
else
    X(5) = power(10,-11.195+(1.5408E-1*z)-(1.4348E-3*power(z,2))+(1.0166E-5*power(z,3)));
end

                                                   % N profile
X(6)  = power(10,-53.746+(1.5439*z)-(1.8824E-2*power(z,2))+(1.1587E-4*power(z,3))-(3.5399E-7*power(z,4))+(4.2609E-10*power(z,5)));

if (z > 30. )                                         % H2O profile
    X(7) = power(10,-4.2563+(7.6245E-2*z)-(2.1824E-3*power(z,2))-(2.3010E-6*power(z,3))+(2.4265E-7*power(z,4))-(1.2500E-09*power(z,5)));
else
    if (z > 100.)
        X(7) = power(10,-0.62534-(8.3665E-2*z));
    else
        X(7) = power(10,-1.7491+(4.4986E-2*z)-(6.8549E-2*power(z,2))+(5.4639E-3*power(z,3))-(1.5539E-4*power(z,4))+(1.5063E-06*power(z,5)));
    end
end

X_ON = (X(1) + X(2))/0.9903;

% -------- Rotational collision number-------------------------------------
Z_rot(1) = 54.1*exp(-17.3*(power(T_z,-1./3.)));   % O2
Z_rot(2) = 63.3*exp(-16.7*(power(T_z,-1./3.)));   % N2
Z_rot_   = 1./((X(2)/Z_rot(2))+(X(1)/Z_rot(1)));

% -------- Nondimensional atmospheric quantities---------------------------
sigma = 5./sqrt(21.);
nn = (4./5.)*sqrt(3./7.)*Z_rot_;
chi=3.*nn*nu/4.;
cchi=2.36*chi;

% ---------Classical + rotational loss/dispersion--------------------------
beta_0  = 2*pi*freq/c_snd_z;
beta_1  = beta_0*sqrt(0.5*(sqrt(1+nu^2)+1)/(1+nu^2));
beta_2  = beta_0*sqrt((1+chi^2)/(1+(sigma*chi)^2));
alpha_1 = beta_0*sqrt(0.5*(sqrt(1+nu^2)-1)/(1+nu^2));
alpha_2 = beta_0*(((sigma/2-1/(2*sigma))*chi)/(sqrt((1+chi^2)*(1+(sigma*chi)^2))));

a_cl    = (2*pi*freq/c_snd_z)*sqrt(0.5*(sqrt(1+nu^2)-1)*(1+cchi^2)/((1+nu^2)*(1+(sigma*cchi)^2)));
a_rot   = (2*pi*freq/c_snd_z)*X_ON*((power(sigma,2)-1)*chi/(2*sigma))*sqrt(0.5*(sqrt(1+power(nu,2))+1)/((1+power(nu,2))*(1+power(cchi,2))));
a_diff  = 0.003*a_cl;

% ---------Vibrational relaxation------------------------------------------
Tr = power(T_z/T_o,-1./3.)-1;
A1 = (X(1)+X(2))*24*exp(-9.16*Tr);
A2 = (X(5)+X(6))*2400;
B  = 40400*exp(10*Tr);
C  = 0.02*exp(-11.2*Tr);
D  = 0.391*exp(8.41*Tr);
E  = 9*exp(-19.9*Tr);
F  = 60000;
G  = 28000*exp(-4.17*Tr);
H  = 22000*exp(-7.68*Tr);
I  = 15100*exp(-10.4*Tr);
J  = 11500*exp(-9.17*Tr);
K  = (8.48E08)*exp(9.17*Tr);
L  = exp(-7.72*Tr);
ZZ = H*X(3)+I*(X(1)+0.5*X(5))+J*(X(2)+0.5*X(6))+K*(X(7)+X(4));
hu = 100*(X(4)+X(7));
f_vib(1) = (P_z/P_o)*(mu_o/mu)*(A1+A2+B*hu*(C+hu)*(D+hu));
f_vib(2) = (P_z/P_o)*(mu_o/mu)*(E+F*X(4)+G*X(7));
f_vib(3) = (P_z/P_o)*(mu_o/mu)*ZZ;
f_vib(4) = (P_z/P_o)*(mu_o/mu)*(1.2E5)*L;

a_vib = 0.;
for m=1:4
    C_R          = ((power(theta(m)/T_z,2))*exp(-theta(m)/T_z))/(power(1-exp(-theta(m)/T_z),2));
    A_max        = (X(m)*(pi/2)*C_R)/(Cp_R(m)*(Cv_R(m)+C_R));
    a_vib_c(m)   = (A_max/c_snd_z)*((2*(power(freq,2))/f_vib(m))/(1+power(freq/f_vib(m),2)));
    a_vib        = a_vib + a_vib_c(m);
end

alpha = a_cl + a_rot + a_diff + a_vib;
