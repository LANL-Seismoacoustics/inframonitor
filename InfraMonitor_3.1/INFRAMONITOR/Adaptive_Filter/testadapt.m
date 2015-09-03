function testadapt
% Testadapt is a test program for the Butterworth adaptive filter
% "zsignal.m", utilizing SAC format IO.  Program also calculates an internal
% MATLAB Butterworth bandpass filter "zbutter.m" for comparison purposes.  
% 
% Written by David R. Russell, 17 April 2012.
% 
% INPUT:
% 
% y:       Input time series
% dt:      Sampling interval for input file 
% f10,f20: Bandpass range (hz) for MATLAB Butterworth bandpass 
% f1,f2:   Bandpass range (hz) for core adaptive bandpass filter 
% t1,t2:   Time range for computing long term average (LTA) standard 
%          deviation "gsdev" from input file
%
%Ask User to Read in SAC file and then run
clear global t y zs zb
global t y zs zb
[filename, pathname] = uigetfile('*.SAC', 'Pick a SAC data file');
y2=rsac(char(filename));
y=y2(:,2);
n=length(y);
dt=lh(y2,'DELTA');
%Option to remove mean from input data -comment out if not desired
ymean=mean(y);
y=y-ymean;
% Ask User to Input Parameters
prompt={'Butterworth BP Low (Hz)','Butterworth BP High (Hz)','Adaptive BP Low (Hz)','Adaptive BP High (Hz)','LTA Start (sec)','LTA Stop (sec)'};
name='Filter Parameters';
numlines=1;
defaultanswer={'2','5','2','5','100','200'};
answer=inputdlg(prompt,name,numlines,defaultanswer);
f10=str2double(answer(1));
f20=str2double(answer(2));
f1=str2double(answer(3));
f2=str2double(answer(4));
t1=str2double(answer(5));
t2=str2double(answer(6));
%
% Default parameters for running "zsignal.m"
%
% nx:      Power of 2 used for binary search routine in "zsignal.m"
%          Equivalent to number of searches at each time point
% nx2:     Total number of signal-to-noise (SNR) values used in search
% smin:    Floor for SNR: smaller value will decrease minimum background
%          noise level calculated by filter
% smax:    Ceiling for SNR:  smax=1000 essentially all pass.
% sdevmul: Multiplier for LTA "gsdev". Setting larger than 1 (eg 1.5) will
%          decrease sensitivity of filter, reducing false alarm rate but missing
%          smaller signals
% firmul:  Multiplier for FIR filter length. FIR filter length is set as
%          firmul*m/dt.  Can use to shorten or lengthen FIR filter as
%          appropriate.
% m:       Adaptive filter order **NOTE** recommend leaving at m=2 to 
%          optimize onset time and minimize group delay
m=2;
nx = 7;
nx2 = 2^nx;
smin = 0.2;
smax = 1000.;
sdevmul=1;
firmul=1;
% Set Matlab Butterworth comparison filter order (default m0=2)
m0=2;
% Call internal MATLAB Butterworth bandpass; only used for comparison here
zb=zbutter(f10,f20,m0,dt,y );
% Call adaptive filter initialization routine "zinit"
[snv,snvec,sdvec,sndif,zfir,del,dsl,nfr] = zinit( f1,f2,dt,m,nx2,smin,smax,firmul);
% Calculate standard deviation "gsdev" for LTA
[gsdev] =  zlta( y,t1,t2,f1,f2,dt,m );
gsdev=sdevmul*gsdev;
% Calculate adaptive filter; use "tic-toc" to time filter execution
tic
[zs,jknr] = zsignal(y,gsdev,del,dsl,smin,snv,sndif,snvec,sdvec,zfir,nfr,m,n,nx,nx2);
toc
% "jknr" is integer index of SNR values.  "sout" calculated here to show
% instantaneous SNR estimates as function of time
sout=zeros(n,1);
for i=1:n
    k=jknr(i);
    sout(i)=snv(k);
end
% Set up simple plot with output raw data "y" (blue), adaptive filter "zs" 
% (red) and standard Butterworth "zb" (green)
t=0:dt:(n-1)*dt;
plot(t,y,'b',t,zs,'r',t,zb,'g');
end

