function [gsdev ] = zlta( y,t1,t2,f1,f2,dt,m )
%ZFILTER is a standard MATLAB Butterworth filter that has the same bandwidth 
%and order as the core adaptive Butterworth filter.  It calculates
%the root-mean-square of the filtered input between time ranges t1,t2 for
%an estimate of long term noise (LTA).
%
%INPUT:
%
%y:     input time series
%t1,t2: time range for computing long term average (LTA) standard 
%       deviation "gsdev" from input file
%dt:    sampling interval for input file
%m:     order of Butterworth filter (same as adaptive)
%sndif: vector filter numerator used for all SNRs (global input)
%snum0: vector filter numerator power for zero SNR (global input)        _
%sden0: vector filter dnominator for zero SNR (global input)
%
%OUTPUT:
%
%zs0:   filtered output data between t1,t2
%gsdev: LTA average (rms) between t1,t2
%
m2 = 2*m;
%nbuf sets up startup time point for Butterworth
nbuf = m2+1;
%i1,i2 calculate integer locations of desired bounds
i1=round(t1/dt+1);
i2=round(t2/dt+1);
%set up time series "x" for filter run
x=y(i1:i2);
n=length(x);
%for loop calculates filter
zs0=zbutter( f1,f2,m,dt,x );
%for loop calculates sum for rms LTA
sm = 0.0;
for i = nbuf:n
    sm = sm+zs0(i)^2;
end
gsdev = sqrt(sm/(n-nbuf+1));
end