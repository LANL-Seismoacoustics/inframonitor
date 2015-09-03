function [ z ] = zbutter( f1,f2,m,dt,y )
%ZBUTTER runs internal MATLAB butterworth bandpass filter using MATLAB
%Signal Processing Toolkit
%
%Initialize filter using internal MATLAB function "butter". "fnyq" is Nyquist
%frequency- this internal program normalizes Nyquist to 1.
fnyq=1/(2*dt);
wn=[f1 f2]/fnyq;
[b,a]=butter(m,wn,'bandpass');
%Call MATLAB function "filter" using zeros and poles calculated in "butter"
z=filter(b,a,y);
end
