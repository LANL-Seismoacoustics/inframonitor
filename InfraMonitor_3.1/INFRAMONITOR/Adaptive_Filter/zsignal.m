function [zs,jknr] = zsignal( y,gsdev,del,dsl,smin,snv,sndif,snvec,sdvec,zfir,nfr,m,n,nx,nx2)
%ZSIGNAL calculates adaptive filter, based on initialization in "zinit" and
%LTA rms average "gsdev".  Method uses a binary search for optimum SNR
%based on ranges determined previously from smin,smax and number of
%possible SNRs pre-calculated in vector "snv". Final optimized filter is
%realized as a Finite Impulse Response (FIR), in order to stabilize long
%term filter memory as a result of dynamically changing filter coefficients
%
%INPUT:
%
%y:     input time series
%gsdev: LTA average (rms)
%del:   vector of exponential decay terms as function of SNR
%dsl:   scalar logarithmic increment of SNR in "snv"
%smin:  minimum SNR
%snv:   vector of SNRs with logarithmic spacing between smin,smax
%sndif: vector filter numerator used for all SNRs
%snvec: vector filter numerator power as function of SNR
%sdvec: matrix filter denominator as function of SNR
%zfir:  matrix of FIR filter coefficients as function of SNR
%nfr:   FIR filter length
%m:     order of core Butterworth
%n:     number of points in y
%nx:    Power of 2 used for binary search routine - equivalent to number 
%       of searches at each time point
%nx2:   Total number of signal-to-noise (SNR) values used in search
%
%OUTPUT:
%
%zs:   output vector of adaptive filter from FIR calculation
%jknr: integer vector of indexes for optimum SNRs in "snv" as a function of 
%      time (corresponds element wise to output signal vector zs). Not 
%      currently used but may be needed for onset times and p-value 
%      calculations
%
%"for" loop below is heart of filter. It runs over entire input data, and
%calls a binary search, to determine optimum SNR integer index
%corresponding to "snv" vector at each time point "i".  It also checks for
%exponential damping of SNRs in order to stabilize adaptive filter - also
%this speeds up routine significantly since it allows bypassing the full
%search if signal SNR is less than that allowed by damping
%
%initialize data
jknr = ones(n,1);
zs = zeros(n,1);
m2=2*m;
jkn =1;
jdmp = jkn;
sdmp = smin;
delx = del(1);
nbuf = nfr+1;
for i = nbuf:n
%calculate IIR search filter numerator (moving average portion)
    znum = y(i);
    j2=0;
    for j=1:m
        j2=j2+2;
        znum=znum+sndif(j2+1)*y(i-j2);
    end
%determine prior values of calculated filter to be used in search
    ztmp=zs(i-m2:i-1);
%Test to see if signal is less than exponential damped prediction
%"tmp" is interim calculated IIR filter signal
    tmp = sdvec(:,jdmp)'*ztmp+snvec(jdmp)*znum;
    f1 = abs(tmp)-gsdev*snv(jdmp);
    if f1 <= 0.0
        jkn = jdmp;
%otherwise, do binary search to optimize SNR index "jkn"
    else
        ndx = nx2;
        jkn = ndx;
        for j = 1:nx
            ndx = ndx/2;
            jmid = jkn - ndx;
            tmp = sdvec(:,jmid)'*ztmp+snvec(jmid)*znum;
            f1 = abs(tmp)-gsdev*snv(jmid);
            if f1 <= 0.0        
                jkn = jmid;
            end
        end
        if jkn<jdmp
            jkn=jdmp;
        end
    end
%when search is done (or damped SNR picked) update output signal vector as
%an equivalent FIR filter - this ensures no memory loss which would occur 
%if IIR filter was used
    zs(i) = zfir(:,jkn)'*y(i-nfr+1:i);
%check for exponential damping; reset damping parameters if damped index
%less than optimum index
    if jdmp < jkn
        sdmp = snv(jkn);
        delx = del(jkn);
    end
%store optimum SNR index for time point "i" ("jknr" not currently used)
    jknr(i) = jkn;
%calculate damped SNR for next time point
    sdmp = delx*sdmp;
%determine index "jdmp" corresponding to damped SNR "sdmp" 
    jdmp = round(1.0+log10(sdmp/smin)/dsl);
%"jdmp" cannot be less than 1 since it is a vector index
    if jdmp < 1
        jdmp =1;
    end
end
end
