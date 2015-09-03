function [snv,snvec,sdvec,sndif,zfir,del,dsl,nfr] = zinit( f1,f2,dt,m,nx2,smin,smax,firmul)
%ZINIT initializes adaptive filter by calculating matrix and vector
%of filter coefficients for range of different signal-to-noise values
%
%INPUT:
%
%f1,f2:     bandwidth range for core Butterworth in adaptive filter
%dt:        data sampling interval
%m:         order of core Butterworth
%nx2:       total number of signal-to-noise values used (must be power of 2)
%smin,smax: minimum and maximum signal-to-noise (SNR)
%firmul:    multiplier for FIR filter length

%OUTPUT: 
%
%snv:   vector of SNRs with logarithmic spacing between smin,smax (nx2)
%snvec: vector filter numerator power as function of SNR (nx2)
%sdvec: matrix filter denominator as function of SNR (m2 X nx2) 
%sndif: vector numerator of filter standard for all SNRs (m2+1)
%zfir:  matrix of FIR filter coefficients as function of SNR (nfr X nx2)
%del:   vector of exponential decay terms as function of SNR (nx2)
%dsl:   scalar logarithmic increment of SNR
%nfr:   FIR filter length
%
%Calculate SNR vector "snv"
[ snv,dsl ] = svec(smin,smax,nx2);
%Set up adaptive filter coefficients as function of SNR
[ snvec,sdvec,sndif,zfir,nfr ] = cval(snv,f1,f2,nx2,dt,m,firmul);
%Calculate vector of exponential decay terms as function of SNR 
del = zfreq(f1,f2,snv,dt,m,nx2);
end

function [ snv,dsl ] = svec( smin,smax,nx2)
%SVEC calculates vector of log base 10 distributed (spaced) SNR's
%
%INPUT:
%
%smin,smax: minimum, maximum SNR
%nx2:       total number of SNRs
%
%OUTPUT:
%
%snv: vector of log base 10 distributed SNRs
%dsl: scalar log increment of snv
%
a = log(10);
smin10 = log10(smin);
smax10 = log10(smax);
dsl = (smax10-smin10)/(nx2-1);
snv = smin10:dsl:smax10;
snv=exp(a*snv);
end

function [ sndif,snum,sden ] = setbut(f1,f2,sn,dt,m)
%SETBUT: core function for calculating filter coefficients for each SNR
%
%INPUT:
%
%f1,f2: bandpass range for core Butterworth
%sn:    scalar SNR to calculate filter coefficients
%dt:    data sampling interval
%m:     order of core Butterworth
%
%OUTPUT:
%
%sndif: vector filter numerator used for all SNRs
%snum: vector numerator powers for input SNR "sn"
%sden: vector filter denominator for input SNR "sn"
%
m2 = 2*m;
%Initialize vectors
c = zeros(m2,1);
sndif = zeros(m2+1,1);
snum = zeros(2,1);
%Calculate bilinear pre-warped frequencies
w1 = 2.0/dt*tan(pi*f1*dt);
w2 = 2.0/dt*tan(pi*f2*dt);
wc = (w2-w1)/2.0;
%This "for" loop is basically setting up z-transform coefficients for a
%standard Butterworth filter, given "sn" =0. For positive values of "sn",
%expands the radius of the circle on which complex Butterworth poles are
%calculated, resulting in coefficients for adaptive filter as function of
%SNR. "smul" is numerator power as function of "sn"
smul = 1.0;
k = 1;
for j = 1:m
    p = exp(1i*pi*(2*j+m-1)/(2.0*m));
    t1 = p*wc*(sn^2 +1.0)^(1.0/(2.0*m));
    t2 = sqrt(t1^2-w1*w2);
    s1 = t1+t2;
    s2 = t1-t2;
    b1 = (2.0*wc*dt)/(2.0-s1*dt);
    b2 = 2.0/(2.0-s2*dt);
    c(k) = -(2.0+s1*dt)/(2.0-s1*dt);
    c(k+1) = -(2.0+s2*dt)/(2.0-s2*dt);
    k = k+2;
    smul = smul*b1*b2;
end
%above "for" loop calculates pole locations; "cexpand" below multiplies out
%poles for a z-polynomial in denominator of filter
sden = cexpnd(c,m2);
%"for" loop below calculates numerator filter coefficients used for any SNR
tmp = 1.0;
k=1;
sndif(k)=1.0;
for j = 1:m
    k=k+2;
    tmp = tmp*(m-j+1)/j;
    sndif(k) = (-1)^j*tmp;
end
%set up output vectors. "snum(2)" without SNR scaling (not used now)."snum"
%and "sdev" set to real values - should be real, but MATLAB calculates very
%small imaginary values due to complex calculation above
snum(1) = smul*sn;
snum(2) = smul;
snum=real(snum);
sden=real(sden);
end

function [ cout ] = cexpnd(c,m2)
%CEXPAND multiplies out poles of Butterworth to form denominator 
%z-polynomial
%
%INPUT:
%
%c:  pole calculations from "setbut"
%m2: total number of denominator poles (2 times Butterworth filter order)
%
%OUTPUT:
%
%cout: vector of expanded denominator z-polynomial filter coefficients
%
cout = zeros(m2,1);
for j = 1:m2
    [ ja,krow ] = combo(m2,j);
    tmpr = 0.0;
    for kr = 1:krow
        tmpc = 1.0;
        for kc = 1:j
            indx = ja(kr,kc);
            tmpc = tmpc*c(indx);
        end
        tmpr = tmpr+tmpc;
    end
    cout(j) = tmpr;
end
end

function [ ja,knt ] = combo(n,r)
%COMBO calculates out combinatorial factors used in multiplying out poles
%in "cexpand"
%
%INPUT:
%
%n,r: combination "n" taken "r" at a time
%
%OUTPUT:
%
%ja: coefficients of product factors in expansion of (x+a)*(x+b)*... for
%each multiple of product factors "r" in expansion
%
jt = zeros(n,1);
ja = zeros(70,n);
knt = 0;
nmr = n-r;
i = 1;
jt(1) = 1;
while i>0
    if i ~= r
        ip1 = i+1;
        for k = ip1:r
            jt(k) = jt(k-1)+1;
        end
    end
    knt = knt+1;
    for k = 1:r
        ja(knt,k) = jt(k);
    end
    i = r;
    while jt(i) >= nmr+i
        i = i-1;
        if i <= 0
            break
        end
    end
    if i > 0
        jt(i) = jt(i)+1;
    end
end
end

function [ snvec,sdvec,sndif,zfir,nfr ] = cval( snv,f1,f2,nx2,dt,m,firmul)
%CVAL combines numerator, denominator vectors as function of "sn" into
%matrices for use in "zsignal"
%
%INPUT:
%
%snv:    vector of SNRs
%f1,f2:  bandpass range for core Butterworth
%nx2:    total number of SNRs
%dt:     data sampling interval
%m:      order of core Butterworth filter
%firmul: FIR filter multiplier length (default 1)
%
%OUTPUT:
%
%snvec: vector filter numerator power as function of SNR
%sdvec: matrix filter denominator as function of SNR
%sndif: vector filter numerator used for all SNRs
%zfir:  matrix of FIR filter coefficients as function of SNR
%nfr:   FIR filter length
%
%initialize data: zimp is impulse vector, and alpha is Tukey window
%parameter for window rolloff
m2 = 2*m;
snvec = zeros(nx2,1);
sdvec = zeros(m2,nx2);
nfr=round(firmul*m/dt);
zfir=zeros(nfr,nx2);
zimp=zeros(nfr,1);
zimp(1)=1.0;
alpha=0.5;
%"for" loop calls "setbut" for each SNR and puts calculated filter
%coefficients into "snvec", "sdvec", and "zfir"
for j = 1:nx2
    sn = snv(j);
    [ sndif,snum,sden ] = setbut(f1,f2,sn,dt,m);
    snvec(j) = snum(1);
%filter denominator for SNR index "j" put in "sdvec".  Change sign and
%reverse for simple dot product IIR filter execution in "zsignal.m"
    for i = 1:m2
        sdvec(i,j) = -sden(m2-i+1);
    end
%calculate impulse response for FIR filter
    a=[1.0; sden];
    b=sndif*snum(1);
    ztmp=filter(b,a,zimp);
%calculate Tukey window to taper impulse response and then multiply by the
%impulse response "ztmp"
    istrt=round((1-alpha)*nfr+1);
    k=istrt;
    for i=istrt:nfr
        xk=(i-1)/nfr;
        w=0.5*(1+cos(pi/alpha*(xk-1+alpha)));
        ztmp(k)=ztmp(k)*w;
        k=k+1;
    end
%reverse impulse response for FIR filter coefficients at SNR index "j"
%reversal done for simple dot product FIR filter execution in "zsignal.m"
    for i=1:nfr
        zfir(i,j)=ztmp(nfr-i+1);
    end
end
end

function [ del ] = zfreq( flo,fhi,snv,dt,m,nx2 )
%ZFREQ uses bilinear pre-warping to give correct calculation of corner
%frequencies of adaptive filter as function of SNR. Used to set up
%exponential decay rate "del", used in "zsignal"
%
%INPUT:
%
%flo,fhi: bandpass range in core Butterworth
%snv:     vector of calculated SNRs
%dt:      data sampling rate
%m:       order of core Butterworth
%nx2:     total number of SNRs in "snv"
%
%OUTPUT:
%
%del:   vector of exponential decay terms as function of SNR
%
del = zeros(nx2,1);
m2 = 2*m;
%pre-warp analog frequencies to digital
pidt = pi*dt;
f1 = tan(pidt*flo)/(pidt);
f2 = tan(pidt*fhi)/(pidt);
%"for" loop calculates "fz1","fz2" bandpass frequency corners of adaptive
%filter as function of SNR
for i = 1:nx2
    z = (1.0+snv(i)^2)^(1.0/m2)*(f2-f1);
    tmp = sqrt(z^2+4.*f1*f2);
    fz1 = (tmp-z)/2.0;
    fz2 = (tmp+z)/2.0;
    fz1 = atan(pidt*fz1)/(pidt);
%"fz2" not used here  but calculated anyway for future use with STA/LTA
    fz2 = atan(pidt*fz2)/(pidt);
%"tlo" is time domain period corresponding to filter highpass corner "fz1".
%"del" uses "tlo" to determine exponential decay rate.  Notice "tlo" will
%not exceed 1 second here; this is empirical cutoff due to larger value not
%being necessary for adaptive filter stability
    tlo = 1.0/(2.0*fz1);
    if tlo > 2.0
        tlo = 2.0;
    end
    del(i) = 0.5^(dt/tlo);
end
end