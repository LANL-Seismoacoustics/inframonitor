function [PBAZ] = p_baz(zen,tzen,wl,swl,dbaz)
% p_baz.m uses grid search to find back azimuth
% using a moving window
% initially just set up for P-waves
% maximize variance for radial component to get P wave backazimuth
%
% ******* INPUT *******
% zen - (n*3) z,e,n time series to be used
%       e and n are like x and y respectively
% tzen - vector with time corresponding to zen
% wl - window length (s)
% swl - shift window length (s)
% dbaz - sampling (deg) in backazimuth to be used in grid search
%        grid search is done between 0 and 180 degrees
%
% ******* OUTPUT *******
% PBAZ structure with following elements
% baz_obs - vector with observed back azimuth for each window
% varmx - vector with max radial variance
% varmxn - normalized varmx by tangential => degree of polarization
% rect - rectilinearity computed from normalized varmxn
% tk - sampled time series shifted by wl at end
%      tk gets shifted forward by mswl*swl to catch beginning of each window

varm = @(x) (sum(x.*x) - length(x)*mean(x).^2)/(length(x)-1);
mswl = 1;
dt = tzen(2)-tzen(1);
npts = length(tzen);
nw = round(wl/dt);
ns = round(swl/dt);
N = fix((npts+ns-nw)/ns)-1;
wi = repmat([1:nw+1]',1,N) + repmat([0:ns:ns*(N-1)],nw+1,1);
tk = tzen(wi(1,1:end)+nw);
nwi = size(wi,1);
bazdi = 0:dbaz:180;
nbaz = length(bazdi);
baz_obs=zeros(N,1);varmx=zeros(N,1);varmxn=zeros(N,1);rect=zeros(N,1);varmn=zeros(N,1);
for i = 1:N
   zenwi = zen(wi(:,i),:);
   thetai = pi/2 - deg2rad(bazdi);
   rcos = repmat(cos(thetai),nwi,1);
   rsin = repmat(sin(thetai),nwi,1);
   rzen2 = repmat(zenwi(:,2),1,nbaz);
   rzen3 = repmat(zenwi(:,3),1,nbaz);
   rr = rcos.*rzen2 + rsin.*rzen3;
   var_rr = varm(rr);
   [varmx(i),ibmp] = max(var_rr);
   varmn(i) = min(var_rr);
   varmxn(i) = varmx(i)/varmn(i);
   rect(i) = 1-varmn(i)/varmx(i);
   baz_obs(i) = bazdi(ibmp);
end
tk = tk - mswl*swl;
PBAZ.tk = tk;
PBAZ.baz_obs = baz_obs;
PBAZ.rect = rect;
PBAZ.varmx = varmx;
PBAZ.varmn = varmn;
PBAZ.varmxn = varmxn;
