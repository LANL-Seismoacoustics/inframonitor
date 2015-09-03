function [staltan,td] = sta_lta(st,t,swl,lwl,K)
% function [staltan,td] = sta_lta(st,t,swl,lwl,K)
% sta_lta.m runs standard sta/lta detector and follows Withers etal. (1998)
% and Allen (1982) for frequency shift detector
% STA window concatenated on end of LTA window and for STA/LTA is end of STA window
% remove mean and applies taper prior to filtering for st
% sta_lta = st^2 + K*(d(st)/dt)^2
%
% ******* INPUT *******
% st - npts*1 vector with signal
% t - npts*1 vector with time corresponding to st
% swl - sta window length (s)
% lwl - lta window length (s)
% K - constant for frequency shift detector (e.g. K=3)
%     if K=0 detector is standard STA/LTA (default 0)
%
% ******* OUTPUT *******
% staltan - nwindows*1 vector with sta/lta + frequency shift detector
% td - times corresponding the ~isnan(stalta)

npts = length(t);
dt = t(2)-t(1);
s2 = st.^2;
ydot2 = diff(st).^2;
ydot2 = [ydot2;ydot2(end)];
ns = round(swl/dt);
nl = round(lwl/dt);
sta = zeros(npts,1);
lta = sta;
sta2 = zeros(npts,1);
lta2 = sta;
% initialize sta,lta
sta(nl+ns) = mean(s2(nl+1:nl+ns) + K*ydot2(nl+1:nl+ns));
lta(nl+ns) = mean(s2(1:nl) + K*ydot2(1:nl));

% set up matrix of values to use in calculation
rinx = nl+ns:ns:npts-ns;
rinx = rinx';
nssteps = 1:ns;
nlsteps = 1:nl;
stainx = repmat(rinx-ns, 1, ns) + repmat(nssteps, numel(rinx), 1);
ltainx = repmat(rinx-nl-ns, 1, nl) + repmat(nlsteps, numel(rinx), 1);

temp = mean(s2(stainx') + K*ydot2(stainx')); % mean will mean the columns
sta(rinx)= temp;
temp = mean(s2(ltainx') + K*ydot2(ltainx'));
lta(rinx) = temp;
sta(find(sta==0))=NaN;
lta(find(lta==0))=NaN;
stalta = sta./lta;
ik = find(~isnan(stalta));
td = t(ik);
staltan = stalta(ik);
