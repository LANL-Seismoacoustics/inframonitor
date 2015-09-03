function F = bfstat(beam)
% Computes the F-statistic using the formulism of Blandford (1974)
% Usage: F = bfstat(beam)
% Stephen Arrowsmith (arrows@lanl.gov)

nchan = size(beam,2);

Num = sum((sum(beam').^2));
term1 = sum(beam')'./nchan;
for i = 1:nchan
    term1(:,i) = term1(:,1);
end
Den = sum(sum((beam-term1).^2'));

F = (nchan-1)*Num/(nchan*Den);
