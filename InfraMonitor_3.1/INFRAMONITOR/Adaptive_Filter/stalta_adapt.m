sta = [0.05 0.1 0.2 0.3 0.4 0.5];
lta = [30];

for i = 1:numel(sta)
    for j = 1:numel(lta)
        
        [staltan,td] = sta_lta(zb,t,sta(i),lta(j),0);
        figure
        ax(1) = subplot(2,1,1);
        plot(t,zb)
        title(['sta = ' num2str(sta(i)) ', lta = ' num2str(lta(j))])
        ax(2) = subplot(2,1,2);
        plot(td,staltan)
        ylabel('STA/LTA')
        xlabel('Time (s)')
        linkaxes(ax,'x')
        %xlim([0 3600])
        
    end
end
