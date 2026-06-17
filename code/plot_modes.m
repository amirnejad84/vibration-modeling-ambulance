function plot_modes(Phi,freqs)
% Plot the 5 mode shapes of the half-car model.
nModes = size(Phi,2);
figure('Name','Mode shapes');
for i=1:nModes
    subplot(ceil(nModes/2),2,i);
    plot(Phi(:,i),'o-','LineWidth',1.2);
    xticks(1:5);
    xticklabels({'x_f','x_r','x_c','theta','x_s'});
    title(sprintf('Mode %d: f=%.3f Hz',i,freqs(i)));
    grid on;
end
end
