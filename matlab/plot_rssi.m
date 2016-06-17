%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RSSI Readings plotter
%
% Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
% Last Change: 2016 Jun 14 22:07:17
% Info: This code is able to plot the RSSI readings from the Communication
% Topology Control Algorithm.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CONFIGURE PLOT %%

% save data options
% 0: dont save
% 1: save .pdf
% 2: save .eps
SAVE_OPTIONS = 1;

% select the kind of plot
% 0: x = log(dij)   y = RSSI
% 1: x = time       y = RSSI
PLOT_TYPE = 0;

%% MAIN CODE %%

% array of colors
colors = hsv(single(n*n));

hold on

N = 350;

s = S_data(1, 3, 1:N);

%h1 = plot(1:N, s(:), '-r');
h2 = plot(1:N, s(:), '-b');

h3 = plot(1:N, repmat(s_min, 1, length(1:N)), 'Col', 'k', 'LineWidth', 1, 'LineStyle', '--');
%h4 = plot(repmat(80, 1, length(-26:-13)), -26:-13, '-.k');
%h5 = plot(repmat(250, 1, length(-26:-13)), -26:-13, '-.k');

legend([h1, h2, h3], {'no-RSSI sensing', 'RSSI sensing', 'RSSI threshold'}, 'Location', 'southeast');

% set axis limits
xlim([1 N]);
%ylim([-26 -13])

xlabel('iterations');
ylabel('RSSI values to link (1,3) [dBm]')
grid on;
box on;

% save image plot
if SAVE_OPTIONS == 1
    imgname = strcat('rssi-',int2str(n),'-',int2str(N),'.pdf');
    print('-dpdf',imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('rssi-',int2str(n),'-',int2str(N),'.eps');
    print('-depsc2','-tiff',imgname);
end

