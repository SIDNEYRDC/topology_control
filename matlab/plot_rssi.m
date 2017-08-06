%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RSSI Readings plotter
%
% Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
% Last Change: 2017 Jul 25 13:58:13
% Info: This code is able to plot the RSSI readings from the Communication
% Topology Control Algorithm.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CONFIGURE PLOT %%

% save data options
% 0: dont save
% 1: save .pdf
% 2: save .eps
SAVE_OPTIONS = 1;

%% MAIN CODE %%

N = single(N);

for k = [30 90 260 350]

    hold on;

    s = S_data(1, 2, 1:k);
    sf = Sf_data(1, 2, 1:k);

    t = 0:h:(k - 1)*h;

    h1 = plot(t, s(:), 'Col', 'k', 'LineWidth', 1, 'LineStyle', '-');
    %h2 = plot(t, sf(:), 'Col', 'k', 'LineWidth', 1, 'LineStyle', '-');
    h5 = plot(0:h:(N - 1)*h, repmat(rssi_lim, 1, length(1:N)), '--k');

    if k == 10
        %legend([h1, h2, h5], {'RSSI raw', 'RSSI filtered', 'RSSI threshold'}, 'Location', 'southeast');
    end

    % configure text font and size (use 'listfonts' to list all known fonts)
    set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman')
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', 16)

    % plot options
    xlabel('Time (s)');
    ylabel('RSSI values (dBm)')
    xlim([0 35])
    ylim([-26 -12])
    grid on;
    box on;

    % save image plot
    if SAVE_OPTIONS == 1
        % set print size
        fig = gcf;
        fig.PaperSize = [4 3.3];

        imgname = strcat('test2_rssi-', int2str(k), '.pdf');
        print(fig, '-dpdf', '-fillpage', imgname);
    elseif SAVE_OPTIONS == 2
        imgname = strcat('rssi-',int2str(n),'-',int2str(N),'.eps');
        print('-depsc2','-tiff',imgname);
    end

    pause(1)

    %clf

end

