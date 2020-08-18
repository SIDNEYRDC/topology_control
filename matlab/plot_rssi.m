%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RSSI Readings plotter
%
% Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
% Last Change: 2020 Ago 18 01:16:04
% Info: This code is able to plot the RSSI readings from the Communication
% Topology Control Algorithm.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CONFIGURE PLOT %%

% save data options
% 0: dont save
% 1: save .pdf
% 2: save .eps
SAVE_OPTIONS = 0;

% set plot width and height [pixels]
WIDTH = 268.346456693 + 24.94488189;
HEIGHT = 241.88976378 + 13.606299213;

%% MAIN CODE %%

N = single(N);

for k = [1, 20, 50, 60, 100, 130, 180, 190, 230, N-1]
%for k = 1 : N

    hold on;

    s = S_data(1, 4, 1:k);
    %sf = Sf_data(1, 2, 1:k);

    t = 0:h:(k - 1)*h;

    h1 = plot(t, s(:), 'Col', 'k', 'LineWidth', 1, 'LineStyle', '-');
    %h2 = plot(t, sf(:), 'Col', 'k', 'LineWidth', 1, 'LineStyle', '-');
    h5 = plot(0:h:(N - 1)*h, repmat(rssi_lim, 1, N), '--k');

    if k == 10
        %legend([h1, h2, h5], {'RSSI raw', 'RSSI filtered', 'RSSI threshold'}, 'Location', 'southeast');
    end

    % configure text font and size (use 'listfonts' to list all known fonts)
    set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman')
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', 9)

    % plot options
    xlabel('Time [s]');
    ylabel('RSSI values [dBm]')
    xlim([0 N*h])
    ylim([-26 -5])
    grid on;
    box on;

    % get picture frame
    frame = getframe(gcf);

    % save image plot
    if SAVE_OPTIONS == 1
        % set frame resolution and paper size
        set(gcf, 'MenuBar', 'none', ...
                 'Units', 'pixels', ...
                 'PaperUnits', 'centimeters', ...
                 'Resize', 'off', ...
                 'Position', [0, 0, WIDTH, HEIGHT], ...
                 'PaperSize', [0.026458*WIDTH, 0.026458*HEIGHT], ...
                 'PaperPosition', [0, 0, 0.026458*WIDTH, 0.026458*HEIGHT]);

        % wait for the correct size setting
        pause(1);

        % save as pdf
        imgname = strcat('tccm_exp2_rssi-', int2str(k));
        print(gcf, '-dpdf', '-painters', imgname);

    elseif SAVE_OPTIONS == 2
        imgname = strcat('rssi-',int2str(n),'-',int2str(N),'.eps');
        print('-depsc2','-tiff',imgname);
    end

    %clf

end

