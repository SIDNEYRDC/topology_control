%% Plot the minimum distance between neighbors nodes %%

%% CONFIGURE PLOT %%

% save data options
% 0: dont save
% 1: save .pdf
% 2: save .eps
SAVE_OPTIONS = 0;

% set plot width and height [pixels]
WIDTH = 300;
HEIGHT = 300 - 37.795275591;

%% MAIN CODE %%

% define minimum distances array
min_dist = ones(1, N)*1e+3;

% temporal series
% t = 0:h:(N - 1)*h;

% calculate the minimum distance for all iteractions
for k = 1 : N
    for i = 1 : n - 1
        for j = i + 1 : n
            if A_data(i, j, k) > 0 || A_data(j, i, k) > 0
                min_dist(k) = min(min_dist(k), norm(x_data(i, 1:2, k) - x_data(j, 1:2, k)));
            end
        end
    end
end

% plot the minimum distance over time
% for k = 1 : 10 : N
for k = [1, 100, 200, 300, 450, 600, 750, 900, 1150, N-1]
    
    hold on;
    
    k
    
    % temporal series
    t = 0:h:(k - 1)*h;
    
    % set axis limits
    plot(t, min_dist(1:k), 'Col' , 'k', 'LineWidth', 1);
    xlim([0, N*h]);
    ylim([1.95, 2.59]);
    
    xlabel('Time [s]');
    ylabel('Minimum Distance [m]');
    grid on;
    box on;
    
    % configure text font and size (use 'listfonts' to list all known fonts)
    set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman')
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', 9)
      
    % get picture frame
    frame = getframe(gcf);
    
    %% figure position [left, bottom, width, height]
    %set(fig, 'Units', 'Inches');
    %pos = get(fig, 'Position');
    %set(fig, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Inches', 'PaperSize', [pos(3), pos(4)]);
    
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
        imgname = strcat('tccm_exp3_mindist-z', int2str(k));
        print(gcf, '-dpdf', '-painters', imgname);

    elseif SAVE_OPTIONS == 2
        imgname = strcat('rssi-',int2str(n),'-',int2str(N),'.eps');
        print('-depsc2','-tiff',imgname);
    end

end

