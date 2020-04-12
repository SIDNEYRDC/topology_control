%% Plot control actions of the robots in the time %%

%% CONFIGURE PLOT %%

% save data options
% 0: dont save
% 1: save .pdf
% 2: save .eps
SAVE_OPTIONS = 0;

% plot line width
LINE_WIDTH = 1;

% plot markers size (use 0 to off)
MARKER_SIZE = 0;

%% MAIN CODE %%

% array of colors
colors = hsv(double(n));

% array of markers
markers = ['^', 'd', '*', 'o', 'x', 's', '.', '^', 'v', '>', '<', 'p', 'h'];

% label names
names = '';

% convert N from int64 to float
N = double(N)

% clear picture labels
clear botpl;

for i = 1 : n
    % plot the x coordinates for all robots
    subplot(2, 1, 1);
    hold on;

    vx_plot = v_data(i, 1, :);

    if MARKER_SIZE == 0
        botpl(i) = plot(h:h:N*h+h, vx_plot(:), 'Col', colors(i, :), 'LineWidth', LINE_WIDTH);
    else
        botpl(i) = plot(h:h:N*h+h, vx_plot(:), 'Marker', markers(i), 'MarkerSize', MARKER_SIZE, 'LineWidth', LINE_WIDTH);
    end

    xlabel('t [s]');
    ylabel('u_x [m/s]');
    xlim([h N*h]);
    ylim([vmin*1.2 vmax*1.2]);

    names{i} = strcat('(',int2str(i),')');

    if i == n
        % plot reference coordinate at x axis
        %refpl = plot(h:h:N*h+h,repmat(x(n+1,1,1),1,size(x,3)),'Col','k','LineWidth',1,'LineStyle','--');
        vx_ref = v_data(n+1, 1, :);
        refpl = plot(h:h:N*h+h, vx_ref(:), 'Col', 'k', 'LineWidth', 1, 'LineStyle', '--');

        % plot labels
        names{n+1} = 'r';
        legend([botpl,refpl],names{:},'Location','southeast');
        grid on;
        box on;
    end

    % plot the y coordinates for all robots
    subplot(2,1,2)
    hold on;

    vy_plot = v_data(i, 2, :);

    if MARKER_SIZE == 0
        plot(h:h:N*h+h, vy_plot(:), 'Col', colors(i,:), 'LineWidth', LINE_WIDTH);
    else
        plot(h:h:N*h+h, vy_plot(:), 'Marker', markers(i), 'MarkerSize', MARKER_SIZE, 'LineWidth', LINE_WIDTH);
    end

    % plot reference coordinate at y axis
    if i == n
        %plot(h:h:N*h+h,repmat(x(n+1,2,1),1,size(x,3)),'Col','k','LineWidth',1,'LineStyle','--');
        vy_ref = v_data(n+1, 2, :);
        refpl = plot(h:h:N*h+h, vy_ref(:), 'Col', 'k', 'LineWidth', 1, 'LineStyle', '--');
    end

    xlabel('t [s]');
    ylabel('u_y [m/s]');
    xlim([h N*h]);
    ylim([vmin*1.2 vmax*1.2]);
    grid on;
    box on;
end

% save image plot
if SAVE_OPTIONS == 1
    imgname = strcat('controltime-',int2str(n),'-',int2str(N),'.pdf');
    print('-dpdf',imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('controltime-',int2str(n),'-',int2str(N),'.eps');
    print('-depsc2','-tiff',imgname);
end

