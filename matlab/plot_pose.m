%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot's Position Plotter
%
% Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
% Last Change: 2017 Fev 13 18:14:02
% Info: This code is able to plot the robot's positions from the topology
% control algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Runnable Options %%

% size of communication arrows
ARROW_LENGTH = 2;

% size of communications links
LINE_WIDTH = 1;

% save plot options
% 0: don't save
% 1: save .pdf
% 2: save .eps
% 3: save .avi
SAVE_OPTIONS = 0;

% sime for each iteration
LOOP_TIME = 0;

% show communication and coverage radii
% 0: off
% 1: show
COM_RAD = 0;
COV_RAD = 1;

% show velocity directions
% 0: off
% 1: show
VEL_DIR = 1;

%% Main Code %%

% disable warnings
warning('off', 'all');

% video writer
if SAVE_OPTIONS == 3
    mov = VideoWriter('sim');
    mov.Quality = 100;
    mov.FrameRate = 3;
    open(mov);
end

% set the slack to graphic limits
if COM_RAD == 1
    slack = single(max(r_com(:)));
elseif COV_RAD == 1
    slack = single(max(r_cov(:)));
else
    slack = single(max(r_cov(:))*0.5);
end

% insert a gain to slack
slack = slack*1.1;

xpose = x_data(:, 1, :);
ypose = x_data(:, 2, :);

% get the axis limits
xmin = min(xpose(:));
xmax = max(xpose(:));
ymin = min(ypose(:));
ymax = max(ypose(:));

% insert the slack to x limits
xmin = xmin - slack;
if xmax < 0
    xmax = xmax - slack;
elseif xmax >= 0
    xmax = xmax + slack;
end

% insert the slack to y limits
ymin = ymin - slack;
if ymax < 0
    ymax = ymax - slack;
elseif ymax >= 0
    ymax = ymax + slack;
end

% set the object sizes according with the plot size
ARROW_LENGTH = 0.04*norm([xmin ymin] - [xmax ymax]);
ROBOT_LENGTH = 0.06*norm([xmin ymin] - [xmax ymax]);
SPEED_LENGTH = 0.03*norm([xmin ymin] - [xmax ymax]);

% start position plotting for robots
for t = 1 : 1 : N
    newplot;
    hold on;

    % show line communication between neighbors agents
    for i = 1 : n - 1
        for j = i + 1 : n
            % get the position for each robot
            xi = x_data(i, 1 : 2, t);
            xj = x_data(j, 1 : 2, t);

            % euclidian distance between i and j
            dij = norm(xi - xj);

            if A_data(i, j, t) == 1 && S_data(i, j, t) >= rssi_lim && dij <= r_com(i, t) && dij <= r_com(j, t)
                % plot a continuous line when the link between i and j is real
                line([xi(1) xj(1)], [xi(2) xj(2)], 'LineStyle', '-', 'Linewidth', LINE_WIDTH, 'Color', 'b');

            elseif A_data(i, j, t) == 1 && S_data(j, i, t) >= rssi_lim && dij <= r_com(i, t) && dij > r_com(j, t)
                % plot a continuous arrow from i to j
                draw_line2([xi(1) xi(2)], [xj(1) xj(2)], 'LineStyle', '-', 'LineColor', 'b', 'ArrowColor', 'b', 'ArrowEdgeColor', 'b', 'ArrowLength', ARROW_LENGTH, 'LineWidth', LINE_WIDTH);

            elseif A_data(i, j, t) == 1 && S_data(i, j, t) >= rssi_lim && dij > r_com(i, t) && dij <= r_com(j, t)
                % plot a continuous arrow from j to i
                draw_line2([xj(1) xj(2)], [xi(1) xi(2)], 'LineStyle', '-', 'LineColor', 'b', 'ArrowColor', 'b', 'ArrowEdgeColor', 'b', 'ArrowLength', ARROW_LENGTH, 'LineWidth', LINE_WIDTH);

            elseif dij > r_com(i, t) && dij > r_com(j, t)
                if A_data(i, j, t) > 0 && A_data(j, i, t) > 0
                    % plot a dashed line when the link between i and j is virtual
                    line([xi(1) xj(1)], [xi(2) xj(2)], 'LineStyle', '--', 'Linewidth', LINE_WIDTH, 'Color', 'm');

                elseif A_data(i, j, t) > 0 && A_data(j, i, t) == 0
                    % plot a dashed arrow from j to i
                    draw_line2([xj(1) xj(2)], [xi(1) xi(2)], 'LineStyle', '--', 'LineColor', 'm', 'ArrowColor', 'm', 'ArrowEdgeColor', 'm', 'ArrowLength', ARROW_LENGTH, 'LineWidth', LINE_WIDTH);

                elseif A_data(i, j, t) == 0 && A_data(j, i, t) > 0
                    % plot a dashed arrow from i to j
                    draw_line2([xi(1) xi(2)], [xj(1) xj(2)], 'LineStyle', '--', 'LineColor', 'm', 'ArrowColor', 'm', 'ArrowEdgeColor', 'm', 'ArrowLength', ARROW_LENGTH, 'LineWidth', LINE_WIDTH);

                end
            end
        end
    end

    for i = 1 : n
        % get the position for each robot i
        xi = x_data(i, 1 : 2, t);

        % show velocities directions
        if VEL_DIR == 1
            if t < N
                % get the velocity unity vector
                xi1 = xi + ROBOT_LENGTH*(x_data(i, 1 : 2, t + 1) - xi)/norm(x_data(i, 1 : 2, t + 1) - xi);

                % plot the velocity unity vector arrow
                if not(isnan(xi1))
                    draw_line2(xi, xi1, 'ArrowLength', SPEED_LENGTH, 'ArrowAngle', 20, 'ArrowEdgeColor', 'r', 'LineColor', 'r', 'LineWidth', 2.5);
                end
            end
        end

        % show communication range
        if COM_RAD == 1
            uistack(viscircles(xi, r_com(i, t), 'EdgeColor', 'k', 'LineWidth', 0.1, 'LineStyle', '-.'), 'bottom');
        end

        % show coverage range
        if COV_RAD == 1
            uistack(viscircles(xi, r_cov(i, t), 'EdgeColor', 'k', 'LineWidth', 0.1, 'LineStyle', '-.'), 'bottom');
        end

        % plot the agent's path
        xpose = x_data(i, 1, 1 : t);
        ypose = x_data(i, 2, 1 : t);
        plot(xpose(:), ypose(:), 'g-');

        % plot the robot's name and shape
        draw_robot(xi, x_data(i, 3, t), ROBOT_LENGTH, 'RobotLabel', num2str(i), 'FillColor', 'w');

    end

    % configure plot properties
    set(gca, 'FontName', 'Helvetica');
    xlabel('x [m]');
    ylabel('y [m]');

    % set axis configurations
    axis([xmin xmax ymin ymax]);
    axis equal;
    axis manual;

    % enable box around the figure
    box on;

    % get picture frame
    frame = getframe(gcf);

    % write frame in video file
    if SAVE_OPTIONS == 3
        writeVideo(mov, frame);
    end

    % enable picture update
    hold off;

    % save file options
    if SAVE_OPTIONS == 1
        imgname = strcat('pose-', int2str(t), '.pdf');
        print('-dpdf', imgname);
    elseif SAVE_OPTIONS == 2
        imgname = strcat('pose-', int2str(t), '.eps');
        print('-depsc2', '-tiff', imgname);
    end

    % print time iteration
    t

    % wait for LOOP_TIME
    pause(LOOP_TIME)
end

% close movie file
if SAVE_OPTIONS == 3
    close(mov);
end

