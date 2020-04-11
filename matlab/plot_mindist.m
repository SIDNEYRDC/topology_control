%% Plot the minimum distance between neighbors nodes %%

%% CONFIGURE PLOT %%

% save data options
% 0: dont save
% 1: save .pdf
% 2: save .eps
SAVE_OPTIONS = 1;

%% MAIN CODE %%

% define minimum distances array
min_dist = ones(1, N)*1e+3;

% temporal series
t = 0:h:(N - 1)*h;

for k = 1 : 1 : N
    k
    for i = 1 : n - 1
        for j = i + 1 : n
            if A_data(i, j, k) > 0 || A_data(j, i, k) > 0
                min_dist(k) = min(min_dist(k), norm(x_data(i, 1:2, k) - x_data(j, 1:2, k)));
            end
        end
    end
end

% set axis limits
plot(t, min_dist, 'Col' , 'k', 'LineWidth', 1);
xlim([t(1) t(end)]);

xlabel('Time (s)');
ylabel('Minimum Distance (m)');
grid on;

% configure text font and size (use 'listfonts' to list all known fonts)
set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman')
set(findall(gcf, '-property', 'FontSize'), 'FontSize', 16)

pause(2)

% get current picture
fig = gcf;

% figure position [left, bottom, width, height]
set(fig, 'Units', 'Inches');
pos = get(fig, 'Position');
set(fig, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Inches', 'PaperSize', [pos(3), pos(4)]);

% save image plot
if SAVE_OPTIONS == 1
    imgname = strcat('mindist-', int2str(n), '-', int2str(N), '.pdf');
    print('-dpdf', '-r0', imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('mindist-', int2str(n), '-', int2str(N), '.eps');
    print('-depsc2', '-tiff', imgname);
end

