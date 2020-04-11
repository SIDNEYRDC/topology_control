%% Plot the states variation for the robots %%

% Number of robots
n = size(x_plot,2);

% Number of iterations
N = size(x_plot,1);

% Array of colors
colors = {'b','g','r','c','m','k','y','b','g','r','c','m','y','k','b','g','r','c','m','y','k'};

% Auxiliar variables
count = 1;
names = '';

for bot = 2 : 2 : n
    % Plot the x coordinates for all robots
    subplot(2,1,1);
    hold on;
    plot(x_plot(:,bot-1),colors{count},'LineWidth',2);
    xlabel('iterations'); 
    ylabel('x [m]');
    names{count} = strcat('(',int2str(count),')');
    legend(names{:},'Location','southeast');
    grid on;
    
    % Plot the y coordinates for all robots
    subplot(2,1,2)
    hold on;
    plot(x_plot(:,bot),colors{count},'LineWidth',2);
    xlabel('iterations'); 
    ylabel('y [m]');
    grid on;
    
    % Print image file
%     imgname = strcat('infstate-',int2str(N),'.pdf');
%     print('-dpdf',imgname);
%     print('-depsc2','-tiff',imgname);
    
    count = count + 1;
end