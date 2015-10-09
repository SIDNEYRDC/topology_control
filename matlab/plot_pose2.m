em%% Plot the final position for robots %%

% Vector of colors for each robots
colors = {'b','g','r','c','m','k','y','b','g','r','c','m','y','k','b','g','r','c','m','y','k'};

% Number of robots
n = size(x_plot,2);

% Number of iteractions
N = size(x_plot,1);

newplot;
hold on;

% Reference number show
if exist('ref')
    for i = 1 : size(ref,1)
        plot(ref(i,1),ref(i,2),'*r','MarkerSize',20);
        
        % Show referece number
        str = strcat('     R',num2str(i));
        text(ref(i,1),ref(i,2),'   ref','Color','r');
    end
end

% Position for agents
for agent = 2 : 2 : n
    % Plot agent in the position (x,y)
    plot(x_plot(N,agent-1),x_plot(N,agent),'Marker','.','Color',colors{agent/2},'MarkerSize',20); 

    % Show agents path
    plot(x_plot(max(1,N-size(x_plot,1)):N,agent-1),x_plot(max(1,N-size(x_plot,1)):N,agent),'Color',colors{agent/2},'LineStyle','-');

    % Show number of each agent
    str = strcat('(',num2str(agent/2),')');
    text(x_plot(1,agent-1),x_plot(1,agent),str);
end

grid on;
%axis([-1.2 1.2 -1.2 1.2]);
xlabel('x [m]'); 
ylabel('y [m]');
%legend('link');    

axis equal;
axis manual;
box on;
hold off;

% imgname = strcat('pose2-',int2str(N),'.pdf');
% print('-dpdf',imgname);
% print('-depsc2','-tiff',imgname);