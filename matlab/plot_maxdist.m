%% Plot the Maximum Distances Between Neighbors Nodes %%
%%

% Number of iteractions
N = size(x_data, 3);

% Number of nodes
n = size(x_data, 1);

% time series
t = 0:h:(N*h - h)

% Define maximum distances array
max_dist = zeros(N, 1);

for k = 1 : 1 : N
    k

    max_dist(k) = 1e-5;

    for i = 1 : n-1
        for j = i + 1 : n
            if A_data(i, j, k) > 0 || A_data(j, i, k) > 0
                max_dist(k) = max(max_dist(k), norm(x_data(i, 1:2, k) - x_data(j, 1:2, k)));
                %max_dist(i) = max(max_dist(i),sqrt((x_plot(i,2*(k-1)+1:2*k)-x_plot(i,2*(j-1)+1:2*j))*(x_plot(i,2*(k-1)+1:2*k)-x_plot(i,2*(j-1)+1:2*j))'));
            end
        end
    end
end

plot(0:N-1, max_dist)
xlabel('Time Step');
ylabel('$ max(x_{ij}) $','Interpreter','latex');
grid on;

