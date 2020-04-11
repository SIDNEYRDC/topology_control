%% Plot the Maximum Distances Between Neighbors Nodes %%
%%

% Number of iteractions
N = size(x_plot,1);

% Number of nodes
n = size(x_plot,2)/2;

% Define maximum distances array
max_dist = zeros(1,N);

for i = 1 : 1 : N
    i
    max_dist(i) = 1e-5;
    for k = 1 : n-1
        for j = k+1 : n
            if A_plot(k,j,i) > 0 || A_plot(j,k,i) > 0
                max_dist(i) = max(max_dist(i),sqrt((x_plot(i,2*(k-1)+1:2*k)-x_plot(i,2*(j-1)+1:2*j))*(x_plot(i,2*(k-1)+1:2*k)-x_plot(i,2*(j-1)+1:2*j))'));
            end
        end
    end
end

plot([1:1:N],max_dist)
xlabel('t'); 
ylabel('$ max(x_{ij}) $','Interpreter','latex');
grid on;