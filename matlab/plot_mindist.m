%% Plot the minimum distance between neighbors nodes %%
%%

% Number of iteractions
N = size(x_plot,1);

% Number of nodes
n = size(x_plot,2)/2;

% Define minimum distances array
min_dist = zeros(1,N);

for i = 1 : 1 : N
    i
    min_dist(i) = 1e+5;
    for k = 1 : n-1
        for j = k+1 : n
            if A_plot(k,j,i) > 0 || A_plot(j,k,i) > 0
                min_dist(i) = min(min_dist(i),sqrt((x_plot(i,2*(k-1)+1:2*k)-x_plot(i,2*(j-1)+1:2*j))*(x_plot(i,2*(k-1)+1:2*k)-x_plot(i,2*(j-1)+1:2*j))'));
            end
        end
    end
end

plot([1:1:N],min_dist)
xlabel('t'); 
ylabel('$ min(x_{ij}) $','Interpreter','latex');
grid on;