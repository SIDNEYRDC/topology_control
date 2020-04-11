%% Plot the Maximum Degree of a Network %%

% Type of adjacency matrix (0: physic, 1: logic)
ADJ_TYPE = 0;

% Number of iterations
N = size(x_plot,1);

% Number of robots
n = size(x_plot,2)/2;

% Define maximum degree array
max_deg = zeros(1,N);

for t = 1 : 1 : N
    if ADJ_TYPE == 0
        A = adjacency_matrix(x_plot(t,:),r_max(t,:));
    elseif ADJ_TYPE == 1 
        A = A_plot(1:n,1:n,t);
    end
    
    t
    max_deg(t) = 0;
    for k = 1 : n
        neig = find(A(:,k));
        if size(neig,1) > max_deg(t)
            max_deg(t) = size(neig,1);
        end
    end
end

plot([1:1:N],max_deg)
axis([0 N 0 n]);
grid on;
xlabel('iterations'); 
ylabel('$\Delta(\mathcal{G})$','Interpreter','latex');

% Print eps image
% imgname = strcat('max_deg-',int2str(N),'.eps');
% print('-depsc2','-tiff',imgname);