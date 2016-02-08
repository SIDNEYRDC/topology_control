%% Plot the Minimum Degree of a Network %%

%% RUNNABLE OPTIONS %%

% Type of adjacency matrix
% 0: physic
% 1: logic
ADJ_TYPE = 0;

% Save data options
% 0: dont save
% 1: save .pdf
% 2: save .eps
SAVE_OPTIONS = 0;

%% MAIN CODE %%;

% Number of iterations
N = size(x_plot,1);

% Number of robots
n = size(x_plot,2)/2;

% Define minimum degree array
min_deg = zeros(1,N);

for t = 1 : 1 : N
    if ADJ_TYPE == 0
        A = adjacency_matrix(x_plot(t,:),r_max(t,:));
    elseif ADJ_TYPE == 1 
        A = A_plot(1:n,1:n,t);
    end
    
    t
    min_deg(t) = n;
    for k = 1 : n
        neig = find(A(:,k));
        if size(neig,1) < min_deg(t)
            min_deg(t) = size(neig,1);
        end
    end
end

plot([1:1:N],min_deg)
axis([0 N 0 n]);
grid on;
xlabel('iterations'); 
ylabel('$\delta(G)$','Interpreter','latex');

% Save graph image
if SAVE_OPTIONS == 1
    imgname = strcat('resistance-',int2str(N),'.pdf');
    print('-dpdf',imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('resistance-',int2str(N),'.eps');
    print('-depsc2','-tiff',imgname);
end