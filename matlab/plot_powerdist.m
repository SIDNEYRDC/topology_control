%% Plot the Energy Cost of the Network %%

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
power = zeros(1,N);

for t = 1 : 1 : N
    if ADJ_TYPE == 0
        A = adjacency_matrix(x_plot(t,:),r_max(t,:));
    elseif ADJ_TYPE == 1 
        A = A_plot(1:n,1:n,t);
    end
    
    t
    
    for i = 1 : n
        neig = find(A(:,i));
        max_d = -1e+6;
        
        for j = 1 : size(neig,1)
            xi = x_plot(t,2*i-1:2*i);
            xj = x_plot(t,2*neig(j)-1:2*neig(j));
            d = sqrt((xi-xj)*(xi-xj)');
            
            if d > max_d
                max_d = d;
            end
        end
        
        power(t) = power(t) + max_d ^ 2;
    end
    
end

plot([1:1:N],power);
grid on;
xlabel('Iterations (\times 5)'); 
ylabel('Energy Cost (\Gamma)');
xlim([1 N]);

% Save graph image
if SAVE_OPTIONS == 1
    imgname = strcat('power-',int2str(N),'.pdf');
    print('-dpdf',imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('power-',int2str(N),'.eps');
    print('-depsc2','-tiff',imgname);
end