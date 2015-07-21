%% Plot the algebraic connectivity when each robot is deleted from the Network %%

%% RUNNABLE OPTIONS %%

% Type of adjacency matrix
% 0: physic
% 1: logic
ADJ_TYPE = 1;

% Save data options
% 0: dont save
% 1: save .pdf
% 2: save .eps
SAVE_OPTIONS = 1;

%% MAIN CODE %%

% Number of iterations
t = 280;

% Number of robots
n = size(x_plot,2)/2;

% Define adjacency matrix type
if ADJ_TYPE == 0
    A = adjacency_matrix(x_plot(t,:),r_max(t,:)) | zeros(n,n);
elseif ADJ_TYPE == 1 
    A = A_plot(1:n,1:n,t) | zeros(n,n);
end

C = submatrix(A,1)
clear A;
A = submatrix(C,2)

n = n - 2;

% Lambda2 array
lambda_2 = zeros(1,n-2);

for i = 1 : n
    % Remove the index from A
    sub_A = submatrix(A,i);
    
    % Laplacian matrix
    L = diag(sum(sub_A)) - sub_A;
    
    % Test if A is symmetric    
    if isequal(A,A')
        eigenvalues = sort(eig(L));
    else
        eigenvalues = sort(eigs(L,n,'SR'));
    end
    
    lambda_2(i) = eigenvalues(2);
end

figure;
bar(lambda_2,'FaceColor',[192 192 192]/255);
grid on;

xlabel('Robots'); 
ylabel('Algebraic Connectivity (\lambda_2)');
xlim([0 n+1]);

% Save graph image
if SAVE_OPTIONS == 1
    imgname = strcat('spantree-',int2str(N),'.pdf');
    print('-dpdf',imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('spantree-',int2str(N),'.eps');
    print('-depsc2','-tiff',imgname);
end