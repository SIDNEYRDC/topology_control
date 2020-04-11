%% Plot for 2nd eigenvalue of Laplacian Matrix %%

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

% Define lambda2 array
lambda2 = zeros(1,N);

% warning ('off','MATLAB:eigs:TooManyRequestedEigsForComplexNonsym');

for t = 1 : N
    if ADJ_TYPE == 0
        A = adjacency_matrix(x_plot(t,:),r_max(t,:)) | zeros(n,n);
    elseif ADJ_TYPE == 1 
        A = A_plot(1:n,1:n,t) | zeros(n,n);
    end
    
    L = diag(sum(A)) - A;
    
    % Test if A is symmetric    
    if isequal(A,A')
        eigenvalues = sort(eig(L));
    else
        eigenvalues = sort(eigs(L,n,'SR'));
    end
    
    lambda2(t) = eigenvalues(2);
    
    t
end

figure;
plot([1:1:N],lambda2(1:N));
% axis([-0 N -1 n/4]);
grid on;
set(gca,'FontName','Helvetica')
xlabel('Iterations'); 
ylabel('Connectivity (\lambda_2)');
xlim([1 N]);

% Save graph image
if SAVE_OPTIONS == 1
    imgname = strcat('resistance-',int2str(N),'.pdf');
    print('-dpdf',imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('resistance-',int2str(N),'.eps');
    print('-depsc2','-tiff',imgname);
end