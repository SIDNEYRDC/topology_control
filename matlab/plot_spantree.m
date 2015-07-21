%% Plot the number of Spanning Trees in the Network %%

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

%% MAIN CODE %%

% Number of iterations
N = size(x_plot,1);

% Number of robots
n = size(x_plot,2)/2;

% Define number of spanning trees array
xi = zeros(1,N);

% warning ('off','MATLAB:eigs:TooManyRequestedEigsForComplexNonsym');

for t = 1 : N
    if ADJ_TYPE == 0
        A = adjacency_matrix(x_plot(t,:),r_max(t,:)) | zeros(n,n);
    elseif ADJ_TYPE == 1 
        A = A_plot(1:n,1:n,t) | zeros(n,n);
    end
    
    % Laplacian matrix
    L = diag(sum(A)) - A;
    
    % Test if A is symmetric    
    if isequal(A,A')
        eigenvalues = sort(eig(L));
    else
        eigenvalues = sort(eigs(L,n,'SR'));
    end
    
    % Process the spannig tree number
    xi(t) = 1 / n * prod(eigenvalues(2:size(eigenvalues,1)));
    
    t
end

figure;
plot([1:1:N],xi(1:N));
% axis([0 100 0 11]);
grid on;

% Set default font and interpreter
% set(gca,'defaulttextinterpreter','latex');
% set(gca,'DefaultTextFontname', 'CMU Serif Roman');
% set(0,'DefaultAxesFontName','CMU Serif Roman')
set(gca,'FontName','Helvetica')
% set(gca,'DefaultAxesFontName', 'Monospaced');

xlabel('Iterations'); 
ylabel('Spanning Trees ( \xi )');
xlim([1 N]);

% Save graph image
if SAVE_OPTIONS == 1
    imgname = strcat('spantree-',int2str(N),'.pdf');
    print('-dpdf',imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('spantree-',int2str(N),'.eps');
    print('-depsc2','-tiff',imgname);
end