%% Plot the number of Edges in the Network %%

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

% Define number of edges array
edges = zeros(1,N);

for t = 1 : N    
    if ADJ_TYPE == 0
        A = adjacency_matrix(x_plot(t,:),r_max(t,:)) | zeros(n,n);
    elseif ADJ_TYPE == 1 
        A = A_plot(1:n,1:n,t) | zeros(n,n);
    end
    
    for i = 1 : n - 1
        for j = i + 1 : n
            if A(i,j) > 0 || A(j,i) > 0
                edges(t) = edges(t) + 1;
            end
        end
    end
    
    t
end

figure;
plot([1:1:N],edges(1:N));
% axis([0 250 0 12500]);
grid on;

% Set default font and interpreter
% set(gca,'defaulttextinterpreter','latex');
% set(gca,'DefaultTextFontname', 'CMU Serif Roman');
% set(0,'DefaultAxesFontName','CMU Serif Roman')
set(gca,'FontName','Helvetica')
% set(gca,'DefaultAxesFontName', 'Monospaced');

xlabel('Iterations'); 
ylabel('Number of Edges (\epsilon)');
xlim([1 N]);

% Save graph image
if SAVE_OPTIONS == 1
    imgname = strcat('edgenum-',int2str(N),'.pdf');
    print('-dpdf',imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('edgenum-',int2str(N),'.eps');
    print('-depsc2','-tiff',imgname);
end