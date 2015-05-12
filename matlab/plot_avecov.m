%% Plot the Average Coverage of the Network %%

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
avcov = zeros(1,N);

for t = 1 : 1 : N
    if ADJ_TYPE == 0
        A = adjacency_matrix(x_plot(t,:),r_max(t,:));
    elseif ADJ_TYPE == 1 
        A = A_plot(1:n,1:n,t);
    end
    
    for i = 1 : n - 1
        for j = i + 1 : n
            cov = 0;
            
            if A(i,j) > 0 || A(j,i) > 0         
                neig_i = find(A(:,i));
                neig_j = find(A(:,j));
                
                size_j = size(neig_j,1);
                size_i = size(neig_i,1);
                
                for ni = 1 : size(neig_i,1)
                    if ~isempty(find(neig_j == neig_i(ni)))
                        if A(i,j) > 0
                            if A(neig_i(ni),i) > 0 && A(neig_i(ni),i) <= A(i,j)
                                cov = cov + 1;
                            elseif A(i,neig_i(ni)) > 0 && A(i,neig_i(ni)) <= A(i,j)
                                cov = cov + 1;
                            end
                        elseif A(j,i) > 0
                            if A(neig_i(ni),i) > 0 && A(neig_i(ni),i) <= A(j,i)
                                cov = cov + 1;
                            elseif A(i,neig_i(ni)) > 0 && A(i,neig_i(ni)) <= A(j,i)
                                cov = cov + 1;
                            end
                        end
                    end
                end            
                avcov(t) = avcov(t) + cov;
            end
        end
    end
    
    avcov(t) = avcov(t) / n;
    
    t
end


plot([1:1:N],avcov);
% axis([-1 N -0.1 0.7]);
grid on;
xlabel('Iterations (\times 5)'); 
ylabel('Average Edge Coverage (AEC)');
xlim([1 N]);

% Save graph image
if SAVE_OPTIONS == 1
    imgname = strcat('avecov-',int2str(N),'.pdf');
    print('-dpdf',imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('avecov-',int2str(N),'.eps');
    print('-depsc2','-tiff',imgname);
end