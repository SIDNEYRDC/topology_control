%% Plot the Medium Convergence Error %%

n = size(x_plot,2)/2;
n_iter = size(x_plot,1);

figure;
hold on;

if exist('ref')
    for t = 1 : n_iter
        med_dist(t) = 0;
        
        for i = 1 : n-1
            xi = x_plot(t,2*i-1:2*i);
            med_dist(t) = med_dist(t) + sqrt((xi-ref(1,:))*(xi-ref(1,:))');
        end
        med_dist(t) = med_dist(t) / n;
    end
    
    plot(med_dist);
    
    box on;
    grid on;
    xlabel('iterations');
    ylabel('medium error');
    
%     imgname = strcat('error-',int2str(t),'.eps');
%     print('-depsc2','-tiff',imgname);
end
