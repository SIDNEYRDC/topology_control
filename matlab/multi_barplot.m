%% Plot the algebraic connectivity when each robot is deleted from the Network %%

%% RUNNABLE OPTIONS %%

% Save data options
% 0: dont save
% 1: save .pdf
% 2: save .eps
SAVE_OPTIONS = 1;

%% MAIN CODE %%

figure;
bar([VLINK',TSP'],'grouped');
grid on;

ax = get(gca);
cat = ax.Children;

%set the first bar chart style
set(cat(2),'FaceColor','k','BarWidth',1);

%set the second bar chart style
set(cat(1),'FaceColor',[238 238 238]/255,'BarWidth',1);

%set the axes style
set(gca,'box','on');

%add legend
legend('vlink','tsp','Location','north');

xlabel('Robots'); 
ylabel('Algebraic Connectivity (\lambda_2)');
xlim([0 n+1]);

% Save graph image
if SAVE_OPTIONS == 1
    imgname = strcat('connectivity.pdf');
    print('-dpdf',imgname);
elseif SAVE_OPTIONS == 2
    imgname = strcat('connectivity.eps');
    print('-depsc2','-tiff',imgname);
end