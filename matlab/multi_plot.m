function [] = multi_plot(data,color,label,labelpose,xl,yl,out) 
% help
% -------------------------------------------------------------------------
% Plot a large number of data in a same picture
% Usage: 
% ------
% multi_plot({data1,data2},{'r','b'},{'data1','data2'},'xlabel','ylabel',1)
%
% Input Arguments:
% ----------------
% data: array of elements to plot
% color: color of draw
% label: label of data
% labelpose: position of label in the graph ('north', 'south', 'northwest'
% 'southwest', 'northeast', 'southeast')
% xl: label on x axis
% yl: label on y axis
% out: output format to data (1:pdf, 2:eps)
%
% -------------------------------------------------------------------------
% Copyright 2015 SIDNEY RDC 
% Last edited: 11/03/2015           
% email: sydney_rdc@hotmail.com
% -------------------------------------------------------------------------

%% main code %%

    % verify data input
    if ~exist('data','var') 
        disp('No input data!');
        return;
    end
    
    % verify the color definition
    if ~exist('color','var')
         color = {'b','g','r','c','m','k','y'};
    end
    
    if ~exist('labelpose','var')
        labelpose = 'northeast';
    end
    
    % Auxiliar variables
    count = 1;
    names = '';
    hold all;
    
    for elem = 1 : size(data,2)
        plot(data{elem},'Color',color{elem});
        
        names{count} = label{elem};
        legend(names{:},'Location',labelpose);
        count = count + 1;
    end
    
    grid on;
    
    if exist('xl','var')
        xlabel(xl);
    end
    
    if exist('yl','var')
        ylabel(yl);
    end
    
    xlim([1 size(data{1},2)]);
    
    % verify if the output is setted
    if exist('out','var')
        if out == 1
            imgname = 'plot.pdf';
            print('-dpdf',imgname);
        elseif out == 2
            imgname = strcat('plot.eps');
            print('-depsc2','-tiff',imgname);
        end
    end

end
