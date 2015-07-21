function A = adjacency_matrix(pose, radius)
% help
% -------------------------------------------------------------------------
% Return the weighted adjacency matrix for a geometric graph
% Usage: 
% ------
% adjacency_matrix(pose,radius)
%
% Input Arguments:
% ----------------
% pose: array of positions for all nodes from the graph
% radius: array of communication radius to all nodes from the graph
%
% -------------------------------------------------------------------------
% Copyright 2015 SIDNEY RDC 
% Last edited: 20/05/2015           
% email: sydney_rdc@hotmail.com
% -------------------------------------------------------------------------

%% main code %%
    % Number of nodes
    n = size(pose,2)/2;
    
    % Define the adjacency matrix
    A = zeros(n,n);

    for i = 1 : n - 1
        for j = i + 1 : n
            xi = pose(2*i-1:2*i);
            xj = pose(2*j-1:2*j);
            d = sqrt((xi-xj)*(xi-xj)');
            
            if radius(i) == 0 || (d <= radius(i) && d <= radius(j))
                A(i,j) = d;
                A(j,i) = d;
            elseif d <= radius(i)
                A(i,j) = d;
            elseif d <= radius(j)
                A(j,i) = d;
            end
        end
    end
end