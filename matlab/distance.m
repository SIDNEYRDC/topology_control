function dist = distance(i,j)
% help
% -------------------------------------------------------------------------
% Return the calcule the euclidian distance between two elements
% Usage: 
% ------
% distance(i,j)
%
% Input Arguments:
% ----------------
% i: array of positions for first element
% j: array of positions for second element
%
% -------------------------------------------------------------------------
% Copyright 2015 SIDNEY RDC 
% Last edited: 20/05/2015           
% email: sydney_rdc@hotmail.com
% -------------------------------------------------------------------------

%% main code %%
    dist = sqrt((i(1) - j(1))^2 + (i(2) - j(2))^2);
end

