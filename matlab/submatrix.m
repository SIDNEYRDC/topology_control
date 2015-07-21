function sub_A = submatrix(A,index)
% help
% -------------------------------------------------------------------------
% Return the submatrix resultant of exclude index row and column from the
% input matrix
% Usage: 
% ------
% submatrix(A,index)
%
% Input Arguments:
% ----------------
% A: square matrix
% index: index of row and column to remove from A
%
% -------------------------------------------------------------------------
% Copyright 2015 SIDNEY RDC 
% Last edited: 20/05/2015           
% email: sydney_rdc@hotmail.com
% -------------------------------------------------------------------------

%% main code %%
    n = size(A,1);
    
    if size(A,2) ~= n
        disp('Only square matrix as input!');
        return;
    elseif index > n || index < 1
        disp('Index out of matrix size!')
        return;
    end

    if index == 1
        sub_A = A(2:n,2:n);
    elseif index == n
        sub_A = A(1:n-1,1:n-1);  
    else
        sub_A(1:index-1,1:index-1) = A(1:index-1,1:index-1);
        sub_A(1:index-1,index:n-1) = A(1:index-1,index+1:n);
        sub_A(index:n-1,1:index-1) = A(index+1:n,1:index-1);
        sub_A(index:n-1,index:n-1) = A(index+1:n,index+1:n);
    end            
end