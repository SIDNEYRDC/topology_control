#==============================================================================
 = Extra functions to Topology Control simulator in Julia
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Jun 27 14:59:31
 = Info: This file contains some extra functions necessary to Topology Control
 = simulator main code.
 =============================================================================#

#
#=
 = Reduce matrix
 = Info: Reduce the dimensions of a square matrix using a set of indeix.
 = Use: M = matreduce(A, index), where A is a square matrix and index is a
 = interest index vertex.
 =#
function matreduce(A, index)
    # get the size of index array
    n = length(index)

    # create output matrix with the new dimensions
    M = zeros(n,n);

    # process each index
    for i = 1 : n
        for j = 1 : n
            M[i, j] = A[index[i], index[j]]
        end
    end

    return M
end

#
#=
 = Neighbourhood
 = Info: get the neighbours from a node i that are a distance hop of it
 = Use: N = neighbourhood(A, i, hop), where A is the adjacency matrix and i
 = is the node's index and hop is the desired distance to get neighbors
 =#
function neighbourhood(A, i, hop, N = zeros(UInt8, 1))
    # add itself to the neighbours array
    if length(N) == 1
        N[1] = i
    end

    # decrease hop counter
    hop -= 1

    # get 1 hop neighbours from i
    N_i = find(A[i, :])

    for j in N_i
        # verify if the neighbour already is on the neighbourhood array
        if length(N[N .== j]) == 0
            N = vec(hcat(N', j))
        end

        # get recursively the deep neighbours
        if hop > 0
            N = neighbourhood(A, j, hop, N)
        end
    end

    return N
end

