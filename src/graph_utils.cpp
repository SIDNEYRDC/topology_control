/**********************************************************************
*   Tools for access, manage and process graphs by its adjacency matrix
*   Written by Sidney RDC, 2014.
*   Last Change: 2015 Mar 22 05:55:21
***********************************************************************/

#include "graph_utils.hpp"

using namespace std;
using namespace arma;
using namespace QuadProgPP;

// Logic or between two adjacency matrices
mat logic_or(mat x, mat y) {
    mat out = x + y;

    for(unsigned int i=0; i<out.n_rows; i++) {
        for(unsigned int j=0; j<out.n_cols; j++) {
            if(out(i,j) > 0) out(i,j) = 1;
            else if(out(i,j) < 0) out(i,j) = 0;
        }
    }

    return out;
}

// Override of logic or between two weighted adjacency matrices
mat logic_or(mat x, mat y, mat pose) {
    mat out = x + y;

    for(unsigned int i = 0; i < out.n_rows; i++) {
        rowvec p_i = pose(i,span());
        for(unsigned int j = 0; j < out.n_cols; j++) {
            rowvec p_j = pose(j,span());
            float dist_ij = dist(p_i,p_j);
            if(out(i,j) > 0 && dist_ij > 0) out(i,j) = dist_ij;
            else if(out(i,j) > 0 && dist_ij == 0) out(i,j) = 1;
        }
    }

    return out;
}

// Logic or between two logic vectors
uvec logic_or_vec(uvec x, uvec y) {
    uvec out = x + y;

    for(unsigned int i = 0; i < out.n_elem; i++) {
        if(out(i) > 1) out(i) = 1;
    }

    return out;
}

// Logic and between two matrices
mat logic_and(mat x, mat y) {
    unsigned int n = x.n_rows;

    return logic_or(x,zeros<mat>(n,n)) % logic_or(y,zeros<mat>(n,n));
}

// Reachability for an adjacency matrix A
mat reach_matrix(mat A) {
    unsigned int i = A.n_rows;
    unsigned int j = A.n_cols;

    mat r = logic_or(A,eye(i,j));
    mat aux = zeros(i,j);

    while(prod(prod(aux == r)) == 0) {
        aux = r;
        r = logic_or(r,(r*r));
    }

    return r;
}

// Reduce the adjacency matrix to another matrix
std::vector<mat> min_mat(unsigned int id, mat A, unsigned int mode) {
    unsigned int count = 0, n = A.n_rows;
    std::vector<mat> out(2);

    // Real index for new matrix
    mat index = ones<mat>(1,n) * (-1);

    // Reduce to reachable robots
    if(mode == RA) {
        // Reachability matrix
        mat r_mat = reach_matrix(A);

        for(unsigned int j = 0; j < n; j++) {
            if(r_mat(id,j) || r_mat(j,id)) {
                index(0,count) = j;
                count++;
            }
        }

    // Reduce to 1-hop neighbors
    } else if(mode == N1) {
        // Neighbors of id
        uvec N = find(A.col(id));

        // Save the real index to id
        index(0,count++) = id;

        // Neighbor of j and temp
        uvec N_j, temp;

        for(unsigned int j = 0; j < N.n_elem; j++) {
            // Verify if N(j) exist in index
            temp = find(index.row(0) == N(j));
            if(temp.is_empty()) index(0,count++) = N(j);

            // Neighbors of N(j)
            N_j = find(A.col(N(j)));

            // Get all neighbors of 1-hop
            for(unsigned int k = 0; k < N_j.n_elem; k++) {
                // Verify if N(j) exist in index
                temp = find(index.row(0) == N_j(k));
                if(temp.is_empty()){
                    index(0,count++) = N_j(k);
                }
            }
        }
    }

    index.resize(1,count);

    mat min_A = zeros<mat>(count,count);

    for(unsigned int i = 0; i < count-1; i++) {
        for(unsigned int j = i+1; j < count; j++) {
            min_A(i,j) = A(index(0,i),index(0,j));
            min_A(j,i) = A(index(0,j),index(0,i));
        }
    }

    if(count > 1) {
        out[0] = min_A;
        out[1] = index;
    } else {
        index.resize(1,n);
        for(unsigned int i = 0; i < n; i++) index(0,i) = i;

        out[0] = A;
        out[1] = index;
    }

    return out;
}

// Reduce the adjacency matrix to connected graph
std::vector<mat> min_mat3(mat A) {
    unsigned int count = 0, n = A.n_rows;
    std::vector<mat> out(2);

    // Reachability matrix
    mat r_mat = reach_matrix(A);

    mat index = zeros<mat>(1,n);
    index.fill(-1);

    for(unsigned int i = 0; i < n; i++) {
        for(unsigned int j = 0; j < n; j++) {
            if(j != i && (r_mat(i,j) || r_mat(j,i))) {
                index(0,count) = i;
                count++;
                break;
            }
        }
    }

    index.resize(1,count);

    mat min_A = zeros<mat>(count,count);

    for(unsigned int i = 0; i < count-1; i++) {
        for(unsigned int j = i+1; j < count; j++) {
            min_A(i,j) = A(index(0,i),index(0,j));
            min_A(j,i) = A(index(0,j),index(0,i));
        }
    }

    if(count > 1) {
        out[0] = min_A;
        out[1] = index;
    } else {
        index.resize(1,n);
        for(unsigned int i = 0; i < n; i++) index(0,i) = i;

        out[0] = A;
        out[1] = index;
    }

    return out;
}

// Verify if a graph is connected
bool is_connected(mat A) {
    // Make the binary adjacency matrix from A
    mat A_b = logic_or(A,zeros<mat>(A.n_rows,A.n_cols));

    // Laplacian matrix
    mat L = diagmat(sum(A_b)) - A_b;

    if(is_symmetric(A)) {
        // L eigenvalues
        vec eigval = sort(eig_sym(L));

        // If lambda2 <= 0, the graph is disconnected
        if(eigval.n_elem > 1 && eigval(1) < MIN_ERR) return false;
    } else {
        // L eigenvalues
        cx_vec eigval = sort(eig_gen(L));

        // If lambda2 <= 0, the graph is disconnected
        if(eigval.n_elem > 1 && eigval(1).real() < MIN_ERR) return false;
    }

    return true;
}

// Verify if a node is critical
bool is_critical(mat A, unsigned int node) {
    // Reduce the adjacency matrix to the reachable nodes only
    std::vector<mat> out = min_mat(node,A,RA);

    // The node is alone
    if(out[0].is_empty()) return false;

    // Real id for the minimized adjacency matrix
    uvec real_id = find(out[1].row(0) == node);

    // Remove the node from the graph
    mat test_A;
    if(real_id(0) == 0) {
        test_A = out[0].submat(1,1,out[0].n_rows-1,out[0].n_rows-1);
    } else if(real_id(0) == out[0].n_rows-1) {
        test_A = out[0].submat(0,0,out[0].n_rows-2,out[0].n_rows-2);
    } else {
        test_A = out[0].submat(0,0,real_id(0)-1,real_id(0)-1);
        test_A.resize(out[0].n_rows-1,out[0].n_rows-1);
        test_A.submat(real_id(0),0,out[0].n_rows-2,real_id(0)-1) = out[0].submat(real_id(0)+1,0,out[0].n_rows-1,real_id(0)-1);
        test_A.submat(0,real_id(0),real_id(0)-1,out[0].n_rows-2) = out[0].submat(0,real_id(0)+1,real_id(0)-1,out[0].n_rows-1);
        test_A.submat(real_id(0),real_id(0),out[0].n_rows-2,out[0].n_rows-2) = out[0].submat(real_id(0)+1,real_id(0)+1,out[0].n_rows-1,out[0].n_rows-1);
    }

    return !is_connected(test_A);
}

// Find all critical nodes in the graph
uvec critical_nodes(mat A) {
    uvec c_nodes;
    unsigned int n, c_num;

    // Reduce A to connected graph only
    vector<mat> out = min_mat3(A);

    // Number of elements in min_A
    n = out[0].n_rows;

    for(unsigned int id = 0; id < n; id++) {
        if(is_critical(out[0],id)) {
            c_num = c_nodes.n_elem;
            c_nodes.resize(c_num+1);
            c_nodes(c_num) = out[1].at(0,id);
        }
    }

    return c_nodes;
}

// Verify if a matrix m is symmetric
bool is_symmetric(mat m) {
    unsigned int n_r = m.n_rows, n_c = m.n_cols;

    for(unsigned int i = 0; i < n_r; i++) {
        for(unsigned int j = i; j < n_c; j++) {
            if(m(i,j) != m(j,i)) return false;
        }
    }

    return true;
}

// Convert a matrix element for a sparse matrix element
sp_mat mat_2_spmat(mat m) {
    unsigned int n_r = m.n_rows, n_c = m.n_cols;

    sp_mat out;
    out.set_size(n_r,n_c);

    for(unsigned int i = 0; i < n_r; i++) {
        for(unsigned int j = i; j < n_c; j++) {
            out(i,j) = m(i,j);
            out(j,i) = m(j,i);
        }
    }

    return out;
}

// Enumerate all connected subgraphs on a graph
void subgraph(umat &S, vector<int> sg, mat A, uvec N, unsigned int depth, int &id_s, int id_sg) {
    for(unsigned int i = 0; i < N.n_elem; i++) {
        // If neighbor i not exist in sg
        if(!find_value(N(i),sg)) {
            // The subgraph set
            sg[id_sg] = N(i);

            // The neighbor of actual node set
            uvec N_j = find(A.col(N(i)));

            // Test the function reach the top of stack
            if(depth > 1) subgraph(S,sg,A,N_j,depth-1,id_s,id_sg+1);
            else {
                bool exist = false;
                vector<int> temp1, temp2;

                temp2 = sg;
                sort(temp2.begin(),temp2.end());

                // Compare the actual subgraph with other save subgraphs
                for(unsigned int r = 0; r < S.n_rows; r++) {
                    fill_vec(temp1,S.row(r));

                    sort(temp1.begin(),temp1.end());

                    if(temp1 == temp2) {
                        exist = true;
                        break;
                    }
                }

                // If the actual subgraph not exist in S, save it
                if(!exist) {
                    id_s++;
                    S.resize(id_s+1,S.n_cols);

                    for(unsigned int k = 0; k < S.n_cols; k++) {
                        S(id_s,k) = sg[k];
                    }
                }
            }
        }
    }
}

// Overload of subgraph generation
void subgraph(umat &S, mat A, unsigned int depth) {
    unsigned int n = A.n_rows;
    int id_s = -1, id_sg = 0;
    vector<int> sg(depth);

    // Fill the void positions with -1
    std::fill(sg.begin(),sg.end(),-1);

    // For all nodes, generate the connected subgraphs
    for(unsigned int i = 0; i < n; i++) {
        sg[id_sg] = i;
        uvec N = find(A.col(i));
        subgraph(S,sg,A,N,depth-1,id_s,id_sg+1);
    }
}

// Combine Cn,d distint
void combine(umat &S, vector<unsigned int> sg, unsigned int id, unsigned int depth, unsigned int n, unsigned int &id_s, unsigned int id_sg) {
    for(unsigned int i = id; i < n - (depth - 1); i++) {
        // The subgraph set
        sg[id_sg] = i;

        // Test the function reach the top of stack
        if(depth > 1) combine(S,sg,i+1,depth-1,n,id_s,id_sg+1);
        else {
            // Increment S index
            id_s++;

            // Resize S
            S.resize(id_s+1,S.n_cols);

            // Store combination in S
            for(unsigned int k = 0; k < S.n_cols; k++) {
                S(id_s,k) = sg[k];
            }
        }
    }
}

// Overload in combine
void combine(umat &S, unsigned int depth, unsigned int n) {
    vector<unsigned int> sg(depth);
    unsigned int id_s = 0;

    combine(S,sg,0,depth,n,id_s,0);
}

