/**********************************************************************
*   Tools for access, manage and process graphs by its adjacency matrix
*   Written by Sidney RDC, 2014.
*   Last Change: 2015 Mar 22 04:47:11
***********************************************************************/

#ifndef GRAPHUTILS_H
#define GRAPHUTILS_H

#include <iostream>
#include <armadillo>
#include <cmath>
#include "utils.hpp"

// Operational modes to min_mat function
#define RA 0
#define N1 1

// Logic or between two adjacency matrices
arma::mat logic_or(arma::mat x, arma::mat y);

// Override of logic or between two weighted adjacency matrices
arma::mat logic_or(arma::mat x, arma::mat y, arma::mat pose);

// Logic or between two logic vectors
arma::uvec logic_or_vec(arma::uvec x, arma::uvec y);

// Logic and between two matrices
arma::mat logic_and(arma::mat x, arma::mat y);

// Reachability for an adjacency matrix A
arma::mat reach_matrix(arma::mat A);

// Reduce the adjacency matrix to another matrix
std::vector<arma::mat> min_mat(unsigned int id, arma::mat A, unsigned int mode);

// Reduce the adjacency matrix to connected graph
std::vector<arma::mat> min_mat3(arma::mat A);

// Verify if a graph is connected
bool is_connected(arma::mat A);

// Verify if a node is critical
bool is_critical(arma::mat A, unsigned int node);

// Find all critical nodes in the graph
arma::uvec critical_nodes(arma::mat A);

// Verify if a matrix m is symmetric
bool is_symmetric(arma::mat m);

// Convert a matrix element for a sparse matrix element
arma::sp_mat mat_2_spmat(arma::mat m);

// Enumerate all connected subgraphs on a graph
void subgraph(arma::umat &S, std::vector<int> sg, arma::mat A, arma::uvec N, unsigned int depth, int &id_s, int id_sg);

// Overload of subgraph generation
void subgraph(arma::umat &S, arma::mat A, unsigned int depth);

// Combine Cn,d distint
void combine(arma::umat &S, std::vector<unsigned int> sg, unsigned int id, unsigned int depth, unsigned int n, unsigned int &id_s, unsigned int id_sg);

// Overload in combine
void combine(arma::umat &S, unsigned int depth, unsigned int n);

#endif

