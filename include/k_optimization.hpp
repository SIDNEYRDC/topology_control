/***********************************************************************
*	k - Minimal Spanning Tree Algorithm (k-MST) with Integer Programming
*	solved by Gorubi©. Written by Sidney RDC, 2014.
*	Last Change: 2015 Mar 14 17:33:08
***********************************************************************/

#ifndef KOPT_H
#define KOPT_H

#include "graph_utils.hpp"
#include "gurobi_c++.h"

// Typedef with edge data
typedef struct edge {
	unsigned int i;	// i vertice
	unsigned int j;	// j vertice
	double w;		// weight of edge
} edge;

// Generate a Minimal Spanning Tree for a weighted adjacency matrix A
arma::mat mst_optimization(arma::mat A);

// Generate a bi-connected Minimal Spanning Tree for a weighted adjacency matrix A
arma::mat tsp_optimization(arma::mat A);

// Show constraints on gurobi format
void show_constraints(GRBLinExpr *lhsExprs, char *senses, double *rhsVals, string *names, unsigned int xn);

// Show variables array on gurobi format
void show_var(GRBVar *vars, unsigned int size);

#endif

