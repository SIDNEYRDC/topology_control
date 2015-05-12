#include "k_optimization.hpp"

using namespace std;
using namespace arma;

// Generate a Minimal Spanning Tree for a weighted adjacency matrix A
mat mst_optimization(mat A) {
    // Verify if A is empty
    if(A.is_empty()) return A;

    // Number of nodes in the graph
    unsigned int n = A.n_rows;

    // Output matrix
    mat out = zeros<mat>(n,n);

    // Determine the size of Edges vector: sum(n-i)_{i=1}^{n-1}
    unsigned int n_xe = 0;

    // Edges and weights arrays
    double *coeffs = (double*)malloc(sizeof(double) * 1);
    edge *edges = (edge*)malloc(sizeof(edge) * 1);

    // Make edges with its weights
    for(unsigned int i = 0; i < n-1; i++) {
        for(unsigned int j = i + 1; j < n; j++) {
            if(A(i,j) > 0) {
                n_xe++;
                coeffs = (double*)realloc(coeffs, sizeof(double) * n_xe);
                coeffs[n_xe-1] = A(i,j);

                edges = (edge*)realloc(edges, sizeof(edge) * n_xe);
                edges[n_xe-1].i = i;
                edges[n_xe-1].j = j;
                edges[n_xe-1].w = A(i,j);
            } else if(A(j,i) > 0) {
                n_xe++;
                coeffs = (double*)realloc(coeffs, sizeof(double) * n_xe);
                coeffs[n_xe-1] = A(j,i);

                edges = (edge*)realloc(edges, sizeof(edge) * n_xe);
                edges[n_xe-1].i = j;
                edges[n_xe-1].j = i;
                edges[n_xe-1].w = A(j,i);
            }
        }
    }

    // Test if MST can be done
    if(n_xe < n) return A;

    try {
        // Gurobi Optimization environment and model
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);

        // Hide optimization results
        model.getEnv().set(GRB_IntParam_OutputFlag, 0);

        // Create and fill variables
        double *lb = new double[n_xe];
        double *ub = new double[n_xe];
        double *obj = new double[n_xe];
        char *type = new char[n_xe];
        string *names = new string[n_xe];
        double *coef1 = new double[n_xe];
        for(unsigned int i = 0; i < n_xe; i++) {
            lb[i] = 0;
            ub[i] = 1;
            obj[i] = 0;
            type[i] = GRB_BINARY;
            stringstream ss; ss << "x" << edges[i].i << "-" << edges[i].j;
            names[i] = ss.str();
            coef1[i] = 1;
        }

        // Add variables to model
        GRBVar* Xe = model.addVars(lb,ub,obj,type,names,n_xe);

        // Integrate new variables
        model.update();

        // Make the objective linear expression
        GRBLinExpr objExpr;
        objExpr.addTerms(coeffs, Xe, n_xe);

        // Set objective: minimize w0*x0 + w1*x1 + ... + wn*xn
        model.setObjective(objExpr,GRB_MINIMIZE);

        // Make the constraint: x0 + x1 + .. + xn = n - 1
        GRBLinExpr c0Expr;
        c0Expr.addTerms(coef1,Xe,n_xe);
        model.addConstr(c0Expr,GRB_EQUAL,n-1,"c0");

        //// Dynamic linear expressions for make constraints
        //GRBLinExpr *lhsExprs = new GRBLinExpr[n];
        //char *senses = new char[n];
        //double *rhsVals = new double[n];
        //names = new string[n];

        //// Make the constraint of type x0 + x1 + ... + xn <= 2
        //for(unsigned int v = 0; v < n; v++) {
            //for(unsigned int e = 0; e < n_xe; e++) {
                //if(edges[e].i == v || edges[e].j == v) {
                    //lhsExprs[v] += Xe[e];
                //}
            //}

            //senses[v] = GRB_LESS_EQUAL;
            //rhsVals[v] = 2;
            //stringstream ss; ss << "c" << v+1;
            //names[v] = ss.str();
        //}

        //show_constraints(lhsExprs,senses,rhsVals,names,n);

        //// Add constraint to model
        //model.addConstrs(lhsExprs,senses,rhsVals,names,n);

        // Make the constraint: x0 + x1 + ... + xi <= |S|/2 for all S \in V
        for(unsigned int depth = 3; depth <= (n / 2) + 1; depth++) {
            // Subgraphs set
            umat S = zeros<umat>(1,depth);

            // Generate all connected subgraphs with size = depth
            subgraph(S,A,depth);

            // Constraint number
            unsigned int n_c = S.n_rows;

            // Dynamic linear expressions for make constraints
            GRBLinExpr *lhsExprs = new GRBLinExpr[n_c];
            char *senses = new char[n_c];
            double *rhsVals = new double[n_c];
            names = new string[n_c];

            for(unsigned int c = 0; c < n_c; c++) {
                for(unsigned int v1 = 0; v1 < depth-1; v1++) {
                    for(unsigned int v2 = v1+1; v2 < depth; v2++) {
                        for(unsigned int e = 0; e < n_xe; e++) {
                            if((edges[e].i == S(c,v1) && edges[e].j == S(c,v2)) || (edges[e].j == S(c,v1) && edges[e].i == S(c,v2))) {
                                lhsExprs[c] += Xe[e];
                            }
                        }
                    }
                }

                senses[c] = GRB_LESS_EQUAL;
                rhsVals[c] = depth - 1;
                stringstream ss; ss << "c" << c << "-" << depth;
                names[c] = ss.str();
            }

            // Add constraint to model
            model.addConstrs(lhsExprs,senses,rhsVals,names,n_c);
        }

        // Optimize model
        model.optimize();

        // Make the MST weighted adjacency matrix
        for(unsigned int i = 0; i < n_xe; i++) {
            if(Xe[i].get(GRB_DoubleAttr_X) > 0) {
                if(A(edges[i].i,edges[i].j) > 0 && A(edges[i].j,edges[i].i) > 0)
                    out(edges[i].i,edges[i].j) = out(edges[i].j,edges[i].i) = edges[i].w;
                else if(A(edges[i].i,edges[i].j) > 0)
                    out(edges[i].i,edges[i].j) = edges[i].w;
                else if(A(edges[i].j,edges[i].i) > 0)
                    out(edges[i].j,edges[i].i) = edges[i].w;
            }
        }

    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
        return A;
    } catch(...) {
        cout << "Exception during optimization" << endl;
        return A;
    }

    return out;
}

// Generate a bi-connected Minimal Spanning Tree for a weighted adjacency
// matrix A using the traveling salesman problem solution
mat tsp_optimization(mat A) {
    // Verify if A is empty
    if(A.is_empty()) return A;

    // Number of nodes in the graph
    unsigned int n = A.n_rows;

    // Output matrix
    mat out = zeros<mat>(n,n);

    // Determine the size of Edges vector: sum(n-i)_{i=1}^{n-1}
    unsigned int n_xe = 0;

    // Edges and weights arrays
    double *coeffs = (double*)malloc(sizeof(double) * 1);
    edge *edges = (edge*)malloc(sizeof(edge) * 1);

    // Make edges with its weights
    for(unsigned int i = 0; i < n-1; i++) {
        for(unsigned int j = i + 1; j < n; j++) {
            if(A(i,j) > 0) {
                n_xe++;
                coeffs = (double*)realloc(coeffs, sizeof(double) * n_xe);
                coeffs[n_xe-1] = A(i,j);

                edges = (edge*)realloc(edges, sizeof(edge) * n_xe);
                edges[n_xe-1].i = i;
                edges[n_xe-1].j = j;
                edges[n_xe-1].w = A(i,j);
            }
            if(A(j,i) > 0) {
                n_xe++;
                coeffs = (double*)realloc(coeffs, sizeof(double) * n_xe);
                coeffs[n_xe-1] = A(j,i);

                edges = (edge*)realloc(edges, sizeof(edge) * n_xe);
                edges[n_xe-1].i = j;
                edges[n_xe-1].j = i;
                edges[n_xe-1].w = A(j,i);
            }
        }
    }

    // Test if 2-MST can be done
    if(n_xe <= n) return A;

    try {
        // Gurobi Optimization environment and model
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);

        // Hide optimization results
        model.getEnv().set(GRB_IntParam_OutputFlag, 0);

        // Create edges variables
        double *lb = new double[n_xe];
        double *ub = new double[n_xe];
        double *obj = new double[n_xe];
        char *type = new char[n_xe];
        string *names = new string[n_xe];
        double *coef1 = new double[n_xe];
        stringstream ss;

        // Fill edges variables
        for(unsigned int i = 0; i < n_xe; i++) {
            lb[i] = 0;
            ub[i] = 1;
            obj[i] = 0;
            type[i] = GRB_BINARY;
            ss.str(""); ss << "x" << edges[i].i << "-" << edges[i].j;
            names[i] = ss.str();
            coef1[i] = 1;
        }

        // Add variables to model
        GRBVar* Xe = model.addVars(lb,ub,obj,type,names,n_xe);

        // Create extra variables to MTZ formulation
        lb = new double[n];
        ub = new double[n];
        obj = new double[n];
        type = new char[n];
        names = new string[n];

        // Fill extra variables
        for(unsigned int i = 0; i < n; i++) {
            lb[i] = 1;
            ub[i] = n - 1;
            obj[i] = 0;
            type[i] = GRB_INTEGER;
            ss.str(""); ss << "u" << i;
            names[i] = ss.str();
        }

        // Add extra variables to model
        GRBVar* U = model.addVars(lb,ub,obj,type,names,n);

        // Integrate new variables
        model.update();

        // Make the objective linear expression
        GRBLinExpr objExpr;
        objExpr.addTerms(coeffs, Xe, n_xe);

        // Set objective: minimize w0*x0 + w1*x1 + ... + wn*xn
        model.setObjective(objExpr,GRB_MINIMIZE);

        // Dynamic linear expressions for make constraints
        GRBLinExpr *lhsExprs = new GRBLinExpr[2*n];
        char *senses = new char[2*n];
        double *rhsVals = new double[2*n];
        names = new string[2*n];

        // Make the flow constraints
        for(unsigned int v = 0; v < n; v++) {
            for(unsigned int e = 0; e < n_xe; e++) {
                if(edges[e].i == v) {
                    lhsExprs[v] += Xe[e];
                } else if(edges[e].j == v) {
                    lhsExprs[n+v] += Xe[e];
                }
            }

            // Make the constraint: xi0 + xi1 + .. + xin = 1 for all i \in V
            senses[v] = GRB_EQUAL;
            rhsVals[v] = 1;
            ss.str(""); ss << "c" << v;
            names[v] = ss.str();

            // Make the constraint: x0j + x1j + .. + xnj = 1 for all j \in V
            senses[v+n] = GRB_EQUAL;
            rhsVals[v+n] = 1;
            ss.str(""); ss << "c" << v+n;
            names[v+n] = ss.str();
        }

        // Add constraint to model
        model.addConstrs(lhsExprs,senses,rhsVals,names,2*n);

        // Find neighbors of r0
        uvec N = find(A.col(0));

        // Number of neighbors and counter
        unsigned int n0 = N.n_elem, nc = 0;

        // Redefine dynamic linear expressions
        lhsExprs = new GRBLinExpr[n_xe-n0];
        senses = new char[n_xe-n0];
        rhsVals = new double[n_xe-n0];
        names = new string[n_xe-n0];

        // Make the constraint: ui - uj + xij * n <= n - 1  with 0 < i,j < n
        for(unsigned int e = 0; e < n_xe; e++) {
            if(edges[e].i != 0 && edges[e].j != 0) {
                lhsExprs[nc] = U[edges[e].i] - U[edges[e].j] + n * Xe[e];
                senses[nc] = GRB_LESS_EQUAL;
                rhsVals[nc] = n - 1;
                ss.str(""); ss << "c" << 2*n+nc;
                names[nc] = ss.str();
                nc++;
            }
        }

        // Add constraint to model
        model.addConstrs(lhsExprs,senses,rhsVals,names,nc);

        // Optimize model
        model.optimize();

        // Make the 2-MST weighted adjacency matrix
        for(unsigned int i = 0; i < n_xe; i++) {
            if(Xe[i].get(GRB_DoubleAttr_X) > 0) {
                if(A(edges[i].i,edges[i].j) > 0 && A(edges[i].j,edges[i].i) > 0)
                    out(edges[i].i,edges[i].j) = out(edges[i].j,edges[i].i) = edges[i].w;
                else if(A(edges[i].i,edges[i].j) > 0)
                    out(edges[i].i,edges[i].j) = edges[i].w;
                else if(A(edges[i].j,edges[i].i) > 0)
                    out(edges[i].j,edges[i].i) = edges[i].w;
            }
        }

    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
        return A;
    } catch(...) {
        cout << "Exception during optimization" << endl;
        return A;
    }

    return out;
}

// Show constraints on gurobi format
void show_constraints(GRBLinExpr *lhsExprs, char *senses, double *rhsVals, string *names, unsigned int xn) {
    for(unsigned int i = 0; i < xn; i++) {
        cout << names[i] << ": " << lhsExprs[i] << " " << senses[i] << " " << rhsVals[i] << endl;
    }
}

// Show variables array on gurobi format
void show_var(GRBVar *vars, unsigned int size) {
    for(unsigned int i = 0; i < size; i++) {
        cout << vars[i].get(GRB_StringAttr_VarName) << " = " << vars[i].get(GRB_DoubleAttr_X) << endl;
    }
}

