/**********************************************************************
*   Motion control algorithms
*   Written by Sidney RDC, 2014.
*   Last Change: 2016 Out 21 15:43:27
***********************************************************************/

#include "motion_control.hpp"

using namespace std;
using namespace arma;
using namespace QuadProgPP;

// Ordoñez algorithm to rendezvous
rowvec rendezvous(unsigned int id, mat A, mat x, rowvec x_r, mat v, float dt, vec r_sec, vec r_com, bool sec_c, bool com_c) {
    // Output array
    rowvec u = zeros<rowvec>(2);

    // Size of prediction horizon
    const unsigned int p = 5;

    // Number of elements in X
    const unsigned int n = p * 2;

    // Neighbors of i: A(j,i) -> i receive from j
    uvec N = find(A.col(id));

    // Gain for control variation
    const float lambda_i = 0.5;

    /*-----------------------------------------------------------------
    Quadractic optimization form:
    min 0.5 * x G x + g0 x
    s.t.
        CE^T x + ce0 = 0
        CI^T x + ci0 >= 0

    The matrix and vectors dimensions are as follows:
        G: n * n
        g0: n

        CE: n * p
        ce0: p

        CI: n * m
        ci0: m

        x: n
    ------------------------------------------------------------------*/

    // Main matrices of quadractic formulation
    Matrix<double> G(n,n), CE, CI(n,4*p);
    Vector<double> g0(n), V(n), ce0, ci0(4*p);

    // Temporary matrices for make formulation
    mat H = zeros<mat>(p,p);
    mat f = zeros<mat>(2,p);

    // Constant auxiliar matrices for make formulation
    const mat T = dt * trimatl(ones(p,p));
    const mat U_aux = inv(T/dt);

    // Initial positions and control signals
    mat U_i0 = zeros<mat>(p,2);
    mat X_i0 = ones<mat>(p,2);
    mat X_r0 = ones<mat>(p,2);
    mat X_j0;

    // Zero fill
    G = 0;
    V = 0;
    CI = 0;
    ci0 = 0;

    // Fill control variables
    U_i0(0,0) = v(id,0);    // Vectorial velocity of i in x
    U_i0(0,1) = v(id,1);    // Vectorial velocity of i in y

    // Fill position variables
    X_i0.col(0) *= x(id,0); // Robot i coordinates to x
    X_i0.col(1) *= x(id,1); // Robot i coordinates to y

    // Make the control signal saturation constraints
    for(unsigned int i = 0; i < p; i++) {
        CI[i][i] = -1;          // - vx + vx,max >= 0
        CI[i][i+p] = 1;         // vx - vx,min >= 0
        CI[i+p][i+p*2] = -1;    // - vy + vy,max >= 0
        CI[i+p][i+p*3] = 1;     // vy - vy,min >= 0

        ci0[i] = 1;         // vx,max
        ci0[i+p] = 1;       // -vx,min
        ci0[i+p*2] = 1;     // vy,max
        ci0[i+p*3] = 1;     // -vy,min
    }

    // For each neighbor of i, compute its distances
    for(unsigned int j = 0; j < N.n_elem; j++) {
        // Positions for j
        const rowvec x_j = x(N(j),span());

        // Clear X_j0
        X_j0 = ones<mat>(p,2);

        // Fill neighbor coordinates to 1st instant
        X_j0.col(0) *= x_j(0);      // Robot j coordinates to x
        X_j0.col(1) *= x_j(1);      // Robot j coordinates to y

        // Scalar distance (euclidian norm) between i and j
        const float dist_ij = dist(x(id,span()),x_j);

        // Distance variations term
        H += T.t() * T;
        f += (X_i0 - X_j0).t() * T;

        // Vectorial distance for x and y coordinates
        const float dx_ij = x_j(0) - x(id,0);
        const float dy_ij = x_j(1) - x(id,1);

        // Verify if the robot is in the risk distance (5% of j communication)
        if(com_c && dist_ij > r_com(N(j)) * 0.5) {
            // Maximum distance constraint
            const float r_max = ((dist_ij > r_com(N(j))) * (dist_ij - norm_2<rowvec>(v(id,span()),2) * dt) + (dist_ij <= r_com(N(j))) * r_com(N(j)));

            // Maximum distance coefficients from vectorial projection
            const float Dx_ij = dt / dist_ij * dx_ij;
            const float Dy_ij = dt / dist_ij * dy_ij;

            // Set new size to disjoint coefficients matrix
            resize<double>(CI,CI.nrows(),CI.ncols()+p);

            // Set new size to disjoint constants array
            resize<double>(ci0,ci0.size()+p);

            // Fill disjoint coefficient matrix
            for(unsigned int i = 0; i < p; i++) {
                CI[i][i+CI.ncols()-p] = Dx_ij;      // dt/||d_ij|| * d_ij_x + r_max - ||d_ij|| >= 0
                CI[i+p][i+CI.ncols()-p] = Dy_ij;    // dt/||d_ij|| * d_ij_y + r_max - ||d_ij|| >= 0

                ci0[i+ci0.size()-p] = r_max - dist_ij;
            }
        }

        // Verify if the robot is in violation of mininum radius (5 % of j + i coverage radius)
        if(sec_c && dist_ij < (r_sec(id) + r_sec(N(j))) * 1.05) {
            // Minimum distance constraint
            const float r_min = ((dist_ij >= r_sec(id) + r_sec(N(j))) * (r_sec(id) + r_sec(N(j))) + (dist_ij < r_sec(id) + r_sec(N(j))) * (dist_ij + norm_2<rowvec>(v(id,span()),2) / N.n_elem * dt));

            // Maximum distance coefficients from vectorial projection
            const float Dx_ij = dt / dist_ij * dx_ij;
            const float Dy_ij = dt / dist_ij * dy_ij;

            // Set new size to disjoint coefficients matrix
            resize<double>(CI,CI.nrows(),CI.ncols()+p);

            // Set new size to disjoint constants array
            resize<double>(ci0,ci0.size()+p);

            // Fill disjoint coefficient matrix
            for(unsigned int i = 0; i < p; i++) {
                CI[i][i+CI.ncols()-p] = -Dx_ij;     // -dt/||d_ij|| * d_ij_x + ||d_ij|| - r_min >= 0
                CI[i+p][i+CI.ncols()-p] = -Dy_ij;   // -dt/||d_ij|| * d_ij_y + ||d_ij|| - r_min >= 0

                ci0[i+ci0.size()-p] = dist_ij - r_min;
            }
        }
    }

    // Verify if the reference is know by i
    if(!x_r.is_empty()) {
        // Fill reference coordinates for 1st instant
        X_r0.col(0) *= x_r(0);  // Reference coordinates to x
        X_r0.col(1) *= x_r(1);  // Reference coordinates to y

        // Make the reference distance terms
        H += T.t() * T;
        f += (X_i0 - X_r0).t() * T;
    }

    // Build the control energy variation
    H += U_aux.t() * lambda_i * U_aux;
    f -= U_i0.t() * lambda_i * U_aux;

    // Fill main matrices with processed matrices
    for(unsigned int i = 0; i < p; i++) {
        for(unsigned int j = i; j < p; j++) {
            G[i][j] = H(i,j);
            G[j][i] = H(j,i);
            G[i+p][j+p] = H(i,j);
            G[j+p][i+p] = H(j,i);
        }

        g0[i] = f(0,i);
        g0[i+p] = f(1,i);
    }

    // Quadprog solver
    double J = solve_quadprog(G,g0,CE,ce0,CI,ci0,V);

    // Verify the feasibility of the problem
    if(std::isinf(J) || std::isnan(J)) {
        V = 0;
        cout << "ERROR: Not feasible problem!" << endl;
    }

    // Get 1st control signals
    u(0) = V[0];
    u(1) = V[p];

    return u;
}

// Control with potential fields
vec potential_fields(int id, mat A, mat x, mat v, float r_max, float r_min) {
    vec u = zeros<vec>(2);

    float a,b;

    // Gains for collision avoidance, velocity adjust and positions and velocities update
    float coll_gain = 0.002;
    float vel_gain = 0.5;

    // Neighbors set for robot
    uvec N = find(A.col(id));

    // Control law as potential fields theory
    for(unsigned int j=0; j < N.n_elem; j++) {
        float norm_x = sqrt(pow(x(id,0) - x(N(j),0),2) + pow(x(id,1) - x(N(j),1),2));

        if(norm_x <= r_min) {
            a = -3 / pow(r_min,4);
            b = 8 / pow(r_min,3);

            u = u + vel_gain*2*(v(id,span::all)-v(N(j),span::all)).t() + coll_gain*(-2/pow((pow(x(id,0) - x(N(j),0),2) + pow(x(id,1) - x(N(j),1),2)),2) + 2*a + b/sqrt(pow(x(id,0) - x(N(j),0),2) + pow(x(id,1) - x(N(j),1),2))) * (x(id,span::all) - x(N(j),span::all)).t();
        } else if(norm_x > r_min && norm_x < (r_max - r_min)) {
            u = u + vel_gain*2*(v(id,span::all)-v(N(j),span::all)).t();
        } else {//if ((norm_x >= r_max-r_min) && (norm_x < r_max)) {
            a = -(pow(r_max,2)+3*pow(r_max-r_min,2))/pow(pow(r_max,2)-pow(r_max-r_min,2),3);
            b = 8*pow(r_max-r_min,3)/pow(pow(r_max,2)-pow(r_max-r_min,2),3);

            u = u + vel_gain*2*(v(id,span::all)-v(N(j),span::all)).t() + coll_gain*(2/pow(pow(r_max,2) - (pow(x(id,0) - x(N(j),0),2) + pow(x(id,1) - x(N(j),1),2)),2) + 2*a + b/sqrt(pow(x(id,0) - x(N(j),0),2) + pow(x(id,1) - x(N(j),1),2))) * (x(id,span::all) - x(N(j),span::all)).t();
        }
    }

    return u;
}

// Connectivity control for topology control algorithms
rowvec connectivity_control(unsigned int id, mat A_l, uvec N_l, mat x, rowvec x_r, mat v, float dt, vec r_sec, vec r_com, bool sec_c, bool com_c, unsigned int opt_type) {
    // Output array
    rowvec u = zeros<rowvec>(2);

    // Size of prediction horizon
    const unsigned int p = 5;

    // Number of elements in X
    const unsigned int n = p * 2;

    // Neighbors of i: A(j,i) -> i receive from j
    uvec N = find(A_l.col(id));

    // Gain for control variation
    const float lambda_i = 0.5;

    /*-----------------------------------------------------------------
    Quadractic optimization form:
    min 0.5 * x G x + g0 x
    s.t.
        CE^T x + ce0 = 0
        CI^T x + ci0 >= 0

    The matrix and vectors dimensions are as follows:
        G: n * n
        g0: n

        CE: n * p
        ce0: p

        CI: n * m
        ci0: m

        x: n
    ------------------------------------------------------------------*/

    // Main matrices of quadractic formulation
    Matrix<double> G(n,n), CE, CI(n,4*p);
    Vector<double> g0(n), V(n), ce0, ci0(4*p);

    // Temporary matrices for make formulation
    mat H = zeros<mat>(p,p);
    mat f = zeros<mat>(2,p);

    // Constant auxiliar matrices for make formulation
    const mat T = dt * trimatl(ones(p,p));
    const mat U_aux = inv(T/dt);

    // Initial positions and control signals
    mat U_i0 = zeros<mat>(p,2);
    mat X_i0 = ones<mat>(p,2);
    mat X_d0 = ones<mat>(p,2);
    mat X_r0 = ones<mat>(p,2);
    mat X_j0;
    mat V_i0 = ones<mat>(p,2);
    mat V_j0;

    // Zero fill
    G = 0;
    V = 0;
    CI = 0;
    ci0 = 0;

    // Fill control variables
    U_i0(0,0) = v(id,0);    // Vectorial velocity of i in x
    U_i0(0,1) = v(id,1);    // Vectorial velocity of i in y

    // Fill position variables
    X_i0.col(0) *= x(id,0); // Robot i coordinates to x
    X_i0.col(1) *= x(id,1); // Robot i coordinates to y

    // Fill velocities variables
    V_i0.col(0) *= v(id,0); // Robot i velocity to x
    V_i0.col(1) *= v(id,1); // Robot i velocity to y

    // Make the control signal saturation constraints
    for(unsigned int i = 0; i < p; i++) {
        CI[i][i] = -1;          // - vx + vx,max >= 0
        CI[i][i+p] = 1;         // vx - vx,min >= 0
        CI[i+p][i+p*2] = -1;    // - vy + vy,max >= 0
        CI[i+p][i+p*3] = 1;     // vy - vy,min >= 0

        ci0[i] = 0.5;         // vx,max
        ci0[i+p] = 0.5;       // -vx,min
        ci0[i+p*2] = 0.5;     // vy,max
        ci0[i+p*3] = 0.5;     // -vy,min
    }

    // Minimization of links term
    if(opt_type == 1) {
        for(unsigned int j = 0; j < A_l.n_rows; j++) {
            // Positions for j
            const rowvec x_j = x(j,span());

            // Scalar distance (euclidian norm) between i and j
            const float dist_ij = dist(x(id,span()),x_j);

            if(j != id && !contains(j,N) && dist_ij < r_com(j)) {
                // Clear init values
                V_j0 = ones<mat>(p,2);
                X_d0 = ones<mat>(p,2);

                // Coverage distance factor
                const float D_m = (dist_ij - (r_com(j) + r_com(j) * 0.1)) / dist_ij;

                // Positions for coverage area
                X_d0.col(0) *= (1 - D_m) * x(id,0) + D_m * x_j(0);
                X_d0.col(1) *= (1 - D_m) * x(id,1) + D_m * x_j(1);

                // Coverage area term
                H += T.t() * T;
                f += (X_i0 - X_d0).t() * T;
            }
        }
    }

    // For each neighbor of i, compute its distances
    for(unsigned int j = 0; j < N.n_elem; j++) {
        // Positions for j
        const rowvec x_j = x(N(j),span());

        // Clear init values
        V_j0 = ones<mat>(p,2);
        X_d0 = ones<mat>(p,2);

        // Scalar distance (euclidian norm) between i and j
        const float dist_ij = dist(x(id,span()),x_j);

        // Enable coverage constraint
        if(sec_c) {
            // Coverage distance factor
            const float D_m = (dist_ij - (r_sec(id) + r_sec(N(j)))) / dist_ij;

            // Positions for coverage area
            X_d0.col(0) *= (1 - D_m) * x(id,0) + D_m * x_j(0);
            X_d0.col(1) *= (1 - D_m) * x(id,1) + D_m * x_j(1);

            // Coverage area term
            H += T.t() * T;
            f += (X_i0 - X_d0).t() * T;
        }

        // Fill velocities to j robot
        V_j0.col(0) *= v(N(j),0);
        V_j0.col(1) *= v(N(j),1);

        // Velocities normalization term (Only mst)
        if(opt_type == 1) {
            H += U_aux.t() * U_aux;
            f -= V_j0.t() * U_aux;
        }

        // Verify if the neighbor j is virtual
        if(contains(N(j),N_l)) {
            // Velocities adjustment term
            H += U_aux.t() * U_aux;
            f += V_j0.t() * U_aux;

        // Verify if the robot is in the risk distance
        } else if(com_c) {
            // Vectorial distance for x and y coordinates
            const float dx_ij = x_j(0) - x(id,0);
            const float dy_ij = x_j(1) - x(id,1);

            // Communication constraint gap (1% of communication radius)
            const float delta_max = r_com(N(j)) * 0.01;

            // Maximum distance constraint
            const float r_max = r_com(N(j)) - delta_max;

            // Maximum distance coefficients from vectorial projection
            const float Dx_ij = dt / dist_ij * dx_ij;
            const float Dy_ij = dt / dist_ij * dy_ij;

            // Set new size to disjoint coefficients matrix
            resize<double>(CI,CI.nrows(),CI.ncols()+p);

            // Set new size to disjoint constants array
            resize<double>(ci0,ci0.size()+p);

            // Fill disjoint coefficient matrix
            for(unsigned int i = 0; i < p; i++) {
                CI[i][i+CI.ncols()-p] = Dx_ij;      // dt/||d_ij|| * d_ij_x + r_max - ||d_ij|| >= 0
                CI[i+p][i+CI.ncols()-p] = Dy_ij;    // dt/||d_ij|| * d_ij_y + r_max - ||d_ij|| >= 0

                ci0[i+ci0.size()-p] = r_max - dist_ij;
            }
        }
    }

    // Verify if the reference is know by i
    if(!x_r.is_empty()) {
        // Fill reference coordinates for 1st instant
        X_r0.col(0) *= x_r(0);  // Reference coordinates to x
        X_r0.col(1) *= x_r(1);  // Reference coordinates to y

        // Make the reference distance terms
        H += T.t() * T;
        f += (X_i0 - X_r0).t() * T;
    }

    // Build the control energy variation
    H += U_aux.t() * lambda_i * U_aux;
    f -= U_i0.t() * lambda_i * U_aux;

    // Fill main matrices with processed matrices
    for(unsigned int i = 0; i < p; i++) {
        for(unsigned int j = i; j < p; j++) {
            G[i][j] = H(i,j);
            G[j][i] = H(j,i);
            G[i+p][j+p] = H(i,j);
            G[j+p][i+p] = H(j,i);
        }

        g0[i] = f(0,i);
        g0[i+p] = f(1,i);
    }

    // Quadprog solver
    double J = solve_quadprog(G,g0,CE,ce0,CI,ci0,V);

    // Verify the feasibility of the problem
    if(std::isinf(J) || std::isnan(J)) {
        V = 0;
        cout << "ERROR: Not feasible problem!" << endl;
    }

    // Get 1st control signals
    u(0) = V[0];
    u(1) = V[p];

    return u;
}

