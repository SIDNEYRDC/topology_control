#==============================================================================
 = Control Module to Topology Control Algorithm in Julia
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Jun 11 16:17:58
 = Info: This file contains the motion control algorithms used in the topology
 = control algorithm.
 =============================================================================#

module CONTROL

using Convex    # cvx interface to solve convex problems
using Gurobi    # to use Gurobi as optimization solver

# public functions
export hl_motion_control

#
#=
 = High Level Motion Control function
 = Info: Computes the omnidirectional velocity that moves the robot to fix the
 = connectivity using MPC and consensus.
 = Use: v = hl_motion_control(i, A, H, x, v, h, p, gamma), where i is the robot
 = index, A is the adjacency matrix, H is the Hamiltonian matrix from the TSP's
 = solution, x is the position array to the neighborhood of i, v is the velocity
 = array to the neighborhood of i, h is the control step time, p is the
 = prediction horizon to the MPC and gamma is a weight array to MPC.
 =#
function hl_motion_control(i, A, H, D, S, T, TI, n_bot, n_ref, x, v, r_com, r_cov, h, p, gamma, phi, RSSI_SENS, vxl, val)
    # auxiliary matrices definition
    Hixy = zeros(p, p)
    Hitheta = zeros(p, p)
    fix = zeros(1, p)
    fiy = zeros(1, p)
    fitheta = zeros(1, p)
    Xi = repmat(x[i, :], p, 1)
    Vi0 = zeros(p, 3)
    Vi0[1, :] = v[i, :]

    # get the 1-hop neighbours of i
    N1 = find(A[i, 1 : n_bot])

    # get the real and virtual neighbours of i
    N2 = find(max(A[i, 1 : n_bot], H[i, :]))

    # find the centroid to the robot i and its neighbors
    ci = [sum(x[[i; N1], 1])/(length(N1) + 1); sum(x[[i; N1], 2])/(length(N1) + 1)]

    # calculate the desired angle theta
    dx = x[i, 1] - ci[1]
    dx != 0 ? dtheta = atan((x[i, 2] - ci[2])/dx) : dtheta = 0

    for j in N2
        # auxiliary matrices definition
        Vj = repmat(v[j, :], p, 1)

        # enable RSSI sensing
        RSSI_SENS == 1 ? dij = 10^(S[j]/(-10*phi)) : dij = D[i, j]

        # calculate the desired position to x and y coordinates
        dxy = (x[i, 1 : 2] - x[j, 1 : 2])*(r_cov[i] + r_cov[j])/dij + x[j, 1 : 2]

        #=println("i:$(i) j:$(j) Dij:$(D[i, j]) Sij:$(S[i, j]) dij:$(exp(-S[i, j]/(10*phi)))")=#

        #dx = x[i, 1] - x[j, 1]
        #dx != 0 ? dtheta = atan((x[i, 2] - x[j, 2])/dx) : dtheta = 0

        Sij = fill(10*phi*log(r_cov[i] + r_cov[j]) - abs(S[j]), p, 1)

        #=println("i:$(i)   sijd:$(10*phi*log(r_cov[i] + r_cov[j]))    sij:$(S[j])")=#

        # auxiliary matrix to desired position
        Xd = repmat([dxy dtheta], p, 1)

        # term activation
        phi_ij = (1 - A[i, j])*H[i, j]
        psi_ij = A[i, j] + phi_ij

        # fill auxiliary matrices
        Hixy = Hixy + T'*h*h*psi_ij*gamma[1]*T + phi_ij*gamma[2] + (psi_ij - H[i, j])*gamma[3] + T'*h*h*psi_ij*gamma[7]*T
        Hitheta =  Hitheta + T'*h*h*psi_ij*gamma[1]*T
        fix = fix + (Xi[:, 1] - Xd[:, 1])'*h*psi_ij*gamma[1]*T + Vj[:, 1]'*phi_ij*gamma[2] - Vj[:, 1]'*(psi_ij - H[i, j])*gamma[3] - Sij'*h*psi_ij*gamma[7]*T
        fiy = fiy + (Xi[:, 2] - Xd[:, 2])'*h*psi_ij*gamma[1]*T + Vj[:, 2]'*phi_ij*gamma[2] - Vj[:, 2]'*(psi_ij - H[i, j])*gamma[3] - Sij'*h*psi_ij*gamma[7]*T
        fitheta = fitheta + (Xi[:, 3] - Xd[:, 3])'*h*psi_ij*gamma[1]*T
    end

    # fill the auxiliary matrices for the references
    if n_ref > 0
        for r = n_bot + 1 : n_bot + n_ref
            # auxiliary vector to each reference
            Xr = repmat(x[r, :], p, 1)
            Vr = repmat(v[r, :], p, 1)

            Hixy = Hixy + T'*h*h*A[i, r]*gamma[4]*T + A[i, r]*gamma[5]
            fix = fix + (Xi[:, 1] - Xr[:, 1])'*h*A[i, r]*gamma[4]*T - Vr[:, 1]'*A[i, r]*gamma[5]
            fiy = fiy + (Xi[:, 2] - Xr[:, 2])'*h*A[i, r]*gamma[4]*T - Vr[:, 2]'*A[i, r]*gamma[5]
        end
    end

    # fill auxiliary matrices
    Hixy = Hixy + TI'*gamma[6]*TI
    Hitheta = Hitheta + TI'*gamma[6]*TI
    fix = fix - Vi0[:, 1]'*gamma[6]*TI
    fiy = fiy - Vi0[:, 2]'*gamma[6]*TI
    fitheta = fitheta - Vi0[:, 3]'*gamma[6]*TI

    # declare optimization problem variables
    Ux = Convex.Variable(p)
    Uy = Convex.Variable(p)
    Utheta = Convex.Variable(p)

    # objective function
    problem = minimize(
        (1/2)*quadform([Ux; Uy; Utheta], [Hixy zeros(p, p) zeros(p, p); zeros(p, p) Hixy zeros(p, p); zeros(p, p) zeros(p, p) Hitheta]) + [fix fiy fitheta]*[Ux; Uy; Utheta],
        # saturation constraints
        Ux >= fill(vxl[1], p),
        Uy >= fill(vxl[1], p),
        Utheta >= fill(val[1], p),
        Ux <= fill(vxl[2], p),
        Uy <= fill(vxl[2], p),
        Utheta <= fill(val[2], p)
    )

    # maximum distance between neighbours constraint
    for j in N1
        problem.constraints += norm(x[i, 1 : 2] + [Ux[1] Uy[1]]*h - x[j, 1 : 2]) <= r_com[j]
    end

    # solve the optimization problem
    solve!(problem, GurobiSolver(OutputFlag = 0))

    # verify if the problem was solved
    if problem.status == :Optimal
        return [Ux.value[1] Uy.value[1] Utheta.value[1]], ci
    else
        return [0.0 0.0 0.0], ci
    end
end

end

