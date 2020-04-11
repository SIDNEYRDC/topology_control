#==============================================================================
 = Control Module to Topology Control Algorithm in Julia
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Set 30 15:00:10
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
function hl_motion_control(i, Ai, Hi, Si, T, TI, n_bot, n_ref, x, v, r_com, r_cov, dt, p, gamma, phi, RSSI_SENS, vxl, val)
    # auxiliary matrices definition
    Gixy = zeros(p, p)
    Githeta = zeros(p, p)
    fix = zeros(1, p)
    fiy = zeros(1, p)
    fitheta = zeros(1, p)
    Xi = repmat(x[i, :]', p, 1)  # NOTE: after julia upgrade to 0.5 version, a subvector of a vector is ALWAYS a column vector.
    Vi0 = zeros(p, 3)
    Vi0[1, :] = v[i, :]

    # get the 1-hop neighbours of i
    N1 = find(Ai[1 : n_bot])

    # get the real and virtual neighbours of i
    N2 = find(max(Ai[1 : n_bot], Hi))

    # find the centroid to the robot i and its neighbors
    ci = [sum(x[[i; N1], 1])/(length(N1) + 1); sum(x[[i; N1], 2])/(length(N1) + 1)]

    # calculate the desired angle theta
    dx = x[i, 1] - ci[1]
    dx != 0 ? dtheta = atan((x[i, 2] - ci[2])/dx) : dtheta = 0

    for j in N2
        # auxiliary matrices definition
        Vj = repmat(v[j, :]', p, 1)
        Sij = fill(10*phi*log(r_cov[i] + r_cov[j]) - abs(Si[j]), p, 1)

        # enable RSSI sensing
        RSSI_SENS == 1 ? dij = 10^(Si[j]/(-10*phi)) : dij = norm(x[i, 1 : 2] - x[j, 1 : 2])

        # calculate the desired position to x and y coordinates
        dxy = (x[i, 1 : 2] - x[j, 1 : 2])*(r_cov[i] + r_cov[j])/dij + x[j, 1 : 2]

        # auxiliary matrix to desired position
        Xd = repmat([dxy' dtheta], p, 1)

        # term activation
        phi_ij = (1 - Ai[j])*Hi[j]
        psi_ij = Ai[j] + phi_ij

        # fill auxiliary matrices
        Gixy = Gixy + T'*dt*dt*psi_ij*gamma[1]*T + phi_ij*gamma[2] + (psi_ij - Hi[j])*gamma[3] + T'*dt*dt*psi_ij*gamma[7]*T
        Githeta = Githeta + T'*dt*dt*psi_ij*gamma[1]*T
        fix = fix + (Xi[:, 1] - Xd[:, 1])'*dt*psi_ij*gamma[1]*T + Vj[:, 1]'*phi_ij*gamma[2] - Vj[:, 1]'*(psi_ij - Hi[j])*gamma[3] - Sij'*dt*psi_ij*gamma[7]*T
        fiy = fiy + (Xi[:, 2] - Xd[:, 2])'*dt*psi_ij*gamma[1]*T + Vj[:, 2]'*phi_ij*gamma[2] - Vj[:, 2]'*(psi_ij - Hi[j])*gamma[3] - Sij'*dt*psi_ij*gamma[7]*T
        fitheta = fitheta + (Xi[:, 3] - Xd[:, 3])'*dt*psi_ij*gamma[1]*T
    end

    # fill the auxiliary matrices for the references
    if n_ref > 0
        for r = n_bot + 1 : n_bot + n_ref
            # auxiliary vector to each reference
            Xr = repmat(x[r, :]', p, 1)
            Vr = repmat(v[r, :]', p, 1)

            Gixy = Gixy + T'*dt*dt*Ai[r]*gamma[4]*T + Ai[r]*gamma[5]
            fix = fix + (Xi[:, 1] - Xr[:, 1])'*dt*Ai[r]*gamma[4]*T - Vr[:, 1]'*Ai[r]*gamma[5]
            fiy = fiy + (Xi[:, 2] - Xr[:, 2])'*dt*Ai[r]*gamma[4]*T - Vr[:, 2]'*Ai[r]*gamma[5]
        end
    end

    # fill auxiliary matrices
    Gixy = Gixy + TI'*gamma[6]*TI
    Githeta = Githeta + TI'*gamma[6]*TI
    fix = fix - Vi0[:, 1]'*gamma[6]*TI
    fiy = fiy - Vi0[:, 2]'*gamma[6]*TI
    fitheta = fitheta - Vi0[:, 3]'*gamma[6]*TI

    # declare optimization problem variables
    Ux = Convex.Variable(p)
    Uy = Convex.Variable(p)
    Utheta = Convex.Variable(p)

    # objective function
    problem = minimize(
        (1/2)*quadform([Ux; Uy; Utheta], [Gixy zeros(p, p) zeros(p, p); zeros(p, p) Gixy zeros(p, p); zeros(p, p) zeros(p, p) Githeta]) + [fix fiy fitheta]*[Ux; Uy; Utheta],
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
        problem.constraints += norm(x[i, 1 : 2] + [Ux[1] Uy[1]]'*dt - x[j, 1 : 2]) <= r_com[j]
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

