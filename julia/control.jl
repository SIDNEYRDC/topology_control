#==============================================================================
 = Control Module to Topology Control Algorithm in Julia
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2017 Feb 21 18:21:06
 = Info: This file contains the motion control algorithms used in the topology
 = control algorithm.
 =============================================================================#

module CONTROL

using Convex    # cvx interface to solve convex problems
using Gurobi    # to use Gurobi as optimization solver

# public functions
export cmc_init, mpc_1st_order, mpc_2nd_order

#
#=
 = Connectivity Motion Control initialization
 = Info: Starts the constant parameters used by the cmc controllers
 =#
function cmc_init(in_n, in_dk, in_p, in_gamma, in_u_min, in_u_max)
    # set global mpc parameters
    global const n = in_n
    global const dk = in_dk
    global const p = in_p
    global const gamma = in_gamma
    global const u_min = in_u_min
    global const u_max = in_u_max

    # constant matrices to motion control
    global const T = tril(ones(p, p))
    global const TI = inv(T)
    global const T2 = T^2
    global const P = diagm(collect(1 : p))
end

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
function mpc_1st_order(i, Ai, Hi, Di, Si, n_ref, x, v, r_cov, r_com, phi, RSSI_SENS)
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
    N1 = find(Ai[1 : n])

    # get the real and virtual neighbours of i
    N2 = find(max(Ai[1 : n], Hi))

    # find the centroid to the robot i and its neighbors
    ci = [sum(x[[i; N1], 1])/(length(N1) + 1); sum(x[[i; N1], 2])/(length(N1) + 1)]

    # calculate the desired angle theta
    #=dx = x[i, 1] - ci[1]=#
    #=dx != 0 ? dtheta = atan((x[i, 2] - ci[2])/dx) : dtheta = 0=#
    dtheta = 3pi/2;

    for j in N2
        # auxiliary matrices definition
        Vj = repmat(v[j, :]', p, 1)
        Sij = fill(10*phi*log(r_cov[i] + r_cov[j]) - abs(Si[j]), p, 1)

        # enable RSSI sensing
        RSSI_SENS == 1 ? dij = 10^(Si[j]/(-10*phi)) : dij = Di[j]

        # calculate the desired position to x and y coordinates
        dxy = (x[i, 1 : 2] - x[j, 1 : 2])*(r_cov[i] + r_cov[j])/dij + x[j, 1 : 2]

        # auxiliary matrix to desired position
        Xd = repmat([dxy' dtheta], p, 1)

        # term activation
        phi_ij = (1 - Ai[j])*Hi[j]
        psi_ij = Ai[j] + phi_ij

        # fill auxiliary matrices
        Gixy = Gixy + T'*dk*dk*psi_ij*gamma[1]/r_com[j]*T + phi_ij*gamma[2]/2*u_max[1] + (psi_ij - Hi[j])*gamma[3]/u_max[1] + T'*dk*dk*psi_ij*gamma[7]*T
        Githeta = Githeta + T'*dk*dk*psi_ij*gamma[1]*T
        fix = fix + (Xi[:, 1] - Xd[:, 1])'*dk*psi_ij*gamma[1]/r_com[j]*T + Vj[:, 1]'*phi_ij*gamma[2]/2*u_max[1] - Vj[:, 1]'*(psi_ij - Hi[j])*gamma[3]/u_max[1] - Sij'*dk*psi_ij*gamma[7]*T
        fiy = fiy + (Xi[:, 2] - Xd[:, 2])'*dk*psi_ij*gamma[1]/r_com[j]*T + Vj[:, 2]'*phi_ij*gamma[2]/2*u_max[2] - Vj[:, 2]'*(psi_ij - Hi[j])*gamma[3]/u_max[2] - Sij'*dk*psi_ij*gamma[7]*T
        fitheta = fitheta + (Xi[:, 3] - Xd[:, 3])'*dk*psi_ij*gamma[1]*T
    end

    # fill the auxiliary matrices for the references
    if n_ref > 0
        for r = n + 1 : n + n_ref
            # auxiliary vector to each reference
            Xr = repmat(x[r, :]', p, 1)
            Vr = repmat(v[r, :]', p, 1)

            Gixy = Gixy + T'*dk*dk*Ai[r]*gamma[4]*T + Ai[r]*gamma[5]
            fix = fix + (Xi[:, 1] - Xr[:, 1])'*dk*Ai[r]*gamma[4]*T - Vr[:, 1]'*Ai[r]*gamma[5]
            fiy = fiy + (Xi[:, 2] - Xr[:, 2])'*dk*Ai[r]*gamma[4]*T - Vr[:, 2]'*Ai[r]*gamma[5]
        end
    end

    # fill auxiliary matrices
    Gixy = Gixy + TI'*gamma[6]/u_max[1]*TI
    Githeta = Githeta + TI'*gamma[6]*TI
    fix = fix - Vi0[:, 1]'*gamma[6]/u_max[1]*TI
    fiy = fiy - Vi0[:, 2]'*gamma[6]/u_max[2]*TI
    fitheta = fitheta - Vi0[:, 3]'*gamma[6]*TI

    # declare optimization problem variables
    Ux = Convex.Variable(p)
    Uy = Convex.Variable(p)
    Utheta = Convex.Variable(p)

    # objective function
    problem = minimize(
        (1/2)*Convex.quadform([Ux; Uy; Utheta], [Gixy zeros(p, p) zeros(p, p); zeros(p, p) Gixy zeros(p, p); zeros(p, p) zeros(p, p) Githeta]) + [fix fiy fitheta]*[Ux; Uy; Utheta],
        # saturation constraints
        Ux >= fill(u_min[1], p),
        Uy >= fill(u_min[2], p),
        Utheta >= fill(u_min[3], p),
        Ux <= fill(u_max[1], p),
        Uy <= fill(u_max[2], p),
        Utheta <= fill(u_max[3], p)
    )

    # maximum distance between neighbours constraint
    for j in N1
        problem.constraints += norm(x[i, 1 : 2] + [Ux[1] Uy[1]]'*dk - x[j, 1 : 2]) <= r_com[j]
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

#
#=
 = Second Order MPC to motion control
 = Info: Move robots with 2nd order dynamics using MPC  algorithm
 =#
function mpc_2nd_order(i, x, v, u, Ai, Hi, Di, Si, r_cov, r_com)
    # auxiliary matrices definition
    Hixy = zeros(p, p)              # Hessian matrix to x and y coordinates
    Hitheta = zeros(p, p)           # Hessian matrix to yaw angle
    gix = zeros(1, p)               # gradient matrix to x coordinate
    giy = zeros(1, p)               # gradient matrix to y coordinate
    githeta = zeros(1, p)           # gradient matrix to yaw angle

    Xi = repmat(x[i, :]', p, 1)     # NOTE: after julia upgrade to 0.5 version, a subvector of a vector is ALWAYS a column vector.
    Vi = repmat(v[i, :]', p, 1)
    Ui0 = zeros(p, 3)
    Ui0[1, :] = u[i, :]

    # limits to velocity variation
    const v_max = u_max[1]*dk

    # get the 1-hop neighbours of i
    N1 = find(Ai[1 : n])

    # get the real and virtual neighbours of i (1-hop + 2-hop neighbours)
    N2 = find(max(Ai[1 : n], Hi))

    # get desired angle
    dtheta = 0

    for j in N2
        # auxiliary matrices definition
        Vj = repmat(v[j, :]', p, 1)

        # calculate the desired position to x and y coordinates
        dxy = (x[i, 1 : 2] - x[j, 1 : 2])*(r_cov[i] + r_cov[j])/Di[j]+ x[j, 1 : 2]

        # auxiliary matrix to desired position
        Xij = repmat([dxy' dtheta], p, 1)

        # term activation
        phi_ij = (1 - Ai[j])*Hi[j]          # activated for virtual neighbors
        psi_ij = Ai[j] + phi_ij             # activated for all neighbors (physical + virtual)

        # fill Hessian matrices for each neighbor
        Hixy += T2'*dk^4*gamma[1]/r_com[j]*psi_ij*T2        # penalize position error
        Hixy += T'*dk^2*gamma[2]/2*v_max*phi_ij*T           # penalize same direction velocities (on virtual neighbors)
        Hixy += T'*dk^2*gamma[3]/v_max*(psi_ij - Hi[j])*T   # penalize different direction velocities (on physical neighbors)
        Hitheta += T2'*dk^4*gamma[1]*psi_ij*T2

        # fill gradient matrices for each neighbor
        gix += (Xi[:, 1] - Xij[:, 1])'*dk^2*gamma[1]/r_com[j]*psi_ij*T2
        giy += (Xi[:, 2] - Xij[:, 2])'*dk^2*gamma[1]/r_com[j]*psi_ij*T2
        githeta += (Xi[:, 3] - Xij[:, 3])'*dk^2*gamma[1]*psi_ij*T2
        gix += Vi[:, 1]'*P'*dk^3*gamma[1]/r_com[j]*psi_ij*T2
        giy += Vi[:, 2]'*P'*dk^3*gamma[1]/r_com[j]*psi_ij*T2
        githeta += Vi[:, 3]'*P'*dk^3*gamma[1]*psi_ij*T2
        gix += (Vi[:, 1] + Vj[:, 1])'*dk*gamma[2]/2*v_max*phi_ij*T
        giy += (Vi[:, 2] + Vj[:, 2])'*dk*gamma[2]/2*v_max*phi_ij*T
        gix += (Vi[:, 1] - Vj[:, 1])'*dk*gamma[3]/v_max*(psi_ij - Hi[j])*T
        giy += (Vi[:, 2] - Vj[:, 2])'*dk*gamma[3]/v_max*(psi_ij - Hi[j])*T
    end

    # fill Hessian matrices
    Hixy += TI'*gamma[4]/u_max[1]*TI
    Hitheta += TI'*gamma[4]*TI

    # fill gradient matrices
    gix -= Ui0[:, 1]'*gamma[4]/u_max[1]*TI
    giy -= Ui0[:, 2]'*gamma[4]/u_max[1]*TI
    githeta -= Ui0[:, 3]'*gamma[4]*TI

    # declare optimization problem variables
    Ux = Convex.Variable(p)
    Uy = Convex.Variable(p)
    Utheta = Convex.Variable(p)

    # objective function
    problem = minimize(
        (1/2)*quadform([Ux; Uy; Utheta], [Hixy zeros(p, p) zeros(p, p); zeros(p, p) Hixy zeros(p, p); zeros(p, p) zeros(p, p) Hitheta]) + [gix giy githeta]*[Ux; Uy; Utheta],
        # saturation constraints
        Ux >= fill(u_min[1], p),
        Uy >= fill(u_min[2], p),
        Utheta >= fill(u_min[3], p),
        Ux <= fill(u_max[1], p),
        Uy <= fill(u_max[2], p),
        Utheta <= fill(u_max[3], p)
    )

    # maximum distance between neighbours constraint
    for j in N1
        problem.constraints += norm(x[i, 1 : 2] + v[i, 1 : 2]*dk + [Ux[1] Uy[1]]'*dk^2 - x[j, 1 : 2]) <= r_com[j]
    end

    # solve the optimization problem
    solve!(problem, GurobiSolver(OutputFlag = 0))

    # verify if the problem was solved
    if problem.status == :Optimal
        return [Ux.value[1] Uy.value[1] Utheta.value[1]]
    else
        return [0.0 0.0 0.0]
    end
end

end

