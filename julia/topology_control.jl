#!/usr/bin/env julia

#==============================================================================
 = Topology Control Algorithm using Consensus and MPC
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2020 Abr 11 18:37:23
 = Info: This code is able to adapts the network topology to RSSI variations
 = and adjust the angle between the robots to reach the best connectivity
 =============================================================================#

# load external files
include("opt.jl")               # tsp solution
include("control.jl")           # motion control algorithms
include("utils.jl")             # auxiliary functions
include("network.jl")           # network interface
include("file.jl")              # external files access

# load external modules
using MAT                       # to save .mat files
using OPT                       # to use tsp solver
using CONTROL                   # to use motion control algorithms
using NETWORK                   # to use omnet++ interface
using FILE                      # to read configuration files

#
#=
 = Load initial configurations
 =#

# check the parameter number
if !isinteractive() && length(ARGS) < 1
    print_with_color(:red, "ERROR: configuration file expected!\n")
    println("Usage: 'julia topology_control.jl topology.conf'")
    exit(1)

elseif !isinteractive()
    filename = ARGS[1]
end

# read configuration file
cfg = read_conf(string("../conf/", filename))

# initial positions
# x(i | n_bot + r, [x y θ], t)
x = zeros(cfg.n_bot + cfg.n_ref, 3, cfg.n_iter)
x[1:cfg.n_bot, :, 1] = cfg.x

# initial velocities
# v(i | n_bot + r, [vx vy vθ], t)
v = zeros(cfg.n_bot + cfg.n_ref, 3, cfg.n_iter)
v[1:cfg.n_bot, :, 1] = cfg.v

# control array
# u(i | n_bot + r, [vx vy vθ], t)
u = zeros(cfg.n_bot, 3, cfg.n_iter)

# initial communication radius
# rcom(i, rcom, t)
r_com = repmat(cfg.r_com', cfg.n_iter)'

# initial coverage radius
# rcov(i, rcov, t)
r_cov = repmat(cfg.r_cov', cfg.n_iter)'

# initial safety radius
r_sec = repmat(cfg.r_sec', cfg.n_iter)'

#
#=
 = Data matrices
 =#

# centroid array
# c(i, 2, t)
c = zeros(cfg.n_bot, 2, cfg.n_iter)

# topology adjacency matrix
# A(i, j | n_bot + r, t)
A = zeros(UInt8, cfg.n_bot, cfg.n_bot + cfg.n_ref, cfg.n_iter)

# knowledge about the reference
#=A[6, n_bot + 1, :] = fill(1, n_iter)=#

# euclidian distance matrix
# D(i, j, t)
D = zeros(cfg.n_bot, cfg.n_bot, cfg.n_iter);

# RSSI readings matrix (dBm)
# S(i, j, t)
S = fill(-90.0, cfg.n_bot, cfg.n_bot, cfg.n_iter)

# RSSI filtered readings (dBm)
# Sf(i, j, t)
Sf = zeros(cfg.n_bot, cfg.n_bot, cfg.n_iter)

# zero-mean white Gaussian noise matrix
# G(i, j, t)
srand(1); const G = cfg.sigma*randn(cfg.n_bot, cfg.n_bot, cfg.n_iter)

# RSSI attenuation coefficient matrix (dBm)
# R(i, j)
srand(2); R = cfg.mu*tril(rand(cfg.n_bot, cfg.n_bot))
R = R + R'

# Hamiltonian's cycle matrix
# H(i, j, t)
H = zeros(UInt8, cfg.n_bot, cfg.n_bot, cfg.n_iter)

#
#=
 = Main loop
 =#

# start cmc module
cmc_init(cfg.n_bot, cfg.dt, cfg.ph, cfg.gamma, cfg.u_min, cfg.u_max)

# enable omnet++ interface
cfg.omnet == 1 ? omnet_interface_init(cfg.n_bot) : 0

# start timer counter
tic()

for t = 1 : cfg.n_iter

    # communication loop
    for i = 1 : cfg.n_bot - 1
        for j = i + 1 : cfg.n_bot
            # detect and create links
            if cfg.top_type == 1
                # euclidean distance between i and j
                D[i, j, t] = D[j, i, t] = norm(x[i, 1 : 2, t] - x[j, 1 : 2, t])

                #=i == 1 && t > 80 && t < 250 ? R[i, j] = R[j, i] = -5 : R[i, j] = R[j, i] = 0=#

                # fill RSSI matrix using the model: Sij = - 10 ∙ Φ ∙ log(dij) + C
                # can be found in: doi.org/10.1109/ICIT.2013.6505900
                S[i, j, t] = - 10*cfg.phi*log10(D[i, j, t]) + R[i, j] + G[i, j, t]*cfg.rssi_noise
                S[j, i, t] = - 10*cfg.phi*log10(D[i, j, t]) + R[j, i] + G[j, i, t]*cfg.rssi_noise

                #=
                 = Filter to RSSI readings (exponential smoothing, must be improved)
                 =#
                # filter factor
                ff = 5

                # RSSI readings filter
                if t >= ff
                    alpha = 0.2

                    for tt = 0 : t - 1
                        Sf[i, j, t] += alpha*(1 - alpha)^tt*S[i, j, t-tt]
                        Sf[j, i, t] += alpha*(1 - alpha)^tt*S[j, i, t-tt]
                    end

                    Sf[i, j, t] += (1 - alpha)^t*S[i, j, 1]
                    Sf[j, i, t] += (1 - alpha)^t*S[j, i, 1]
                else
                    Sf[i, j, t] = S[i, j, t]
                    Sf[j, i, t] = S[j, i, t]
                end

                # update adjacency matrix
                if D[i, j, t] <= r_com[j, t] && D[i, j, t] <= r_com[i, t]
                    A[i, j, t] = A[j, i, t] = 1
                elseif D[i, j, t] <= r_com[j, t] && D[i, j, t] > r_com[i, t]
                    A[i, j, t] = 1
                    A[j, i, t] = 0
                elseif D[i, j, t] > r_com[j, t] && D[i, j, t] <= r_com[i, t]
                    A[i, j, t] = 0
                    A[j, i, t] = 1
                elseif D[i, j, t] > r_com[j, t] && D[i, j, t] > r_com[i, t]
                    A[i, j, t] = A[j, i, t] = 0
                end

                # use the RSSI threshold to change the adjacency matrix
                if A[i, j, t] == 1 && S[i, j, t] < cfg.rssi_lim && S[j, i, t] < cfg.rssi_lim
                    A[i, j, t] = A[j, i, t] = 0
                elseif A[i, j, t] == 1 && S[i, j, t] < cfg.rssi_lim && S[j, i, t] >= cfg.rssi_lim
                    A[i, j, t] = 0
                elseif A[j, i, t] == 1 && S[i, j, t] >= cfg.rssi_lim && S[j, i, t] < cfg.rssi_lim
                    A[j, i, t] = 0
                end

            # keep the adjacency matrix state
            elseif cfg.top_type == 0 && t > 1
                A[i, j, t] = A[i, j, t - 1]
                A[j, i, t] = A[j, i, t - 1]
            end
        end
    end

    # process timeout
    for i in find(cfg.timeout .!= Inf)
        if t > cfg.timeout[i]
            A[i, :, t] = zeros(1, cfg.n_bot)
            A[:, i, t] = zeros(cfg.n_bot, 1)
        end
    end

    # omnet++ package received indicator
    omnet_rec = zeros(UInt8, cfg.n_bot)

    # omnet++ communication loop
    while cfg.omnet == 1 && sum(omnet_rec) < cfg.n_bot
        # wait for omnet++ data
        in_msg = get_data()

        # identify who robot receives omnet++ message with payload
        in_msg.pay_size > 0 ? omnet_rec[in_msg.id] = 1 : 0

        # 1-hop neighbourhood
        N1 = find(A[in_msg.id, 1 : cfg.n_bot, t])

        # 2-hop neighbourhood
        N2 = neighbourhood(A[:, 1 : cfg.n_bot, t], in_msg.id, 2)

        # build the output message
        out_msg = MData(in_msg.id,
                        length(N1),
                        length(N2),
                        N1, N2,
                        x[N2, :, t],
                        v[N2, :, t],
                        x[:, 1 : 2, t],
                        in_msg.sim_time, 0)

        # send message to omnet++ interface
        send_data(out_msg, "127.0.0.1", 50001 + in_msg.id)
    end

    # control loop
    for i = 1 : cfg.n_bot

        # check if the i's near neighbourhood has changed
        if t == 1 || sum(A[i, 1 : cfg.n_bot, t] - A[i, 1 : cfg.n_bot, t - 1]) != 0
            # get the neighbourhood from i (2-hop)
            global N = neighbourhood(A[:, 1 : cfg.n_bot, t], i, 2)

            # calculate the TSP's solution according with the information scope
            # local knowledge (2-hop)
            if cfg.info_scope == 0
                if length(N) > 2
                    # reduce the distance matrix to i's neighbourhood and get
                    # the Hamiltonian cycle from TSP's solution
                    H_i = tsp(matreduce(D[:, :, t], N))

                    # fill the Hamiltonian's cycle matrix with H_i line
                    for j = 1 : length(N)
                        H[i, N[j], t] = H_i[1, j]
                    end
                else
                    H[i, :, t] = zeros(1, cfg.n_bot)
                end

            # global knowledge
            elseif cfg.info_scope == 1
                H_i = tsp(D[:, :, t])
                H[:, :, t] = H_i
            end
        else
            # keep the earlier Hamiltonian matrix state to the actual iteration
            H[i, :, t] = H[i, :, t - 1]
        end

        if t != cfg.n_iter && length(N) > 1
            # solve 1st order connectivity motion control (CMC)
            v[i, :, t + 1], c[i, :, t + 1] = mpc_1st_order(i, A[i, :, t],
                                                              H[i, :, t],
                                                              D[i, :, t],
                                                              Sf[i, :, t],
                                                              cfg.n_ref,
                                                              x[:, :, t],
                                                              v[:, :, t],
                                                              cfg.c_sec,
                                                              r_sec[:, t],
                                                              r_cov[:, t],
                                                              r_com[:, t],
                                                              cfg.phi,
                                                              cfg.rssi)

            # solve 2nd order connectivity motion control (CMC)
#=            u[i, :, t + 1] = mpc_2nd_order(i, x[:, :, t], v[:, :, t], u[:, :, t],=#
                                           #=A[i, :, t], H[i, :, t], D[i, :, t],=#
                                           #=S[i, :, t], r_cov[:, t], r_com[:, t])=#

            # integrate the velocity
            #=v[i, :, t + 1] =  v[i, :, t] + u[i, :, t + 1]*cfg.dt=#

            # integrate the position
            x[i, :, t + 1] = x[i, :, t] + v[i, :, t + 1]*cfg.dt

        elseif t != cfg.n_iter
            # keep the older position if there are no neighbors
            x[i, :, t + 1] = x[i, :, t]
        end
    end

    println("iter = $(t)")
end

# finish timer counter
toc()

#
#=
 = Save simulation data
 =#

# create .mat file
file = matopen(join(["../matlab/sim", cfg.n_bot, cfg.n_iter, "mat"], "-", "."), "w")

# write data into .mat file
write(file, "N", Float32(cfg.n_iter))
write(file, "n", Float32(cfg.n_bot))
write(file, "nr", Float32(cfg.n_ref))
write(file, "h", Float32(cfg.dt))
write(file, "p", Int32(cfg.ph))
write(file, "rssi_lim", cfg.rssi_lim)
write(file, "R", R)
write(file, "x_data", x)
write(file, "v_data", v)
write(file, "u_data", u)
write(file, "r_com", r_com)
write(file, "r_cov", r_cov)
write(file, "r_sec", r_sec)
write(file, "A_data", max.(A[:, 1 : cfg.n_bot, :], H))
write(file, "H_data", H)
write(file, "S_data", S)
write(file, "Sf_data", Sf)
write(file, "G_data", G)
write(file, "D_data", D)
write(file, "c_data", c)

# close .mat file
close(file)

