#==============================================================================
 = Topology Control Algorithm using Consensus and MPC
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Jul 04 18:09:49
 = Info: This code is able to adapts the network topology to RSSI variations
 = and adjust the angle between the robots to reach the best connectivity
 =============================================================================#

# load external files
include("opt.jl")               # tsp solution
include("utils.jl")             # auxiliary functions
include("control.jl")           # motion control algorithms
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
if length(ARGS) < 1
    print_with_color(:red, "ERROR: configuration file expected!\n")
    println("Usage: 'julia topology_control.jl topology.conf'")
    exit(1)
end

# read configuration file
cfg = read_conf(ARGS[1])

# initial positions
# x(i | n_bot + r, [x y θ], t)
x = zeros(cfg.n_bot + cfg.n_ref, 3, cfg.n_iter)
x[1:cfg.n_bot, :, 1] = cfg.x

# initial velocities
# v(i | n_bot + r, [vx vy vθ], t)
v = zeros(cfg.n_bot + cfg.n_ref, 3, cfg.n_iter)
v[1:cfg.n_bot, :, 1] = cfg.v

# initial communication radius
# rcom(i, rcom, t)
r_com = repmat(cfg.r_com', cfg.n_iter)'

# initial coverage radius
# rcov(i, rcov, t)
r_cov = repmat(cfg.r_cov', cfg.n_iter)'

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

# constant matrices to motion control
const T = tril(ones(Int8, cfg.ph, cfg.ph))
const TI = inv(T)

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

        # check if the i's near neighbourhood was changed
        if t == 1 || sum(A[i, 1 : cfg.n_bot, t] - A[i, 1 : cfg.n_bot, t - 1]) != 0
            # get the neighbourhood from i (2-hop)
            N = neighbourhood(A[:, 1 : cfg.n_bot, t], i, 2)

            # reduce the distance matrix to i's neighbourhood and get
            # the Hamiltonian cycle from TSP's solution
            H_i = tsp(matreduce(D[:, :, t], N))

            # fill the Hamiltonian's cycle matrix with H_i line
            for j = 1 : length(N)
                H[i, N[j], t] = H_i[1, j]
            end
        else
            # keep the earlier Hamiltonian matrix state to the actual iteration
            H[i, :, t] = H[i, :, t - 1]
        end

        if t != cfg.n_iter
            # high level motion control
            v[i, :, t + 1], c[i, :, t + 1] = hl_motion_control(i, A[:, :, t],
                                                               H[:, :, t],
                                                               D[:, :, t],
                                                               S[i, :, t], T,
                                                               TI, cfg.n_bot,
                                                               cfg.n_ref,
                                                               x[:, :, t],
                                                               v[:, :, t],
                                                               r_com[:, t],
                                                               r_cov[:, t],
                                                               cfg.dt, cfg.ph,
                                                               cfg.gamma,
                                                               cfg.phi,
                                                               cfg.rssi,
                                                               cfg.vx_lim,
                                                               cfg.va_lim)

            # upgrade the position
            x[i, :, t + 1] = x[i, :, t] + v[i, :, t + 1]*cfg.dt
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
write(file, "N", cfg.n_iter)
write(file, "n", cfg.n_bot)
write(file, "nr", cfg.n_ref)
write(file, "h", cfg.dt)
write(file, "p", cfg.ph)
write(file, "rssi_lim", cfg.rssi_lim)
write(file, "R", R)
write(file, "x_data", x)
write(file, "v_data", v)
write(file, "r_com", r_com)
write(file, "r_cov", r_cov)
write(file, "A_data", max(A[:, 1 : cfg.n_bot, :], H))
write(file, "S_data", S)
write(file, "G_data", G)
write(file, "D_data", D)
write(file, "c_data", c)

# close .mat file
close(file)

