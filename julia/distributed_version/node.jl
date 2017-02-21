#==============================================================================
 = Topology Control Algorithm using Consensus and MPC (Distributed version)
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Nov 29 13:56:07
 = Info: This code is able to adapt the network topology to RSSI variations
 = and adjust the angle between the robots to reach the best connectivity
 =============================================================================#

# load external files
include("../opt.jl")            # tsp solution
include("../utils.jl")          # auxiliary functions
include("control.jl")           # motion control algorithms
include("communication.jl")     # network algorithms

# load external modules
using MAT                       # to save .mat files
using OPT                       # to use tsp solver
using CONTROL                   # to use motion control algorithms
using NETWORK                   # to use communication threads and udp sockets

#
#=
 = Constants
 =#

const UDP_PORT_BASE_SRC_80211   = 40000
const UDP_PORT_BASE_SRC_ACT     = 41000
const UDP_PORT_BASE_RX_ALL      = 50000
const UDP_PORT_BASE_TX_80211    = 60000
const UDP_PORT_BASE_TX_SENSOR   = 61000

#
#=
 = Load initial configurations
 =#

# check the parameter number
if length(ARGS) < 1
    print_with_color(:red, "ERROR: node index needed!\n")
    println("Usage: 'julia node.jl index'")
    exit(1)
end

# load index
id = parse(Int64, ARGS[1])

# number of robots
n = 4

# prediction horizon
p = 5

# time step (s)
dt = 0.6

# weights to the motion control algorithm
gamma = zeros(7)
gamma[1] = 1        # weight to desired position error
gamma[2] = 1        # weight to virtual neighbors velocity error
gamma[3] = 1        # weight to real neighbors velocity error
gamma[4] = 0        # weight to reference position error
gamma[5] = 0        # weight to reference velocity error
gamma[6] = 1        # weight to control effort
gamma[7] = 0

# saturation to linear control signal (m/s)
const vx_lim = [-1 1]

# saturation to angular control signal (rad/s)
const va_lim = [-1 1]

# control vector
# u = [ux uy uθ]'
u = zeros(Float32, n, 3)

# state vector
# x = [x y θ]
x = zeros(Float32, n, 3)

# state message
state_msg = MState()

# history array
com_hist = MHistory(n, 10)

# adjacency array
# A = [n]
A = zeros(UInt8, n)

# Hamiltonian array
H = zeros(UInt8, n)

# communication and coverage radius arrays
rcom = zeros(Float32, n); rcom[id] = Float32(100)
rcov = zeros(Float32, n); rcov[id] = Float32(24)

# 1-hop neighbors set
N1 = []

# 1-hop + 2-hop neighbors set
N2 = []

# destination array
dst = collect(1:n)
dst = dst[dst .!= id]

# transmission threads
tx_actuators = TXThread(dt, "127.0.0.1", UDP_PORT_BASE_SRC_ACT + id, UDP_PORT_BASE_RX_ALL + id)
tx_wifi = TXThread(0.1*id, "127.0.0.1", UDP_PORT_BASE_SRC_80211 + id, UDP_PORT_BASE_RX_ALL + dst)

# reception threads
rx_sensors = RXThread("127.0.0.1", UDP_PORT_BASE_TX_SENSOR + id)
rx_wifi = RXThread("127.0.0.1", UDP_PORT_BASE_TX_80211 + id)

# constant matrices to motion control
const T = tril(ones(Int8, p, p))
const TI = inv(T)

a = [0, 0]

# main loop
while true
    #
    #=
     = Communication Stuff (COM)
     ==========================================================================#

    # prior information about neighborhood
    N1_ = N1
    N2_ = N2

    x_ = x[id, 1:2]

    # clean 2-hop neighborhood
    N2 = []

    # send actuation
    tx_actuators.send_msg([num2bytes(Float32(u[id, 1]), false) ...   # ux (4 Bytes)
                           num2bytes(Float32(u[id, 2]), false) ...   # uy (4 Bytes)
                           num2bytes(Float32(0), false) ...      # uz (4 Bytes)
                           num2bytes(Float32(0), false)])        # uθ (4 Bytes)

    # send state message
    tx_wifi.send_msg(state_msg.state2bytes(id, N1, x[vcat(id, N1), :], u[vcat(id, N1), :], rcov[vcat(id, N1)], rcom[vcat(id, N1)]))

    # get sensors information
    sensor_msg = rx_sensors.get_msg(dt)[end]

    println("id:$(id) \nΔt:$(rx_sensors.dt)s")

    # convert sensor bytes array into position coordinates
    if sensor_msg != nothing
        x[id, 1] = bytes2num(sensor_msg[1:4, end], Float32, false)
        x[id, 2] = bytes2num(sensor_msg[5:8, end], Float32, false)
        x[id, 3] = bytes2num(sensor_msg[13:16, end], Float32, false)
    end

    # wait for message from neighbors
    wifi_msg = rx_wifi.get_msg(1)

    if wifi_msg != nothing && length(wifi_msg[1]) > 2
        for msg in wifi_msg
            # convert each byte message into state
            state_msg.bytes2state(msg)

            # mark received on history array
            com_hist.mark_hist(state_msg.id)

            # set sender as 1-hop neighbor
            prod(N1 .!= state_msg.id) ? N1 = vcat(N1, state_msg.id) : 0

            # remove sender from 2-hop neighborhood
            prod(N2 .== state_msg.id) ? N2 = N2[N2 .!= state_msg.id] : 0

            # get sender's state, control, coverage and communication
            x[state_msg.id, :] = state_msg.x[1, :]
            u[state_msg.id, :] = state_msg.u[1, :]
            rcom[state_msg.id] = state_msg.rcom[1]
            rcov[state_msg.id] = state_msg.rcov[1]

            for j = 1 : state_msg.n1_size
                if state_msg.n1[j] != id
                    # get 2-hop neighbors' state, control, coverage and communication
                    x[state_msg.n1[j], :] = state_msg.x[j + 1, :]
                    u[state_msg.n1[j], :] = state_msg.u[j + 1, :]
                    rcom[state_msg.n1[j]] = state_msg.rcom[j + 1]
                    rcov[state_msg.n1[j]] = state_msg.rcov[j + 1]

                    # set 2-hop neighbors
                    prod(N2 .!= state_msg.n1[j]) && prod(N1 .!= state_msg.n1[j]) ? N2 = vcat(N2, state_msg.n1[j]) : 0
                end
            end
        end
    end

    # remove mute nodes from 1-hop neighborhood
    for j in N1
        sum(com_hist.hist[j, :]) == 0 ? N1 = N1[N1 .!= j] : 0
    end

    # advance the time slot on history array
    com_hist.adv_time()

    #
    #=
     = Control Stuff (TSP + CMC)
     ==========================================================================#

    # clean and update current adjacent array
    A = zeros(UInt8, n)
    A[N1] = 1

    # check if the neighborhood has changed
    if (N1 != N1_ || N2 != N2_) && length(N2) > 0
        # 1-hop + 2-hop neighbor's set
        N = vcat(id, N1, N2)

        # distance matrix
        D = zeros(length(N), length(N))

        # fill a 1-hop + 2-hop neighborhood distance matrix
        for i = 1 : length(N) - 1
            for j = i + 1 : length(N)
                D[i, j] = D[j, i] = norm(x[N[i], 1:2] - x[N[j], 1:2])
            end
        end

        # get the Hamiltonian cycle from TSP's solution
        H_i = tsp(D)

        # clear hamiltonian cycle
        H = zeros(UInt8, n)

        # fill the Hamiltonian cycle array with H_i information
        for j = 1 : length(N)
            H[N[j]] = H_i[1, j]
        end
    elseif length(N2) == 0
        H = zeros(UInt8, n)
    end

    # connectivity motion control (CMC)
    u[id, :], c = hl_motion_control(id, A, H, zeros(n), T, TI, n, 0,
                                 x, u, rcom, rcov, dt, p, gamma,
                                 0, 0, vx_lim, va_lim)

    #=a = (u[id, 1:2] - (x[id, 1:2] - x_)/dt)*1=#

    for j in N1
        println("D$(id)$(j)=$(norm(x[id, 1:2] - x[j, 1:2]))")
    end

    println("N1:$(N1) \nN2:$(N2) \nH:$(H) \nx:$(x) \nu:$(u) \na:$(a)")
    println("------------------------------------------")
end

