#==============================================================================
 = Topology Control Algorithm using Consensus and MPC (Distributed version)
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Set 23 12:35:31
 = Info: This code is able to adapt the network topology to RSSI variations
 = and adjust the angle between the robots to reach the best connectivity
 =============================================================================#

# load external files
include("../opt.jl")            # tsp solution
include("../utils.jl")          # auxiliary functions
include("../control.jl")        # motion control algorithms
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

# transmission threads
tx_actuators = TXThread(1, "127.0.0.1", UDP_PORT_BASE_SRC_ACT + id, UDP_PORT_BASE_RX_ALL + id)
#=tx_wifi = TXThread(1, "127.0.0.1", UDP_PORT_BASE_SRC_80211 + id, UDP_PORT_BASE_RX_ALL + dst)=#

# reception threads
rx_sensors = RXThread("127.0.0.1", UDP_PORT_BASE_TX_SENSOR + id)
rx_wifi = RXThread("127.0.0.1", UDP_PORT_BASE_TX_80211 + id)

msg = [num2bytes(Float32(10), false) ...
       num2bytes(Float32(0), false) ...
       num2bytes(Float32(0), false) ...
       num2bytes(Float32(0), false)]

tx_actuators.send_msg(msg)

while true
    println(rx_sensors.get_msg(1))
    println("-------------------------")
    sleep(1)
    #=println("x:$(bytes2num(msg[1:4, size(msg, 2)], Float32, false)) y:$(bytes2num(msg[5:8, size(msg, 2)], Float32, false)) z:$(bytes2num(msg[9:12, size(msg, 2)], Float32, false)) θ:$(bytes2num(msg[13:16, size(msg, 2)], Float32, false))")=#

end

