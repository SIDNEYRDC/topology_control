#==============================================================================
 = Network Module for Topology Control Algorithm in Julia
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Jun 27 15:49:23
 = Info: This file contains the functions to access the network interface and
 = protocols by the Topology Control main algorithm.
 =============================================================================#

module NETWORK

# public functions
export omnet_interface_init, get_data, send_data, MData

#
#=
 = Global definitions
 =#

# byte positions of UDP packets
global const id_init            = 1
global const current_time_init  = id_init + 1
global const buffers_init       = current_time_init + 8

# udp sockets
global udps = UDPSocket()   # sender
global udpr = UDPSocket()   # receiver

# received message
global msgr = zeros(UInt8, 1)

# message reception indicator
global received = false

# message data type
type MData
    id::UInt8                   # sender id
    n1_size::UInt8                  # 1-hop neighborhood size
    n2_size::UInt8                  # 2-hop neighborhood size
    n1::Array{UInt8, 1}             # 1-hop neighborhood
    n2::Array{UInt8, 1}             # 2-hop neighborhood
    x::Array{Float32, 2}            # state array
    v::Array{Float32, 2}            # velocity array
    sim_pose::Array{Float32, 2}     # position array to omnet++
    sim_time::Float64               # simulation time
    pay_size::UInt16                # payload size
end

#
#=
 = Start Omnet++ interface
 = Info: starts a socket interface with RA-TDMA algorithm implemented in Omnet++
 =#
function omnet_interface_init(n, ip = "127.0.0.1", port = 50001)
    # associates the udp socket with omnet++ transmission port
    bind(udpr, IPv4(ip), port)

    # starts receiver thread
    @async begin
        while true
            global msgr = recv(udpr)
            global received = true
        end
    end

    # set the number of robots
    global n_bot = n

    # start global indicators of data begin
    global size_init        = buffers_init + 4*n
    global payload_init     = size_init + 2
end

function omnet_interface_close()
    # close udp connections
    close(udps)
    close(udpr)
end

#
#=
 = get_data function
 = Info: allows the package reception from omnet++ (it is blocking) and extract
 = the data according with the RA-TDMA implementation by Oliveira et al. (2015)
 = found in: doi.org/10.1109/WFCS.2015.7160566
 = Use: data = get_data(), where data is a MData struct.
 =#
function get_data()
    # data output
    data = MData(0, 0, 0, [0, 0], [0, 0], [0 0], [0 0], [0 0], 0, 0)

    # wait for omnet++ data
    while received == false
        sleep(0.001)
    end

    # reset received status
    global received = false

    # process the message package and extract the data
    data.id = msgr[id_init]                                                # sender id (1 Byte)
    data.sim_time = bytes2num(msgr[current_time_init + (0:7)])                 # actual simulation time in omnet++ (4 Bytes)
    buffers = convert(Array{UInt32, 1}, msgr[buffers_init + (0:4*n_bot - 1)])  # buffer state (4∙n Bytes)
    data.pay_size = bytes2num(msgr[size_init + (0:1)], UInt16)                 # payload size (2 Bytes)

    # if the payload size is greater than zero, gets the payload data
    if data.pay_size > 0
        # data indexes
        n1_size_init     = payload_init + 0
        n1_init          = n1_size_init + 1

        data.n1_size = msgr[n1_size_init]                      # get the size of 1-hop neighborhood (1 Byte)
        data.n1 = msgr[n1_init + (0:1*data.n1_size - 1)]       # get the 1-hop neighborhood (1∙|N₁| Bytes)

        # data indexes
        n2_size_init    = n1_init + 1*data.n1_size
        n2_init         = n2_size_init + 1

        data.n2_size = msgr[n2_size_init]                      # get the size of 2-hop neighborhood (1 Byte)
        data.n2 = msgr[n2_init + (0:1*data.n2_size - 1)]       # get the 2-hop neighborhood (1∙|N₂| Bytes)

        # data indexes
        x_init          = n2_init + 1*data.n2_size
        v_init          = x_init + 3*4*data.n2_size

        # allocate pose array (x[i, x | y | θ]) (4∙3∙|N₂| Bytes)
        data.x = zeros(Float32, data.n2_size, 3)

        # allocate velocity array (x[i, vx | vy | vθ]) (4∙3∙|N₂| Bytes)
        data.v = zeros(Float32, data.n2_size, 3)

        # auxiliary indexes
        xx_init = x_init
        xy_init = xx_init + 1*4
        xt_init = xx_init + 2*4
        vx_init = v_init
        vy_init = vx_init + 1*4
        vt_init = vx_init + 2*4

        for i = 1 : data.n2_size
            # get the position values
            data.x[i, 1] = bytes2num(msgr[xx_init + (0:3)])    # x coordinate (4 Bytes)
            data.x[i, 2] = bytes2num(msgr[xy_init + (0:3)])    # y coordinate (4 Bytes)
            data.x[i, 3] = bytes2num(msgr[xt_init + (0:3)])    # θ coordinate (4 Bytes)

            # get the velocity values
            data.v[i, 1] = bytes2num(msgr[vx_init + (0:3)])    # x coordinate (4 Bytes)
            data.v[i, 2] = bytes2num(msgr[vy_init + (0:3)])    # y coordinate (4 Bytes)
            data.v[i, 3] = bytes2num(msgr[vt_init + (0:3)])    # θ coordinate (4 Bytes)

            # increase auxiliary indexes
            xx_init = xx_init + 3*4
            xy_init = xx_init + 1*4
            xt_init = xx_init + 2*4
            vx_init = vx_init + 3*4
            vy_init = vx_init + 1*4
            vt_init = vx_init + 2*4
        end
    end

    return data
end

#
#=
 = send_data function
 = Info: allows the package sending to omnet++ from a abtract data type
 = Use: send_data(data, ip, port), where ip is a string with a destin ip
 = address, port is a integer value representing the port to send and
 = data is a MData struct.
 =#
function send_data(data::MData, ip = "127.0.0.1", port = 50001)
    # pose array in bytes
    x = zeros(UInt8, 1, 8*n_bot)

    xx_init = 1
    xy_init = 5

    # build the pose array
    for i = 1 : n_bot
        x[xx_init + (0:3)] = num2bytes(Float32(data.sim_pose[i, 1]))
        x[xy_init + (0:3)] = num2bytes(Float32(data.sim_pose[i, 2]))

        xx_init = xx_init + 2*4
        xy_init = xx_init + 1*4
    end

    # build the message package
    msg = [UInt8(0) ...                             # message type (0:sync, 1:payload) (1 Byte)
           UInt8(data.id) ...                       # sender id (1 Byte)
           num2bytes(Float64(data.sim_time)) ...    # simulation time (8 Bytes)
           UInt8(1) ...                             # active node flag (1:yes, 0:no) (1 Byte)
           x ...                                    # absolute positions to omnet++ (4∙2∙n Bytes)
           UInt8(data.n1_size)]                     # number of subsequent messages (1 Byte)

    # send the package
    send(udps, IPv4(ip), port, msg)

    # verify if there are payload messages
    for j in data.n1
        # pose array to payload (x[i, x | y | θ]) (4∙3∙|N₂| bytes)
        x = zeros(UInt8, 4*3*data.n2_size)

        # velocity array to payload (v[i, vx | vy | vθ]) (4∙3∙|N₂| bytes)
        v = zeros(UInt8, 4*3*data.n2_size)

        # auxiliary indexes
        xx_init = 1
        xy_init = 5
        xt_init = 9

        for i = 1 : data.n2_size
            # get the position values
            x[xx_init + (0:3)] = num2bytes(Float32(data.x[i, 1]))      # x coordinate (4 Bytes)
            x[xy_init + (0:3)] = num2bytes(Float32(data.x[i, 2]))      # y coordinate (4 Bytes)
            x[xt_init + (0:3)] = num2bytes(Float32(data.x[i, 3]))      # θ coordinate (4 Bytes)

            # get the velocity values
            v[xx_init + (0:3)] = num2bytes(Float32(data.v[i, 1]))      # x coordinate (4 Bytes)
            v[xy_init + (0:3)] = num2bytes(Float32(data.v[i, 2]))      # y coordinate (4 Bytes)
            v[xt_init + (0:3)] = num2bytes(Float32(data.v[i, 3]))      # θ coordinate (4 Bytes)

            # increase auxiliary indexes
            xx_init = xx_init + 3*4
            xy_init = xx_init + 1*4
            xt_init = xx_init + 2*4
        end

        # build the payload message
        payload = [UInt8(data.n1_size) ...                  # size of 1-hop neighborhood (1 Byte)
                   convert(Array{UInt8, 1}, data.n1) ...    # 1-hop neighborhood (1∙|N₁| Bytes)
                   UInt8(data.n2_size) ...                  # size of 2-hop neighborhood (1 Byte)
                   convert(Array{UInt8, 1}, data.n2) ...    # 2-hop neighborhood (1∙|N₂| Bytes)
                   x ...                                    # pose array (4∙3∙|N₂| Bytes)
                   v]                                       # velocity array (4∙3∙|N₂| Bytes)

        # build message package
        msg = [UInt8(1) ...                                 # message type (1:payload) (1 Byte)
               UInt8(j) ...                                 # destination id (1 Byte)
               num2bytes(UInt16(length(payload))) ...       # payload size (2 Bytes)
               payload]

        # send the package
        send(udps, IPv4(ip), port, msg)
    end
end

#
#=
 = Num2Bytes function
 = Info: converts a float number to an UInt8 array
 = Use: out = num2bytes(in), where in is a Float64 value and out is an array of
 = the type UInt8.
 =#
function num2bytes(input)
    return hex2bytes(num2hex(input))
end

#
#=
 = Bytes2Num function
 = Info: converts a UInt8 array to a number
 = Use: out = bytes2num(in, T), where in is an UInt8 array and T is the data
 = type of the output. If T is not specified then out is a Float64 number.
 =#
function bytes2num{T<:Number}(input, ::Type{T} = Float64)
    # verify if the input type is Float
    if T == Float32 || T == Float64
        return hex2num(bytes2hex(input))
    else
        return parse(T, bytes2hex(input), 16)
    end
end

end

