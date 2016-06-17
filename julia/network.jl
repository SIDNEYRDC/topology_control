#==============================================================================
 = Network Module to Topology Control Algorithm in Julia
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Jun 07 22:45:54
 = Info: This file contains the functions to access the network interface and
 = protocols by the Topology Control main algorithm.
 =============================================================================#

module NETWORK

# public functions
export omnet_interface_init, get_data, send_data

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

#
#=
 = Start Omnet++ interface
 = Info: starts a socket interface with RA-TDMA algorithm implemented in Omnet++
 =#
function omnet_interface_init(n, ip = "127.0.0.1", port = 50001)
    # associates the udp socket with omnet++ transmission port
    bind(udpr, IPv4(ip), port)

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
 = Use: data = get_data(), where data is a generic array from julia, defined as
 = follows: data[1] = sender id (UInt8)
 =          data[2] = current simulation time on omnet++ (Float64)
 =          data[3] = source message id (UInt8)
 =          data[4] = 1-hop neighborhood size (UInt8)
 =          data[5] = 1-hop neighborhood (Array{UInt8, 1, |N₁|})
 =          data[6] = 2-hop neighborhood size (UInt8)
 =          data[7] = 2-hop neighborhood (Array{UInt8, 1, |N₂|})
 =          data[8] = pose array (Array{Float32, |N₂|, 3})
 =          data[9] = velocity array (Array{Float32, |N₂|, 3})
 =#
function get_data()
    # wait for data from omnet++
    msg = recv(udpr)

    # generic data array
    data = cell(8)

    # process the message package and extract the data
    data[1] = msg[id_init]                                                      # sender id (1 Byte)
    data[2] = bytes2num(msg[current_time_init + (0 : 7)])                       # actual simulation time in omnet++ (4 Bytes)
    buffers = convert(Array{UInt32, 1}, msg[buffers_init + (0 : 4*n_bot - 1)])  # buffer state (4∙n Bytes)
    payload_size = bytes2num(msg[size_init + (0 : 1)])                          # payload size (1 Byte)

    println("--------------------------------------")
    println(msg)

    # if the payload size is greater than zero, gets the payload data
    if payload_size > 0
        # data indexes
        global src_init         = payload_init + 0
        global ptype_init       = src_init + 1
        global n1_size_init     = ptype_init + 1
        global n1_init          = n1_size_init + 1

        data[3] = msg[src_init]                          # get the source message id (1 Byte)
        msg_type = msg[ptype_init]                       # get the message type (1 Byte)
        data[4] = msg[n1_size_init]                      # get the size of 1-hop neighborhood (1 Byte)
        data[5] = msg[n1_init + (0 : 1*data[4] - 1)]     # get the 1-hop neighborhood (1∙|N₁| Bytes)

        # data indexes
        global n2_size_init    = n1_init + 1*data[4]
        global n2_init         = n2_size_init + 1

        data[6] = msg[n2_size_init + 1]                  # get the size of 2-hop neighborhood (1 Byte)
        data[7] = msg[n2_init + (0 : 1*data[6] - 1)]     # get the 2-hop neighborhood (1∙|N₂| Bytes)

        # data indexes
        global x_init          = n2_init + 1*data[6]
        global v_init          = x_init + 3*4*data[6]

        # allocate pose array (x[i, x | y | θ]) (4∙3∙|N₂| Bytes)
        data[8] = zeros(Float32, data[6], 3)

        # allocate velocity array (x[i, vx | vy | vθ]) (4∙3∙|N₂| Bytes)
        data[9] = zeros(Float32, data[6], 3)

        #=
         =for i = 0 : 12/3-1
         =    println("i:$(1+i) x:$(init+3*i) y:$(init+3*i+1) theta:$(init+3*i+2)")
         =end
         =#

        for i = 0 : data[6] - 1
            # get the position values
            data[8][i + 1, 1] = bytes2num(msg[x_init + 3*i])        # x coordinate (4 Bytes)
            data[8][i + 1, 2] = bytes2num(msg[x_init + 3*i + 1])    # y coordinate (4 Bytes)
            data[8][i + 1, 3] = bytes2num(msg[x_init + 3*i + 2])    # θ coordinate (4 Bytes)

            # get the velocity values
            data[9][i + 1, 1] = bytes2num(msg[v_init + 3*i])        # x coordinate (4 Bytes)
            data[9][i + 1, 2] = bytes2num(msg[v_init + 3*i + 1])    # y coordinate (4 Bytes)
            data[9][i + 1, 3] = bytes2num(msg[v_init + 3*i + 2])    # θ coordinate (4 Bytes)
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
 = data is a generic array from julia, defined as
 = follows: data[1]  = sender id (UInt8)
 =          data[2]  = current simulation time omnet++ (Float64)
 =          data[3]  = pose array to be send to omnet++ (Array{Float32, n, 2})
 =          data[4]  = number of subsequent messages (UInt8)
 =          data[5]  = destination id (UInt8)
 =          data[6]  = payload size (UInt16)
 =          data[7]  = source id (UInt8)
 =          data[8]  = message type (UInt8)
 =          data[9]  = 1-hop neighborhood size (UInt8)
 =          data[10] = 1-hop neighborhood (Array{UInt8, |N₁|})
 =          data[11] = 2-hop neighborhood size (UInt8)
 =          data[12] = 2-hop neighborhood (Array{UInt8, |N₂|})
 =          data[13] = position array (Array{Float32, |N₂|, 3})
 =          data[14] = velocity array (Array{Float32, |N₂|, 3})
 =#
function send_data(data, ip = "127.0.0.1", port = 50001)
    # pose array in bytes
    x = zeros(UInt8, 1, 8*n_bot)

    xx_init = 1
    xy_init = 5

    # build the pose array
    for i = 1 : n_bot
        x[xx_init + (0:3)] = num2bytes(Float32(data[3][i, 1]))
        x[xy_init + (0:3)] = num2bytes(Float32(data[3][i, 2]))

        xx_init = xx_init + 2*4
        xy_init = xx_init + 1*4
    end

    # build the message package
    msg = [UInt8(0) ...                      # message type (0:sync, 1:payload) (1 Byte)
           UInt8(data[1]) ...                # sender id (1 Byte)
           num2bytes(Float64(data[2])) ...   # simulation time (8 Bytes)
           UInt8(1) ...                      # active node flag (1:yes, 0:no) (1 Byte)
           x ...                             # absolute positions to omnet++ (4∙2∙n Bytes)
           UInt8(data[4])]                   # number of subsequent messages (1 Byte)

    # send the package
    send(udps, IPv4(ip), port, msg)

    # verify if there are payload messages
    for m = 1 : data[4]
        # pose array to payload (x[i, x | y | θ]) (4∙3∙|N₂| bytes)
        x = zeros(UInt8, 4*3*data[11])

        # velocity array to payload (v[i, vx | vy | vθ]) (4∙3∙|N₂| bytes)
        v = zeros(UInt8, 4*3*data[11])

        xx_init = 1
        xy_init = 5
        xt_init = 9

        for i = 1 : data[11]
            # get the position values
            x[xx_init + (0:3)] = num2bytes(Float32(data[13][i, 1]))      # x coordinate (4 Bytes)
            x[xy_init + (0:3)] = num2bytes(Float32(data[13][i, 2]))      # y coordinate (4 Bytes)
            x[xt_init + (0:3)] = num2bytes(Float32(data[13][i, 3]))      # θ coordinate (4 Bytes)

            # get the velocity values
            v[xx_init + (0:3)] = num2bytes(Float32(data[14][i, 1]))      # x coordinate (4 Bytes)
            v[xy_init + (0:3)] = num2bytes(Float32(data[14][i, 2]))      # y coordinate (4 Bytes)
            v[xt_init + (0:3)] = num2bytes(Float32(data[14][i, 3]))      # θ coordinate (4 Bytes)

            xx_init = xx_init + 3*4
            xy_init = xx_init + 1*4
            xt_init = xx_init + 2*4
        end

        # build the payload message
        payload = [UInt8(data[7]) ...                       # source id (1 Byte)
                   UInt8(data[8]) ...                       # message type (1 Byte)
                   UInt8(data[9]) ...                       # size of 1-hop neighborhood (1 Byte)
                   convert(Array{UInt8, 1}, data[10]) ...   # 1-hop neighborhood (1∙|N₁| Bytes)
                   UInt8(data[11]) ...                      # size of 2-hop neighborhood (1 Byte)
                   convert(Array{UInt8, 1}, data[12]) ...   # 2-hop neighborhood (1∙|N₂| Bytes)
                   x ...                                    # pose array (4∙3∙|N₂| Bytes)
                   v]                                       # velocity array (4∙3∙|N₂| Bytes)

        println("payload size (Bytes):$(length(payload))")

        # build message package
        msg = [UInt8(1) ...                                 # message type (1:payload) (1 Byte)
               UInt8(data[5]) ...                           # destination id (1 Byte)
               num2bytes(UInt16(data[6])) ...               # payload size (2 Bytes)
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
 = Info: converts a UInt8 array to a Float64 number
 = Use: out = bytes2num(in), where in is an UInt8 array and out is a Float64.
 =#
function bytes2num(input)
    return hex2num(bytes2hex(input))
end

end

