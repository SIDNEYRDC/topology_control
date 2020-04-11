#==============================================================================
 = Network Module for Topology Control Algorithm in Julia
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Out 11 14:04:17
 = Info: This file contains the functions to access the network interface and
 = protocols by the Topology Control main algorithm.
 =============================================================================#

module NETWORK

#= public functions =#
export RXThread, TXThread, num2bytes, bytes2num, MState, MHistory, wait_sim, get_simtime

#
#=
 = Global definitions
 =#

# message history
type MHistory
    #= variables =#
    hist::Array{Bool, 2}            # history array
    t_slot::UInt64                  # time slot

    #= functions =#
    mark_hist::Function             # mark received message to a node
    adv_time::Function              # advance with the current time slot

    #= constructor =#
    function MHistory(nodes_num, window_size)
        # start new object of MHistory type
        this = new()

        # create the history array
        this.hist = zeros(Bool, nodes_num, window_size)

        # time slot
        this.t_slot = 1

        # mark received message to a node
        this.mark_hist = function(id::UInt8)
            this.hist[id, this.t_slot] = true
        end

        # advance with the current time slot
        this.adv_time = function()
            this.t_slot < window_size ? this.t_slot += 1 : this.t_slot = 1

            # set as false all history for this time slot
            this.hist[:, this.t_slot] = zeros(Bool, nodes_num)
        end

        return this
    end
end

# state message data type
type MState
    #= variables =#
    id::UInt8                       # sender id (1 Byte)
    n1_size::UInt8                  # 1-hop neighborhood size (1 Byte)
    n1::Array{UInt8, 1}             # 1-hop neighborhood (n1_size ∙ 1 Bytes)
    x::Array{Float32, 2}            # state array ((1 + n1_size) ∙ 3 ∙ 4 Bytes)
    u::Array{Float32, 2}            # control array ((1 + n1_size) ∙ 3 ∙ 4 Bytes)
    rcov::Array{Float32, 1}         # coverage radius array ((1 + n1_size) ∙ 4 Bytes)
    rcom::Array{Float32, 1}         # communication radius array((1 +n1_size) ∙ 4 Bytes)

    #= functions =#
    bytes2state::Function           # return bytes from state
    state2bytes::Function           # return state from bytes

    #= constructor =#
    function MState()
        # start new object of MState type
        this = new()

        # return state data from bytes input
        this.bytes2state = function(byte_array::Array{UInt8, 1})
            # get id (1 Byte)
            this.id = byte_array[1]

            # get 1-hop neighborhood size (1 Byte)
            this.n1_size = byte_array[2]

            # get 1-hop neighborhood (n1_size ∙ 1 Bytes)
            this.n1_size > 0 ? this.n1 = byte_array[3 + (0:1*this.n1_size - 1)] : this.n1 = []

            # auxiliary indexes
            xx_init = 3 + this.n1_size
            xy_init = xx_init + 1*4
            xt_init = xx_init + 2*4
            ux_init = xx_init + 3*4*(this.n1_size + 1)
            uy_init = ux_init + 1*4
            ut_init = ux_init + 2*4
            rcov_init = ux_init + 3*4*(this.n1_size + 1)
            rcom_init = rcov_init + 1*4*(this.n1_size + 1)

            # initialize data arrays
            this.x = zeros(this.n1_size + 1, 3)
            this.u = zeros(this.n1_size + 1, 3)
            this.rcov = zeros(this.n1_size + 1)
            this.rcom = zeros(this.n1_size + 1)

            for i = 1 : this.n1_size + 1
                # get the position array
                this.x[i, 1] = bytes2num(byte_array[xx_init + (0:3)]) # x (4 Bytes)
                this.x[i, 2] = bytes2num(byte_array[xy_init + (0:3)]) # y (4 Bytes)
                this.x[i, 3] = bytes2num(byte_array[xt_init + (0:3)]) # θ (4 Bytes)

                # get the control array
                this.u[i, 1] = bytes2num(byte_array[ux_init + (0:3)]) # ux (4 Bytes)
                this.u[i, 2] = bytes2num(byte_array[uy_init + (0:3)]) # uy (4 Bytes)
                this.u[i, 3] = bytes2num(byte_array[ut_init + (0:3)]) # uθ (4 Bytes)

                # get coverage radius and communication arrays
                this.rcov[i] = bytes2num(byte_array[rcov_init + (0:3)])
                this.rcom[i] = bytes2num(byte_array[rcom_init + (0:3)])

                # increase auxiliary indexes
                xx_init += 3*4
                xy_init += 3*4
                xt_init += 3*4
                ux_init += 3*4
                uy_init += 3*4
                ut_init += 3*4
                rcov_init += 1*4
                rcom_init += 1*4
            end

            return this.id, this.n1_size, this.n1, this.x, this.u, this.rcov, this.rcom
        end

        # return a byte array from state input
        this.state2bytes = function(id, n1, x, u, rcov, rcom)
            # fill variables
            this.id = id
            this.n1_size = length(n1)
            this.n1 = n1
            this.x = x
            this.u = u
            this.rcov = rcov
            this.rcom = rcom

            # bytes array of pose and control array
            x_bytes = zeros(UInt8, 4*3*(this.n1_size + 1))
            u_bytes = zeros(UInt8, 4*3*(this.n1_size + 1))
            rcov_bytes = zeros(UInt8, 4*(this.n1_size + 1))
            rcom_bytes = zeros(UInt8, 4*(this.n1_size + 1))

            # auxiliary indexes
            x_init = 1
            y_init = 5
            t_init = 9
            rcov_init = 1
            rcom_init = 1

            for i = 1 : this.n1_size + 1
                # get the position values
                x_bytes[x_init + (0:3)] = num2bytes(Float32(x[i, 1])) # x (4 Bytes)
                x_bytes[y_init + (0:3)] = num2bytes(Float32(x[i, 2])) # y (4 Bytes)
                x_bytes[t_init + (0:3)] = num2bytes(Float32(x[i, 3])) # θ (4 Bytes)

                # get the control values
                u_bytes[x_init + (0:3)] = num2bytes(Float32(u[i, 1])) # ux (4 Bytes)
                u_bytes[y_init + (0:3)] = num2bytes(Float32(u[i, 2])) # uy (4 Bytes)
                u_bytes[t_init + (0:3)] = num2bytes(Float32(u[i, 3])) # uθ (4 Bytes)

                # get coverage and communication radius arrays
                rcov_bytes[rcov_init + (0:3)] = num2bytes(Float32(rcov[i]))
                rcom_bytes[rcom_init + (0:3)] = num2bytes(Float32(rcom[i]))

                # increase auxiliary indexes
                x_init += 3*4
                y_init += 3*4
                t_init += 3*4
                rcov_init += 1*4
                rcom_init += 1*4
            end

            # build the byte array
            byte_array = [UInt8(id) ...
                          UInt8(this.n1_size) ...
                          convert(Array{UInt8, 1}, n1) ...
                          x_bytes ...
                          u_bytes ...
                          rcov_bytes ...
                          rcom_bytes]

            return byte_array
        end

        return this
    end
end

#= thread to receive udp messages =#
type RXThread
    #= variables =#
    new_msg::Bool                      # indicator for received messages
    run::Bool                          # indicator to stop the thread
    msg_pool::Array{Any, 1}            # received messages buffer
    dt::Float32                        # time difference between get_msg calls

    #= functions =#
    get_msg::Function                  # get received message buffer
    stop::Function                     # to stop the thread

    #= constructor =#
    function RXThread(ip = "127.0.0.1", port = 50000)
        # start new object of RXThread type
        this = new()

        # udp socket
        udpr = UDPSocket()

        # received message
        this.msg_pool = []

        # associate udp socket with the desired port and ip
        bind(udpr, IPv4(ip), port)

        # set new message indicator as false
        this.new_msg = false

        # set thread run indicator as true
        this.run = true

        # time when the message pool is take
        t0 = get_simtime()

        # get message locker
        get_msg_sem = false

        # start receiver thread
        thread = @async begin
            while this.run == true
                # get the message from udp socket
                msg = recv(udpr)

                # set new message indicator as true
                this.new_msg = true

                # wait get message semaphore be free
                while get_msg_sem; end

                # analyse the message and store it in the message buffer
                if isempty(this.msg_pool) || this.msg_pool[end] != msg
                    resize!(this.msg_pool, length(this.msg_pool) + 1)
                    this.msg_pool[end] = msg
                end
            end
        end

        #= get received message =#
        this.get_msg = function(timeout)
            # waiting time
            time = 0

            # wait for a new message
            while this.new_msg == false && this.run == true
                # wait a time
                wait_sim(0.001)
                #=sleep(0.001)=#

                # increase the waiting time
                time += 0.001

                # if the waiting time is bigger than timeout, exit
                if time > timeout
                    return
                end
            end

            # set new message indicator as false
            this.new_msg = false

            # lock message get semaphore
            get_msg_sem = true

            # copy message buffer to a temporary array
            msg = this.msg_pool

            # get the current time from simulator
            t1 = get_simtime()

            # calculate the time difference between now and the last call
            this.dt = (t1 - t0)*1e-6

            # update last call time
            t0 = t1

            # clean msg buffer
            this.msg_pool = []

            # unlock get message semaphore
            get_msg_sem = false

            return msg
        end

        #= to stop the thread =#
        this.stop = function()
            # set run indicator to false
            this.run = false

            # wait for thread response
            wait(thread)

            # close the upd socket
            close(udpr)
        end

        return this
    end
end

#= thread to send upd messages =#
type TXThread
    #= variables =#
    msg::Array{UInt8, 1}                # message to be sent
    run::Bool                           # indicator to stop the thread

    #= functions =#
    send_msg::Function                  # send a message
    stop::Function                      # to stop the thread

    #= constructor =#
    function TXThread(dt = 0.1, ip = "127.0.0.1", src_port = 50000, dst_port = [50000])
        # start new object of TXThread type
        this = new()

        # udp socket
        udps = UDPSocket()

        # associate udp socket with the desired port and ip
        bind(udps, IPv4(ip), src_port)

        # initialize msg with null
        this.msg = [0]

        # set thread run indicator as true
        this.run = true

        # start sender thread
        thread = @async begin
            while this.run == true
                # send the message to all destinations
                for port in dst_port
                    send(udps, IPv4(ip), port, this.msg)

                    # wait a time between transmissions
                    wait_sim(dt)
                    #=sleep(dt)=#
                end
            end
        end

        #= send a message =#
        this.send_msg = function(msg::Array{UInt8, 1})
            this.msg = msg
        end

        #= to stop the thread =#
        this.stop = function()
            # set run indicator to false
            this.run = false

            # wait for thread response
            wait(thread)

            # close the upd socket
            close(udps)
        end

        return this
    end
end

#
#=
 = Num2Bytes function
 = Info: converts a float number into an UInt8 array
 = Use: out = num2bytes(in), where in is a Float64 value and out is an array of
 = the type UInt8. If bigend is false, a little endian representation (DCBA)
 = will be used instead of big endian representation (ABCD).
 =#
function num2bytes(input, bigend = true)
    if bigend == false
        return reverse(hex2bytes(num2hex(input)))
    else
        return hex2bytes(num2hex(input))
    end
end

#
#=
 = Bytes2Num function
 = Info: converts a UInt8 array to a number
 = Use: out = bytes2num(in, T), where in is an UInt8 array and T is the data
 = type of the output. If T is not specified then out is a Float64 number. If
 = bigend is false, a little endian representation (DCBA) will be used instead
 = of big endian representation (ABCD).
 =#
function bytes2num{T<:Number}(input, ::Type{T} = Float64, bigend = true)
    # verify the endian-ness
    bigend == false ? input = reverse(input) : input

    # verify if the input type is Float
    if T == Float32 || T == Float64
        return hex2num(bytes2hex(input))
    else
        return parse(T, bytes2hex(input), 16)
    end
end

#
#=
 = get_simtime function
 = Info: get the world simulator time using its shared space memory
 = Use: time = get_simtime(), where time is in micro second (μs).
 =#
function get_simtime()
    # open shared memory file
    isfile("/dev/shm/clock_memspace") ? shm = open(read, "/dev/shm/clock_memspace") : shm = [UInt8(0)]

    return bytes2num(shm, UInt64, false)
end

#
#=
 = wait_sim function
 = Info: wait a time according with the world simulator time
 = Use: wait_sim(time), where time is in second(s)
 =#
function wait_sim(time)
    # get initial time
    t0 = get_simtime()

    # wait till the difference time is bigger than time
    while get_simtime() - t0 < time*10^6
        sleep(0.0001)
    end
end

end

