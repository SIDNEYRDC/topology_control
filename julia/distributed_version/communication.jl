#==============================================================================
 = Network Module for Topology Control Algorithm in Julia
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Set 22 16:18:28
 = Info: This file contains the functions to access the network interface and
 = protocols by the Topology Control main algorithm.
 =============================================================================#

module NETWORK

#= public functions =#
export RXThread, TXThread, num2bytes, bytes2num

#
#=
 = Global definitions
 =#

#= thread to receive udp messages =#
type RXThread
    #= variables =#
    new_msg::Bool                      # indicator for received messages
    run::Bool                          # indicator to stop the thread
    msg_pool                           # received messages buffer

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
        this.msg_pool = Array{UInt8, 1}

        # associate udp socket with the desired port and ip
        bind(udpr, IPv4(ip), port)

        # set new message indicator as false
        this.new_msg = false

        # set thread run indicator as true
        this.run = true

        # start receiver thread
        thread = @async begin
            while this.run == true
                # get the message from udp socket
                msg = recv(udpr)

                # analyse the message and store it in the message buffer
                isempty(this.msg_pool[]) ? this.msg_pool = msg : this.msg_pool[:, size(this.msg_pool, 2)] != msg ? this.msg_pool = hcat(this.msg_pool, msg) : 0

                # set new message indicator as true
                this.new_msg = true
            end
        end

        #= get received message =#
        this.get_msg = function(timeout)
            # waiting time
            time = 0

            # wait for a new message
            while this.new_msg == false && this.run == true
                # wait a time
                sleep(0.001)

                # increase the waiting time
                time += 0.001

                # if the waiting time is bigger than timeout, exit
                if time > timeout
                    return
                end
            end

            # set new message indicator as false
            this.new_msg = false

            # copy message buffer to a temporary array
            msg = this.msg_pool

            # clean msg buffer
            this.msg_pool = Array{UInt8, 1}

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
    function TXThread(dt = 0.1, ip = "127.0.0.1", src_port = 50000, dst_port = 50000)
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

        # start receiver thread
        thread = @async begin
            while this.run == true
                send(udps, IPv4(ip), dst_port, this.msg)
                sleep(dt)
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

end

