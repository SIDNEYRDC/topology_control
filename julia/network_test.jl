include("network.jl")

using NETWORK

n_bot = 12

x = zeros(n_bot, 3, 1)
x[1, :, 1] = [0 0 0]
x[2, :, 1] = [0 6 0]
x[3, :, 1] = [5 -0.3 0]
x[4, :, 1] = [2.5 -0.9 0]
x[5, :, 1] = [10.0 -5.2 0]
x[6, :, 1] = [11.0 5.0 0]
x[7, :, 1] = [11.0 12.0 0]
x[8, :, 1] = [11.0 16.0 0]
x[9, :, 1] = [15.0 5.0 0]
x[10, :, 1] = [24.0 5.0 0]
x[11, :, 1] = [6 6 0]
x[12, :, 1] = [11 -5 0]

omnet_interface_init(n_bot)

out_data = cell(14)

#=while true=#
    in_data = get_data()
    println(in_data)

    in_id = in_data[1]
    in_omnet_time = in_data[2]

    println("in_id:$(in_id) time:$(in_omnet_time)")

    out_data[1] = in_id
    out_data[2] = in_omnet_time

    out_data[3] = zeros(Float32, n_bot, 2)

    for i = 1 : n_bot
        out_data[3][i, 1] = x[i, 1]
        out_data[3][i, 2] = x[i, 2]
    end

    out_data[4] = 1
    out_data[5] = 3
    out_data[6] = 133
    out_data[7] = 9
    out_data[8] = 1
    out_data[9] = 4
    out_data[10] = [1; 2; 3; 5]
    out_data[11] = 5
    out_data[12] = [1; 2; 3; 5; 8]

    out_data[13] = zeros(Float32, length(out_data[12]), 3)
    out_data[14] = zeros(Float32, length(out_data[12]), 3)

    for i = 1 : length(out_data[12])
        out_data[13][i, :] = x[out_data[12][i], :, 1]
        out_data[14][i, :] = x[out_data[12][i], :, 1]
    end

    send_data(out_data, "127.0.0.1", 50001+in_id)

    in_data = get_data()
    println(in_data)
    println("in_id:$(in_data[1]) time:$(in_data[2])")

#=end=#

