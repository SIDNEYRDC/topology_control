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

rec = zeros(UInt8, n_bot)

#=for event = 1 : 1e4=#
while sum(rec) < n_bot
    in_data = get_data()

    if in_data.pay_size > 0
        println(in_data)
        rec[in_data.id] = 1
        println("rec:$(rec)")
    end

    in_id = in_data.id
    in_omnet_time = in_data.sim_time

    #=println("in_id:$(in_id) time:$(in_omnet_time)")=#

    out_data = MData(0, 0, 0, [0, 0], [0, 0], [0 0], [0 0], [0 0], 0, 0)

    out_data.id = in_id
    out_data.sim_time = in_omnet_time

    out_data.sim_pose = x[:, 1:2, 1] #zeros(Float32, n_bot, 2)

    #=for i = 1 : n_bot=#
        #=out_data.sim_pose[i, 1:2] = x[i, 1:2]=#
    #=end=#

    out_data.n1_size = 5
    out_data.n1 = [1; 2; 3; 5; 8]
    out_data.n2_size = n_bot
    out_data.n2 = collect(1:n_bot)
    #=out_data.x = zeros(Float32, out_data.n2_size, 3)=#
    out_data.v = zeros(Float32, out_data.n2_size, 3)

    out_data.x = x[:, :, 1]

#=    for i = 1 : out_data.n2_size=#
        #=out_data.x[i, :] = x[out_data.n2[i], :, 1]=#
    #=end=#

    send_data(out_data, "127.0.0.1", 50001 + in_id)

    #=in_data = get_data()=#
    #=println(in_data)=#
    #=println("in_id:$(in_data[1]) time:$(in_data[2])")=#
end

