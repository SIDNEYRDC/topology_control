#==============================================================================
 = File Module to Topology Control Algorithm
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2020 Abr 14 21:32:03
 = Info: This source contains the module to access text files in julia.
 =============================================================================#

module FILE

# public functions
export read_conf

#
#=
 = Global definitions
 =#

# configuration data type
type Conf
    n_iter::Int64
    n_bot::Int64
    n_ref::Int64
    dc::Int64
    ros::UInt8
    info_scope::UInt8
    dt::Float32
    ph::Int64
    s_max::Int64
    rho::Float32
    gamma::Array{Float32, 1}
    u_min::Array{Float32, 1}
    u_max::Array{Float32, 1}
    c_com::UInt8
    c_sec::UInt8
    sigma::Float32
    mu::Float32
    phi::Float32
    rssi::UInt8
    rssi_noise::UInt8
    rssi_lim::Float32
    top_type::UInt8
    omnet::UInt8
    x::Array{Float32, 2}
    v::Array{Float32, 2}
    r_sec::Array{Float32, 1}
    r_cov::Array{Float32, 1}
    r_com::Array{Float32, 1}
    timeout::Array{Float32, 1}
    rssi_var::Array{Float32, 2}
end

#
#=
 = Read Configuration File
 = Info: This function is used to read a simulation configuration file to the
 = topology control algorithm.
 = Use: out = read_conf(input), where input is a string with the path to the
 = configuration file, and out is a variable of type Conf.
 =#
function read_conf(input)
    # open and read the whole configuration file
    file = readstring(open(input))

    # remove comments and insert line feed
    file = replace(replace(string(split(file, r"\#.*\n")), "\",\"", ""), "\\n", '\n')

    # the output file
    output = Conf(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, [0], [0], [0], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, [0 0], [0 0], [0], [0], [0], [0], [0 0 0 0 0])

    # extract data
    output.n_iter = extract_data("n\\_iter:\\s*\\d+", "simulation-setup", file)
    output.n_bot = extract_data("n\\_bot:\\s*\\d+", "simulation-setup", file)
    output.n_ref = extract_data("n\\_ref:\\s*\\d+", "simulation-setup", file)
    output.dc = extract_data("dc:\\s*\\d+", "simulation-setup", file)
    output.ros = extract_data("ros:\\s*\\d+", "simulation-setup", file)
    output.info_scope = extract_data("info_scope:\\s*\\d+", "simulation-setup", file)
    output.dt = extract_data("dt:\\s*\\d+\\.?\\d*", "control-setup", file)
    output.ph = extract_data("ph:\\s*\\d+", "control-setup", file)
    output.s_max = extract_data("s\\_max:\\s*\\d+", "control-setup", file)
    output.rho = extract_data("rho:\\s*\\d+\\.?\\d*", "control-setup", file)
    output.gamma = extract_data("gamma:(\\s*\\-?\\d+\\.?\\d*(\\s+|\\n)){7}", "control-setup", file)
    output.u_min = extract_data("u\\_min:(\\s*\\-?\\d+\\.?\\d*(\\s+|\\n)){3}", "control-setup", file)
    output.u_max = extract_data("u\\_max:(\\s*\\-?\\d+\\.?\\d*(\\s+|\\n)){3}", "control-setup", file)
    output.c_com = extract_data("c\\_com:\\s*\\d+", "control-setup", file)
    output.c_sec = extract_data("c\\_sec:\\s*\\d+", "control-setup", file)
    output.sigma = extract_data("sigma:\\s*\\d+\\.?\\d*", "network-setup", file)
    output.mu = extract_data("mu:\\s*\\-?\\d+\\.?\\d*", "network-setup", file)
    output.phi = extract_data("phi:\\s*\\d+\\.?\\d*", "network-setup", file)
    output.rssi = extract_data("rssi:\\s*\\d+", "network-setup", file)
    output.rssi_noise = extract_data("rssi\\_noise:\\s*\\d+", "network-setup", file)
    output.rssi_lim = extract_data("rssi\\_lim:\\s*\\-?\\d+\\.?\\d*", "network-setup", file)
    output.top_type = extract_data("top\\_type:\\s*\\d+", "network-setup", file)
    output.omnet = extract_data("omnet:\\s*\\d+", "network-setup", file)

    # pose and velocity arrays
    output.x = zeros(Float32, output.n_bot, 3)
    output.v = zeros(Float32, output.n_bot, 3)

    # range arrays
    output.r_sec = zeros(UInt8, output.n_bot)
    output.r_cov = zeros(UInt8, output.n_bot)
    output.r_com = zeros(UInt8, output.n_bot)

    # timeout array
    output.timeout = fill(Inf32, output.n_bot)

    for i = 1 : output.n_bot
        # extract pose and velocity
        output.x[i, :] = extract_data("$(i):(\\s*\\-?\\d+\\.?\\d*(\\s+|\\n)){3}", "robot-positions", file)
        v = extract_data("$(i):(\\s*\\-?\\d+\\.?\\d*(\\s+|\\n)){3}", "robot-velocities", file, false)
        v != nothing ? output.v[i, :] = v : 0

        # extract ranges
        range = extract_data("$(i):(\\s*\\d+\\.?\\d*(\\s+|\\n)){3}", "robot-ranges", file)
        output.r_sec[i] = range[1]
        output.r_cov[i] = range[2]
        output.r_com[i] = range[3]

        # extract timeout
        timeout = extract_data("$(i):\\s*\\d+", "timeout", file, false)
        timeout != nothing ? output.timeout[i] = timeout : 0

        # extract rssi-variation
        for j = 1 : output.n_bot
            rssi_var = extract_data("$(i)-$(j):(\\s*\\-?\\d+\\.?\\d*(\\s+|\\n)){3}", "rssi-variation", file, false)

            # check if there is rssi attenuation to the link (i, j)
            if rssi_var != nothing
                output.rssi_var = [output.rssi_var; i j rssi_var']
            end
        end
    end

    # crop rssi_var array removing the extra zeros
    if size(output.rssi_var, 1) > 1
        output.rssi_var = output.rssi_var[2:end, :]
    end

    return output
end

#
#=
 = Extract Data Function
 = Info: This function is used to get data values from configuration files using
 = regular expressions as search keys.
 = Use: data = extract_data(expr, group, input, mandatory), where expr is a
 = string representing a regular expression, input is the string where the search
 = will be done, mandatory is a optional parameter that is used to show an error
 = message when the search key is not found, and data can be a scalar Float32
 = value or a Float32 array.
 =#
function extract_data(expr, group, input, mandatory = true)
    # find the group string
    gs = match(Regex("\\[$(replace(group, "-", "\\-"))\\]"), input)

    # verify if the group has been found
    if gs == nothing
        print_with_color(:red, "ERROR: group '$(group)' not found in the configuration file!\n")
        exit(1)
    end

    # find the group boundary
    gb = match(r"\[", input, gs.offset + length(gs.match))

    # find the compatible string with expr starting in group
    gb != nothing ? m = match(Regex(expr), input[gs.offset:gb.offset]) : m = match(Regex(expr), input, gs.offset)

    # exit if there is no compatible string
    if m == nothing && mandatory == true
        print_with_color(:red, "ERROR: key '$(expr)' not found in the configuration file!\n")
        exit(1)
    elseif m == nothing
        return
    end

    # find the separator character (:)
    index = search(m.match, ':') + 1

    # split the string using space (' ')
    splitted = split(m.match[index:end], ' ')

    # remove the null characters ("")
    splitted = splitted[find(splitted .!= "")]

    if length(splitted) > 1
        return map(x->parse(Float32, x), splitted)
    elseif length(splitted) > 0
        return parse(Float32, splitted[1])
    end
end

end

