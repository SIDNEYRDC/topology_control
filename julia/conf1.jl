#
#=
 = Initial configurations
 =#

# robots number
const n_bot = 10

# reference (target) number
const n_ref = 0

# iterations number
const n_iter = 200

# initial positions
# x(i | n_bot + r, [x y theta], t)
x = zeros(n_bot + n_ref, 3, n_iter)
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

x *= 10

# reference initial position
#=x[n_bot + 1, :, :] = repmat([40 30 0], 1, n_iter)=#

# initial velocities
# v(i | n_bot + r, [vx vy vtheta], t)
v = zeros(n_bot + n_ref, 3, n_iter)

# initial communication radius
# rcom(i, rcom, t)
r_com = fill(10, (n_bot, n_iter))*10

# initial coverage radius
# rcov(i, rcov, t)
r_cov = fill(4.5, (n_bot, n_iter))*10

#
#=
 = Network parameters
 =#

 # Gaussian white noise standard deviation (σ)
const sigma = 0.25

# RSSI maximum attenuation factor (μ) (dBm)
# max: 0 min: -90
const mu = 0;

# path loss exponent (Φ)
# free space = 2
# urban area = 2.7 to 3.5
# suburban area = 3 to 5
# indoor (line-of-sight) = 1.6 to 1.8
const phi = 2

# use RSSI readings
# 0: RSSI sensing off
# 1: RSSI senging on
const RSSI_SENS = 0

# RSSI threshold (dBm)
# max: 0
# min: -90
s_min = -23

# RSSI noise
# 0: noise off
# 1: noise on
const RSSI_NOISE = 1

# topology type
# 0: fixed
# 1: dynamic
const TOP_TYPE = 1

# omnet++ interface
# 0: on
# 1: off
const OMNET = 1

#
#=
 = Control Configurations
 =#

# integration step
const h = 0.1

# prediction horizon
const p = 5

# wheights to the motion control algorithm
gamma = zeros(7)
gamma[1] = 1        # weight to desired position error
gamma[2] = 1        # weight to virtual neighbors velocity error
gamma[3] = 1        # weight to real neighbors velocity error
gamma[4] = 0        # weight to reference position error
gamma[5] = 0        # weight to reference velocity error
gamma[6] = 0.1      # weight to control effort
gamma[7] = 0

# saturation to linear control signal (m/s)
const vx_max = 1
const vx_min = -1

# saturation to angular control signal (rad/s)
const va_max = 1
const va_min = -1

