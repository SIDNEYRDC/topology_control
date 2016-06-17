#
#=
 = Initial state configurations
 =#

# robots number
const n_bot = 6

# reference (target) number
const n_ref = 0

# iterations number
const n_iter = 100

# initial positions
# x(i | n_bot + r, [x y θ], t)
x = zeros(n_bot + n_ref, 3, n_iter)
x[1, :, 1] = [0 0 0]
x[2, :, 1] = [50 0 0]
x[3, :, 1] = [80 0 0]
x[4, :, 1] = [130 0 0]
x[5, :, 1] = [180 0 0]
x[6, :, 1] = [200 0 0]

# initial velocities
# v(i | n_bot + r, [vx vy vθ], t)
v = zeros(n_bot + n_ref, 3, n_iter)

# initial communication radius
# rcom(i, rcom, t)
r_com = fill(50, (n_bot, n_iter))

# initial coverage radius
# rcov(i, rcov, t)
r_cov = fill(24, (n_bot, n_iter))

#
#=
 = Network parameters
 =#

# Gaussian white noise standard deviation (σ)
const sigma = 0.5

# RSSI maximum attenuation factor (μ) (dBm)
# max: 0 min: -90
const mu = -1;

# path loss exponent (Φ)
# free space = 2
# urban area = 2.7 to 3.5
# suburban area = 3 to 5
# indoor (line-of-sight) = 1.6 to 1.8
const phi = 2

# use RSSI readings
# 0: RSSI sensing off
# 1: RSSI senging on
const RSSI_SENS = 1

# RSSI noise
# 0: noise off
# 1: noise on
const RSSI_NOISE = 1

# topology type
# 0: fixed
# 1: dynamic
const TOP_TYPE = 1

#
#=
 = Control Configurations
 =#

# integration step
const h = 1

# prediction horizon
const p = 5

# wheights to the motion control algorithm
gamma = zeros(6)
gamma[1] = 1        # weight to desired position error
gamma[2] = 1e5      # weight to virtual neighbors velocity error
gamma[3] = 1        # weight to real neighbors velocity error
gamma[4] = 1        # weight to reference position error
gamma[5] = 0        # weight to reference velocity error
gamma[6] = 0.1        # weight to control effort

# saturation to linear control signal (m/s)
const vx_max = 1
const vx_min = -1

# saturation to angular control signal (rad/s)
const va_max = 1
const va_min = -1

