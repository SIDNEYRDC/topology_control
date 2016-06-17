#
#=
 = Initial configurations
 =#

# robots number
const n_bot = 10

# reference (target) number
const n_ref = 0

# iterations number
const n_iter = 100

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

# reference initial position
#=x[n_bot + 1, :, :] = repmat([40 30 0], 1, n_iter)=#

# initial velocities
# v(i | n_bot + r, [vx vy vtheta], t)
v = zeros(n_bot + n_ref, 3, n_iter)

# initial communication radius
# rcom(i, rcom, t)
r_com = fill(10, (n_bot, n_iter))

# initial coverage radius
# rcov(i, rcov, t)
r_cov = fill(4.5, (n_bot, n_iter))

