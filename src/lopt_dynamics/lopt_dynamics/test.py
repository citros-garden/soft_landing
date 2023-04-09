import sys

print(sys.version)

import lopt
import inspect

x_x0 = 0
x_y0 = 0
x_z0 = 0

x_xf = 0
x_yf = 0
x_zf = 100

um = 20
ul = -um

g0 = 9.81


def terminal_cost0(xf, tf, x0, t0):
    return tf


# def running_cost0(x, u, t):
#    return u[0]


def dynamics0(x, u, t):  # Defining vessel dynamics
    v_x = x[3]
    v_y = x[4]
    v_z = x[5]

    return [v_x, v_y, v_z, u[0], u[1], u[2] - g0]


def terminal_constraints0(xf, tf, x0, t0):
    tc = [
        x0[0] - x_x0,
        x0[1] - x_y0,
        x0[2] - x_z0,
        x0[3] - 0,
        x0[4] - 0,
        x0[5] - 0,
        xf[0] - x_xf,
        xf[1] - x_yf,
        xf[2] - x_zf,
        xf[3] - 0,
        xf[4] - 0,
        xf[5] - 0,
    ]
    return tc


#            [ x[0],  x[1],  x[2],  x[3],  x[4],  x[5] ]
#            [   x,     y,     z,    Vx,    Vy,    Vz  ]
tf0 = 15  # guess tf
xf0 = [x_xf, x_yf, x_zf, 0, 0, 0]  # target conditions
x00 = [x_x0, x_y0, x_z0, 0, 0, 0]  # starting conditions
lbx = [0, 0, 0, -10000, -10000, -10000]
ubx = [1000, 10000, 10000, 10000, 10000, 10000]
lbu = [ul, ul, ul]  # u_x, u_y, u_z
ubu = [um, um, um]  # u_x, u_y, u_z
btf = [1, 10000]  # tf_min tf_max

method_list = [
    func
    for func in dir(lopt)
    if callable(getattr(lopt, func)) and not func.startswith("__")
]

print(method_list)

res_x, res_u, res_t = lopt.solve(
    dyn_func=dynamics0,
    term_cost=terminal_cost0,  # runn_cost=running_cost0,
    term_constr=terminal_constraints0,
    ocp_tf0=tf0,
    ocp_btf=btf,
    ocp_lbu=lbu,
    ocp_lbx=lbx,
    ocp_ubu=ubu,
    ocp_ubx=ubx,
    ocp_x00=x00,
    ocp_xf0=xf0,
)
