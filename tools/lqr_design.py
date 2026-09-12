#!/usr/bin/env python3
"""Offline LQR gain design for the SKR Pico cart-pole (acceleration input).

State s = [x, xdot, theta, thetadot] in SI units (m, m/s, rad, rad/s), with
theta measured from upright and positive when the pole tips toward +x (RIGHT).
The stepper is treated as an ideal cart-acceleration source, so u = xddot:

    xddot        = u
    thetadotdot  = (g/l) * theta - (1/l) * u

    A = [ 0 1 0 0 ; 0 0 0 0 ; 0 0 0 1 ; 0 0 g/l 0 ]
    B = [ 0 ; 1 ; 0 ; -1/l ]

l is the pivot -> pole CENTER OF MASS distance (half the rod length for a
uniform rod). This formulation needs only (l, g), not the cart/pole masses.

To reject the steady cart creep caused by a small upright-calibration bias, the
plant is augmented with an integrator on the cart position error:

    xi_dot = x - x_ref

giving a 5th state xi (m*s). LQR then solves for a gain on xi as well, so the
law pulls the cart to x_ref and nulls the bias instead of settling at an offset:

    A_aug = [ A 0 ; 1 0 0 0 0 ],  B_aug = [ B ; 0 ]

Solve the continuous-time algebraic Riccati equation (CARE) for P:

    A^T P + P A - P B R^-1 B^T P + Q = 0,   K = R^-1 B^T P

then u = -K s. Gains are written to src/lqr_gains.h. Re-run this script after
changing l/g/Q/qxi/R to regenerate the header.

Usage:
    python3 tools/lqr_design.py                 # default l, g, Q, qxi, R
    python3 tools/lqr_design.py --l 0.15 --q 1,0.1,100,10 --qxi 0.5 --r 0.5
    python3 tools/lqr_design.py --no-write      # print only
"""
import argparse
import os
import sys

import numpy as np

DEFAULT_L = 0.2          # pivot -> pole center of mass (m)
DEFAULT_G = 9.81          # gravity (m/s^2)
DEFAULT_Q = (1.0, 1.0, 100.0, 10.0)   # diag: x, xdot, theta, thetadot
DEFAULT_QXI = 1.0         # weight on the cart position-error integral (xi)
DEFAULT_R = 0.1           # scalar control effort weight


def build_ab(l, g):
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, g / l, 0.0],
    ])
    B = np.array([[0.0], [1.0], [0.0], [-1.0 / l]])

    # Augment with xi_dot = x - x_ref (position-error integral).
    A_aug = np.zeros((5, 5))
    A_aug[:4, :4] = A
    A_aug[4, 0] = 1.0
    B_aug = np.vstack([B, [[0.0]]])
    return A_aug, B_aug


def solve_care(A, B, Q, R):
    """Return (P, solver_name). Prefer scipy; fall back to a numpy Hamiltonian."""
    try:
        from scipy.linalg import solve_continuous_are
        P = solve_continuous_are(A, B, Q, R)
        return P, "scipy.linalg.solve_continuous_are"
    except ImportError:
        pass

    # Numpy fallback: P = U2 @ inv(U1), where [U1; U2] spans the stable
    # invariant subspace (negative-real-part eigenvalues) of the Hamiltonian
    # H = [[A, -B R^-1 B^T], [-Q, -A^T]].
    Rinv = np.linalg.inv(R)
    H = np.block([
        [A, -B @ Rinv @ B.T],
        [-Q, -A.T],
    ])
    w, V = np.linalg.eig(H)
    order = np.argsort(w.real)
    Vstable = V[:, order[:A.shape[0]]]
    n = A.shape[0]
    U1 = Vstable[:n, :]
    U2 = Vstable[n:, :]
    P = np.real(U2 @ np.linalg.inv(U1))
    P = 0.5 * (P + P.T)   # symmetrize to kill numerical asymmetry
    return P, "numpy Hamiltonian eigen-decomposition (scipy not installed)"


def parse_floats(text):
    return tuple(float(x) for x in text.split(","))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--l", type=float, default=DEFAULT_L,
                    help="pivot -> pole center of mass, meters (default %(default)s)")
    ap.add_argument("--g", type=float, default=DEFAULT_G,
                    help="gravity, m/s^2 (default %(default)s)")
    ap.add_argument("--q", type=parse_floats, default=DEFAULT_Q,
                    metavar="x,xd,th,thd",
                    help="diagonal state weights (default 1,0.1,100,10)")
    ap.add_argument("--qxi", type=float, default=DEFAULT_QXI,
                    help="weight on the cart position-error integral (default %(default)s)")
    ap.add_argument("--r", type=float, default=DEFAULT_R,
                    help="scalar control-effort weight (default %(default)s)")
    ap.add_argument("--no-write", action="store_true",
                    help="print gains without rewriting src/lqr_gains.h")
    args = ap.parse_args()

    if args.l <= 0.0:
        ap.error("--l must be positive")
    if args.g <= 0.0:
        ap.error("--g must be positive")
    if args.r <= 0.0:
        ap.error("--r must be positive")
    if args.qxi < 0.0:
        ap.error("--qxi must be >= 0")
    if len(args.q) != 4:
        ap.error("--q must have exactly 4 comma-separated values")

    A, B = build_ab(args.l, args.g)
    Q = np.diag(np.array(list(args.q) + [args.qxi], dtype=float))
    R = np.array([[args.r]], dtype=float)

    P, solver = solve_care(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P      # 1x5 row vector
    K = K.ravel()

    residual = A.T @ P + P @ A - P @ B @ np.linalg.inv(R) @ B.T @ P + Q
    eig = np.linalg.eigvals(A - B @ K.reshape(1, -1))

    # Non-minimum-phase sanity check: theta > 0 (pole tipped RIGHT) must command
    # u > 0 (accelerate RIGHT) when the cart is otherwise settled.
    u_for_right_tilt = -K[2] * 1.0
    if u_for_right_tilt <= 0.0:
        print("ERROR: sign check failed (theta>0 should give u>0)", file=sys.stderr)
        return 1

    print(f"l = {args.l} m, g = {args.g} m/s^2")
    print(f"Q = diag({list(args.q) + [args.qxi]}), R = [{args.r}]")
    print(f"solver: {solver}")
    print("A_aug =\n", A)
    print("B_aug =\n", B)
    print(f"K = [{' '.join(f'{k: .6f}' for k in K)}]")
    print(f"closed-loop eig = {np.round(eig, 4)}")
    print(f"ARE residual (max abs) = {np.max(np.abs(residual)):.3e}")
    print(f"sign check: theta=+1 -> u = {-K[2]:.6f} (>0 required, OK)")

    if args.no_write:
        return 0

    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(repo, "src", "lqr_gains.h")
    header = f"""\
/* Auto-generated by tools/lqr_design.py -- do not edit by hand.
 *
 * Cart-pole LQR, acceleration input, with a cart position-error integrator so
 * the cart is pulled to x_ref (default: middle of travel) and a small upright
 * calibration bias cannot make it creep. State s = [x, xdot, theta, thetadot,
 * xi] in SI units; theta is from upright, positive = pole tips toward +x
 * (RIGHT); xi = integral of (x - x_ref) in m*s. Input u = xddot (m/s^2). Law:
 *
 *     u = -( K0*(x - x_ref) + K1*xdot + K2*theta + K3*thetadot + K4*xi )
 *
 * Model (augmented, xi_dot = x - x_ref):
 *     A = [ 0 1 0 0 0 ; 0 0 0 0 0 ; 0 0 0 1 0 ; 0 0 g/l 0 0 ; 1 0 0 0 0 ]
 *     B = [ 0 ; 1 ; 0 ; -1/l ; 0 ]
 *
 * Parameters:
 *     l = {args.l} m   (pivot -> pole center of mass; half a uniform rod's length)
 *     g = {args.g} m/s^2
 * Weights:
 *     Q = diag({list(args.q) + [args.qxi]})   R = [{args.r}]
 *       Q0 x        cart position pull toward x_ref (via K0)
 *       Q1 xdot     light cart-velocity damping
 *       Q2 theta    upright angle penalty (main aggressiveness knob)
 *       Q3 thetadot angle-rate damping
 *       Q4 xi       position-error integral weight (bias rejection / centering)
 *       R           control-effort penalty (larger = gentler cart motion)
 * Solver: {solver}
 * Regenerate with:
 *     python3 tools/lqr_design.py --l {args.l} --q {",".join(str(q) for q in args.q)} --qxi {args.qxi} --r {args.r}
 */
#ifndef LQR_GAINS_H
#define LQR_GAINS_H

#define LQR_K0 {K[0]: .6f}f   /* (x - x_ref) error, 1/m */
#define LQR_K1 {K[1]: .6f}f   /* xdot, s/m */
#define LQR_K2 {K[2]: .6f}f   /* theta, 1/rad */
#define LQR_K3 {K[3]: .6f}f   /* thetadot, s/rad */
#define LQR_K4 {K[4]: .6f}f   /* xi = integral of (x - x_ref), 1/(m*s) */

#endif /* LQR_GAINS_H */
"""
    with open(out, "w") as f:
        f.write(header)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
