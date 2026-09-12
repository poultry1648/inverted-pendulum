/*
 * Cart-pole LQR design in the browser: the same math as tools/lqr_design.py,
 * ported to plain JS so the UI can turn Q/R/l into a gain vector K live.
 *
 * State s = [x, xdot, theta, thetadot] SI (m, m/s, rad, rad/s), theta from
 * upright, positive = pole tips toward +x (RIGHT). Input u = xddot:
 *
 *     A = [ 0 1 0 0 ; 0 0 0 0 ; 0 0 0 1 ; 0 0 g/l 0 ],  B = [ 0 ; 1 ; 0 ; -1/l ]
 *
 * Augmented with xi_dot = x - x_ref for steady cart-creep rejection:
 *     A_aug[4][0] = 1, B_aug[4] = 0, Q = diag([qx, qxd, qth, qthd, qxi]).
 *
 * The continuous algebraic Riccati equation is solved by Kleinman-Newton
 * iteration: with a stabilizing K, solve the Lyapunov equation
 *     (A - BK)^T P + P (A - BK) + Q + K^T R K = 0
 * then update K = R^-1 B^T P. Convergence is quadratic and there is no stiff
 * ODE step to tune. A stabilizing seed comes from Ackermann pole placement,
 * which also handles any (l, g). Closed-loop poles are found from a
 * Faddeev-LeVerrier characteristic polynomial + Durand-Kerner root finder.
 */

export function zeros(n, m) {
  return Array.from({ length: n }, () => new Float64Array(m));
}

export function identity(n) {
  const I = zeros(n, n);
  for (let i = 0; i < n; i++) I[i][i] = 1;
  return I;
}

function matmul(A, B) {
  const n = A.length;
  const m = B[0].length;
  const p = B.length;
  const C = zeros(n, m);
  for (let i = 0; i < n; i++)
    for (let k = 0; k < p; k++) {
      const a = A[i][k];
      if (a === 0) continue;
      const Bk = B[k];
      const Ci = C[i];
      for (let j = 0; j < m; j++) Ci[j] += a * Bk[j];
    }
  return C;
}

function transpose(A) {
  const n = A.length;
  const m = A[0].length;
  const T = zeros(m, n);
  for (let i = 0; i < n; i++)
    for (let j = 0; j < m; j++) T[j][i] = A[i][j];
  return T;
}

function trace(A) {
  let s = 0;
  for (let i = 0; i < A.length; i++) s += A[i][i];
  return s;
}

function symmetrize(P) {
  const n = P.length;
  for (let i = 0; i < n; i++)
    for (let j = i + 1; j < n; j++) {
      const v = 0.5 * (P[i][j] + P[j][i]);
      P[i][j] = v;
      P[j][i] = v;
    }
}

/* Solve A x = b by Gaussian elimination with partial pivoting. */
function solveLinear(A, b) {
  const n = A.length;
  const M = A.map((row, i) => Float64Array.from([...row, b[i]]));
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++)
      if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    if (Math.abs(M[piv][col]) < 1e-14) throw new Error('singular matrix');
    if (piv !== col) [M[piv], M[col]] = [M[col], M[piv]];
    const d = M[col][col];
    for (let r = col + 1; r < n; r++) {
      const f = M[r][col] / d;
      if (f === 0) continue;
      for (let c = col; c <= n; c++) M[r][c] -= f * M[col][c];
    }
  }
  const x = new Float64Array(n);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let c = r + 1; c < n; c++) s -= M[r][c] * x[c];
    x[r] = s / M[r][r];
  }
  return x;
}

function matInverse(A) {
  const n = A.length;
  const M = A.map((row, i) =>
    Float64Array.from([...row, ...Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))])
  );
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++)
      if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    if (Math.abs(M[piv][col]) < 1e-12) throw new Error('singular matrix');
    if (piv !== col) [M[piv], M[col]] = [M[col], M[piv]];
    const d = M[col][col];
    for (let c = 0; c < 2 * n; c++) M[col][c] /= d;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = M[r][col];
      if (f === 0) continue;
      for (let c = 0; c < 2 * n; c++) M[r][c] -= f * M[col][c];
    }
  }
  const Inv = zeros(n, n);
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) Inv[i][j] = M[i][n + j];
  return Inv;
}

function kron(A, B) {
  const p = A.length;
  const q = A[0].length;
  const m = B.length;
  const n = B[0].length;
  const K = zeros(p * m, q * n);
  for (let i = 0; i < p; i++)
    for (let j = 0; j < q; j++) {
      const a = A[i][j];
      if (a === 0) continue;
      for (let k = 0; k < m; k++)
        for (let l = 0; l < n; l++) K[i * m + k][j * n + l] = a * B[k][l];
    }
  return K;
}

export function buildAugmented(l, g) {
  const A = zeros(5, 5);
  A[0][1] = 1;
  A[2][3] = 1;
  A[3][2] = g / l;
  A[4][0] = 1; // xi_dot = x - x_ref
  const B = zeros(5, 1);
  B[1][0] = 1;
  B[3][0] = -1 / l;
  return { A, B };
}

function diag(values) {
  const n = values.length;
  const D = zeros(n, n);
  for (let i = 0; i < n; i++) D[i][i] = values[i];
  return D;
}

/* Monic polynomial coefficients (highest power first) with the given roots. */
function polyFromRoots(roots) {
  let c = [1];
  for (const r of roots) {
    const nc = new Array(c.length + 1).fill(0);
    for (let i = 0; i < c.length; i++) {
      nc[i] += c[i];
      nc[i + 1] += -r * c[i];
    }
    c = nc;
  }
  return c;
}

/* Ackermann pole placement: K with eig(A - BK) == poles. */
function acker(A, B, poles) {
  const n = A.length;
  const Co = zeros(n, n);
  let col = B;
  for (let k = 0; k < n; k++) {
    for (let i = 0; i < n; i++) Co[i][k] = col[i][0];
    col = matmul(A, col);
  }
  const Coin = matInverse(Co);

  const c = polyFromRoots(poles); // c[0] = 1
  const powers = [identity(n)];
  for (let k = 1; k <= n; k++) powers.push(matmul(A, powers[k - 1]));
  const phiA = zeros(n, n);
  for (let j = 0; j <= n; j++) {
    const Ap = powers[n - j];
    const cj = c[j];
    for (let i = 0; i < n; i++)
      for (let l = 0; l < n; l++) phiA[i][l] += cj * Ap[i][l];
  }
  const M = matmul(Coin, phiA);
  return M[n - 1].slice(); // last row = e_n^T * Coin * phiA
}

/* Solve A^T P + P A = -W (A, W real n x n) via Kronecker vectorization. */
function lyapunov(A, W) {
  const n = A.length;
  const At = transpose(A);
  const I = identity(n);
  const M = zeros(n * n, n * n);
  const K1 = kron(I, At);
  const K2 = kron(At, I);
  for (let i = 0; i < n * n; i++)
    for (let j = 0; j < n * n; j++) M[i][j] = K1[i][j] + K2[i][j];

  const rhs = new Float64Array(n * n);
  for (let j = 0; j < n; j++)
    for (let i = 0; i < n; i++) rhs[j * n + i] = -W[i][j]; // column-major vec

  const sol = solveLinear(M, rhs);
  const P = zeros(n, n);
  for (let j = 0; j < n; j++)
    for (let i = 0; i < n; i++) P[i][j] = sol[j * n + i];
  symmetrize(P);
  return P;
}

function kleinman(A, B, Q, R, K0) {
  const n = A.length;
  const r = R[0][0];
  let K = K0.slice();
  let P = zeros(n, n);
  for (let iter = 0; iter < 100; iter++) {
    const Ak = zeros(n, n);
    const W = zeros(n, n);
    for (let i = 0; i < n; i++)
      for (let j = 0; j < n; j++) {
        Ak[i][j] = A[i][j] - B[i][0] * K[j];
        W[i][j] = Q[i][j] + r * K[i] * K[j];
      }
    P = lyapunov(Ak, W);
    const Knew = new Array(n).fill(0);
    for (let j = 0; j < n; j++) {
      let s = 0;
      for (let i = 0; i < n; i++) s += B[i][0] * P[i][j];
      Knew[j] = s / r;
    }
    let diff = 0;
    for (let j = 0; j < n; j++) diff = Math.max(diff, Math.abs(Knew[j] - K[j]));
    K = Knew;
    if (diff < 1e-13) break;
  }
  return { K, P };
}

/* Faddeev-LeVerrier: monic characteristic polynomial coefficients of M. */
function charPoly(M) {
  const n = M.length;
  const powers = [identity(n)];
  for (let i = 1; i <= n; i++) powers.push(matmul(M, powers[i - 1]));
  const c = new Array(n + 1).fill(0);
  c[0] = 1;
  for (let k = 1; k <= n; k++) {
    let s = 0;
    for (let i = 1; i <= k; i++) s += c[k - i] * trace(powers[i]);
    c[k] = -s / k;
  }
  return c;
}

const cadd = (a, b) => ({ re: a.re + b.re, im: a.im + b.im });
const csub = (a, b) => ({ re: a.re - b.re, im: a.im - b.im });
const cmul = (a, b) => ({
  re: a.re * b.re - a.im * b.im,
  im: a.re * b.im + a.im * b.re,
});
function cdiv(a, b) {
  const d = b.re * b.re + b.im * b.im;
  return { re: (a.re * b.re + a.im * b.im) / d, im: (a.im * b.re - a.re * b.im) / d };
}
const cabs = (a) => Math.hypot(a.re, a.im);

function polyEval(coeffs, x) {
  let r = { re: 0, im: 0 };
  for (const c of coeffs) r = cadd(cmul(r, x), { re: c, im: 0 });
  return r;
}

function durandKerner(coeffs, maxIter = 500, tol = 1e-13) {
  const n = coeffs.length - 1;
  const seed = { re: 0.4, im: 0.9 };
  const roots = [];
  let cur = { re: 1, im: 0 };
  for (let i = 0; i < n; i++) {
    cur = cmul(cur, seed);
    roots.push({ ...cur });
  }
  for (let iter = 0; iter < maxIter; iter++) {
    let maxd = 0;
    for (let i = 0; i < n; i++) {
      const num = polyEval(coeffs, roots[i]);
      let den = { re: 1, im: 0 };
      for (let j = 0; j < n; j++) if (j !== i) den = cmul(den, csub(roots[i], roots[j]));
      const d = cdiv(num, den);
      roots[i] = csub(roots[i], d);
      maxd = Math.max(maxd, cabs(d));
    }
    if (maxd < tol) break;
  }
  return roots;
}

export function closedLoopPoles(A, B, K) {
  const n = A.length;
  const M = zeros(n, n);
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++) M[i][j] = A[i][j] - B[i][0] * K[j];
  return durandKerner(charPoly(M));
}

function careResidual(A, B, Q, R, P) {
  const At = transpose(A);
  const AtP = matmul(At, P);
  const PA = matmul(P, A);
  const PB = matmul(P, B);
  const BtP = matmul(transpose(B), P);
  const n = A.length;
  let max = 0;
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++) {
      const v = AtP[i][j] + PA[i][j] - (PB[i][0] * BtP[0][j]) / R[0][0] + Q[i][j];
      if (Math.abs(v) > max) max = Math.abs(v);
    }
  return max;
}

/*
 * Design the LQR. q is [qx, qxd, qth, qthd], qxi the position-error-integral
 * weight, r the scalar control weight. Returns { k, poles, residual, stable,
 * signOk }.
 */
export function solveLqr({ l, g, q, qxi, r }) {
  const { A, B } = buildAugmented(l, g);
  const Q = diag([...q, qxi]);
  const R = [[r]];

  const seed = acker(A, B, [-1, -2, -3, -4, -5]);
  const { K: k, P } = kleinman(A, B, Q, R, seed);

  const poles = closedLoopPoles(A, B, k);
  const residual = careResidual(A, B, Q, R, P);
  const stable = poles.every((p) => p.re < -1e-9);
  const signOk = k[2] < 0; // theta>0 must command u>0
  return { k, poles, residual, stable, signOk };
}
