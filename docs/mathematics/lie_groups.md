# Lie groups SO(2), SE(2), SO(3), SE(3)

Headers: [`so2.hpp`](../../include/roboticscore/geometry/so2.hpp), [`se2.hpp`](../../include/roboticscore/geometry/se2.hpp), [`so3.hpp`](../../include/roboticscore/geometry/so3.hpp), [`se3.hpp`](../../include/roboticscore/geometry/se3.hpp) · sources: [`so2_se2.cpp`](../../src/geometry/so2_se2.cpp), [`so3_se3.cpp`](../../src/geometry/so3_se3.cpp)

## Conventions

- Transforms act as $p' = Rp + t$; composition `a * b` applies `b` first; `a.between(b)` is
  $a^{-1}b$.
- Tangent vectors put the **translational part first**: $\xi = (v_x, v_y, \omega)$ for SE(2)
  and $\xi = (v, \omega) \in \mathbb{R}^6$ for SE(3).
- `exp`/`log` are the exact group maps, not the decoupled "translation plus angle"
  parameterisation. Adjoints satisfy $T\exp(\xi)T^{-1} = \exp(\mathrm{Ad}_T\,\xi)$.
- Geodesic interpolation: $\mathrm{interp}(a, b, t) = a\exp(t\log(a^{-1}b))$.
- $[\omega]_\times$ denotes `SO3::hat`, the skew matrix with $[\omega]_\times b = \omega \times b$.

## SO(2)

Stored as the unit complex number $(\cos\theta, \sin\theta)$, so composition and point rotation
need no trigonometric calls. $\exp(\theta) = (\cos\theta, \sin\theta)$ and
$\log = \mathrm{wrap}(\mathrm{atan2}(s, c)) \in [-\pi, \pi)$. The algebra so(2) is identified with
$\mathbb{R}$:

$$
\mathrm{hat}(w) = \begin{bmatrix} 0 & -w \\ w & 0 \end{bmatrix}, \qquad \mathrm{vee}(M) = \tfrac12 (M_{10} - M_{01}),
$$

so `vee` uses the antisymmetric part. `normalized()` re-projects onto the unit circle;
`from_matrix` uses atan2 of the first column.

## SE(2)

$$
\exp(\xi) = \big(R(\omega),\ V(\omega)\,\rho\big), \qquad
V(\omega) = \begin{bmatrix} a & -b \\ b & a \end{bmatrix}, \quad a = \frac{\sin\omega}{\omega}, \quad b = \frac{1 - \cos\omega}{\omega} = \frac{2\sin^2(\omega/2)}{\omega},
$$

with $\rho = (v_x, v_y)$; $V$ is `SE2::translation_jacobian`. The logarithm takes
$\omega = \log R \in [-\pi, \pi)$ and inverts $V$ in closed form,

$$
\rho = V(\omega)^{-1} t = \frac{1}{a^2 + b^2}\begin{bmatrix} a & b \\ -b & a \end{bmatrix} t,
$$

which stays finite at the half turn ($a = 0$, $\lvert b\rvert = 2/\pi$). Hat (vee is its
inverse, reading $(v_x, v_y)$ from the last column and $\omega$ from the antisymmetric part)
and adjoint:

$$
\hat\xi = \begin{bmatrix} 0 & -\omega & v_x \\ \omega & 0 & v_y \\ 0 & 0 & 0 \end{bmatrix}, \qquad
\mathrm{Ad}_T = \begin{bmatrix} R & (t_y,\ -t_x)^\top \\ 0 & 1 \end{bmatrix}.
$$

## SO(3)

Stored as a unit quaternion, normalised after every operation; $q$ and $-q$ are not
canonicalised. With $\theta = \lVert\omega\rVert$:

$$
\exp(\omega) = \Big(\cos\tfrac{\theta}{2},\ \frac{\sin(\theta/2)}{\theta}\,\omega\Big), \qquad
\log(q) = \frac{2\,\mathrm{atan2}(\lVert v\rVert, w)}{\lVert v\rVert}\, v,
$$

where $q = (w, v)$ and `log` first picks the hemisphere $w \ge 0$, so
$\lVert\log q\rVert \in [0, \pi]$. `hat` is the skew matrix $[\omega]_\times$ and `vee` reads
$\tfrac12(M_{21} - M_{12},\ M_{02} - M_{20},\ M_{10} - M_{01})$, i.e. the antisymmetric part. Left Jacobian
and its inverse (valid for $\theta < 2\pi$):

$$
J_l(\omega) = I + A\,[\omega]_\times + B\,[\omega]_\times^2, \qquad
J_l^{-1}(\omega) = I - \tfrac12[\omega]_\times + C\,[\omega]_\times^2,
$$

$$
A = \frac{1 - \cos\theta}{\theta^2} = \frac{2\sin^2(\theta/2)}{\theta^2}, \quad
B = \frac{\theta - \sin\theta}{\theta^3}, \quad
C = \frac{1 - \frac{\theta}{2}\cot\frac{\theta}{2}}{\theta^2}.
$$

`from_matrix` returns the Frobenius-nearest rotation $UV^\top$ from an SVD (flipping the last
column of $U$ if the determinant is negative) and rejects matrices with
$\det \le 10^{-12}$. The adjoint of SO(3) is $R$ itself.

**Euler angles.** `from_euler_zyx(yaw, pitch, roll)` builds $R = R_z(\psi)R_y(\theta)R_x(\phi)$.
`to_euler_zyx` uses $\theta = \mathrm{atan2}(-r_{20}, \sqrt{r_{00}^2 + r_{10}^2})$ (well conditioned
near $\pm\pi/2$, unlike $\arcsin(-r_{20})$), $\phi = \mathrm{atan2}(r_{21}, r_{22})$,
$\psi = \mathrm{atan2}(r_{10}, r_{00})$. At gimbal lock ($\sqrt{r_{00}^2 + r_{10}^2} < 10^{-12}$) yaw
and roll rotate about the same axis and only their combination is defined; the code sets
$\phi = 0$ and $\psi = \mathrm{atan2}(-r_{01}, r_{11})$, so `from_euler_zyx(to_euler_zyx())`
still reproduces the rotation.

## SE(3)

$$
\exp(v, \omega) = \big(\exp(\omega),\ J_l(\omega)\,v\big), \qquad
\log(R, t) = \big(J_l^{-1}(\log R)\,t,\ \log R\big), \qquad
\mathrm{Ad}_T = \begin{bmatrix} R & [t]_\times R \\ 0 & R \end{bmatrix}.
$$

`hat` maps $(v, \omega)$ to the $4\times 4$ twist below; `vee` reads $v$ from the last column
and $\omega$ through `SO3::vee`.

$$
\hat\xi = \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix} \in \mathbb{R}^{4\times 4}
$$

## Numerical considerations

Closed forms such as $\sin\theta/\theta$ are $0/0$ at the identity, and several coefficients
lose accuracy long before that through cancellation. The implementation uses two thresholds:

| Quantity | Closed form used | Series below the threshold | Threshold |
|---|---|---|---|
| SE(2) $a$, $b$ | $\sin\omega/\omega$, $2\sin^2(\omega/2)/\omega$ | $1 - \omega^2/6$, $\omega/2 - \omega^3/24$ | $10^{-6}$ |
| SO(3) exp | $\cos(\theta/2)$, $\sin(\theta/2)/\theta$ | $1 - \theta^2/8$, $1/2 - \theta^2/48$ | $10^{-6}$ |
| SO(3) log, $n = \lVert v\rVert$ of $q = (w, v)$ | $2\,\mathrm{atan2}(n, w)/n$ | $(2/w)(1 - n^2/(3w^2))$ | $10^{-6}$ on $n$ |
| $A$ | $2\sin^2(\theta/2)/\theta^2$ | $1/2 - \theta^2/24$ | $10^{-6}$ |
| $B$ | $(\theta - \sin\theta)/\theta^3$ | $1/6 - \theta^2/120 + \theta^4/5040$ | $10^{-2}$ |
| $C$ | $(1 - \frac{\theta}{2}\cot\frac{\theta}{2})/\theta^2$ | $1/12 + \theta^2/720 + \theta^4/30240$ | $10^{-2}$ |

**Cancellation-free forms.** $1 - \cos x$ subtracts two nearly equal numbers; it is evaluated as
$2\sin^2(x/2)$, which has no cancellation, so $A$ and $b$ only need the series to avoid
$0/0$, and a threshold of $10^{-6}$ suffices (the next series term is below $10^{-26}$).

**Why $10^{-2}$ for $B$ and $C$.** The numerators $\theta - \sin\theta \approx \theta^3/6$ and
$1 - \frac{\theta}{2}\cot\frac{\theta}{2} \approx \theta^2/12$ are differences of $O(1)$ (or
$O(\theta)$) quantities, so the closed forms carry a relative rounding error of order
$10\,\varepsilon/\theta^2$ ($\varepsilon \approx 2.2\cdot10^{-16}$), about $10^{-11}$ at
$\theta = 10^{-2}$ and growing as $\theta$ shrinks.
The three-term series truncation error there is of order $\theta^6/9! \approx 3\cdot10^{-18}$, so
switching at $10^{-2}$ keeps both branches accurate.

**Wrapping.** Angles from `SO2::log`, `SE2::theta()` and `SE2::log()` follow the
[angle convention](numerical_methods.md) $[-\pi, \pi)$; differences of poses straddling
$\pm\pi$ are taken on the manifold via `between().log()`, never by subtracting angles.

## Tests

- [`tests/unit/geometry/test_se2.cpp`](../../tests/unit/geometry/test_se2.cpp),
  [`test_se3.cpp`](../../tests/unit/geometry/test_se3.cpp): composition, inverse, matrix
  round trips, hat/vee, Euler round trip and gimbal lock, `log` near $\pi$, random-input
  properties (exp/log inverse, adjoint identity).
- [`tests/numerical/test_lie_numerics.cpp`](../../tests/numerical/test_lie_numerics.cpp):
  exact identity at zero motion, exp/log round trips just below and above both series
  thresholds (to $10^{-15}$ for SE(2), $10^{-13}$ for SE(3)), headings straddling $\pm\pi$,
  the SE(2) half turn, and $10^6$ repeated SO(3) compositions staying orthonormal to $10^{-13}$.

The microbenchmark `BM_Se2ComposeExpLog` measures 40.3 ns real time in
[`docs/benchmarks/README.md`](../benchmarks/README.md).

## References

- J. Solà, J. Deray and D. Atchuthan, "A micro Lie theory for state estimation in robotics",
  arXiv preprint, 2018.
- T. D. Barfoot, *State Estimation for Robotics*, Cambridge University Press, 2017.
- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge
  University Press, 2017.
