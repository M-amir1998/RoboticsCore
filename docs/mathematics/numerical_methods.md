# Numerical methods

Headers: [`angles.hpp`](../../include/roboticscore/math/angles.hpp), [`numerical_diff.hpp`](../../include/roboticscore/math/numerical_diff.hpp), [`integration.hpp`](../../include/roboticscore/math/integration.hpp), [`statistics.hpp`](../../include/roboticscore/math/statistics.hpp), [`linear_algebra.hpp`](../../include/roboticscore/math/linear_algebra.hpp), [`random.hpp`](../../include/roboticscore/math/random.hpp) (all in `include/roboticscore/math/`)

## Angle wrapping

**Convention.** Wrapped angles lie in the half-open interval $[-\pi, \pi)$. A half-open interval
makes wrapping a function: $\pi$ and $-\pi$ map to the same representative ($-\pi$), so equal
headings compare equal in tests and regression data. Non-finite inputs give NaN.

`wrap_to_pi` computes $r = \mathrm{remainder}(\theta, 2\pi_{\mathrm{fp}}) \in [-\pi, \pi]$ and maps
$r \ge \pi$ to $r - 2\pi$. `std::remainder` is exact with respect to the floating-point value
$2\pi_{\mathrm{fp}}$, which differs from the true $2\pi$ by about $2.4\cdot10^{-16}$. Each of the
$\lvert\theta\rvert / 2\pi$ removed turns contributes that error, so the result is off by roughly
$\lvert\theta\rvert \cdot 4\cdot10^{-17}$ rad (about $4\cdot10^{-11}$ rad at $\lvert\theta\rvert = 10^6$).
Headings are wrapped after every update, so inputs stay within a few turns in practice.

Related functions:

| Function | Definition |
|---|---|
| `wrap_to_2pi` | `fmod` into $[0, 2\pi)$; a tiny negative input that would round up to $2\pi$ returns 0 |
| `angle_difference(to, from)` | `wrap_to_pi(to - from)`; an exact half turn is reported as $-\pi$ |
| `interpolate_angle(a, b, t)` | `wrap_to_pi(a + t * angle_difference(b, a))`, shortest arc |
| `circular_mean` | $\mathrm{atan2}(\sum w\sin\theta, \sum w\cos\theta)$, 0 when undefined |
| `mean_resultant_length` | $R = \lVert\sum w(\cos\theta, \sin\theta)\rVert / \sum w \in [0, 1]$; check it before trusting a mean |

## Central differences

`numerical_derivative` and `numerical_jacobian` use $f'(x) \approx (f(x + h) - f(x - h)) / 2h$.
The truncation error is $\tfrac{h^2}{6}\lvert f'''\rvert$ and the rounding error about
$\varepsilon\lvert f\rvert / h$; their sum is minimal for $h \propto \varepsilon^{1/3}$. The default
relative step is therefore `kCentralDifferenceStep` $= \varepsilon^{1/3} \approx 6.06\cdot10^{-6}$,
scaled per coordinate as $h_i = \varepsilon^{1/3}\max(1, \lvert x_i\rvert)$ so large coordinates
keep significant digits and small ones do not get a vanishing step. The achievable relative
accuracy is of order $\varepsilon^{2/3} \approx 4\cdot10^{-11}$.

The Jacobian overload with a `difference(a, b)` callable returns $a - b$ in the tangent space
of the output, which is required when outputs are angles (e.g. bearings near $\pm\pi$):
differencing wrapped values with plain subtraction would produce jumps of $2\pi$.

## Integration

| Function | Formula | Order |
|---|---|---|
| `euler_step` | $x + \Delta t\, f(t, x)$ | global $O(\Delta t)$ |
| `rk4_step` | $x + \tfrac{\Delta t}{6}(k_1 + 2k_2 + 2k_3 + k_4)$, classical stages | global $O(\Delta t^4)$ |
| `trapezoid` | $\sum \tfrac12 (x_i - x_{i-1})(y_i + y_{i-1})$, non-uniform samples | $O(h^2)$ |
| `simpson` | composite Simpson, even number of intervals | $O(h^4)$, exact for cubics |

The steppers are templates over the state type, so they work with scalars and fixed-size or
dynamic Eigen vectors without allocation or type erasure. RK4 stages are evaluated as
$k_1 = f(t, x)$, $k_2 = f(t + \tfrac{\Delta t}{2}, x + \tfrac{\Delta t}{2}k_1)$,
$k_3 = f(t + \tfrac{\Delta t}{2}, x + \tfrac{\Delta t}{2}k_2)$, $k_4 = f(t + \Delta t, x + \Delta t\,k_3)$.
`KinematicBicycle::step` takes one RK4 step per call.

## Chi-square and Gaussian functions

Consistency tests (NEES, NIS, gating) need chi-square probabilities:
$F_{\chi^2_k}(x) = P(k/2,\ x/2)$, the regularised lower incomplete gamma function. `regularized_gamma_p`
uses the power series for $x < a + 1$ and the continued fraction for $Q = 1 - P$ (modified Lentz
method) otherwise, each stopping at relative change $\varepsilon$. `chi_squared_quantile`
brackets the quantile by doubling the upper bound and then bisects on the monotone CDF with
the relative stopping rule $(hi - lo) \le 10^{-15}\,hi$ (at most 2000 steps); the relative rule
keeps tiny quantiles accurate ($p = 10^{-10}$, $k = 1$ gives about $1.6\cdot10^{-20}$).

`mahalanobis_squared` and `gaussian_log_pdf` whiten the residual with a Cholesky factor
($r^\top S^{-1} r = \lVert L^{-1} r\rVert^2$, $\log\det S = 2\sum\log L_{ii}$) instead of forming
$S^{-1}$, and the log-density stays finite far in the tail where the density underflows.

## Covariance hygiene

Round-off makes updated covariances slightly asymmetric or indefinite. The helpers in
`linear_algebra.hpp`:

| Function | Purpose |
|---|---|
| `symmetrize(M)` | $(M + M^\top)/2$ after every covariance update |
| `is_symmetric(M, tol)` | $\lVert M - M^\top\rVert_{\max} \le tol \cdot \max(1, \lVert M\rVert_{\max})$ |
| `is_positive_definite(M)` | symmetric (tolerance $10^{-9}$) and Cholesky succeeds |
| `nearest_positive_definite(M, λ)` | clip the spectrum of the symmetric part at $\lambda$ (Frobenius-nearest) |
| `cholesky_lower(M)` | plain LLT; on failure add jitter $10^{-12}\cdot\mathrm{mean}\lvert\mathrm{diag}\rvert$, growing ×10, up to 8 attempts, then `NumericalError` |
| `solve_spd(A, b)` | LLT, with LDLT fallback for semidefinite systems and a residual check |
| `condition_number(M)` | $\sigma_{\max}/\sigma_{\min}$ from an SVD (infinite if singular) |

## Portable deterministic random numbers

The C++ standard fully specifies the output sequence of `std::mt19937_64`, but not the
algorithms of `std::uniform_real_distribution` or `std::normal_distribution`; libstdc++,
libc++ and the MSVC STL produce different variates from the same engine. Simulations and
regression tests would then differ between compilers. `math::Random` therefore uses the
standard engine for raw bits and implements every distribution itself:

| Method | Algorithm |
|---|---|
| `next_u64` | raw `std::mt19937_64` output (checked against the standard's 10000th-value requirement) |
| `uniform01` | top 53 bits times $2^{-53}$, every multiple of $2^{-53}$ in $[0, 1)$ |
| `uniform(lo, hi)` | $lo + (hi - lo)\,u$ |
| `uniform_index(n)` | rejection of the incomplete final block, then modulo (unbiased) |
| `normal` | Box-Muller with $u_1 = 1 - u \in (0, 1]$ so $\log u_1$ is finite; the second variate is cached |
| `multivariate_normal` | $\mu + Lz$ with $L$ from `cholesky_lower` |
| `derive(stream)` | new generator seeded with $\mathrm{splitmix64}(\mathrm{seed} \oplus \mathrm{splitmix64}(\mathrm{stream}))$ |

There is no hidden global generator: every stochastic API takes a `Random&` and every generator
is constructed with an explicit seed. `derive` gives each simulated sensor its own reproducible
stream that does not depend on how many numbers other components consumed.

## Tests

- [`tests/unit/math/test_angles.cpp`](../../tests/unit/math/test_angles.cpp): half-open interval
  at $\pm\pi$, multiple turns, NaN propagation, circular mean across the discontinuity.
- [`tests/unit/math/test_math_utilities.cpp`](../../tests/unit/math/test_math_utilities.cpp):
  derivatives and Jacobians (including the wrapped-angle difference), measured convergence order
  of Euler (1) and RK4 (4), quadrature, chi-square table values, Cholesky with jitter.
- [`tests/unit/math/test_random.cpp`](../../tests/unit/math/test_random.cpp): engine reference
  value, same seed same sequence, derived streams, moments, unbiased indices.
- [`tests/numerical/test_math_numerics.cpp`](../../tests/numerical/test_math_numerics.cpp):
  wrapping of inputs up to $10^{15}$ within the stated bound, subnormals and signed zero,
  covariance repair, ill-conditioned SPD solves, chi-square for $k = 1000$ and $p = 10^{-10}$,
  log-density underflow, step scaling at $x = 10^{10}$.

## References

- J. Nocedal and S. J. Wright, *Numerical Optimization*, 2nd ed., Springer, 2006.
- M. Matsumoto and T. Nishimura, "Mersenne Twister: A 623-dimensionally equidistributed uniform
  pseudo-random number generator", *ACM Transactions on Modeling and Computer Simulation*, 1998.
- G. E. P. Box and M. E. Muller, "A Note on the Generation of Random Normal Deviates",
  *The Annals of Mathematical Statistics*, 1958.
