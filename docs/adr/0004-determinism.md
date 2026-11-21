# ADR 0004: Explicit seeding and portable random distributions

Status: Accepted

## Context

Particle filters, sampling-based planners and the simulator are stochastic. Regression tests,
benchmarks and documentation animations must still be reproducible.

The C++ standard fully specifies `std::mt19937_64`. It does not specify the algorithms behind
`std::uniform_real_distribution` or `std::normal_distribution`. As a result, libstdc++, libc++
and the MSVC STL produce different variates from the same engine state.

## Decision

- `roboticscore::math::Random`
  ([`random.hpp`](../../include/roboticscore/math/random.hpp)) wraps `std::mt19937_64`. It
  derives every variate (uniform, Gaussian, multivariate Gaussian and index sampling) with
  algorithms implemented in the library.
- There is no global generator. Every stochastic API takes a `Random&` or a seed explicitly.
  Independent streams are derived from a parent seed with `derive(stream)`.
- Demos and benchmarks record the seed and configuration they ran with. The animation
  manifest stores the seed, configuration and commit for each GIF.

## Consequences

- The same seed gives the same random stream with every supported standard library.
- Results can still differ slightly between compilers or CPUs. Floating-point evaluation order
  and math-library functions such as `std::sin` and `std::exp` are not bit-identical
  everywhere.
- Regression tests therefore compare with tolerances. Golden values are recorded for the
  reference configuration (MSVC, x64).
