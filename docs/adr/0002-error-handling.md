# ADR 0002: Error handling

Status: Accepted

## Context

Robotics algorithms fail in three different ways:

1. **Contract violation.** The caller broke a precondition: mismatched matrix sizes, a
   negative time step, or a start cell outside the map. This is a programming error.
2. **Numerical breakdown.** A computation failed although the inputs were formally valid. For
   example, a covariance lost positive definiteness or a normal-equation system was singular.
3. **Expected outcome.** The algorithm ran correctly but did not reach its goal. For example,
   no path exists, an iterative solver hit its iteration limit, or a planner used up its
   sampling budget.

Handling all three the same way causes problems either way:

- Status codes for contract violations hide bugs, because callers can easily ignore them.
- Exceptions for normal outcomes make ordinary control flow exception-driven, for example
  throwing when no path exists.

`std::expected` would suit case 3, but it needs C++23 and the library targets C++20.

## Decision

The policy is defined in
[`include/roboticscore/core/error.hpp`](../../include/roboticscore/core/error.hpp):

| Case | Mechanism |
|---|---|
| Contract violation | `std::invalid_argument`, raised through `detail::require(condition, message)` |
| Numerical breakdown | `roboticscore::NumericalError`, derived from `std::runtime_error` |
| Expected algorithmic outcome | A result struct with an explicit status or success flag; never throws |

Every public function documents the exceptions it can raise with `@throws`.

Result structs carry the status together with diagnostics such as iteration counts, residuals
and expanded nodes. Examples are `GridPath` (with `PlanStatus`), `SamplingResult`,
`HybridAStarResult`, `IcpResult`, `SolverResult`, `QpResult` and `PoseGraphResult`.

## Consequences

- If no exception is thrown, the algorithm ran. Callers must still check the returned status
  to learn whether it succeeded.
- Precondition checks stay on in all builds, including Release. They are cheap compared with
  the algorithms they guard. Hot inner loops do not re-check invariants that a constructor has
  already established.
- Unit tests check both the exception type thrown for invalid inputs and the status reported
  for runs that are valid but unsuccessful.
