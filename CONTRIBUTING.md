# Contributing to RoboticsCore

Thank you for considering a contribution. Bug reports, documentation fixes, tests and new
classical algorithms are all welcome. This project contains no learned or data-driven
components.

## Workflow

1. Open an issue that describes the bug or the proposed algorithm. For a new algorithm, cite
   the original reference and explain what it adds beyond the existing ones.
2. Fork the repository and create a topic branch.
3. Make focused commits with messages in the form `type(scope): summary`. For example,
   `fix(planning): handle start == goal in theta_star`. Types: `feat`, `fix`, `perf`,
   `refactor`, `test`, `docs`, `build`, `ci`, `chore`.
4. Run the checks below locally and open a pull request against `main`.

## Local checks

```bash
cmake --preset ci-debug && cmake --build --preset ci-debug && ctest --preset ci-debug
cmake --preset sanitize && cmake --build --preset sanitize && ctest --preset sanitize   # GCC / Clang
clang-format --dry-run --Werror $(git ls-files '*.hpp' '*.cpp')
ruff check python scripts && ruff format --check python scripts
pytest
python scripts/docs/generate_status.py --check
```

Static analysis: configure with `-DROBOTICSCORE_ENABLE_CLANG_TIDY=ON`. The pipeline in
[`Jenkinsfile`](Jenkinsfile) runs the same commands.

## Coding standards

- C++20, formatted with the repository's `.clang-format`. Follow the `.clang-tidy` rules.
- Public headers carry Doxygen comments. Each comment states units, frames, preconditions
  (`@throws`) and complexity where relevant.
- Follow the error policy in [ADR 0002](docs/adr/0002-error-handling.md):
  - `std::invalid_argument` for contract violations;
  - `NumericalError` for numerical breakdown;
  - a status in a result struct for expected outcomes.
- No hidden global state. Randomness comes from an explicitly passed `math::Random`
  ([ADR 0004](docs/adr/0004-determinism.md)).
- No I/O inside the library. Serialisation belongs in `apps/`.
- New dependencies need an ADR. The library itself depends only on Eigen.

## What a new algorithm needs

| Item | Location |
|---|---|
| Header with documented API, config struct with defaults, result struct with status | `include/roboticscore/<area>/` |
| Implementation | `src/<area>/` |
| Unit tests for analytic cases, edge cases and invalid inputs | `tests/unit/<area>/` |
| Numerical or regression tests where they apply | `tests/numerical/`, `tests/regression/` |
| Algorithm page following the template in [docs/README.md](docs/README.md) | `docs/algorithms/<area>/` |
| Registry entry | [`configs/algorithms.yaml`](configs/algorithms.yaml) |
| Optional: demo, animation, benchmark | `apps/`, `python/`, `benchmarks/` |

## Documentation and benchmarks

- References must be real and verifiable: authors, title, venue and year. Do not cite page or
  theorem numbers you have not checked.
- Benchmark numbers in the documentation must come from `scripts/benchmark/run_benchmarks.py`
  run on a Release build. Never edit them by hand.
- Do not claim novelty or "state of the art". Describe what the code does and how it was
  measured.

## Licensing

By contributing, you agree that your contributions are licensed under the [MIT License](LICENSE).
