# ADR 0006: Jenkins as the only CI system

Status: Accepted

## Context

The project needs a pipeline that configures and builds the code, runs static analysis and runs the test
suites. It should also be able to run sanitizers, benchmarks and documentation rendering on
request. The project uses Jenkins. Running a second CI system alongside it would duplicate the
configuration and could produce diverging results.

## Decision

- The pipeline is defined in the root [`Jenkinsfile`](../../Jenkinsfile) as a declarative
  pipeline. It runs each step with `sh` or `bat`, depending on the agent's operating system.
- The repository contains no `.github/workflows/` directory.
- Expensive stages, such as animation rendering and full benchmarks, run only when a pipeline
  parameter enables them. These parameters are off by default.
- Every stage can be reproduced locally with the CMake presets and scripts it calls.

## Consequences

- The README has no CI status badge; build health is visible in Jenkins.
- Contributors run the same presets locally before opening a pull request.
