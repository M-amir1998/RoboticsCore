# ADR 0005: MIT license

Status: Accepted

## Context

The repository was created with an MIT license. The library is meant for study, teaching,
research prototypes and integration into other projects, including commercial ones.

## Decision

The MIT license covers all code, documentation, scripts and generated assets in this
repository. Dependencies keep their own licenses (see [ADR 0003](0003-dependencies.md)).
None of them is copyleft. Eigen's MPL-2.0 applies file by file, to Eigen's own headers only.

## Consequences

- Downstream users may use, modify and redistribute the library, provided they keep the
  attribution.
- Contributions are accepted under the same license (see
  [`CONTRIBUTING.md`](../../CONTRIBUTING.md)).
