# Security policy

## Supported versions

| Version | Supported |
|---|---|
| 0.1.x | Yes |

## Reporting a vulnerability

Please do not report security issues in public issues. Use GitHub's private vulnerability
reporting instead: open the repository's **Security** tab and choose **Report a vulnerability**.
Include:

- the affected version or commit;
- a minimal input or program that reproduces the problem;
- the impact you observed, such as a crash, out-of-bounds access or unbounded resource use.

You should receive an acknowledgement within 7 days. Confirmed issues are fixed on `main` and
noted in [CHANGELOG.md](CHANGELOG.md).

## Scope

RoboticsCore is an algorithms library. It does not open network connections or parse
untrusted file formats, and it runs no external processes. The demo and benchmark executables
read JSON configuration files with nlohmann/json, and the Python tooling runs the demo
executable. Relevant issues include:

- memory-safety bugs reachable through the public API with valid or invalid inputs;
- unbounded memory or time consumption on inputs that pass the documented preconditions.

The library is not certified for safety-critical use. Do not use it to control physical
systems without independent validation.
