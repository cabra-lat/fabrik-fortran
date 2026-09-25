# fabrik_core

Standalone deterministic 3D Forward And Backward Reaching Inverse Kinematics
(FABRIK) prototype written in Fortran, exposed through a flat C ABI.

This repository is independent from the game and shooter addon. It does not
replace production IK. No third-party source code is copied into this package.

## Algorithm and provenance

The implementation follows the published algorithm description:

- Andreas Aristidou, *FABRIK: A Fast, Iterative Solver for the Inverse Kinematics
  Problem*, Eurographics 2011, <https://andreasaristidou.com/Fabrik>.
- [yamahigashi/fabric-fabrik-fullbody-ik](https://github.com/yamahigashi/fabric-fabrik-fullbody-ik)
  was inspected as a design reference for full-body/closed-loop FABRIK. It is an
  MIT-licensed discontinued Fabric Engine project; no source was copied.
- The original article and reference implementations were used only as
  algorithm/design references. This implementation, API, tests, and
  documentation were written for this prototype.

FABRIK alternates a backward pass (tip constrained to the target) with a
forward pass (root and segment lengths restored), then checks tip error. See
`docs/ALGORITHM.md` for the exact contract.

## Build and test

Dependencies: Fortran compiler, FPM, and a C/C++ compiler.

```sh
nix shell nixpkgs#fortran-fpm nixpkgs#gfortran -c \
  fortran-fpm test --profile release
nix shell nixpkgs#fortran-fpm nixpkgs#gfortran -c \
  fortran-fpm build --profile release

# Reproducible install prefix from the pinned Nix flake:
nix build .#packages.x86_64-linux.default \
  --no-link --print-out-paths
```

The tests cover convergence, unreachable targets, zero-length chains, and
bitwise determinism. `fpm_status_string` is available for diagnostics; callers
must use the numeric status as the stable contract.

## Flat ABI

`include/fabrik_core.h` is the only public ABI. It uses contiguous `float`
buffers, fixed-width integers, and one target coordinate array. No Fortran
derived type, allocatable descriptor, or compiler runtime object crosses into
C/C++.

The FPM package and Nix derivation build a static library. Binaries and build
directories are ignored and are not source dependencies. GitHub CI runs the
FPM tests, build, and example on Linux, macOS, and Windows; Nix supplies the
locally verified reproducible derivation.

## License

The original code in this repository is MIT licensed (see `LICENSE`).
Third-party dependencies retain their own licenses. The algorithm itself is not
presented as new code; see `docs/LICENSE_REVIEW.md`.
