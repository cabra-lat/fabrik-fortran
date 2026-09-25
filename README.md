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
- Xu et al., *A Combined Inverse Kinematics Algorithm Using FABRIK with
  Optimization* (arXiv:2209.02532, 2022),
  <https://arxiv.org/abs/2209.02532>, documents unstable FABRIK convergence
  under very tight Cartesian-error constraints and seeds an SQP refiner with the
  FABRIK result. This motivates the explicit `FABRIK_NOT_CONVERGED` status; the
  optimization stage is deliberately out of scope for this first core.

The FABRIK algorithm itself is due to Aristidou and Lasenby, *FABRIK: A fast,
iterative solver for the Inverse Kinematics problem*, Graphical Models 73(5),
2011, <https://doi.org/10.1016/j.gmod.2011.05.003>; the model-constraint,
closed-loop and unreachable-target extensions are Aristidou, Chrysanthou and
Lasenby, *Extending FABRIK with model constraints*, Computer Animation and
Virtual Worlds, 2015, <https://doi.org/10.1002/cav.1630>; and the
single-DOF-joint manipulator treatment is Santos et al., *FABRIK-R: An
Extension Developed Based on FABRIK for Robotics Manipulators*, IEEE Access,
2021, <https://doi.org/10.1109/ACCESS.2021.3070693>. These works are cited as
the source of the algorithm; no code, text, or figures were copied from them.
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

The tests cover convergence, unreachable targets, zero-length chains, bitwise
determinism, rejection of NaN/Inf input, minimum chain size, and a
deterministic 200-chain property test that asserts every status is
self-consistent and that segment lengths survive every solve.
`fpm_status_string` is available for diagnostics; callers must use the numeric
status as the stable contract.

> If you change compiler flags or switch profiles, delete `build/` before
> rebuilding. `fpm clean` does not remove the per-flag-set directories FPM
> keeps, and a stale object can silently be linked into the test binary.

An ASan/UBSan build of the C ABI is driven by `ci/sanitize.sh` (run by CI):

```sh
bash ci/sanitize.sh
```

## Flat ABI

`include/fabrik_core.h` is the only public ABI. It uses contiguous `float`
buffers, fixed-width integers, and one target coordinate array. No Fortran
derived type, allocatable descriptor, or compiler runtime object crosses into
C/C++.

The FPM package and Nix derivation build a static library. Binaries and build
directories are ignored and are not source dependencies. GitHub CI runs the
FPM tests, build, and example on Linux, macOS, and Windows, plus an
ASan/UBSan job over the C ABI; Nix supplies the locally verified reproducible
derivation.

## License

The original code in this repository is MIT licensed (see `LICENSE`).
Third-party dependencies retain their own licenses. The algorithm itself is not
presented as new code; see `docs/LICENSE_REVIEW.md`.
