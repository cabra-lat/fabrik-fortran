# License and provenance review

- The FABRIK algorithm was implemented from its published description and is
  attributed to Andreas Aristidou's Eurographics 2011 paper and linked summary.
- `yamahigashi/fabric-fabrik-fullbody-ik` was reviewed for full-body/closed-loop
  design ideas. It is MIT licensed but its source was not copied; this package
  intentionally implements only a deterministic single chain.
- Xu et al., *A Combined Inverse Kinematics Algorithm Using FABRIK with
  Optimization* (arXiv:2209.02532), was read as an algorithm reference for
  convergence behaviour under tight error constraints. It is cited in the
  documentation; no code, text, or data was copied.
- No source, image, text, or binary was copied from the reference website,
  GodotIK, monxa, or any other implementation.
- This repository's original Fortran, C ABI, tests, metadata, and docs are MIT
  licensed.
- FPM and a Fortran runtime are build dependencies and retain their own
  licenses/toolchain terms. They are not vendored here.
- The Godot adapter is a separate repository and fetches `godot-cpp` source
  under its own MIT license; it does not vendor opaque binaries.
- A production integration still requires a project-level dependency/license
  review and Windows toolchain validation.
