# FABRIK algorithm contract

References: Andreas Aristidou, *FABRIK: A Fast, Iterative Solver for the
Inverse Kinematics Problem* (Eurographics 2011), summary page:
<https://andreasaristidou.com/Fabrik>; and the MIT-licensed discontinued
`yamahigashi/fabric-fabrik-fullbody-ik` project for full-body/closed-loop design
context; and Xu et al., *A Combined Inverse Kinematics Algorithm Using FABRIK
with Optimization* (arXiv:2209.02532) for the observed instability of FABRIK
under very tight error constraints. This document describes an independent
single-chain implementation; it contains no copied reference code.

Xu et al. refine a FABRIK result with SQP when a high-accuracy solver is needed.
This core keeps the contract smaller and deterministic: it reports
`FABRIK_NOT_CONVERGED` when the iteration budget is exhausted at a tolerance it
cannot reach, and deliberately ships no optimizer stage.

## Input validation

Every caller-supplied value is screened for finiteness before any arithmetic.
`x < 0` is false for NaN, so range checks alone cannot reject it; a NaN target
would otherwise propagate silently through the iterations. Non-finite joints,
target, segment lengths or tolerance return `FABRIK_INVALID_ARGUMENT` and leave
the caller's output buffer untouched.

## Convergence reporting

FABRIK converges linearly near the solution, so a tight tolerance combined with a
small iteration budget legitimately ends in `FABRIK_NOT_CONVERGED` - the status
exists so callers can distinguish "did not finish" from "cannot finish". The
property test asserts each status against its own invariant: `OK` implies the
residual is inside tolerance and the target within reach, `NOT_CONVERGED` implies
the residual is still above tolerance, `UNREACHABLE` implies the target lies
outside the chain's total length.

Primary sources for the algorithm and its known failure modes:

- Aristidou & Lasenby, *FABRIK: A fast, iterative solver for the Inverse
  Kinematics problem*, Graphical Models 73(5), 2011,
  <https://doi.org/10.1016/j.gmod.2011.05.003>.
- Aristidou, Chrysanthou & Lasenby, *Extending FABRIK with model constraints*,
  Computer Animation and Virtual Worlds, 2015,
  <https://doi.org/10.1002/cav.1630> (closed loops, leaf joints, fixed
  inter-joint distance, unreachable-target behaviour, convergence proof).
- Santos et al., *FABRIK-R: An Extension Developed Based on FABRIK for Robotics
  Manipulators*, IEEE Access, 2021,
  <https://doi.org/10.1109/ACCESS.2021.3070693> (singularities on 1-DOF joints).
- Xu et al., *A Combined Inverse Kinematics Algorithm Using FABRIK with
  Optimization*, arXiv:2209.02532, 2022,
  <https://arxiv.org/abs/2209.02532> (convergence under tight error bounds).

For `n` joints, segment `i` has length `length(i)` from joint `i` to joint
`i + 1` in row-major coordinates.

1. Reject fewer than two joints, a non-positive iteration budget, or negative
   tolerance as `INVALID_ARGUMENT`.
2. Reject any segment no longer than `1e-7` as `DEGENERATE_CHAIN`; output is
   copied unchanged.
3. Return `OK` if the original tip is already within tolerance.
4. If root-to-target distance exceeds the summed segment length plus tolerance,
   extend the chain in a deterministic straight line. Return `UNREACHABLE` and
   the final residual; this is a failure, never a false successful solve.
5. Repeat until tolerance or iteration budget:
   - Backward: pin tip to target, then walk toward the root preserving each
     segment length.
   - Forward: pin the root (original root when anchored, previous tip when
     free), then walk toward the tip preserving each segment length.
   - Return `OK` when the tip is within tolerance.
6. Return `NOT_CONVERGED` with the last deterministic chain if the budget is
   exhausted.

The implementation performs no randomness, time dependence, threading, or
compiler-specific ABI operation. All status values are stable integers in
`include/fabrik_core.h`.
