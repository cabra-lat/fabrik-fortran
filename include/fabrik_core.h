#ifndef FABRIK_CORE_H
#define FABRIK_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum FabrikStatus {
    FABRIK_OK = 0,
    FABRIK_INVALID_ARGUMENT = 1,
    FABRIK_UNREACHABLE = 2,
    FABRIK_NOT_CONVERGED = 3,
    FABRIK_DEGENERATE_CHAIN = 4,
};

/*
 * Solve one 3D chain in a flat row-major buffer.
 *
 * joints:      3 * joint_count input coordinates; overwritten by out_joints.
 * lengths:     joint_count - 1 segment lengths, or NULL to measure input.
 * out_joints:  3 * joint_count writable coordinates (may alias joints).
 * out_residual: optional writable final tip-to-target distance.
 *
 * The ABI contains only integers and contiguous float arrays. No Fortran
 * derived type or compiler-specific symbol crosses this boundary.
 */
int32_t fabrik_solve_f32(
    const float *joints,
    int32_t joint_count,
    const float *lengths,
    const float target_xyz[3],
    int32_t root_anchored,
    float tolerance,
    int32_t max_iterations,
    float *out_joints,
    float *out_residual
);

const char *fabrik_status_string(int32_t status);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
