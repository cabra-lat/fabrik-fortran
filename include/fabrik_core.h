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
    /* Returned by fabrik_order_dependencies_f32() only: the dependency graph
     * contains a cycle, so no valid order exists. The whole rig is refused
     * rather than partially ordered. */
    FABRIK_CYCLE = 6,
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

/*
 * The chain pipeline.
 *
 * These are the operations the C++ adapter used to own as inline arithmetic.
 * They live here so they are covered by `fpm test` and the sanitizer job, which
 * run in seconds, instead of only by a GDScript test that needs a full godot-cpp
 * build. None of them knows what a Godot type is.
 *
 * All buffers are flat and row-major, in the same layout as fabrik_solve_f32:
 * joints are 3 * joint_count, quaternions are 4 * joint_count in (w, x, y, z),
 * angle limits are 2 * limit_count. Functions that transform a chain write it
 * back through out_joints, which may alias joints.
 *
 * The interface has one shape throughout, so there is no function to remember
 * differently: EVERY routine below returns an int32_t status and writes its
 * results through pointers. Optional outputs are passed as a pointer that may be
 * NULL. As with fabrik_solve_f32, a rejected call leaves the output buffers
 * untouched, and non-finite input is rejected with FABRIK_INVALID_ARGUMENT
 * rather than solved.
 */

/* Segment lengths of the current pose: 3 * joint_count in, joint_count - 1 out. */
int32_t fabrik_measure_lengths_f32(
    const float *joints,
    int32_t joint_count,
    float *out_lengths
);

/* Tip-to-target distance of the current pose. out_residual is required. */
int32_t fabrik_residual_f32(
    const float *joints,
    int32_t joint_count,
    const float target_xyz[3],
    float *out_residual
);

/*
 * Per-joint FLEXION angle in degrees: 0 is straight, 180 is folded back on
 * itself. The two ends have no such angle and report 0.
 */
int32_t fabrik_joint_angles_f32(
    const float *joints,
    int32_t joint_count,
    float *out_angles
);

/*
 * Rotate the intermediate joints about the root-to-tip axis so the bend faces
 * pole_target. Root and tip are fixed and every segment length is preserved. A
 * zero pole target is a no-op.
 */
int32_t fabrik_apply_pole_f32(
    const float *joints,
    int32_t joint_count,
    const float pole_target_xyz[3],
    float *out_joints
);

/*
 * Project the chain into per-joint interior-angle limits, given in DEGREES
 * (180 straight, 0 folded). An entry with x >= y means unlimited, and so does a
 * joint past the end of the array.
 *
 * limit_passes bounds the relaxation, because satisfying one joint can violate
 * the next. out_projections counts the projections applied and out_violations
 * reports the limits still unsatisfied afterwards, so a caller never has to
 * assume the limits hold.
 */
int32_t fabrik_apply_joint_limits_f32(
    const float *joints,
    int32_t joint_count,
    const float *joint_limits,
    int32_t limit_count,
    int32_t limit_passes,
    float *out_joints,
    int32_t *out_projections,
    int32_t *out_violations
);

/*
 * One orientation per bone, with the bone convention local +Y. out_quaternions
 * is 4 * joint_count in (w, x, y, z), which is the order Godot's Quaternion
 * stores, so crossing the boundary is a copy in both directions.
 */
int32_t fabrik_derive_rotations_f32(
    const float *joints,
    int32_t joint_count,
    float *out_quaternions
);

/*
 * Ease towards the solved orientations in rotation space, then rebuild the
 * positions by forward kinematics with the declared lengths. Smoothing in
 * POSITION space would change every segment length and stretch the bones, which
 * is why this exists at all.
 *
 * previous and rotations are both 4 * joint_count; out_joints is 3 * joint_count
 * and out_quaternions is 4 * joint_count. A smoothing value of 0 is a copy.
 */
int32_t fabrik_smooth_rotations_f32(
    const float *joints,
    int32_t joint_count,
    const float *lengths,
    const float *previous_quaternions,
    const float *quaternions,
    float smoothing,
    float *out_joints,
    float *out_quaternions
);

/* GodotIK's influence, matched exactly: lerp the current position towards the
 * goal. influence is clamped to 0-1. */
int32_t fabrik_blend_influence_f32(
    const float current_xyz[3],
    const float goal_xyz[3],
    float influence,
    float out_xyz[3]
);

/*
 * Deterministic dependency order: every parent before its children, ties broken
 * by declaration index. edges is 2 * edge_count of (child, parent) pairs; out_order
 * is node_count entries. Returns FABRIK_CYCLE and writes nothing usable if the
 * graph contains a cycle - a partial order would solve half a rig.
 */
int32_t fabrik_order_dependencies_f32(
    int32_t node_count,
    const int32_t *edges,
    int32_t edge_count,
    int32_t *out_order
);
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
