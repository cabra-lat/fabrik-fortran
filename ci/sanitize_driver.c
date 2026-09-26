/* Minimal C driver for the sanitizer job: exercises the flat C ABI with a
 * reachable solve, an unreachable solve, aliased buffers and a null optional
 * pointer, then a non-finite input that must be rejected rather than propagated. */
#include "fabrik_core.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void expect(int condition, const char *label) {
    if (condition) {
        printf("PASS  %s\n", label);
    } else {
        failures++;
        printf("FAIL  %s\n", label);
    }
}

int main(void) {
    float joints[9] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f};
    float lengths[2] = {1.0f, 1.0f};
    float target[3] = {1.0f, 1.0f, 0.0f};
    float out[9];
    float sentinel[9] = {7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f};
    float measured[2];
    float far_target[3] = {0.0f, 4.0f, 0.0f};
    float nan_target[3] = {NAN, 1.0f, 0.0f};
    float residual = 0.0f;
    int32_t status;
    int i;

    status = fabrik_solve_f32(joints, 3, lengths, target, 1, 1.0e-5f, 64, out, &residual);
    expect(status == FABRIK_OK, "reachable chain converges");
    expect(residual <= 1.0e-5f, "reachable chain residual within tolerance");

    /* aliased input/output buffers must behave like a separate copy */
    memcpy(out, joints, sizeof(joints));
    status = fabrik_solve_f32(out, 3, lengths, target, 1, 1.0e-5f, 64, out, &residual);
    expect(status == FABRIK_OK, "aliased in/out buffers accepted");

    /* null lengths means "measure the current chain" */
    status = fabrik_solve_f32(joints, 3, NULL, target, 1, 1.0e-5f, 64, out, &residual);
    expect(status == FABRIK_OK, "null lengths measures the chain");

    status = fabrik_solve_f32(joints, 3, lengths, far_target, 1, 1.0e-5f, 64, out, measured);
    expect(status == FABRIK_UNREACHABLE, "unreachable target is reported");

    memcpy(out, sentinel, sizeof(out));
    status = fabrik_solve_f32(joints, 3, lengths, nan_target, 1, 1.0e-5f, 64, out, NULL);
    expect(status == FABRIK_INVALID_ARGUMENT, "NaN target is rejected");
    expect(memcmp(out, sentinel, sizeof(out)) == 0, "rejected input leaves C output unchanged");

    status = fabrik_solve_f32(joints, 0, lengths, target, 1, 1.0e-5f, 64, out, &residual);
    expect(status == FABRIK_INVALID_ARGUMENT, "zero-joint chain is rejected");

    /* ---- the chain pipeline, through the same flat C ABI ---- */

    {
        float chain[12] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 2.0f, 1.0f, 0.0f};
        float out_chain[12];
        float seg[3];
        float pole[3] = {0.0f, 0.0f, 5.0f};
        float quats[16], previous[16], eased[12], eased_quats[16];
        float residual2 = 0.0f;
        float blended[3];
        float current[3] = {1.0f, 2.0f, 3.0f};
        float goal[3] = {3.0f, 6.0f, 9.0f};
        float limit_pole[3] = {1.0f, 0.0f, 0.0f};
        int32_t projections = 0, violations = 0;
        int32_t edges[2] = {1, 0};
        int32_t order[2] = {-1, -1};

        status = fabrik_measure_lengths_f32(chain, 4, seg);
        expect(status == FABRIK_OK, "pipeline: lengths measured");
        expect(fabsf(seg[0] - 1.0f) < 1.0e-5f, "pipeline: first segment is 1.0");
        expect(fabsf(seg[1] - 1.0f) < 1.0e-5f, "pipeline: second segment is 1.0");

        status = fabrik_residual_f32(chain, 4, limit_pole, &residual2);
        expect(status == FABRIK_OK, "pipeline: residual measured");
        /* tip is (2,1,0) and the target is (1,0,0), so the distance is sqrt(2). */
        expect(fabsf(residual2 - sqrtf(2.0f)) < 1.0e-4f, "pipeline: residual is the tip distance");

        /* A null residual pointer is a caller error, not a silent no-op. */
        status = fabrik_residual_f32(chain, 4, limit_pole, NULL);
        expect(status == FABRIK_INVALID_ARGUMENT, "pipeline: null residual pointer is rejected");

        memcpy(out_chain, chain, sizeof(chain));
        status = fabrik_apply_pole_f32(out_chain, 4, pole, out_chain);
        expect(status == FABRIK_OK, "pipeline: pole target applied");
        /* Joint 2 (flat index 8) is the bend; it must leave the XY plane. The
         * ROOT is indices 0-2 and a pole target must not move it. */
        expect(fabsf(out_chain[8]) > 1.0e-3f, "pipeline: pole target moved the bend out of plane");
        expect(out_chain[0] == chain[0] && out_chain[1] == chain[1] && out_chain[2] == chain[2],
               "pipeline: pole target left the root alone");
        expect(out_chain[9] == chain[9] && out_chain[10] == chain[10] && out_chain[11] == chain[11],
               "pipeline: pole target left the tip alone");

        status = fabrik_apply_pole_f32(out_chain, 4, chain, out_chain);
        expect(status == FABRIK_OK, "pipeline: zero pole target is accepted");

        /* A 90 degree bend against a 60 degree limit must be projected. */
        {
            float limits[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 60.0f, 0.0f, 0.0f};
            memcpy(out_chain, chain, sizeof(chain));
            status = fabrik_apply_joint_limits_f32(out_chain, 4, limits, 4, 4, out_chain,
                                                    &projections, &violations);
            expect(status == FABRIK_OK, "pipeline: joint limits applied");
            expect(projections > 0, "pipeline: a violated limit is projected");
            expect(violations == 0, "pipeline: a satisfiable limit reports no violations");
        }

        status = fabrik_derive_rotations_f32(chain, 4, quats);
        expect(status == FABRIK_OK, "pipeline: rotations derived");
        {
            float norm = 0.0f;
            for (i = 0; i < 4; i++) norm += quats[i] * quats[i];
            expect(fabsf(norm - 1.0f) < 1.0e-4f, "pipeline: derived rotation is a unit quaternion");
        }

        memcpy(previous, quats, sizeof(quats));
        status = fabrik_smooth_rotations_f32(chain, 4, seg, previous, quats, 0.0f,
                                             eased, eased_quats);
        expect(status == FABRIK_OK, "pipeline: smoothing accepted");
        expect(memcmp(eased, chain, sizeof(chain)) == 0, "pipeline: zero smoothing is a copy");
        {
            float stretched = sqrtf((eased[3] - eased[0]) * (eased[3] - eased[0]) +
                                    (eased[4] - eased[1]) * (eased[4] - eased[1]) +
                                    (eased[5] - eased[2]) * (eased[5] - eased[2]));
            expect(fabsf(stretched - 1.0f) < 1.0e-4f,
                   "pipeline: smoothing preserved the first segment length");
        }

        status = fabrik_blend_influence_f32(current, goal, 0.5f, blended);
        expect(status == FABRIK_OK, "pipeline: influence blended");
        expect(fabsf(blended[0] - 2.0f) < 1.0e-5f, "pipeline: influence 0.5 is the midpoint");

        status = fabrik_order_dependencies_f32(2, edges, 1, order);
        expect(status == FABRIK_OK, "pipeline: dependency order computed");
        expect(order[0] == 0 && order[1] == 1, "pipeline: the parent is solved first");

        /* A cycle must be refused, and must not leave a truncated order behind. */
        {
            int32_t cyclic[4] = {0, 1, 1, 0};
            int32_t cyclic_order[2] = {-7, -7};
            status = fabrik_order_dependencies_f32(2, cyclic, 2, cyclic_order);
            expect(status == FABRIK_CYCLE, "pipeline: a cycle is refused");
            expect(cyclic_order[0] == -7, "pipeline: a refused order writes nothing");
        }

        /* Non-finite input is rejected at the boundary, and the output buffer is
         * left exactly as the caller left it. */
        {
            float bad[12];
            float out_guard[12];
            memcpy(bad, chain, sizeof(chain));
            bad[5] = NAN;
            memcpy(out_guard, chain, sizeof(chain));
            out_guard[5] = 12345.0f;
            status = fabrik_derive_rotations_f32(bad, 4, out_guard);
            expect(status == FABRIK_INVALID_ARGUMENT, "pipeline: NaN input is rejected");
            expect(fabsf(out_guard[5] - 12345.0f) < 1.0e-6f,
                   "pipeline: a rejected call leaves the output untouched");
        }
    }

    printf("RESULT: %s (sanitizer driver, %d failure(s))\n", failures == 0 ? "PASS" : "FAIL", failures);
    return failures == 0 ? 0 : 1;
}
