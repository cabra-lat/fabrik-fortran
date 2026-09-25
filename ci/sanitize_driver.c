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

    printf("RESULT: %s (sanitizer driver, %d failure(s))\n", failures == 0 ? "PASS" : "FAIL", failures);
    return failures == 0 ? 0 : 1;
}
