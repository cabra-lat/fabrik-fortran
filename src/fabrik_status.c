#include "fabrik_core.h"

const char *fabrik_status_string(int32_t status) {
    switch (status) {
        case FABRIK_OK: return "OK";
        case FABRIK_INVALID_ARGUMENT: return "INVALID_ARGUMENT";
        case FABRIK_UNREACHABLE: return "UNREACHABLE";
        case FABRIK_NOT_CONVERGED: return "NOT_CONVERGED";
        case FABRIK_DEGENERATE_CHAIN: return "DEGENERATE_CHAIN";
        default: return "UNKNOWN";
    }
}
