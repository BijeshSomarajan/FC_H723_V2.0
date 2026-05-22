#include "Interpolator.h"

void interpolatorInit(INTERPOLATOR *i, float initialValue, float duration) {
    i->prev = initialValue;
    i->curr = initialValue;
    i->alpha = 1.0f;
    i->duration = duration;
}

void interpolatorReset(INTERPOLATOR *i, float initialValue) {
    i->prev = initialValue;
    i->curr = initialValue;
    i->alpha = 1.0f;
}

void interpolatorSetTarget(INTERPOLATOR *i, float newTarget) {
    i->prev = i->curr;
    i->curr = newTarget;
    i->alpha = 0.0f;
}

float interpolatorUpdate(INTERPOLATOR *i, float dt) {
    if (i->alpha < 1.0f) {
        i->alpha += dt / i->duration;

        if (i->alpha > 1.0f) {
            i->alpha = 1.0f;
        }
    }
    return i->prev + (i->curr - i->prev) * i->alpha;
}
