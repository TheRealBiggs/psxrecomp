/*
 * frame_pacing.c — race-free wall-clock frame pacing.
 * See frame_pacing.h for the Bug B history this replaces.
 */
#include "frame_pacing.h"

#include <math.h>
#include <string.h>

/* FRAME_PACING_PURE_ONLY: tests compile only the SDL-free decision
 * function (tests/test_frame_pacing.c includes this file directly). */
#ifndef FRAME_PACING_PURE_ONLY
#ifdef FRAME_PACING_TEST_STUBS
uint64_t SDL_GetPerformanceFrequency(void);
uint64_t SDL_GetPerformanceCounter(void);
void psx_host_sleep_ms(unsigned ms);
#else
#include "psx_sdl.h"
#include "host_time.h"
#endif
#endif

uint32_t frame_pacing_sleep_ms(uint64_t now, uint64_t deadline,
                               uint64_t freq, uint64_t period) {
    if (now >= deadline) return 0;            /* compare BEFORE subtract */
    uint64_t remaining = deadline - now;       /* cannot underflow */
    if (remaining > period) remaining = period;/* hard cap: one frame max */
    if (freq == 0) return 0;
    /* remaining <= period (~one frame of ticks), so *1000 cannot overflow. */
    uint64_t ms = (remaining * 1000u) / freq;
    if (ms < 2) return 0;                      /* sub-2ms: spin instead */
    return (uint32_t)(ms - 1);                 /* undershoot; spin covers rest */
}

#ifndef FRAME_PACING_PURE_ONLY

/* Bounded catch-up window, in periods. A transient stall (heavy frame, CD
 * burst) leaves next_deadline in the past; KEEPING that debt and running
 * unpaced until it is repaid preserves the long-term rate at exactly one
 * period per frame — which the audio pipeline depends on (the SPU produces
 * 768 guest cycles per output sample; every re-anchor that forgives debt
 * permanently starves the audio ring by the forgiven amount. Measured on
 * MMX5: forgiving all >1-period debt averaged 59.80 Hz against a 59.94
 * target = -0.4% chronic audio underrun). Only debt beyond this window —
 * sustained sub-realtime emulation, not a hiccup — is forgiven, else the
 * pacer would demand unbounded catch-up. */
/* Vigilante 8's streamed FMV transitions have measured host stalls near
 * 140 ms. Eight 60 Hz periods are only 133.3 ms, so the old bound classified
 * those finite transitions as sustained slowness and permanently forgave the
 * guest/audio debt. Keep a bounded 12-period (200 ms at 60 Hz) window: enough
 * to repay the observed transition without turning a real hang, suspend, or
 * sub-realtime workload into an unbounded catch-up burst. */
#define FRAME_PACER_CATCHUP_MAX_PERIODS 12u
#define FRAME_PACER_MAX_PERIOD_MS 1000.0

static int frame_pacer_period_to_ticks(double period_ms, uint64_t freq,
                                       uint64_t *period_out) {
    if (!period_out) return 0;
    *period_out = 0;
    if (freq == 0)
        return 0;
    if (!isfinite(period_ms) || period_ms <= 0.0 ||
        period_ms > FRAME_PACER_MAX_PERIOD_MS) {
        return 0;
    }
    double ticks = (double)freq * (period_ms / 1000.0);
    if (!isfinite(ticks) || ticks < 1.0)
        return 0;
    if (ticks > (double)(UINT64_MAX / FRAME_PACER_CATCHUP_MAX_PERIODS))
        return 0;
    *period_out = (uint64_t)ticks;
    return *period_out != 0;
}

void frame_pacer_wait(FramePacer *p, double period_ms) {
    if (!p) return;
    uint64_t freq = SDL_GetPerformanceFrequency();
    uint64_t now = SDL_GetPerformanceCounter();
    uint64_t period = 0;

    p->wait_calls++;
    p->last_now = now;
    p->last_freq = freq;
    p->last_period_ms = period_ms;
    p->last_period_ticks = 0;
    p->last_sleep_ms = 0;

    if (!frame_pacer_period_to_ticks(period_ms, freq, &period)) {
        p->invalid_periods++;
        p->next_deadline = 0;
        return;
    }
    p->last_period_ticks = period;

    if (p->next_deadline == 0 ||
        now >= p->next_deadline + period * FRAME_PACER_CATCHUP_MAX_PERIODS) {
        /* First frame, or sustained slowness beyond the catch-up window:
         * re-anchor (forgive the debt). */
        p->next_deadline = now + period;
        p->reanchors++;
        return;
    }
    if (now >= p->next_deadline) {
        /* In debt from a recent stall: run this frame unpaced and advance
         * the deadline, repaying one period of debt per fast frame. */
        p->next_deadline += period;
        p->catchup_skips++;
        return;
    }

    for (;;) {
        now = SDL_GetPerformanceCounter();     /* ONE read per iteration */
        uint32_t ms = frame_pacing_sleep_ms(now, p->next_deadline, freq, period);
        if (ms == 0) break;
        p->last_now = now;
        p->last_sleep_ms = ms;
        /* Waitable timer on Win32; usleep on Unix — not coarse Sleep/SDL_Delay. */
        psx_host_sleep_ms(ms);
    }
    while (SDL_GetPerformanceCounter() < p->next_deadline) {
        /* final sub-ms spin */
    }
    p->next_deadline += period;
}

void frame_pacer_get_diag(const FramePacer *p, FramePacerDiag *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!p) return;
    out->next_deadline = p->next_deadline;
    out->wait_calls = p->wait_calls;
    out->reanchors = p->reanchors;
    out->catchup_skips = p->catchup_skips;
    out->invalid_periods = p->invalid_periods;
    out->last_now = p->last_now;
    out->last_freq = p->last_freq;
    out->last_period_ticks = p->last_period_ticks;
    out->last_sleep_ms = p->last_sleep_ms;
    out->last_period_ms = p->last_period_ms;
}
#endif /* FRAME_PACING_PURE_ONLY */
