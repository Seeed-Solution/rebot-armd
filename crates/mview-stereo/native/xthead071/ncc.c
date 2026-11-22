#include <riscv_vector.h>

#include "ncc.h"

static void mvs071_coarse_dots_n5(const uint8_t *p, size_t stride,
                                  const float *a, size_t count, float *out) {
    for (size_t done = 0; done < count;) {
        size_t vl = vsetvl_e32m4(count - done);
        vfloat32m4_t acc = vfmv_v_f_f32m4(0.0f, vl);
        for (size_t pj = 0; pj < 5; pj++) {
            const uint8_t *row = p + pj * stride + done;
            const float *ar = a + pj * 5;
            for (size_t pi = 0; pi < 5; pi++) {
                vuint8m1_t b = vle8_v_u8m1(row + pi, vl);
                vuint16m2_t h = vwcvtu_x_x_v_u16m2(b, vl);
                vfloat32m4_t v = vfwcvt_f_xu_v_f32m4(h, vl);
                vfloat32m4_t prod = vfmul_vf_f32m4(v, ar[pi], vl);
                acc = vfadd_vv_f32m4(acc, prod, vl);
            }
        }
        vse32_v_f32m4(out + done, acc, vl);
        done += vl;
    }
}

static void mvs071_coarse_dots_general(const uint8_t *p, size_t stride,
                                       const float *a, size_t n,
                                       size_t count, float *out) {
    for (size_t done = 0; done < count;) {
        size_t vl = vsetvl_e32m4(count - done);
        vfloat32m4_t acc = vfmv_v_f_f32m4(0.0f, vl);
        for (size_t pj = 0; pj < n; pj++) {
            const uint8_t *row = p + pj * stride + done;
            const float *ar = a + pj * n;
            for (size_t pi = 0; pi < n; pi++) {
                vuint8m1_t b = vle8_v_u8m1(row + pi, vl);
                vuint16m2_t h = vwcvtu_x_x_v_u16m2(b, vl);
                vfloat32m4_t v = vfwcvt_f_xu_v_f32m4(h, vl);
                vfloat32m4_t prod = vfmul_vf_f32m4(v, ar[pi], vl);
                acc = vfadd_vv_f32m4(acc, prod, vl);
            }
        }
        vse32_v_f32m4(out + done, acc, vl);
        done += vl;
    }
}

void mvs071_coarse_dots(const uint8_t *p, size_t stride,
                        const float *a, size_t n, size_t count,
                        float *out) {
    if (count == 0 || n == 0) {
        return;
    }
    if (n == 5) {
        mvs071_coarse_dots_n5(p, stride, a, count, out);
    } else {
        mvs071_coarse_dots_general(p, stride, a, n, count, out);
    }
}

static inline __attribute__((always_inline)) vuint16m2_t
mvs071_load_half_sum(const uint8_t *r0, const uint8_t *r1, size_t col,
                     uint32_t phase, size_t vl) {
    vuint8m1_t b00 = vle8_v_u8m1(r0 + col, vl);
    if (phase == 0) {
        return vwcvtu_x_x_v_u16m2(b00, vl);
    }
    if (phase == 1) {
        vuint8m1_t b01 = vle8_v_u8m1(r0 + col + 1, vl);
        return vwaddu_vv_u16m2(b00, b01, vl);
    }
    if (phase == 2) {
        vuint8m1_t b10 = vle8_v_u8m1(r1 + col, vl);
        return vwaddu_vv_u16m2(b00, b10, vl);
    }
    vuint8m1_t b01 = vle8_v_u8m1(r0 + col + 1, vl);
    vuint8m1_t b10 = vle8_v_u8m1(r1 + col, vl);
    vuint8m1_t b11 = vle8_v_u8m1(r1 + col + 1, vl);
    return vadd_vv_u16m2(vwaddu_vv_u16m2(b00, b01, vl),
                         vwaddu_vv_u16m2(b10, b11, vl), vl);
}

static inline __attribute__((always_inline)) vfloat32m4_t
mvs071_scale_half_sample(vuint16m2_t s, uint32_t phase, size_t vl) {
    vfloat32m4_t sample = vfwcvt_f_xu_v_f32m4(s, vl);
    if (phase == 1 || phase == 2) {
        return vfmul_vf_f32m4(sample, 0.5f, vl);
    }
    if (phase == 3) {
        return vfmul_vf_f32m4(sample, 0.25f, vl);
    }
    return sample;
}

static inline __attribute__((always_inline)) void
mvs071_half_moments_n9_phase(const uint8_t *p, size_t stride, const float *a,
                             uint32_t phase, size_t vl, uint32_t *sum_raw,
                             uint32_t *sumsq_raw, float *dot) {
    vuint32m4_t vsum = vmv_v_x_u32m4(0, vl);
    vuint32m4_t vsumsq = vmv_v_x_u32m4(0, vl);
    vfloat32m1_t dacc = vfmv_v_f_f32m1(0.0f, 1);

    for (size_t j = 0; j < 9; j++) {
        const uint8_t *r0 = p + j * stride;
        const uint8_t *r1 = (phase >= 2) ? (r0 + stride) : r0;
        const float *ar = a + j * 9;

        vuint16m2_t s = mvs071_load_half_sum(r0, r1, 0, phase, vl);
        vsum = vwaddu_wv_u32m4(vsum, s, vl);
        vsumsq = vadd_vv_u32m4(vsumsq, vwmulu_vv_u32m4(s, s, vl), vl);

        vfloat32m4_t sample = mvs071_scale_half_sample(s, phase, vl);
        vfloat32m4_t prod = vfmul_vv_f32m4(vle32_v_f32m4(ar, vl), sample, vl);
        dacc = vfredosum_vs_f32m4_f32m1(dacc, prod, dacc, vl);
    }

    vuint32m1_t z = vmv_v_x_u32m1(0, 1);
    *sum_raw = vmv_x_s_u32m1_u32(vredsum_vs_u32m4_u32m1(z, vsum, z, vl));
    *sumsq_raw = vmv_x_s_u32m1_u32(vredsum_vs_u32m4_u32m1(z, vsumsq, z, vl));
    *dot = vfmv_f_s_f32m1_f32(dacc);
}

static inline __attribute__((always_inline)) void
mvs071_half_moments_general_phase(const uint8_t *p, size_t stride,
                                  const float *a, size_t n, uint32_t phase,
                                  uint32_t *sum_raw, uint32_t *sumsq_raw,
                                  float *dot) {
    vuint32m1_t vsum = vmv_v_x_u32m1(0, 1);
    vuint32m1_t vsumsq = vmv_v_x_u32m1(0, 1);
    vfloat32m1_t dacc = vfmv_v_f_f32m1(0.0f, 1);

    for (size_t j = 0; j < n; j++) {
        const uint8_t *r0 = p + j * stride;
        const uint8_t *r1 = (phase >= 2) ? (r0 + stride) : r0;
        const float *ar = a + j * n;

        for (size_t col = 0; col < n;) {
            size_t vl = vsetvl_e32m4(n - col);
            vuint16m2_t s = mvs071_load_half_sum(r0, r1, col, phase, vl);
            vsum = vredsum_vs_u32m4_u32m1(vsum, vwcvtu_x_x_v_u32m4(s, vl), vsum, vl);
            vsumsq = vredsum_vs_u32m4_u32m1(vsumsq, vwmulu_vv_u32m4(s, s, vl), vsumsq, vl);

            vfloat32m4_t sample = mvs071_scale_half_sample(s, phase, vl);
            vfloat32m4_t prod = vfmul_vv_f32m4(vle32_v_f32m4(ar + col, vl), sample, vl);
            dacc = vfredosum_vs_f32m4_f32m1(dacc, prod, dacc, vl);
            col += vl;
        }
    }

    *sum_raw = vmv_x_s_u32m1_u32(vsum);
    *sumsq_raw = vmv_x_s_u32m1_u32(vsumsq);
    *dot = vfmv_f_s_f32m1_f32(dacc);
}

void mvs071_half_moments(const uint8_t *p, size_t stride,
                         const float *a, size_t n, uint32_t phase,
                         uint32_t *sum_raw, uint32_t *sumsq_raw,
                         float *dot) {
    if (n == 0 || phase > 3) {
        *sum_raw = 0;
        *sumsq_raw = 0;
        *dot = 0.0f;
        return;
    }
    if (n == 9) {
        size_t vl9 = vsetvl_e32m4(9);
        if (vl9 == 9) {
            switch (phase) {
                case 0:
                    mvs071_half_moments_n9_phase(p, stride, a, 0, vl9, sum_raw, sumsq_raw, dot);
                    return;
                case 1:
                    mvs071_half_moments_n9_phase(p, stride, a, 1, vl9, sum_raw, sumsq_raw, dot);
                    return;
                case 2:
                    mvs071_half_moments_n9_phase(p, stride, a, 2, vl9, sum_raw, sumsq_raw, dot);
                    return;
                default:
                    mvs071_half_moments_n9_phase(p, stride, a, 3, vl9, sum_raw, sumsq_raw, dot);
                    return;
            }
        }
    }
    switch (phase) {
        case 0:
            mvs071_half_moments_general_phase(p, stride, a, n, 0, sum_raw, sumsq_raw, dot);
            break;
        case 1:
            mvs071_half_moments_general_phase(p, stride, a, n, 1, sum_raw, sumsq_raw, dot);
            break;
        case 2:
            mvs071_half_moments_general_phase(p, stride, a, n, 2, sum_raw, sumsq_raw, dot);
            break;
        default:
            mvs071_half_moments_general_phase(p, stride, a, n, 3, sum_raw, sumsq_raw, dot);
            break;
    }
}
