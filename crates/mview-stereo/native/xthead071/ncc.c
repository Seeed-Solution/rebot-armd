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
