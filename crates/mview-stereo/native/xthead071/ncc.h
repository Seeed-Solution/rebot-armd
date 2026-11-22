#ifndef MVIEW_STEREO_XTHEAD071_NCC_H
#define MVIEW_STEREO_XTHEAD071_NCC_H

#include <stddef.h>
#include <stdint.h>

/* One dot per adjacent candidate center; out has count elements. */
void mvs071_coarse_dots(const uint8_t *p, size_t stride,
                        const float *a, size_t n, size_t count,
                        float *out);

/* phase: 0 integer, 1 horizontal half, 2 vertical half, 3 both. */
void mvs071_half_moments(const uint8_t *p, size_t stride,
                         const float *a, size_t n, uint32_t phase,
                         uint32_t *sum_raw, uint32_t *sumsq_raw,
                         float *dot);

/* weights == NULL means uniform; bilerp == NULL means nearest.
   Otherwise bilerp contains [w00, w10, w01, w11]. */
void mvs071_patch_moments(const uint8_t *p, size_t stride,
                          const float *a, const float *weights,
                          size_t n, const float *bilerp,
                          double *sum, double *sumsq, float *dot);

#endif /* MVIEW_STEREO_XTHEAD071_NCC_H */
