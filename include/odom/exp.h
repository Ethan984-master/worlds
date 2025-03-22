#pragma once

static inline float32x4_t fast_exp_neon(float32x4_t x)
{
    // Constants:
    // a = (1 << 23) / log(2)
    float32x4_t a = vdupq_n_f32(12102203.0f);
    // m = 0xff800000 (mask for integer bits)
    int32x4_t m = vdupq_n_s32(0xff800000);
    // ttm23 = 2^(-23)
    float32x4_t ttm23 = vdupq_n_f32(1.1920929e-7f);
    // Polynomial coefficients
    float32x4_t c0 = vdupq_n_f32(0.3371894346f);
    float32x4_t c1 = vdupq_n_f32(0.657636276f);
    float32x4_t c2 = vdupq_n_f32(1.00172476f);

    // Compute t = (int)(a * x)
    float32x4_t ax = vmulq_f32(a, x);
    int32x4_t t = vcvtq_s32_f32(ax);
    
    // j = t & m  (extract the integer part shifted left by 23 bits)
    int32x4_t j = vandq_s32(t, m);
    
    // t = t - j gives the fractional component in integer representation
    t = vsubq_s32(t, j);
    
    // Convert fractional part to float and scale: f = ttm23 * (float)t
    float32x4_t f = vmulq_f32(ttm23, vcvtq_f32_s32(t));
    
    // Evaluate polynomial: p = (c0 * f + c1) * f + c2.
    // Using multiply-accumulate intrinsics:
    float32x4_t p = vmlaq_f32(c1, c0, f);  // p = c0*f + c1
    p = vmlaq_f32(c2, p, f);                // p = p*f + c2
    
    // Combine integer and polynomial parts:
    // First, reinterpret p as integer bits
    int32x4_t pi = vreinterpretq_s32_f32(p);
    // Then add j: r_int = j + pi
    int32x4_t r_int = vaddq_s32(j, pi);
    // Reinterpret back to float
    float32x4_t r = vreinterpretq_f32_s32(r_int);
    
    return r;
}
