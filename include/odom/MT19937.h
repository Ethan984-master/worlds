#pragma once
#include <unistd.h>
#include <time.h>
#ifndef _XOPEN_SOURCE
#define _XOPEN_SOURCE 600
#endif

/** 
 * SFMT-based PRNG using Mersenne prime 19937.
 * 
 * This version seeds only with getpid() and time(), avoiding getppid.
 * It supports SIMD implementations via ARM NEON or SSE2.
 *
 * Functions provided:
 *   - mt_init(): Initializes the generator.
 *   - wide_uniform(): Returns 128-bit (4 float) uniform PRNs.
 *   - uniform_float_PRN(): Returns one uniform float in [0,1).
 *   - rand_long(n): Returns a uniform unsigned long in [0, n).
 *   - rand_long64(): Returns a uniform unsigned long (64-bit).
 *
 * Define REPORT_PRNS to report total PRNs used at exit.
 */

#define __cycle__ 500

#ifndef SFMT_H
#define SFMT_H

#define MEXP 19937

#include <stdio.h>
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
  #include <inttypes.h>
#elif defined(_MSC_VER) || defined(__BORLANDC__)
  typedef unsigned int uint32_t;
  typedef unsigned __int64 uint64_t;
  #define inline __inline
#else
  #include <inttypes.h>
  #if defined(__GNUC__)
    #define inline __inline__
  #endif
#endif

#ifndef PRIu64
  #if defined(_MSC_VER) || defined(__BORLANDC__)
    #define PRIu64 "I64u"
    #define PRIx64 "I64x"
  #else
    #define PRIu64 "llu"
    #define PRIx64 "llx"
  #endif
#endif

#if defined(__GNUC__)
#define ALWAYSINLINE __attribute__((always_inline))
#else
#define ALWAYSINLINE
#endif

#if defined(_MSC_VER)
  #if _MSC_VER >= 1200
    #define PRE_ALWAYS __forceinline
  #else
    #define PRE_ALWAYS inline
  #endif
#else
  #define PRE_ALWAYS inline
#endif

/*--------------------
  Conditional SIMD definitions
  --------------------*/
#if defined(__ARM_NEON)
  #include <arm_neon.h>
  union W128_T {
      int32x4_t si;     // 128-bit integer vector
      uint32_t u[4];
      float32x4_t f32;  // for float operations
      int64x2_t i64;    // for 64-bit operations if needed
  };
  typedef union W128_T w128_t;
  
  union dW128_T {
      int32x4_t si;
      float32x4_t f32;
      uint32_t i[4];
      float f[4];
  };
  typedef union dW128_T dw128_t;
  
  #define EXP_SET_F 0x3f800000
  
  static float32x4_t neon_float_m_one;  // set to -1.0f in each lane
  static uint32x4_t neon_int_set;         // set to EXP_SET_F in each lane

#elif defined(__SSE2__)
  #include <emmintrin.h>
  union W128_T {
      __m128i si;
      uint32_t u[4];
      float f[4];
      double d[2];
  };
  typedef union W128_T w128_t;
  
  union dW128_T {
      __m128i si;
      __m128d sd;
      uint32_t i[4];
      float f[4];
      double d[2];
  };
  typedef union dW128_T dw128_t;
  
  #define EXP_SET_D 0x3ff0000000000000ULL
  
  static __m128d sse2_double_m_one;
  static __m128i sse2_int_set;
#else
  /* Fallback (non-SIMD): Provide scalar implementations if needed */
#endif

/*----------------------
  SFMT parameters
 ----------------------*/
#define iN (MEXP / 128 + 1)
#define N32 (iN * 4)
#define N64 (iN * 2)

/* SFMT parameter definitions */
#ifndef SFMT_PARAMS19937_H
#define POS1    122
#define SL1     18
#define SL2     1
#define SR1     11
#define SR2     1
#define MSK1    0xdfffffefU
#define MSK2    0xddfecb7fU
#define MSK3    0xbffaffffU
#define MSK4    0xbffffff6U
#define PARITY1 0x00000001U
#define PARITY2 0x00000000U
#define PARITY3 0x00000000U
#define PARITY4 0x13c9e684U

#ifdef __APPLE__
    #define ALTI_SL1   (vector unsigned int)(SL1, SL1, SL1, SL1)
    #define ALTI_SR1   (vector unsigned int)(SR1, SR1, SR1, SR1)
    #define ALTI_MSK   (vector unsigned int)(MSK1, MSK2, MSK3, MSK4)
    #define ALTI_MSK64 (vector unsigned int)(MSK2, MSK1, MSK4, MSK3)
    #define ALTI_SL2_PERM (vector unsigned char)(1,2,3,23,5,6,7,0,9,10,11,4,13,14,15,8)
    #define ALTI_SL2_PERM64 (vector unsigned char)(1,2,3,4,5,6,7,31,9,10,11,12,13,14,15,0)
    #define ALTI_SR2_PERM (vector unsigned char)(7,0,1,2,11,4,5,6,15,8,9,10,17,12,13,14)
    #define ALTI_SR2_PERM64 (vector unsigned char)(15,0,1,2,3,4,5,6,17,8,9,10,11,12,13,14)
#else
    #define ALTI_SL1   {SL1, SL1, SL1, SL1}
    #define ALTI_SR1   {SR1, SR1, SR1, SL1}
    #define ALTI_MSK   {MSK1, MSK2, MSK3, MSK4}
    #define ALTI_MSK64 {MSK2, MSK1, MSK4, MSK3}
    #define ALTI_SL2_PERM {1,2,3,23,5,6,7,0,9,10,11,4,13,14,15,8}
    #define ALTI_SL2_PERM64 {1,2,3,4,5,6,7,31,9,10,11,12,13,14,15,0}
    #define ALTI_SR2_PERM {7,0,1,2,11,4,5,6,15,8,9,10,17,12,13,14}
    #define ALTI_SR2_PERM64 {15,0,1,2,3,4,5,6,17,8,9,10,11,12,13,14}
#endif
#define IDSTR "SFMT-19937:122-18-1-11-1:dfffffef-ddfecb7f-bffaffff-bffffff6"
#endif /* SFMT_PARAMS19937_H */
#endif /* SFMT_H */

/* 
 * SIMD recursion function for ARM NEON.
 */
#if defined(__ARM_NEON)
PRE_ALWAYS static int32x4_t mm_recursion_neon(int32x4_t *a, int32x4_t *b, int32x4_t c,
                                              int32x4_t d, int32x4_t mask) ALWAYSINLINE {
    int32x4_t x = *a;
    int32x4_t y = vshrq_n_s32(*b, SR1);
    uint8x16_t c_vec = vreinterpretq_u8_s32(c);
    uint8x16_t z_vec = vextq_u8(c_vec, c_vec, SR2);
    int32x4_t z = vreinterpretq_s32_u8(z_vec);
    int32x4_t v = vshlq_n_s32(d, SL1);
    z = veorq_s32(z, x);
    z = veorq_s32(z, v);
    uint8x16_t x_vec = vreinterpretq_u8_s32(x);
    uint8x16_t zero = vdupq_n_u8(0);
    uint8x16_t x_shifted_vec = vextq_u8(zero, x_vec, 16 - SL2);
    int32x4_t x_shifted = vreinterpretq_s32_u8(x_shifted_vec);
    y = vandq_s32(y, mask);
    z = veorq_s32(z, x_shifted);
    z = veorq_s32(z, y);
    return z;
}
#endif

#include <string.h>
#include <assert.h>

#ifndef SFMT_PARAMS_H
#define SFMT_PARAMS_H

/* Container for random digits */
typedef union RAND64_t {
    uint8_t  s[8];
    uint32_t i[2];
    double d;
    float f[2];
    uint64_t l;
    signed long sl;
} rand64_t;

#if defined(__ARM_NEON) || defined(__SSE2__)
static w128_t sfmt[iN];
static uint32_t *psfmt32 = &sfmt[0].u[0];
static int idx;
static uint32_t parity[4] = {PARITY1, PARITY2, PARITY3, PARITY4};
#endif

/* Function prototypes */
static void gen_rand_array(w128_t *array, int size);
inline static uint32_t func1(uint32_t x);
inline static uint32_t func2(uint32_t x);
static void period_certification(void);
static void mt_init(void);
#if defined(__ARM_NEON)
static inline dw128_t wide_uniform(void);
static inline float uniform_float_PRN(void);
#elif defined(__SSE2__)
static inline dw128_t wide_uniform(void);
static inline float uniform_float_PRN(void);
#endif
static inline unsigned long rand_long(unsigned long n);
static inline unsigned long rand_long64(void);
void _report_PRN_total(void);

#if defined(__ARM_NEON)
static void gen_rand_array(w128_t *array, int size) {
    int i, j;
    int32x4_t r, r1, r2, mask;
    uint32_t msk_arr[4] = {MSK1, MSK2, MSK3, MSK4};
    mask = vld1q_s32((const int*)msk_arr);
    r1 = sfmt[iN - 2].si;
    r2 = sfmt[iN - 1].si;
    for (i = 0; i < iN - POS1; i++) {
        r = mm_recursion_neon(&sfmt[i].si, &sfmt[i + POS1].si, r1, r2, mask);
        array[i].si = r;
        r1 = r2;
        r2 = r;
    }
    for (; i < iN; i++) {
        r = mm_recursion_neon(&sfmt[i].si, &array[i + POS1 - iN].si, r1, r2, mask);
        array[i].si = r;
        r1 = r2;
        r2 = r;
    }
    for (; i < size - iN; i++) {
        r = mm_recursion_neon(&array[i - iN].si, &array[i + POS1 - iN].si, r1, r2, mask);
        array[i].si = r;
        r1 = r2;
        r2 = r;
    }
    for (j = 0; j < 2 * iN - size; j++) {
        array[j].si = sfmt[j + size - iN].si;
    }
    for (; i < size; i++) {
        r = mm_recursion_neon(&array[i - iN].si, &array[i + POS1 - iN].si, r1, r2, mask);
        array[i].si = r;
        sfmt[j++].si = r;
        r1 = r2;
        r2 = r;
    }
}
#elif defined(__SSE2__)
static void gen_rand_array(w128_t *array, int size) {
    int i, j;
    __m128i r, r1, r2, mask;
    mask = _mm_set_epi32(MSK4, MSK3, MSK2, MSK1);
    r1 = _mm_load_si128(&sfmt[iN - 2].si);
    r2 = _mm_load_si128(&sfmt[iN - 1].si);
    for (i = 0; i < iN - POS1; i++) {
        r = mm_recursion(&sfmt[i].si, &sfmt[i + POS1].si, r1, r2, mask);
        _mm_store_si128(&array[i].si, r);
        r1 = r2;
        r2 = r;
    }
    for (; i < iN; i++) {
        r = mm_recursion(&sfmt[i].si, &array[i + POS1 - iN].si, r1, r2, mask);
        _mm_store_si128(&array[i].si, r);
        r1 = r2;
        r2 = r;
    }
    for (; i < size - iN; i++) {
        r = mm_recursion(&array[i - iN].si, &array[i + POS1 - iN].si, r1, r2, mask);
        _mm_store_si128(&array[i].si, r);
        r1 = r2;
        r2 = r;
    }
    for (j = 0; j < 2 * iN - size; j++) {
        r = _mm_load_si128(&array[j + size - iN].si);
        _mm_store_si128(&sfmt[j].si, r);
    }
    for (; i < size; i++) {
        r = mm_recursion(&array[i - iN].si, &array[i + POS1 - iN].si, r1, r2, mask);
        _mm_store_si128(&array[i].si, r);
        _mm_store_si128(&sfmt[j++].si, r);
        r1 = r2;
        r2 = r;
    }
}
#endif

static inline uint32_t func1(uint32_t x) {
    return (x ^ (x >> 27)) * 1664525UL;
}

static inline uint32_t func2(uint32_t x) {
    return (x ^ (x >> 27)) * 1566083941UL;
}

static void period_certification(void) {
    int inner = 0;
    int i, j;
    uint32_t work;
    for (i = 0; i < 4; i++)
        inner ^= psfmt32[i] & parity[i];
    for (i = 16; i > 0; i >>= 1)
        inner ^= inner >> i;
    inner &= 1;
    if (inner == 1) return;
    for (i = 0; i < 4; i++) {
        work = 1;
        for (j = 0; j < 32; j++) {
            if ((work & parity[i]) != 0) {
                psfmt32[i] ^= work;
                return;
            }
            work <<= 1;
        }
    }
}

/* Global internal state */
static w128_t iRandS[__cycle__], *iRend = &iRandS[__cycle__ - 1];
rand64_t *Rand;

#ifdef REPORT_PRNS
static long __n_cycles__ = 0;
static void _report_PRN_total(void) {
    printf("Used ~%ld 64-bit uniform PRNs.\n", 2 * __cycle__ * __n_cycles__ + Rand - (rand64_t *)iRandS);
}
#endif

// mt_init() – optimized initialization without getppid
static void mt_init(void) {
    static int old = 0;
    if (old) return;
    old = 1;
    int init_key[] = { (int)getpid(), (int)time(NULL) };
    int key_length = 2;
#ifdef REPORT_PRNS
    atexit(_report_PRN_total);
#endif
#if defined(__ARM_NEON)
    neon_float_m_one = vdupq_n_f32(-1.0f);
    neon_int_set = vdupq_n_u32(EXP_SET_F);
#elif defined(__SSE2__)
    sse2_double_m_one = _mm_set_pd(-1.0, -1.0);
    sse2_int_set = _mm_set_epi64x((long long)EXP_SET_D, (long long)EXP_SET_D);
#endif
    int i, j, count;
    uint32_t r;
    int lag, mid;
    int size = iN * 4;
    if (size >= 623)
        lag = 11;
    else if (size >= 68)
        lag = 7;
    else if (size >= 39)
        lag = 5;
    else
        lag = 3;
    mid = (size - lag) / 2;
    memset(sfmt, 0x8b, sizeof(sfmt));
    count = (key_length + 1 > N32) ? key_length + 1 : N32;
    r = func1(psfmt32[0] ^ psfmt32[mid] ^ psfmt32[N32 - 1]);
    psfmt32[mid] += r;
    r += key_length;
    psfmt32[mid + lag] += r;
    psfmt32[0] = r;
    count--;
    for (i = 1, j = 0; (j < count) && (j < key_length); j++) {
        r = func1(psfmt32[i] ^ psfmt32[(i + mid) % N32] ^ psfmt32[(i + N32 - 1) % N32]);
        psfmt32[(i + mid) % N32] += r;
        r += init_key[j] + i;
        psfmt32[(i + mid + lag) % N32] += r;
        psfmt32[i] = r;
        i = (i + 1) % N32;
    }
    for (; j < count; j++) {
        r = func1(psfmt32[i] ^ psfmt32[(i + mid) % N32] ^ psfmt32[(i + N32 - 1) % N32]);
        psfmt32[(i + mid) % N32] += r;
        r += i;
        psfmt32[(i + mid + lag) % N32] += r;
        psfmt32[i] = r;
        i = (i + 1) % N32;
    }
    for (j = 0; j < N32; j++) {
        r = func2(psfmt32[i] + psfmt32[(i + mid) % N32] + psfmt32[(i + N32 - 1) % N32]);
        psfmt32[(i + mid) % N32] ^= r;
        r -= i;
        psfmt32[(i + mid + lag) % N32] ^= r;
        psfmt32[i] = r;
        i = (i + 1) % N32;
    }
    idx = N32;
    period_certification();
    gen_rand_array(iRandS, __cycle__);
    Rand = (rand64_t *)iRandS;
}

#ifdef REPORT_PRNS
#define INCREMENT_N_CYCLES() (__n_cycles__++)
#else
#define INCREMENT_N_CYCLES() ;
#endif

#define MT_FLUSH() { if (Rand > (rand64_t *)iRend) { \
    gen_rand_array(iRandS, __cycle__); \
    Rand = (rand64_t *) iRandS; \
    INCREMENT_N_CYCLES() \
}; }

#if defined(__ARM_NEON)
static inline dw128_t wide_uniform(void) {
    MT_FLUSH();
    dw128_t W;
    uint32_t rand_vals[4] = { Rand[0].i[0], Rand[0].i[1], Rand[1].i[0], Rand[1].i[1] };
    for (int i = 0; i < 4; i++) {
        rand_vals[i] = (rand_vals[i] >> 9) | EXP_SET_F;
    }
    uint32x4_t result_vec = vld1q_u32((const unsigned int*)rand_vals);
    float32x4_t fvec = vreinterpretq_f32_u32(result_vec);
    float32x4_t one = vdupq_n_f32(1.0f);
    fvec = vsubq_f32(fvec, one);
    W.f32 = fvec;
    Rand += 2;
    return W;
}
#elif defined(__SSE2__)
static inline dw128_t wide_uniform(void) {
    MT_FLUSH();
    dw128_t W;
    __m128i temp = _mm_set_epi64x(Rand[0].l, Rand[1].l);
    uint32_t vals[4];
    _mm_storeu_si128((__m128i*)vals, temp);
    for (int i = 0; i < 4; i++) {
        vals[i] = (vals[i] >> 9) | 0x3f800000;
    }
    __m128 result = _mm_castsi128_ps(_mm_loadu_si128((__m128i*)vals));
    result = _mm_sub_ps(result, _mm_set1_ps(1.0f));
    ((float*)W.f)[0] = ((float*)&result)[0];
    ((float*)W.f)[1] = ((float*)&result)[1];
    ((float*)W.f)[2] = ((float*)&result)[2];
    ((float*)W.f)[3] = ((float*)&result)[3];
    Rand += 2;
    return W;
}
#endif

#if defined(__ARM_NEON)
static inline float uniform_float_PRN(void) {
    MT_FLUSH();
    uint32_t r = (Rand->i[0] >> 9) | EXP_SET_F;
    float f;
    memcpy(&f, &r, sizeof(f));
    Rand++;
    return f - 1.0f;
}
#elif defined(__SSE2__)
static inline float uniform_float_PRN(void) {
    MT_FLUSH();
    uint32_t r = (Rand->i[0] >> 9) | 0x3f800000;
    float f;
    memcpy(&f, &r, sizeof(f));
    Rand++;
    return f - 1.0f;
}
#endif

static inline unsigned long rand_long(unsigned long n) {
    MT_FLUSH();
    return Rand++->l % n;
}

static inline unsigned long rand_long64(void) {
    MT_FLUSH();
    return Rand++->l;
}

#endif  // End of SFMT_PARAMS_H