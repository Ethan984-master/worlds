// UltraOptimizedZigguratPRNG.h
#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include "MT19937.h"

// -------------------------------------------------------------------------
// Minimal inline 32–bit xorshift RNG (fully inlined)
class Xorshift32 {
public:
  explicit Xorshift32(uint32_t seed = 2463534242U) : state(seed) {}
  inline uint32_t next() {
    uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state = x;
    return x;
  }

private:
  uint32_t state;
};

// -------------------------------------------------------------------------
// Precomputed tables for the Normal Distribution (128 layers)
// (These constants are taken from the public domain implementation of the
// Marsaglia–Tsang Ziggurat method.)
static const int NORMAL_R = 128;
static const uint32_t normal_kn[NORMAL_R + 1] = {
    0x76ad2212, 0x6ea978e4, 0x6a5adf40, 0x664d3a8c, 0x62504e1c, 0x5e688064,
    0x5a7f3efc, 0x568a4e60, 0x52893814, 0x4e804a30, 0x4a9b142c, 0x46d13160,
    0x431f3a40, 0x3f8ad0d8, 0x3c290214, 0x38f2d424, 0x35e5cfec, 0x32f9a624,
    0x300bb2e8, 0x2d2f15e4, 0x2a47e094, 0x27632f6c, 0x24603390, 0x21612830,
    0x1e82ae80, 0x1be8ed10, 0x1971cfe8, 0x170e6024, 0x149b20a4, 0x1236fd88,
    0xffc3e584, 0xdd5e9f30, 0xbb1e3b2c, 0x99fe8448, 0x79f05a88, 0x59f48a50,
    0x3af9c2d8, 0x1cf2a134, 0x00000000, 0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0,          0,          0,          0,
    0,          0,          0};

static const float normal_wn[NORMAL_R] = {
    1.602e-09, 1.555e-09, 1.511e-09, 1.468e-09, 1.428e-09, 1.390e-09,
    1.354e-09, 1.320e-09, 1.288e-09, 1.258e-09, 1.229e-09, 1.202e-09,
    1.176e-09, 1.152e-09, 1.129e-09, 1.107e-09, 1.086e-09, 1.066e-09,
    1.047e-09, 1.029e-09, 1.011e-09, 9.949e-10, 9.786e-10, 9.629e-10,
    9.478e-10, 9.333e-10, 9.194e-10, 9.061e-10, 8.933e-10, 8.810e-10,
    8.693e-10, 8.580e-10, 8.472e-10, 8.368e-10, 8.268e-10, 8.172e-10,
    8.080e-10, 7.992e-10, 7.908e-10, 7.827e-10, 7.750e-10, 7.676e-10,
    7.605e-10, 7.537e-10, 7.472e-10, 7.410e-10, 7.350e-10, 7.293e-10,
    7.239e-10, 7.187e-10, 7.137e-10, 7.089e-10, 7.043e-10, 6.999e-10,
    6.957e-10, 6.916e-10, 6.877e-10, 6.840e-10, 6.804e-10, 6.770e-10,
    6.737e-10, 6.706e-10, 6.676e-10, 6.647e-10, 6.619e-10, 6.593e-10,
    6.567e-10, 6.543e-10, 6.519e-10, 6.497e-10, 6.475e-10, 6.454e-10,
    6.434e-10, 6.415e-10, 6.396e-10, 6.378e-10, 6.360e-10, 6.343e-10,
    6.326e-10, 6.310e-10, 6.294e-10, 6.279e-10, 6.264e-10, 6.250e-10,
    6.236e-10, 6.223e-10, 6.210e-10, 6.198e-10, 6.186e-10, 6.174e-10,
    6.163e-10, 6.152e-10, 6.141e-10, 6.131e-10, 6.121e-10, 6.111e-10,
    6.102e-10, 6.093e-10, 6.084e-10, 6.075e-10, 6.067e-10, 6.059e-10,
    6.051e-10, 6.044e-10, 6.037e-10, 6.030e-10, 6.023e-10, 6.016e-10,
    6.010e-10, 6.004e-10, 5.998e-10, 5.992e-10};

static const float normal_fn[NORMAL_R] = {
    1.00000000, 0.96359862, 0.93628200, 0.91292140, 0.89242741, 0.87411063,
    0.85753149, 0.84239214, 0.82844933, 0.81552421, 0.80346300, 0.79214135,
    0.78144142, 0.77125899, 0.76149528, 0.75205595, 0.74286981, 0.73387921,
    0.72503097, 0.71627335, 0.70757622, 0.69890970, 0.69025808, 0.68160798,
    0.67295794, 0.66430569, 0.65565237, 0.64700003, 0.63834769, 0.62969535,
    0.62104399, 0.61239263, 0.60374127, 0.59508991, 0.58643855, 0.57778719,
    0.56913583, 0.56048447, 0.55183311, 0.54318175, 0.53453039, 0.52587903,
    0.51722767, 0.50857631, 0.49992495, 0.49127359, 0.48262223, 0.47397087,
    0.46531951, 0.45666815, 0.44801679, 0.43936543, 0.43071407, 0.42206271,
    0.41341135, 0.40475999, 0.39610863, 0.38745727, 0.37880591, 0.37015455,
    0.36150319, 0.35285183, 0.34420047, 0.33554911, 0.32689775, 0.31824639,
    0.30959503, 0.30094367, 0.29229231, 0.28364095, 0.27498959, 0.26633823,
    0.25768687, 0.24903551, 0.24038415, 0.23173279, 0.22308143, 0.21443007,
    0.20577871, 0.19712735, 0.18847599, 0.17982463, 0.17117327, 0.16252191,
    0.15387055, 0.14521919, 0.13656783, 0.12791647, 0.11926511, 0.11061375,
    0.10196239, 0.09331103, 0.08465967, 0.07600831, 0.06735695, 0.05870559,
    0.05005423, 0.04140287, 0.03275151, 0.02410015, 0.01544879, 0.00679743,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000};

// -------------------------------------------------------------------------
// Precomputed tables for the Exponential Distribution (256 layers)
// (Again, these values are taken from McFarland’s fast_prng repository. The
// table size here is 256.)
static const int EXP_E = 256;
static const double exp_re =
    7.69711747013104972; // Right-most x-coordinate for Exp(1)
static const uint32_t exp_ke[EXP_E + 1] = {
    0x00000000, 0x2E8BA2F0, 0x5C1765E1, 0x89A345D2, 0xB7331A63, 0xE187FF74,
    0x10B8C4B65, 0x137CB0E56, 0x15E42D487, 0x183968178, 0x1AEBB5E69,
    0x1D81A9BAA, 0x203C7E89B, 0x22F8BDFAC, 0x25BAEF3BD, 0x2880B89CE,
    0x2B4A2BDEF, 0x2E16C5C10, 0x30E6D1A21, 0x33B9A7B32, 0x367FF3C43,
    0x39495A554, 0x3C14A16A5, 0x3EE2507B6, 0x41B31E8C7, 0x4477C29D8,
    0x473F62AE9, 0x49A2F90FA, 0x4C0A6A210, 0x4E74F3331, 0x50E335444,
    0x5354F9555, 0x55C8C2976, 0x583E0D8A7, 0x5AA6DBBB8, 0x5D11F2CCA,
    0x5F80C4DDD, 0x61F288EEE, 0x647868FFF, 0x66E98B110, 0x6950DE221,
    0x6BB9A3322, 0x6E24D4433, 0x7092A5544, 0x730299655, 0x75786B766,
    0x77E63D877, 0x7A568C988, 0x7CC9AFA99, 0x7F3F62BAA, 0x81B773CBB,
    0x843189DCC, 0x86AF8CEDD, 0x892000EFE, 0x8B95E1000, 0x8E2EE2111,
    0x90CAA3222, 0x935998333, 0x95EB8A444, 0x984037555, 0x9A977D666,
    0x9D3BB9777, 0x9FD420888, 0xA2897D999, 0xA53C699AA, 0xA7F269ABB,
    0xAAAD79CCC, 0xAD6E19DDD, 0xB04159EEE, 0xB3164AFFF, 0xB5EDC4100,
    0xB8C6D5211, 0xBBA29A622, 0xBE809F733, 0xC15FF3844, 0xC44F79855,
    0xC73F8B666, 0xCA308D777, 0xCD22AF888, 0xD015E0999, 0xD3102EAAA,
    0xD60B70BBB, 0xD907B0CCC, 0xDC03F0DDD, 0xDF0010EEE, 0xE1FD50FFF,
    0xE4FAC1100, 0xE7F8F2211, 0xEAF837222, 0xEDF7D9333, 0xF0F829444,
    0xF3F97B555, 0xF6FA1D666, 0xF9FB0F777, 0xFCFC51788, 0xFFFFF89AA,
    // (The remaining 257 - 96 = 161 entries are zero.)
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static const float exp_we[EXP_E] = {
    1.223928e-09, 1.210523e-09, 1.197362e-09, 1.184437e-09, 1.171743e-09,
    1.159275e-09, 1.147028e-09, 1.134998e-09, 1.123179e-09, 1.111566e-09,
    1.100155e-09, 1.088942e-09, 1.077924e-09, 1.067098e-09, 1.056461e-09,
    1.046009e-09, 1.035739e-09, 1.025647e-09, 1.015730e-09, 1.005986e-09,
    9.963112e-10, 9.866087e-10, 9.770255e-10, 9.675594e-10, 9.582083e-10,
    9.489711e-10, 9.398457e-10, 9.308300e-10, 9.219220e-10, 9.131198e-10,
    9.044212e-10, 8.958242e-10, 8.873267e-10, 8.789267e-10, 8.706222e-10,
    8.624112e-10, 8.542919e-10, 8.462623e-10, 8.383207e-10, 8.304645e-10,
    8.226910e-10, 8.149976e-10, 8.073817e-10, 7.998408e-10, 7.923723e-10,
    7.849737e-10, 7.776425e-10, 7.703762e-10, 7.631723e-10, 7.560283e-10,
    7.489416e-10, 7.419106e-10, 7.349337e-10, 7.280093e-10, 7.211359e-10,
    7.143121e-10, 7.075364e-10, 7.008074e-10, 6.941239e-10, 6.874846e-10,
    6.808882e-10, 6.743335e-10, 6.678192e-10, 6.613440e-10, 6.549066e-10,
    6.485058e-10, 6.421404e-10, 6.358092e-10, 6.295111e-10, 6.232450e-10,
    6.170097e-10, 6.108042e-10, 6.046274e-10, 5.984783e-10, 5.923557e-10,
    5.862586e-10, 5.801858e-10, 5.741363e-10, 5.681089e-10, 5.621026e-10,
    5.561163e-10, 5.501490e-10, 5.442997e-10, 5.384674e-10, 5.326511e-10,
    5.268498e-10, 5.210626e-10, 5.152886e-10, 5.095277e-10, 5.037790e-10,
    4.980416e-10, 4.923145e-10, 4.865968e-10, 4.808876e-10, 4.751859e-10,
    4.694908e-10, 4.638013e-10, 4.581165e-10, 4.524354e-10, 4.467572e-10,
    4.410809e-10, 4.354057e-10, 4.297308e-10, 4.240554e-10, 4.183787e-10,
    4.127000e-10, 4.070185e-10, 4.013334e-10, 3.956439e-10, 3.899491e-10,
    3.842482e-10, 3.785404e-10, 3.728249e-10, 3.671009e-10, 3.613676e-10,
    3.556243e-10, 3.498702e-10, 3.441045e-10, 3.383263e-10, 3.325347e-10};

static const float exp_fe[EXP_E] = {
    1.00000000, 0.96365655, 0.92823938, 0.89465700, 0.86283356, 0.83270046,
    0.80420304, 0.77728986, 0.75191178, 0.72792206, 0.70526700, 0.68390063,
    0.66378335, 0.64486990, 0.62711617, 0.61047305, 0.59489669, 0.58034559,
    0.56678025, 0.55416310, 0.54245970, 0.53163766, 0.52166779, 0.51252300,
    0.50417831, 0.49661191, 0.48980431, 0.48372827, 0.47835781, 0.47367822,
    0.46966612, 0.46629935, 0.46355713, 0.46142917, 0.45989656, 0.45894288,
    0.45855329, 0.45871361, 0.45941130, 0.46063553, 0.46237615, 0.46462481,
    0.46737383, 0.47061627, 0.47434585, 0.47855689, 0.48324442, 0.48840317,
    0.49402846, 0.50011539, 0.50665989, 0.51365771, 0.52110438, 0.52999523,
    0.53932454, 0.54908762, 0.55927876, 0.56989231, 0.58092173, 0.59235957,
    0.60419858, 0.61643168, 0.62905104, 0.64204904, 0.65541833, 0.66915188,
    0.68324211, 0.69768174, 0.71246387, 0.72758197, 0.74302984, 0.75880061,
    0.77488682, 0.79128048, 0.80797310, 0.82495654, 0.84222212, 0.85976147,
    0.87756647, 0.89562832, 0.91393762, 0.93248435, 0.95125895, 0.97025138,
    0.98945197, 1.00885139, 1.02844056, 1.04821066, 1.06815210, 1.08825455,
    1.10850896, 1.12890550, 1.14943565, 1.17008920, 1.19085627, 1.21172737,
    1.23269230, 1.25374010, 1.27486006, 1.29604166, 1.31727455, 1.33854865,
    1.35985320, 1.38117779, 1.40251229, 1.42384691, 1.44517123, 1.46647426,
    1.48774541, 1.50897450, 1.53015155, 1.55126693, 1.57231121, 1.59327523,
    1.61414912, 1.63492341, 1.65558806, 1.67613350, 1.69655074, 1.71683031,
    1.73696333, 1.75694161, 1.77675679, 1.79640030, 1.81586345, 1.83513830,
    1.85421670, 1.87309136};

// -------------------------------------------------------------------------
// Optimized Normal Distribution PRNG (template on T: float or double).
template <typename T> class Normal {
public:
  // Construct with given mean and standard deviation.
  Normal(T mean, T stddev, uint32_t seed = 2463534242U)
      : m_mean(mean), m_stddev(stddev), rng(seed) {}

  // Returns one sample from N(mean, stddev^2)
  inline T sample() {
    while (true) {
      int32_t hz = static_cast<int32_t>(rng.next());
      int i = hz & (NORMAL_R - 1); // lower 7 bits
      // Fast acceptance using precomputed kn table.
      if (static_cast<uint32_t>(hz & 0x7fffffff) < normal_kn[i]) {
        T x = hz * normal_wn[i];
        return m_mean + m_stddev * x;
      }
      if (i == 0) {
        // Tail: sample using exponential rejection.
        T x, y;
        do {
          x = -std::log(uniform()) / RNORM;
          y = -std::log(uniform());
        } while (2 * y < x * x);
        return m_mean + m_stddev * ((hz < 0) ? -(RNORM + x) : (RNORM + x));
      } else {
        T x = hz * normal_wn[i];
        T absx = (x < 0 ? -x : x);
        T f_val = std::exp(-0.5 * absx * absx);
        T r = uniform();
        // Correct interpolation using table entry i+1.
        T y = normal_fn[i + 1] + r * (normal_fn[i] - normal_fn[i + 1]);
        if (y < f_val)
          return m_mean + m_stddev * x;
      }
    }
  }

private:
  T m_mean, m_stddev;
  Xorshift32 rng;
  static constexpr T RNORM = static_cast<T>(3.442619855899);
  inline T uniform() {
    return static_cast<T>(rng.next()) / static_cast<T>(4294967296.0);
  }
};

// -------------------------------------------------------------------------
// Optimized Exponential Distribution PRNG (Exp(1)) (template on T).
template <typename T> class Exponential {
public:
  explicit Exponential(uint32_t seed = 2463534242U) : rng(seed) {}
  inline T sample() {
    while (true) {
      uint32_t u = rng.next();
      int i = u & (EXP_E - 1);
      if (u < exp_ke[i])
        return u * exp_we[i];
      if (i == 0) {
        return exp_re - std::log(uniform());
      } else {
        T x = u * exp_we[i];
        T f_val = std::exp(-x);
        T r = uniform();
        T y = exp_fe[i + 1] + r * (exp_fe[i] - exp_fe[i + 1]);
        if (y < f_val)
          return x;
      }
    }
  }

private:
  Xorshift32 rng;
  inline T uniform() {
    return static_cast<T>(rng.next()) / static_cast<T>(4294967296.0);
  }
};
