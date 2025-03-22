#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "ZigguratPRNG.h"
#include "exp.h"
#include "normal.h"
#include <algorithm>
#include <arm_neon.h>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

constexpr float MAP_HALF = 24.0f;
constexpr float MAP_MIN = -MAP_HALF;
constexpr float MAP_MAX = MAP_HALF;
constexpr float PI = 3.14159265358979323846f;
constexpr float PI_2 = PI / 2.0f;
constexpr float PI2 = 2 * PI;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float INV_SQRT_2PI = 0.3989422804014327f;
constexpr float MAX_DIST_RECIPROCAL = 1.0f / 78.74016f;
float SQRT3 = 1 / sqrtf(3.0f);
float PI_SQRT3 = -PI / sqrtf(3.0f);

template <typename T> T clamp(T val, T mn, T mx) {
  return std::max(std::min(val, mx), mn);
}

#define cast_uint32_t static_cast<uint32_t>
static inline float fastpow2(float p) {
  float offset = (p < 0) ? 1.0f : 0.0f;
  float clipp = (p < -126) ? -126.0f : p;
  int w = clipp;
  float z = clipp - w + offset;
  union {
    uint32_t i;
    float f;
  } v = {cast_uint32_t((1 << 23) *
                       (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) -
                        1.49012907f * z))};

  return v.f;
}

static inline float fastexp(float p) { return fastpow2(1.442695040f * p); }

static inline float fasterpow2(float p) {
  float clipp = (p < -126) ? -126.0f : p;
  union {
    uint32_t i;
    float f;
  } v = {cast_uint32_t((1 << 23) * (clipp + 126.94269504f))};
  return v.f;
}

static inline float fasterexp(float p) { return fasterpow2(1.442695040f * p); }

// In this compass heading system:
//   0 radians (0°) is North (positive Y axis)
//   π/2 radians (90°) is East (positive X axis)
//   π radians (180°) is South (negative Y axis)
// Angles increase clockwise.

struct Particle {
  float x, y, weight;
};

struct SensorConfig {
  float dx, dy, angle; // dx: forward offset, dy: right offset, angle: mounting
                       // offset in degrees
};

class ParticleFilter {
  std::vector<Particle> particles;
  std::mt19937 gen{std::random_device{}()};
  std::ranlux24_base de;
  float cum_distance = 0.0f;
  float pshort = 0.1f;

  // Additional parameters to ensure unit consistency and reduce drift:
  const float motion_scale; // scales the odometry displacement to map units
  const float sensor_scale; // converts sensor readings to map units
  const float resample_distance_threshold; // threshold (in map units) to
                                           // trigger resampling

  // Sensor cache (local robot frame)
  struct Sensor {
    float dx, dy; // sensor offset: dx forward, dy right
    float angle;  // sensor mounting offset (in radians, compass convention)
    float cos_a, sin_a;
  };
  std::vector<Sensor> sensors;

public:
  // The constructor now accepts optional parameters for scaling and noise
  // tuning.
  Normal<float> pos_noise;
  float cos_h, sin_h;
  ParticleFilter(
      size_t n, const std::vector<SensorConfig> &sc, float start_x,
      float start_y, float start_spread = 1.0f, float noise = 0.25f,
      float motion_scale_ = 1.0f, // default: same units as map
      float sensor_scale_ =
          1.0f, // default: sensor reading already in map units
      float lateral_noise_factor_ = 0.01f, // much lower than forward noise
      float resample_distance_threshold_ = 2.0f) // threshold in map units
      : pos_noise(0.0f, 0.25f), motion_scale(motion_scale_),
        sensor_scale(sensor_scale_),
        resample_distance_threshold(resample_distance_threshold_) {
    particles.resize(n);
    std::normal_distribution<float> dist_x(start_x, start_spread);
    std::normal_distribution<float> dist_y(start_y, start_spread);
    normal_setup();

    for (auto &p : particles) {
      p.x = clamp(dist_x(gen), MAP_MIN, MAP_MAX);
      p.y = clamp(dist_y(gen), MAP_MIN, MAP_MAX);
      p.weight = 1.0f / n;
    }
    // Convert sensor mounting angles from degrees to radians.
    float fein = M_PI / 180.0f;
    for (const auto &s : sc) {
      float angle = s.angle * fein;
      sensors.push_back(
          {s.dx, s.dy, angle, PI_2 + std::cos(angle), PI_2 + std::sin(angle)});
    }
  }

  // Update the particle filter.
  // Parameters:
  //   compass_heading: current robot heading (in radians, compass style: 0 =
  //   North, CW positive) vl, vr: left and right wheel speeds (in odometry
  //   units per second) dt: time delta readings: array of sensor measurements
  //   (in sensor units)
  void update(float prev_heading, float compass_heading, float vl, float vr,
              const float *readings) {
    // In compass coordinates:
    //   forward vector: (sin(compass_heading), cos(compass_heading))
    //   right vector: (cos(compass_heading), -sin(compass_heading))
    float deltaY = (vl + vr) / 2;
    cum_distance += std::fabs(deltaY);

    float avg_heading = prev_heading + compass_heading / 2;

    float disX = deltaY * std::cos(avg_heading);
    float disY = deltaY * std::sin(avg_heading);

    for (auto &p : particles) {
      // Update particle's position:
      //   forward component: (sin_h, cos_h)
      //   lateral component: (cos_h, -sin_h)
      p.x = clamp(p.x + disX + normal() * 0.25f, MAP_MIN, MAP_MAX);
      p.y = clamp(p.y + disY + normal() * 0.25f, MAP_MIN, MAP_MAX);
    }
    cos_h = std::cos(prev_heading);
    sin_h = std::sin(prev_heading);

    // Sensor update: adjust particle weights based on sensor readings.
    float localX[sensors.size()], localY[sensors.size()];
    float cos_theta[sensors.size()], sin_theta[sensors.size()];
    size_t size_sensor = sensors.size() - 1;
    bool scan = false;
    float total_weight = 0.0f;
    for (auto &p : particles) {
      float weight = 1.0f;
      for (size_t i = 0; i < sensors.size(); ++i) {
        const auto &s = sensors[i];
        if (readings[i] >= 9998.0f) {
          if (!scan) {
            localX[i] = s.dx * cos_h + s.dy * sin_h;
            localY[i] = -s.dx * sin_h + s.dy * cos_h;
            cos_theta[i] = cos_h * s.cos_a + sin_h * s.sin_a;
            sin_theta[i] = s.sin_a * cos_h - s.cos_a * sin_h;
          }
          if (i == size_sensor && !scan) {
            scan = true;
          }
          continue;
        }

        // Convert to math angle (0 along positive X, CCW positive).
        // float sensor_math_angle =
        // std::fmod(PI_2 - prev_heading + s.angle, PI2);
        if (!scan) {
          localX[i] = s.dx * cos_h + s.dy * sin_h;
          localY[i] = -s.dx * sin_h + s.dy * cos_h;
          cos_theta[i] = cos_h * s.cos_a + sin_h * s.sin_a;
          sin_theta[i] = s.sin_a * cos_h - s.cos_a * sin_h;
        }
        if (i == size_sensor && !scan) {
          scan = true;
        }
        // Update weight using a Robust Likelihood Model
        weight *= calc_weight(p.x + localX[i], p.y + localY[i], cos_theta[i],
                              sin_theta[i], 1.85f, readings[i]);
      }
      p.weight = std::max(weight, 0.000001f);
      total_weight += weight;
    }
    float inv_total = 1.0f / total_weight;
    for (auto &p : particles)
      p.weight *= inv_total;

    if (total_weight == 0.0f)
      return;

    // Resample if enough (scaled) distance has been accumulated.
    if (cum_distance < resample_distance_threshold)
      return;
    cum_distance = 0.0f;

    std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
    float randWeight = distribution(de);

    std::vector<Particle> oldParticles = particles;

    size_t j = 0;
    float cumulativeWeight = 0.0f;
    size_t numParticles = particles.size();

    for (size_t i = 0; i < numParticles; i++) {
      float target = (static_cast<float>(i) + randWeight) / numParticles;
      while (cumulativeWeight < target && j < numParticles) {
        cumulativeWeight += oldParticles[j].weight;
        j++;
      }
      if (j > 0) {
        particles[i] = oldParticles[j - 1];
        particles[i].weight = 1.0f / numParticles;
      }
    }
  }

  // Estimate the robot's position as a weighted average of the particles.
  std::pair<float, float> estimate() const {
    float x = 0.0f, y = 0.0f;
    for (const auto &p : particles) {
      x += p.x * p.weight;
      y += p.y * p.weight;
    }
    return {x, y};
  }

private:
  // Compute the distance from (x0, y0) to the nearest wall along the direction
  // theta. theta is given in math coordinates (0 along positive X, CCW
  // positive).
  float wall_distance(double x0, double y0, float cos_theta, float sin_theta) {
    const double wall_min = MAP_MIN;
    const double wall_max = MAP_MAX;
    double min_dist = std::numeric_limits<double>::infinity();

    // Check intersection with vertical walls.
    if (fabs(cos_theta) > 1e-6) {
      double target_x = (cos_theta < 0) ? wall_min : wall_max;
      double t_x = (target_x - x0) / cos_theta;
      if (t_x >= 0) {
        double y_inter = y0 + t_x * sin_theta;
        if (y_inter >= wall_min && y_inter <= wall_max)
          min_dist = t_x;
      }
    }

    // Check intersection with horizontal walls.
    if (fabs(sin_theta) > 1e-6) {
      double target_y = (sin_theta < 0) ? wall_min : wall_max;
      double t_y = (target_y - y0) / sin_theta;
      if (t_y >= 0) {
        double x_inter = x0 + t_y * cos_theta;
        if (x_inter >= wall_min && x_inter <= wall_max)
          min_dist = std::min(min_dist, t_y);
      }
    }
    return min_dist;
  }

  // Gaussian probability approximation for sensor error.
  float exp_approx(float error, float sigma) {
    float diff = error / sigma;
    return (INV_SQRT_2PI / sigma) *
           vgetq_lane_f32(fast_exp_neon(vdupq_n_f32(-0.5f * diff * diff)), 0);
  }

  float calc_weight(float x, float y, float cos_theta, float sin_theta,
                    float stddev, float measurement) {
    // Compute the expected distance to the wall from the robot's pose and
    // orientation.
    float distance = wall_distance(x, y, cos_theta, sin_theta);

    // Calculate the difference between the expected distance and the actual
    // measurement.
    float distanceSub = distance - measurement;

    // Normalize the difference by the sensor noise (stddev).
    float diff = distanceSub / stddev;

    // Build a vector (pre_values) of arguments for the exponential function.
    // We assume each lane corresponds to a different component of the sensor
    // model: Lane 0: Used for the "max" component. Lane 1: Used for the lower
    // bound (or one part) of the "hit" component. Lane 2: Used for the upper
    // bound (or another part) of the "hit" component. Lane 3: Used for the
    // Gaussian (normal) "hit" likelihood component. (Make sure these
    // assignments match your mathematical derivation.)
    float32x4_t pre_values = {
        PI_SQRT3 * (78.74016f - distance) / stddev *
            SQRT3, // Lane 0: for max component
        PI_SQRT3 * -distance / stddev *
            SQRT3,          // Lane 1: for hit component part 1
        -pshort * distance, // Lane 2: for hit component part 2
        -0.5f * diff * diff // Lane 3: Gaussian exponent for hit likelihood
    };

    // Compute the exponentials for all four components at once.
    // That is, exp_values[i] = exp(pre_values[i]) for i = 0,1,2,3.
    float32x4_t exp_values = fast_exp_neon(pre_values);

    // Initialize the weight accumulator.
    float weight = 0;

    // If the actual measurement is less than (distance - measurement),
    // add the short-range component weight.
    // (This condition and formula should match your model for short
    // measurements.)
    if (measurement < distanceSub) {
      // The short component weight is scaled by 0.25 (i.e., 25% weight) and
      // computed as: 0.25 * (1 / (1 - exp(Gaussian term))) * pshort *
      // exp(Gaussian term), where exp(Gaussian term) is taken from lane 3.
      weight += 0.25f * (1.0f / (1.0f - vgetq_lane_f32(exp_values, 3))) *
                pshort * vgetq_lane_f32(exp_values, 3);
    }

    // If the measurement is at or beyond a high threshold (here, 75 units),
    // add the max-range component weight (again, scaled by 0.25).
    if (measurement >= 75.0f) {
      weight += 0.25f;
    }

    // Add the "hit" component weight.
    // This term is computed using a combination of the exponential values in
    // lanes 1 and 2:
    // - The denominator is the difference between the reciprocals of (1 +
    // exp_value from lane 1)
    //   and (1 + exp_value from lane 2).
    // - Multiply that by the normalization factor (INV_SQRT_2PI / stddev)
    //   and by exp_value from lane 2.
    // Finally, add a constant term (0.25 * MAX_DIST_RECIPROCAL) for further
    // adjustment.
    weight += 0.25f /
                  (1.0f / (1.0f + vgetq_lane_f32(exp_values, 1)) -
                   1.0f / (1.0f + vgetq_lane_f32(exp_values, 2))) *
                  (INV_SQRT_2PI / stddev) * vgetq_lane_f32(exp_values, 2) +
              0.25f * MAX_DIST_RECIPROCAL;

    // Return the final combined weight.
    return weight;
  }
};

#endif // LOCALIZATION_H