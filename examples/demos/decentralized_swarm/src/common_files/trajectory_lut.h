/**
 * Trajectory Lookup Table (LUT) Module
 *
 * Pre-computes spiral trajectory positions at regular intervals for efficient
 * collision avoidance prediction. Instead of evaluating polynomials in real-time,
 * the entire trajectory is sampled during initialization and stored in memory.
 */

#ifndef __TRAJECTORY_LUT_H__
#define __TRAJECTORY_LUT_H__

#include <stdint.h>
#include <stdbool.h>

// Tunable parameters
#define TRAJECTORY_LUT_SAMPLE_INTERVAL_MS 100      // Time between samples in LUT
#define TRAJECTORY_LUT_PREDICTION_HORIZON_SEC 1.0f // How far ahead to predict

// Maximum number of samples in LUT (calculated based on max trajectory duration)
// Trajectory duration is ~27.5s (23 segments × 0.55s × 2 timescale)
// At 100ms intervals: 27.5s / 0.1s = 275 samples
#define TRAJECTORY_LUT_MAX_SAMPLES 300

typedef struct {
    float x;
    float y;
    float z;
} TrajectoryPosition;

/**
 * Initialize the trajectory lookup table.
 * This must be called once during system startup before any trajectory execution.
 * Pre-computes the entire spiral trajectory at TRAJECTORY_LUT_SAMPLE_INTERVAL_MS intervals.
 */
void initializeTrajectoryLUT(void);

/**
 * Get predicted positions for a trajectory at a given elapsed time.
 *
 * @param t_elapsed Time elapsed since trajectory start (in seconds)
 * @param positions Output array to store predicted positions
 * @param max_positions Maximum number of positions to return (size of positions array)
 * @param out_count Output: actual number of positions returned
 * @return true if successful, false if t_elapsed is out of bounds
 *
 * The function returns positions from t_elapsed to t_elapsed + PREDICTION_HORIZON_SEC,
 * sampled at TRAJECTORY_LUT_SAMPLE_INTERVAL_MS intervals.
 */
bool getTrajectoryPredictedPositions(
    float t_elapsed,
    TrajectoryPosition positions[],
    int max_positions,
    int *out_count
);

/**
 * Get the total duration of the trajectory in seconds.
 * @return Total trajectory duration
 */
float getTrajectoryDuration(void);

#endif // __TRAJECTORY_LUT_H__
