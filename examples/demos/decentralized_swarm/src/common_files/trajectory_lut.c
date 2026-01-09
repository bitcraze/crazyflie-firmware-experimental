/**
 * Trajectory Lookup Table (LUT) Implementation
 */

#include "trajectory_lut.h"
#include "pptraj.h"
#include "settings.h"
#include "movement.h"
#include "debug.h"
#include <string.h>
#include <math.h>

// Lookup table storage
static TrajectoryPosition spiral_trajectory_lut[TRAJECTORY_LUT_MAX_SAMPLES];
static int trajectory_lut_sample_count = 0;
static float trajectory_duration_sec = 0.0f;
static bool lut_initialized = false;

/**
 * Calculate total trajectory duration including timescale
 */
static float calculateTrajectoryDuration(struct poly4d sequence[], int count, float timescale) {
    float total = 0.0f;
    for (int i = 0; i < count; i++) {
        total += sequence[i].duration;
    }
    return total * timescale;
}

/**
 * Evaluate trajectory position at a given time
 *
 * @param sequence Array of polynomial pieces
 * @param n_pieces Number of pieces
 * @param timescale Time scaling factor
 * @param t_global Time since trajectory start (in seconds)
 * @param out_pos Output position
 * @return true if successful, false if time is out of bounds
 */
static bool evaluateTrajectoryAtTime(
    struct poly4d sequence[],
    int n_pieces,
    float timescale,
    float t_global,
    TrajectoryPosition *out_pos
) {
    // Find which polynomial piece contains this time
    float t_remaining = t_global;

    for (int i = 0; i < n_pieces; i++) {
        float piece_duration = sequence[i].duration * timescale;

        if (t_remaining <= piece_duration) {
            // This is the right piece - evaluate it
            // Need to scale time back to unscaled domain for polynomial evaluation
            float t_local = t_remaining / timescale;

            // Evaluate position using poly4d_eval
            struct traj_eval eval = poly4d_eval(&sequence[i], t_local);

            out_pos->x = eval.pos.x;
            out_pos->y = eval.pos.y;
            out_pos->z = eval.pos.z;

            return true;
        }

        t_remaining -= piece_duration;
    }

    // Time is beyond trajectory duration
    return false;
}

void initializeTrajectoryLUT(void) {
    if (lut_initialized) {
        DEBUG_PRINT("Trajectory LUT already initialized\n");
        return;
    }

    // Get trajectory sequence and timescale from movement.c
    struct poly4d* sequence = getTrajectorySequence();
    uint8_t trajectory_timescale = getTrajectoryTimescale();

    // The sequence array has 23 pieces
    const int n_pieces = 23;

    // Calculate total duration
    trajectory_duration_sec = calculateTrajectoryDuration(sequence, n_pieces, trajectory_timescale);

    DEBUG_PRINT("Initializing Trajectory LUT: duration=%.2fs, interval=%dms\n",
                (double)trajectory_duration_sec, TRAJECTORY_LUT_SAMPLE_INTERVAL_MS);

    // Sample trajectory at regular intervals
    float sample_interval_sec = TRAJECTORY_LUT_SAMPLE_INTERVAL_MS / 1000.0f;
    int sample_idx = 0;

    for (float t = 0.0f; t <= trajectory_duration_sec && sample_idx < TRAJECTORY_LUT_MAX_SAMPLES;
         t += sample_interval_sec) {

        TrajectoryPosition pos;
        if (evaluateTrajectoryAtTime(sequence, n_pieces, trajectory_timescale, t, &pos)) {
            spiral_trajectory_lut[sample_idx] = pos;
            sample_idx++;
        } else {
            DEBUG_PRINT("Warning: Failed to evaluate trajectory at t=%.2f\n", (double)t);
            break;
        }
    }

    trajectory_lut_sample_count = sample_idx;
    lut_initialized = true;

    DEBUG_PRINT("Trajectory LUT initialized: %d samples, %.2fKB\n",
                trajectory_lut_sample_count,
                (double)((trajectory_lut_sample_count * sizeof(TrajectoryPosition)) / 1024.0f));
}

bool getTrajectoryPredictedPositions(
    float t_elapsed,
    TrajectoryPosition positions[],
    int max_positions,
    int *out_count
) {
    if (!lut_initialized) {
        DEBUG_PRINT("Error: Trajectory LUT not initialized\n");
        *out_count = 0;
        return false;
    }

    if (t_elapsed < 0.0f || t_elapsed > trajectory_duration_sec) {
        *out_count = 0;
        return false;
    }

    // Calculate sample interval
    float sample_interval_sec = TRAJECTORY_LUT_SAMPLE_INTERVAL_MS / 1000.0f;

    // Calculate how many samples to return based on prediction horizon
    int prediction_samples = (int)(TRAJECTORY_LUT_PREDICTION_HORIZON_SEC / sample_interval_sec);
    if (prediction_samples > max_positions) {
        prediction_samples = max_positions;
    }

    // Find starting index in LUT
    int start_idx = (int)(t_elapsed / sample_interval_sec);
    if (start_idx >= trajectory_lut_sample_count) {
        *out_count = 0;
        return false;
    }

    // Copy predicted positions from LUT
    int count = 0;
    for (int i = 0; i < prediction_samples; i++) {
        int idx = start_idx + i;
        if (idx >= trajectory_lut_sample_count) {
            break; // Reached end of trajectory
        }

        positions[count] = spiral_trajectory_lut[idx];
        count++;
    }

    *out_count = count;
    return count > 0;
}

float getTrajectoryDuration(void) {
    return trajectory_duration_sec;
}
