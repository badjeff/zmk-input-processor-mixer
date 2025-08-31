/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_mixer

#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(d) ((d) * M_PI / 180.0f)


// sync interal
static const uint32_t sync_report_ms = 4;

// unit of sync interal, to purge xy remainders after idle
static const uint8_t sync_purge_unit = 5;


// enable z-axis for scrolling
static bool scroll_enabled = true;

// enable z-axis lock for scrolling
// static bool scroll_lock_enabled = false;
static bool scroll_lock_enabled = true;

// scroll lock active, while continuous {N} ticks within {N} ms
static const uint8_t scroll_lock_active_tick_min = 3;
static const uint32_t scroll_lock_active_tick_within_ms = 350;

// scroll lock deactive, while no tick within {N} ms
static const uint32_t scroll_lock_deactive_cooldown_ms = 350;


struct zip_input_processor_mixer_config {
};

struct zip_input_processor_mixer_data {
    const struct device *dev;
    float cpi_scale, x_scalar, y_scalar, z_scalar, wheel_scalar;
    float mat0[3][3], mat1[3][3];
    int16_t x0, x1, y0, y1;
    bool sync0, sync1;
    int64_t last_rpt_time;
    int64_t last_whl_time;
    float xrmd, yrmd, zrmd;
    int16_t x, y, z;
};

// Prepare matrix using Rodrigues rotation formula
static void mat_from_vectors(const float from[3], const float to[3], float matrix[3][3]) {
    // 1. Calculate the rotation axis (k) and the cosine of the rotation angle (c)
    // The rotation axis k is the cross product of the from and to vectors.
    float k_x = from[1] * to[2] - from[2] * to[1];
    float k_y = from[2] * to[0] - from[0] * to[2];
    float k_z = from[0] * to[1] - from[1] * to[0];
    float k_len = sqrtf(k_x*k_x + k_y*k_y + k_z*k_z);

    // If the vectors are collinear (k_len is near zero), they are either aligned or opposite.
    if (k_len < 1.0e-6f) {
        // If they are aligned, the rotation is the identity matrix.
        if (from[0] == to[0] && from[1] == to[1] && from[2] == to[2]) {
            matrix[0][0] = 1.0f; matrix[0][1] = 0.0f; matrix[0][2] = 0.0f;
            matrix[1][0] = 0.0f; matrix[1][1] = 1.0f; matrix[1][2] = 0.0f;
            matrix[2][0] = 0.0f; matrix[2][1] = 0.0f; matrix[2][2] = 1.0f;
        } else {
            // If they are opposite, the rotation is a 180-degree rotation.
            // The rotation axis can be any vector perpendicular to 'from'.
            // Choose a simple one like the cross product with (1,0,0) or (0,1,0).
            float temp_axis_x = 0, temp_axis_y = 0, temp_axis_z = 0;
            if (fabs(from[0]) > 0.5f) { // If from is not parallel to yz plane, use (0,1,0) as temp vector
                 temp_axis_x = from[1]*0 - from[2]*1;
                 temp_axis_y = from[2]*0 - from[0]*0;
                 temp_axis_z = from[0]*1 - from[1]*0;
            } else { // Otherwise use (1,0,0)
                 temp_axis_x = from[1]*0 - from[2]*0;
                 temp_axis_y = from[2]*1 - from[0]*0;
                 temp_axis_z = from[0]*0 - from[1]*1;
            }
            float temp_len = sqrtf(temp_axis_x*temp_axis_x + temp_axis_y*temp_axis_y + temp_axis_z*temp_axis_z);
            k_x = temp_axis_x/temp_len;
            k_y = temp_axis_y/temp_len;
            k_z = temp_axis_z/temp_len;
            
            // Rodrigues' formula for 180 degrees
            matrix[0][0] = 2 * k_x * k_x - 1;
            matrix[0][1] = 2 * k_x * k_y;
            matrix[0][2] = 2 * k_x * k_z;
            
            matrix[1][0] = 2 * k_y * k_x;
            matrix[1][1] = 2 * k_y * k_y - 1;
            matrix[1][2] = 2 * k_y * k_z;
            
            matrix[2][0] = 2 * k_z * k_x;
            matrix[2][1] = 2 * k_z * k_y;
            matrix[2][2] = 2 * k_z * k_z - 1;
        }
    } else {
        // Normalize the rotation axis
        k_x /= k_len;
        k_y /= k_len;
        k_z /= k_len;

        // Calculate cosine and sine of the rotation angle
        float c = from[0] * to[0] + from[1] * to[1] + from[2] * to[2];
        float s = sqrtf(1 - c*c);

        // 2. Apply Rodrigues' rotation formula
        float omc = 1.0f - c;
        matrix[0][0] = c + k_x*k_x*omc;
        matrix[0][1] = k_x*k_y*omc - k_z*s;
        matrix[0][2] = k_x*k_z*omc + k_y*s;

        matrix[1][0] = k_y*k_x*omc + k_z*s;
        matrix[1][1] = c + k_y*k_y*omc;
        matrix[1][2] = k_y*k_z*omc - k_x*s;

        matrix[2][0] = k_z*k_x*omc - k_y*s;
        matrix[2][1] = k_z*k_y*omc + k_x*s;
        matrix[2][2] = c + k_z*k_z*omc;
    }
}

// theta: elevation (tilt from equator)
// phi:   azimuth (left/right around equator)
static void sph_to_cart(float theta, float phi, float *x, float *y, float *z) {
    // Convert equator-origin theta to polar angle
    float polar = (float)M_PI/2.0f - theta;
    *x = cosf(phi) * sinf(polar);
    *y = sinf(phi) * sinf(polar);
    *z = cosf(polar);
}

// Prepare matrix from spherical coordinates
static void prepare_mat(float theta_from, float phi_from, float theta_to, float phi_to,
                        float matrix[3][3]) {

    float from_x, from_y, from_z;
    float to_x, to_y, to_z;
    sph_to_cart(theta_from, phi_from, &from_x, &from_y, &from_z);
    sph_to_cart(theta_to,   phi_to,   &to_x,   &to_y,   &to_z);

    // Normalize
    float from_len = sqrtf(from_x*from_x + from_y*from_y + from_z*from_z);
    from_x /= from_len;
    from_y /= from_len;
    from_z /= from_len;

    float to_len = sqrtf(to_x*to_x + to_y*to_y + to_z*to_z);
    to_x /= to_len;
    to_y /= to_len;
    to_z /= to_len;

    float from_vec[3] = {from_x, from_y, from_z};
    float to_vec[3] = {to_x, to_y, to_z};
    mat_from_vectors(from_vec, to_vec, matrix);
}

static void dot_mat(float matrix[3][3], const float dx, const float dy,
                    float *x, float *y, float *z) {
    float local_vec[3] = { dx, dy, 0.0f };

    // Rotate into global 3D space
    float rotated[3];
    rotated[0] = matrix[0][0] * local_vec[0] + matrix[0][1] * local_vec[1];
    rotated[1] = matrix[1][0] * local_vec[0] + matrix[1][1] * local_vec[1];
    rotated[2] = matrix[2][0] * local_vec[0] + matrix[2][1] * local_vec[1];

    // Target normal is the rotated (0,0,1), i.e. third column of matrix
    float nx = matrix[0][2];
    float ny = matrix[1][2];
    float nz = matrix[2][2];

    // Project rotated vector onto plane perpendicular to (nx,ny,nz)
    float dot = rotated[0]*nx + rotated[1]*ny + rotated[2]*nz;
    rotated[0] -= dot * nx;
    rotated[1] -= dot * ny;
    rotated[2] -= dot * nz;

    // Return projected XY
    *x = rotated[0];
    *y = rotated[1];
    *z = rotated[2];
}

static int sy_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state) {

    // const struct zip_input_processor_mixer_config *config = dev->config;
    struct zip_input_processor_mixer_data *data = dev->data;

    if (param1 == 0) {
        if      (event->code == INPUT_REL_X) data->x0 += event->value;
        else if (event->code == INPUT_REL_Y) data->y0 += -event->value;
        data->sync0 |= event->sync;
        // if (event->sync) LOG_DBG("x0: %d, y0: %d", data->x0, data->y0);
    }
    else if (param1 == 1) {
        if      (event->code == INPUT_REL_X) data->x1 += -event->value;
        else if (event->code == INPUT_REL_Y) data->y1 += event->value;
        data->sync1 |= event->sync;
        // if (event->sync) LOG_DBG("x1: %d, y1: %d", data->x1, data->y1);
    }

    event->value = 0;
    event->sync = false;

    int64_t now = k_uptime_get();
    int64_t rpt_diff = now - data->last_rpt_time;

    // Purge remainders if they are holden too long
    if (rpt_diff > sync_report_ms * sync_purge_unit) {
        data->xrmd = 0;
        data->yrmd = 0;
        data->zrmd = 0;
        data->x = 0;
        data->y = 0;
        data->z = 0;
    }

    if (rpt_diff > sync_report_ms) {
        // LOG_DBG("x0: %d, y0: %d, x1: %d, y1: %d", data->x0, data->y0, data->x1, data->y1);

        float dx = 0, dy = 0, dz = 0;
        if ( (data->x0 != 0 || data->y0 != 0) && data->sync0) {
            dot_mat(data->mat0, data->x0, data->y0, &dx, &dy, &dz);
            data->xrmd += dx * data->x_scalar;
            data->yrmd += dy * data->y_scalar;
            data->zrmd += dz * data->z_scalar;
            data->x0 = 0;
            data->y0 = 0;
            data->sync0 = false;
        }
        if ( (data->x1 != 0 || data->y1 != 0) && data->sync1) {
            dot_mat(data->mat1, data->x1, data->y1, &dx, &dy, &dz);
            data->xrmd += dx * data->x_scalar;
            data->yrmd += dy * data->y_scalar;
            data->zrmd += dz * data->z_scalar;
            data->x1 = 0;
            data->y1 = 0;
            data->sync1 = false;
        }

        data->x = (int16_t) data->xrmd;
        data->y = (int16_t) data->yrmd;
        data->z = (int16_t) data->zrmd;
        data->xrmd -= data->x;
        data->yrmd -= data->y;
        data->zrmd -= data->z;
        // LOG_DBG("x: %d, y: %d, z: %d", data->x, data->y, data->z);

        if (scroll_enabled) {
            int64_t whl_diff = now - data->last_whl_time;
            bool prefer_scroll = false;

            static uint32_t scroll_tick = 0;
            if (scroll_lock_enabled) {
                if (whl_diff > scroll_lock_deactive_cooldown_ms) {
                    scroll_tick = 0;
                }
                prefer_scroll = (scroll_tick >= scroll_lock_active_tick_min)
                                && (whl_diff <= scroll_lock_active_tick_within_ms);
            }

            const bool have_z = data->z != 0;
            if (have_z) {
                const bool z_ccw = data->x < 0 && data->z > 0;
                const bool z_cw  = data->x > 0 && data->z < 0;
                if (z_ccw || z_cw) {
                    int16_t z = data->z * data->wheel_scalar;
                    if (scroll_lock_enabled) {
                        if (prefer_scroll) {
                            input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, z, true, K_NO_WAIT);
                        }
                        scroll_tick++;
                    } else {
                        input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, z, true, K_NO_WAIT);
                    }
                    data->z = 0;
                    data->x = 0;
                    data->y = 0;
                }
                data->last_whl_time = now;
            }
            if (scroll_lock_enabled && prefer_scroll) {
                data->x = 0;
                data->y = 0;
            }
        }

        const bool have_x = data->x != 0;
        const bool have_y = data->y != 0;
        if (have_x || have_y) {
            if (have_x) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_X, data->x, !have_y, K_NO_WAIT);
                data->x = 0;
            }
            if (have_y) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_Y, data->y, true, K_NO_WAIT);
                data->y = 0;
            }
        }
        data->last_rpt_time = now;
    }

    return 0;
}

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

static int sy_init(const struct device *dev) {
    // const struct zip_input_processor_mixer_config *config = dev->config;
    struct zip_input_processor_mixer_data *data = dev->data;
    data->dev = dev;

    // Sensor A spherical coordinates
    // - theta: elevation (tilt from equator)
    // - phi:   azimuth (left/right around equator)
    const float theta_a = DEG2RAD(-25.0f);
    const float phi_a   = DEG2RAD(-60.0f);  // at 60° CCW from six o'clock position,

    // Sensor B spherical coordinates
    // - theta: elevation (tilt from equator)
    // - phi:   azimuth (left/right around equator)
    const float theta_b = DEG2RAD(-25.0f);
    const float phi_b   = DEG2RAD(+60.0f); // at 60° CW from six o'clock position,

    // Target orientation
    // - theta: elevation (tilt from equator)
    // - phi:   azimuth (left/right around equator)
    const float theta_target = DEG2RAD(-90.0f); // nadir
    const float phi_target   = DEG2RAD(0.0f); // azimuth (left/right around equator)

    // Build matrices for both sensors
    prepare_mat(theta_a, phi_a, theta_target, phi_target, data->mat0);
    prepare_mat(theta_b, phi_b, theta_target, phi_target, data->mat1);

    // Set Virtual CPI
    data->cpi_scale = 600.0f / 3200.0f;

    // Reduce thumb movement that lead to spin
    data->x_scalar = 1.0f * data->cpi_scale * 0.533f;
    data->y_scalar = 1.0f * data->cpi_scale;

    // z-axis sensitive, must have same direction to x_scalar
    data->z_scalar = 1.0f * data->cpi_scale * 0.0125f;
    data->wheel_scalar = 1.0f;

    return 0;
}

#define TL_INST(n)                                                                                 \
    static struct zip_input_processor_mixer_data data_##n = {};                                    \
    static struct zip_input_processor_mixer_config config_##n = {};                                \
    DEVICE_DT_INST_DEFINE(n, &sy_init, NULL, &data_##n, &config_##n, POST_KERNEL,                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TL_INST)
