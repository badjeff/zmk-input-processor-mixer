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

struct zip_input_processor_mixer_config {
};

struct zip_input_processor_mixer_data {
    const struct device *dev;
    float cpi_scale, x_scalar, y_scalar;
    float mat0[3][3], mat1[3][3];
    int16_t x0, x1, y0, y1;
    bool sync0, sync1;
    int64_t last_rpt_time;
    float xrmd, yrmd;
    int16_t x, y;
};

// Prepare matrix with spherical angles
// theta = polar angle from +Z axis (in radians)
// phi   = azimuthal angle from +X axis in XY plane (in radians)
static void prepare_mat(float theta_from, float phi_from, float theta_to, float phi_to,
                        float matrix[3][3]) {

    // --- Convert spherical to Cartesian (unit vectors) ---
    float from_x = sinf(theta_from) * cosf(phi_from);
    float from_y = sinf(theta_from) * sinf(phi_from);
    float from_z = cosf(theta_from);

    float to_x   = sinf(theta_to) * cosf(phi_to);
    float to_y   = sinf(theta_to) * sinf(phi_to);
    float to_z   = cosf(theta_to);

    // --- Normalize both vectors ---
    float from_len = sqrtf(from_x*from_x + from_y*from_y + from_z*from_z);
    from_x /= from_len;
    from_y /= from_len;
    from_z /= from_len;

    float to_len = sqrtf(to_x*to_x + to_y*to_y + to_z*to_z);
    to_x /= to_len;
    to_y /= to_len;
    to_z /= to_len;

    // --- Rotation axis (cross product) ---
    float axis_x = from_y * to_z - from_z * to_y;
    float axis_y = from_z * to_x - from_x * to_z;
    float axis_z = from_x * to_y - from_y * to_x;

    float axis_len = sqrtf(axis_x*axis_x + axis_y*axis_y + axis_z*axis_z);
    if (axis_len < 1e-6f) {
        LOG_ERR("Rotation axis degenerate — from and to vectors collinear.");
        return;
    }
    axis_x /= axis_len;
    axis_y /= axis_len;
    axis_z /= axis_len;

    // --- Angle between from and to (dot product) ---
    float cos_angle = from_x*to_x + from_y*to_y + from_z*to_z;
    if (cos_angle > 1.0f) cos_angle = 1.0f;
    if (cos_angle < -1.0f) cos_angle = -1.0f;
    float sin_angle = sqrtf(1.0f - cos_angle*cos_angle);

    // --- Rodrigues’ rotation formula (3x3) ---
    matrix[0][0] = cos_angle + axis_x*axis_x*(1-cos_angle);
    matrix[0][1] = axis_x*axis_y*(1-cos_angle) - axis_z*sin_angle;
    // matrix[0][2] = axis_x*axis_z*(1-cos_angle) + axis_y*sin_angle;

    matrix[1][0] = axis_y*axis_x*(1-cos_angle) + axis_z*sin_angle;
    matrix[1][1] = cos_angle + axis_y*axis_y*(1-cos_angle);
    // matrix[1][2] = axis_y*axis_z*(1-cos_angle) - axis_x*sin_angle;

    // matrix[2][0] = axis_z*axis_x*(1-cos_angle) - axis_y*sin_angle;
    // matrix[2][1] = axis_z*axis_y*(1-cos_angle) + axis_x*sin_angle;
    // matrix[2][2] = cos_angle + axis_z*axis_z*(1-cos_angle);
}

static void dot_rot(float matrix[3][3], const float dx, const float dy, float *x, float *y) {
    *x = matrix[0][0] * dx + matrix[0][1] * dy;
    *y = matrix[1][0] * dx + matrix[1][1] * dy;
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

    static uint32_t sync_report_ms = 4;
    int64_t now = k_uptime_get();
    int64_t ts_diff = now - data->last_rpt_time;

    if (ts_diff > sync_report_ms) {
        // LOG_DBG("x0: %d, y0: %d, x1: %d, y1: %d", data->x0, data->y0, data->x1, data->y1);

        // Purge remainders if they are holden too long
        if (ts_diff > sync_report_ms * 5) {
            data->xrmd = 0;
            data->yrmd = 0;
        }

        float dx = 0, dy = 0;
        if ( (data->x0 != 0 || data->y0 != 0) && data->sync0) {
            dot_rot(data->mat0, data->x0, data->y0, &dx, &dy);
            data->xrmd += dx * data->x_scalar;
            data->yrmd += dy * data->y_scalar;
            data->x0 = 0;
            data->y0 = 0;
            data->sync0 = false;
        }
        if ( (data->x1 != 0 || data->y1 != 0) && data->sync1) {
            dot_rot(data->mat1, data->x1, data->y1, &dx, &dy);
            data->xrmd += dx * data->x_scalar;
            data->yrmd += dy * data->y_scalar;
            data->x1 = 0;
            data->y1 = 0;
            data->sync1 = false;
        }

        data->x = (int16_t) data->xrmd;
        data->y = (int16_t) data->yrmd;
        data->xrmd -= data->x;
        data->yrmd -= data->y;
        // LOG_DBG("x: %d, y: %d", data->x, data->y);

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

    // Sensor A:
    //   theta: side view -> tilt down from north pole
    //   phi:   top view ->  at 60° CCW from six o'clock position,
    const float theta_a = DEG2RAD(115.0f);  // polar (downward tilt)
    const float phi_a   = DEG2RAD(-60.0f);  // azimuth (left/right around equator)

    // Sensor B: 
    //   theta: side view -> tilt down from north pole
    //   phi:   top view ->  at 60° CW from six o'clock position,
    const float theta_b = DEG2RAD(115.0f); // polar (downward tilt)
    const float phi_b   = DEG2RAD(+60.0f); // azimuth (left/right around equator)

    // Target orientation:
    const float theta_target = DEG2RAD(-20.0f); // tilt from north pole, index finger rest point
    const float phi_target   = 0.0f;             // azimuth (left/right around equator)

    // Build matrices for both sensors
    prepare_mat(theta_a, phi_a, theta_target, phi_target, data->mat0);
    prepare_mat(theta_b, phi_b, theta_target, phi_target, data->mat1);

    // Set Virtual CPI
    data->cpi_scale = 500.0f / 3200.0f;

    // Reduce thumb movement that lead to spin (near north pole)
    data->x_scalar = data->cpi_scale * 0.533;
    data->y_scalar = data->cpi_scale;

    // invert Y axis if the target orientation located on north hemisphere
    if (theta_target >= DEG2RAD(-90.0f) && theta_target <= DEG2RAD(90.0f) ) {
        data->y_scalar *= -1.0f;
    }

    return 0;
}

#define TL_INST(n)                                                                                 \
    static struct zip_input_processor_mixer_data data_##n = {};                                    \
    static struct zip_input_processor_mixer_config config_##n = {};                                \
    DEVICE_DT_INST_DEFINE(n, &sy_init, NULL, &data_##n, &config_##n, POST_KERNEL,                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TL_INST)
