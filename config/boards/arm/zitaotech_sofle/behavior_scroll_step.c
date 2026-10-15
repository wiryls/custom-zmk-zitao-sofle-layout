/* SPDX-License-Identifier: MIT */
#define DT_DRV_COMPAT zitaotech_behavior_scroll_step

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <drivers/behavior.h>
#include <dt-bindings/zmk/pointing.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int scroll_step_pressed(struct zmk_behavior_binding *binding,
                               struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);
    int16_t x = MOVE_X_DECODE(binding->param1);
    int16_t y = MOVE_Y_DECODE(binding->param1);
    /* One axis per event, within the ordinary HID wheel range. */
    if ((x && y) || (!x && !y) || x < -127 || x > 127 || y < -127 || y > 127) {
        return -EINVAL;
    }
    return input_report_rel(DEVICE_DT_INST_GET(0),
                            x ? INPUT_REL_HWHEEL : INPUT_REL_WHEEL,
                            x ? x : y, true, K_NO_WAIT);
}

static int scroll_step_released(struct zmk_behavior_binding *binding,
                                struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api scroll_step_api = {
    .binding_pressed = scroll_step_pressed,
    .binding_released = scroll_step_released,
    .locality = BEHAVIOR_LOCALITY_CENTRAL,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &scroll_step_api);
#endif
