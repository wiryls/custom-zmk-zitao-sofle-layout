/*
 * Explicit sensitivity bindings. GLOBAL dispatch uses ZMK's existing split transport;
 * each half updates and saves its own independent level.
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zitaotech_behavior_pointing_sensitivity

#include <errno.h>
#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zitaotech/pointing_sensitivity.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
#include <zmk/split/bluetooth/service.h>
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
/* BLE has a smaller device-name buffer than the generic split command. */
BUILD_ASSERT(sizeof(DEVICE_DT_NAME(DT_DRV_INST(0))) <= ZMK_SPLIT_RUN_BEHAVIOR_DEV_LEN,
             "Sensitivity behavior name is too long for BLE split transport");
#endif

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static const struct behavior_parameter_value_metadata command_values[] = {
    {
        .display_name = "Decrease sensitivity",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = 0,
    },
    {
        .display_name = "Increase sensitivity",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = 1,
    },
};

static const struct behavior_parameter_metadata_set metadata_sets[] = {
    {
        .param1_values_len = ARRAY_SIZE(command_values),
        .param1_values = command_values,
    },
};

static const struct behavior_parameter_metadata metadata = {
    .sets_len = ARRAY_SIZE(metadata_sets),
    .sets = metadata_sets,
};
#endif

static int sensitivity_pressed(struct zmk_behavior_binding *binding,
                               struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);
    switch (binding->param1) {
    case 0:
        return sofle_pointer_sensitivity_adjust(-1);
    case 1:
        return sofle_pointer_sensitivity_adjust(1);
    default:
        return -EINVAL;
    }
}

static int sensitivity_released(struct zmk_behavior_binding *binding,
                                struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api sensitivity_api = {
    .binding_pressed = sensitivity_pressed,
    .binding_released = sensitivity_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sensitivity_api);
#endif
