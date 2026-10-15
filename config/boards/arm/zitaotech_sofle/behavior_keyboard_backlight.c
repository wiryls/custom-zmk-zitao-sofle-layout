/* SPDX-License-Identifier: MIT */
#define DT_DRV_COMPAT zitaotech_behavior_keyboard_backlight

#include <errno.h>
#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zitaotech/keyboard_backlight.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
#include <zmk/split/bluetooth/service.h>
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
/* BLE has a smaller device-name buffer than the generic split command. */
BUILD_ASSERT(sizeof(DEVICE_DT_NAME(DT_DRV_INST(0))) <= ZMK_SPLIT_RUN_BEHAVIOR_DEV_LEN,
             "Backlight behavior name is too long for BLE split transport");
#endif

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static const struct behavior_parameter_value_metadata commands[] = {
    {.display_name = "Brightness down", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = SOFLE_BL_DEC},
    {.display_name = "Brightness up", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = SOFLE_BL_INC},
    {.display_name = "Toggle backlight", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = SOFLE_BL_TOGGLE},
    {.display_name = "Previous mode", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = SOFLE_BL_PREV},
    {.display_name = "Next mode", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = SOFLE_BL_NEXT},
};
static const struct behavior_parameter_metadata_set sets[] = {
    {.param1_values_len = ARRAY_SIZE(commands), .param1_values = commands},
};
static const struct behavior_parameter_metadata metadata = {
    .sets_len = ARRAY_SIZE(sets), .sets = sets,
};
#endif

static int convert_command(struct zmk_behavior_binding *binding,
                           struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);
    /* Calculated on central before GLOBAL dispatch: both halves get the same state,
     * even if their stored brightness/mode differed while disconnected. */
    uint32_t packed;
    int ret = sofle_backlight_calculate(binding->param1, &packed);
    if (ret < 0) {
        return ret;
    }
    binding->param1 = SOFLE_BL_SET;
    binding->param2 = packed;
    return 0;
}

static int backlight_pressed(struct zmk_behavior_binding *binding,
                             struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);
    if (binding->param1 != SOFLE_BL_SET) {
        return -EINVAL;
    }
    return sofle_backlight_apply(binding->param2);
}

static int backlight_released(struct zmk_behavior_binding *binding,
                              struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api backlight_api = {
    .binding_convert_central_state_dependent_params = convert_command,
    .binding_pressed = backlight_pressed,
    .binding_released = backlight_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &backlight_api);
#endif
