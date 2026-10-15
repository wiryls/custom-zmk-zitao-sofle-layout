/*
 * Per-half pointing sensitivity. No dependency on keyboard or LED state.
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zitaotech/pointing_sensitivity.h>

#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if defined(CONFIG_BOARD_ZITAOTECH_SOFLE_LEFT)
#define SENSITIVITY_DEFAULT CONFIG_SOFLE_TOUCHPAD_SENSITIVITY_DEFAULT
#define SENSITIVITY_KEY "pointing/touchpad"
#define SENSITIVITY_NAME "touchpad"
#elif defined(CONFIG_BOARD_ZITAOTECH_SOFLE_RIGHT)
#define SENSITIVITY_DEFAULT CONFIG_SOFLE_TRACKPOINT_SENSITIVITY_DEFAULT
#define SENSITIVITY_KEY "pointing/trackpoint"
#define SENSITIVITY_NAME "trackpoint"
#else
#error "Pointing sensitivity is supported only on ZitaoTech Sofle halves"
#endif

BUILD_ASSERT(SENSITIVITY_DEFAULT >= SOFLE_POINTER_LEVEL_MIN &&
                 SENSITIVITY_DEFAULT <= SOFLE_POINTER_LEVEL_MAX,
             "Invalid default pointer sensitivity");

static atomic_t sensitivity = ATOMIC_INIT(SENSITIVITY_DEFAULT);

uint8_t sofle_pointer_sensitivity_get(void) {
    return (uint8_t)atomic_get(&sensitivity);
}

#if IS_ENABLED(CONFIG_SETTINGS)
static int sensitivity_settings_load(const char *name, size_t len, settings_read_cb read_cb,
                                     void *cb_arg) {
    if (strcmp(name, SENSITIVITY_NAME) != 0) {
        return -ENOENT;
    }
    if (len != sizeof(uint8_t)) {
        return -EINVAL;
    }

    uint8_t stored;
    int ret = read_cb(cb_arg, &stored, sizeof(stored));
    if (ret < 0) {
        return ret;
    }
    if (ret != sizeof(stored) || stored < SOFLE_POINTER_LEVEL_MIN ||
        stored > SOFLE_POINTER_LEVEL_MAX) {
        return -EINVAL;
    }

    atomic_set(&sensitivity, stored);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(pointing_sensitivity, "pointing", NULL,
                               sensitivity_settings_load, NULL, NULL);

static void sensitivity_save(struct k_work *work) {
    ARG_UNUSED(work);
    uint8_t stored = sofle_pointer_sensitivity_get();
    int ret = settings_save_one(SENSITIVITY_KEY, &stored, sizeof(stored));
    if (ret < 0) {
        LOG_ERR("Failed to save pointer sensitivity (%d)", ret);
    }
}

static struct k_work_delayable sensitivity_save_work;

static int sensitivity_init(void) {
    k_work_init_delayable(&sensitivity_save_work, sensitivity_save);
    return 0;
}

SYS_INIT(sensitivity_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

int sofle_pointer_sensitivity_adjust(int direction) {
    if (direction != -1 && direction != 1) {
        return -EINVAL;
    }

    atomic_val_t before;
    atomic_val_t after;
    do {
        before = atomic_get(&sensitivity);
        after = CLAMP(before + direction * CONFIG_SOFLE_POINTER_SENSITIVITY_STEP,
                      SOFLE_POINTER_LEVEL_MIN, SOFLE_POINTER_LEVEL_MAX);
        if (after == before) {
            return 0;
        }
    } while (!atomic_cas(&sensitivity, before, after));

    LOG_INF("Pointer sensitivity %s: %d", SENSITIVITY_NAME, (int)after);
#if IS_ENABLED(CONFIG_SETTINGS)
    /* Reset the deadline on each change, rather than writing every key press. */
    int ret = k_work_reschedule(&sensitivity_save_work,
                               K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
#else
    return 0;
#endif
}
