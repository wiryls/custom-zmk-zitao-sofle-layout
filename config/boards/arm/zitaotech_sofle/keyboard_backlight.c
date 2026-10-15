/*
 * Real keyboard PWM backlight: independent switch, brightness and mode.
 * No RGB, shared external-power, activity or pointing-state dependencies.
 * SPDX-License-Identifier: MIT
 */
#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zitaotech/keyboard_backlight.h>
#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);
static const struct device *const backlight_dev = DEVICE_DT_GET(DT_NODELABEL(keyboard_backlight));

#define BRIGHTNESS_MASK 0xFFU
#define MODE_SHIFT 8
#define MODE_MASK (0xFFU << MODE_SHIFT)
#define ON_BIT (1U << 16)
#define VALID_MASK (BRIGHTNESS_MASK | MODE_MASK | ON_BIT)
#define PACK(brt, mode, on) ((uint32_t)(brt) | ((uint32_t)(mode) << MODE_SHIFT) | ((on) ? ON_BIT : 0U))
#define BRIGHTNESS(p) ((p) & BRIGHTNESS_MASK)
#define MODE(p) (((p) & MODE_MASK) >> MODE_SHIFT)
#define BREATH_PERIOD_MS 6000
#define BREATH_TICK_MS 40
#define SETTING_KEY "kb_light/state"

static atomic_t state = ATOMIC_INIT(PACK(20, SOFLE_BL_SOLID, 1));

static bool state_is_valid(uint32_t packed) {
    return !(packed & ~VALID_MASK) && BRIGHTNESS(packed) >= 1 &&
           BRIGHTNESS(packed) <= SOFLE_BL_MAX_BRIGHTNESS && MODE(packed) < SOFLE_BL_MODE_COUNT;
}

static void render_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(render_work, render_work_handler);

#if IS_ENABLED(CONFIG_SETTINGS)
static void save_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(save_work, save_work_handler);
#endif

int sofle_backlight_calculate(uint32_t command, uint32_t *packed) {
    if (!packed) {
        return -EINVAL;
    }
    uint32_t current = (uint32_t)atomic_get(&state);
    uint32_t brightness = BRIGHTNESS(current);
    uint32_t mode = MODE(current);
    bool on = (current & ON_BIT) != 0;
    switch (command) {
    case SOFLE_BL_DEC:
        brightness = MAX(1, (int)brightness - 1);
        break;
    case SOFLE_BL_INC:
        brightness = MIN(SOFLE_BL_MAX_BRIGHTNESS, brightness + 1);
        break;
    case SOFLE_BL_TOGGLE:
        on = !on;
        break;
    case SOFLE_BL_PREV:
        mode = (mode + SOFLE_BL_MODE_COUNT - 1) % SOFLE_BL_MODE_COUNT;
        break;
    case SOFLE_BL_NEXT:
        mode = (mode + 1) % SOFLE_BL_MODE_COUNT;
        break;
    default:
        return -EINVAL;
    }
    *packed = PACK(brightness, mode, on);
    return 0;
}

int sofle_backlight_apply(uint32_t packed) {
    if (!state_is_valid(packed)) {
        return -EINVAL;
    }
    atomic_val_t old = atomic_set(&state, (atomic_val_t)packed);
    if ((uint32_t)old == packed) {
        return 0;
    }
    int ret = k_work_reschedule(&render_work, K_NO_WAIT);
#if IS_ENABLED(CONFIG_SETTINGS)
    int save_ret = k_work_reschedule(&save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    if (save_ret < 0) {
        return save_ret;
    }
#endif
    return MIN(ret, 0);
}

static uint8_t breath_brightness(uint32_t elapsed, uint32_t peak) {
    uint32_t phase = elapsed % BREATH_PERIOD_MS;
    uint32_t half = BREATH_PERIOD_MS / 2;
    uint32_t distance = phase <= half ? phase : BREATH_PERIOD_MS - phase;
    uint32_t position = distance * 1000U / half;
    uint32_t eased = position * position * (3000U - 2U * position) / 1000000U;
    return eased * peak / 1000U;
}

static void render_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    /* Only this worker writes PWM and owns animation timing. */
    static uint32_t rendered_state = UINT32_MAX;
    static int64_t breath_started_at;
    uint32_t current = (uint32_t)atomic_get(&state);
    int64_t now = k_uptime_get();
    if (current != rendered_state) {
        if (rendered_state == UINT32_MAX || !(rendered_state & ON_BIT) ||
            MODE(rendered_state) != MODE(current)) {
            breath_started_at = now;
        }
        rendered_state = current;
    }

    uint8_t level = 0;
    bool breathe = (current & ON_BIT) && MODE(current) == SOFLE_BL_BREATHE;
    if (current & ON_BIT) {
        level = breathe ? breath_brightness((uint32_t)((now - breath_started_at) % BREATH_PERIOD_MS),
                                           BRIGHTNESS(current))
                        : BRIGHTNESS(current);
    }
    if (!device_is_ready(backlight_dev)) {
        return;
    }
    int ret = led_set_brightness(backlight_dev, 0, MIN(level, SOFLE_BL_MAX_BRIGHTNESS));
    if (ret < 0) {
        LOG_ERR("Keyboard backlight PWM update failed (%d)", ret);
        return;
    }
    if (breathe) {
        k_work_reschedule(&render_work, K_MSEC(BREATH_TICK_MS));
    }
}

#if IS_ENABLED(CONFIG_SETTINGS)
static int backlight_settings_load(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg) {
    if (strcmp(name, "state") != 0) {
        return -ENOENT;
    }
    uint8_t stored[3]; /* Explicit byte format: brightness, mode, on. */
    if (len != sizeof(stored)) {
        return -EINVAL;
    }
    int ret = read_cb(cb_arg, stored, sizeof(stored));
    if (ret < 0) {
        return ret;
    }
    if (ret != sizeof(stored) || stored[2] > 1) {
        return -EINVAL;
    }
    uint32_t packed = PACK(stored[0], stored[1], stored[2]);
    if (!state_is_valid(packed)) {
        return -EINVAL;
    }
    atomic_set(&state, (atomic_val_t)packed);
    ret = k_work_reschedule(&render_work, K_NO_WAIT);
    return MIN(ret, 0);
}

SETTINGS_STATIC_HANDLER_DEFINE(keyboard_light, "kb_light", NULL, backlight_settings_load, NULL,
                               NULL);

static void save_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    uint32_t current = (uint32_t)atomic_get(&state);
    uint8_t stored[3] = {BRIGHTNESS(current), MODE(current), (current & ON_BIT) ? 1 : 0};
    int ret = settings_save_one(SETTING_KEY, stored, sizeof(stored));
    if (ret < 0) {
        LOG_ERR("Failed to save keyboard backlight settings (%d)", ret);
    }
}
#endif

static int keyboard_backlight_init(void) {
    if (!device_is_ready(backlight_dev)) {
        return -ENODEV;
    }
    k_work_reschedule(&render_work, K_NO_WAIT);
    return 0;
}
SYS_INIT(keyboard_backlight_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
