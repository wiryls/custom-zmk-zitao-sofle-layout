/*
 * Touchpad lamp: breathe only while waiting for a new BLE pairing.
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_ZMK_BLE)
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_HAS_CHOSEN(zmk_trackpad_led), "Touchpad lamp device is required");
static const struct device *const led_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_trackpad_led));

#define CHILD_COUNT(...) +1
#define LAMP_LED_COUNT (DT_FOREACH_CHILD(DT_CHOSEN(zmk_trackpad_led), CHILD_COUNT))

static int set_lamp_brightness(uint8_t level) {
    for (int i = 0; i < LAMP_LED_COUNT; i++) {
        int ret = led_set_brightness(led_dev, i, level);
        if (ret < 0) {
            LOG_ERR("Failed to set touchpad lamp brightness (%d)", ret);
            return ret;
        }
    }
    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_BLE)
#define BREATH_PERIOD_MS 2400
#define BREATH_TICK_MS 40

static void lamp_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(lamp_work, lamp_work_handler);
/* Animation state belongs to lamp_work only; event handlers merely schedule it. */
static bool breathing;
static int64_t breath_started_at;

static bool waiting_for_new_pairing(void) {
    return zmk_endpoints_selected().transport == ZMK_TRANSPORT_BLE &&
           zmk_ble_active_profile_is_open() && !zmk_ble_active_profile_is_connected();
}

/* Smoothstep up/down, using bounded integer arithmetic and no floating point. */
static uint8_t breathing_brightness(uint32_t elapsed_ms) {
    uint32_t phase = elapsed_ms % BREATH_PERIOD_MS;
    uint32_t half = BREATH_PERIOD_MS / 2;
    uint32_t distance = phase <= half ? phase : BREATH_PERIOD_MS - phase;
    uint32_t position = distance * 1000U / half;
    uint32_t eased = position * position * (3000U - 2U * position) / 1000000U;
    return eased * CONFIG_TOUCHPAD_PAIRING_LAMP_BRIGHTNESS / 1000U;
}

static void lamp_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    if (!waiting_for_new_pairing()) {
        breathing = false;
        set_lamp_brightness(0);
        return; /* No polling or animation while the lamp is off. */
    }

    int64_t now = k_uptime_get();
    if (!breathing) {
        breathing = true;
        breath_started_at = now;
    }
    uint32_t elapsed = (uint32_t)((now - breath_started_at) % BREATH_PERIOD_MS);
    if (set_lamp_brightness(breathing_brightness(elapsed)) < 0) {
        return;
    }
    k_work_reschedule(&lamp_work, K_MSEC(BREATH_TICK_MS));
}

static int lamp_state_changed(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    k_work_reschedule(&lamp_work, K_NO_WAIT);
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(touchpad_pairing_lamp, lamp_state_changed);
ZMK_SUBSCRIPTION(touchpad_pairing_lamp, zmk_endpoint_changed);
ZMK_SUBSCRIPTION(touchpad_pairing_lamp, zmk_ble_active_profile_changed);
#endif

static int touchpad_lamp_init(void) {
    if (!device_is_ready(led_dev)) {
        return -ENODEV;
    }
    int ret = set_lamp_brightness(0);
    if (ret < 0) {
        return ret;
    }
#if IS_ENABLED(CONFIG_ZMK_BLE)
    /* Initial refresh after startup; subsequent changes are event-driven. */
    k_work_reschedule(&lamp_work, K_MSEC(100));
#endif
    return 0;
}

SYS_INIT(touchpad_lamp_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
