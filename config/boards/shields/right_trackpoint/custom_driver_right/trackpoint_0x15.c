/*
 * TrackPoint HID over I2C Driver (Zephyr Input Subsystem)
 * Interrupt-driven pointer-only driver; key bindings are owned by the keymap.
 * Copyright (c) 2025 ZitaoTech
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_trackpoint

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <stdlib.h>
#include <math.h>

#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#include <zitaotech/pointing_sensitivity.h>

LOG_MODULE_REGISTER(trackpoint, LOG_LEVEL_DBG);

/* Dedicated work queue. */
#define TP_WORKQ_STACK_SIZE 2048
#define TP_WORKQ_PRIORITY 5

/* Serialize sensor transactions. */
static struct k_mutex trackpoint_i2c_mutex;

K_THREAD_STACK_DEFINE(tp_workq_stack, TP_WORKQ_STACK_SIZE);
static struct k_work_q tp_workq;

/* Pointer calibration (Kconfig percentages). */
#define MOUSE_BASE_SPEED (CONFIG_TRACKPOINT_MOUSE_BASE_SPEED_PERCENT / 100.0f)
#define MOUSE_SENS_BASE (CONFIG_TRACKPOINT_MOUSE_SENS_BASE_PERCENT / 100.0f)
#define MOUSE_SENS_STEP (CONFIG_TRACKPOINT_MOUSE_SENS_STEP_PERCENT / 100.0f)

/* Motion GPIO. */

#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 14
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

/* Packet format. */
#define TRACKPOINT_PACKET_LEN 7
#define TRACKPOINT_MAGIC_BYTE0 0x50

struct trackpoint_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
};

struct trackpoint_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback motion_cb_data;
    struct k_work_delayable enable_irq_work;
    uint32_t last_packet_time;
};

/* Optional pointer acceleration. */
#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
#define TP_MAX_MULT 2.0f
static inline float trackpoint_exponential_factor(int8_t dx, int8_t dy, uint32_t delta_ms) {
    if (delta_ms == 0)
        delta_ms = 1;

    int dist = abs(dx) + abs(dy);
    if (dist < 1)
        return 1.0f;

    float speed = (float)dist / (float)delta_ms;
    float mult = expf(speed * 1.307357f);

    return (mult > TP_MAX_MULT) ? TP_MAX_MULT : mult;
}
#endif

/* Sensor packet read. */
static int trackpoint_read_packet(const struct device *dev, int8_t *dx, int8_t *dy) {
    const struct trackpoint_config *cfg = dev->config;
    uint8_t buf[TRACKPOINT_PACKET_LEN] = {0};

    int ret = k_mutex_lock(&trackpoint_i2c_mutex, K_NO_WAIT);
    if (ret < 0) {
        return ret;
    }

    ret = i2c_read_dt(&cfg->i2c, buf, TRACKPOINT_PACKET_LEN);

    k_mutex_unlock(&trackpoint_i2c_mutex);

    if (ret < 0)
        return ret;

    if (buf[0] != TRACKPOINT_MAGIC_BYTE0)
        return -EIO;

    *dx = (int8_t)buf[2];
    *dy = (int8_t)buf[3];
    return 0;
}

/* Report pointer motion only. Preserve the existing gain, acceleration and axis signs. */
static void trackpoint_report_motion(const struct device *dev, int8_t dx, int8_t dy,
                                    uint32_t now) {
    float tp_factor = MOUSE_SENS_BASE + MOUSE_SENS_STEP * sofle_pointer_sensitivity_get();

#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
    struct trackpoint_data *data = dev->data;
    uint32_t delta = now - data->last_packet_time;
    float exp_mult = trackpoint_exponential_factor(dx, dy, delta);
#else
    (void)now;
    float exp_mult = 1.0f;
#endif

    float fx = dx * MOUSE_BASE_SPEED * tp_factor * exp_mult;
    float fy = dy * MOUSE_BASE_SPEED * tp_factor * exp_mult;

    input_report_rel(dev, INPUT_REL_X, -(int)fx, false, K_NO_WAIT);
    input_report_rel(dev, INPUT_REL_Y, -(int)fy, true, K_NO_WAIT);
}

static void trackpoint_work_cb(struct k_work *work) {
    struct trackpoint_data *data = CONTAINER_OF(work, struct trackpoint_data, work);
    const struct device *dev = data->dev;

    uint32_t now = k_uptime_get_32();

    int8_t dx = 0, dy = 0;

    /* Read one packet per motion event. */
    int ret = trackpoint_read_packet(dev, &dx, &dy);

    if (ret != 0) {
        LOG_WRN("TrackPoint packet read failed (%d)", ret);

        return;
    }

    trackpoint_report_motion(dev, dx, dy, now);

    data->last_packet_time = now;
    k_msleep(5);
}

/* Motion interrupt. */
static void motion_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(pins);
    struct trackpoint_data *data = CONTAINER_OF(cb, struct trackpoint_data, motion_cb_data);
    k_work_submit_to_queue(&tp_workq, &data->work);
}

static void trackpoint_enable_irq_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct trackpoint_data *data = CONTAINER_OF(dwork, struct trackpoint_data, enable_irq_work);
    const struct device *dev = data->dev;
    const struct trackpoint_config *cfg = dev->config;

    int ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to enable TrackPoint IRQ (%d)", ret);
        return;
    }
    ret = gpio_pin_get_dt(&cfg->motion_gpio);
    if (ret < 0) {
        LOG_ERR("Failed to read TrackPoint motion GPIO (%d)", ret);
        return;
    }
    if (ret > 0) {
        k_work_submit_to_queue(&tp_workq, &data->work);
    }
    LOG_INF("TrackPoint IRQ enabled (delayed)");
}
/* Device initialization. */
static int trackpoint_init(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;
    if (!i2c_is_ready_dt(&cfg->i2c))
        return -ENODEV;
    if (!gpio_is_ready_dt(&cfg->motion_gpio))
        return -ENODEV;

    int ret = gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
    if (ret < 0) {
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_DISABLE);
    if (ret < 0) {
        return ret;
    }
    k_mutex_init(&trackpoint_i2c_mutex);

    data->dev = dev;
    data->last_packet_time = k_uptime_get_32();

    k_work_init(&data->work, trackpoint_work_cb);

    k_work_init_delayable(&data->enable_irq_work, trackpoint_enable_irq_work_cb);
    gpio_init_callback(&data->motion_cb_data, motion_isr, BIT(cfg->motion_gpio.pin));
    ret = gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb_data);
    if (ret < 0) {
        return ret;
    }
    /* Start the queue only after synchronous GPIO setup has succeeded. */
    k_work_queue_start(&tp_workq, tp_workq_stack, K_THREAD_STACK_SIZEOF(tp_workq_stack),
                       TP_WORKQ_PRIORITY, NULL);
    ret = k_work_schedule(&data->enable_irq_work, K_MSEC(200));
    if (ret < 0) {
        gpio_remove_callback(cfg->motion_gpio.port, &data->motion_cb_data);
        return ret;
    }

    LOG_INF("TrackPoint Driver Initialized (IRQ delayed)");
    return 0;
}

#define TRACKPOINT_DEFINE(inst)                                                                    \
    static struct trackpoint_data trackpoint_data_##inst;                                          \
    static const struct trackpoint_config trackpoint_config_##inst = {                             \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .motion_gpio = {.port = DEVICE_DT_GET(MOTION_GPIO_NODE),                                   \
                        .pin = MOTION_GPIO_PIN,                                                    \
                        .dt_flags = MOTION_GPIO_FLAGS},                                            \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, trackpoint_init, NULL, &trackpoint_data_##inst,                    \
                          &trackpoint_config_##inst, POST_KERNEL, 70, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_DEFINE);
