/*
 * A320 I2C driver: motion and optional optical contact input only.
 * Key bindings and lamp policy live elsewhere.
 * Copyright (c) 2025 ZitaoTech
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT avago_a320

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zitaotech/pointing_sensitivity.h>

LOG_MODULE_REGISTER(a320, CONFIG_A320_LOG_LEVEL);

#define A320_WORKQ_STACK_SIZE 2048
#define A320_WORKQ_PRIORITY 5
K_THREAD_STACK_DEFINE(a320_workq_stack, A320_WORKQ_STACK_SIZE);
static struct k_work_q a320_workq;

#define MOUSE_SENS_BASE (CONFIG_A320_MOUSE_SENS_BASE_PERCENT / 100.0f)
#define MOUSE_SENS_STEP (CONFIG_A320_MOUSE_SENS_STEP_PERCENT / 100.0f)
#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 5
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)
#define A320_PACKET_LEN 3
#define A320_MAX_PACKETS_PER_WORK 32
#define A320_IRQ_DELAY_MS 200
#if IS_ENABLED(CONFIG_A320_FINGER_PRESENCE)
#define A320_REG_PRODUCT_ID 0x00
#define A320_PRODUCT_ID 0x83
#define A320_REG_MOTION 0x02
#define A320_MOTION_READY BIT(7)
#define A320_GPIO_STATUS BIT(0)
#define A320_REG_OFN_ENGINE 0x60
#define A320_ENGINE_ENABLE BIT(7)
#define A320_FPD_ENABLE BIT(2)
#define A320_REG_FPD_CTRL 0x75
#define A320_FPD_ACTIVE_HIGH BIT(6)
#define A320_REG_SHUTTER_BURST 0x86 /* TWI auto-increment from Shutter_Upper. */
#define A320_CONTACT_SHUTTER_ON 350
#define A320_CONTACT_SHUTTER_OFF 450
#define A320_FPD_POLL_MS 100
#define A320_FPD_MOTION_POLL_MS 20
#define A320_CONTACT_OFF_DELAY_MS 500
#define A320_FPD_MAX_ERRORS 3
#endif

struct a320_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
};

struct a320_data {
    const struct device *dev;
    struct k_mutex i2c_lock;
    struct k_work work;
    struct gpio_callback motion_cb_data;
    struct k_work_delayable enable_irq_work;
    /* Fractional HID motion is retained instead of becoming a small dead zone. */
    float motion_residual_x;
    float motion_residual_y;
#if IS_ENABLED(CONFIG_A320_FINGER_PRESENCE)
    struct k_work_delayable presence_work;
    /* Polling state is owned by a320_workq after startup, never by the ISR. */
    bool presence_polling;
    uint32_t presence_last_read_at;
    bool presence_reported;
    bool presence_candidate;
    uint32_t presence_candidate_at;
    uint8_t presence_samples;
    uint8_t presence_errors;
#if IS_ENABLED(CONFIG_A320_FPD_DIAGNOSTICS)
    uint32_t diagnostic_last_at;
#endif
#endif
};

static int a320_read_packet(const struct device *dev, int16_t *dx, int16_t *dy) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    uint8_t buf[A320_PACKET_LEN];
    uint8_t reg = 0x82;
    int ret = k_mutex_lock(&data->i2c_lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }

    /* Keep the sensor's existing register transactions; only fix their lifetime. */
    ret = i2c_write_dt(&cfg->i2c, &reg, 1);
    if (ret < 0) {
        goto out;
    }
    ret = i2c_burst_read_dt(&cfg->i2c, 0x82, buf, sizeof(buf));
    if (ret < 0) {
        goto out;
    }

    *dx = (int8_t)buf[1];
    /* Negate after widening: -(-128) must be +128, not another int8 wrap. */
    *dy = -(int16_t)(int8_t)buf[2];
out:
    k_mutex_unlock(&data->i2c_lock);
    return ret;
}

#if IS_ENABLED(CONFIG_A320_FINGER_PRESENCE)
/* A320 datasheet pp. 34, 45, 53: FPD drives GPIO; Motion bit 0 reports it.
 * TWI bit 7 in a register address means auto-increment, NOT write direction. */
static int a320_enable_finger_presence(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    uint8_t product, engine, control, verify;
    bool control_written = false, engine_written = false;
    int ret = k_mutex_lock(&data->i2c_lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }
    ret = i2c_burst_read_dt(&cfg->i2c, A320_REG_PRODUCT_ID, &product, 1);
    if (ret < 0) {
        goto out;
    }
    if (product != A320_PRODUCT_ID) {
        ret = -ENODEV;
        goto out;
    }
    ret = i2c_burst_read_dt(&cfg->i2c, A320_REG_OFN_ENGINE, &engine, 1);
    if (ret < 0) {
        goto out;
    }
    ret = i2c_burst_read_dt(&cfg->i2c, A320_REG_FPD_CTRL, &control, 1);
    if (ret < 0) {
        goto out;
    }
    /* Preserve the module's existing threshold, speed and motion settings. */
    uint8_t ctrl_write[] = {A320_REG_FPD_CTRL, control | A320_FPD_ACTIVE_HIGH};
    control_written = true;
    ret = i2c_write_dt(&cfg->i2c, ctrl_write, sizeof(ctrl_write));
    if (ret < 0) {
        goto out;
    }
    uint8_t engine_write[] = {A320_REG_OFN_ENGINE,
                              engine | A320_ENGINE_ENABLE | A320_FPD_ENABLE};
    engine_written = true;
    ret = i2c_write_dt(&cfg->i2c, engine_write, sizeof(engine_write));
    if (ret < 0) {
        goto out;
    }
    ret = i2c_burst_read_dt(&cfg->i2c, A320_REG_OFN_ENGINE, &verify, 1);
    if (ret < 0) {
        goto out;
    }
    if ((verify & (A320_ENGINE_ENABLE | A320_FPD_ENABLE)) !=
        (A320_ENGINE_ENABLE | A320_FPD_ENABLE)) {
        ret = -EIO;
        goto out;
    }
    ret = i2c_burst_read_dt(&cfg->i2c, A320_REG_FPD_CTRL, &verify, 1);
    if (ret == 0 && !(verify & A320_FPD_ACTIVE_HIGH)) {
        ret = -EIO;
    }
out:
    if (ret < 0) {
        /* A failed write may still have reached the sensor: restore best-effort. */
        if (engine_written) {
            uint8_t restore[] = {A320_REG_OFN_ENGINE, engine};
            int restore_ret = i2c_write_dt(&cfg->i2c, restore, sizeof(restore));
            if (restore_ret < 0) {
                LOG_WRN("A320 engine restore failed (%d)", restore_ret);
            }
        }
        if (control_written) {
            uint8_t restore[] = {A320_REG_FPD_CTRL, control};
            int restore_ret = i2c_write_dt(&cfg->i2c, restore, sizeof(restore));
            if (restore_ret < 0) {
                LOG_WRN("A320 FPD control restore failed (%d)", restore_ret);
            }
        }
    }
    k_mutex_unlock(&data->i2c_lock);
    return ret;
}

static void a320_report_presence(struct a320_data *data, bool present) {
    if (present == data->presence_reported) {
        return;
    }
    int ret = input_report_key(data->dev, INPUT_BTN_TOUCH, present, true, K_NO_WAIT);
    if (ret < 0) {
        LOG_WRN("A320 presence input queue unavailable (%d)", ret);
        return; /* Retry next sample; never mark an undelivered event as sent. */
    }
    data->presence_reported = present;
    LOG_INF("A320 finger presence: %s", present ? "present" : "absent");
}

#if IS_ENABLED(CONFIG_A320_FPD_DIAGNOSTICS)
static void a320_log_contact_diagnostics(struct a320_data *data, uint8_t status,
                                         uint16_t decision_shutter) {
    uint32_t now = k_uptime_get_32();
    if (now - data->diagnostic_last_at < 500) {
        return;
    }
    data->diagnostic_last_at = now;
    const struct a320_config *cfg = data->dev->config;
    /* Auto-increment from SQUAL through Minimum_Pixel; no delta registers. */
    uint8_t optics[6], mode, engine, control;
    int ret = k_mutex_lock(&data->i2c_lock, K_FOREVER);
    if (ret < 0) {
        LOG_WRN("A320 diagnostic lock failed (%d)", ret);
        return;
    }
    ret = i2c_burst_read_dt(&cfg->i2c, 0x85, optics, sizeof(optics));
    if (ret == 0) {
        ret = i2c_burst_read_dt(&cfg->i2c, 0x2e, &mode, 1);
    }
    if (ret == 0) {
        ret = i2c_burst_read_dt(&cfg->i2c, A320_REG_OFN_ENGINE, &engine, 1);
    }
    if (ret == 0) {
        ret = i2c_burst_read_dt(&cfg->i2c, A320_REG_FPD_CTRL, &control, 1);
    }
    k_mutex_unlock(&data->i2c_lock);
    if (ret < 0) {
        LOG_WRN("A320 diagnostic read failed (%d)", ret);
        return;
    }
    uint16_t shutter = ((uint16_t)optics[1] << 8) | optics[2];
    LOG_INF("A320 raw: t=%u status=%02x gpio=%u sq=%u shutter=%u max=%u sum=%u min=%u "
            "mode=%02x engine=%02x fpd=%02x candidate=%u samples=%u reported=%u "
            "decision_shutter=%u",
            now, status, !!(status & A320_GPIO_STATUS), optics[0], shutter, optics[3],
            optics[4], optics[5], mode, engine, control, data->presence_candidate,
            data->presence_samples, data->presence_reported, decision_shutter);
}
#endif

static void a320_presence_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct a320_data *data = CONTAINER_OF(dwork, struct a320_data, presence_work);
    const struct a320_config *cfg = data->dev->config;
    uint8_t status, shutter_bytes[2];
    data->presence_last_read_at = k_uptime_get_32();
    int ret = k_mutex_lock(&data->i2c_lock, K_FOREVER);
    if (ret == 0) {
        ret = i2c_burst_read_dt(&cfg->i2c, A320_REG_MOTION, &status, 1);
        if (ret == 0) {
            ret = i2c_burst_read_dt(&cfg->i2c, A320_REG_SHUTTER_BURST,
                                    shutter_bytes, sizeof(shutter_bytes));
        }
        k_mutex_unlock(&data->i2c_lock);
    }
    if (ret < 0) {
        data->presence_samples = 0;
        if (data->presence_errors < A320_FPD_MAX_ERRORS) {
            data->presence_errors++;
            if (data->presence_errors == A320_FPD_MAX_ERRORS) {
                LOG_WRN("A320 presence reads failed (%d); releasing contact", ret);
            }
        }
        if (data->presence_errors == A320_FPD_MAX_ERRORS) {
            a320_report_presence(data, false);
        }
    } else {
        data->presence_errors = 0;
        uint16_t shutter = ((uint16_t)shutter_bytes[0] << 8) | shutter_bytes[1];
        /* Experimental optical contact estimate, independent of movement.
         * GPIO/FPD can drop in Rest2 while optics still show a stationary finger.
         * Dead-band samples retain the delivered state and interrupt confirmation. */
        if (shutter < A320_CONTACT_SHUTTER_ON || shutter > A320_CONTACT_SHUTTER_OFF) {
            bool present = shutter < A320_CONTACT_SHUTTER_ON;
            if (data->presence_samples == 0 || present != data->presence_candidate) {
                data->presence_candidate = present;
                data->presence_candidate_at = data->presence_last_read_at;
                data->presence_samples = 1;
            } else if (data->presence_samples < UINT8_MAX) {
                data->presence_samples++;
            }
            /* Entry is immediate. Release uses elapsed time so motion-accelerated
             * diagnostic reads cannot shorten the requested layer grace period. */
            if (present || data->presence_last_read_at - data->presence_candidate_at >=
                               A320_CONTACT_OFF_DELAY_MS) {
                a320_report_presence(data, present);
            }
        } else {
            data->presence_candidate = data->presence_reported;
            data->presence_candidate_at = data->presence_last_read_at;
            data->presence_samples = 0;
        }
        if (status & A320_MOTION_READY) {
            /* Reading status can acknowledge MOTION; drain without relying on a new IRQ. */
            k_work_submit_to_queue(&a320_workq, &data->work);
        }
#if IS_ENABLED(CONFIG_A320_FPD_DIAGNOSTICS)
        a320_log_contact_diagnostics(data, status, shutter);
#endif
    }
    ret = k_work_reschedule_for_queue(&a320_workq, &data->presence_work,
                                      K_MSEC(A320_FPD_POLL_MS));
    if (ret < 0) {
        data->presence_polling = false;
        LOG_ERR("A320 presence polling stopped (%d)", ret);
        if (data->presence_reported) {
            /* No next poll can retry: wait for input queue space on this
             * dedicated worker, without an I2C lock or blocking the ISR. */
            int release_ret = input_report_key(data->dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
            if (release_ret < 0) {
                LOG_ERR("Failed to release A320 contact (%d)", release_ret);
            } else {
                data->presence_reported = false;
            }
        }
    }
}

static void a320_presence_on_motion(struct a320_data *data) {
    /* Called only on a320_workq. Motion does not imply contact, but it can
     * advance the next optical sample to catch a short transition. Repeated
     * motion never schedules reads closer than the accelerated interval. */
    if (!data->presence_polling) {
        return;
    }
    uint32_t elapsed = k_uptime_get_32() - data->presence_last_read_at;
    uint32_t delay = elapsed >= A320_FPD_MOTION_POLL_MS
                         ? 0
                         : A320_FPD_MOTION_POLL_MS - elapsed;
    int ret = k_work_reschedule_for_queue(&a320_workq, &data->presence_work,
                                          K_MSEC(delay));
    if (ret < 0) {
        LOG_WRN("Failed to advance A320 presence read (%d)", ret);
    }
}
#endif

/* Preserve the exact gain over time: report whole HID units now and carry
 * sub-unit motion forward instead of truncating raw +/-1 to a permanent zero. */
static void a320_report_motion(struct a320_data *data, int32_t dx, int32_t dy) {
    float a320_factor = MOUSE_SENS_BASE + MOUSE_SENS_STEP * sofle_pointer_sensitivity_get();
    float scaled_x = dx * 0.75f * a320_factor + data->motion_residual_x;
    float scaled_y = dy * 0.75f * a320_factor + data->motion_residual_y;
    int report_x = (int)scaled_x;
    int report_y = (int)scaled_y;

    data->motion_residual_x = scaled_x - report_x;
    data->motion_residual_y = scaled_y - report_y;
    if (report_x == 0 && report_y == 0) {
        return;
    }
    input_report_rel(data->dev, INPUT_REL_X, report_x, false, K_NO_WAIT);
    input_report_rel(data->dev, INPUT_REL_Y, report_y, true, K_NO_WAIT);
}

static void a320_work_cb(struct k_work *work) {
    struct a320_data *data = CONTAINER_OF(work, struct a320_data, work);
    const struct device *dev = data->dev;
    int32_t total_dx = 0;
    int32_t total_dy = 0;
    bool exhausted = true;

    for (int packet = 0; packet < A320_MAX_PACKETS_PER_WORK; packet++) {
        int16_t dx, dy;
        int ret = a320_read_packet(dev, &dx, &dy);
        if (ret < 0) {
            LOG_WRN("A320 I2C read failed (%d)", ret);
            exhausted = false;
            break;
        }
        if (dx == 0 && dy == 0) {
            exhausted = false;
            break;
        }
        total_dx += dx;
        total_dy += dy;
    }

    if (total_dx != 0 || total_dy != 0) {
        a320_report_motion(data, total_dx, total_dy);
#if IS_ENABLED(CONFIG_A320_FINGER_PRESENCE)
        a320_presence_on_motion(data);
#endif
    }
    if (exhausted) {
        /* Yield between bounded batches; a pending packet need not generate a new edge. */
        int ret = k_work_submit_to_queue(&a320_workq, &data->work);
        if (ret < 0) {
            LOG_ERR("Failed to continue A320 drain (%d)", ret);
        }
    }
}

static void motion_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(pins);
    struct a320_data *data = CONTAINER_OF(cb, struct a320_data, motion_cb_data);
    k_work_submit_to_queue(&a320_workq, &data->work);
}

static void a320_enable_irq_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct a320_data *data = CONTAINER_OF(dwork, struct a320_data, enable_irq_work);
    const struct a320_config *cfg = data->dev->config;
#if IS_ENABLED(CONFIG_A320_FINGER_PRESENCE)
    int presence_ret = a320_enable_finger_presence(data->dev);
    if (presence_ret < 0) {
        LOG_WRN("A320 hardware finger presence unavailable (%d); motion only", presence_ret);
    } else {
        /* Publish startup state before the queue can run its first sample. */
        data->presence_polling = true;
        int poll_ret = k_work_reschedule_for_queue(&a320_workq, &data->presence_work, K_NO_WAIT);
        if (poll_ret < 0) {
            data->presence_polling = false;
            LOG_ERR("Failed to start A320 presence polling (%d)", poll_ret);
        } else {
            LOG_INF("A320 optical contact estimate enabled (350/450 shutter, 100/20 ms poll)");
        }
    }
#endif
    int ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to enable A320 motion IRQ (%d)", ret);
        return;
    }
    ret = gpio_pin_get_dt(&cfg->motion_gpio);
    if (ret < 0) {
        LOG_ERR("Failed to read A320 motion GPIO (%d)", ret);
        return;
    }
    if (ret > 0) {
        k_work_submit_to_queue(&a320_workq, &data->work);
    }
}

static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    if (!i2c_is_ready_dt(&cfg->i2c) || !gpio_is_ready_dt(&cfg->motion_gpio)) {
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
    if (ret < 0) {
        return ret;
    }
    /* Never enable the IRQ before work and mutex initialization are complete. */
    ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_DISABLE);
    if (ret < 0) {
        return ret;
    }
    data->dev = dev;
    k_mutex_init(&data->i2c_lock);
    k_work_init(&data->work, a320_work_cb);
    k_work_init_delayable(&data->enable_irq_work, a320_enable_irq_work_cb);
#if IS_ENABLED(CONFIG_A320_FINGER_PRESENCE)
    k_work_init_delayable(&data->presence_work, a320_presence_work_cb);
#endif
    gpio_init_callback(&data->motion_cb_data, motion_isr, BIT(cfg->motion_gpio.pin));
    ret = gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb_data);
    if (ret < 0) {
        return ret;
    }
    k_work_queue_start(&a320_workq, a320_workq_stack, K_THREAD_STACK_SIZEOF(a320_workq_stack),
                       A320_WORKQ_PRIORITY, NULL);
    ret = k_work_schedule(&data->enable_irq_work, K_MSEC(A320_IRQ_DELAY_MS));
    if (ret < 0) {
        gpio_remove_callback(cfg->motion_gpio.port, &data->motion_cb_data);
        return ret;
    }
    return 0;
}

#define A320_DEFINE(inst)                                                                          \
    static struct a320_data a320_data_##inst;                                                       \
    static const struct a320_config a320_config_##inst = {                                          \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                          \
        .motion_gpio = {.port = DEVICE_DT_GET(MOTION_GPIO_NODE),                                    \
                        .pin = MOTION_GPIO_PIN,                                                     \
                        .dt_flags = MOTION_GPIO_FLAGS},                                             \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320_data_##inst, &a320_config_##inst,              \
                          POST_KERNEL, CONFIG_INPUT_A320_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE);
