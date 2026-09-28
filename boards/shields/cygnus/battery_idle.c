/*
 * ZMK stops its battery timer when a half leaves the active state, so a
 * split peripheral only publishes a new level after a keypress. This keeps
 * the same sample interval running through idle and pushes BAS when the
 * percentage changes. Deep sleep still powers the MCU off; the timer does
 * not survive that.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/logging/log.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/workqueue.h>

LOG_MODULE_REGISTER(zmk_split_battery_idle, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_CHOSEN(zmk_battery)

static const struct device *const battery = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_LITHIUM_VOLTAGE)
static uint8_t lithium_ion_mv_to_pct(int16_t bat_mv)
{
    if (bat_mv >= 4200) {
        return 100;
    } else if (bat_mv <= 3450) {
        return 0;
    }

    return bat_mv * 2 / 15 - 459;
}
#endif

static int read_state_of_charge(uint8_t *soc)
{
    struct sensor_value state_of_charge;
    int rc;

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_STATE_OF_CHARGE)
    rc = sensor_sample_fetch_chan(battery, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE);
    if (rc != 0) {
        return rc;
    }

    rc = sensor_channel_get(battery, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &state_of_charge);
    if (rc != 0) {
        return rc;
    }
#elif IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_LITHIUM_VOLTAGE)
    rc = sensor_sample_fetch_chan(battery, SENSOR_CHAN_VOLTAGE);
    if (rc != 0) {
        return rc;
    }

    struct sensor_value voltage;

    rc = sensor_channel_get(battery, SENSOR_CHAN_VOLTAGE, &voltage);
    if (rc != 0) {
        return rc;
    }

    uint16_t mv = voltage.val1 * 1000 + (voltage.val2 / 1000);

    state_of_charge.val1 = lithium_ion_mv_to_pct(mv);
#else
#error "Not a supported battery reporting fetch mode"
#endif

    *soc = state_of_charge.val1;
    return 0;
}

static void idle_battery_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    uint8_t soc;
    int rc = read_state_of_charge(&soc);

    if (rc != 0) {
        LOG_DBG("idle battery sample failed: %d", rc);
        return;
    }

    if (bt_bas_get_battery_level() == soc) {
        return;
    }

    rc = bt_bas_set_battery_level(soc);
    if (rc != 0) {
        LOG_WRN("idle BAS update failed: %d", rc);
        return;
    }

    LOG_DBG("idle battery update soc=%u", soc);
}

K_WORK_DEFINE(idle_battery_work, idle_battery_work_handler);

static void idle_battery_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &idle_battery_work);
}

K_TIMER_DEFINE(idle_battery_timer, idle_battery_timer_handler, NULL);

static void sync_idle_battery_timer(void)
{
    if (!device_is_ready(battery)) {
        return;
    }

    if (zmk_activity_get_state() == ZMK_ACTIVITY_IDLE) {
        k_timer_start(&idle_battery_timer, K_SECONDS(CONFIG_ZMK_BATTERY_REPORT_INTERVAL),
                      K_SECONDS(CONFIG_ZMK_BATTERY_REPORT_INTERVAL));
        return;
    }

    k_timer_stop(&idle_battery_timer);
}

static int activity_listener(const zmk_event_t *eh)
{
    if (as_zmk_activity_state_changed(eh)) {
        sync_idle_battery_timer();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_split_battery_idle, activity_listener);
ZMK_SUBSCRIPTION(zmk_split_battery_idle, zmk_activity_state_changed);

static int zmk_split_battery_idle_init(void)
{
    if (!device_is_ready(battery)) {
        LOG_ERR("battery device is not ready");
        return -ENODEV;
    }

    sync_idle_battery_timer();
    return 0;
}

SYS_INIT(zmk_split_battery_idle_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#else

static int zmk_split_battery_idle_init(void)
{
    return 0;
}

SYS_INIT(zmk_split_battery_idle_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif
