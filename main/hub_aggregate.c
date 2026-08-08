/*
 * hub_aggregate.c — Hub presence aggregate
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 */

#include "hub_aggregate.h"
#include "uart_master.h"

#include <time.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "HUB_AGG";

static void uptime_str(char *buf, size_t len)
{
    uint64_t sec = esp_timer_get_time() / 1000000ULL;
    snprintf(buf, len, "%02u:%02u:%02u",
             (unsigned)(sec / 3600),
             (unsigned)((sec % 3600) / 60),
             (unsigned)(sec % 60));
}

#define PROD_LOG(tag, fmt, ...) do {                 \
    char _ts[10]; uptime_str(_ts, sizeof(_ts));      \
    ESP_LOGI(tag, "[%s] " fmt, _ts, ##__VA_ARGS__);  \
} while (0)

void update_hub_aggregate_locked(hub_config_t *c)
{
    bool any = false;
    for (int j = 0; j < c->sensor_count; j++) {
        sensor_type_t t = (sensor_type_t)c->sensors[j].sensor_type;
        if (t == SENSOR_ZG_204ZL
            && c->sensors[j].presence
            && c->sensors[j].online)
            any = true;
    }
    hub_aggregate_t na = any ? HUB_AGG_OCCUPIED : HUB_AGG_VACANT;
    if (na != c->hub_status.aggregate) {
        c->hub_status.aggregate   = na;
        c->hub_status.last_change = time(NULL);
        c->hub_status.timestamp   = time(NULL);
        PROD_LOG(TAG, "[HUB] aggregate=%s", hub_aggregate_str(na));
        uart_master_send_hub_aggregate(hub_aggregate_str(na));
    } else {
        c->hub_status.timestamp = time(NULL);
    }
}
