/*
 * sensor_registry.c — Sensor registry, NVS persistence, and utilities
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 */

#include "sensor_registry.h"

#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "main.h"
#include "hub_aggregate.h"
#include "uart_master.h"

static const char *TAG = "REGISTRY";

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

/* ── Global state ─────────────────────────────────────────────────────────── */

hub_config_safe_t     g_config            = {0};
sensor_runtime_meta_t g_meta[MAX_SENSORS] = {0};
volatile bool         g_watchdog_started  = false;
volatile int          g_new_sensor_count  = 0;
volatile int64_t      g_utc_boot_epoch    = 0;

static volatile bool  s_dirty             = false;

/* ── Config mutex helpers ─────────────────────────────────────────────────── */

hub_config_t *lock_config(void)
{
    if (!g_config.mutex) return NULL;
    if (xSemaphoreTake(g_config.mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
        return &g_config.data;
    ESP_LOGW(TAG, "lock_config timeout");
    return NULL;
}

void unlock_config(void)
{
    if (g_config.mutex) xSemaphoreGive(g_config.mutex);
}

void mark_dirty(void) { s_dirty = true; }

/* ── Friendly name lookup ─────────────────────────────────────────────────── */

static const char *k_friendly[] = {
    /* 0 */ "UNKNOWN",
    /* 1 */ "ZG-204ZL",
    /* 2 */ "UNKNOWN",   /* was ZG-205Z/A mmWave, removed */
    /* 3 */ "ZG-102Z",
    /* 4 */ "ZG-102ZA",
};

const char *friendly_name_from_type(sensor_type_t t)
{
    if ((unsigned)t < sizeof(k_friendly) / sizeof(k_friendly[0]))
        return k_friendly[(unsigned)t];
    return "UNKNOWN";
}

const char *hub_aggregate_str(hub_aggregate_t a)
{
    return a == HUB_AGG_OCCUPIED ? "OCCUPIED" : "VACANT";
}

const char *role_str(sensor_role_t r)
{
    switch (r) {
    case ROLE_DOOR:     return "DOOR";
    case ROLE_PRESENCE: return "PRESENCE";
    default:            return "UNKNOWN";
    }
}

/* ── Runtime meta ─────────────────────────────────────────────────────────── */

void reset_meta_runtime(int i)
{
    g_meta[i].miss_count           = 0;
    g_meta[i].reporting_configured = false;
    g_meta[i].config_sent          = false;
    g_meta[i].bound_once           = false;
    g_meta[i].power_config_bound   = false;
    g_meta[i].enroll_sent          = false;
    g_meta[i].ep_active            = 1;
    g_meta[i].bind_pending         = 0;
    g_meta[i].bind_confirmed       = 0;
    g_meta[i].bind_failed          = 0;
    g_meta[i].model_id_pending     = false;
    g_meta[i].model_id_req_ms      = 0;
    g_meta[i].ping_pending         = false;
    g_meta[i].keep_time_sec        = PIR_KEEP_TIME_DEFAULT_SEC;
    g_meta[i].sensitivity          = PIR_SENSITIVITY_DEFAULT;
}

/* ── NVS ─────────────────────────────────────────────────────────────────── */

esp_err_t save_config(hub_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open("sensor_hub", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    nvs_set_u8(h, "mode",         (uint8_t)config->mode);
    nvs_set_u8(h, "sensor_count", config->sensor_count);
    nvs_set_u8(h, "hub_agg",      (uint8_t)config->hub_status.aggregate);
    for (int i = 0; i < config->sensor_count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "sensor_%d", i);
        nvs_set_blob(h, key, &config->sensors[i], sizeof(sensor_t));
    }
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void restore_meta_from_nvs(hub_config_t *config)
{
    /* Maps sensor_type_t enum value → model_id string (NULL = unknown). */
    static const char * const k_model_strings[] = {
        NULL,         /* SENSOR_UNKNOWN = 0 */
        "ZG-204ZL",   /* SENSOR_ZG_204ZL = 1 */
        NULL,         /* 2 = unused (was ZG-205Z/A) */
        "ZG-102Z",    /* SENSOR_ZG_102Z = 3 */
        "ZG-102ZA",   /* SENSOR_ZG_102ZA = 4 */
    };

    for (int i = 0; i < config->sensor_count; i++) {
        sensor_type_t t = (sensor_type_t)config->sensors[i].sensor_type;
        config->sensors[i].online = false;
        reset_meta_runtime(i);
        if ((unsigned)t < sizeof(k_model_strings) / sizeof(k_model_strings[0])
            && k_model_strings[(unsigned)t] != NULL) {
            g_meta[i].model_known = true;
            strncpy(g_meta[i].model_id, k_model_strings[(unsigned)t],
                    sizeof(g_meta[i].model_id) - 1);
        }
    }
}

esp_err_t load_config(hub_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    memset(g_meta, 0, sizeof(g_meta));

    nvs_handle_t h;
    esp_err_t err = nvs_open("sensor_hub", NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        config->mode                 = MODE_PAIRING;
        config->sensor_count         = 0;
        config->hub_status.aggregate = HUB_AGG_VACANT;
        PROD_LOG(TAG, "NVS empty — first boot");
        return ESP_OK;
    }
    if (err != ESP_OK) return err;

    uint8_t mode = MODE_PAIRING;
    nvs_get_u8(h, "mode", &mode);
    config->mode = (hub_mode_t)mode;
    config->hub_status.aggregate = HUB_AGG_VACANT;

    uint8_t count = 0;
    nvs_get_u8(h, "sensor_count", &count);
    if (count > MAX_SENSORS) count = MAX_SENSORS;

    config->sensor_count = 0;
    for (int i = 0; i < count; i++) {
        char key[16]; size_t sz = sizeof(sensor_t);
        snprintf(key, sizeof(key), "sensor_%d", i);
        if (nvs_get_blob(h, key, &config->sensors[i], &sz) == ESP_OK)
            config->sensor_count++;
    }
    nvs_close(h);
    restore_meta_from_nvs(config);
    PROD_LOG(TAG, "Loaded %d sensor(s) from NVS", config->sensor_count);
    return ESP_OK;
}

/* ── Online helper ────────────────────────────────────────────────────────── */

void mark_online_locked(hub_config_t *c, int idx)
{
    bool was_off = !c->sensors[idx].online;
    c->sensors[idx].online    = true;
    c->sensors[idx].last_seen = (time_t)(esp_timer_get_time() / 1000000ULL);
    g_meta[idx].ping_pending  = false;
    g_meta[idx].miss_count    = 0;
    if (was_off) {
        PROD_LOG(TAG, "[WDG] %s ONLINE", c->sensors[idx].sensor_name);
        uart_master_send_sensor_health(c->sensors[idx].sensor_name, true);
        update_hub_aggregate_locked(c);
    }
}

/* ── Registry ────────────────────────────────────────────────────────────── */

int find_index_by_short(uint16_t short_addr)
{
    hub_config_t *c = lock_config();
    if (!c) return -1;
    int found = -1;
    for (int i = 0; i < c->sensor_count; i++)
        if (c->sensors[i].short_addr == short_addr) { found = i; break; }
    unlock_config();
    return found;
}

/* Forward declaration — pairing_active is a static in zigbee_core.c but we
 * need to read it here.  Exposed via this thin accessor to avoid global leakage. */
extern bool zigbee_is_pairing_active(void);

void register_or_update(uint16_t short_addr, const char *ieee)
{
    hub_config_t *c = lock_config();
    if (!c) return;
    int idx = -1; bool is_new = false;
    for (int i = 0; i < c->sensor_count; i++)
        if (strcmp(c->sensors[i].ieee_addr, ieee) == 0) { idx = i; break; }

    if (idx < 0) {
        if (c->sensor_count >= MAX_SENSORS) { unlock_config(); return; }
        idx = c->sensor_count++; is_new = true;
        memset(&c->sensors[idx], 0, sizeof(sensor_t));
        memset(&g_meta[idx], 0, sizeof(sensor_runtime_meta_t));
        reset_meta_runtime(idx);
    } else {
        g_meta[idx].reporting_configured = false;
        g_meta[idx].config_sent          = false;
        g_meta[idx].power_config_bound   = false;
        g_meta[idx].ping_pending         = false;
        g_meta[idx].miss_count           = 0;
        g_meta[idx].model_id_pending     = false;
    }

    c->sensors[idx].short_addr = short_addr;
    strncpy(c->sensors[idx].ieee_addr, ieee, IEEE_ADDR_STR_LEN - 1);
    c->sensors[idx].ieee_addr[IEEE_ADDR_STR_LEN - 1] = '\0';
    c->sensors[idx].endpoint  = 1;
    c->sensors[idx].last_seen = (time_t)(esp_timer_get_time() / 1000000ULL);
    c->sensors[idx].online    = true;
    if (c->sensors[idx].sensor_name[0] == '\0')
        snprintf(c->sensors[idx].sensor_name, SENSOR_NAME_LEN, "Sensor_%d", idx + 1);

    if (is_new && zigbee_is_pairing_active()) {
        sensor_t *s = &c->sensors[idx];
        char name[SENSOR_NAME_LEN];
        strncpy(name, s->sensor_name, SENSOR_NAME_LEN - 1);
        name[SENSOR_NAME_LEN - 1] = '\0';
        const char *fn = friendly_name_from_type((sensor_type_t)s->sensor_type);
        const char *rl = role_str((sensor_role_t)s->sensor_role);
        unlock_config();
        uart_master_send_new_sensor_joined(idx, name, fn, rl);
        g_new_sensor_count++;
        mark_dirty();
        return;
    }
    unlock_config();
    mark_dirty();
}

/* ── Persist task ────────────────────────────────────────────────────────── */

void persist_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (s_dirty) {
            hub_config_t *c = lock_config();
            if (c) {
                esp_err_t e = save_config(c);
                unlock_config();
                if (e == ESP_OK) s_dirty = false;
            }
        }
    }
}
