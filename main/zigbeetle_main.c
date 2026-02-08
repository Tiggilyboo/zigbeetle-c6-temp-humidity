#include <stdbool.h>
#include <stdint.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "lp_core_i2c.h"
#include "lp_core_main.h"
#include "nvs_flash.h"
#include "ulp_lp_core.h"

#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"

#define TAG "ZIGBEETLE"

#define AHT_LP_SDA GPIO_NUM_6
#define AHT_LP_SCL GPIO_NUM_7

#define HA_ENDPOINT 10
#define MANUFACTURER_NAME "\x09" "ZigBeetle"
#define MODEL_IDENTIFIER "\x06" "AHT21B"

#define INSTALLCODE_POLICY_ENABLE false
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE 3000
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

#define TEMP_DELTA_CENTI 30
#define HUM_DELTA_CENTI 300
#define HEARTBEAT_MS (900000UL)

#define SAMPLE_FAST_S 5
#define SAMPLE_NORMAL_S 15
#define SAMPLE_SLOW_S 60
#define MIN_VALID_SAMPLES_TO_REPORT 3

#define ESP_ZB_ZED_CONFIG() { \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
    .nwk_cfg.zed_cfg = { .ed_timeout = ED_AGING_TIMEOUT, .keep_alive = ED_KEEP_ALIVE, }, \
}

#define ESP_ZB_DEFAULT_RADIO_CONFIG() { .radio_mode = ZB_RADIO_MODE_NATIVE }
#define ESP_ZB_DEFAULT_HOST_CONFIG() { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE }

extern const uint8_t lp_core_main_bin_start[] asm("_binary_lp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[] asm("_binary_lp_core_main_bin_end");

static bool s_have_last_sample;
static int32_t s_prev_sample_temp_centi;
static uint32_t s_prev_sample_hum_centi;

static bool s_have_report;
static int32_t s_last_report_temp_centi;
static uint32_t s_last_report_hum_centi;
static uint32_t s_last_report_ms;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static int16_t temp_centi_to_zcl(int32_t centi)
{
    return (int16_t)centi;
}

static uint16_t hum_centi_to_zcl(uint32_t centi)
{
    if (centi > 10000U) {
        centi = 10000U;
    }
    return (uint16_t)centi;
}

static uint32_t abs_diff_u32(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static uint32_t abs_diff_i32(int32_t a, int32_t b)
{
    return (uint32_t)((a >= b) ? (a - b) : (b - a));
}

static uint32_t choose_next_interval_s(int32_t temp_centi, uint32_t hum_centi)
{
    if (!s_have_last_sample) {
        return SAMPLE_NORMAL_S;
    }

    uint32_t dtemp = abs_diff_i32(temp_centi, s_prev_sample_temp_centi);
    uint32_t dhum = abs_diff_u32(hum_centi, s_prev_sample_hum_centi);

    if (dtemp >= TEMP_DELTA_CENTI || dhum >= HUM_DELTA_CENTI) {
        return SAMPLE_FAST_S;
    }
    if (dtemp >= 10U || dhum >= 100U) {
        return SAMPLE_NORMAL_S;
    }
    return SAMPLE_SLOW_S;
}

static esp_zb_cluster_list_t *create_clusters(void)
{
    esp_zb_temperature_sensor_cfg_t cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    cfg.temp_meas_cfg.min_value = -4000;
    cfg.temp_meas_cfg.max_value = 12500;

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&(cfg.basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(cfg.identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(&(cfg.temp_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_humidity_meas_cluster_cfg_t hum_cfg = {
        .measured_value = 0,
        .min_value = 0,
        .max_value = 10000,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create(&hum_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}

static esp_zb_ep_list_t *create_endpoint(void)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep = {
        .endpoint = HA_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, create_clusters(), ep);
    return ep_list;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg = signal_struct->p_app_signal;
    esp_zb_app_signal_type_t sig_type = *p_sg;
    esp_err_t st = signal_struct->esp_err_status;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (st == ESP_OK) {
            ESP_LOGI(TAG, "Start network steering (factory_new=%d)", esp_zb_bdb_is_factory_new() ? 1 : 0);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (st == ESP_OK) {
            ESP_LOGI(TAG, "Joined Zigbee network");
        } else {
            ESP_LOGW(TAG, "Network steering failed, retrying");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        break;
    }
}

static void zigbee_task(void *arg)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = create_endpoint();
    esp_zb_device_register(ep_list);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

static void lp_i2c_init(void)
{
    lp_core_i2c_cfg_t i2c_cfg = LP_CORE_I2C_DEFAULT_CONFIG();
    i2c_cfg.i2c_pin_cfg.sda_io_num = AHT_LP_SDA;
    i2c_cfg.i2c_pin_cfg.scl_io_num = AHT_LP_SCL;
    i2c_cfg.i2c_timing_cfg.clk_speed_hz = 100000;
    ESP_ERROR_CHECK(lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c_cfg));
}

static void lp_core_start(void)
{
    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER | ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
        .lp_timer_sleep_duration_us = 0,
    };

    ESP_ERROR_CHECK(ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start)));
    ESP_ERROR_CHECK(ulp_lp_core_run(&cfg));
}

static void publish_measurement(int32_t temp_centi, uint32_t hum_centi)
{
    int16_t zcl_temp = temp_centi_to_zcl(temp_centi);
    uint16_t zcl_hum = hum_centi_to_zcl(hum_centi);

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                 &zcl_temp,
                                 false);
    esp_zb_zcl_set_attribute_val(HA_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                                 &zcl_hum,
                                 false);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Report avg=%.2f C %.2f %%", temp_centi / 100.0f, hum_centi / 100.0f);
}

static void monitor_task(void *arg)
{
    uint32_t seen_seq = 0;
    uint32_t last_diag_ms = 0;

    while (1) {
        uint32_t seq1 = 0;
        uint32_t seq2 = 0;
        uint32_t count = 0;
        int32_t temp_sum = 0;
        uint32_t hum_sum = 0;
        int32_t last_temp = 0;
        uint32_t last_hum = 0;

        do {
            seq1 = ulp_lp_data_seq;
            count = ulp_lp_sample_count;
            temp_sum = ulp_lp_temp_sum_centi;
            hum_sum = ulp_lp_hum_sum_centi;
            last_temp = ulp_lp_last_temp_centi;
            last_hum = ulp_lp_last_hum_centi;
            seq2 = ulp_lp_data_seq;
        } while (seq1 != seq2);

        if (seq1 != seen_seq) {
            uint32_t next_s = choose_next_interval_s(last_temp, last_hum);
            ulp_lp_sample_interval_s = next_s;

            s_prev_sample_temp_centi = last_temp;
            s_prev_sample_hum_centi = last_hum;
            s_have_last_sample = true;
            seen_seq = seq1;
        }

        bool heartbeat_due = (now_ms() - s_last_report_ms) >= HEARTBEAT_MS;
        bool change_due = false;
        if (s_have_report && s_have_last_sample) {
            change_due = abs_diff_i32(last_temp, s_last_report_temp_centi) >= TEMP_DELTA_CENTI ||
                         abs_diff_u32(last_hum, s_last_report_hum_centi) >= HUM_DELTA_CENTI;
        }

        if (count >= MIN_VALID_SAMPLES_TO_REPORT && (!s_have_report || heartbeat_due || change_due)) {
            int32_t avg_temp = temp_sum / (int32_t)count;
            uint32_t avg_hum = hum_sum / count;

            if (avg_temp < -4000 || avg_temp > 8500 || avg_hum > 10000U) {
                ESP_LOGW(TAG, "Reject invalid average: %.2f C %.2f %% (%lu samples)",
                         avg_temp / 100.0f, avg_hum / 100.0f, (unsigned long)count);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            publish_measurement(avg_temp, avg_hum);

            s_last_report_temp_centi = avg_temp;
            s_last_report_hum_centi = avg_hum;
            s_last_report_ms = now_ms();
            s_have_report = true;

            ulp_lp_sample_count = 0;
            ulp_lp_temp_sum_centi = 0;
            ulp_lp_hum_sum_centi = 0;
        }

        uint32_t now = now_ms();
        if ((now - last_diag_ms) >= 5000U) {
            ESP_LOGI(TAG,
                     "LP diag: seq=%lu cnt=%lu interval=%lus last=%.2fC %.2f%% err=%lu i2c=%lu status=%lu zero=%lu crc=%lu range=%lu raw=%02lx %02lx %02lx %02lx %02lx %02lx %02lx",
                     (unsigned long)ulp_lp_data_seq,
                     (unsigned long)ulp_lp_sample_count,
                     (unsigned long)ulp_lp_sample_interval_s,
                     ulp_lp_last_temp_centi / 100.0f,
                     ulp_lp_last_hum_centi / 100.0f,
                     (unsigned long)ulp_lp_error_count,
                     (unsigned long)ulp_lp_fail_i2c_count,
                     (unsigned long)ulp_lp_fail_status_count,
                     (unsigned long)ulp_lp_fail_zero_count,
                     (unsigned long)ulp_lp_fail_crc_count,
                     (unsigned long)ulp_lp_fail_range_count,
                     (unsigned long)ulp_lp_last_raw0,
                     (unsigned long)ulp_lp_last_raw1,
                     (unsigned long)ulp_lp_last_raw2,
                     (unsigned long)ulp_lp_last_raw3,
                     (unsigned long)ulp_lp_last_raw4,
                     (unsigned long)ulp_lp_last_raw5,
                     (unsigned long)ulp_lp_last_raw6);
            last_diag_ms = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    ulp_lp_sample_interval_s = SAMPLE_NORMAL_S;
    ulp_lp_sample_count = 0;
    ulp_lp_temp_sum_centi = 0;
    ulp_lp_hum_sum_centi = 0;
    ulp_lp_data_seq = 0;

    lp_i2c_init();
    lp_core_start();

    xTaskCreate(zigbee_task, "zigbee", 6144, NULL, 5, NULL);
    xTaskCreate(monitor_task, "monitor", 4096, NULL, 4, NULL);
}
