#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
#include "lp_core_i2c.h"
#include "lp_core_main.h"
#include "ulp_lp_core.h"
#include "lp_core/aht21b_defs.h"
#endif
#include "nvs_flash.h"

#include "esp_zigbee_core.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_ota.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/esp_zigbee_zcl_ota.h"

#define TAG "ZIGBEETLE"

#if defined(CONFIG_DEBUG) && CONFIG_DEBUG
#define ZB_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#define ZB_LOGW(...) ESP_LOGW(TAG, __VA_ARGS__)
#define ZB_DIAGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define ZB_LOGI(...) do { } while (0)
#define ZB_LOGW(...) do { } while (0)
#define ZB_DIAGI(...) do { } while (0)
#endif

#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
#define AHT_LP_SDA GPIO_NUM_6
#define AHT_LP_SCL GPIO_NUM_7
#endif

#define HA_ENDPOINT 10
#define MANUFACTURER_NAME "\x09" "ZigBeetle"
#if defined(CONFIG_USE_104NT4) && CONFIG_USE_104NT4
#define MODEL_IDENTIFIER "\x07" "104NT-4"
#else
#define MODEL_IDENTIFIER "\x06" "AHT21B"
#endif

#define INSTALLCODE_POLICY_ENABLE false
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE 30000
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

#define TEMP_DELTA_CENTI 30
#define HUM_DELTA_CENTI 300
#define HEARTBEAT_MS (900000UL)
#define OTA_QUERY_INTERVAL_MIN 720U
#define OTA_FILE_VERSION 0x00000001UL
#define OTA_MANUFACTURER_CODE ESP_ZB_OTA_UPGRADE_MANUFACTURER_CODE_DEF_VALUE
#define OTA_IMAGE_TYPE ESP_ZB_OTA_UPGRADE_IMAGE_TYPE_DEF_VALUE
#define OTA_HW_VERSION 0x0101U
#define OTA_MAX_DATA_SIZE 223U
#define OTA_SERVER_SHORT_ADDR 0xFFFFU
#define OTA_SERVER_ENDPOINT 0xFFU

#define SAMPLE_MIN_S 10U
#define SAMPLE_MAX_S 120U
#define SAMPLE_DEFAULT_S 30U
#define MIN_VALID_SAMPLES_TO_REPORT 3
#define AHT_BOOT_SETTLE_MS 50
#define AHT_MEASUREMENT_WAIT_MS 90
#define THERM_SAMPLE_PERIOD_MS (SAMPLE_DEFAULT_S * 1000U)

#if defined(CONFIG_USE_104NT4) && CONFIG_USE_104NT4
#define THERM_GPIO GPIO_NUM_4
#define THERM_ADC_CHANNEL ADC_CHANNEL_4
#define THERM_ADC_MAX 4095.0f
#define THERM_PULLUP_OHMS 100000.0f
#define THERM_R25_OHMS 100000.0f
#define THERM_BETA_K 4267.0f
#define THERM_T0_K 298.15f
#define THERM_HUMIDITY_CENTI 0U
#define THERM_SAMPLE_COUNT 32
#define THERM_TRIM_COUNT 4
// Final reported temperature calibration:
// T_out = T_raw * THERM_CAL_GAIN + THERM_CAL_OFFSET_C
// Prefilled from observed room delta (~12.59C measured vs ~19.9C actual).
#define THERM_CAL_GAIN 1.0f
#define THERM_CAL_OFFSET_C 7.31f

static adc_oneshot_unit_handle_t s_therm_adc_handle;
#endif

#define ESP_ZB_ZED_CONFIG() { \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
    .nwk_cfg.zed_cfg = { .ed_timeout = ED_AGING_TIMEOUT, .keep_alive = ED_KEEP_ALIVE, }, \
}

#define ESP_ZB_DEFAULT_RADIO_CONFIG() { .radio_mode = ZB_RADIO_MODE_NATIVE }
#define ESP_ZB_DEFAULT_HOST_CONFIG() { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE }

#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
extern const uint8_t lp_core_main_bin_start[] asm("_binary_lp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[] asm("_binary_lp_core_main_bin_end");
#endif

static bool s_have_last_sample;
static int32_t s_prev_sample_temp_centi;
static uint32_t s_prev_sample_hum_centi;

static bool s_have_report;
static int32_t s_last_report_temp_centi;
static uint32_t s_last_report_hum_centi;
static uint32_t s_last_report_ms;
static volatile bool s_zb_network_joined;
static volatile uint32_t s_zb_join_seq;
static esp_zb_zcl_ota_upgrade_client_variable_t s_ota_client_variable = {
    .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
    .hw_version = OTA_HW_VERSION,
    .max_data_size = OTA_MAX_DATA_SIZE,
};
static uint16_t s_ota_server_addr = OTA_SERVER_SHORT_ADDR;
static uint8_t s_ota_server_endpoint = OTA_SERVER_ENDPOINT;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static int16_t temp_centi_to_zcl(int32_t centi)
{
    return (int16_t)centi;
}

#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
static uint16_t hum_centi_to_zcl(uint32_t centi)
{
    if (centi > 10000U) {
        centi = 10000U;
    }
    return (uint16_t)centi;
}
#endif

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
        return SAMPLE_DEFAULT_S;
    }

    uint32_t dtemp = abs_diff_i32(temp_centi, s_prev_sample_temp_centi);
    uint32_t dhum = abs_diff_u32(hum_centi, s_prev_sample_hum_centi);
    uint32_t temp_activity = (dtemp * 100U + (TEMP_DELTA_CENTI - 1U)) / TEMP_DELTA_CENTI;
    uint32_t hum_activity = (dhum * 100U + (HUM_DELTA_CENTI - 1U)) / HUM_DELTA_CENTI;
    uint32_t activity = (temp_activity > hum_activity) ? temp_activity : hum_activity;

    if (activity == 0U) {
        activity = 1U;
    }

    uint32_t interval_s = (SAMPLE_DEFAULT_S * 100U + (activity - 1U)) / activity;

    if (interval_s < SAMPLE_MIN_S) {
        interval_s = SAMPLE_MIN_S;
    }
    if (interval_s > SAMPLE_MAX_S) {
        interval_s = SAMPLE_MAX_S;
    }

    return interval_s;
}

static void reset_report_state_on_join(uint32_t *seen_join_seq)
{
    if (*seen_join_seq != s_zb_join_seq) {
        *seen_join_seq = s_zb_join_seq;
        s_have_report = false;
        s_last_report_ms = 0;
        ZB_LOGI("Join/rejoin detected, forcing first post-join report");
    }
}

static uint32_t note_latest_sample(int32_t temp_centi, uint32_t hum_centi)
{
    uint32_t next_s = choose_next_interval_s(temp_centi, hum_centi);

    s_prev_sample_temp_centi = temp_centi;
    s_prev_sample_hum_centi = hum_centi;
    s_have_last_sample = true;

    return next_s;
}

static bool report_due_for_measurement(int32_t temp_centi, uint32_t hum_centi)
{
    bool heartbeat_due = (now_ms() - s_last_report_ms) >= HEARTBEAT_MS;
    bool change_due = false;

    if (s_have_report) {
        change_due = abs_diff_i32(temp_centi, s_last_report_temp_centi) >= TEMP_DELTA_CENTI ||
                     abs_diff_u32(hum_centi, s_last_report_hum_centi) >= HUM_DELTA_CENTI;
    }

    return !s_have_report || heartbeat_due || change_due;
}

static void note_published_measurement(int32_t temp_centi, uint32_t hum_centi)
{
    s_last_report_temp_centi = temp_centi;
    s_last_report_hum_centi = hum_centi;
    s_last_report_ms = now_ms();
    s_have_report = true;
}

#if defined(CONFIG_USE_104NT4) && CONFIG_USE_104NT4
static void thermistor_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_therm_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_therm_adc_handle, THERM_ADC_CHANNEL, &chan_cfg));
}

static bool thermistor_read_temp_centi(int32_t *temp_centi_out)
{
    int samples[THERM_SAMPLE_COUNT] = {0};
    for (int i = 0; i < THERM_SAMPLE_COUNT; ++i) {
        int raw = 0;
        if (adc_oneshot_read(s_therm_adc_handle, THERM_ADC_CHANNEL, &raw) != ESP_OK) {
            return false;
        }
        samples[i] = raw;
    }

    // Insertion sort is cheap for this fixed small sample size.
    for (int i = 1; i < THERM_SAMPLE_COUNT; ++i) {
        int key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            --j;
        }
        samples[j + 1] = key;
    }

    int32_t raw_sum = 0;
    int start = THERM_TRIM_COUNT;
    int end = THERM_SAMPLE_COUNT - THERM_TRIM_COUNT;
    if (start >= end) {
        return false;
    }
    for (int i = start; i < end; ++i) {
        raw_sum += samples[i];
    }

    float raw = (float)raw_sum / (float)(end - start);
    if (raw <= 1.0f || raw >= (THERM_ADC_MAX - 1.0f)) {
        return false;
    }

    // Wiring used: 3V3 -> NTC -> ADC -> fixed resistor -> GND
    // For this divider orientation: R_ntc = R_fixed * ((Vcc - Vout) / Vout)
    float r_ntc = THERM_PULLUP_OHMS * ((THERM_ADC_MAX - raw) / raw);
    float inv_t = (1.0f / THERM_T0_K) + (1.0f / THERM_BETA_K) * logf(r_ntc / THERM_R25_OHMS);
    float temp_c = (1.0f / inv_t) - 273.15f;
    temp_c = (temp_c * THERM_CAL_GAIN) + THERM_CAL_OFFSET_C;

    *temp_centi_out = (int32_t)(temp_c * 100.0f);
    return true;
}
#endif

static esp_zb_cluster_list_t *create_clusters(void)
{
    esp_zb_temperature_sensor_cfg_t cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    cfg.basic_cfg.power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_BATTERY;
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
    esp_zb_ota_cluster_cfg_t ota_cfg = {
        .ota_upgrade_file_version = OTA_FILE_VERSION,
        .ota_upgrade_manufacturer = OTA_MANUFACTURER_CODE,
        .ota_upgrade_image_type = OTA_IMAGE_TYPE,
        .ota_min_block_reque = ESP_ZB_OTA_UPGRADE_MIN_BLOCK_PERIOD_DEF_VALUE,
        .ota_upgrade_file_offset = ESP_ZB_ZCL_OTA_UPGRADE_FILE_OFFSET_DEF_VALUE,
        .ota_upgrade_downloaded_file_ver = ESP_ZB_ZCL_OTA_UPGRADE_DOWNLOADED_FILE_VERSION_DEF_VALUE,
        .ota_image_upgrade_status = ESP_ZB_ZCL_OTA_UPGRADE_IMAGE_STATUS_DEF_VALUE,
    };
    uint8_t ota_server_id[8] = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_DEF_VALUE;
    memcpy(ota_cfg.ota_upgrade_server_id, ota_server_id, sizeof(ota_cfg.ota_upgrade_server_id));
    esp_zb_attribute_list_t *ota_cluster = esp_zb_ota_cluster_create(&ota_cfg);
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &s_ota_client_variable));
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, &s_ota_server_addr));
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, &s_ota_server_endpoint));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(cluster_list, ota_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
    esp_zb_humidity_meas_cluster_cfg_t hum_cfg = {
        .measured_value = 0,
        .min_value = 0,
        .max_value = 10000,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create(&hum_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
#endif
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

static void send_one_shot_report(uint16_t cluster_id, uint16_t attr_id)
{
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = HA_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID = cluster_id,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .attributeID = attr_id,
    };
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    if (err != ESP_OK) {
        ZB_LOGW("One-shot report failed (cluster=0x%04x attr=0x%04x): %s",
                cluster_id, attr_id, esp_err_to_name(err));
    }
}

static esp_err_t zigbee_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    switch (callback_id) {
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID: {
        const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *msg = (const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message;
#if defined(CONFIG_DEBUG) && CONFIG_DEBUG
        ZB_LOGI("OTA query rsp status=0x%02x mfg=0x%04x type=0x%04x ver=0x%08lx size=%lu",
                msg->query_status,
                msg->manufacturer_code,
                msg->image_type,
                (unsigned long)msg->file_version,
                (unsigned long)msg->image_size);
#else
        (void)msg;
#endif
        break;
    }
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID: {
        const esp_zb_zcl_ota_upgrade_value_message_t *msg = (const esp_zb_zcl_ota_upgrade_value_message_t *)message;
#if defined(CONFIG_DEBUG) && CONFIG_DEBUG
        ZB_LOGI("OTA status=%u img_type=0x%04x file_ver=0x%08lx img_size=%lu block=%u",
                (unsigned)msg->upgrade_status,
                msg->ota_header.image_type,
                (unsigned long)msg->ota_header.file_version,
                (unsigned long)msg->ota_header.image_size,
                (unsigned)msg->payload_size);
#else
        (void)msg;
#endif
        break;
    }
    default:
        break;
    }

    return ESP_OK;
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
            s_zb_network_joined = true;
            s_zb_join_seq++;
            esp_err_t ota_err = esp_zb_ota_upgrade_client_query_interval_set(HA_ENDPOINT, OTA_QUERY_INTERVAL_MIN);
            if (ota_err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to set OTA query interval: %s", esp_err_to_name(ota_err));
            }
        } else {
            ESP_LOGW(TAG, "Network steering failed, retrying");
            s_zb_network_joined = false;
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
#ifdef ESP_ZB_COMMON_SIGNAL_CAN_SLEEP
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        esp_zb_sleep_now();
        break;
#endif
    default:
        break;
    }
}

static void zigbee_task(void *arg)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_sleep_enable(true);

    esp_zb_ep_list_t *ep_list = create_endpoint();
    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zigbee_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_set_node_descriptor_power_source(false);
    esp_zb_stack_main_loop();
}

#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
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
#endif

#if defined(CONFIG_DEBUG) && CONFIG_DEBUG
#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
static void hp_i2c_probe_aht(void)
{
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_dev_handle_t dev = NULL;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = AHT_LP_SDA,
        .scl_io_num = AHT_LP_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ZB_LOGW("HP I2C probe skipped: bus init failed (%s)", esp_err_to_name(err));
        return;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT21B_ADDR,
        .scl_speed_hz = 100000,
    };
    err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err != ESP_OK) {
        ZB_LOGW("HP I2C probe failed: add device (%s)", esp_err_to_name(err));
        (void)i2c_del_master_bus(bus);
        return;
    }

    uint8_t init_cmd[3] = {0xBE, 0x08, 0x00};
    uint8_t trig_cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t raw[7] = {0};

    err = i2c_master_transmit(dev, init_cmd, sizeof(init_cmd), 100);
    if (err != ESP_OK) {
        ZB_LOGW("HP I2C probe init write failed (%s)", esp_err_to_name(err));
        goto out;
    }
    vTaskDelay(pdMS_TO_TICKS(AHT_BOOT_SETTLE_MS));

    err = i2c_master_transmit(dev, trig_cmd, sizeof(trig_cmd), 100);
    if (err != ESP_OK) {
        ZB_LOGW("HP I2C probe trigger write failed (%s)", esp_err_to_name(err));
        goto out;
    }
    vTaskDelay(pdMS_TO_TICKS(AHT_MEASUREMENT_WAIT_MS));

    err = i2c_master_receive(dev, raw, sizeof(raw), 100);
    if (err != ESP_OK) {
        ZB_LOGW("HP I2C probe read failed (%s)", esp_err_to_name(err));
        goto out;
    }

    ZB_DIAGI("HP probe raw=%02x %02x %02x %02x %02x %02x %02x",
             raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]);
    if (raw[1] == 0 && raw[2] == 0 && raw[3] == 0 && raw[4] == 0 && raw[5] == 0) {
        ZB_LOGW("HP probe read all-zero payload; likely pull-up/wiring/power issue");
    }

out:
    if (dev) {
        (void)i2c_master_bus_rm_device(dev);
    }
    if (bus) {
        (void)i2c_del_master_bus(bus);
    }
}
#endif
#endif

static void publish_measurement(int32_t temp_centi, uint32_t hum_centi)
{
    int16_t zcl_temp = temp_centi_to_zcl(temp_centi);

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                 &zcl_temp,
                                 false);
#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
    uint16_t zcl_hum = hum_centi_to_zcl(hum_centi);
    esp_zb_zcl_set_attribute_val(HA_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                                 &zcl_hum,
                                 false);
#endif
    send_one_shot_report(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                         ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
    send_one_shot_report(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                         ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID);
#endif
    esp_zb_lock_release();

#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
    ZB_LOGI("Report avg=%.2f C %.2f %%", temp_centi / 100.0f, hum_centi / 100.0f);
#else
    ZB_LOGI("Report avg=%.2f C", temp_centi / 100.0f);
#endif
}

#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
static void monitor_task(void *arg)
{
    uint32_t seen_seq = 0;
    uint32_t last_diag_ms = 0;
    bool agg_reset_pending = false;
    uint32_t seen_join_seq = 0;

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
            last_temp = (int32_t)ulp_lp_last_temp_centi;
            last_hum = ulp_lp_last_hum_centi;
            seq2 = ulp_lp_data_seq;
        } while (seq1 != seq2);

        reset_report_state_on_join(&seen_join_seq);

        if (seq1 != seen_seq) {
            ulp_lp_sample_interval_s = note_latest_sample(last_temp, last_hum);
            seen_seq = seq1;
        }

        if (agg_reset_pending) {
            agg_reset_pending = (ulp_lp_agg_reset_ack != ulp_lp_agg_reset_req);
        }

        uint32_t min_samples_to_report = s_have_report ? MIN_VALID_SAMPLES_TO_REPORT : 1U;
        int32_t avg_temp = (count > 0U) ? (temp_sum / (int32_t)count) : 0;
        uint32_t avg_hum = (count > 0U) ? (hum_sum / count) : 0U;
        if (s_zb_network_joined &&
            !agg_reset_pending &&
            count >= min_samples_to_report &&
            report_due_for_measurement(last_temp, last_hum)) {
            if (avg_temp < -4000 || avg_temp > 8500 || avg_hum > 10000U) {
                ZB_LOGW("Reject invalid average: %.2f C %.2f %% (%lu samples)",
                        avg_temp / 100.0f, avg_hum / 100.0f, (unsigned long)count);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            publish_measurement(avg_temp, avg_hum);
            note_published_measurement(avg_temp, avg_hum);

            ulp_lp_agg_reset_req = ulp_lp_agg_reset_req + 1;
            agg_reset_pending = true;
        }

        uint32_t now = now_ms();
        if ((now - last_diag_ms) >= 5000U) {
            ZB_DIAGI("LP diag: seq=%lu cnt=%lu interval=%lus last=%.2fC %.2f%% err=%lu i2c=%lu status=%lu zero=%lu crc=%lu range=%lu raw=%02lx %02lx %02lx %02lx %02lx %02lx %02lx",
                     (unsigned long)ulp_lp_data_seq,
                     (unsigned long)ulp_lp_sample_count,
                     (unsigned long)ulp_lp_sample_interval_s,
                     ((int32_t)ulp_lp_last_temp_centi) / 100.0f,
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
#else
static void monitor_task(void *arg)
{
    uint32_t seen_join_seq = 0;
    uint32_t delay_ms = THERM_SAMPLE_PERIOD_MS;

    while (1) {
        reset_report_state_on_join(&seen_join_seq);

        if (s_zb_network_joined) {
            int32_t temp_centi = 0;
            if (thermistor_read_temp_centi(&temp_centi)) {
                delay_ms = note_latest_sample(temp_centi, THERM_HUMIDITY_CENTI) * 1000U;

                if (report_due_for_measurement(temp_centi, THERM_HUMIDITY_CENTI)) {
                    publish_measurement(temp_centi, THERM_HUMIDITY_CENTI);
                    note_published_measurement(temp_centi, THERM_HUMIDITY_CENTI);
                }
            } else {
                ZB_LOGW("Thermistor read failed on GPIO%d", (int)THERM_GPIO);
                delay_ms = THERM_SAMPLE_PERIOD_MS;
            }
        } else {
            delay_ms = THERM_SAMPLE_PERIOD_MS;
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
#endif

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_log_level_set(TAG, ESP_LOG_INFO);

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
    ulp_lp_sample_interval_s = SAMPLE_DEFAULT_S;
    ulp_lp_sample_count = 0;
    ulp_lp_temp_sum_centi = 0;
    ulp_lp_hum_sum_centi = 0;
    ulp_lp_data_seq = 0;
    ulp_lp_agg_reset_req = 0;
    ulp_lp_agg_reset_ack = 0;
#else
    thermistor_init();
#endif
    s_zb_network_joined = false;
    s_zb_join_seq = 0;

#if defined(CONFIG_DEBUG) && CONFIG_DEBUG
#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
    hp_i2c_probe_aht();
#endif
#endif
#if !defined(CONFIG_USE_104NT4) || !CONFIG_USE_104NT4
    lp_i2c_init();
    lp_core_start();
#endif

    xTaskCreate(zigbee_task, "zigbee", 6144, NULL, 5, NULL);
    xTaskCreate(monitor_task, "monitor", 4096, NULL, 4, NULL);
}
