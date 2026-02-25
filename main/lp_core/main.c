#include <stdbool.h>
#include <stdint.h>

#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_lp_timer_shared.h"
#include "ulp_lp_core_utils.h"
#include "aht21b_defs.h"

#define LP_I2C_NUM LP_I2C_NUM_0

#define POLL_DELAY_US 5000
#define POLL_TIMEOUT_US 200000
#define AHT_CRC_POLY 0x31
#define AHT_CRC_INIT 0xFF

volatile uint32_t lp_sample_count = 0;
volatile int32_t lp_temp_sum_centi = 0;
volatile uint32_t lp_hum_sum_centi = 0;
volatile int32_t lp_last_temp_centi = 0;
volatile uint32_t lp_last_hum_centi = 0;
volatile uint32_t lp_error_count = 0;
volatile uint32_t lp_data_seq = 0;
volatile uint32_t lp_sample_interval_s = 15;
volatile uint32_t lp_sensor_inited = 0;
volatile uint32_t lp_fail_i2c_count = 0;
volatile uint32_t lp_fail_status_count = 0;
volatile uint32_t lp_fail_zero_count = 0;
volatile uint32_t lp_fail_crc_count = 0;
volatile uint32_t lp_fail_range_count = 0;
volatile uint32_t lp_last_raw0 = 0;
volatile uint32_t lp_last_raw1 = 0;
volatile uint32_t lp_last_raw2 = 0;
volatile uint32_t lp_last_raw3 = 0;
volatile uint32_t lp_last_raw4 = 0;
volatile uint32_t lp_last_raw5 = 0;
volatile uint32_t lp_last_raw6 = 0;
volatile uint32_t lp_agg_reset_req = 0;
volatile uint32_t lp_agg_reset_ack = 0;

static bool aht_write3(uint8_t b0, uint8_t b1, uint8_t b2)
{
    uint8_t data[3] = {b0, b1, b2};
    if (lp_core_i2c_master_write_to_device(LP_I2C_NUM, AHT21B_ADDR, data, sizeof(data), -1) != ESP_OK) {
        lp_fail_i2c_count++;
        return false;
    }
    return true;
}

static bool aht_init(void)
{
    if (aht_write3(AHT21B_INIT_CMD, AHT21B_INIT_P1, AHT21B_INIT_P2)) {
        lp_sensor_inited = 1;
        return true;
    }
    return false;
}

static bool aht_read(int32_t *temp_centi, uint32_t *hum_centi)
{
    if (!aht_write3(AHT21B_TRIG_CMD, AHT21B_TRIG_P1, AHT21B_TRIG_P2)) {
        return false;
    }

    uint8_t status = 0x80;
    uint32_t elapsed = 0;
    while ((status & 0x80) && elapsed < POLL_TIMEOUT_US) {
        if (lp_core_i2c_master_read_from_device(LP_I2C_NUM, AHT21B_ADDR, &status, 1, -1) != ESP_OK) {
            lp_fail_i2c_count++;
            return false;
        }
        if ((status & 0x80) == 0) {
            break;
        }
        ulp_lp_core_delay_us(POLL_DELAY_US);
        elapsed += POLL_DELAY_US;
    }
    if (status & 0x80) {
        return false;
    }

    uint8_t raw[7] = {0};
    if (lp_core_i2c_master_read_from_device(LP_I2C_NUM, AHT21B_ADDR, raw, sizeof(raw), -1) != ESP_OK) {
        lp_fail_i2c_count++;
        return false;
    }
    lp_last_raw0 = raw[0];
    lp_last_raw1 = raw[1];
    lp_last_raw2 = raw[2];
    lp_last_raw3 = raw[3];
    lp_last_raw4 = raw[4];
    lp_last_raw5 = raw[5];
    lp_last_raw6 = raw[6];

    /* Status byte should indicate not busy + calibrated. */
    if ((raw[0] & 0x80) != 0 || (raw[0] & 0x08) == 0) {
        lp_fail_status_count++;
        return false;
    }

    /* Reject all-zero payloads frequently seen on bus glitches. */
    if (raw[1] == 0 && raw[2] == 0 && raw[3] == 0 && raw[4] == 0 && raw[5] == 0) {
        lp_fail_zero_count++;
        return false;
    }

    uint32_t raw_h = ((uint32_t)raw[1] << 12) | ((uint32_t)raw[2] << 4) | ((uint32_t)raw[3] >> 4);
    uint32_t raw_t = (((uint32_t)raw[3] & 0x0F) << 16) | ((uint32_t)raw[4] << 8) | (uint32_t)raw[5];

    /* Optional CRC byte check (AHT21/AHT21B). */
    uint8_t crc = AHT_CRC_INIT;
    for (int i = 0; i < 6; ++i) {
        crc ^= raw[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ AHT_CRC_POLY) : (uint8_t)(crc << 1);
        }
    }
    if (crc != raw[6]) {
        lp_fail_crc_count++;
        return false;
    }

    uint32_t h = (uint32_t)(((uint64_t)raw_h * 10000ULL) / 1048576ULL);
    int32_t t = (int32_t)(((int64_t)raw_t * 20000LL) / 1048576LL) - 5000;
    if (h > 10000U || t < -4000 || t > 8500) {
        lp_fail_range_count++;
        return false;
    }

    *hum_centi = h;
    *temp_centi = t;
    return true;
}

int main(void)
{
    if (lp_agg_reset_req != lp_agg_reset_ack) {
        lp_sample_count = 0;
        lp_temp_sum_centi = 0;
        lp_hum_sum_centi = 0;
        lp_agg_reset_ack = lp_agg_reset_req;
    }

    uint32_t interval_s = lp_sample_interval_s;
    if (interval_s == 0) {
        interval_s = 15;
    }

    if (!lp_sensor_inited) {
        (void)aht_init();
    }

    int32_t temp_centi = 0;
    uint32_t hum_centi = 0;
    if (aht_read(&temp_centi, &hum_centi)) {
        lp_sample_count++;
        lp_temp_sum_centi += temp_centi;
        lp_hum_sum_centi += hum_centi;
        lp_last_temp_centi = temp_centi;
        lp_last_hum_centi = hum_centi;
        lp_data_seq++;
    } else {
        lp_error_count++;
    }

    ulp_lp_core_lp_timer_set_wakeup_time((uint64_t)interval_s * 1000000ULL);
    ulp_lp_core_halt();
}
