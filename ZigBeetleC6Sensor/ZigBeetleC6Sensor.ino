
#include <Arduino.h>
#include <Zigbee.h>
#include <Wire.h>
#include <ep/ZigbeeTempSensor.h>

#define DEBUG

namespace {
    constexpr uint8_t I2C_SDA_PIN = 19;
    constexpr uint8_t I2C_SCL_PIN = 20;
    constexpr uint8_t AHT21B_I2C_ADDRESS = 0x38;
    constexpr uint8_t AHT21B_STATUS_BYTES = 1;
    constexpr uint8_t AHT21B_DATA_BYTES = 6;
    constexpr uint8_t AHT21B_TRIGGER_MEASUREMENT = 0xAC;
    constexpr uint8_t AHT21B_MEASUREMENT_PARAM_1 = 0x33;
    constexpr uint8_t AHT21B_MEASUREMENT_PARAM_2 = 0x00;
    constexpr uint8_t AHT21B_INIT_COMMAND = 0xBE;
    constexpr uint8_t AHT21B_INIT_PARAM_1 = 0x08;
    constexpr uint8_t AHT21B_INIT_PARAM_2 = 0x00;
    constexpr unsigned long AHT21B_POLL_DELAY_MS = 5;
    constexpr unsigned long AHT21B_POLL_TIMEOUT_MS = 200;
    constexpr unsigned long AHT21B_POWER_UP_DELAY_MS = 100;
    constexpr unsigned long AHT21B_RETRY_DELAY_MS = 20;
    constexpr float AHT21B_HUMIDITY_SCALE = 100.0f / 1048576.0f;
    constexpr float AHT21B_TEMPERATURE_SCALE = 200.0f / 1048576.0f;
    constexpr float AHT21B_TEMPERATURE_OFFSET_C = 50.0f;
    constexpr uint8_t AHT21B_CRC_POLY = 0x31;
    constexpr uint8_t AHT21B_CRC_INIT = 0xFF;
    constexpr uint16_t REPORT_MIN_SECONDS = 1;
    constexpr uint16_t REPORT_MAX_SECONDS = 600;
    constexpr float REPORT_CHANGE_C = 1.0f;
    constexpr float HUMIDITY_REPORT_CHANGE = 100.0f; // 1% in 0.01% units
    constexpr float HUMIDITY_TOLERANCE = 3.0f;
    constexpr unsigned long REPORT_INTERVAL_MS = 5000;

    ZigbeeTempSensor external_sensor(1);
    float external_temp_c = NAN;
    float external_humidity = NAN;
    unsigned long last_report_ms = 0;
    bool reported_join = false;
    bool external_sensor_ok = false;
    bool external_has_reading = false;

    bool initAht21b() {
      Wire.requestFrom(AHT21B_I2C_ADDRESS, AHT21B_STATUS_BYTES);
      if (Wire.available() >= AHT21B_STATUS_BYTES) {
        uint8_t status = Wire.read();
        if (status & 0x08) {
          return true;
        }
      }

      Wire.beginTransmission(AHT21B_I2C_ADDRESS);
      Wire.write(AHT21B_INIT_COMMAND);
      Wire.write(AHT21B_INIT_PARAM_1);
      Wire.write(AHT21B_INIT_PARAM_2);
      if (Wire.endTransmission() != 0) {
        return false;
      }
      delay(AHT21B_RETRY_DELAY_MS);
      return true;
    }

    uint8_t aht21bCrc8(const uint8_t *data, size_t len) {
      uint8_t crc = AHT21B_CRC_INIT;
      for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
          if (crc & 0x80) {
            crc = static_cast<uint8_t>((crc << 1) ^ AHT21B_CRC_POLY);
          } else {
            crc = static_cast<uint8_t>(crc << 1);
          }
        }
      }
      return crc;
    }

    bool readAht21bRaw(uint8_t *buffer, size_t buffer_len) {
      constexpr uint8_t read_len = AHT21B_DATA_BYTES + 1;
      if (buffer_len < AHT21B_DATA_BYTES) {
        return false;
      }

      if (!initAht21b()) {
        return false;
      }

      Wire.beginTransmission(AHT21B_I2C_ADDRESS);
      Wire.write(AHT21B_TRIGGER_MEASUREMENT);
      Wire.write(AHT21B_MEASUREMENT_PARAM_1);
      Wire.write(AHT21B_MEASUREMENT_PARAM_2);
      if (Wire.endTransmission() != 0) {
        return false;
      }

      unsigned long start_ms = millis();
      bool ready = false;
      while (millis() - start_ms < AHT21B_POLL_TIMEOUT_MS) {
        Wire.requestFrom(AHT21B_I2C_ADDRESS, AHT21B_STATUS_BYTES);
        if (Wire.available() >= AHT21B_STATUS_BYTES) {
          uint8_t status = Wire.read();
          if ((status & 0x80) == 0) {
            ready = true;
            break;
          }
        }
        delay(AHT21B_POLL_DELAY_MS);
      }
      if (!ready) {
        return false;
      }

      Wire.requestFrom(AHT21B_I2C_ADDRESS, read_len);
      if (Wire.available() < AHT21B_DATA_BYTES) {
        return false;
      }

      uint8_t raw[read_len] = {0};
      size_t available = 0;
      while (Wire.available() && available < read_len) {
        raw[available++] = Wire.read();
      }
      if (available < AHT21B_DATA_BYTES) {
        return false;
      }
      if (available >= read_len) {
        uint8_t crc = aht21bCrc8(raw, AHT21B_DATA_BYTES);
        if (crc != raw[AHT21B_DATA_BYTES]) {
          return false;
        }
      }

      for (size_t i = 0; i < AHT21B_DATA_BYTES; ++i) {
        buffer[i] = raw[i];
      }

      bool all_zero = true;
      for (size_t i = 1; i < AHT21B_DATA_BYTES; ++i) {
        if (buffer[i] != 0) {
          all_zero = false;
          break;
        }
      }
      if (all_zero) {
        delay(AHT21B_RETRY_DELAY_MS);
        return false;
      }
      return true;
    }

    bool readAht21bData(float *temp_c, float *humidity) {
      uint8_t buffer[AHT21B_DATA_BYTES] = {0};
      if (!readAht21bRaw(buffer, sizeof(buffer))) {
        return false;
      }

      uint32_t raw_humidity = (static_cast<uint32_t>(buffer[1]) << 12) |
                              (static_cast<uint32_t>(buffer[2]) << 4) |
                              (static_cast<uint32_t>(buffer[3]) >> 4);
      uint32_t raw_temp = ((static_cast<uint32_t>(buffer[3]) & 0x0F) << 16) |
                          (static_cast<uint32_t>(buffer[4]) << 8) |
                          static_cast<uint32_t>(buffer[5]);

      *humidity = raw_humidity * AHT21B_HUMIDITY_SCALE;
      *temp_c = raw_temp * AHT21B_TEMPERATURE_SCALE - AHT21B_TEMPERATURE_OFFSET_C;
      return true;
    }
}

#ifdef DEBUG
namespace {
bool debugAht21bAddress() {
  Wire.beginTransmission(AHT21B_I2C_ADDRESS);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.println("AHT21B I2C address acknowledged");
    return true;
  }
  Serial.print("AHT21B I2C address not acknowledged, error=");
  Serial.println(err);
  return false;
}

void debugAht21bStatus() {
  Wire.requestFrom(AHT21B_I2C_ADDRESS, AHT21B_STATUS_BYTES);
  if (Wire.available() < AHT21B_STATUS_BYTES) {
    Serial.println("AHT21B status read failed");
    return;
  }
  uint8_t status = Wire.read();
  Serial.print("AHT21B status=0x");
  Serial.println(status, HEX);
  Serial.print("AHT21B status busy=");
  Serial.println((status & 0x80) ? "1" : "0");
  Serial.print("AHT21B status calibrated=");
  Serial.println((status & 0x08) ? "1" : "0");
}

void debugAht21bRawData() {
  uint8_t buffer[AHT21B_DATA_BYTES] = {0};
  constexpr uint8_t read_len = AHT21B_DATA_BYTES + 1;
  uint8_t raw[read_len] = {0};
  if (!initAht21b()) {
    Serial.println("AHT21B raw data read failed");
    return;
  }

  Wire.beginTransmission(AHT21B_I2C_ADDRESS);
  Wire.write(AHT21B_TRIGGER_MEASUREMENT);
  Wire.write(AHT21B_MEASUREMENT_PARAM_1);
  Wire.write(AHT21B_MEASUREMENT_PARAM_2);
  if (Wire.endTransmission() != 0) {
    Serial.println("AHT21B raw data read failed");
    return;
  }

  unsigned long start_ms = millis();
  bool ready = false;
  while (millis() - start_ms < AHT21B_POLL_TIMEOUT_MS) {
    Wire.requestFrom(AHT21B_I2C_ADDRESS, AHT21B_STATUS_BYTES);
    if (Wire.available() >= AHT21B_STATUS_BYTES) {
      uint8_t status = Wire.read();
      if ((status & 0x80) == 0) {
        ready = true;
        break;
      }
    }
    delay(AHT21B_POLL_DELAY_MS);
  }
  if (!ready) {
    Serial.println("AHT21B raw data read failed");
    return;
  }

  Wire.requestFrom(AHT21B_I2C_ADDRESS, read_len);
  size_t available = 0;
  while (Wire.available() && available < read_len) {
    raw[available++] = Wire.read();
  }
  if (available < AHT21B_DATA_BYTES) {
    Serial.println("AHT21B raw data read failed");
    return;
  }

  Serial.print("AHT21B raw data:");
  for (size_t i = 0; i < AHT21B_DATA_BYTES; ++i) {
    buffer[i] = raw[i];
    uint8_t value = raw[i];
    Serial.print(" 0x");
    if (value < 0x10) {
      Serial.print('0');
    }
    Serial.print(value, HEX);
  }
  if (available >= read_len) {
    uint8_t crc_calc = aht21bCrc8(raw, AHT21B_DATA_BYTES);
    Serial.print(" crc=0x");
    if (raw[AHT21B_DATA_BYTES] < 0x10) {
      Serial.print('0');
    }
    Serial.print(raw[AHT21B_DATA_BYTES], HEX);
    Serial.print(" calc=0x");
    if (crc_calc < 0x10) {
      Serial.print('0');
    }
    Serial.print(crc_calc, HEX);
  } else {
    Serial.print(" crc=missing");
  }
  Serial.println();
}

void debugI2cScan() {
  Serial.println("I2C scan start");
  uint8_t found = 0;
  for (uint8_t address = 0x03; address <= 0x77; ++address) {
    Wire.beginTransmission(address);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 0x10) {
        Serial.print('0');
      }
      Serial.println(address, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("I2C scan found no devices");
  } else {
    Serial.print("I2C scan complete, devices found=");
    Serial.println(found);
  }
}
} // namespace
#endif

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Starting Zigbeetle sensor");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(AHT21B_POWER_UP_DELAY_MS);

  external_sensor_ok = initAht21b();
  if (external_sensor_ok) {
    Serial.println("AHT21B ready");
  } else {
    Serial.println("AHT21B init failed");
  }
#ifdef DEBUG
  debugI2cScan();
  if (debugAht21bAddress()) {
    debugAht21bStatus();
    debugAht21bRawData();
  }
#endif

  external_sensor.setManufacturerAndModel("ZigBeetle", "AHT21B");
  external_sensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
  external_sensor.setDefaultValue(0);
  external_sensor.addHumiditySensor(0, 100, HUMIDITY_TOLERANCE);
  Zigbee.addEndpoint(&external_sensor);

#ifdef DEBUG
  Zigbee.setDebugMode(true);
#endif

  if (!Zigbee.begin(ZIGBEE_END_DEVICE, true)) {
    Serial.println("Zigbee start failed");
    return;
  }

  if (readAht21bData(&external_temp_c, &external_humidity)) {
    external_sensor_ok = true;
    external_has_reading = true;
  } else {
    Serial.println("AHT21B initial read failed");
  }

  external_sensor.setReporting(REPORT_MIN_SECONDS, REPORT_MAX_SECONDS, REPORT_CHANGE_C);
  external_sensor.setHumidityReporting(REPORT_MIN_SECONDS, REPORT_MAX_SECONDS, HUMIDITY_REPORT_CHANGE);
  if (external_has_reading) {
    external_sensor.setTemperature(external_temp_c);
    external_sensor.setHumidity(external_humidity);
    external_sensor.report();
    external_sensor.reportHumidity();
  }

  Serial.println("Zigbee started");
}

void loop() {
  if (!Zigbee.connected()) {
    if (!reported_join) {
      Serial.println("Waiting to join Zigbee network...");
      reported_join = true;
    }
    delay(250);
    return;
  }

  if (millis() - last_report_ms < REPORT_INTERVAL_MS) {
    delay(10);
    return;
  }

  last_report_ms = millis();
  if (readAht21bData(&external_temp_c, &external_humidity)) {
    external_sensor_ok = true;
    external_has_reading = true;
  } else {
    Serial.println("AHT21B read failed");
  }

  if (external_has_reading) {
    external_sensor.setTemperature(external_temp_c);
    external_sensor.setHumidity(external_humidity);
    external_sensor.report();
    external_sensor.reportHumidity();
  }


  if (external_has_reading) {
    Serial.print("Reported external temp/humidity: ");
    Serial.print(external_temp_c, 2);
    Serial.print(" C, ");
    Serial.print(external_humidity, 1);
    Serial.println(" %");
  } else {
    Serial.println("Reported external temp/humidity: unavailable");
  }
}
