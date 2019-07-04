#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#ifndef BME280_H_
#define BME280_H_

const uint8_t DEFAULT_BME280_ADDRESS = 0x76;
const uint8_t BME280_CHIP_ID = 0x60;
enum BME280_COMMS_MODE { BME280_I2C_MODE, BME280_SPI_MODE };

typedef struct {
    int32_t temperature;
    int32_t pressure;
    int32_t humidity;
} bme280_raw_reading_t;

typedef struct {
    float temperature;  // Temperature in deg C
    uint32_t pressure;  // Pressure in P
    float humidity;     // Relative humidity in %
} bme280_reading_t;

enum BME280_OVERSAMPLING_MODE {
    BME280_OVERSAMPLING_SKIPPED = 0,
    BME280_OVERSAMPLING_x1 = 1,
    BME280_OVERSAMPLING_x2 = 2,
    BME280_OVERSAMPLING_x4 = 3,
    BME280_OVERSAMPLING_x8 = 4,
    BME280_OVERSAMPLING_x16 = 5
};

enum BME280_POWER_MODE {
    BME280_SLEEP_MODE = 0,
    BME280_FORCED_MODE = 1,
    BME280_FORCED_MODE = 2,
    BME280_NORMAL_MODE = 3
};

enum BME280_STANDBY_TIME {
    BME280_STANDBY_TIME_500NS = 0,
    BME280_STANDBY_TIME_62MS = 1,
    BME280_STANDBY_TIME_125MS = 2,
    BME280_STANDBY_TIME_250MS = 3,
    BME280_STANDBY_TIME_500MS = 4,
    BME280_STANDBY_TIME_1S = 5,
    BME280_STANDBY_TIME_10MS = 6,
    BME280_STANDBY_TIME_20MS = 7
};

enum BME280_FILTER_COEFFICIENT {
    BME280_FILTER_OFF = 0,
    BME280_FILTER_COEFFICIENT_2 = 1,
    BME280_FILTER_COEFFICIENT_4 = 2,
    BME280_FILTER_COEFFICIENT_8 = 3,
    BME280_FILTER_COEFFICIENT_16 = 4
};

typedef union {
    uint8_t raw;
    struct {
        uint8_t oversampling_mode : 3;
        uint8_t _reserved : 5;
    };
} bme_humidity_control_t;

typedef union {
    uint8_t raw;
    struct {
        uint8_t copying_from_image : 1;
        uint8_t _reserved1 : 2;
        uint8_t measuring : 1;
        uint8_t _reserved2 : 4;
    };
} bme280_status_t;

typedef union {
    uint8_t raw;
    struct {
        uint8_t power_mode : 2;
        uint8_t pressure_oversampling_mode : 3;
        uint8_t temperature_oversampling_mode : 3;
    };
} bme280_control_t;

typedef union {
    uint8_t raw;
    struct {
        uint8_t spi_3_wire_enabled : 1;
        uint8_t filter_coefficient : 3;
        uint8_t standby_time : 3;
    };
} bme280_config_t;

typedef struct {
    // Temperature calibration factors
    uint16_t T1;
    int16_t T2;
    int16_t T3;
    int32_t t_fine;
    float temperature_offset = 0.0;

    // Pressure calibration factors
    uint16_t P1;
    int16_t P2;
    int16_t P3;
    int16_t P4;
    int16_t P5;
    int16_t P6;
    int16_t P7;
    int16_t P8;
    int16_t P9;

    // Humidity calibration factors
    uint8_t H1;
    int16_t H2;
    uint8_t H3;
    int16_t H4;
    int16_t H5;
    int8_t H6;
} bme280_calibration_t;

class BME280 {
   public:
    bool begin(uint8_t comms_mode = BME280_I2C_MODE, uint8_t address_or_cs = DEFAULT_BME280_ADDRESS);
    bool comms_check();
    void set_address(uint8_t address);
    void set_temperature_offset(float offset);

   private:
    typedef enum BME280_REGISTER {
        ID = 0xD0,                // Contains chip id (0x60)
        RESET = 0xE0,             // Soft reset trigger. Activates when 0xB6 written to register
        CONTROL_HUMIDITY = 0xF2,  // Humidity data acquisition options. Applied after write to CONTROL_MEASURE
        STATUS = 0xF3,            // Indicates if the device is measuring or copying from its memory image
        CONTROL_MEASURE = 0xF4,   // Temperature and pressure acquisition options.
        POWER_CONTROL = 0xF4,     // Temperature and pressure acquisition options.
        CONFIG = 0xF5,         // Rate, filter, and interface options. May be ignored in normal mode, but not in sleep.
        PRESSURE_HIGH = 0XF7,  // Pressure data - high byte [19:12]
        PRESSURE_MID = 0XF8,   // Pressure data - mid byte [11:4]
        PRESSURE_LOW = 0XF9,   // Pressure data - low byte [3:0]
        TEMPERATURE_HIGH = 0XFA,  // Temperature data - high byte [19:12]
        TEMPERATURE_MID = 0XFB,   // Temperature data - mid byte [11:4]
        TEMPERATURE_LOW = 0XFC,   // Temperature data - low byte [3:0]
        HUMIDITY_HIGH = 0XFD,     // Humidity data - high byte [15:8]
        HUMIDITY_LOW = 0XFE,      // Humidity data - mid byte [7:0]

        // Calibration registers
        TEMP_PRESS_CALIBRATION_START = 0x88,
        HUMIDITY_CALIBRATION_START = 0xE1

    } bme280_reg_t;

    enum BME280_DATA_LENGTH { CALIBRATION_TEMP_PRESS = 26, CALIBRATION_HUMIDITY = 7, RAW_READING = 8 };

    bme280_calibration_t calibration;
    uint8_t _device_address;
    uint8_t _comms_mode = BME280_I2C_MODE;
    uint8_t _chip_select_pin;

    bool write(uint8_t* input, bme280_reg_t address, uint8_t length = 1);
    bool read(uint8_t* output, bme280_reg_t address, uint8_t length = 1);

    bool write_i2c(uint8_t* input, bme280_reg_t address, uint8_t length);
    bool read_i2c(uint8_t* output, bme280_reg_t address, uint8_t length);

    bool write_spi(uint8_t* input, bme280_reg_t address, uint8_t length);
    bool read_spi(uint8_t* output, bme280_reg_t address, uint8_t length);

    void load_calibration();
    bme280_raw_reading_t get_raw_reading();
    bme280_reading_t calibrate_raw_reading(bme280_raw_reading_t raw);
    float get_calibrated_temperature(int32_t raw_temperature);
    uint32_t get_calibrated_pressure(int32_t raw_pressure);
    float get_calibrated_humidity(int32_t raw_humidity)
};

#endif