#include "BME280.h"

bool BME280::begin(uint8_t comms_mode, uint8_t address_or_cs) {
    _comms_mode = comms_mode;
    if (comms_mode == BME280_I2C_MODE) set_address(address_or_cs);
    if (comms_mode == BME280_SPI_MODE) _chip_select_pin = address_or_cs;
    bool success = comms_check();

    if (success) load_calibration();
}

bool BME280::comms_check() {
    uint8_t chip_id;
    read(&chip_id, BME280_REGISTER::ID);
    return chip_id == BME280_CHIP_ID;
}

void BME280::set_address(uint8_t address) {
    address &= 0x77;
    _device_address = address;
}

bme280_raw_reading_t BME280::get_raw_reading() {
    uint8_t buffer[8];
    read(buffer, BME280_REGISTER::PRESSURE_HIGH, 8);

    bme280_raw_reading_t reading;
    reading.pressure = (buffer[0] << 12) + (buffer[1] << 4) + (buffer[2] >> 4);
    reading.temperature = (buffer[3] << 12) + (buffer[4] << 4) + (buffer[5] >> 4);
    reading.humidity = (buffer[6] << 8) + buffer[7];
    return reading;
}

bool BME280::write(uint8_t *input, bme280_reg_t address, uint8_t length) {
    bool success = false;
    if (_comms_mode == BME280_I2C_MODE) success = write_i2c(input, address, length);
    if (_comms_mode == BME280_SPI_MODE) success = write_spi(input, address, length);
    return success;
}

bool BME280::read(uint8_t *output, bme280_reg_t address, uint8_t length) {
    bool success = false;
    if (_comms_mode == BME280_I2C_MODE) success = read_i2c(output, address, length);
    if (_comms_mode == BME280_SPI_MODE) success = read_spi(output, address, length);
    return success;
}

/**
 * Write a value to a register using I2C
 *
 * @param input: Byte to write to the register.
 * @param address: Address of register to write to.
 * @return: Success/error result of the write.
 */
bool BME280::write_i2c(uint8_t *input, bme280_reg_t address, uint8_t length) {
    bool result = true;
    Wire.beginTransmission(_device_address);
    Wire.write(address);
    for (size_t i = 0; i < length; i++) {
        Wire.write(input[i]);
    }

    if (Wire.endTransmission() != 0) {
        result = false;
    }
    return result;
}

/**
 * Read a specified number of bytes using the I2C bus.
 * @param output: The buffer in which to store the read values.
 * @param address: Register address to read (or starting address in burst reads)
 * @param length: Number of bytes to read.
 */
bool BME280::read_i2c(uint8_t *output, bme280_reg_t address, uint8_t length) {
    bool result = true;
    Wire.beginTransmission(address);
    Wire.write(address);
    if (Wire.endTransmission() != 0)
        result = false;

    else  // OK, all worked, keep going
    {
        Wire.requestFrom(address, length);
        for (size_t i = 0; (i < length) and Wire.available(); i++) {
            uint8_t c = Wire.read();
            output[i] = c;
        }
    }
    return result;
}

bool BME280::write_spi(uint8_t *input, bme280_reg_t address, uint8_t length) {
    digitalWrite(_chip_select_pin, LOW);
    SPI.transfer(address);
    for (size_t i = 0; i < length; i++) {
        SPI.transfer(input[i]);
    }
    digitalWrite(_chip_select_pin, HIGH);
    return true;
}

/**
 * Read a specified number of bytes using the SPI bus.
 * @param output: The buffer in which to store the read values.
 * @param address: Register address to read (or starting address in burst reads)
 * @param length: Number of bytes to read.
 */
bool BME280::read_spi(uint8_t *output, bme280_reg_t address, uint8_t length) {
    bool result = true;
    uint8_t num_empty_bytes = 0;

    digitalWrite(_chip_select_pin, LOW);
    SPI.transfer(address | 0x80 | 0x40);
    for (size_t i = 0; i < length; i++) {
        uint8_t c = SPI.transfer(0x00);
        if (c == 0xFF) num_empty_bytes++;
        *output = c;
        output++;
    }
    if (num_empty_bytes == length) result = false;
    digitalWrite(_chip_select_pin, HIGH);

    return result;
}

void BME280::load_calibration() {
    uint8_t buffer[BME280_DATA_LENGTH::CALIBRATION_TEMP_PRESS];

    read(buffer, BME280_REGISTER::TEMP_PRESS_CALIBRATION_START, CALIBRATION_TEMP_PRESS);
    calibration.T1 = buffer[0] | ((uint16_t)buffer[1] << 8);
    calibration.T2 = (int16_t)(buffer[2] | ((uint16_t)buffer[3] << 8));
    calibration.T3 = (int16_t)(buffer[4] | ((uint16_t)buffer[5] << 8));

    calibration.P1 = buffer[6] | ((uint16_t)buffer[7] << 8);
    calibration.P2 = (int16_t)(buffer[8] | ((uint16_t)buffer[9] << 8));
    calibration.P3 = (int16_t)(buffer[10] | ((uint16_t)buffer[11] << 8));
    calibration.P4 = (int16_t)(buffer[12] | ((uint16_t)buffer[13] << 8));
    calibration.P5 = (int16_t)(buffer[14] | ((uint16_t)buffer[15] << 8));
    calibration.P6 = (int16_t)(buffer[16] | ((uint16_t)buffer[17] << 8));
    calibration.P7 = (int16_t)(buffer[18] | ((uint16_t)buffer[19] << 8));
    calibration.P8 = (int16_t)(buffer[20] | ((uint16_t)buffer[21] << 8));
    calibration.P9 = (int16_t)(buffer[22] | ((uint16_t)buffer[23] << 8));
    // Nothing in buffer[24]
    calibration.H1 = buffer[25];

    read(buffer, BME280_REGISTER::HUMIDITY_CALIBRATION_START, BME280_DATA_LENGTH::CALIBRATION_HUMIDITY);
    calibration.H2 = (int16_t)(buffer[0] | (uint16_t)(buffer[1] << 8));
    calibration.H3 = buffer[2];
    calibration.H4 = (int16_t)((int16_t)buffer[3] << 4 | (buffer[4] & 0x0F));
    calibration.H5 = (int16_t)((int16_t)buffer[5] << 4 | ((buffer[4] >> 4) & 0x0F));
    calibration.H6 = (int8_t)buffer[6];
}

bme280_reading_t BME280::calibrate_raw_reading(bme280_raw_reading_t raw) {
    bme280_reading_t output;
    output.temperature = get_calibrated_temperature(raw.temperature);
    output.pressure = get_calibrated_pressure(raw.pressure);
    output.humidity = get_calibrated_humidity(raw.humidity);
    return output;
}

float BME280::get_calibrated_temperature(int32_t raw_temperature) {
    int64_t var1 = ((((raw_temperature >> 3) - ((int32_t)calibration.T1 << 1))) * ((int32_t)calibration.T2)) >> 11;
    int64_t var2 = (((((raw_temperature >> 4) - ((int32_t)calibration.T1)) *
                      ((raw_temperature >> 4) - ((int32_t)calibration.T1))) >>
                     12) *
                    ((int32_t)calibration.T3)) >>
                   14;

    calibration.t_fine = var1 + var2;
    float output = ((calibration.t_fine * 5 + 128) >> 8) / 100.0 + calibration.temperature_offset;
}

uint32_t BME280::get_calibrated_pressure(int32_t raw_pressure) {
    int32_t var1;
    int32_t var2;
    uint32_t pressure;

    var1 = ((int32_t)calibration.t_fine >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * int32_t(calibration.P6) + ((var1 * int32_t(calibration.P5)) << 1);
    var2 = (var2 >> 2) + (int32_t(calibration.P4) << 16);
    var1 =
        (((calibration.P3 * ((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + (((int32_t(calibration.P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * int32_t(calibration.P1)) >> 15);

    /* To avoid divide by zero exception */
    if (var1 == 0)
        pressure = 0;

    else {
        pressure = uint32_t((int32_t(1048576) - raw_pressure) - (var2 >> 12)) * 3125;
        if (pressure < 0x8000000)
            pressure = (pressure << 1) / uint32_t(var1);
        else
            pressure = (pressure / uint32_t(var1)) * 2;

        var1 = ((int32_t(calibration.P9)) * (int32_t(((pressure >> 3) * (pressure >> 3)) >> 13))) >> 12;
        var2 = (int32_t(pressure >> 2) * int32_t(calibration.P8)) >> 13;
        pressure = int32_t(pressure) + ((var1 + var2 + calibration.P7) >> 4);
    }

    return pressure;
}

float BME280::get_calibrated_humidity(int32_t raw_humidity) {
    int32_t var1;
    var1 = (calibration.t_fine - ((int32_t)76800));
    var1 = (((((raw_humidity << 14) - (((int32_t)calibration.H4) << 20) - (((int32_t)calibration.H5) * var1)) +
              ((int32_t)16384)) >>
             15) *
            (((((((var1 * ((int32_t)calibration.H6)) >> 10) *
                 (((var1 * ((int32_t)calibration.H3)) >> 11) + ((int32_t)32768))) >>
                10) +
               ((int32_t)2097152)) *
                  ((int32_t)calibration.H2) +
              8192) >>
             14));
    var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.H1)) >> 4));
    var1 = (var1 < 0 ? 0 : var1);
    var1 = (var1 > 419430400 ? 419430400 : var1);

    return (float)(var1 >> 12) / 1024.0;
}

void BME280::set_temperature_offset(float offset) { calibration.temperature_offset = offset; }