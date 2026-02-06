#include "bus_i2c.h"
#include "esp_log.h"

static const char *TAG_I2C = "BUS_I2C"; // just a name for the file

//esp_err_t is a type just returns error type
esp_err_t bus_i2c_init(i2c_port_t port, int sda, int scl, uint32_t hz) { // sda is data pin scl is clock pin hz is bus speed so how fast it repeats
    i2c_config_t conf = { // this is making a struct - the same as an object in oop
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda, //tells esp what pin to use
        .scl_io_num = scl, // same as above
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = hz,
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf)); //  loads it and sees if it errors
    esp_err_t err = i2c_driver_install(port, conf.mode, 0, 0, 0); // this starts the actual hardware and allocates memory
    
    // Set I2C timeout for BNO055 clock stretching (critical fix)
    if (err == ESP_OK) {
        i2c_set_timeout(port, 400000); // 400ms timeout for BNO055 clock stretching
        ESP_LOGI("BUS_I2C", "I2C initialized with 400ms timeout for BNO055 compatibility"); // prints to console 
    }
    
    return err;
}

esp_err_t bus_i2c_wr8(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val, TickType_t to) { // writes data to a register 1 byte
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(port, addr, buf, sizeof(buf), to); 
}

esp_err_t bus_i2c_wrrd(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len, TickType_t to) { // this writes what register the sensor is using then reads
    return i2c_master_write_read_device(port, addr, &reg, 1, buf, len, to);
}
