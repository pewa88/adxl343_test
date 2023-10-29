#pragma once
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/// Configurations of the spi_tmp126
typedef struct {
    spi_host_device_t host; ///< The SPI host used, set before calling `spi_adxl343_init()`
    gpio_num_t cs_io;       ///< CS gpio number, set before calling `spi_adxl343_init()`
    gpio_num_t miso_io;     ///< MISO gpio number, set before calling `spi_adxl343_init()`
    bool intr_used;         ///< Whether to use polling or interrupt when waiting for write to be done. Set before calling `spi_adxl343_init()`.
} adxl343_config_t;

/* adxl343_handle_t is a pointer to adxl343_context_t */
typedef struct adxl343_context_t* adxl343_handle_t;

enum
{
    ADXL343_DEVICE_ID_REG_ADDR = 0x0U,
    ADXL343_THRESH_TAP_REG_ADDR = 0x1DU,
    ADXL343_OFSX_REG_ADDR = 0x1EU,
    ADXL343_OFSY_REG_ADDR = 0x1FU,
    ADXL343_OFSZ_REG_ADDR = 0x20U,
    ADXL343_TAP_DUR_REG_ADDR = 0x21U,
    /* TODO */
    ADXL343_INT_SRC_REG_ADDR = 0x30U,
    ADXL343_DATAX0_REG_ADDR = 0x32U,
    ADXL343_DATAX1_REG_ADDR = 0x33U,
    ADXL343_DATAY0_REG_ADDR = 0x34U,
    ADXL343_DATAY1_REG_ADDR = 0x35U,
    ADXL343_DATAZ0_REG_ADDR = 0x36U,
    ADXL343_DATAZ1_REG_ADDR = 0x37U,
    ADXL343_FIFO_CTRL_REG_ADDR = 0x38U,
    ADXL343_FIFO_STAT_REG_ADDR = 0x39U
};

enum
{
    ADXL343_FIFO_MODE_BYPASS    = 0x00U,
    ADXL343_FIFO_MODE_FIFO      = 0x40U,
    ADXL343_FIFO_MODE_STREAM    = 0x80U,
    ADXL343_FIFO_MODE_TRIGGER   = 0xC0U
};

enum
{
    ADXL343_INT_SRC_OVERRUN     = 0x01U,
    ADXL343_INT_SRC_WATERMARK   = 0x02U,
    ADXL343_INT_SRC_FREEFALL    = 0x04U,
    ADXL343_INT_SRC_INACTIVITY  = 0x08U,
    ADXL343_INT_SRC_ACTIVITY    = 0x10U,
    ADXL343_INT_SRC_DOUBLE_TAP  = 0x20U,
    ADXL343_INT_SRC_SINGLE_TAP  = 0x40U,
    ADXL343_INT_SRC_DATA_RDY    = 0x80U
};

/**
 * @brief Initialize the hardware.
 */
void ADXL343_Init(adxl343_handle_t *ctx);

/**
 * @brief Release the resources used by the tmp126.
 *
 * @param handle Context of tmp126 communication.
 * @return Always ESP_OK
 */
esp_err_t Adxl343_DeInit(adxl343_handle_t ctx);

void Adxl343_Task(void *pParameter);
