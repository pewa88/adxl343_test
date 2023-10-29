/* *****************************************************************************
 *  Includes
 * ****************************************************************************/
#include "adxl343.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include <unistd.h>
#include <sys/param.h>
#include "sdkconfig.h"

/* *****************************************************************************
 *  Macros
 * ****************************************************************************/
#define ADXL343_SPI_HOST        SPI2_HOST
#define ADXL343_SCLK_PIN        (GPIO_NUM_42)
#define ADXL343_MOSI_PIN        (GPIO_NUM_39)
#define ADXL343_MISO_PIN        (GPIO_NUM_40)
#define ADXL343_nCS_PIN         (GPIO_NUM_41)
#define ADXL343_SPI_CLK_FREQ    (1U*1000U*1000U)   //max freq is 5MHz!

/** \brief Bit 1 of the command is the read/write bit. 
 * 
 * Setting this bit to 1 will issue a read command.
 * Setting this bit to 0 will issue a write command.
*/
#define CMD_READ             (0x2U)
#define CMD_WRITE            (0x0U)

/** \brief Bitmask for the multi byte bit of the command.
 * 
 * To read or write multiple bytes in a single transmission, the multiple-byte 
 * bit must be set. After the register addressing and the first byte of data, 
 * each subsequent set of clock pulses (eight clock pulses) causes the ADXL343 
 * to point to the next register for a read or write.
*/
#define CMD_MULTI_BYTE_MASK  (0x1U)

#define ADXL_NUM_CALIB_SAMPLES  (16U)

/* *****************************************************************************
 *  Types
 * ****************************************************************************/
struct adxl343_context_t
{
    adxl343_config_t cfg;        ///< Configuration by the caller.
    spi_device_handle_t spi;    ///< SPI device handle
    SemaphoreHandle_t ready_sem; ///< Semaphore for ready signal
};

typedef struct adxl343_context_t adxl343_context_t;

/* *****************************************************************************
 *  Local Variables and constants
 * ****************************************************************************/
static const char TAG[] = "adxl343";

/* *****************************************************************************
 *  Local Functions
 * ****************************************************************************/
/** \brief Read multiple registers in a single SPI frame.
 * 
 * \param[in] ctx The ADXL343 context
 * \param[in] regAddrStart The first register to read
 * \param[in] numRegs The number of registers to read
 * \param[out] pBuffer Pointer to buffer where the read content will be stored
 */
static esp_err_t _RegReadMultiple(adxl343_context_t *ctx, uint8_t regAddrStart, uint8_t numRegs, uint8_t *pBuffer)
{
    spi_transaction_t t = 
        {
            .cmd = (CMD_READ | CMD_MULTI_BYTE_MASK),
            .addr = regAddrStart,
            .rxlength = (numRegs*8U),
            .rx_buffer = (uint8_t*)pBuffer,
            .user = ctx,
        };
    return spi_device_polling_transmit(ctx->spi, &t);
}

static esp_err_t _RegRead(adxl343_context_t *ctx, uint8_t RegAddr, uint8_t *pRegVal)
{
    return _RegReadMultiple(ctx, RegAddr, 1U, pRegVal);
}

static esp_err_t _RegWrite(adxl343_context_t *ctx, uint8_t RegAddr, uint8_t RegVal)
{
    spi_transaction_t t = 
        {
            .cmd = CMD_WRITE,
            .addr = RegAddr,
            .length = 8U,
            .flags = SPI_TRANS_USE_TXDATA,
            .user = ctx,
            .tx_data = {RegVal}
        };
    return spi_device_polling_transmit(ctx->spi, &t);
}

/* *****************************************************************************
 *  Global Functions
 * ****************************************************************************/
void ADXL343_Init(adxl343_context_t **out_ctx)
{
    spi_bus_config_t buscfg = 
        {
            .miso_io_num = ADXL343_MISO_PIN,
            .mosi_io_num = ADXL343_MOSI_PIN,
            .sclk_io_num = ADXL343_SCLK_PIN,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 32
        };
    adxl343_config_t adxl343_cfg = 
        {
            .host = ADXL343_SPI_HOST,
            .cs_io = ADXL343_nCS_PIN,
            .miso_io = ADXL343_MISO_PIN,
            .intr_used = false
        };
    spi_device_interface_config_t devCfg = 
        {
            .command_bits = 2U,
            .address_bits = 6U,
            .dummy_bits = 0U,
            .flags = SPI_DEVICE_HALFDUPLEX,
            .cs_ena_pretrans = 1U,
            .clock_speed_hz = ADXL343_SPI_CLK_FREQ,
            .mode = 3U,          //SPI mode 3
            .spics_io_num = adxl343_cfg.cs_io,
            .queue_size = 1,
        };

    ESP_LOGI(TAG, "Initializing SPI%d for ADXL343...", ADXL343_SPI_HOST+1U);
    if (ESP_OK != spi_bus_initialize(ADXL343_SPI_HOST, &buscfg, SPI_DMA_DISABLED))
    {
        ESP_LOGE(TAG, "Initializing SPI bus failed!\n");
        return;
    }

    ESP_LOGI(TAG, "Initializing ADXL343 device...");
    if ((adxl343_cfg.intr_used) && (adxl343_cfg.host == SPI1_HOST))
    {
        ESP_LOGE(TAG, "interrupt cannot be used on SPI1 host.");
        return;
    }

    adxl343_context_t *ctx = (adxl343_context_t*)malloc(sizeof(adxl343_context_t));
    if (!ctx)
    {
        ESP_LOGE(TAG, "Couldn't create ADXL343 context. Out of memory.");
        return;
    }

    *ctx = (adxl343_context_t)
        {
            .cfg = adxl343_cfg,
        };

    //Attach the tmp126 to the SPI bus
    if  (ESP_OK != spi_bus_add_device(ctx->cfg.host, &devCfg, &ctx->spi))
    {
        ESP_LOGE(TAG, "spi_bus_add_device() failed.");
        /* cleanup */
        if (ctx->spi)
        {
            spi_bus_remove_device(ctx->spi);
            ctx->spi = NULL;
        }
        free(ctx);
    }
    else
    {
        *out_ctx = ctx;
    }

}

esp_err_t Adxl343_DeInit(adxl343_context_t *ctx)
{
    spi_bus_remove_device(ctx->spi);
    if (ctx->cfg.intr_used)
    {
        vSemaphoreDelete(ctx->ready_sem);
    }
    free(ctx);
    return ESP_OK;
}

static void _CalibrationTask(void *pParameter)
{
    adxl343_handle_t *pAdxl343Hdl = (adxl343_handle_t*)pParameter;
    esp_err_t err;
    const TickType_t xFrequency = (((ADXL_NUM_CALIB_SAMPLES*100U)/1000U)+1U) / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime;

    /* initialize calibration */
    err = _RegWrite(*pAdxl343Hdl, 
                    ADXL343_FIFO_CTRL_REG_ADDR, 
                    (ADXL343_FIFO_MODE_FIFO | ADXL_NUM_CALIB_SAMPLES)
                   );
    if (ESP_OK == err)
    {
        // Initialise the xLastWakeTime variable with the current time.
        xLastWakeTime = xTaskGetTickCount();
        while (1)
        {
            uint8_t int_src;

            // Wait for the next cycle.
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            err = _RegRead(*pAdxl343Hdl, ADXL343_INT_SRC_REG_ADDR, &int_src);
            if (ESP_OK == err)
            {
                if (int_src & ADXL343_INT_SRC_WATERMARK)
                {
                    /* there are ADXL_NUM_CALIB_SAMPLES in the FIFO -> read them */
                    uint8_t sample[6U]; /* x, y and z each have 2 bytes */
                    spi_device_acquire_bus((*pAdxl343Hdl)->spi, portMAX_DELAY);
                    for (unsigned int s=0U; s<ADXL_NUM_CALIB_SAMPLES; s++)
                    {
                        if (ESP_OK == _RegReadMultiple(*pAdxl343Hdl, ADXL343_DATAX0_REG_ADDR, 6U, sample))
                        {
                            uint16_t datax = ((uint16_t)sample[1U]<<8U) | sample[0U];
                            uint16_t datay = ((uint16_t)sample[3U]<<8U) | sample[2U];
                            uint16_t dataz = ((uint16_t)sample[5U]<<8U) | sample[4U];
                            ESP_LOGI(TAG, "DATAX=%04x DATAY=%04x DATAZ=%04x", datax, datay, dataz);
                        }
                    }
                    spi_device_release_bus((*pAdxl343Hdl)->spi);
                }
            }
        }   
    }
    else
    {
        ESP_LOGE(TAG, "Error during calibration initialization");
    }

}

/** \brief This task gets the ambient temperatur (in C) from TI TMP126 via SPI */
void Adxl343_Task(void *pParameter)
{
    adxl343_handle_t *pAdxl343Hdl = (adxl343_handle_t*)pParameter;
    uint8_t regVal=0U;
    esp_err_t err;
    const TickType_t xFrequency = 1000U / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    if (pdTRUE != xTaskCreatePinnedToCore(_CalibrationTask, "calib343", 2048, pAdxl343Hdl, tskIDLE_PRIORITY+1U, NULL, 1U))
    {
        ESP_LOGE(TAG, "creating ADXL343 calibration task failed");
    }

    while (1)
    {
        ESP_LOGI(TAG, "Device ID");
        err = _RegRead(*pAdxl343Hdl, ADXL343_DEVICE_ID_REG_ADDR, &regVal);
        if (ESP_OK == err)
        {
            ESP_LOGI(TAG, "--> %02x", regVal);
        }
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}
