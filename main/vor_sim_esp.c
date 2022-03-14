#include <unistd.h>
#include "math.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "cJSON.h"
#include "driver/dac.h"
#include "driver/uart.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include "soc/rtc_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(x) (sizeof((x)) / sizeof((x)[0]))
#endif

#define COM_TASK_PRIORITY   5
#define DDS_TASK_PRIORITY   10
#define COM_TASK_STACK_SIZE 4096
#define DDS_TASK_STACK_SIZE 4096

#define TWDT_TIMEOUT_S          3
#define TASK_RESET_PERIOD_S     2

#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 1024

#define PI 3.141592654

#define F_SAMPLE 820000L                        // Sample rate in Hz
#define T_SAMPLE (1000000L / F_SAMPLE)          // Sampling period in us
#define DDS_RES  4294967296L                    // DDS accumulator resolution 2^32

#define F_IF 9690L                              // VOR IF freq in Hz
#define F_MOD 30L                               // VOR modulation freq in Hz
#define FM_SWING 960L                           // FM Frequency swing
#define FTW_1 ((DDS_RES) / F_SAMPLE)            // Frequency tuning word for IF DDS
#define FTW_IF ((F_IF*DDS_RES) / F_SAMPLE)      // Frequency tuning word for IF DDS
#define FTW_MOD ((F_MOD*DDS_RES) / F_SAMPLE)    // Frequency tuning word for MOD DDS

void dds_task(void* arg);
void com_task(void* arg);
void fill_sin_lookup(uint16_t* lookup_table, uint16_t table_size, uint16_t amplitude);
void fill_cos_lookup(uint16_t* lookup_table, uint16_t table_size, uint16_t amplitude);
void set_var_offset(double angle);

static const char* TAG = "vor_sim";

static TaskHandle_t dds_task_handle;
static TaskHandle_t com_task_handle;

uint16_t cos_table[1024];
uint16_t cos_table_mod[1024];

volatile uint32_t var_offset;
volatile double zero_offset;
volatile bool output_enable;


void app_main(void)
{
    //Initialize or reinitialize TWDT
    ESP_ERROR_CHECK(esp_task_wdt_init(TWDT_TIMEOUT_S, false));

    //Unsubscribe Idle Tasks to TWDT if they were subscribed at startup
#ifndef CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
    ESP_ERROR_CHECK(esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0)));
#endif
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 && !CONFIG_FREERTOS_UNICORE
    ESP_ERROR_CHECK(esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(1)));
#endif

    xTaskCreatePinnedToCore(com_task,
                            "COM Task",
                            COM_TASK_STACK_SIZE,
                            NULL,
                            COM_TASK_PRIORITY,
                            &com_task_handle,
                            0);

    xTaskCreatePinnedToCore(dds_task,
                            "DDS Task",
                            DDS_TASK_STACK_SIZE,
                            NULL,
                            DDS_TASK_PRIORITY,
                            &dds_task_handle,
                            1);

    zero_offset = -13;
    set_var_offset(90);
    output_enable = true;
}

void dds_task(void* arg)
{
    static uint32_t phase_accu_REF = 0;
    static uint32_t phase_accu_REF_FM = 0;

    // Subscribe Task to TWDT
    //ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    //ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    //Enable DAC
    //Disable Channel Tone
    CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
    //Channel output enable
    SET_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC | RTC_IO_PDAC1_DAC_XPD_FORCE);

    fill_cos_lookup(cos_table, ARRAY_SIZE(cos_table), 1023);
    fill_cos_lookup(cos_table_mod, ARRAY_SIZE(cos_table_mod), 511);

    for(;;)
    {
        if (output_enable)
        {
            //esp_task_wdt_reset();
            phase_accu_REF_FM += FTW_MOD;
            uint32_t fm_sin_osc = ((FM_SWING * (uint32_t) cos_table[(phase_accu_REF_FM >> 22) & 0x3FF]) / 1023);
            phase_accu_REF += ((F_IF - (FM_SWING / 2)) * FTW_1) + (fm_sin_osc * FTW_1);
            uint32_t ref_index = ((phase_accu_REF) >> 22) & 0x3FF;
            uint32_t sig_ref = cos_table_mod[ref_index];
            uint32_t var_index = ((phase_accu_REF_FM - var_offset) >> 22) & 0x3FF;
            uint32_t sig_var = cos_table_mod[var_index];
            //uint8_t sig_out = (((sig_ref + sig_var) / 2) >> 2) & 0xFF;
            uint8_t sig_out = ((sig_ref + sig_var) >> 2) & 0xFF;
            // DAC Output signal
            SET_PERI_REG_BITS(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC, sig_out, RTC_IO_PDAC1_DAC_S);
        } else {
            // DAC Output nothing
            SET_PERI_REG_BITS(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC, 0, RTC_IO_PDAC1_DAC_S);
        }
    }
}

void com_task(void* arg)
{
    // Subscribe Task to TWDT
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
            .baud_rate = UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    // Configure a temporary buffer for the incoming data
    const char data[UART_BUF_SIZE];

    for(;;)
    {
        // Read data from the UART
        uint32_t len = uart_read_bytes(UART_NUM_0, data, UART_BUF_SIZE, 200 / portTICK_RATE_MS);

        // Write data back to the UART
        //uart_write_bytes(UART_NUM_0, (const char *) data, len);
        if(len >= 1)
        {
            const cJSON *cdi_data_json = NULL;
            const cJSON *radial_json = NULL;
            const cJSON *radial_cal_json = NULL;
            const cJSON *output_enable_json = NULL;
            cJSON *input_json = cJSON_ParseWithLength(data, len);
            if (input_json == NULL) {
                const char *error_ptr = cJSON_GetErrorPtr();
                if (error_ptr != NULL) {
                    fprintf(stderr, "JSON Error\n");
                }
                continue;
            }

            cdi_data_json = cJSON_GetObjectItem(input_json, "CDI_data");
            if(cdi_data_json != NULL)
            {
                radial_cal_json = cJSON_GetObjectItem(cdi_data_json, "radial_cal");
                if (radial_cal_json != NULL && cJSON_IsNumber(radial_cal_json))
                {
                    zero_offset = cJSON_GetNumberValue(radial_cal_json);
                }

                radial_json = cJSON_GetObjectItem(cdi_data_json, "radial");
                if (radial_json != NULL && cJSON_IsNumber(radial_json))
                {
                    set_var_offset(cJSON_GetNumberValue(radial_json));
                }

                output_enable_json = cJSON_GetObjectItem(cdi_data_json, "output_enable");
                if(output_enable_json != NULL && cJSON_IsBool(output_enable_json))
                {
                    output_enable = cJSON_IsTrue(output_enable_json);
                }
            }
            cJSON_Delete(input_json);
        }
        esp_task_wdt_reset();
    }
}

void fill_sin_lookup(uint16_t* lookup_table, uint16_t table_size, uint16_t amplitude)
{
    for (uint16_t i = 0; i < table_size; i++)
    {
        double phase = ((double) i / (double) table_size);
        double rad = (2 * PI) * phase;
        lookup_table[i] = (uint16_t) round(((sin(rad) + 1) * ((double) amplitude / 2)));
        //ESP_LOGI(TAG, "%d > %f sin(%f) %d", i, phase, rad, lookup_table[i]);
    }
}

void fill_cos_lookup(uint16_t* lookup_table ,uint16_t table_size, uint16_t amplitude)
{
    for (uint16_t i = 0; i < table_size; i++)
    {
        double phase = ((double) i / (double) table_size);
        double rad = (2 * PI) * phase;
        lookup_table[i] = (uint16_t) round(((cos(rad) + 1) * ((double) amplitude / 2)));
        //ESP_LOGI(TAG, "%d > %f cos(%f) %d", i, phase, rad, lookup_table[i]);
    }
}

void set_var_offset(double angle)
{
    double correction = angle - zero_offset;
    if (correction < 0)
    {
        correction += 360;
    } else if (correction >= 360) {
        correction -= 360;
    }

    var_offset = (uint32_t) round((DDS_RES / 360) * correction);
}
