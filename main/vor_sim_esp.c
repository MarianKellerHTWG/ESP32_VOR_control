#include <unistd.h>
#include "math.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "cJSON.h"
#include "driver/dac.h"
#include "driver/uart.h"
#include "driver/gpio.h"
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

#define LIGHT_EN_GPIO GPIO_NUM_33
#define ILS_EN_GPIO GPIO_NUM_32

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

#define F_SAMPLE_LOC 820000L                    // Localizer sample rate in Hz
#define F_LEFT_LOC 90L                          // LOC left freq in Hz
#define F_RIGHT_LOC 150L                        // LOC right freq in Hz
#define FTW_LEFT_LOC ((F_LEFT_LOC*DDS_RES) / F_SAMPLE_LOC)  // Frequency tuning word for left LOC DDS
#define FTW_RIGHT_LOC ((F_RIGHT_LOC*DDS_RES) / F_SAMPLE_LOC)  // Frequency tuning word for left LOC DDS
#define LONG_COS_MAX 4294967295L
#define AMPL_MIN_DIV_LOC 33038210L
#define REL_SCALE_FACT_LOC 56.0

void dds_task(void* arg) __attribute__((noreturn));
void com_task(void* arg) __attribute__((noreturn));
void fill_sin_lookup(uint16_t* lookup_table, uint16_t table_size, uint16_t amplitude);
void fill_cos_lookup(uint16_t* lookup_table, uint16_t table_size, uint16_t amplitude);
void fill_long_cos_lookup(uint32_t* lookup_table ,uint16_t table_size, uint32_t amplitude);
void set_var_offset(double angle);
void set_loc_angle(double angle);
void set_vor_enable(bool enable);
void set_loc_enable(bool enable);
void set_light_enable(bool enable);

static const char* TAG = "vor_sim";

static TaskHandle_t dds_task_handle;
static TaskHandle_t com_task_handle;

uint16_t cos_table[1024];
uint16_t cos_table_mod[1024];
uint32_t cos_table_long[1024];

volatile uint32_t var_offset;
volatile double zero_offset;
volatile bool vor_enable;
volatile bool loc_enable;
volatile bool light_enable;

volatile uint32_t left_ampl_div_loc;
volatile uint32_t right_ampl_div_loc;
volatile uint32_t left_ampl_offset_loc;
volatile uint32_t right_ampl_offset_loc;


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

    // Initialize GPIO
    gpio_set_direction(LIGHT_EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(ILS_EN_GPIO, GPIO_MODE_OUTPUT);

    // Initialize standard values
    zero_offset = -13;
    set_var_offset(90);
    set_loc_angle(2.5);
    vor_enable = false;
    loc_enable = true;
    light_enable = true;
    gpio_set_level(LIGHT_EN_GPIO, light_enable);
    gpio_set_level(ILS_EN_GPIO, loc_enable);
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

void set_loc_angle(double angle)
{
    if (angle < -3.5) angle = -3.5;
    if (angle > 3.5) angle = 3.5;

    double right_rel = 180 / (360 + angle * REL_SCALE_FACT_LOC);
    double left_rel = 180 / (360 - angle * REL_SCALE_FACT_LOC);

    right_ampl_div_loc = AMPL_MIN_DIV_LOC / right_rel;
    left_ampl_div_loc = AMPL_MIN_DIV_LOC / left_rel;

    if (left_ampl_div_loc < right_ampl_div_loc)
    {
        right_ampl_offset_loc = ((LONG_COS_MAX / left_ampl_div_loc) / 2) - ((LONG_COS_MAX / right_ampl_div_loc) / 2);
        left_ampl_offset_loc = 0;
    }
    else
    {
        left_ampl_offset_loc = ((LONG_COS_MAX / right_ampl_div_loc) / 2) - ((LONG_COS_MAX / left_ampl_div_loc) / 2);
        right_ampl_offset_loc = 0;
    }
    //ESP_LOGI(TAG, "%d %d",left_ampl_offset_loc, right_ampl_offset_loc);
    //ESP_LOGI(TAG, "%lld %lld",(LONG_COS_MAX/left_ampl_div_loc), (LONG_COS_MAX/right_ampl_div_loc));
}

void dds_task(void* arg)
{
    static uint32_t phase_accu_REF = 0;
    static uint32_t phase_accu_REF_FM = 0;
    static uint32_t phase_accu_left_LOC = 0;
    static uint32_t phase_accu_right_LOC = 0;

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
    fill_long_cos_lookup(cos_table_long, ARRAY_SIZE(cos_table_long), LONG_COS_MAX);

    for(;;)
    {
        if (vor_enable)
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
        }
        else if (loc_enable)
        {
            phase_accu_right_LOC += FTW_RIGHT_LOC;
            phase_accu_left_LOC += FTW_LEFT_LOC;
            uint32_t right_loc_index = (phase_accu_right_LOC >> 22) & 0x3FF;
            uint32_t left_loc_index = (phase_accu_left_LOC >> 22) & 0x3FF;
            uint32_t sig_right = (cos_table_long[right_loc_index] / right_ampl_div_loc) + right_ampl_offset_loc;
            uint32_t sig_left = (cos_table_long[left_loc_index] / left_ampl_div_loc) + left_ampl_offset_loc;
            uint8_t sig_out = sig_left + sig_right;
            //uint32_t sig_out = cos_table_long[right_loc_index] / AMPL_MIN_DIV_LOC;
            // DAC Output signal
            SET_PERI_REG_BITS(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC, sig_out, RTC_IO_PDAC1_DAC_S);
            //ESP_LOGI(TAG, "%d", sig_out);
        }
        else
        {
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
            const cJSON *loc_angle_json = NULL;
            const cJSON *vor_enable_json = NULL;
            const cJSON *loc_enable_json = NULL;
            const cJSON *light_enable_json = NULL;
            cJSON *ok_list_json = NULL;
            //cJSON *ok_radial = NULL;
            //cJSON *ok_radial_cal = NULL;
            //cJSON *ok_loc_angle = NULL;
            //cJSON *ok_vor_enable = NULL;
            //cJSON *ok_loc_enable = NULL;
            //cJSON *ok_light_enable = NULL;
            cJSON *input_json = cJSON_ParseWithLength(data, len);
            if (input_json == NULL) {
                const char *error_ptr = cJSON_GetErrorPtr();
                if (error_ptr != NULL) {
                    fprintf(stderr, "{\"error\":{\"JSON\":\"parsing error\"}}\n");
                }
                goto com_task_loop_end;
            }
            cJSON *output_json = cJSON_CreateObject();
            ok_list_json = cJSON_CreateObject();
            if (output_json == NULL || ok_list_json == NULL)
            {
                fprintf(stderr, "{\"error\":{\"JSON\":\"Could not initialize data structure\"}}\n");
                cJSON_Delete(output_json);
                goto com_task_loop_end;
            }
            cJSON_AddItemToObject(output_json, "ok", ok_list_json);

            cdi_data_json = cJSON_GetObjectItem(input_json, "CDI_data");
            if(cdi_data_json != NULL)
            {
                radial_cal_json = cJSON_GetObjectItem(cdi_data_json, "radial_cal");
                if (radial_cal_json != NULL && cJSON_IsNumber(radial_cal_json))
                {
                    cJSON_AddNumberToObject(ok_list_json, "radial_cal", 0);
                    zero_offset = cJSON_GetNumberValue(radial_cal_json);
                }

                radial_json = cJSON_GetObjectItem(cdi_data_json, "radial");
                if (radial_json != NULL && cJSON_IsNumber(radial_json))
                {
                    cJSON_AddNumberToObject(ok_list_json, "radial", 0);
                    set_var_offset(cJSON_GetNumberValue(radial_json));
                }

                loc_angle_json = cJSON_GetObjectItem(cdi_data_json, "loc_angle");
                if (loc_angle_json != NULL && cJSON_IsNumber(loc_angle_json))
                {
                    cJSON_AddNumberToObject(ok_list_json, "loc_angle", 0);
                    set_loc_angle(cJSON_GetNumberValue(loc_angle_json));
                }

                vor_enable_json = cJSON_GetObjectItem(cdi_data_json, "vor_enable");
                if(vor_enable_json != NULL && cJSON_IsBool(vor_enable_json))
                {
                    cJSON_AddNumberToObject(ok_list_json, "vor_enable", 0);
                    set_vor_enable(cJSON_IsTrue(vor_enable_json));
                }

                loc_enable_json = cJSON_GetObjectItem(cdi_data_json, "loc_enable");
                if(loc_enable_json != NULL && cJSON_IsBool(loc_enable_json))
                {
                    cJSON_AddNumberToObject(ok_list_json, "loc_enable", 0);
                    set_loc_enable(cJSON_IsTrue(loc_enable_json));
                }

                light_enable_json = cJSON_GetObjectItem(cdi_data_json, "light_enable");
                if(light_enable_json != NULL && cJSON_IsBool(light_enable_json))
                {
                    cJSON_AddNumberToObject(ok_list_json, "light_enable", 0);
                    light_enable = cJSON_IsTrue(light_enable_json);
                    gpio_set_level(LIGHT_EN_GPIO, light_enable);
                }
            }

            if (output_json != NULL)
            {
                char* out = cJSON_Print(output_json);
                printf("%s\n", out);
                cJSON_Delete(output_json);
            }
            com_task_loop_end:
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

void fill_long_cos_lookup(uint32_t* lookup_table ,uint16_t table_size, uint32_t amplitude)
{
    for (uint16_t i = 0; i < table_size; i++)
    {
        double phase = ((double) i / (double) table_size);
        double rad = (2 * PI) * phase;
        lookup_table[i] = (uint32_t) round(((cos(rad) + 1) * ((double) amplitude / 2)));
    }
}

void set_vor_enable(bool enable)
{
    vor_enable = enable;

    // only affect other modes when this is enabled
    if (enable)
    {
        // disable loc
        loc_enable = false;
        gpio_set_level(ILS_EN_GPIO, false);
    }
}

void set_loc_enable(bool enable)
{
    loc_enable = enable;
    gpio_set_level(ILS_EN_GPIO, enable);

    // only affect other modes when this is enabled
    if (enable)
    {
        // disable vor
        vor_enable = false;
    }
}

void set_light_enable(bool enable)
{
    light_enable = enable;
    gpio_set_level(LIGHT_EN_GPIO, enable);
}