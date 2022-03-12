#include "math.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/dac.h"
#include <unistd.h>

#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(x) (sizeof((x)) / sizeof((x)[0]))
#endif

#define PI 3.141592654

#define F_SAMPLE 460000L                // Sample rate in Hz
#define T_SAMPLE (1000000L / F_SAMPLE)  // Sampling period in us
#define DDS_RES 4294967296L             // DDS accumulator resolution 2^32

#define F_IF 9690L                              // VOR IF freq in Hz
#define F_MOD 30L                               // VOR modulation freq in Hz
#define FM_SWING 960L                             // FM Frequency swing
#define FTW_1 ((DDS_RES) / F_SAMPLE)            // Frequency tuning word for IF DDS
#define FTW_IF ((F_IF*DDS_RES) / F_SAMPLE)      // Frequency tuning word for IF DDS
#define FTW_MOD ((F_MOD*DDS_RES) / F_SAMPLE)    // Frequency tuning word for MOD DDS

static void periodic_timer_callback(void* arg);
static void periodic_timer_callback2(void* arg);
void fill_sin_lookup(uint16_t* lookup_table, uint16_t table_size, uint16_t amplitude);
void fill_cos_lookup(uint16_t* lookup_table, uint16_t table_size, uint16_t amplitude);
void set_var_offset(double angle);

static const char* TAG = "vor_sim";
volatile uint16_t cos_table[1024];
volatile uint32_t var_offset;
volatile double zero_offset;

void app_main(void)
{
    fill_cos_lookup(cos_table, ARRAY_SIZE(cos_table), 1023);

    zero_offset = 12;

    set_var_offset(90);

    //dac_output_enable(DAC_CHANNEL_1);
    //Disable Channel Tone
    CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
    //Channel output enable
    SET_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC | RTC_IO_PDAC1_DAC_XPD_FORCE);
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback2,
            .name = "periodic"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    //ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 500000));
    ESP_LOGI(TAG, "Started periodic timer, time since boot: %lld us", esp_timer_get_time());

    for(;;) {
        //ESP_ERROR_CHECK(esp_timer_dump(stdout));
        usleep(1);
        periodic_timer_callback(0);
    }
}

static void periodic_timer_callback(void* arg)
{
    static uint32_t phase_accu_REF = 0;
    static uint32_t phase_accu_REF_FM = 0;
    static uint32_t toggle = 0;

    /*
    gpio_set_level(GPIO_NUM_16, toggle);
    if (toggle == 0) {
        toggle = 1;
    } else {
        toggle = 0;
    }
    */

    phase_accu_REF_FM += FTW_MOD;
    uint32_t fm_sin_osc = ((FM_SWING * (uint32_t)cos_table[(phase_accu_REF_FM>>22) & 0x3FF]) / 1023);
    phase_accu_REF += ((F_IF-(FM_SWING/2)) * FTW_1) + (fm_sin_osc * FTW_1);
    uint32_t ref_index = ((phase_accu_REF)>>22) & 0x3FF;
    uint32_t sig_ref = cos_table[ref_index];
    uint32_t var_index = ((phase_accu_REF_FM+var_offset)>>22) & 0x3FF;
    uint32_t sig_var = cos_table[var_index];
    //uint8_t  sig_out = (((sig_ref + (sig_var/3)) / 2)>>2)&0xFF; // AM modulation index ≈ 0.3
    uint8_t sig_out = (((sig_ref + sig_var) / 2)>>2) & 0xFF;
    //dac_output_voltage(DAC_CHANNEL_1, (sig_var>>2 & 0xFF));
    //dac_output_voltage(DAC_CHANNEL_1, (sig_ref>>2 & 0xFF));
    //Set the Dac value
    //SET_PERI_REG_BITS(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC, (sig_var>>2 & 0xFF), RTC_IO_PDAC1_DAC_S);   //dac_output
    SET_PERI_REG_BITS(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC, sig_out, RTC_IO_PDAC1_DAC_S);   //dac_output
    //ESP_LOGI(TAG, "%d",fm_sin_osc);
}

static void periodic_timer_callback2(void* arg) {
    static int32_t angle = 0;
    if (angle == 360) {
        angle=0;
    } else {
        set_var_offset(angle);
        ESP_LOGI(TAG, "Angle: %d °", angle);
        angle++;
    }
}

void fill_sin_lookup(uint16_t* lookup_table, uint16_t table_size, uint16_t amplitude) {
    for (uint16_t i = 0; i < table_size; i++) {
        double phase = ((double) i / (double) table_size);
        double rad = (2 * PI) * phase;
        lookup_table[i] = (uint16_t) round(((sin(rad) + 1) * ((double) amplitude / 2)));
        //ESP_LOGI(TAG, "%d > %f sin(%f) %d", i, phase, rad, lookup_table[i]);
    }
}

void fill_cos_lookup(uint16_t* lookup_table ,uint16_t table_size, uint16_t amplitude) {
    for (uint16_t i = 0; i < table_size; i++) {
        double phase = ((double) i / (double) table_size);
        double rad = (2 * PI) * phase;
        lookup_table[i] = (uint16_t) round(((cos(rad) + 1) * ((double) amplitude / 2)));
        //ESP_LOGI(TAG, "%d > %f cos(%f) %d", i, phase, rad, lookup_table[i]);
    }
}

void set_var_offset(double angle) {
    double correction = angle - zero_offset;
    if (correction < 0) {
        correction += 360;
    } else if (correction >= 360) {
        correction -= 360;
    }

    var_offset = (uint32_t) round((DDS_RES / 360) * correction);
}
