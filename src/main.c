#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

#define SIMULATION_MODE 0 // 1 = no hardware, use simulated values; 0 = real sensor

// --- LDC1612 Register Addresses ---
#define REG_DATA_CH1_MSB        0x02
#define REG_DATA_CH1_LSB        0x03
#define REG_RCOUNT_CH1          0x09  // Reference Count setting for Channel 1
#define REG_SETTLECOUNT_CH1     0x11  // Channel 1 Settling Reference Count
#define REG_CLOCK_DIVIDERS_CH1  0x15  // Reference and Sensor Divider settings for Channel 1
#define REG_DRIVE_CURRENT_CH1   0x1F  // Channel 1 sensor current drive configuration
#define REG_CONFIG              0x1A  // Conversion Configuration
#define REG_MUX_CONFIG          0x1B  // Channel Multiplexing Configuration
#define REG_ERROR_CONFIG        0x19

// --- I2C Master Configuration ---
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
#define LDC1612_ADDR            0x2B

static const char *TAG = "LDC1612_I2C";

// --- Sensor/Calculation Constants ---
#define TANK_CAPACITANCE  76e-12
#define BASE_FREQ         4800000.0    // Hz
#define BASE_L_VALUE      1.3642e-05   // H
#define R_COUNT_VALUE     0xFFFF
#define CLK_SPEED         4000000.0    // Internal LDC clock (datasheet scale factor)

// --- Function Prototypes ---
esp_err_t ldc1612_i2c_write(uint8_t reg_addr, uint16_t data);
esp_err_t ldc1612_i2c_read(uint8_t reg_addr, uint16_t *data);
esp_err_t ldc1612_init(void);

// --- Delta L Calculation (expects frequencies in Hz) ---
double calc_delta_L(double measured_freq_hz, double base_freq_hz, double base_L)
{
    double delta_f = measured_freq_hz - base_freq_hz;
    double delta_L = (delta_f * base_L * -2.0) / base_freq_hz;
    return delta_L;
}

// --- I2C Master Initialization ---
void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "I2C Master Initialized.");
}

// --- I2C Write Function ---
esp_err_t ldc1612_i2c_write(uint8_t reg_addr, uint16_t data)
{
    uint8_t write_buf[3];
    write_buf[0] = reg_addr;
    // Data is MSB first
    write_buf[1] = (uint8_t)(data >> 8);
    write_buf[2] = (uint8_t)(data & 0xFF);

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        LDC1612_ADDR,
        write_buf,
        3,
        1000 / portTICK_PERIOD_MS
    );
}

// --- I2C Read Function ---
esp_err_t ldc1612_i2c_read(uint8_t reg_addr, uint16_t *data)
{
    uint8_t read_buf[2];
    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        LDC1612_ADDR,
        &reg_addr,
        1,
        read_buf,
        2,
        1000 / portTICK_PERIOD_MS
    );

    if (ret == ESP_OK)
    {
        // Data is MSB first
        *data = (read_buf[0] << 8) | read_buf[1];
    }
    return ret;
}

esp_err_t ldc1612_init(void)
{
    ESP_LOGI(TAG, "Starting LDC1612 initialization for Channel 1...");
    esp_err_t ret;

    // Wait for sensor to be ready after power-up
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 1. Set RCOUNT for Channel 1 (max resolution)
    ret = ldc1612_i2c_write(REG_RCOUNT_CH1, R_COUNT_VALUE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "RCOUNT_CH1 write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "RCOUNT_CH1 set to 0x%04X", R_COUNT_VALUE);

    // 2. Set SETTLECOUNT for Channel 1
    uint16_t settle_val = 0x000A;
    ret = ldc1612_i2c_write(REG_SETTLECOUNT_CH1, settle_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SETTLECOUNT_CH1 write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "SETTLECOUNT_CH1 set to 0x%04X", settle_val);

    // 3. Set CLOCK_DIVIDERS for Channel 1
    uint16_t clk_div_val = 0x1001;  // FIN_DIVIDER1 = 1, FREF_DIVIDER1 = 1
    ret = ldc1612_i2c_write(REG_CLOCK_DIVIDERS_CH1, clk_div_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "CLOCK_DIVIDERS_CH1 write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "CLOCK_DIVIDERS_CH1 set to 0x%04X", clk_div_val);

    // 4. Set ERROR_CONFIG (enable data ready interrupt)
    uint16_t error_config = 0x0001; // Enable DRDY_2INT
    ret = ldc1612_i2c_write(REG_ERROR_CONFIG, error_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR_CONFIG write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "ERROR_CONFIG set to 0x%04X", error_config);

    // 5. Set MUX_CONFIG
    uint16_t mux_val = 0x020D; // Single channel, deglitch filter 10 MHz
    ret = ldc1612_i2c_write(REG_MUX_CONFIG, mux_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MUX_CONFIG write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "MUX_CONFIG set to 0x%04X", mux_val);

    // 6. DRIVE_CURRENT for Channel 1
    uint16_t drive_current = 0x9000; // IDRIVE1 ~195 µA
    ret = ldc1612_i2c_write(REG_DRIVE_CURRENT_CH1, drive_current);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "DRIVE_CURRENT_CH1 write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "DRIVE_CURRENT_CH1 set to 0x%04X", drive_current);

    // 7. CONFIG: Channel 1 active, RP override, internal osc
    uint16_t config_val = 0x5401; // Channel 1 selected
    ret = ldc1612_i2c_write(REG_CONFIG, config_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "CONFIG write failed!");
        return ret;
    }
    ESP_LOGI(TAG, "CONFIG set to 0x%04X - Channel 1 Active", config_val);

    // Wait for sensor to stabilize
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "LDC1612 Channel 1 initialized successfully.");
    return ESP_OK;
}

// ===================== LUT & INTERPOLATION =====================

#define NUM_POINTS 41

static const float pos_mm[NUM_POINTS] = {
    0.0, 2.5, 5.0, 7.5, 10.0,
    12.5, 15.0, 17.5, 20.0, 22.5,
    25.0, 27.5, 30.0, 32.5, 35.0,
    37.5, 40.0, 42.5, 45.0, 47.5,
    50.0, 52.5, 55.0, 57.5, 60.0,
    62.5, 65.0, 67.5, 70.0, 72.5,
    75.0, 77.5, 80.0, 82.5, 85.0,
    87.5, 90.0, 92.5, 95.0, 97.5,
    100.0
};

static const float freq_MHz[NUM_POINTS] = {
    5.89807, 5.89984, 5.90123, 5.90786, 5.91271,
    5.92365, 5.92891, 5.94361, 5.95148, 5.96751,
    5.98287, 5.99505, 6.01061, 6.03432, 6.04099,
    6.06224, 6.07113, 6.09508, 6.10909, 6.12351,
    6.13959, 6.16958, 6.17209, 6.20116, 6.19695,
    6.21887, 6.25129, 6.24627, 6.29026, 6.30239,
    6.32946, 6.37288, 6.37561, 6.38429, 6.38255,
    6.38967, 6.39334, 6.44768, 6.47235, 6.41289,
    6.31169
};

// Converts measured frequency in MHz → position_mm using LUT + linear interpolation
float interpolate_position(float f_meas_MHz)
{
    // Clamping
    if (f_meas_MHz <= freq_MHz[0])
        return pos_mm[0];
    if (f_meas_MHz >= freq_MHz[NUM_POINTS - 1])
        return pos_mm[NUM_POINTS - 1];

    // Find interval where freq lies
    for (int i = 0; i < NUM_POINTS - 1; i++)
    {
        float f1 = freq_MHz[i];
        float f2 = freq_MHz[i + 1];

        if ((f_meas_MHz >= f1 && f_meas_MHz <= f2) ||
            (f_meas_MHz >= f2 && f_meas_MHz <= f1))
        {
            float x1 = pos_mm[i];
            float x2 = pos_mm[i + 1];

            // Linear interpolation:
            // x = x1 + (f - f1) * (x2 - x1) / (f2 - f1)
            float position = x1 + (f_meas_MHz - f1) * (x2 - x1) / (f2 - f1);
            return position;
        }
    }

    // Fallback (should never occur if LUT covers the range)
    return -1.0f;
}

// ===================== MAIN APP =====================

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    printf("---- BOOTED ----\n");

    vTaskDelay(pdMS_TO_TICKS(500));
    i2c_master_init();

    // Initialize LDC1612 Channel 1
    if (ldc1612_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LDC1612. Aborting.");
        return;
    }

    // Wait for first conversion to complete
    vTaskDelay(pdMS_TO_TICKS(500));

    // --- Main Loop for Reading Channel 1 Data ---
    while (1)
    {
        uint16_t msb, lsb;

        esp_err_t ret_msb = ldc1612_i2c_read(REG_DATA_CH1_MSB, &msb);
        esp_err_t ret_lsb = ldc1612_i2c_read(REG_DATA_CH1_LSB, &lsb);

        if (ret_msb == ESP_OK && ret_lsb == ESP_OK)
        {
            uint8_t err_flags = (msb >> 12) & 0x0F;
            if (err_flags != 0)
            {
                ESP_LOGW(TAG, "CH1 Error flags: 0x%X", err_flags);
                if (err_flags & 0x08)
                    ESP_LOGW(TAG, "  - ERR_UR1: Under-range error");
                if (err_flags & 0x04)
                    ESP_LOGW(TAG, "  - ERR_OR1: Over-range error");
                if (err_flags & 0x02)
                    ESP_LOGW(TAG, "  - ERR_WD1: Watchdog timeout");
                if (err_flags & 0x01)
                    ESP_LOGW(TAG, "  - ERR_AE1: Amplitude error");
            }

            // Combine into 28-bit value (bits 27:0)
            uint32_t raw28 = (((uint32_t)(msb & 0x0FFF)) << 16) | lsb;

            // Special conditions
            if (raw28 == 0x00000000)
            {
                ESP_LOGE(TAG, "CH1 Under-range condition detected!");
            }
            else if (raw28 == 0x0FFFFFFF)
            {
                ESP_LOGE(TAG, "CH1 Over-range condition detected!");
            }

            // Calculate frequency using friend's scaling (this yields values ~6.0 which match your LUT in MHz)
            const double SCALE_FACTOR = pow(2.0, 28.0) * (double)R_COUNT_VALUE;
            double measured_freq_mhz = ((double)raw28 * CLK_SPEED) / SCALE_FACTOR; // treat as MHz for LUT + display
            double measured_freq_hz  = measured_freq_mhz * 1e6;                    // derive Hz for physics calcs

            // Calculate change in inductance (using Hz)
            double delta_L = calc_delta_L(measured_freq_hz, BASE_FREQ, BASE_L_VALUE);

            // Calculate absolute inductance using L = 1/(4π²f²C) (f in Hz)
            double calculated_L = 0.0;
            if (measured_freq_hz > 0.0)
            {
                calculated_L = 1.0 /
                    (4.0 * M_PI * M_PI * measured_freq_hz * measured_freq_hz * TANK_CAPACITANCE);
            }

            // Look up + Interpolation using MHz (LUT is in MHz)
            float position_mm = interpolate_position((float)measured_freq_mhz);

            // Debug logs
            ESP_LOGI(TAG, "Measured Resonant Frequency: %.5f MHz ", measured_freq_mhz);
            ESP_LOGI(TAG, "Target position: %.3f mm", position_mm);
            // ESP_LOGI(TAG, "Calculated Inductance: %.6e H (%.3f µH)", calculated_L, calculated_L * 1e6);
            // ESP_LOGI(TAG, "Change in Inductance (Delta L): %e H", delta_L);

            // ---- JSON OUTPUT for backend/UI ----
            // One clean JSON line per reading
            printf("{\"freq_hz\": %.3f, \"freq_mhz\": %.5f, \"pos_mm\": %.3f}\n",
                   measured_freq_hz, measured_freq_mhz, position_mm);
            fflush(stdout);
        }
        else
        {
            ESP_LOGE(TAG, "I2C read failed (MSB=%s, LSB=%s)",
                     esp_err_to_name(ret_msb), esp_err_to_name(ret_lsb));
        }

        // Wait for next measurement
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz update
    }
}
